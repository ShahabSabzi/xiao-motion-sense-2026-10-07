/*
 * XIAO nRF52840 Sense motion streamer.
 * Board: Seeed nRF52 Boards 1.1.13 (Adafruit/Bluefruit, NON-mbed).
 * Only core-bundled Wire, TinyUSB and Bluefruit headers are used.
 */
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>
#include <bluefruit.h>
#include "MotionMath.h"

using motion::Vec;
using motion::Quat;

// EDIT HERE: signed input axes for output X, Y, Z. Must be right-handed.
// Examples: {1,2,3}, {2,-1,3}, {1,-2,-3}.
const int8_t AXIS_REMAP[3] = {1,2,3};
// For an arbitrary mounting angle, rotate the remapped vectors with this
// unit quaternion (w,x,y,z), from sensor frame into desired board frame.
const Quat MOUNT_ROTATION = {1,0,0,0};

class MotionUart : public BLEUart {
public:
  // BLEUart::write uses a blocking semaphore when the notification queue
  // is full. Direct SoftDevice submission returns immediately instead.
  // TX completion accounting in Bluefruit remains safe: its counting
  // semaphore saturates at the configured queue depth.
  bool send(const uint8_t* data,uint16_t len) {
    if(!len||len>20||!Bluefruit.connected()||!notifyEnabled()) return false;
    ble_gatts_hvx_params_t params={};
    params.handle=_txd.handles().value_handle;
    params.type=BLE_GATT_HVX_NOTIFICATION;
    params.p_len=&len; params.p_data=data;
    return sd_ble_gatts_hvx(Bluefruit.connHandle(),&params)==NRF_SUCCESS;
  }
};
MotionUart uart;
BLEBas battery;
motion::Madgwick fusion;
motion::GyroBias bias;
Quat reference={1,0,0,0};
Vec accel={0,0,1}, gyro={0,0,0};
bool imuOK=false, busReady=false, haveSample=false, binary=false;
uint8_t address=0x6A;
uint16_t streamHz=20;
uint32_t lastPoll=0,lastSample=0,lastStream=0,lastBattery=0,powerAt=0;
uint32_t startedAt=0;
bool initialised=false;
char textLine[160];
uint16_t textLength=0,textPosition=0;
volatile bool resetSession=false;

// The Seeed variant's RGB LED hardware is active LOW; LED_STATE_ON in
// that variant is misleading. Red fault overrides green/blue status.
void updateLED() {
  bool blink=(millis()/350)%2==0;
  digitalWrite(LED_RED,(!imuOK && initialised && blink)?LOW:HIGH);
  digitalWrite(LED_GREEN,(imuOK && Bluefruit.connected())?LOW:HIGH);
  digitalWrite(LED_BLUE,(!Bluefruit.connected() && (imuOK||!initialised) && blink)?LOW:HIGH);
}

// Bounded transfers on Wire1's TWIM1 peripheral. Wire1.begin() sets the
// internal SDA=17,SCL=16 pins after the plain-input idle check.
// Core Wire1.endTransmission/requestFrom have unbounded polling loops;
// this small hardware wrapper enforces a deadline even on a runtime fault.
// NEVER initialise or probe Wire (external D4/D5).
bool transfer(uint8_t* tx,uint8_t tn,uint8_t* rx,uint8_t rn) {
  if(!busReady) return false;
  NRF_TWIM_Type* t=NRF_TWIM1;
  if(!digitalRead(PIN_WIRE1_SDA)||!digitalRead(PIN_WIRE1_SCL)) return false;
  t->EVENTS_STOPPED=0; t->EVENTS_ERROR=0;
  t->EVENTS_LASTTX=0; t->EVENTS_LASTRX=0;
  t->EVENTS_TXSTARTED=0; t->EVENTS_RXSTARTED=0;
  t->ERRORSRC=7;
  t->ADDRESS=address;
  t->TXD.PTR=(uint32_t)tx; t->TXD.MAXCNT=tn;
  t->RXD.PTR=(uint32_t)rx; t->RXD.MAXCNT=rn;
  t->SHORTS=rn?(TWIM_SHORTS_LASTTX_STARTRX_Msk|TWIM_SHORTS_LASTRX_STOP_Msk)
              :TWIM_SHORTS_LASTTX_STOP_Msk;
  uint32_t start=micros();
  t->TASKS_STARTTX=1;
  while(!t->EVENTS_STOPPED && !t->EVENTS_ERROR &&
        (uint32_t)(micros()-start)<3000) { /* BLE interrupts stay enabled */ }
  bool ok=t->EVENTS_STOPPED && !t->EVENTS_ERROR &&
          t->TXD.AMOUNT==tn && (!rn||t->RXD.AMOUNT==rn);
  if(!ok) {
    t->TASKS_STOP=1;
    uint32_t stopAt=micros();
    while(!t->EVENTS_STOPPED && (uint32_t)(micros()-stopAt)<1000) {}
    Wire1.end(); busReady=false;
  }
  t->SHORTS=0;
  return ok;
}
bool readReg(uint8_t reg,uint8_t* out,uint8_t len) {
  uint8_t tx[1]={reg}; return transfer(tx,1,out,len);
}
bool writeReg(uint8_t reg,uint8_t value) {
  uint8_t tx[2]={reg,value}; return transfer(tx,2,nullptr,0);
}
bool startIMU() {
  if(!motion::validRemap(AXIS_REMAP)) return false;
  pinMode(PIN_WIRE1_SDA,INPUT); // NO internal pull-ups for the check.
  pinMode(PIN_WIRE1_SCL,INPUT);
  for(unsigned i=0;i<10;++i) {
    if(!digitalRead(PIN_WIRE1_SDA)||!digitalRead(PIN_WIRE1_SCL)) return false;
    delayMicroseconds(100);
  }
  Wire1.begin(); Wire1.setClock(400000); busReady=true;
  uint8_t who=0;
  // Sense's fixed onboard address, no external bus scanning.
  if(!readReg(0x0F,&who,1)||who!=0x6A) return false;
  if(!writeReg(0x12,0x01)) return false; // software reset
  uint32_t resetAt=millis();
  do {
    delay(1);
    if(!readReg(0x12,&who,1)) return false;
    if((uint32_t)(millis()-resetAt)>100) return false;
  } while(who&1);
  // BDU + auto-increment; 208Hz, +/-8g; 208Hz, +/-2000 deg/s.
  if(!writeReg(0x12,0x44)||!writeReg(0x10,0x5C)||
     !writeReg(0x11,0x5C)||!writeReg(0x13,0x02)) return false;
  // CTRL4_C bit1 LPF1_SEL_G enables gyro LPF1, other defaults retained.
  uint8_t regs[3];
  if(!readReg(0x10,regs,3)||regs[0]!=0x5C||
     regs[1]!=0x5C||regs[2]!=0x44) return false;
  bias.start(); lastSample=0; startedAt=millis();
  return true;
}
int16_t signedLE(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8));
}
void sensorStep() {
  if(!imuOK) return;
  uint32_t now=micros();
  if((uint32_t)(now-lastPoll)<1000) return;
  lastPoll=now;
  uint8_t status=0;
  if(!readReg(0x1E,&status,1)) { imuOK=false; return; }
  if((status&3)!=3) {
    if((uint32_t)(millis()-startedAt)>100 &&
       (!haveSample || (uint32_t)(now-lastSample)>100000)) imuOK=false;
    return;
  }
  uint8_t raw[12];
  if(!readReg(0x22,raw,12)) { imuOK=false; return; }
  Vec nativeG={signedLE(raw)*0.070f,signedLE(raw+2)*0.070f,signedLE(raw+4)*0.070f};
  Vec nativeA={signedLE(raw+6)*0.000244f,signedLE(raw+8)*0.000244f,signedLE(raw+10)*0.000244f};
  bool calibrated=bias.sample(nativeA,nativeG);
  accel=motion::rotate(motion::normalise(MOUNT_ROTATION),motion::remap(nativeA,AXIS_REMAP));
  gyro=motion::rotate(motion::normalise(MOUNT_ROTATION),motion::remap(bias.subtract(nativeG),AXIS_REMAP));
  uint32_t sampledAt=micros();
  float dt=lastSample?(uint32_t)(sampledAt-lastSample)*1e-6f:1.0f/208;
  lastSample=sampledAt; haveSample=true;
  if(calibrated) { fusion.q=motion::tiltFromGravity(accel); reference={1,0,0,0}; }
  // Suppress motion packets and fusion while calibrating; BLE remains active.
  if(!bias.active) {
    if(dt>0.03f) dt=1.0f/208; // recover after a long scheduler pause
    fusion.update({gyro.x/motion::DEG,gyro.y/motion::DEG,gyro.z/motion::DEG},accel,dt);
  }
}
void commands() {
  while(uart.available()) {
    switch(uart.read()) {
      case 'b': textLength=textPosition=0; binary=true; streamHz=50; lastStream=micros(); break;
      case 't': textLength=textPosition=0; binary=false; streamHz=20; lastStream=micros(); break;
      case 'z': reference=motion::conjugate(fusion.q); break;
      case 'c': if(imuOK) bias.start(); break;
      case '1': streamHz=20; break;
      case '2': streamHz=50; break;
      case '3': streamHz=100; break;
      default: break;
    }
  }
}
void stream() {
  uint32_t now=micros();
  if(!imuOK||!haveSample||bias.active||!Bluefruit.connected()||!uart.notifyEnabled()) return;
  // Finish a CSV line before starting another, even at MTU 23.
  // No sample queue: if the link is slow, keep only this partial line.
  if(!binary && textPosition<textLength) {
    uint16_t chunk=textLength-textPosition;
    if(chunk>20) chunk=20;
    if(uart.send((const uint8_t*)textLine+textPosition,chunk)) textPosition+=chunk;
    return;
  }
  if((uint32_t)(now-lastStream)<1000000UL/streamHz) return;
  lastStream=now; // drop obsolete samples; never build a latency backlog.
  Quat q=motion::normalise(motion::multiply(reference,fusion.q));
  if(binary) {
    uint8_t packet[20];
    motion::encode(packet,q,accel,gyro);
    uart.send(packet,sizeof(packet)); // one exact 20-byte notification; drop if busy
  } else {
    Vec e=motion::euler(q);
    int len=snprintf(textLine,sizeof(textLine),"%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
      (double)accel.x,(double)accel.y,(double)accel.z,
      (double)gyro.x,(double)gyro.y,(double)gyro.z,(double)e.x,(double)e.y,(double)e.z);
    textPosition=0;
    textLength=(len>0 && len<(int)sizeof(textLine))?(uint16_t)len:0;
  }
}
void batteryStep() {
  static bool measuring=false;
  static uint32_t enabledAt=0;
  uint32_t now=millis();
  if(!measuring && (lastBattery==0||(uint32_t)(now-lastBattery)>=10000)) {
    digitalWrite(VBAT_ENABLE,LOW); enabledAt=now; measuring=true; return;
  }
  if(measuring && (uint32_t)(now-enabledAt)>=5) {
    float volts=analogRead(PIN_VBAT)*(3.0f/4095.0f)*(1510.0f/510.0f);
    // Approximate resting single-cell LiPo state of charge, not a fuel gauge.
    const float v[11]={3.30f,3.60f,3.70f,3.74f,3.77f,3.80f,3.85f,3.90f,3.98f,4.08f,4.20f};
    uint8_t level=0;
    if(volts>=v[10]) level=100;
    else for(int i=0;i<10;++i) if(volts>=v[i] && volts<v[i+1]) {
      level=(uint8_t)(10*i+10*(volts-v[i])/(v[i+1]-v[i])); break;
    }
    battery.write(level); battery.notify(level);
    digitalWrite(VBAT_ENABLE,HIGH); measuring=false; lastBattery=now;
  }
}
void connected(uint16_t) { resetSession=true; }
void disconnected(uint16_t,uint8_t) { resetSession=true; }
void setup() {
  pinMode(LED_RED,OUTPUT); pinMode(LED_GREEN,OUTPUT); pinMode(LED_BLUE,OUTPUT);
  digitalWrite(LED_RED,HIGH); digitalWrite(LED_GREEN,HIGH); digitalWrite(LED_BLUE,HIGH);
  Serial.begin(115200); // never wait for USB
  // Bluetooth FIRST, before power, bus checks or sensor/calibration work.
  Bluefruit.autoConnLed(false);
  Bluefruit.configPrphConn(23,BLE_GAP_EVENT_LENGTH_DEFAULT,16,4);
  Bluefruit.begin(1,0);
  Bluefruit.setName("XIAO-IMU"); Bluefruit.setTxPower(4);
  Bluefruit.Periph.setConnectCallback(connected);
  Bluefruit.Periph.setDisconnectCallback(disconnected);
  Bluefruit.Periph.setConnInterval(6,12); // 7.5..15 ms, phone may choose otherwise
  uart.begin(); uart.bufferTXD(false);
  battery.begin(); battery.write(0);
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(uart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32,160);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  pinMode(VBAT_ENABLE,OUTPUT); digitalWrite(VBAT_ENABLE,HIGH);
  analogReadResolution(12); analogReference(AR_INTERNAL_3_0); analogSampleTime(40);
  pinMode(PIN_LSM6DS3TR_C_POWER,OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER,HIGH);
  powerAt=millis();
}
void loop() {
  if(resetSession) {
    resetSession=false; binary=false; streamHz=20;
    uart.flush(); textLength=textPosition=0; lastStream=micros();
  }
  if(!initialised && (uint32_t)(millis()-powerAt)>=50) {
    imuOK=startIMU(); initialised=true;
    if(!imuOK && busReady) { Wire1.end(); busReady=false; }
    Serial.println(imuOK?"IMU ready. Keep still for gyro calibration.":"IMU fault. Bluetooth stays available.");
  }
  commands(); sensorStep(); stream(); batteryStep(); updateLED();
  delay(1);
}

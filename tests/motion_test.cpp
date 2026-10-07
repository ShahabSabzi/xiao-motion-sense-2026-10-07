#include "../firmware/XiaoMotion/MotionMath.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <initializer_list>
using namespace motion;
void check(bool ok,const char* msg) { if(!ok) { fprintf(stderr,"FAIL: %s\n",msg); exit(1); } }
bool near(float a,float b,float tolerance=0.05f) { return fabsf(a-b)<tolerance; }
int main() {
  Madgwick m;
  for(int i=0;i<100000;++i) m.update({0,0,0},{0,0,1},1.f/208);
  check(isfinite(m.q.w)&&near(m.q.w,1)&&near(m.q.x,0),"exact level zero-gradient stays finite");
  m.q={0,0,0,0}; m.update({0,0,0},{0,0,0},1.f/208);
  check(near(m.q.w,1),"invalid quaternion/zero acceleration recovers");
  m.beta=0;
  for(int i=0;i<208;++i) m.update({0,0,PI_F/2},{0,0,1},1.f/208);
  check(near(euler(m.q).z,90,0.01f),"integrated +90-degree yaw");
  m.q={1,0,0,0};
  for(int i=0;i<208;++i) m.update({PI_F/2,0,0},{0,0,0},1.f/208);
  check(near(euler(m.q).x,90,0.01f),"integrated +90-degree roll");
  m.q={1,0,0,0};
  for(int i=0;i<208;++i) m.update({0,PI_F/2,0},{0,0,0},1.f/208);
  check(near(euler(m.q).y,90,0.05f),"integrated pitch and asin clamp");
  m=Madgwick();
  Vec a={0,0.5f,0.8660254038f};
  for(int i=0;i<4160;++i) m.update({0,0,0},a,1.f/208);
  check(near(euler(m.q).x,30,0.15f),"accelerometer converges to 30-degree roll");
  for(int axis=0;axis<3;++axis) {
    Quat truth={0.9238795f,0,0,0};
    if(axis==0) truth.x=0.3826834f;
    if(axis==1) truth.y=0.3826834f;
    if(axis==2) truth.z=0.3826834f;
    Quat zero=multiply(conjugate(truth),truth);
    check(near(zero.w,1)&&near(zero.x,0)&&near(zero.y,0)&&near(zero.z,0),"full quaternion reference zero");
  }
  for(Vec gravity: {Vec{0,0,-1},Vec{0,1,0},Vec{1,0,0},Vec{0,0.5f,0.8660254f}}) {
    Quat tilted=tiltFromGravity(gravity);
    Vec world=rotate(tilted,gravity);
    check(near(world.x,0)&&near(world.y,0)&&near(world.z,1),"tilt initialisation including upside down");
  }
  int8_t map[3]={2,-1,3};
  check(validRemap(map),"right-handed axis swap");
  Vec v=remap({1,2,3},map);
  check(v.x==2&&v.y==-1&&v.z==3,"remap values");
  int8_t bad[3]={1,2,-3}, duplicate[3]={1,1,3};
  check(!validRemap(bad)&&!validRemap(duplicate),"reject reflections and duplicates");
  GyroBias b;
  for(int i=0;i<415;++i) check(!b.sample({0,0,1},{1,-2,0.5f}),"bias window");
  check(b.sample({0,0,1},{1,-2,0.5f})&&!b.active,"bias completes");
  v=b.subtract({1,-2,0.5f});
  check(near(v.x,0)&&near(v.y,0)&&near(v.z,0),"bias subtraction");
  b.start(); b.sample({0,0,1},{0,0,0}); b.sample({0,0,1},{10,0,0});
  check(b.count==0&&b.active,"movement resets calibration");
  uint8_t bytes[22]; memset(bytes,0xA5,sizeof(bytes));
  encode(bytes+1,{1,0,0,0},{1,-2,0.5f},{90,-2000,2000});
  const uint8_t expected[20]={0x00,0x40,0,0,0,0,0,0,0xE8,0x03,0x30,0xF8,0xF4,0x01,0xA0,0x05,0x00,0x83,0x00,0x7D};
  check(memcmp(bytes+1,expected,20)==0,"exact little-endian golden 20-byte packet");
  check(bytes[0]==0xA5&&bytes[21]==0xA5,"exact length, no buffer overwrite");
  check(quantise(100000,1000)==32767&&quantise(-100000,1000)==-32768,"saturation");
  // Deterministic fixture decoded independently in Java and JavaScript.
  FILE* f=fopen("packet.bin","wb"); check(f!=nullptr,"fixture write");
  fwrite(bytes+1,1,20,f); fclose(f);
  printf("PASS: level, zero acceleration, gyro axes, tilt convergence, reference zero, remap, bias, packet and saturation\n");
}

package org.xiao.motion;
import android.annotation.SuppressLint;
import android.bluetooth.*;
import android.content.Context;
import android.os.Build;
import android.os.Handler;
import java.util.ArrayDeque;
import java.util.UUID;

@SuppressLint("MissingPermission") // MainActivity gates each entry with runtime permission.
public final class BleLink {
    public static final UUID SERVICE=UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    private static final UUID RX=UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
    private static final UUID TX=UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
    private static final UUID CCC=UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
    private static final UUID BAS=UUID.fromString("0000180f-0000-1000-8000-00805f9b34fb");
    private static final UUID LEVEL=UUID.fromString("00002a19-0000-1000-8000-00805f9b34fb");
    public interface Listener {
        void status(String s,boolean ready);
        void sample(MotionPacket p);
        void battery(int n);
    }
    private final Context context;
    private final Handler h;
    private final Listener listener;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic rx,tx,battery;
    private BluetoothGattDescriptor ccc;
    private final ArrayDeque<Character> commands=new ArrayDeque<>();
    private String operation="";
    private boolean ready=false,closed=true;
    private long generation=0;
    public BleLink(Context c,Handler handler,Listener l) { context=c;h=handler;listener=l; }
    public boolean active() { return !closed; }
    public void connect(BluetoothDevice device) {
        close();closed=false;++generation;listener.status("Connecting…",false);
        try {
            gatt=device.connectGatt(context,false,callback,BluetoothDevice.TRANSPORT_LE);
            if(gatt==null) { fail("Could not open Bluetooth connection");return; }
            armTimeout("Connection timed out",15000);
        } catch(SecurityException e) { fail("Bluetooth permission was removed"); }
    }
    private void armTimeout(String message,int ms) {
        final long gen=++generation;
        h.postDelayed(()->{if(!closed&&generation==gen) fail(message);},ms);
    }
    private void cancelTimeout() { ++generation; }
    private void fail(String s) { close();listener.status(s,false); }
    public void close() {
        cancelTimeout();closed=true;ready=false;
        commands.clear();operation="";rx=null;tx=null;battery=null;ccc=null;
        BluetoothGatt old=gatt;gatt=null;
        if(old!=null) { try { old.disconnect(); } catch(SecurityException ignored) {} old.close(); }
    }
    public void send(char c) {
        if(!ready||closed||commands.size()>8) return;
        commands.add(c);pump();
    }
    public void readBattery() {
        if(ready&&!closed&&operation.isEmpty()&&commands.isEmpty()&&battery!=null) {
            operation="battery";
            try {
                if(!gatt.readCharacteristic(battery)) {operation="";return;}
                armTimeout("Battery read timed out; reconnect",8000);
            } catch(SecurityException e) {fail("Bluetooth permission was removed");}
        }
    }
    private void pump() {
        if(closed||!operation.isEmpty()||commands.isEmpty()) return;
        char c=commands.remove();operation="write:"+c;
        byte[] value={(byte)c};boolean ok;
        try {
            if(Build.VERSION.SDK_INT>=33)
                ok=gatt.writeCharacteristic(rx,value,BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)==BluetoothStatusCodes.SUCCESS;
            else {
                rx.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
                rx.setValue(value);ok=gatt.writeCharacteristic(rx);
            }
        } catch(SecurityException e) {fail("Bluetooth permission was removed");return;}
        if(!ok) {fail("Command could not be sent; reconnect");return;}
        armTimeout("Command timed out; reconnect",8000);
    }
    private final BluetoothGattCallback callback=new BluetoothGattCallback() {
        @Override public void onConnectionStateChange(BluetoothGatt g,int status,int state) {
            h.post(()->{
                if(g!=gatt||closed) return;
                cancelTimeout();
                if(status!=BluetoothGatt.GATT_SUCCESS||state==BluetoothProfile.STATE_DISCONNECTED) {
                    fail("Disconnected ("+status+"). Tap Connect to retry.");return;
                }
                if(state==BluetoothProfile.STATE_CONNECTED) {
                    listener.status("Finding motion service…",false);
                    try {
                        if(!g.discoverServices()) {fail("Service discovery failed");return;}
                        armTimeout("Service discovery timed out",10000);
                    } catch(SecurityException e) {fail("Bluetooth permission was removed");}
                }
            });
        }
        @Override public void onServicesDiscovered(BluetoothGatt g,int status) {
            h.post(()->{
                if(g!=gatt||closed) return;
                cancelTimeout();
                BluetoothGattService service=g.getService(SERVICE);
                if(status!=BluetoothGatt.GATT_SUCCESS||service==null) {fail("Nordic UART Service not found");return;}
                rx=service.getCharacteristic(RX);tx=service.getCharacteristic(TX);
                if(rx==null||tx==null||(rx.getProperties()&BluetoothGattCharacteristic.PROPERTY_WRITE)==0||
                    (tx.getProperties()&BluetoothGattCharacteristic.PROPERTY_NOTIFY)==0) {
                    fail("UART characteristics are incompatible");return;
                }
                ccc=tx.getDescriptor(CCC);
                BluetoothGattService bas=g.getService(BAS);
                battery=bas==null?null:bas.getCharacteristic(LEVEL);
                try {
                    if(ccc==null||!g.setCharacteristicNotification(tx,true)) {fail("Cannot subscribe to motion");return;}
                    operation="subscribe";boolean ok;
                    if(Build.VERSION.SDK_INT>=33)
                        ok=g.writeDescriptor(ccc,BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)==BluetoothStatusCodes.SUCCESS;
                    else {ccc.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);ok=g.writeDescriptor(ccc);}
                    if(!ok) {fail("Notification subscription failed");return;}
                    armTimeout("Notification subscription timed out",8000);
                } catch(SecurityException e) {fail("Bluetooth permission was removed");}
            });
        }
        @Override public void onDescriptorWrite(BluetoothGatt g,BluetoothGattDescriptor d,int status) {
            h.post(()->{
                if(g!=gatt||closed||d!=ccc||!operation.equals("subscribe")) return;
                cancelTimeout();operation="";
                if(status!=BluetoothGatt.GATT_SUCCESS) {fail("Notification subscription rejected");return;}
                commands.add('b');pump(); // only after CCC acknowledgement
            });
        }
        @Override public void onCharacteristicWrite(BluetoothGatt g,BluetoothGattCharacteristic c,int status) {
            h.post(()->{
                if(g!=gatt||closed||c!=rx||!operation.startsWith("write:")) return;
                String finished=operation;cancelTimeout();operation="";
                if(status!=BluetoothGatt.GATT_SUCCESS) {fail("Board rejected command");return;}
                if(finished.equals("write:b")) {
                    ready=true;listener.status("Connected • waiting for motion (keep board still)",true);
                }
                pump();if(finished.equals("write:b")) readBattery();
            });
        }
        @Override public void onCharacteristicChanged(BluetoothGatt g,BluetoothGattCharacteristic c) {
            changed(g,c,c.getValue()==null?null:c.getValue().clone());
        }
        @Override public void onCharacteristicChanged(BluetoothGatt g,BluetoothGattCharacteristic c,byte[] value) {
            changed(g,c,value.clone());
        }
        private void changed(BluetoothGatt g,BluetoothGattCharacteristic c,byte[] value) {
            h.post(()->{
                if(g!=gatt||closed||!ready||c!=tx) return;
                MotionPacket p=MotionPacket.decode(value);if(p!=null) listener.sample(p);
            });
        }
        @Override public void onCharacteristicRead(BluetoothGatt g,BluetoothGattCharacteristic c,int status) {
            read(g,c,c.getValue()==null?null:c.getValue().clone(),status);
        }
        @Override public void onCharacteristicRead(BluetoothGatt g,BluetoothGattCharacteristic c,byte[] value,int status) {
            read(g,c,value.clone(),status);
        }
        private void read(BluetoothGatt g,BluetoothGattCharacteristic c,byte[] value,int status) {
            h.post(()->{
                if(g!=gatt||closed||c!=battery||!operation.equals("battery")) return;
                cancelTimeout();operation="";
                if(status==BluetoothGatt.GATT_SUCCESS&&value!=null&&value.length>0) listener.battery(Math.min(100,value[0]&255));
                pump();
            });
        }
    };
}

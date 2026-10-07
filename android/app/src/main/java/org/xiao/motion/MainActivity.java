package org.xiao.motion;
import android.Manifest;
import android.annotation.SuppressLint;
import android.app.*;
import android.bluetooth.*;
import android.bluetooth.le.*;
import android.content.*;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.location.LocationManager;
import android.os.*;
import android.provider.Settings;
import android.view.*;
import android.widget.*;
import java.util.*;

@SuppressLint("MissingPermission")
public final class MainActivity extends Activity implements BleLink.Listener {
    private final Handler h=new Handler(Looper.getMainLooper());
    private BluetoothAdapter adapter;
    private BluetoothLeScanner scanner;
    private BleLink link;
    private MotionView motion;
    private TextView status,battery;
    private Button connect,zero,calibrate;
    private Spinner rate;
    private AlertDialog picker;
    private ArrayAdapter<String> names;
    private final ArrayList<BluetoothDevice> devices=new ArrayList<>();
    private boolean scanning=false,ready=false,destroying=false;
    private int scanGeneration=0;
    private long lastPacket=0,rateAt=0;
    private int packetCount=0;
    private static final int PERMISSIONS=7,ENABLE_BT=8;
    private int dp(float v) {return (int)(v*getResources().getDisplayMetrics().density+0.5f);}
    @Override public void onCreate(Bundle saved) {
        super.onCreate(saved);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        LinearLayout root=new LinearLayout(this);root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(Color.rgb(16,26,40));root.setPadding(dp(16),dp(10),dp(16),dp(10));
        root.setOnApplyWindowInsetsListener((v,insets)->{
            v.setPadding(dp(16)+insets.getSystemWindowInsetLeft(),dp(10)+insets.getSystemWindowInsetTop(),
                dp(16)+insets.getSystemWindowInsetRight(),dp(10)+insets.getSystemWindowInsetBottom());
            return insets;
        });
        TextView title=new TextView(this);title.setText("XIAO / MOTION");title.setTextSize(23);title.setTextColor(Color.WHITE);root.addView(title);
        status=new TextView(this);status.setTextSize(13);status.setTextColor(Color.rgb(170,190,210));
        status.setText("Connect your XIAO • keep still at boot");root.addView(status);
        LinearLayout bar=new LinearLayout(this);
        connect=new Button(this);connect.setText("Connect");connect.setOnClickListener(v->begin());
        bar.addView(connect,new LinearLayout.LayoutParams(0,dp(48),1));
        zero=new Button(this);zero.setText("Zero");zero.setEnabled(false);
        zero.setOnClickListener(v->link.send('z'));bar.addView(zero,new LinearLayout.LayoutParams(0,dp(48),1));
        calibrate=new Button(this);calibrate.setText("Calibrate");calibrate.setEnabled(false);
        calibrate.setOnClickListener(v->new AlertDialog.Builder(this).setTitle("Measure gyro bias")
            .setMessage("Set the board down and keep it still for about 2 seconds. Motion pauses until calibration completes.")
            .setPositiveButton("Start",(d,w)->{link.send('c');status.setText("Calibrating • keep still for 2 seconds");})
            .setNegativeButton("Cancel",null).show());
        bar.addView(calibrate,new LinearLayout.LayoutParams(0,dp(48),1));root.addView(bar);
        LinearLayout info=new LinearLayout(this);info.setGravity(Gravity.CENTER_VERTICAL);
        battery=new TextView(this);battery.setText("Battery —");battery.setTextColor(Color.LTGRAY);
        info.addView(battery,new LinearLayout.LayoutParams(0,dp(38),1));
        rate=new Spinner(this);
        rate.setAdapter(new ArrayAdapter<>(this,android.R.layout.simple_spinner_dropdown_item,new String[]{"20 Hz","50 Hz","100 Hz"}));
        rate.setSelection(1);rate.setEnabled(false);
        rate.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            public void onNothingSelected(AdapterView<?> p) {}
            public void onItemSelected(AdapterView<?> p,View v,int pos,long id) {if(ready) link.send((char)('1'+pos));}
        });
        info.addView(rate,new LinearLayout.LayoutParams(dp(120),dp(38)));root.addView(info);
        motion=new MotionView(this);root.addView(motion,new LinearLayout.LayoutParams(-1,0,1));
        TextView note=new TextView(this);note.setText("X red  •  Y green  •  Z blue    |    yaw drifts without a magnetometer");
        note.setTextColor(Color.rgb(140,162,183));note.setTextSize(11);root.addView(note);
        setContentView(root);
        BluetoothManager manager=(BluetoothManager)getSystemService(BLUETOOTH_SERVICE);
        adapter=manager==null?null:manager.getAdapter();link=new BleLink(this,h,this);h.postDelayed(ticker,1000);
    }
    private boolean permitted() {
        if(Build.VERSION.SDK_INT>=31) return checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN)==PackageManager.PERMISSION_GRANTED&&
            checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT)==PackageManager.PERMISSION_GRANTED;
        return checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION)==PackageManager.PERMISSION_GRANTED;
    }
    private void begin() {
        if(scanning) {stopScan();return;}
        if(link.active()) {link.close();status("Disconnected",false);return;}
        if(adapter==null) {status("This phone has no Bluetooth LE adapter",false);return;}
        if(!permitted()) {
            requestPermissions(Build.VERSION.SDK_INT>=31?
                new String[]{Manifest.permission.BLUETOOTH_SCAN,Manifest.permission.BLUETOOTH_CONNECT}:
                new String[]{Manifest.permission.ACCESS_FINE_LOCATION},PERMISSIONS);return;
        }
        try {
            if(!adapter.isEnabled()) {startActivityForResult(new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE),ENABLE_BT);return;}
            if(Build.VERSION.SDK_INT<31) {
                LocationManager lm=(LocationManager)getSystemService(LOCATION_SERVICE);
                boolean enabled=lm!=null&&(lm.isProviderEnabled(LocationManager.GPS_PROVIDER)||lm.isProviderEnabled(LocationManager.NETWORK_PROVIDER));
                if(!enabled) {
                    new AlertDialog.Builder(this).setTitle("Enable Location for Bluetooth scanning")
                        .setMessage("Android 6–11 requires Location to find nearby Bluetooth devices. This app does not collect your location.")
                        .setPositiveButton("Settings",(d,w)->startActivity(new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS)))
                        .setNegativeButton("Cancel",null).show();return;
                }
            }
            scan();
        } catch(SecurityException e) {status("Allow Bluetooth in app permissions",false);}
    }
    private void scan() {
        scanner=adapter.getBluetoothLeScanner();
        if(scanner==null) {status("Bluetooth is unavailable",false);return;}
        devices.clear();names=new ArrayAdapter<>(this,android.R.layout.simple_list_item_1);
        picker=new AlertDialog.Builder(this).setTitle("Choose your XIAO")
            .setAdapter(names,(d,index)->{
                BluetoothDevice selected=devices.get(index);stopScan();motion.clear();
                lastPacket=0;packetCount=0;battery.setText("Battery —");link.connect(selected);
            }).setNegativeButton("Cancel",(d,w)->stopScan()).create();
        picker.setOnCancelListener(d->stopScan());picker.show();
        scanning=true;connect.setText("Stop scan");status.setText("Scanning for XIAO-IMU / Nordic UART…");
        List<ScanFilter> filters=Arrays.asList(
            new ScanFilter.Builder().setServiceUuid(new ParcelUuid(BleLink.SERVICE)).build(),
            new ScanFilter.Builder().setDeviceName("XIAO-IMU").build());
        try {scanner.startScan(filters,new ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(),scanCallback);}
        catch(SecurityException|IllegalStateException e) {stopScan();status("Cannot scan. Check Bluetooth permissions.",false);return;}
        final int gen=++scanGeneration;
        h.postDelayed(()->{
            if(scanning&&scanGeneration==gen) {
                stopScan();status(devices.isEmpty()?"No XIAO found. Check blue advertising LED and retry.":"Scan ended. Tap Connect to retry.",false);
            }
        },15000);
    }
    private final ScanCallback scanCallback=new ScanCallback() {
        @Override public void onScanResult(int type,ScanResult result) {
            h.post(()->{
                if(!scanning||destroying) return;
                BluetoothDevice d=result.getDevice();if(devices.contains(d)) return;
                devices.add(d);
                String name=result.getScanRecord()==null?null:result.getScanRecord().getDeviceName();
                names.add((name==null?"Nordic UART device":name)+"\n"+d.getAddress()+"  •  "+result.getRssi()+" dBm");
            });
        }
        @Override public void onScanFailed(int error) {
            h.post(()->{stopScan();status("Scan failed ("+error+"). Retry Bluetooth.",false);});
        }
    };
    private void stopScan() {
        ++scanGeneration;
        if(scanning&&scanner!=null) {try {scanner.stopScan(scanCallback);}catch(SecurityException|IllegalStateException ignored) {}}
        scanning=false;scanner=null;if(picker!=null){picker.dismiss();picker=null;}connect.setText("Connect");
    }
    @Override public void onRequestPermissionsResult(int request,String[] permissions,int[] grants) {
        super.onRequestPermissionsResult(request,permissions,grants);
        if(request==PERMISSIONS) {
            if(permitted()) begin();
            else {
                status("Bluetooth permission denied",false);
                new AlertDialog.Builder(this).setTitle("Bluetooth access needed")
                    .setMessage("Allow Nearby devices (Android 12+) or Location (Android 6–11) to connect.")
                    .setPositiveButton("App settings",(d,w)->startActivity(new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS,android.net.Uri.parse("package:"+getPackageName()))))
                    .setNegativeButton("Cancel",null).show();
            }
        }
    }
    @Override protected void onActivityResult(int request,int result,Intent data) {
        super.onActivityResult(request,result,data);if(request==ENABLE_BT&&result==RESULT_OK) begin();
    }
    @Override public void status(String text,boolean available) {
        ready=available;status.setText(text);connect.setText(link!=null&&link.active()?"Disconnect":"Connect");
        zero.setEnabled(available);calibrate.setEnabled(available);
        if(available&&!rate.isEnabled()) rate.setSelection(1);
        rate.setEnabled(available);
    }
    @Override public void battery(int level) {battery.setText("Battery ~"+level+"%");}
    @Override public void sample(MotionPacket p) {
        lastPacket=SystemClock.elapsedRealtime();packetCount++;motion.accept(p);
    }
    private final Runnable ticker=new Runnable() {
        public void run() {
            if(destroying) return;
            long now=SystemClock.elapsedRealtime();
            if(ready) {
                if(lastPacket==0||now-lastPacket>3000) status.setText("No motion • keep still to calibrate; check red fault LED");
                else if(now-lastPacket>800) status.setText("Motion paused • calibrating or link stalled");
                else status.setText("Live motion • "+packetCount+" samples/s");
                packetCount=0;
                if(now-rateAt>10000) {rateAt=now;link.readBattery();}
            }
            h.postDelayed(this,1000);
        }
    };
    @Override protected void onStop() {super.onStop();stopScan();link.close();status("Paused • tap Connect to resume",false);}
    @Override protected void onDestroy() {destroying=true;h.removeCallbacksAndMessages(null);link.close();super.onDestroy();}
}

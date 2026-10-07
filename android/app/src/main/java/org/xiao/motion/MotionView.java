package org.xiao.motion;
import android.content.Context;
import android.graphics.*;
import android.view.View;
import java.util.*;

public final class MotionView extends View {
    private final Paint paint=new Paint(Paint.ANTI_ALIAS_FLAG);
    private final int[] colors={Color.rgb(255,120,120),Color.rgb(83,225,179),Color.rgb(106,168,255)};
    private final float[][] history=new float[6][500];
    private final long[] times=new long[500];
    private int head=0,count=0;
    private MotionPacket packet;
    private final float density;
    private static final class Face {
        final float[][] v;final int color;float z;float[][] p;
        Face(float[][] v,int color) {this.v=v;this.color=color;}
    }
    private final ArrayList<Face> model=new ArrayList<>();
    public MotionView(Context c) {
        super(c);density=getResources().getDisplayMetrics().density;
        box(0,0,0,1.55f,2.05f,0.10f,Color.rgb(28,132,97));
        box(0,-0.72f,0.13f,0.70f,0.55f,0.28f,Color.rgb(184,200,208));
        box(0,0.15f,0.14f,1.02f,0.93f,0.22f,Color.rgb(67,85,97));
        box(0,0.82f,0.08f,0.84f,0.22f,0.07f,Color.rgb(30,38,50));
        for(int side=-1;side<=1;side+=2) for(int i=0;i<7;i++)
            box(side*0.72f,-0.82f+i*0.27f,0.06f,0.19f,0.13f,0.06f,Color.rgb(224,185,91));
    }
    private void box(float x,float y,float z,float w,float h,float t,int color) {
        float[][] v=new float[8][3];
        for(int i=0;i<8;i++) v[i]=new float[]{x+((i&1)==0?-w:w)/2,y+((i&2)==0?-h:h)/2,z+((i&4)==0?-t:t)/2};
        int[][] faces={{0,2,3,1},{4,5,7,6},{0,1,5,4},{2,6,7,3},{0,4,6,2},{1,3,7,5}};
        for(int[] f:faces) model.add(new Face(new float[][]{v[f[0]],v[f[1]],v[f[2]],v[f[3]]},color));
    }
    public void clear() {packet=null;head=0;count=0;invalidate();}
    public void accept(MotionPacket p) {
        packet=p;times[head]=android.os.SystemClock.elapsedRealtime();
        for(int i=0;i<3;i++) {history[i][head]=p.a[i];history[i+3][head]=p.g[i];}
        head=(head+1)%500;count=Math.min(500,count+1);postInvalidateOnAnimation();
    }
    private void text(Canvas c,String s,float x,float y,float size,int color) {
        paint.setStyle(Paint.Style.FILL);paint.setColor(color);paint.setTextSize(size*density);c.drawText(s,x,y,paint);
    }
    private float[] transform(float[] v,float[] q) {
        float w=q[0],x=q[1],y=q[2],z=q[3];
        float tx=2*(y*v[2]-z*v[1]),ty=2*(z*v[0]-x*v[2]),tz=2*(x*v[1]-y*v[0]);
        float a=v[0]+w*tx+y*tz-z*ty,b=v[1]+w*ty+z*tx-x*tz,d=v[2]+w*tz+x*ty-y*tx;
        return new float[]{a,0.88f*b-0.475f*d,0.475f*b+0.88f*d};
    }
    private float[] project(float[] v,float cx,float cy,float scale) {
        float perspective=5f/(5f-v[2]);return new float[]{cx+v[0]*scale*perspective,cy-v[1]*scale*perspective};
    }
    @Override protected void onDraw(Canvas c) {
        super.onDraw(c);
        float width=getWidth(),height=getHeight();if(width<=0||height<=0) return;
        float[] q=packet==null?new float[]{1,0,0,0}:packet.q;
        float boardHeight=height*0.47f,scale=Math.min(width*0.21f,boardHeight*0.29f),cx=width/2,cy=boardHeight*0.51f;
        for(Face f:model) {
            f.p=new float[4][2];f.z=0;
            for(int i=0;i<4;i++) {float[] v=transform(f.v[i],q);f.z+=v[2]/4;f.p[i]=project(v,cx,cy,scale);}
        }
        ArrayList<Face> sorted=new ArrayList<>(model);
        Collections.sort(sorted,(a,b)->Float.compare(a.z,b.z));
        for(Face f:sorted) {
            Path path=new Path();path.moveTo(f.p[0][0],f.p[0][1]);
            for(int i=1;i<4;i++) path.lineTo(f.p[i][0],f.p[i][1]);path.close();
            paint.setStyle(Paint.Style.FILL);paint.setColor(f.color);c.drawPath(path,paint);
            paint.setStyle(Paint.Style.STROKE);paint.setStrokeWidth(density*0.6f);
            paint.setColor(Color.argb(95,9,22,30));c.drawPath(path,paint);
        }
        float[] origin=project(transform(new float[]{0,0,0.4f},q),cx,cy,scale);
        for(int i=0;i<3;i++) {
            float[] end={0,0,0.4f};end[i]+=1.45f;
            float[] e=project(transform(end,q),cx,cy,scale);
            paint.setColor(colors[i]);paint.setStrokeWidth(2*density);c.drawLine(origin[0],origin[1],e[0],e[1],paint);
            text(c,new String[]{"X","Y","Z"}[i],e[0]+4,e[1],12,colors[i]);
        }
        float y=boardHeight;
        for(int i=0;i<3;i++) {
            String value=packet==null?"—":String.format(Locale.US,"%.1f°",packet.rpy[i]);
            text(c,new String[]{"ROLL","PITCH","YAW"}[i],width*(i/3f)+5,y,10,Color.rgb(145,167,186));
            text(c,value,width*(i/3f)+5,y+23*density,22,Color.WHITE);
        }
        float top=y+38*density,chartHeight=Math.max(30,(height-top)/2-10*density);
        chart(c,top,chartHeight,0,"ACCELERATION / g",1.2f);
        chart(c,top+chartHeight+12*density,chartHeight,3,"ANGULAR RATE / °/s",100);
        if(packet==null) text(c,"Waiting for live motion",10,20*density,12,Color.LTGRAY);
    }
    private void chart(Canvas c,float top,float height,int offset,String title,float minimum) {
        float width=getWidth(),label=17*density;
        text(c,title,4,top+12*density,10,Color.rgb(164,184,202));
        float range=minimum;long now=android.os.SystemClock.elapsedRealtime();
        for(int j=0;j<count;j++) {
            int idx=(head-1-j+500)%500;if(now-times[idx]>5000) break;
            for(int a=0;a<3;a++) range=Math.max(range,Math.abs(history[offset+a][idx])*1.08f);
        }
        float middle=top+label+(height-label)/2,span=(height-label)/2;
        paint.setColor(Color.rgb(51,67,84));paint.setStrokeWidth(density);c.drawLine(0,middle,width,middle,paint);
        text(c,String.format(Locale.US,"±%.1f",range),width-65*density,top+12*density,10,Color.rgb(164,184,202));
        int save=c.save();c.clipRect(0,top+label,width,top+height);
        for(int a=0;a<3;a++) {
            Path path=new Path();boolean first=true;
            for(int j=0;j<count;j++) {
                int idx=(head-1-j+500)%500;long age=now-times[idx];if(age>5000) break;
                float x=width*(1-age/5000f),y=middle-history[offset+a][idx]/range*span;
                if(first){path.moveTo(x,y);first=false;}else path.lineTo(x,y);
            }
            paint.setColor(colors[a]);paint.setStyle(Paint.Style.STROKE);paint.setStrokeWidth(1.6f*density);c.drawPath(path,paint);
        }
        c.restoreToCount(save);paint.setStyle(Paint.Style.FILL);
    }
}

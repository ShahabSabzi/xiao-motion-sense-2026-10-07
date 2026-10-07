package org.xiao.motion;
// Pure Java, compiled independently by the host tests.
public final class MotionPacket {
    public final float[] q=new float[4],a=new float[3],g=new float[3],rpy=new float[3];
    private static int read(byte[] b,int n) { return (short)((b[n]&255)|((b[n+1]&255)<<8)); }
    public static MotionPacket decode(byte[] b) {
        if(b==null||b.length!=20) return null;
        MotionPacket p=new MotionPacket();
        float norm=0;
        for(int i=0;i<4;i++) { p.q[i]=read(b,2*i)/16384f;norm+=p.q[i]*p.q[i]; }
        // Reject transitional CSV fragments and invalid quaternion data.
        if(norm<0.98f||norm>1.02f) return null;
        float inv=(float)(1/Math.sqrt(norm));
        for(int i=0;i<4;i++) p.q[i]*=inv;
        for(int i=0;i<3;i++) { p.a[i]=read(b,8+2*i)/1000f;p.g[i]=read(b,14+2*i)/16f; }
        float w=p.q[0],x=p.q[1],y=p.q[2],z=p.q[3];
        p.rpy[0]=(float)Math.toDegrees(Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)));
        p.rpy[1]=(float)Math.toDegrees(Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x)))));
        p.rpy[2]=(float)Math.toDegrees(Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)));
        return p;
    }
}

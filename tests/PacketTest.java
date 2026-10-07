package org.xiao.motion;
import java.nio.file.Files;
import java.nio.file.Paths;
public final class PacketTest {
    private static void check(boolean ok,String s) {if(!ok) throw new AssertionError(s);}
    public static void main(String[] args) throws Exception {
        byte[] bytes=Files.readAllBytes(Paths.get(args[0]));
        MotionPacket p=MotionPacket.decode(bytes);
        check(p!=null&&p.q[0]==1,"identity quaternion");
        check(p.a[0]==1&&p.a[1]==-2&&p.a[2]==0.5f,"acceleration");
        check(p.g[0]==90&&p.g[1]==-2000&&p.g[2]==2000,"signed rates");
        check(p.rpy[0]==0&&p.rpy[1]==0&&p.rpy[2]==0,"Euler");
        check(MotionPacket.decode(new byte[19])==null,"length rejection");
        check(MotionPacket.decode(new byte[20])==null,"invalid quaternion");
        bytes[0]=0x41;bytes[1]=0x2D;bytes[2]=0x41;bytes[3]=0x2D;
        p=MotionPacket.decode(bytes);
        check(p!=null&&Math.abs(p.rpy[0]-90)<0.01,"+90 roll decode");
        System.out.println("PASS: independent Java decoder matches C++ packet fixture");
    }
}

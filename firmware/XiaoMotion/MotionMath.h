#pragma once
// Shared by the sketch and host tests. No Arduino or third-party dependencies.
#include <stdint.h>
#include <math.h>

namespace motion {
constexpr float PI_F = 3.14159265358979323846f;
constexpr float DEG = 180.0f / PI_F;
struct Vec { float x, y, z; };
struct Quat { float w, x, y, z; };

inline Quat multiply(Quat a, Quat b) {
  return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
          a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
          a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
          a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
inline Quat conjugate(Quat q) { return {q.w,-q.x,-q.y,-q.z}; }
inline Quat normalise(Quat q) {
  float n=q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
  if (!isfinite(n) || n<1e-12f) return {1,0,0,0};
  float s=1.0f/sqrtf(n);
  return {q.w*s,q.x*s,q.y*s,q.z*s};
}
inline Vec rotate(Quat q, Vec v) {
  Quat r=multiply(multiply(q,{0,v.x,v.y,v.z}),conjugate(q));
  return {r.x,r.y,r.z};
}
inline Vec euler(Quat q) {
  q=normalise(q);
  float s=2*(q.w*q.y-q.z*q.x);
  if (s>1) s=1;
  if (s< -1) s=-1;
  return {atan2f(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))*DEG,
          asinf(s)*DEG,
          atan2f(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))*DEG};
}
// Initialise roll/pitch from gravity, including an upside-down boot.
// Yaw is unobservable with this 6-axis IMU and starts at zero.
inline Quat tiltFromGravity(Vec a) {
  float n=a.x*a.x+a.y*a.y+a.z*a.z;
  if(!isfinite(n)||n<1e-12f) return {1,0,0,0};
  float roll=atan2f(a.y,a.z);
  float pitch=atan2f(-a.x,sqrtf(a.y*a.y+a.z*a.z));
  float cr=cosf(roll/2),sr=sinf(roll/2);
  float cp=cosf(pitch/2),sp=sinf(pitch/2);
  return normalise({cr*cp,sr*cp,cr*sp,-sr*sp});
}
// Signed permutation: output X,Y,Z selects +/- input 1,2,3.
// Keep a right-handed frame (determinant +1), e.g. {2,-1,3}.
inline bool validRemap(const int8_t m[3]) {
  int a=m[0]<0?-m[0]:m[0], b=m[1]<0?-m[1]:m[1], c=m[2]<0?-m[2]:m[2];
  if(a<1||a>3||b<1||b>3||c<1||c>3||a==b||a==c||b==c) return false;
  int inv=(a>b)+(a>c)+(b>c);
  int sign=(m[0]<0?-1:1)*(m[1]<0?-1:1)*(m[2]<0?-1:1);
  return sign*(inv%2?-1:1)==1;
}
inline Vec remap(Vec v,const int8_t m[3]) {
  float a[3]={v.x,v.y,v.z}, out[3];
  for(int i=0;i<3;++i) out[i]=m[i]<0?-a[-m[i]-1]:a[m[i]-1];
  return {out[0],out[1],out[2]};
}
class Madgwick {
public:
  Quat q={1,0,0,0};
  float beta=0.08f;
  // Gyro in rad/s, accelerometer in any consistent unit, dt in seconds.
  void update(Vec g,Vec a,float dt) {
    if(!isfinite(dt)||dt<=0||dt>0.1f||
       !isfinite(g.x)||!isfinite(g.y)||!isfinite(g.z)) return;
    q=normalise(q);
    float w=q.w,x=q.x,y=q.y,z=q.z;
    Quat d={-0.5f*(x*g.x+y*g.y+z*g.z),
             0.5f*(w*g.x+y*g.z-z*g.y),
             0.5f*(w*g.y-x*g.z+z*g.x),
             0.5f*(w*g.z+x*g.y-y*g.x)};
    float an=a.x*a.x+a.y*a.y+a.z*a.z;
    if(isfinite(an)&&an>1e-12f) {
      float r=1/sqrtf(an); a.x*=r; a.y*=r; a.z*=r;
      // Gravity objective and its analytic Jacobian transpose.
      float f1=2*(x*z-w*y)-a.x;
      float f2=2*(w*x+y*z)-a.y;
      float f3=1-2*(x*x+y*y)-a.z;
      Quat s={-2*y*f1+2*x*f2,
               2*z*f1+2*w*f2-4*x*f3,
              -2*w*f1+2*z*f2-4*y*f3,
               2*x*f1+2*y*f2};
      float sn=s.w*s.w+s.x*s.x+s.y*s.y+s.z*s.z;
      // At exactly level (or converged) the gradient is zero.
      if(isfinite(sn)&&sn>1e-12f) {
        float k=beta/sqrtf(sn);
        d.w-=k*s.w; d.x-=k*s.x; d.y-=k*s.y; d.z-=k*s.z;
      }
    }
    q=normalise({w+d.w*dt,x+d.x*dt,y+d.y*dt,z+d.z*dt});
  }
};
inline int16_t quantise(float value,float scale) {
  float v=value*scale;
  if(!isfinite(v)) return 0;
  if(v>=32767) return 32767;
  if(v<=-32768) return -32768;
  return (int16_t)lroundf(v);
}
inline void encode(uint8_t out[20],Quat q,Vec a,Vec g) {
  q=normalise(q);
  const int16_t v[10]={quantise(q.w,16384),quantise(q.x,16384),
    quantise(q.y,16384),quantise(q.z,16384),quantise(a.x,1000),
    quantise(a.y,1000),quantise(a.z,1000),quantise(g.x,16),
    quantise(g.y,16),quantise(g.z,16)};
  for(int i=0;i<10;++i) {
    uint16_t n=(uint16_t)v[i];
    out[2*i]=(uint8_t)n; out[2*i+1]=(uint8_t)(n>>8);
  }
}
class GyroBias {
public:
  Vec value={0,0,0};
  Vec sum={0,0,0};
  unsigned count=0;
  bool active=true;
  void start() { sum={0,0,0}; count=0; active=true; }
  // Require 416 consecutive stationary samples (~2s).
  // Motion restarts the window; previous bias is retained until success.
  bool sample(Vec a,Vec g) {
    if(!active) return false;
    float an=a.x*a.x+a.y*a.y+a.z*a.z;
    if(!isfinite(an)||fabsf(an-1)>0.15f||
       fabsf(g.x)>5||fabsf(g.y)>5||fabsf(g.z)>5) {
      sum={0,0,0}; count=0; return false;
    }
    sum.x+=g.x; sum.y+=g.y; sum.z+=g.z;
    if(++count<416) return false;
    value={sum.x/count,sum.y/count,sum.z/count}; active=false; return true;
  }
  Vec subtract(Vec g) const { return {g.x-value.x,g.y-value.y,g.z-value.z}; }
};
}

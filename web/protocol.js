// Shared browser/Node decoder. Packet order: qw,qx,qy,qz,ax,ay,az,gx,gy,gz.
export function decode(value) {
  if(!(value instanceof DataView)||value.byteLength!==20) return null;
  const q=Array.from({length:4},(_,i)=>value.getInt16(i*2,true)/16384);
  const norm=q.reduce((s,n)=>s+n*n,0);
  if(norm<0.98||norm>1.02) return null;
  q.forEach((n,i)=>q[i]=n/Math.sqrt(norm));
  const a=Array.from({length:3},(_,i)=>value.getInt16(8+i*2,true)/1000);
  const g=Array.from({length:3},(_,i)=>value.getInt16(14+i*2,true)/16);
  return {q,a,g,rpy:euler(q)};
}
export function euler([w,x,y,z]) {
  const degrees=180/Math.PI;
  return [Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))*degrees,
    Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x))))*degrees,
    Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))*degrees];
}
export function rotate(v,[w,x,y,z]) {
  const [a,b,c]=v,tx=2*(y*c-z*b),ty=2*(z*a-x*c),tz=2*(x*b-y*a);
  return [a+w*tx+y*tz-z*ty,b+w*ty+z*tx-x*tz,c+w*tz+x*ty-y*tx];
}

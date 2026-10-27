(()=>{/**
 * @license
 * Copyright 2010-2025 Three.js Authors
 * SPDX-License-Identifier: MIT
 */var Vi={LEFT:0,MIDDLE:1,RIGHT:2,ROTATE:0,DOLLY:1,PAN:2},Gi={ROTATE:0,PAN:1,DOLLY_PAN:2,DOLLY_ROTATE:3},td=0,hl=1,nd=2;var ul=1,fa=2,ni=3,mn=0,en=1,Kt=2,Mi=0,ns=1,dl=2,fl=3,pl=4,id=5,Di=100,sd=101,rd=102,od=103,ad=104,cd=200,ld=201,hd=202,ud=203,Ko=204,Zo=205,dd=206,fd=207,pd=208,md=209,gd=210,_d=211,xd=212,yd=213,vd=214,pa=0,ma=1,ga=2,is=3,_a=4,xa=5,ya=6,va=7,ml=0,bd=1,Md=2,Si=0,Sd=1,Ed=2,Td=3,ba=4,wd=5,Ad=6,Rd=7,$c="attached",Cd="detached",gl=300,us=301,ds=302,Ma=303,Sa=304,ro=306,jn=1e3,En=1001,Ui=1002,Ct=1003,ir=1004;var Ei=1005;var Ht=1006,Wi=1007;var gn=1008;var zn=1009,_l=1010,xl=1011,sr=1012,Ea=1013,Xi=1014,wn=1015,rr=1016,Ta=1017,wa=1018,or=1020,yl=35902,vl=35899,bl=1021,Ml=1022,tn=1023,Ws=1026,ar=1027,Aa=1028,Ra=1029,Sl=1030,Ca=1031;var Ia=1033,oo=33776,ao=33777,co=33778,lo=33779,Pa=35840,La=35841,Na=35842,Da=35843,Ua=36196,Oa=37492,Fa=37496,Ba=37808,ka=37809,za=37810,Ha=37811,Va=37812,Ga=37813,Wa=37814,Xa=37815,qa=37816,Ya=37817,ja=37818,Ka=37819,Za=37820,Ja=37821,$a=36492,Qa=36494,ec=36495,tc=36283,nc=36284,ic=36285,sc=36286;var pi=2300,mi=2301,jo=2302,Qc=2400,el=2401,tl=2402,Id=2500;var El=0,ho=1,cr=2,Pd=3200,Ld=3201;var Tl=0,Nd=1,Hn="",_t="srgb",Vt="srgb-linear",Pr="linear",ot="srgb";var ts=7680;var nl=519,Dd=512,Ud=513,Od=514,wl=515,Fd=516,Bd=517,kd=518,zd=519,Jo=35044;var Al="300 es",Un=2e3,Lr=2001;var Kn=class{addEventListener(e,t){this._listeners===void 0&&(this._listeners={});let n=this._listeners;n[e]===void 0&&(n[e]=[]),n[e].indexOf(t)===-1&&n[e].push(t)}hasEventListener(e,t){let n=this._listeners;return n===void 0?!1:n[e]!==void 0&&n[e].indexOf(t)!==-1}removeEventListener(e,t){let n=this._listeners;if(n===void 0)return;let i=n[e];if(i!==void 0){let r=i.indexOf(t);r!==-1&&i.splice(r,1)}}dispatchEvent(e){let t=this._listeners;if(t===void 0)return;let n=t[e.type];if(n!==void 0){e.target=this;let i=n.slice(0);for(let r=0,o=i.length;r<o;r++)i[r].call(this,e);e.target=null}}},Xt=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"],mu=1234567,Cr=Math.PI/180,ss=180/Math.PI;function On(){let s=Math.random()*4294967295|0,e=Math.random()*4294967295|0,t=Math.random()*4294967295|0,n=Math.random()*4294967295|0;return(Xt[s&255]+Xt[s>>8&255]+Xt[s>>16&255]+Xt[s>>24&255]+"-"+Xt[e&255]+Xt[e>>8&255]+"-"+Xt[e>>16&15|64]+Xt[e>>24&255]+"-"+Xt[t&63|128]+Xt[t>>8&255]+"-"+Xt[t>>16&255]+Xt[t>>24&255]+Xt[n&255]+Xt[n>>8&255]+Xt[n>>16&255]+Xt[n>>24&255]).toLowerCase()}function qe(s,e,t){return Math.max(e,Math.min(t,s))}function Rl(s,e){return(s%e+e)%e}function gp(s,e,t,n,i){return n+(s-e)*(i-n)/(t-e)}function _p(s,e,t){return s!==e?(t-s)/(e-s):0}function Ir(s,e,t){return(1-t)*s+t*e}function xp(s,e,t,n){return Ir(s,e,1-Math.exp(-t*n))}function yp(s,e=1){return e-Math.abs(Rl(s,e*2)-e)}function vp(s,e,t){return s<=e?0:s>=t?1:(s=(s-e)/(t-e),s*s*(3-2*s))}function bp(s,e,t){return s<=e?0:s>=t?1:(s=(s-e)/(t-e),s*s*s*(s*(s*6-15)+10))}function Mp(s,e){return s+Math.floor(Math.random()*(e-s+1))}function Sp(s,e){return s+Math.random()*(e-s)}function Ep(s){return s*(.5-Math.random())}function Tp(s){s!==void 0&&(mu=s);let e=mu+=1831565813;return e=Math.imul(e^e>>>15,e|1),e^=e+Math.imul(e^e>>>7,e|61),((e^e>>>14)>>>0)/4294967296}function wp(s){return s*Cr}function Ap(s){return s*ss}function Rp(s){return(s&s-1)===0&&s!==0}function Cp(s){return Math.pow(2,Math.ceil(Math.log(s)/Math.LN2))}function Ip(s){return Math.pow(2,Math.floor(Math.log(s)/Math.LN2))}function Pp(s,e,t,n,i){let r=Math.cos,o=Math.sin,a=r(t/2),c=o(t/2),l=r((e+n)/2),h=o((e+n)/2),u=r((e-n)/2),d=o((e-n)/2),f=r((n-e)/2),g=o((n-e)/2);switch(i){case"XYX":s.set(a*h,c*u,c*d,a*l);break;case"YZY":s.set(c*d,a*h,c*u,a*l);break;case"ZXZ":s.set(c*u,c*d,a*h,a*l);break;case"XZX":s.set(a*h,c*g,c*f,a*l);break;case"YXY":s.set(c*f,a*h,c*g,a*l);break;case"ZYZ":s.set(c*g,c*f,a*h,a*l);break;default:console.warn("THREE.MathUtils: .setQuaternionFromProperEuler() encountered an unknown order: "+i)}}function Dn(s,e){switch(e.constructor){case Float32Array:return s;case Uint32Array:return s/4294967295;case Uint16Array:return s/65535;case Uint8Array:return s/255;case Int32Array:return Math.max(s/2147483647,-1);case Int16Array:return Math.max(s/32767,-1);case Int8Array:return Math.max(s/127,-1);default:throw new Error("Invalid component type.")}}function rt(s,e){switch(e.constructor){case Float32Array:return s;case Uint32Array:return Math.round(s*4294967295);case Uint16Array:return Math.round(s*65535);case Uint8Array:return Math.round(s*255);case Int32Array:return Math.round(s*2147483647);case Int16Array:return Math.round(s*32767);case Int8Array:return Math.round(s*127);default:throw new Error("Invalid component type.")}}var Ti={DEG2RAD:Cr,RAD2DEG:ss,generateUUID:On,clamp:qe,euclideanModulo:Rl,mapLinear:gp,inverseLerp:_p,lerp:Ir,damp:xp,pingpong:yp,smoothstep:vp,smootherstep:bp,randInt:Mp,randFloat:Sp,randFloatSpread:Ep,seededRandom:Tp,degToRad:wp,radToDeg:Ap,isPowerOfTwo:Rp,ceilPowerOfTwo:Cp,floorPowerOfTwo:Ip,setQuaternionFromProperEuler:Pp,normalize:rt,denormalize:Dn},Ee=class s{constructor(e=0,t=0){s.prototype.isVector2=!0,this.x=e,this.y=t}get width(){return this.x}set width(e){this.x=e}get height(){return this.y}set height(e){this.y=e}set(e,t){return this.x=e,this.y=t,this}setScalar(e){return this.x=e,this.y=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y)}copy(e){return this.x=e.x,this.y=e.y,this}add(e){return this.x+=e.x,this.y+=e.y,this}addScalar(e){return this.x+=e,this.y+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this}subScalar(e){return this.x-=e,this.y-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this}multiply(e){return this.x*=e.x,this.y*=e.y,this}multiplyScalar(e){return this.x*=e,this.y*=e,this}divide(e){return this.x/=e.x,this.y/=e.y,this}divideScalar(e){return this.multiplyScalar(1/e)}applyMatrix3(e){let t=this.x,n=this.y,i=e.elements;return this.x=i[0]*t+i[3]*n+i[6],this.y=i[1]*t+i[4]*n+i[7],this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this}clamp(e,t){return this.x=qe(this.x,e.x,t.x),this.y=qe(this.y,e.y,t.y),this}clampScalar(e,t){return this.x=qe(this.x,e,t),this.y=qe(this.y,e,t),this}clampLength(e,t){let n=this.length();return this.divideScalar(n||1).multiplyScalar(qe(n,e,t))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(e){return this.x*e.x+this.y*e.y}cross(e){return this.x*e.y-this.y*e.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(e){let t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;let n=this.dot(e)/t;return Math.acos(qe(n,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){let t=this.x-e.x,n=this.y-e.y;return t*t+n*n}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this}lerpVectors(e,t,n){return this.x=e.x+(t.x-e.x)*n,this.y=e.y+(t.y-e.y)*n,this}equals(e){return e.x===this.x&&e.y===this.y}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this}rotateAround(e,t){let n=Math.cos(t),i=Math.sin(t),r=this.x-e.x,o=this.y-e.y;return this.x=r*n-o*i+e.x,this.y=r*i+o*n+e.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}},It=class{constructor(e=0,t=0,n=0,i=1){this.isQuaternion=!0,this._x=e,this._y=t,this._z=n,this._w=i}static slerpFlat(e,t,n,i,r,o,a){let c=n[i+0],l=n[i+1],h=n[i+2],u=n[i+3],d=r[o+0],f=r[o+1],g=r[o+2],_=r[o+3];if(a===0){e[t+0]=c,e[t+1]=l,e[t+2]=h,e[t+3]=u;return}if(a===1){e[t+0]=d,e[t+1]=f,e[t+2]=g,e[t+3]=_;return}if(u!==_||c!==d||l!==f||h!==g){let m=1-a,p=c*d+l*f+h*g+u*_,S=p>=0?1:-1,E=1-p*p;if(E>Number.EPSILON){let w=Math.sqrt(E),C=Math.atan2(w,p*S);m=Math.sin(m*C)/w,a=Math.sin(a*C)/w}let y=a*S;if(c=c*m+d*y,l=l*m+f*y,h=h*m+g*y,u=u*m+_*y,m===1-a){let w=1/Math.sqrt(c*c+l*l+h*h+u*u);c*=w,l*=w,h*=w,u*=w}}e[t]=c,e[t+1]=l,e[t+2]=h,e[t+3]=u}static multiplyQuaternionsFlat(e,t,n,i,r,o){let a=n[i],c=n[i+1],l=n[i+2],h=n[i+3],u=r[o],d=r[o+1],f=r[o+2],g=r[o+3];return e[t]=a*g+h*u+c*f-l*d,e[t+1]=c*g+h*d+l*u-a*f,e[t+2]=l*g+h*f+a*d-c*u,e[t+3]=h*g-a*u-c*d-l*f,e}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get w(){return this._w}set w(e){this._w=e,this._onChangeCallback()}set(e,t,n,i){return this._x=e,this._y=t,this._z=n,this._w=i,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(e){return this._x=e.x,this._y=e.y,this._z=e.z,this._w=e.w,this._onChangeCallback(),this}setFromEuler(e,t=!0){let n=e._x,i=e._y,r=e._z,o=e._order,a=Math.cos,c=Math.sin,l=a(n/2),h=a(i/2),u=a(r/2),d=c(n/2),f=c(i/2),g=c(r/2);switch(o){case"XYZ":this._x=d*h*u+l*f*g,this._y=l*f*u-d*h*g,this._z=l*h*g+d*f*u,this._w=l*h*u-d*f*g;break;case"YXZ":this._x=d*h*u+l*f*g,this._y=l*f*u-d*h*g,this._z=l*h*g-d*f*u,this._w=l*h*u+d*f*g;break;case"ZXY":this._x=d*h*u-l*f*g,this._y=l*f*u+d*h*g,this._z=l*h*g+d*f*u,this._w=l*h*u-d*f*g;break;case"ZYX":this._x=d*h*u-l*f*g,this._y=l*f*u+d*h*g,this._z=l*h*g-d*f*u,this._w=l*h*u+d*f*g;break;case"YZX":this._x=d*h*u+l*f*g,this._y=l*f*u+d*h*g,this._z=l*h*g-d*f*u,this._w=l*h*u-d*f*g;break;case"XZY":this._x=d*h*u-l*f*g,this._y=l*f*u-d*h*g,this._z=l*h*g+d*f*u,this._w=l*h*u+d*f*g;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+o)}return t===!0&&this._onChangeCallback(),this}setFromAxisAngle(e,t){let n=t/2,i=Math.sin(n);return this._x=e.x*i,this._y=e.y*i,this._z=e.z*i,this._w=Math.cos(n),this._onChangeCallback(),this}setFromRotationMatrix(e){let t=e.elements,n=t[0],i=t[4],r=t[8],o=t[1],a=t[5],c=t[9],l=t[2],h=t[6],u=t[10],d=n+a+u;if(d>0){let f=.5/Math.sqrt(d+1);this._w=.25/f,this._x=(h-c)*f,this._y=(r-l)*f,this._z=(o-i)*f}else if(n>a&&n>u){let f=2*Math.sqrt(1+n-a-u);this._w=(h-c)/f,this._x=.25*f,this._y=(i+o)/f,this._z=(r+l)/f}else if(a>u){let f=2*Math.sqrt(1+a-n-u);this._w=(r-l)/f,this._x=(i+o)/f,this._y=.25*f,this._z=(c+h)/f}else{let f=2*Math.sqrt(1+u-n-a);this._w=(o-i)/f,this._x=(r+l)/f,this._y=(c+h)/f,this._z=.25*f}return this._onChangeCallback(),this}setFromUnitVectors(e,t){let n=e.dot(t)+1;return n<1e-8?(n=0,Math.abs(e.x)>Math.abs(e.z)?(this._x=-e.y,this._y=e.x,this._z=0,this._w=n):(this._x=0,this._y=-e.z,this._z=e.y,this._w=n)):(this._x=e.y*t.z-e.z*t.y,this._y=e.z*t.x-e.x*t.z,this._z=e.x*t.y-e.y*t.x,this._w=n),this.normalize()}angleTo(e){return 2*Math.acos(Math.abs(qe(this.dot(e),-1,1)))}rotateTowards(e,t){let n=this.angleTo(e);if(n===0)return this;let i=Math.min(1,t/n);return this.slerp(e,i),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(e){return this._x*e._x+this._y*e._y+this._z*e._z+this._w*e._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let e=this.length();return e===0?(this._x=0,this._y=0,this._z=0,this._w=1):(e=1/e,this._x=this._x*e,this._y=this._y*e,this._z=this._z*e,this._w=this._w*e),this._onChangeCallback(),this}multiply(e){return this.multiplyQuaternions(this,e)}premultiply(e){return this.multiplyQuaternions(e,this)}multiplyQuaternions(e,t){let n=e._x,i=e._y,r=e._z,o=e._w,a=t._x,c=t._y,l=t._z,h=t._w;return this._x=n*h+o*a+i*l-r*c,this._y=i*h+o*c+r*a-n*l,this._z=r*h+o*l+n*c-i*a,this._w=o*h-n*a-i*c-r*l,this._onChangeCallback(),this}slerp(e,t){if(t===0)return this;if(t===1)return this.copy(e);let n=this._x,i=this._y,r=this._z,o=this._w,a=o*e._w+n*e._x+i*e._y+r*e._z;if(a<0?(this._w=-e._w,this._x=-e._x,this._y=-e._y,this._z=-e._z,a=-a):this.copy(e),a>=1)return this._w=o,this._x=n,this._y=i,this._z=r,this;let c=1-a*a;if(c<=Number.EPSILON){let f=1-t;return this._w=f*o+t*this._w,this._x=f*n+t*this._x,this._y=f*i+t*this._y,this._z=f*r+t*this._z,this.normalize(),this}let l=Math.sqrt(c),h=Math.atan2(l,a),u=Math.sin((1-t)*h)/l,d=Math.sin(t*h)/l;return this._w=o*u+this._w*d,this._x=n*u+this._x*d,this._y=i*u+this._y*d,this._z=r*u+this._z*d,this._onChangeCallback(),this}slerpQuaternions(e,t,n){return this.copy(e).slerp(t,n)}random(){let e=2*Math.PI*Math.random(),t=2*Math.PI*Math.random(),n=Math.random(),i=Math.sqrt(1-n),r=Math.sqrt(n);return this.set(i*Math.sin(e),i*Math.cos(e),r*Math.sin(t),r*Math.cos(t))}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._w===this._w}fromArray(e,t=0){return this._x=e[t],this._y=e[t+1],this._z=e[t+2],this._w=e[t+3],this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._w,e}fromBufferAttribute(e,t){return this._x=e.getX(t),this._y=e.getY(t),this._z=e.getZ(t),this._w=e.getW(t),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}},A=class s{constructor(e=0,t=0,n=0){s.prototype.isVector3=!0,this.x=e,this.y=t,this.z=n}set(e,t,n){return n===void 0&&(n=this.z),this.x=e,this.y=t,this.z=n,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this}multiplyVectors(e,t){return this.x=e.x*t.x,this.y=e.y*t.y,this.z=e.z*t.z,this}applyEuler(e){return this.applyQuaternion(gu.setFromEuler(e))}applyAxisAngle(e,t){return this.applyQuaternion(gu.setFromAxisAngle(e,t))}applyMatrix3(e){let t=this.x,n=this.y,i=this.z,r=e.elements;return this.x=r[0]*t+r[3]*n+r[6]*i,this.y=r[1]*t+r[4]*n+r[7]*i,this.z=r[2]*t+r[5]*n+r[8]*i,this}applyNormalMatrix(e){return this.applyMatrix3(e).normalize()}applyMatrix4(e){let t=this.x,n=this.y,i=this.z,r=e.elements,o=1/(r[3]*t+r[7]*n+r[11]*i+r[15]);return this.x=(r[0]*t+r[4]*n+r[8]*i+r[12])*o,this.y=(r[1]*t+r[5]*n+r[9]*i+r[13])*o,this.z=(r[2]*t+r[6]*n+r[10]*i+r[14])*o,this}applyQuaternion(e){let t=this.x,n=this.y,i=this.z,r=e.x,o=e.y,a=e.z,c=e.w,l=2*(o*i-a*n),h=2*(a*t-r*i),u=2*(r*n-o*t);return this.x=t+c*l+o*u-a*h,this.y=n+c*h+a*l-r*u,this.z=i+c*u+r*h-o*l,this}project(e){return this.applyMatrix4(e.matrixWorldInverse).applyMatrix4(e.projectionMatrix)}unproject(e){return this.applyMatrix4(e.projectionMatrixInverse).applyMatrix4(e.matrixWorld)}transformDirection(e){let t=this.x,n=this.y,i=this.z,r=e.elements;return this.x=r[0]*t+r[4]*n+r[8]*i,this.y=r[1]*t+r[5]*n+r[9]*i,this.z=r[2]*t+r[6]*n+r[10]*i,this.normalize()}divide(e){return this.x/=e.x,this.y/=e.y,this.z/=e.z,this}divideScalar(e){return this.multiplyScalar(1/e)}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this}clamp(e,t){return this.x=qe(this.x,e.x,t.x),this.y=qe(this.y,e.y,t.y),this.z=qe(this.z,e.z,t.z),this}clampScalar(e,t){return this.x=qe(this.x,e,t),this.y=qe(this.y,e,t),this.z=qe(this.z,e,t),this}clampLength(e,t){let n=this.length();return this.divideScalar(n||1).multiplyScalar(qe(n,e,t))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this}lerpVectors(e,t,n){return this.x=e.x+(t.x-e.x)*n,this.y=e.y+(t.y-e.y)*n,this.z=e.z+(t.z-e.z)*n,this}cross(e){return this.crossVectors(this,e)}crossVectors(e,t){let n=e.x,i=e.y,r=e.z,o=t.x,a=t.y,c=t.z;return this.x=i*c-r*a,this.y=r*o-n*c,this.z=n*a-i*o,this}projectOnVector(e){let t=e.lengthSq();if(t===0)return this.set(0,0,0);let n=e.dot(this)/t;return this.copy(e).multiplyScalar(n)}projectOnPlane(e){return Ec.copy(this).projectOnVector(e),this.sub(Ec)}reflect(e){return this.sub(Ec.copy(e).multiplyScalar(2*this.dot(e)))}angleTo(e){let t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;let n=this.dot(e)/t;return Math.acos(qe(n,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){let t=this.x-e.x,n=this.y-e.y,i=this.z-e.z;return t*t+n*n+i*i}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)+Math.abs(this.z-e.z)}setFromSpherical(e){return this.setFromSphericalCoords(e.radius,e.phi,e.theta)}setFromSphericalCoords(e,t,n){let i=Math.sin(t)*e;return this.x=i*Math.sin(n),this.y=Math.cos(t)*e,this.z=i*Math.cos(n),this}setFromCylindrical(e){return this.setFromCylindricalCoords(e.radius,e.theta,e.y)}setFromCylindricalCoords(e,t,n){return this.x=e*Math.sin(t),this.y=n,this.z=e*Math.cos(t),this}setFromMatrixPosition(e){let t=e.elements;return this.x=t[12],this.y=t[13],this.z=t[14],this}setFromMatrixScale(e){let t=this.setFromMatrixColumn(e,0).length(),n=this.setFromMatrixColumn(e,1).length(),i=this.setFromMatrixColumn(e,2).length();return this.x=t,this.y=n,this.z=i,this}setFromMatrixColumn(e,t){return this.fromArray(e.elements,t*4)}setFromMatrix3Column(e,t){return this.fromArray(e.elements,t*3)}setFromEuler(e){return this.x=e._x,this.y=e._y,this.z=e._z,this}setFromColor(e){return this.x=e.r,this.y=e.g,this.z=e.b,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){let e=Math.random()*Math.PI*2,t=Math.random()*2-1,n=Math.sqrt(1-t*t);return this.x=n*Math.cos(e),this.y=t,this.z=n*Math.sin(e),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}},Ec=new A,gu=new It,Ve=class s{constructor(e,t,n,i,r,o,a,c,l){s.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],e!==void 0&&this.set(e,t,n,i,r,o,a,c,l)}set(e,t,n,i,r,o,a,c,l){let h=this.elements;return h[0]=e,h[1]=i,h[2]=a,h[3]=t,h[4]=r,h[5]=c,h[6]=n,h[7]=o,h[8]=l,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(e){let t=this.elements,n=e.elements;return t[0]=n[0],t[1]=n[1],t[2]=n[2],t[3]=n[3],t[4]=n[4],t[5]=n[5],t[6]=n[6],t[7]=n[7],t[8]=n[8],this}extractBasis(e,t,n){return e.setFromMatrix3Column(this,0),t.setFromMatrix3Column(this,1),n.setFromMatrix3Column(this,2),this}setFromMatrix4(e){let t=e.elements;return this.set(t[0],t[4],t[8],t[1],t[5],t[9],t[2],t[6],t[10]),this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){let n=e.elements,i=t.elements,r=this.elements,o=n[0],a=n[3],c=n[6],l=n[1],h=n[4],u=n[7],d=n[2],f=n[5],g=n[8],_=i[0],m=i[3],p=i[6],S=i[1],E=i[4],y=i[7],w=i[2],C=i[5],R=i[8];return r[0]=o*_+a*S+c*w,r[3]=o*m+a*E+c*C,r[6]=o*p+a*y+c*R,r[1]=l*_+h*S+u*w,r[4]=l*m+h*E+u*C,r[7]=l*p+h*y+u*R,r[2]=d*_+f*S+g*w,r[5]=d*m+f*E+g*C,r[8]=d*p+f*y+g*R,this}multiplyScalar(e){let t=this.elements;return t[0]*=e,t[3]*=e,t[6]*=e,t[1]*=e,t[4]*=e,t[7]*=e,t[2]*=e,t[5]*=e,t[8]*=e,this}determinant(){let e=this.elements,t=e[0],n=e[1],i=e[2],r=e[3],o=e[4],a=e[5],c=e[6],l=e[7],h=e[8];return t*o*h-t*a*l-n*r*h+n*a*c+i*r*l-i*o*c}invert(){let e=this.elements,t=e[0],n=e[1],i=e[2],r=e[3],o=e[4],a=e[5],c=e[6],l=e[7],h=e[8],u=h*o-a*l,d=a*c-h*r,f=l*r-o*c,g=t*u+n*d+i*f;if(g===0)return this.set(0,0,0,0,0,0,0,0,0);let _=1/g;return e[0]=u*_,e[1]=(i*l-h*n)*_,e[2]=(a*n-i*o)*_,e[3]=d*_,e[4]=(h*t-i*c)*_,e[5]=(i*r-a*t)*_,e[6]=f*_,e[7]=(n*c-l*t)*_,e[8]=(o*t-n*r)*_,this}transpose(){let e,t=this.elements;return e=t[1],t[1]=t[3],t[3]=e,e=t[2],t[2]=t[6],t[6]=e,e=t[5],t[5]=t[7],t[7]=e,this}getNormalMatrix(e){return this.setFromMatrix4(e).invert().transpose()}transposeIntoArray(e){let t=this.elements;return e[0]=t[0],e[1]=t[3],e[2]=t[6],e[3]=t[1],e[4]=t[4],e[5]=t[7],e[6]=t[2],e[7]=t[5],e[8]=t[8],this}setUvTransform(e,t,n,i,r,o,a){let c=Math.cos(r),l=Math.sin(r);return this.set(n*c,n*l,-n*(c*o+l*a)+o+e,-i*l,i*c,-i*(-l*o+c*a)+a+t,0,0,1),this}scale(e,t){return this.premultiply(Tc.makeScale(e,t)),this}rotate(e){return this.premultiply(Tc.makeRotation(-e)),this}translate(e,t){return this.premultiply(Tc.makeTranslation(e,t)),this}makeTranslation(e,t){return e.isVector2?this.set(1,0,e.x,0,1,e.y,0,0,1):this.set(1,0,e,0,1,t,0,0,1),this}makeRotation(e){let t=Math.cos(e),n=Math.sin(e);return this.set(t,-n,0,n,t,0,0,0,1),this}makeScale(e,t){return this.set(e,0,0,0,t,0,0,0,1),this}equals(e){let t=this.elements,n=e.elements;for(let i=0;i<9;i++)if(t[i]!==n[i])return!1;return!0}fromArray(e,t=0){for(let n=0;n<9;n++)this.elements[n]=e[n+t];return this}toArray(e=[],t=0){let n=this.elements;return e[t]=n[0],e[t+1]=n[1],e[t+2]=n[2],e[t+3]=n[3],e[t+4]=n[4],e[t+5]=n[5],e[t+6]=n[6],e[t+7]=n[7],e[t+8]=n[8],e}clone(){return new this.constructor().fromArray(this.elements)}},Tc=new Ve;function Cl(s){for(let e=s.length-1;e>=0;--e)if(s[e]>=65535)return!0;return!1}function Xs(s){return document.createElementNS("http://www.w3.org/1999/xhtml",s)}function Hd(){let s=Xs("canvas");return s.style.display="block",s}var _u={};function qs(s){s in _u||(_u[s]=!0,console.warn(s))}function Vd(s,e,t){return new Promise(function(n,i){function r(){switch(s.clientWaitSync(e,s.SYNC_FLUSH_COMMANDS_BIT,0)){case s.WAIT_FAILED:i();break;case s.TIMEOUT_EXPIRED:setTimeout(r,t);break;default:n()}}setTimeout(r,t)})}var xu=new Ve().set(.4123908,.3575843,.1804808,.212639,.7151687,.0721923,.0193308,.1191948,.9505322),yu=new Ve().set(3.2409699,-1.5373832,-.4986108,-.9692436,1.8759675,.0415551,.0556301,-.203977,1.0569715);function Lp(){let s={enabled:!0,workingColorSpace:Vt,spaces:{},convert:function(i,r,o){return this.enabled===!1||r===o||!r||!o||(this.spaces[r].transfer===ot&&(i.r=fi(i.r),i.g=fi(i.g),i.b=fi(i.b)),this.spaces[r].primaries!==this.spaces[o].primaries&&(i.applyMatrix3(this.spaces[r].toXYZ),i.applyMatrix3(this.spaces[o].fromXYZ)),this.spaces[o].transfer===ot&&(i.r=Gs(i.r),i.g=Gs(i.g),i.b=Gs(i.b))),i},workingToColorSpace:function(i,r){return this.convert(i,this.workingColorSpace,r)},colorSpaceToWorking:function(i,r){return this.convert(i,r,this.workingColorSpace)},getPrimaries:function(i){return this.spaces[i].primaries},getTransfer:function(i){return i===Hn?Pr:this.spaces[i].transfer},getToneMappingMode:function(i){return this.spaces[i].outputColorSpaceConfig.toneMappingMode||"standard"},getLuminanceCoefficients:function(i,r=this.workingColorSpace){return i.fromArray(this.spaces[r].luminanceCoefficients)},define:function(i){Object.assign(this.spaces,i)},_getMatrix:function(i,r,o){return i.copy(this.spaces[r].toXYZ).multiply(this.spaces[o].fromXYZ)},_getDrawingBufferColorSpace:function(i){return this.spaces[i].outputColorSpaceConfig.drawingBufferColorSpace},_getUnpackColorSpace:function(i=this.workingColorSpace){return this.spaces[i].workingColorSpaceConfig.unpackColorSpace},fromWorkingColorSpace:function(i,r){return qs("THREE.ColorManagement: .fromWorkingColorSpace() has been renamed to .workingToColorSpace()."),s.workingToColorSpace(i,r)},toWorkingColorSpace:function(i,r){return qs("THREE.ColorManagement: .toWorkingColorSpace() has been renamed to .colorSpaceToWorking()."),s.colorSpaceToWorking(i,r)}},e=[.64,.33,.3,.6,.15,.06],t=[.2126,.7152,.0722],n=[.3127,.329];return s.define({[Vt]:{primaries:e,whitePoint:n,transfer:Pr,toXYZ:xu,fromXYZ:yu,luminanceCoefficients:t,workingColorSpaceConfig:{unpackColorSpace:_t},outputColorSpaceConfig:{drawingBufferColorSpace:_t}},[_t]:{primaries:e,whitePoint:n,transfer:ot,toXYZ:xu,fromXYZ:yu,luminanceCoefficients:t,outputColorSpaceConfig:{drawingBufferColorSpace:_t}}}),s}var $e=Lp();function fi(s){return s<.04045?s*.0773993808:Math.pow(s*.9478672986+.0521327014,2.4)}function Gs(s){return s<.0031308?s*12.92:1.055*Math.pow(s,.41666)-.055}var Ts,Ys=class{static getDataURL(e,t="image/png"){if(/^data:/i.test(e.src)||typeof HTMLCanvasElement>"u")return e.src;let n;if(e instanceof HTMLCanvasElement)n=e;else{Ts===void 0&&(Ts=Xs("canvas")),Ts.width=e.width,Ts.height=e.height;let i=Ts.getContext("2d");e instanceof ImageData?i.putImageData(e,0,0):i.drawImage(e,0,0,e.width,e.height),n=Ts}return n.toDataURL(t)}static sRGBToLinear(e){if(typeof HTMLImageElement<"u"&&e instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&e instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&e instanceof ImageBitmap){let t=Xs("canvas");t.width=e.width,t.height=e.height;let n=t.getContext("2d");n.drawImage(e,0,0,e.width,e.height);let i=n.getImageData(0,0,e.width,e.height),r=i.data;for(let o=0;o<r.length;o++)r[o]=fi(r[o]/255)*255;return n.putImageData(i,0,0),t}else if(e.data){let t=e.data.slice(0);for(let n=0;n<t.length;n++)t instanceof Uint8Array||t instanceof Uint8ClampedArray?t[n]=Math.floor(fi(t[n]/255)*255):t[n]=fi(t[n]);return{data:t,width:e.width,height:e.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),e}},Np=0,Oi=class{constructor(e=null){this.isSource=!0,Object.defineProperty(this,"id",{value:Np++}),this.uuid=On(),this.data=e,this.dataReady=!0,this.version=0}getSize(e){let t=this.data;return typeof HTMLVideoElement<"u"&&t instanceof HTMLVideoElement?e.set(t.videoWidth,t.videoHeight,0):t instanceof VideoFrame?e.set(t.displayHeight,t.displayWidth,0):t!==null?e.set(t.width,t.height,t.depth||0):e.set(0,0,0),e}set needsUpdate(e){e===!0&&this.version++}toJSON(e){let t=e===void 0||typeof e=="string";if(!t&&e.images[this.uuid]!==void 0)return e.images[this.uuid];let n={uuid:this.uuid,url:""},i=this.data;if(i!==null){let r;if(Array.isArray(i)){r=[];for(let o=0,a=i.length;o<a;o++)i[o].isDataTexture?r.push(wc(i[o].image)):r.push(wc(i[o]))}else r=wc(i);n.url=r}return t||(e.images[this.uuid]=n),n}};function wc(s){return typeof HTMLImageElement<"u"&&s instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&s instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&s instanceof ImageBitmap?Ys.getDataURL(s):s.data?{data:Array.from(s.data),width:s.width,height:s.height,type:s.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}var Dp=0,Ac=new A,At=class s extends Kn{constructor(e=s.DEFAULT_IMAGE,t=s.DEFAULT_MAPPING,n=En,i=En,r=Ht,o=gn,a=tn,c=zn,l=s.DEFAULT_ANISOTROPY,h=Hn){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:Dp++}),this.uuid=On(),this.name="",this.source=new Oi(e),this.mipmaps=[],this.mapping=t,this.channel=0,this.wrapS=n,this.wrapT=i,this.magFilter=r,this.minFilter=o,this.anisotropy=l,this.format=a,this.internalFormat=null,this.type=c,this.offset=new Ee(0,0),this.repeat=new Ee(1,1),this.center=new Ee(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new Ve,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,this.colorSpace=h,this.userData={},this.updateRanges=[],this.version=0,this.onUpdate=null,this.renderTarget=null,this.isRenderTargetTexture=!1,this.isArrayTexture=!!(e&&e.depth&&e.depth>1),this.pmremVersion=0}get width(){return this.source.getSize(Ac).x}get height(){return this.source.getSize(Ac).y}get depth(){return this.source.getSize(Ac).z}get image(){return this.source.data}set image(e=null){this.source.data=e}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}addUpdateRange(e,t){this.updateRanges.push({start:e,count:t})}clearUpdateRanges(){this.updateRanges.length=0}clone(){return new this.constructor().copy(this)}copy(e){return this.name=e.name,this.source=e.source,this.mipmaps=e.mipmaps.slice(0),this.mapping=e.mapping,this.channel=e.channel,this.wrapS=e.wrapS,this.wrapT=e.wrapT,this.magFilter=e.magFilter,this.minFilter=e.minFilter,this.anisotropy=e.anisotropy,this.format=e.format,this.internalFormat=e.internalFormat,this.type=e.type,this.offset.copy(e.offset),this.repeat.copy(e.repeat),this.center.copy(e.center),this.rotation=e.rotation,this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrix.copy(e.matrix),this.generateMipmaps=e.generateMipmaps,this.premultiplyAlpha=e.premultiplyAlpha,this.flipY=e.flipY,this.unpackAlignment=e.unpackAlignment,this.colorSpace=e.colorSpace,this.renderTarget=e.renderTarget,this.isRenderTargetTexture=e.isRenderTargetTexture,this.isArrayTexture=e.isArrayTexture,this.userData=JSON.parse(JSON.stringify(e.userData)),this.needsUpdate=!0,this}setValues(e){for(let t in e){let n=e[t];if(n===void 0){console.warn(`THREE.Texture.setValues(): parameter '${t}' has value of undefined.`);continue}let i=this[t];if(i===void 0){console.warn(`THREE.Texture.setValues(): property '${t}' does not exist.`);continue}i&&n&&i.isVector2&&n.isVector2||i&&n&&i.isVector3&&n.isVector3||i&&n&&i.isMatrix3&&n.isMatrix3?i.copy(n):this[t]=n}}toJSON(e){let t=e===void 0||typeof e=="string";if(!t&&e.textures[this.uuid]!==void 0)return e.textures[this.uuid];let n={metadata:{version:4.7,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(e).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(n.userData=this.userData),t||(e.textures[this.uuid]=n),n}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(e){if(this.mapping!==gl)return e;if(e.applyMatrix3(this.matrix),e.x<0||e.x>1)switch(this.wrapS){case jn:e.x=e.x-Math.floor(e.x);break;case En:e.x=e.x<0?0:1;break;case Ui:Math.abs(Math.floor(e.x)%2)===1?e.x=Math.ceil(e.x)-e.x:e.x=e.x-Math.floor(e.x);break}if(e.y<0||e.y>1)switch(this.wrapT){case jn:e.y=e.y-Math.floor(e.y);break;case En:e.y=e.y<0?0:1;break;case Ui:Math.abs(Math.floor(e.y)%2)===1?e.y=Math.ceil(e.y)-e.y:e.y=e.y-Math.floor(e.y);break}return this.flipY&&(e.y=1-e.y),e}set needsUpdate(e){e===!0&&(this.version++,this.source.needsUpdate=!0)}set needsPMREMUpdate(e){e===!0&&this.pmremVersion++}};At.DEFAULT_IMAGE=null;At.DEFAULT_MAPPING=gl;At.DEFAULT_ANISOTROPY=1;var nt=class s{constructor(e=0,t=0,n=0,i=1){s.prototype.isVector4=!0,this.x=e,this.y=t,this.z=n,this.w=i}get width(){return this.z}set width(e){this.z=e}get height(){return this.w}set height(e){this.w=e}set(e,t,n,i){return this.x=e,this.y=t,this.z=n,this.w=i,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this.w=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setW(e){return this.w=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;case 3:this.w=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this.w=e.w!==void 0?e.w:1,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this.w+=e.w,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this.w+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this.w=e.w+t.w,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this.w+=e.w*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this.w-=e.w,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this.w-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this.w=e.w-t.w,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this.w*=e.w,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this.w*=e,this}applyMatrix4(e){let t=this.x,n=this.y,i=this.z,r=this.w,o=e.elements;return this.x=o[0]*t+o[4]*n+o[8]*i+o[12]*r,this.y=o[1]*t+o[5]*n+o[9]*i+o[13]*r,this.z=o[2]*t+o[6]*n+o[10]*i+o[14]*r,this.w=o[3]*t+o[7]*n+o[11]*i+o[15]*r,this}divide(e){return this.x/=e.x,this.y/=e.y,this.z/=e.z,this.w/=e.w,this}divideScalar(e){return this.multiplyScalar(1/e)}setAxisAngleFromQuaternion(e){this.w=2*Math.acos(e.w);let t=Math.sqrt(1-e.w*e.w);return t<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=e.x/t,this.y=e.y/t,this.z=e.z/t),this}setAxisAngleFromRotationMatrix(e){let t,n,i,r,c=e.elements,l=c[0],h=c[4],u=c[8],d=c[1],f=c[5],g=c[9],_=c[2],m=c[6],p=c[10];if(Math.abs(h-d)<.01&&Math.abs(u-_)<.01&&Math.abs(g-m)<.01){if(Math.abs(h+d)<.1&&Math.abs(u+_)<.1&&Math.abs(g+m)<.1&&Math.abs(l+f+p-3)<.1)return this.set(1,0,0,0),this;t=Math.PI;let E=(l+1)/2,y=(f+1)/2,w=(p+1)/2,C=(h+d)/4,R=(u+_)/4,N=(g+m)/4;return E>y&&E>w?E<.01?(n=0,i=.707106781,r=.707106781):(n=Math.sqrt(E),i=C/n,r=R/n):y>w?y<.01?(n=.707106781,i=0,r=.707106781):(i=Math.sqrt(y),n=C/i,r=N/i):w<.01?(n=.707106781,i=.707106781,r=0):(r=Math.sqrt(w),n=R/r,i=N/r),this.set(n,i,r,t),this}let S=Math.sqrt((m-g)*(m-g)+(u-_)*(u-_)+(d-h)*(d-h));return Math.abs(S)<.001&&(S=1),this.x=(m-g)/S,this.y=(u-_)/S,this.z=(d-h)/S,this.w=Math.acos((l+f+p-1)/2),this}setFromMatrixPosition(e){let t=e.elements;return this.x=t[12],this.y=t[13],this.z=t[14],this.w=t[15],this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this.w=Math.min(this.w,e.w),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this.w=Math.max(this.w,e.w),this}clamp(e,t){return this.x=qe(this.x,e.x,t.x),this.y=qe(this.y,e.y,t.y),this.z=qe(this.z,e.z,t.z),this.w=qe(this.w,e.w,t.w),this}clampScalar(e,t){return this.x=qe(this.x,e,t),this.y=qe(this.y,e,t),this.z=qe(this.z,e,t),this.w=qe(this.w,e,t),this}clampLength(e,t){let n=this.length();return this.divideScalar(n||1).multiplyScalar(qe(n,e,t))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z+this.w*e.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this.w+=(e.w-this.w)*t,this}lerpVectors(e,t,n){return this.x=e.x+(t.x-e.x)*n,this.y=e.y+(t.y-e.y)*n,this.z=e.z+(t.z-e.z)*n,this.w=e.w+(t.w-e.w)*n,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z&&e.w===this.w}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this.w=e[t+3],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e[t+3]=this.w,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this.w=e.getW(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}},$o=class extends Kn{constructor(e=1,t=1,n={}){super(),n=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:Ht,depthBuffer:!0,stencilBuffer:!1,resolveDepthBuffer:!0,resolveStencilBuffer:!0,depthTexture:null,samples:0,count:1,depth:1,multiview:!1},n),this.isRenderTarget=!0,this.width=e,this.height=t,this.depth=n.depth,this.scissor=new nt(0,0,e,t),this.scissorTest=!1,this.viewport=new nt(0,0,e,t);let i={width:e,height:t,depth:n.depth},r=new At(i);this.textures=[];let o=n.count;for(let a=0;a<o;a++)this.textures[a]=r.clone(),this.textures[a].isRenderTargetTexture=!0,this.textures[a].renderTarget=this;this._setTextureOptions(n),this.depthBuffer=n.depthBuffer,this.stencilBuffer=n.stencilBuffer,this.resolveDepthBuffer=n.resolveDepthBuffer,this.resolveStencilBuffer=n.resolveStencilBuffer,this._depthTexture=null,this.depthTexture=n.depthTexture,this.samples=n.samples,this.multiview=n.multiview}_setTextureOptions(e={}){let t={minFilter:Ht,generateMipmaps:!1,flipY:!1,internalFormat:null};e.mapping!==void 0&&(t.mapping=e.mapping),e.wrapS!==void 0&&(t.wrapS=e.wrapS),e.wrapT!==void 0&&(t.wrapT=e.wrapT),e.wrapR!==void 0&&(t.wrapR=e.wrapR),e.magFilter!==void 0&&(t.magFilter=e.magFilter),e.minFilter!==void 0&&(t.minFilter=e.minFilter),e.format!==void 0&&(t.format=e.format),e.type!==void 0&&(t.type=e.type),e.anisotropy!==void 0&&(t.anisotropy=e.anisotropy),e.colorSpace!==void 0&&(t.colorSpace=e.colorSpace),e.flipY!==void 0&&(t.flipY=e.flipY),e.generateMipmaps!==void 0&&(t.generateMipmaps=e.generateMipmaps),e.internalFormat!==void 0&&(t.internalFormat=e.internalFormat);for(let n=0;n<this.textures.length;n++)this.textures[n].setValues(t)}get texture(){return this.textures[0]}set texture(e){this.textures[0]=e}set depthTexture(e){this._depthTexture!==null&&(this._depthTexture.renderTarget=null),e!==null&&(e.renderTarget=this),this._depthTexture=e}get depthTexture(){return this._depthTexture}setSize(e,t,n=1){if(this.width!==e||this.height!==t||this.depth!==n){this.width=e,this.height=t,this.depth=n;for(let i=0,r=this.textures.length;i<r;i++)this.textures[i].image.width=e,this.textures[i].image.height=t,this.textures[i].image.depth=n,this.textures[i].isArrayTexture=this.textures[i].image.depth>1;this.dispose()}this.viewport.set(0,0,e,t),this.scissor.set(0,0,e,t)}clone(){return new this.constructor().copy(this)}copy(e){this.width=e.width,this.height=e.height,this.depth=e.depth,this.scissor.copy(e.scissor),this.scissorTest=e.scissorTest,this.viewport.copy(e.viewport),this.textures.length=0;for(let t=0,n=e.textures.length;t<n;t++){this.textures[t]=e.textures[t].clone(),this.textures[t].isRenderTargetTexture=!0,this.textures[t].renderTarget=this;let i=Object.assign({},e.textures[t].image);this.textures[t].source=new Oi(i)}return this.depthBuffer=e.depthBuffer,this.stencilBuffer=e.stencilBuffer,this.resolveDepthBuffer=e.resolveDepthBuffer,this.resolveStencilBuffer=e.resolveStencilBuffer,e.depthTexture!==null&&(this.depthTexture=e.depthTexture.clone()),this.samples=e.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}},Zn=class extends $o{constructor(e=1,t=1,n={}){super(e,t,n),this.isWebGLRenderTarget=!0}},Nr=class extends At{constructor(e=null,t=1,n=1,i=1){super(null),this.isDataArrayTexture=!0,this.image={data:e,width:t,height:n,depth:i},this.magFilter=Ct,this.minFilter=Ct,this.wrapR=En,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1,this.layerUpdates=new Set}addLayerUpdate(e){this.layerUpdates.add(e)}clearLayerUpdates(){this.layerUpdates.clear()}};var Qo=class extends At{constructor(e=null,t=1,n=1,i=1){super(null),this.isData3DTexture=!0,this.image={data:e,width:t,height:n,depth:i},this.magFilter=Ct,this.minFilter=Ct,this.wrapR=En,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}};var Rt=class{constructor(e=new A(1/0,1/0,1/0),t=new A(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=e,this.max=t}set(e,t){return this.min.copy(e),this.max.copy(t),this}setFromArray(e){this.makeEmpty();for(let t=0,n=e.length;t<n;t+=3)this.expandByPoint(Pn.fromArray(e,t));return this}setFromBufferAttribute(e){this.makeEmpty();for(let t=0,n=e.count;t<n;t++)this.expandByPoint(Pn.fromBufferAttribute(e,t));return this}setFromPoints(e){this.makeEmpty();for(let t=0,n=e.length;t<n;t++)this.expandByPoint(e[t]);return this}setFromCenterAndSize(e,t){let n=Pn.copy(t).multiplyScalar(.5);return this.min.copy(e).sub(n),this.max.copy(e).add(n),this}setFromObject(e,t=!1){return this.makeEmpty(),this.expandByObject(e,t)}clone(){return new this.constructor().copy(this)}copy(e){return this.min.copy(e.min),this.max.copy(e.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(e){return this.isEmpty()?e.set(0,0,0):e.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(e){return this.isEmpty()?e.set(0,0,0):e.subVectors(this.max,this.min)}expandByPoint(e){return this.min.min(e),this.max.max(e),this}expandByVector(e){return this.min.sub(e),this.max.add(e),this}expandByScalar(e){return this.min.addScalar(-e),this.max.addScalar(e),this}expandByObject(e,t=!1){e.updateWorldMatrix(!1,!1);let n=e.geometry;if(n!==void 0){let r=n.getAttribute("position");if(t===!0&&r!==void 0&&e.isInstancedMesh!==!0)for(let o=0,a=r.count;o<a;o++)e.isMesh===!0?e.getVertexPosition(o,Pn):Pn.fromBufferAttribute(r,o),Pn.applyMatrix4(e.matrixWorld),this.expandByPoint(Pn);else e.boundingBox!==void 0?(e.boundingBox===null&&e.computeBoundingBox(),bo.copy(e.boundingBox)):(n.boundingBox===null&&n.computeBoundingBox(),bo.copy(n.boundingBox)),bo.applyMatrix4(e.matrixWorld),this.union(bo)}let i=e.children;for(let r=0,o=i.length;r<o;r++)this.expandByObject(i[r],t);return this}containsPoint(e){return e.x>=this.min.x&&e.x<=this.max.x&&e.y>=this.min.y&&e.y<=this.max.y&&e.z>=this.min.z&&e.z<=this.max.z}containsBox(e){return this.min.x<=e.min.x&&e.max.x<=this.max.x&&this.min.y<=e.min.y&&e.max.y<=this.max.y&&this.min.z<=e.min.z&&e.max.z<=this.max.z}getParameter(e,t){return t.set((e.x-this.min.x)/(this.max.x-this.min.x),(e.y-this.min.y)/(this.max.y-this.min.y),(e.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(e){return e.max.x>=this.min.x&&e.min.x<=this.max.x&&e.max.y>=this.min.y&&e.min.y<=this.max.y&&e.max.z>=this.min.z&&e.min.z<=this.max.z}intersectsSphere(e){return this.clampPoint(e.center,Pn),Pn.distanceToSquared(e.center)<=e.radius*e.radius}intersectsPlane(e){let t,n;return e.normal.x>0?(t=e.normal.x*this.min.x,n=e.normal.x*this.max.x):(t=e.normal.x*this.max.x,n=e.normal.x*this.min.x),e.normal.y>0?(t+=e.normal.y*this.min.y,n+=e.normal.y*this.max.y):(t+=e.normal.y*this.max.y,n+=e.normal.y*this.min.y),e.normal.z>0?(t+=e.normal.z*this.min.z,n+=e.normal.z*this.max.z):(t+=e.normal.z*this.max.z,n+=e.normal.z*this.min.z),t<=-e.constant&&n>=-e.constant}intersectsTriangle(e){if(this.isEmpty())return!1;this.getCenter(xr),Mo.subVectors(this.max,xr),ws.subVectors(e.a,xr),As.subVectors(e.b,xr),Rs.subVectors(e.c,xr),Ri.subVectors(As,ws),Ci.subVectors(Rs,As),Ji.subVectors(ws,Rs);let t=[0,-Ri.z,Ri.y,0,-Ci.z,Ci.y,0,-Ji.z,Ji.y,Ri.z,0,-Ri.x,Ci.z,0,-Ci.x,Ji.z,0,-Ji.x,-Ri.y,Ri.x,0,-Ci.y,Ci.x,0,-Ji.y,Ji.x,0];return!Rc(t,ws,As,Rs,Mo)||(t=[1,0,0,0,1,0,0,0,1],!Rc(t,ws,As,Rs,Mo))?!1:(So.crossVectors(Ri,Ci),t=[So.x,So.y,So.z],Rc(t,ws,As,Rs,Mo))}clampPoint(e,t){return t.copy(e).clamp(this.min,this.max)}distanceToPoint(e){return this.clampPoint(e,Pn).distanceTo(e)}getBoundingSphere(e){return this.isEmpty()?e.makeEmpty():(this.getCenter(e.center),e.radius=this.getSize(Pn).length()*.5),e}intersect(e){return this.min.max(e.min),this.max.min(e.max),this.isEmpty()&&this.makeEmpty(),this}union(e){return this.min.min(e.min),this.max.max(e.max),this}applyMatrix4(e){return this.isEmpty()?this:(ai[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(e),ai[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(e),ai[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(e),ai[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(e),ai[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(e),ai[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(e),ai[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(e),ai[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(e),this.setFromPoints(ai),this)}translate(e){return this.min.add(e),this.max.add(e),this}equals(e){return e.min.equals(this.min)&&e.max.equals(this.max)}toJSON(){return{min:this.min.toArray(),max:this.max.toArray()}}fromJSON(e){return this.min.fromArray(e.min),this.max.fromArray(e.max),this}},ai=[new A,new A,new A,new A,new A,new A,new A,new A],Pn=new A,bo=new Rt,ws=new A,As=new A,Rs=new A,Ri=new A,Ci=new A,Ji=new A,xr=new A,Mo=new A,So=new A,$i=new A;function Rc(s,e,t,n,i){for(let r=0,o=s.length-3;r<=o;r+=3){$i.fromArray(s,r);let a=i.x*Math.abs($i.x)+i.y*Math.abs($i.y)+i.z*Math.abs($i.z),c=e.dot($i),l=t.dot($i),h=n.dot($i);if(Math.max(-Math.max(c,l,h),Math.min(c,l,h))>a)return!1}return!0}var Up=new Rt,yr=new A,Cc=new A,sn=class{constructor(e=new A,t=-1){this.isSphere=!0,this.center=e,this.radius=t}set(e,t){return this.center.copy(e),this.radius=t,this}setFromPoints(e,t){let n=this.center;t!==void 0?n.copy(t):Up.setFromPoints(e).getCenter(n);let i=0;for(let r=0,o=e.length;r<o;r++)i=Math.max(i,n.distanceToSquared(e[r]));return this.radius=Math.sqrt(i),this}copy(e){return this.center.copy(e.center),this.radius=e.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(e){return e.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(e){return e.distanceTo(this.center)-this.radius}intersectsSphere(e){let t=this.radius+e.radius;return e.center.distanceToSquared(this.center)<=t*t}intersectsBox(e){return e.intersectsSphere(this)}intersectsPlane(e){return Math.abs(e.distanceToPoint(this.center))<=this.radius}clampPoint(e,t){let n=this.center.distanceToSquared(e);return t.copy(e),n>this.radius*this.radius&&(t.sub(this.center).normalize(),t.multiplyScalar(this.radius).add(this.center)),t}getBoundingBox(e){return this.isEmpty()?(e.makeEmpty(),e):(e.set(this.center,this.center),e.expandByScalar(this.radius),e)}applyMatrix4(e){return this.center.applyMatrix4(e),this.radius=this.radius*e.getMaxScaleOnAxis(),this}translate(e){return this.center.add(e),this}expandByPoint(e){if(this.isEmpty())return this.center.copy(e),this.radius=0,this;yr.subVectors(e,this.center);let t=yr.lengthSq();if(t>this.radius*this.radius){let n=Math.sqrt(t),i=(n-this.radius)*.5;this.center.addScaledVector(yr,i/n),this.radius+=i}return this}union(e){return e.isEmpty()?this:this.isEmpty()?(this.copy(e),this):(this.center.equals(e.center)===!0?this.radius=Math.max(this.radius,e.radius):(Cc.subVectors(e.center,this.center).setLength(e.radius),this.expandByPoint(yr.copy(e.center).add(Cc)),this.expandByPoint(yr.copy(e.center).sub(Cc))),this)}equals(e){return e.center.equals(this.center)&&e.radius===this.radius}clone(){return new this.constructor().copy(this)}toJSON(){return{radius:this.radius,center:this.center.toArray()}}fromJSON(e){return this.radius=e.radius,this.center.fromArray(e.center),this}},ci=new A,Ic=new A,Eo=new A,Ii=new A,Pc=new A,To=new A,Lc=new A,Jn=class{constructor(e=new A,t=new A(0,0,-1)){this.origin=e,this.direction=t}set(e,t){return this.origin.copy(e),this.direction.copy(t),this}copy(e){return this.origin.copy(e.origin),this.direction.copy(e.direction),this}at(e,t){return t.copy(this.origin).addScaledVector(this.direction,e)}lookAt(e){return this.direction.copy(e).sub(this.origin).normalize(),this}recast(e){return this.origin.copy(this.at(e,ci)),this}closestPointToPoint(e,t){t.subVectors(e,this.origin);let n=t.dot(this.direction);return n<0?t.copy(this.origin):t.copy(this.origin).addScaledVector(this.direction,n)}distanceToPoint(e){return Math.sqrt(this.distanceSqToPoint(e))}distanceSqToPoint(e){let t=ci.subVectors(e,this.origin).dot(this.direction);return t<0?this.origin.distanceToSquared(e):(ci.copy(this.origin).addScaledVector(this.direction,t),ci.distanceToSquared(e))}distanceSqToSegment(e,t,n,i){Ic.copy(e).add(t).multiplyScalar(.5),Eo.copy(t).sub(e).normalize(),Ii.copy(this.origin).sub(Ic);let r=e.distanceTo(t)*.5,o=-this.direction.dot(Eo),a=Ii.dot(this.direction),c=-Ii.dot(Eo),l=Ii.lengthSq(),h=Math.abs(1-o*o),u,d,f,g;if(h>0)if(u=o*c-a,d=o*a-c,g=r*h,u>=0)if(d>=-g)if(d<=g){let _=1/h;u*=_,d*=_,f=u*(u+o*d+2*a)+d*(o*u+d+2*c)+l}else d=r,u=Math.max(0,-(o*d+a)),f=-u*u+d*(d+2*c)+l;else d=-r,u=Math.max(0,-(o*d+a)),f=-u*u+d*(d+2*c)+l;else d<=-g?(u=Math.max(0,-(-o*r+a)),d=u>0?-r:Math.min(Math.max(-r,-c),r),f=-u*u+d*(d+2*c)+l):d<=g?(u=0,d=Math.min(Math.max(-r,-c),r),f=d*(d+2*c)+l):(u=Math.max(0,-(o*r+a)),d=u>0?r:Math.min(Math.max(-r,-c),r),f=-u*u+d*(d+2*c)+l);else d=o>0?-r:r,u=Math.max(0,-(o*d+a)),f=-u*u+d*(d+2*c)+l;return n&&n.copy(this.origin).addScaledVector(this.direction,u),i&&i.copy(Ic).addScaledVector(Eo,d),f}intersectSphere(e,t){ci.subVectors(e.center,this.origin);let n=ci.dot(this.direction),i=ci.dot(ci)-n*n,r=e.radius*e.radius;if(i>r)return null;let o=Math.sqrt(r-i),a=n-o,c=n+o;return c<0?null:a<0?this.at(c,t):this.at(a,t)}intersectsSphere(e){return e.radius<0?!1:this.distanceSqToPoint(e.center)<=e.radius*e.radius}distanceToPlane(e){let t=e.normal.dot(this.direction);if(t===0)return e.distanceToPoint(this.origin)===0?0:null;let n=-(this.origin.dot(e.normal)+e.constant)/t;return n>=0?n:null}intersectPlane(e,t){let n=this.distanceToPlane(e);return n===null?null:this.at(n,t)}intersectsPlane(e){let t=e.distanceToPoint(this.origin);return t===0||e.normal.dot(this.direction)*t<0}intersectBox(e,t){let n,i,r,o,a,c,l=1/this.direction.x,h=1/this.direction.y,u=1/this.direction.z,d=this.origin;return l>=0?(n=(e.min.x-d.x)*l,i=(e.max.x-d.x)*l):(n=(e.max.x-d.x)*l,i=(e.min.x-d.x)*l),h>=0?(r=(e.min.y-d.y)*h,o=(e.max.y-d.y)*h):(r=(e.max.y-d.y)*h,o=(e.min.y-d.y)*h),n>o||r>i||((r>n||isNaN(n))&&(n=r),(o<i||isNaN(i))&&(i=o),u>=0?(a=(e.min.z-d.z)*u,c=(e.max.z-d.z)*u):(a=(e.max.z-d.z)*u,c=(e.min.z-d.z)*u),n>c||a>i)||((a>n||n!==n)&&(n=a),(c<i||i!==i)&&(i=c),i<0)?null:this.at(n>=0?n:i,t)}intersectsBox(e){return this.intersectBox(e,ci)!==null}intersectTriangle(e,t,n,i,r){Pc.subVectors(t,e),To.subVectors(n,e),Lc.crossVectors(Pc,To);let o=this.direction.dot(Lc),a;if(o>0){if(i)return null;a=1}else if(o<0)a=-1,o=-o;else return null;Ii.subVectors(this.origin,e);let c=a*this.direction.dot(To.crossVectors(Ii,To));if(c<0)return null;let l=a*this.direction.dot(Pc.cross(Ii));if(l<0||c+l>o)return null;let h=-a*Ii.dot(Lc);return h<0?null:this.at(h/o,r)}applyMatrix4(e){return this.origin.applyMatrix4(e),this.direction.transformDirection(e),this}equals(e){return e.origin.equals(this.origin)&&e.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}},De=class s{constructor(e,t,n,i,r,o,a,c,l,h,u,d,f,g,_,m){s.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],e!==void 0&&this.set(e,t,n,i,r,o,a,c,l,h,u,d,f,g,_,m)}set(e,t,n,i,r,o,a,c,l,h,u,d,f,g,_,m){let p=this.elements;return p[0]=e,p[4]=t,p[8]=n,p[12]=i,p[1]=r,p[5]=o,p[9]=a,p[13]=c,p[2]=l,p[6]=h,p[10]=u,p[14]=d,p[3]=f,p[7]=g,p[11]=_,p[15]=m,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new s().fromArray(this.elements)}copy(e){let t=this.elements,n=e.elements;return t[0]=n[0],t[1]=n[1],t[2]=n[2],t[3]=n[3],t[4]=n[4],t[5]=n[5],t[6]=n[6],t[7]=n[7],t[8]=n[8],t[9]=n[9],t[10]=n[10],t[11]=n[11],t[12]=n[12],t[13]=n[13],t[14]=n[14],t[15]=n[15],this}copyPosition(e){let t=this.elements,n=e.elements;return t[12]=n[12],t[13]=n[13],t[14]=n[14],this}setFromMatrix3(e){let t=e.elements;return this.set(t[0],t[3],t[6],0,t[1],t[4],t[7],0,t[2],t[5],t[8],0,0,0,0,1),this}extractBasis(e,t,n){return e.setFromMatrixColumn(this,0),t.setFromMatrixColumn(this,1),n.setFromMatrixColumn(this,2),this}makeBasis(e,t,n){return this.set(e.x,t.x,n.x,0,e.y,t.y,n.y,0,e.z,t.z,n.z,0,0,0,0,1),this}extractRotation(e){let t=this.elements,n=e.elements,i=1/Cs.setFromMatrixColumn(e,0).length(),r=1/Cs.setFromMatrixColumn(e,1).length(),o=1/Cs.setFromMatrixColumn(e,2).length();return t[0]=n[0]*i,t[1]=n[1]*i,t[2]=n[2]*i,t[3]=0,t[4]=n[4]*r,t[5]=n[5]*r,t[6]=n[6]*r,t[7]=0,t[8]=n[8]*o,t[9]=n[9]*o,t[10]=n[10]*o,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromEuler(e){let t=this.elements,n=e.x,i=e.y,r=e.z,o=Math.cos(n),a=Math.sin(n),c=Math.cos(i),l=Math.sin(i),h=Math.cos(r),u=Math.sin(r);if(e.order==="XYZ"){let d=o*h,f=o*u,g=a*h,_=a*u;t[0]=c*h,t[4]=-c*u,t[8]=l,t[1]=f+g*l,t[5]=d-_*l,t[9]=-a*c,t[2]=_-d*l,t[6]=g+f*l,t[10]=o*c}else if(e.order==="YXZ"){let d=c*h,f=c*u,g=l*h,_=l*u;t[0]=d+_*a,t[4]=g*a-f,t[8]=o*l,t[1]=o*u,t[5]=o*h,t[9]=-a,t[2]=f*a-g,t[6]=_+d*a,t[10]=o*c}else if(e.order==="ZXY"){let d=c*h,f=c*u,g=l*h,_=l*u;t[0]=d-_*a,t[4]=-o*u,t[8]=g+f*a,t[1]=f+g*a,t[5]=o*h,t[9]=_-d*a,t[2]=-o*l,t[6]=a,t[10]=o*c}else if(e.order==="ZYX"){let d=o*h,f=o*u,g=a*h,_=a*u;t[0]=c*h,t[4]=g*l-f,t[8]=d*l+_,t[1]=c*u,t[5]=_*l+d,t[9]=f*l-g,t[2]=-l,t[6]=a*c,t[10]=o*c}else if(e.order==="YZX"){let d=o*c,f=o*l,g=a*c,_=a*l;t[0]=c*h,t[4]=_-d*u,t[8]=g*u+f,t[1]=u,t[5]=o*h,t[9]=-a*h,t[2]=-l*h,t[6]=f*u+g,t[10]=d-_*u}else if(e.order==="XZY"){let d=o*c,f=o*l,g=a*c,_=a*l;t[0]=c*h,t[4]=-u,t[8]=l*h,t[1]=d*u+_,t[5]=o*h,t[9]=f*u-g,t[2]=g*u-f,t[6]=a*h,t[10]=_*u+d}return t[3]=0,t[7]=0,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromQuaternion(e){return this.compose(Op,e,Fp)}lookAt(e,t,n){let i=this.elements;return dn.subVectors(e,t),dn.lengthSq()===0&&(dn.z=1),dn.normalize(),Pi.crossVectors(n,dn),Pi.lengthSq()===0&&(Math.abs(n.z)===1?dn.x+=1e-4:dn.z+=1e-4,dn.normalize(),Pi.crossVectors(n,dn)),Pi.normalize(),wo.crossVectors(dn,Pi),i[0]=Pi.x,i[4]=wo.x,i[8]=dn.x,i[1]=Pi.y,i[5]=wo.y,i[9]=dn.y,i[2]=Pi.z,i[6]=wo.z,i[10]=dn.z,this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){let n=e.elements,i=t.elements,r=this.elements,o=n[0],a=n[4],c=n[8],l=n[12],h=n[1],u=n[5],d=n[9],f=n[13],g=n[2],_=n[6],m=n[10],p=n[14],S=n[3],E=n[7],y=n[11],w=n[15],C=i[0],R=i[4],N=i[8],M=i[12],b=i[1],P=i[5],H=i[9],G=i[13],q=i[2],W=i[6],Y=i[10],B=i[14],D=i[3],Q=i[7],ae=i[11],ce=i[15];return r[0]=o*C+a*b+c*q+l*D,r[4]=o*R+a*P+c*W+l*Q,r[8]=o*N+a*H+c*Y+l*ae,r[12]=o*M+a*G+c*B+l*ce,r[1]=h*C+u*b+d*q+f*D,r[5]=h*R+u*P+d*W+f*Q,r[9]=h*N+u*H+d*Y+f*ae,r[13]=h*M+u*G+d*B+f*ce,r[2]=g*C+_*b+m*q+p*D,r[6]=g*R+_*P+m*W+p*Q,r[10]=g*N+_*H+m*Y+p*ae,r[14]=g*M+_*G+m*B+p*ce,r[3]=S*C+E*b+y*q+w*D,r[7]=S*R+E*P+y*W+w*Q,r[11]=S*N+E*H+y*Y+w*ae,r[15]=S*M+E*G+y*B+w*ce,this}multiplyScalar(e){let t=this.elements;return t[0]*=e,t[4]*=e,t[8]*=e,t[12]*=e,t[1]*=e,t[5]*=e,t[9]*=e,t[13]*=e,t[2]*=e,t[6]*=e,t[10]*=e,t[14]*=e,t[3]*=e,t[7]*=e,t[11]*=e,t[15]*=e,this}determinant(){let e=this.elements,t=e[0],n=e[4],i=e[8],r=e[12],o=e[1],a=e[5],c=e[9],l=e[13],h=e[2],u=e[6],d=e[10],f=e[14],g=e[3],_=e[7],m=e[11],p=e[15];return g*(+r*c*u-i*l*u-r*a*d+n*l*d+i*a*f-n*c*f)+_*(+t*c*f-t*l*d+r*o*d-i*o*f+i*l*h-r*c*h)+m*(+t*l*u-t*a*f-r*o*u+n*o*f+r*a*h-n*l*h)+p*(-i*a*h-t*c*u+t*a*d+i*o*u-n*o*d+n*c*h)}transpose(){let e=this.elements,t;return t=e[1],e[1]=e[4],e[4]=t,t=e[2],e[2]=e[8],e[8]=t,t=e[6],e[6]=e[9],e[9]=t,t=e[3],e[3]=e[12],e[12]=t,t=e[7],e[7]=e[13],e[13]=t,t=e[11],e[11]=e[14],e[14]=t,this}setPosition(e,t,n){let i=this.elements;return e.isVector3?(i[12]=e.x,i[13]=e.y,i[14]=e.z):(i[12]=e,i[13]=t,i[14]=n),this}invert(){let e=this.elements,t=e[0],n=e[1],i=e[2],r=e[3],o=e[4],a=e[5],c=e[6],l=e[7],h=e[8],u=e[9],d=e[10],f=e[11],g=e[12],_=e[13],m=e[14],p=e[15],S=u*m*l-_*d*l+_*c*f-a*m*f-u*c*p+a*d*p,E=g*d*l-h*m*l-g*c*f+o*m*f+h*c*p-o*d*p,y=h*_*l-g*u*l+g*a*f-o*_*f-h*a*p+o*u*p,w=g*u*c-h*_*c-g*a*d+o*_*d+h*a*m-o*u*m,C=t*S+n*E+i*y+r*w;if(C===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);let R=1/C;return e[0]=S*R,e[1]=(_*d*r-u*m*r-_*i*f+n*m*f+u*i*p-n*d*p)*R,e[2]=(a*m*r-_*c*r+_*i*l-n*m*l-a*i*p+n*c*p)*R,e[3]=(u*c*r-a*d*r-u*i*l+n*d*l+a*i*f-n*c*f)*R,e[4]=E*R,e[5]=(h*m*r-g*d*r+g*i*f-t*m*f-h*i*p+t*d*p)*R,e[6]=(g*c*r-o*m*r-g*i*l+t*m*l+o*i*p-t*c*p)*R,e[7]=(o*d*r-h*c*r+h*i*l-t*d*l-o*i*f+t*c*f)*R,e[8]=y*R,e[9]=(g*u*r-h*_*r-g*n*f+t*_*f+h*n*p-t*u*p)*R,e[10]=(o*_*r-g*a*r+g*n*l-t*_*l-o*n*p+t*a*p)*R,e[11]=(h*a*r-o*u*r-h*n*l+t*u*l+o*n*f-t*a*f)*R,e[12]=w*R,e[13]=(h*_*i-g*u*i+g*n*d-t*_*d-h*n*m+t*u*m)*R,e[14]=(g*a*i-o*_*i-g*n*c+t*_*c+o*n*m-t*a*m)*R,e[15]=(o*u*i-h*a*i+h*n*c-t*u*c-o*n*d+t*a*d)*R,this}scale(e){let t=this.elements,n=e.x,i=e.y,r=e.z;return t[0]*=n,t[4]*=i,t[8]*=r,t[1]*=n,t[5]*=i,t[9]*=r,t[2]*=n,t[6]*=i,t[10]*=r,t[3]*=n,t[7]*=i,t[11]*=r,this}getMaxScaleOnAxis(){let e=this.elements,t=e[0]*e[0]+e[1]*e[1]+e[2]*e[2],n=e[4]*e[4]+e[5]*e[5]+e[6]*e[6],i=e[8]*e[8]+e[9]*e[9]+e[10]*e[10];return Math.sqrt(Math.max(t,n,i))}makeTranslation(e,t,n){return e.isVector3?this.set(1,0,0,e.x,0,1,0,e.y,0,0,1,e.z,0,0,0,1):this.set(1,0,0,e,0,1,0,t,0,0,1,n,0,0,0,1),this}makeRotationX(e){let t=Math.cos(e),n=Math.sin(e);return this.set(1,0,0,0,0,t,-n,0,0,n,t,0,0,0,0,1),this}makeRotationY(e){let t=Math.cos(e),n=Math.sin(e);return this.set(t,0,n,0,0,1,0,0,-n,0,t,0,0,0,0,1),this}makeRotationZ(e){let t=Math.cos(e),n=Math.sin(e);return this.set(t,-n,0,0,n,t,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(e,t){let n=Math.cos(t),i=Math.sin(t),r=1-n,o=e.x,a=e.y,c=e.z,l=r*o,h=r*a;return this.set(l*o+n,l*a-i*c,l*c+i*a,0,l*a+i*c,h*a+n,h*c-i*o,0,l*c-i*a,h*c+i*o,r*c*c+n,0,0,0,0,1),this}makeScale(e,t,n){return this.set(e,0,0,0,0,t,0,0,0,0,n,0,0,0,0,1),this}makeShear(e,t,n,i,r,o){return this.set(1,n,r,0,e,1,o,0,t,i,1,0,0,0,0,1),this}compose(e,t,n){let i=this.elements,r=t._x,o=t._y,a=t._z,c=t._w,l=r+r,h=o+o,u=a+a,d=r*l,f=r*h,g=r*u,_=o*h,m=o*u,p=a*u,S=c*l,E=c*h,y=c*u,w=n.x,C=n.y,R=n.z;return i[0]=(1-(_+p))*w,i[1]=(f+y)*w,i[2]=(g-E)*w,i[3]=0,i[4]=(f-y)*C,i[5]=(1-(d+p))*C,i[6]=(m+S)*C,i[7]=0,i[8]=(g+E)*R,i[9]=(m-S)*R,i[10]=(1-(d+_))*R,i[11]=0,i[12]=e.x,i[13]=e.y,i[14]=e.z,i[15]=1,this}decompose(e,t,n){let i=this.elements,r=Cs.set(i[0],i[1],i[2]).length(),o=Cs.set(i[4],i[5],i[6]).length(),a=Cs.set(i[8],i[9],i[10]).length();this.determinant()<0&&(r=-r),e.x=i[12],e.y=i[13],e.z=i[14],Ln.copy(this);let l=1/r,h=1/o,u=1/a;return Ln.elements[0]*=l,Ln.elements[1]*=l,Ln.elements[2]*=l,Ln.elements[4]*=h,Ln.elements[5]*=h,Ln.elements[6]*=h,Ln.elements[8]*=u,Ln.elements[9]*=u,Ln.elements[10]*=u,t.setFromRotationMatrix(Ln),n.x=r,n.y=o,n.z=a,this}makePerspective(e,t,n,i,r,o,a=Un,c=!1){let l=this.elements,h=2*r/(t-e),u=2*r/(n-i),d=(t+e)/(t-e),f=(n+i)/(n-i),g,_;if(c)g=r/(o-r),_=o*r/(o-r);else if(a===Un)g=-(o+r)/(o-r),_=-2*o*r/(o-r);else if(a===Lr)g=-o/(o-r),_=-o*r/(o-r);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+a);return l[0]=h,l[4]=0,l[8]=d,l[12]=0,l[1]=0,l[5]=u,l[9]=f,l[13]=0,l[2]=0,l[6]=0,l[10]=g,l[14]=_,l[3]=0,l[7]=0,l[11]=-1,l[15]=0,this}makeOrthographic(e,t,n,i,r,o,a=Un,c=!1){let l=this.elements,h=2/(t-e),u=2/(n-i),d=-(t+e)/(t-e),f=-(n+i)/(n-i),g,_;if(c)g=1/(o-r),_=o/(o-r);else if(a===Un)g=-2/(o-r),_=-(o+r)/(o-r);else if(a===Lr)g=-1/(o-r),_=-r/(o-r);else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+a);return l[0]=h,l[4]=0,l[8]=0,l[12]=d,l[1]=0,l[5]=u,l[9]=0,l[13]=f,l[2]=0,l[6]=0,l[10]=g,l[14]=_,l[3]=0,l[7]=0,l[11]=0,l[15]=1,this}equals(e){let t=this.elements,n=e.elements;for(let i=0;i<16;i++)if(t[i]!==n[i])return!1;return!0}fromArray(e,t=0){for(let n=0;n<16;n++)this.elements[n]=e[n+t];return this}toArray(e=[],t=0){let n=this.elements;return e[t]=n[0],e[t+1]=n[1],e[t+2]=n[2],e[t+3]=n[3],e[t+4]=n[4],e[t+5]=n[5],e[t+6]=n[6],e[t+7]=n[7],e[t+8]=n[8],e[t+9]=n[9],e[t+10]=n[10],e[t+11]=n[11],e[t+12]=n[12],e[t+13]=n[13],e[t+14]=n[14],e[t+15]=n[15],e}},Cs=new A,Ln=new De,Op=new A(0,0,0),Fp=new A(1,1,1),Pi=new A,wo=new A,dn=new A,vu=new De,bu=new It,Fn=class s{constructor(e=0,t=0,n=0,i=s.DEFAULT_ORDER){this.isEuler=!0,this._x=e,this._y=t,this._z=n,this._order=i}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get order(){return this._order}set order(e){this._order=e,this._onChangeCallback()}set(e,t,n,i=this._order){return this._x=e,this._y=t,this._z=n,this._order=i,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(e){return this._x=e._x,this._y=e._y,this._z=e._z,this._order=e._order,this._onChangeCallback(),this}setFromRotationMatrix(e,t=this._order,n=!0){let i=e.elements,r=i[0],o=i[4],a=i[8],c=i[1],l=i[5],h=i[9],u=i[2],d=i[6],f=i[10];switch(t){case"XYZ":this._y=Math.asin(qe(a,-1,1)),Math.abs(a)<.9999999?(this._x=Math.atan2(-h,f),this._z=Math.atan2(-o,r)):(this._x=Math.atan2(d,l),this._z=0);break;case"YXZ":this._x=Math.asin(-qe(h,-1,1)),Math.abs(h)<.9999999?(this._y=Math.atan2(a,f),this._z=Math.atan2(c,l)):(this._y=Math.atan2(-u,r),this._z=0);break;case"ZXY":this._x=Math.asin(qe(d,-1,1)),Math.abs(d)<.9999999?(this._y=Math.atan2(-u,f),this._z=Math.atan2(-o,l)):(this._y=0,this._z=Math.atan2(c,r));break;case"ZYX":this._y=Math.asin(-qe(u,-1,1)),Math.abs(u)<.9999999?(this._x=Math.atan2(d,f),this._z=Math.atan2(c,r)):(this._x=0,this._z=Math.atan2(-o,l));break;case"YZX":this._z=Math.asin(qe(c,-1,1)),Math.abs(c)<.9999999?(this._x=Math.atan2(-h,l),this._y=Math.atan2(-u,r)):(this._x=0,this._y=Math.atan2(a,f));break;case"XZY":this._z=Math.asin(-qe(o,-1,1)),Math.abs(o)<.9999999?(this._x=Math.atan2(d,l),this._y=Math.atan2(a,r)):(this._x=Math.atan2(-h,f),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+t)}return this._order=t,n===!0&&this._onChangeCallback(),this}setFromQuaternion(e,t,n){return vu.makeRotationFromQuaternion(e),this.setFromRotationMatrix(vu,t,n)}setFromVector3(e,t=this._order){return this.set(e.x,e.y,e.z,t)}reorder(e){return bu.setFromEuler(this),this.setFromQuaternion(bu,e)}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._order===this._order}fromArray(e){return this._x=e[0],this._y=e[1],this._z=e[2],e[3]!==void 0&&(this._order=e[3]),this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._order,e}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}};Fn.DEFAULT_ORDER="XYZ";var js=class{constructor(){this.mask=1}set(e){this.mask=(1<<e|0)>>>0}enable(e){this.mask|=1<<e|0}enableAll(){this.mask=-1}toggle(e){this.mask^=1<<e|0}disable(e){this.mask&=~(1<<e|0)}disableAll(){this.mask=0}test(e){return(this.mask&e.mask)!==0}isEnabled(e){return(this.mask&(1<<e|0))!==0}},Bp=0,Mu=new A,Is=new It,li=new De,Ao=new A,vr=new A,kp=new A,zp=new It,Su=new A(1,0,0),Eu=new A(0,1,0),Tu=new A(0,0,1),wu={type:"added"},Hp={type:"removed"},Ps={type:"childadded",child:null},Nc={type:"childremoved",child:null},pt=class s extends Kn{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:Bp++}),this.uuid=On(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=s.DEFAULT_UP.clone();let e=new A,t=new Fn,n=new It,i=new A(1,1,1);function r(){n.setFromEuler(t,!1)}function o(){t.setFromQuaternion(n,void 0,!1)}t._onChange(r),n._onChange(o),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:e},rotation:{configurable:!0,enumerable:!0,value:t},quaternion:{configurable:!0,enumerable:!0,value:n},scale:{configurable:!0,enumerable:!0,value:i},modelViewMatrix:{value:new De},normalMatrix:{value:new Ve}}),this.matrix=new De,this.matrixWorld=new De,this.matrixAutoUpdate=s.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=s.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new js,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.customDepthMaterial=void 0,this.customDistanceMaterial=void 0,this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(e){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(e),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(e){return this.quaternion.premultiply(e),this}setRotationFromAxisAngle(e,t){this.quaternion.setFromAxisAngle(e,t)}setRotationFromEuler(e){this.quaternion.setFromEuler(e,!0)}setRotationFromMatrix(e){this.quaternion.setFromRotationMatrix(e)}setRotationFromQuaternion(e){this.quaternion.copy(e)}rotateOnAxis(e,t){return Is.setFromAxisAngle(e,t),this.quaternion.multiply(Is),this}rotateOnWorldAxis(e,t){return Is.setFromAxisAngle(e,t),this.quaternion.premultiply(Is),this}rotateX(e){return this.rotateOnAxis(Su,e)}rotateY(e){return this.rotateOnAxis(Eu,e)}rotateZ(e){return this.rotateOnAxis(Tu,e)}translateOnAxis(e,t){return Mu.copy(e).applyQuaternion(this.quaternion),this.position.add(Mu.multiplyScalar(t)),this}translateX(e){return this.translateOnAxis(Su,e)}translateY(e){return this.translateOnAxis(Eu,e)}translateZ(e){return this.translateOnAxis(Tu,e)}localToWorld(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(this.matrixWorld)}worldToLocal(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(li.copy(this.matrixWorld).invert())}lookAt(e,t,n){e.isVector3?Ao.copy(e):Ao.set(e,t,n);let i=this.parent;this.updateWorldMatrix(!0,!1),vr.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?li.lookAt(vr,Ao,this.up):li.lookAt(Ao,vr,this.up),this.quaternion.setFromRotationMatrix(li),i&&(li.extractRotation(i.matrixWorld),Is.setFromRotationMatrix(li),this.quaternion.premultiply(Is.invert()))}add(e){if(arguments.length>1){for(let t=0;t<arguments.length;t++)this.add(arguments[t]);return this}return e===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",e),this):(e&&e.isObject3D?(e.removeFromParent(),e.parent=this,this.children.push(e),e.dispatchEvent(wu),Ps.child=e,this.dispatchEvent(Ps),Ps.child=null):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",e),this)}remove(e){if(arguments.length>1){for(let n=0;n<arguments.length;n++)this.remove(arguments[n]);return this}let t=this.children.indexOf(e);return t!==-1&&(e.parent=null,this.children.splice(t,1),e.dispatchEvent(Hp),Nc.child=e,this.dispatchEvent(Nc),Nc.child=null),this}removeFromParent(){let e=this.parent;return e!==null&&e.remove(this),this}clear(){return this.remove(...this.children)}attach(e){return this.updateWorldMatrix(!0,!1),li.copy(this.matrixWorld).invert(),e.parent!==null&&(e.parent.updateWorldMatrix(!0,!1),li.multiply(e.parent.matrixWorld)),e.applyMatrix4(li),e.removeFromParent(),e.parent=this,this.children.push(e),e.updateWorldMatrix(!1,!0),e.dispatchEvent(wu),Ps.child=e,this.dispatchEvent(Ps),Ps.child=null,this}getObjectById(e){return this.getObjectByProperty("id",e)}getObjectByName(e){return this.getObjectByProperty("name",e)}getObjectByProperty(e,t){if(this[e]===t)return this;for(let n=0,i=this.children.length;n<i;n++){let o=this.children[n].getObjectByProperty(e,t);if(o!==void 0)return o}}getObjectsByProperty(e,t,n=[]){this[e]===t&&n.push(this);let i=this.children;for(let r=0,o=i.length;r<o;r++)i[r].getObjectsByProperty(e,t,n);return n}getWorldPosition(e){return this.updateWorldMatrix(!0,!1),e.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(vr,e,kp),e}getWorldScale(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(vr,zp,e),e}getWorldDirection(e){this.updateWorldMatrix(!0,!1);let t=this.matrixWorld.elements;return e.set(t[8],t[9],t[10]).normalize()}raycast(){}traverse(e){e(this);let t=this.children;for(let n=0,i=t.length;n<i;n++)t[n].traverse(e)}traverseVisible(e){if(this.visible===!1)return;e(this);let t=this.children;for(let n=0,i=t.length;n<i;n++)t[n].traverseVisible(e)}traverseAncestors(e){let t=this.parent;t!==null&&(e(t),t.traverseAncestors(e))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(e){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||e)&&(this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),this.matrixWorldNeedsUpdate=!1,e=!0);let t=this.children;for(let n=0,i=t.length;n<i;n++)t[n].updateMatrixWorld(e)}updateWorldMatrix(e,t){let n=this.parent;if(e===!0&&n!==null&&n.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),t===!0){let i=this.children;for(let r=0,o=i.length;r<o;r++)i[r].updateWorldMatrix(!1,!0)}}toJSON(e){let t=e===void 0||typeof e=="string",n={};t&&(e={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},n.metadata={version:4.7,type:"Object",generator:"Object3D.toJSON"});let i={};i.uuid=this.uuid,i.type=this.type,this.name!==""&&(i.name=this.name),this.castShadow===!0&&(i.castShadow=!0),this.receiveShadow===!0&&(i.receiveShadow=!0),this.visible===!1&&(i.visible=!1),this.frustumCulled===!1&&(i.frustumCulled=!1),this.renderOrder!==0&&(i.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(i.userData=this.userData),i.layers=this.layers.mask,i.matrix=this.matrix.toArray(),i.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(i.matrixAutoUpdate=!1),this.isInstancedMesh&&(i.type="InstancedMesh",i.count=this.count,i.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(i.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(i.type="BatchedMesh",i.perObjectFrustumCulled=this.perObjectFrustumCulled,i.sortObjects=this.sortObjects,i.drawRanges=this._drawRanges,i.reservedRanges=this._reservedRanges,i.geometryInfo=this._geometryInfo.map(a=>({...a,boundingBox:a.boundingBox?a.boundingBox.toJSON():void 0,boundingSphere:a.boundingSphere?a.boundingSphere.toJSON():void 0})),i.instanceInfo=this._instanceInfo.map(a=>({...a})),i.availableInstanceIds=this._availableInstanceIds.slice(),i.availableGeometryIds=this._availableGeometryIds.slice(),i.nextIndexStart=this._nextIndexStart,i.nextVertexStart=this._nextVertexStart,i.geometryCount=this._geometryCount,i.maxInstanceCount=this._maxInstanceCount,i.maxVertexCount=this._maxVertexCount,i.maxIndexCount=this._maxIndexCount,i.geometryInitialized=this._geometryInitialized,i.matricesTexture=this._matricesTexture.toJSON(e),i.indirectTexture=this._indirectTexture.toJSON(e),this._colorsTexture!==null&&(i.colorsTexture=this._colorsTexture.toJSON(e)),this.boundingSphere!==null&&(i.boundingSphere=this.boundingSphere.toJSON()),this.boundingBox!==null&&(i.boundingBox=this.boundingBox.toJSON()));function r(a,c){return a[c.uuid]===void 0&&(a[c.uuid]=c.toJSON(e)),c.uuid}if(this.isScene)this.background&&(this.background.isColor?i.background=this.background.toJSON():this.background.isTexture&&(i.background=this.background.toJSON(e).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(i.environment=this.environment.toJSON(e).uuid);else if(this.isMesh||this.isLine||this.isPoints){i.geometry=r(e.geometries,this.geometry);let a=this.geometry.parameters;if(a!==void 0&&a.shapes!==void 0){let c=a.shapes;if(Array.isArray(c))for(let l=0,h=c.length;l<h;l++){let u=c[l];r(e.shapes,u)}else r(e.shapes,c)}}if(this.isSkinnedMesh&&(i.bindMode=this.bindMode,i.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(r(e.skeletons,this.skeleton),i.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){let a=[];for(let c=0,l=this.material.length;c<l;c++)a.push(r(e.materials,this.material[c]));i.material=a}else i.material=r(e.materials,this.material);if(this.children.length>0){i.children=[];for(let a=0;a<this.children.length;a++)i.children.push(this.children[a].toJSON(e).object)}if(this.animations.length>0){i.animations=[];for(let a=0;a<this.animations.length;a++){let c=this.animations[a];i.animations.push(r(e.animations,c))}}if(t){let a=o(e.geometries),c=o(e.materials),l=o(e.textures),h=o(e.images),u=o(e.shapes),d=o(e.skeletons),f=o(e.animations),g=o(e.nodes);a.length>0&&(n.geometries=a),c.length>0&&(n.materials=c),l.length>0&&(n.textures=l),h.length>0&&(n.images=h),u.length>0&&(n.shapes=u),d.length>0&&(n.skeletons=d),f.length>0&&(n.animations=f),g.length>0&&(n.nodes=g)}return n.object=i,n;function o(a){let c=[];for(let l in a){let h=a[l];delete h.metadata,c.push(h)}return c}}clone(e){return new this.constructor().copy(this,e)}copy(e,t=!0){if(this.name=e.name,this.up.copy(e.up),this.position.copy(e.position),this.rotation.order=e.rotation.order,this.quaternion.copy(e.quaternion),this.scale.copy(e.scale),this.matrix.copy(e.matrix),this.matrixWorld.copy(e.matrixWorld),this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrixWorldAutoUpdate=e.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=e.matrixWorldNeedsUpdate,this.layers.mask=e.layers.mask,this.visible=e.visible,this.castShadow=e.castShadow,this.receiveShadow=e.receiveShadow,this.frustumCulled=e.frustumCulled,this.renderOrder=e.renderOrder,this.animations=e.animations.slice(),this.userData=JSON.parse(JSON.stringify(e.userData)),t===!0)for(let n=0;n<e.children.length;n++){let i=e.children[n];this.add(i.clone())}return this}};pt.DEFAULT_UP=new A(0,1,0);pt.DEFAULT_MATRIX_AUTO_UPDATE=!0;pt.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;var Nn=new A,hi=new A,Dc=new A,ui=new A,Ls=new A,Ns=new A,Au=new A,Uc=new A,Oc=new A,Fc=new A,Bc=new nt,kc=new nt,zc=new nt,pn=class s{constructor(e=new A,t=new A,n=new A){this.a=e,this.b=t,this.c=n}static getNormal(e,t,n,i){i.subVectors(n,t),Nn.subVectors(e,t),i.cross(Nn);let r=i.lengthSq();return r>0?i.multiplyScalar(1/Math.sqrt(r)):i.set(0,0,0)}static getBarycoord(e,t,n,i,r){Nn.subVectors(i,t),hi.subVectors(n,t),Dc.subVectors(e,t);let o=Nn.dot(Nn),a=Nn.dot(hi),c=Nn.dot(Dc),l=hi.dot(hi),h=hi.dot(Dc),u=o*l-a*a;if(u===0)return r.set(0,0,0),null;let d=1/u,f=(l*c-a*h)*d,g=(o*h-a*c)*d;return r.set(1-f-g,g,f)}static containsPoint(e,t,n,i){return this.getBarycoord(e,t,n,i,ui)===null?!1:ui.x>=0&&ui.y>=0&&ui.x+ui.y<=1}static getInterpolation(e,t,n,i,r,o,a,c){return this.getBarycoord(e,t,n,i,ui)===null?(c.x=0,c.y=0,"z"in c&&(c.z=0),"w"in c&&(c.w=0),null):(c.setScalar(0),c.addScaledVector(r,ui.x),c.addScaledVector(o,ui.y),c.addScaledVector(a,ui.z),c)}static getInterpolatedAttribute(e,t,n,i,r,o){return Bc.setScalar(0),kc.setScalar(0),zc.setScalar(0),Bc.fromBufferAttribute(e,t),kc.fromBufferAttribute(e,n),zc.fromBufferAttribute(e,i),o.setScalar(0),o.addScaledVector(Bc,r.x),o.addScaledVector(kc,r.y),o.addScaledVector(zc,r.z),o}static isFrontFacing(e,t,n,i){return Nn.subVectors(n,t),hi.subVectors(e,t),Nn.cross(hi).dot(i)<0}set(e,t,n){return this.a.copy(e),this.b.copy(t),this.c.copy(n),this}setFromPointsAndIndices(e,t,n,i){return this.a.copy(e[t]),this.b.copy(e[n]),this.c.copy(e[i]),this}setFromAttributeAndIndices(e,t,n,i){return this.a.fromBufferAttribute(e,t),this.b.fromBufferAttribute(e,n),this.c.fromBufferAttribute(e,i),this}clone(){return new this.constructor().copy(this)}copy(e){return this.a.copy(e.a),this.b.copy(e.b),this.c.copy(e.c),this}getArea(){return Nn.subVectors(this.c,this.b),hi.subVectors(this.a,this.b),Nn.cross(hi).length()*.5}getMidpoint(e){return e.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(e){return s.getNormal(this.a,this.b,this.c,e)}getPlane(e){return e.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(e,t){return s.getBarycoord(e,this.a,this.b,this.c,t)}getInterpolation(e,t,n,i,r){return s.getInterpolation(e,this.a,this.b,this.c,t,n,i,r)}containsPoint(e){return s.containsPoint(e,this.a,this.b,this.c)}isFrontFacing(e){return s.isFrontFacing(this.a,this.b,this.c,e)}intersectsBox(e){return e.intersectsTriangle(this)}closestPointToPoint(e,t){let n=this.a,i=this.b,r=this.c,o,a;Ls.subVectors(i,n),Ns.subVectors(r,n),Uc.subVectors(e,n);let c=Ls.dot(Uc),l=Ns.dot(Uc);if(c<=0&&l<=0)return t.copy(n);Oc.subVectors(e,i);let h=Ls.dot(Oc),u=Ns.dot(Oc);if(h>=0&&u<=h)return t.copy(i);let d=c*u-h*l;if(d<=0&&c>=0&&h<=0)return o=c/(c-h),t.copy(n).addScaledVector(Ls,o);Fc.subVectors(e,r);let f=Ls.dot(Fc),g=Ns.dot(Fc);if(g>=0&&f<=g)return t.copy(r);let _=f*l-c*g;if(_<=0&&l>=0&&g<=0)return a=l/(l-g),t.copy(n).addScaledVector(Ns,a);let m=h*g-f*u;if(m<=0&&u-h>=0&&f-g>=0)return Au.subVectors(r,i),a=(u-h)/(u-h+(f-g)),t.copy(i).addScaledVector(Au,a);let p=1/(m+_+d);return o=_*p,a=d*p,t.copy(n).addScaledVector(Ls,o).addScaledVector(Ns,a)}equals(e){return e.a.equals(this.a)&&e.b.equals(this.b)&&e.c.equals(this.c)}},Gd={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},Li={h:0,s:0,l:0},Ro={h:0,s:0,l:0};function Hc(s,e,t){return t<0&&(t+=1),t>1&&(t-=1),t<1/6?s+(e-s)*6*t:t<1/2?e:t<2/3?s+(e-s)*6*(2/3-t):s}var Re=class{constructor(e,t,n){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(e,t,n)}set(e,t,n){if(t===void 0&&n===void 0){let i=e;i&&i.isColor?this.copy(i):typeof i=="number"?this.setHex(i):typeof i=="string"&&this.setStyle(i)}else this.setRGB(e,t,n);return this}setScalar(e){return this.r=e,this.g=e,this.b=e,this}setHex(e,t=_t){return e=Math.floor(e),this.r=(e>>16&255)/255,this.g=(e>>8&255)/255,this.b=(e&255)/255,$e.colorSpaceToWorking(this,t),this}setRGB(e,t,n,i=$e.workingColorSpace){return this.r=e,this.g=t,this.b=n,$e.colorSpaceToWorking(this,i),this}setHSL(e,t,n,i=$e.workingColorSpace){if(e=Rl(e,1),t=qe(t,0,1),n=qe(n,0,1),t===0)this.r=this.g=this.b=n;else{let r=n<=.5?n*(1+t):n+t-n*t,o=2*n-r;this.r=Hc(o,r,e+1/3),this.g=Hc(o,r,e),this.b=Hc(o,r,e-1/3)}return $e.colorSpaceToWorking(this,i),this}setStyle(e,t=_t){function n(r){r!==void 0&&parseFloat(r)<1&&console.warn("THREE.Color: Alpha component of "+e+" will be ignored.")}let i;if(i=/^(\w+)\(([^\)]*)\)/.exec(e)){let r,o=i[1],a=i[2];switch(o){case"rgb":case"rgba":if(r=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return n(r[4]),this.setRGB(Math.min(255,parseInt(r[1],10))/255,Math.min(255,parseInt(r[2],10))/255,Math.min(255,parseInt(r[3],10))/255,t);if(r=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return n(r[4]),this.setRGB(Math.min(100,parseInt(r[1],10))/100,Math.min(100,parseInt(r[2],10))/100,Math.min(100,parseInt(r[3],10))/100,t);break;case"hsl":case"hsla":if(r=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return n(r[4]),this.setHSL(parseFloat(r[1])/360,parseFloat(r[2])/100,parseFloat(r[3])/100,t);break;default:console.warn("THREE.Color: Unknown color model "+e)}}else if(i=/^\#([A-Fa-f\d]+)$/.exec(e)){let r=i[1],o=r.length;if(o===3)return this.setRGB(parseInt(r.charAt(0),16)/15,parseInt(r.charAt(1),16)/15,parseInt(r.charAt(2),16)/15,t);if(o===6)return this.setHex(parseInt(r,16),t);console.warn("THREE.Color: Invalid hex color "+e)}else if(e&&e.length>0)return this.setColorName(e,t);return this}setColorName(e,t=_t){let n=Gd[e.toLowerCase()];return n!==void 0?this.setHex(n,t):console.warn("THREE.Color: Unknown color "+e),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(e){return this.r=e.r,this.g=e.g,this.b=e.b,this}copySRGBToLinear(e){return this.r=fi(e.r),this.g=fi(e.g),this.b=fi(e.b),this}copyLinearToSRGB(e){return this.r=Gs(e.r),this.g=Gs(e.g),this.b=Gs(e.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(e=_t){return $e.workingToColorSpace(qt.copy(this),e),Math.round(qe(qt.r*255,0,255))*65536+Math.round(qe(qt.g*255,0,255))*256+Math.round(qe(qt.b*255,0,255))}getHexString(e=_t){return("000000"+this.getHex(e).toString(16)).slice(-6)}getHSL(e,t=$e.workingColorSpace){$e.workingToColorSpace(qt.copy(this),t);let n=qt.r,i=qt.g,r=qt.b,o=Math.max(n,i,r),a=Math.min(n,i,r),c,l,h=(a+o)/2;if(a===o)c=0,l=0;else{let u=o-a;switch(l=h<=.5?u/(o+a):u/(2-o-a),o){case n:c=(i-r)/u+(i<r?6:0);break;case i:c=(r-n)/u+2;break;case r:c=(n-i)/u+4;break}c/=6}return e.h=c,e.s=l,e.l=h,e}getRGB(e,t=$e.workingColorSpace){return $e.workingToColorSpace(qt.copy(this),t),e.r=qt.r,e.g=qt.g,e.b=qt.b,e}getStyle(e=_t){$e.workingToColorSpace(qt.copy(this),e);let t=qt.r,n=qt.g,i=qt.b;return e!==_t?`color(${e} ${t.toFixed(3)} ${n.toFixed(3)} ${i.toFixed(3)})`:`rgb(${Math.round(t*255)},${Math.round(n*255)},${Math.round(i*255)})`}offsetHSL(e,t,n){return this.getHSL(Li),this.setHSL(Li.h+e,Li.s+t,Li.l+n)}add(e){return this.r+=e.r,this.g+=e.g,this.b+=e.b,this}addColors(e,t){return this.r=e.r+t.r,this.g=e.g+t.g,this.b=e.b+t.b,this}addScalar(e){return this.r+=e,this.g+=e,this.b+=e,this}sub(e){return this.r=Math.max(0,this.r-e.r),this.g=Math.max(0,this.g-e.g),this.b=Math.max(0,this.b-e.b),this}multiply(e){return this.r*=e.r,this.g*=e.g,this.b*=e.b,this}multiplyScalar(e){return this.r*=e,this.g*=e,this.b*=e,this}lerp(e,t){return this.r+=(e.r-this.r)*t,this.g+=(e.g-this.g)*t,this.b+=(e.b-this.b)*t,this}lerpColors(e,t,n){return this.r=e.r+(t.r-e.r)*n,this.g=e.g+(t.g-e.g)*n,this.b=e.b+(t.b-e.b)*n,this}lerpHSL(e,t){this.getHSL(Li),e.getHSL(Ro);let n=Ir(Li.h,Ro.h,t),i=Ir(Li.s,Ro.s,t),r=Ir(Li.l,Ro.l,t);return this.setHSL(n,i,r),this}setFromVector3(e){return this.r=e.x,this.g=e.y,this.b=e.z,this}applyMatrix3(e){let t=this.r,n=this.g,i=this.b,r=e.elements;return this.r=r[0]*t+r[3]*n+r[6]*i,this.g=r[1]*t+r[4]*n+r[7]*i,this.b=r[2]*t+r[5]*n+r[8]*i,this}equals(e){return e.r===this.r&&e.g===this.g&&e.b===this.b}fromArray(e,t=0){return this.r=e[t],this.g=e[t+1],this.b=e[t+2],this}toArray(e=[],t=0){return e[t]=this.r,e[t+1]=this.g,e[t+2]=this.b,e}fromBufferAttribute(e,t){return this.r=e.getX(t),this.g=e.getY(t),this.b=e.getZ(t),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}},qt=new Re;Re.NAMES=Gd;var Vp=0,Yt=class extends Kn{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:Vp++}),this.uuid=On(),this.name="",this.type="Material",this.blending=ns,this.side=mn,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=Ko,this.blendDst=Zo,this.blendEquation=Di,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new Re(0,0,0),this.blendAlpha=0,this.depthFunc=is,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=nl,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=ts,this.stencilZFail=ts,this.stencilZPass=ts,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.allowOverride=!0,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(e){this._alphaTest>0!=e>0&&this.version++,this._alphaTest=e}onBeforeRender(){}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(e){if(e!==void 0)for(let t in e){let n=e[t];if(n===void 0){console.warn(`THREE.Material: parameter '${t}' has value of undefined.`);continue}let i=this[t];if(i===void 0){console.warn(`THREE.Material: '${t}' is not a property of THREE.${this.type}.`);continue}i&&i.isColor?i.set(n):i&&i.isVector3&&n&&n.isVector3?i.copy(n):this[t]=n}}toJSON(e){let t=e===void 0||typeof e=="string";t&&(e={textures:{},images:{}});let n={metadata:{version:4.7,type:"Material",generator:"Material.toJSON"}};n.uuid=this.uuid,n.type=this.type,this.name!==""&&(n.name=this.name),this.color&&this.color.isColor&&(n.color=this.color.getHex()),this.roughness!==void 0&&(n.roughness=this.roughness),this.metalness!==void 0&&(n.metalness=this.metalness),this.sheen!==void 0&&(n.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(n.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(n.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(n.emissive=this.emissive.getHex()),this.emissiveIntensity!==void 0&&this.emissiveIntensity!==1&&(n.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(n.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(n.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(n.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(n.shininess=this.shininess),this.clearcoat!==void 0&&(n.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(n.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(n.clearcoatMap=this.clearcoatMap.toJSON(e).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(n.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(e).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(n.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(e).uuid,n.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.sheenColorMap&&this.sheenColorMap.isTexture&&(n.sheenColorMap=this.sheenColorMap.toJSON(e).uuid),this.sheenRoughnessMap&&this.sheenRoughnessMap.isTexture&&(n.sheenRoughnessMap=this.sheenRoughnessMap.toJSON(e).uuid),this.dispersion!==void 0&&(n.dispersion=this.dispersion),this.iridescence!==void 0&&(n.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(n.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(n.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(n.iridescenceMap=this.iridescenceMap.toJSON(e).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(n.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(e).uuid),this.anisotropy!==void 0&&(n.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(n.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(n.anisotropyMap=this.anisotropyMap.toJSON(e).uuid),this.map&&this.map.isTexture&&(n.map=this.map.toJSON(e).uuid),this.matcap&&this.matcap.isTexture&&(n.matcap=this.matcap.toJSON(e).uuid),this.alphaMap&&this.alphaMap.isTexture&&(n.alphaMap=this.alphaMap.toJSON(e).uuid),this.lightMap&&this.lightMap.isTexture&&(n.lightMap=this.lightMap.toJSON(e).uuid,n.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(n.aoMap=this.aoMap.toJSON(e).uuid,n.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(n.bumpMap=this.bumpMap.toJSON(e).uuid,n.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(n.normalMap=this.normalMap.toJSON(e).uuid,n.normalMapType=this.normalMapType,n.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(n.displacementMap=this.displacementMap.toJSON(e).uuid,n.displacementScale=this.displacementScale,n.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(n.roughnessMap=this.roughnessMap.toJSON(e).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(n.metalnessMap=this.metalnessMap.toJSON(e).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(n.emissiveMap=this.emissiveMap.toJSON(e).uuid),this.specularMap&&this.specularMap.isTexture&&(n.specularMap=this.specularMap.toJSON(e).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(n.specularIntensityMap=this.specularIntensityMap.toJSON(e).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(n.specularColorMap=this.specularColorMap.toJSON(e).uuid),this.envMap&&this.envMap.isTexture&&(n.envMap=this.envMap.toJSON(e).uuid,this.combine!==void 0&&(n.combine=this.combine)),this.envMapRotation!==void 0&&(n.envMapRotation=this.envMapRotation.toArray()),this.envMapIntensity!==void 0&&(n.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(n.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(n.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(n.gradientMap=this.gradientMap.toJSON(e).uuid),this.transmission!==void 0&&(n.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(n.transmissionMap=this.transmissionMap.toJSON(e).uuid),this.thickness!==void 0&&(n.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(n.thicknessMap=this.thicknessMap.toJSON(e).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(n.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(n.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(n.size=this.size),this.shadowSide!==null&&(n.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(n.sizeAttenuation=this.sizeAttenuation),this.blending!==ns&&(n.blending=this.blending),this.side!==mn&&(n.side=this.side),this.vertexColors===!0&&(n.vertexColors=!0),this.opacity<1&&(n.opacity=this.opacity),this.transparent===!0&&(n.transparent=!0),this.blendSrc!==Ko&&(n.blendSrc=this.blendSrc),this.blendDst!==Zo&&(n.blendDst=this.blendDst),this.blendEquation!==Di&&(n.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(n.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(n.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(n.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(n.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(n.blendAlpha=this.blendAlpha),this.depthFunc!==is&&(n.depthFunc=this.depthFunc),this.depthTest===!1&&(n.depthTest=this.depthTest),this.depthWrite===!1&&(n.depthWrite=this.depthWrite),this.colorWrite===!1&&(n.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(n.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==nl&&(n.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(n.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(n.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==ts&&(n.stencilFail=this.stencilFail),this.stencilZFail!==ts&&(n.stencilZFail=this.stencilZFail),this.stencilZPass!==ts&&(n.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(n.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(n.rotation=this.rotation),this.polygonOffset===!0&&(n.polygonOffset=!0),this.polygonOffsetFactor!==0&&(n.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(n.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(n.linewidth=this.linewidth),this.dashSize!==void 0&&(n.dashSize=this.dashSize),this.gapSize!==void 0&&(n.gapSize=this.gapSize),this.scale!==void 0&&(n.scale=this.scale),this.dithering===!0&&(n.dithering=!0),this.alphaTest>0&&(n.alphaTest=this.alphaTest),this.alphaHash===!0&&(n.alphaHash=!0),this.alphaToCoverage===!0&&(n.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(n.premultipliedAlpha=!0),this.forceSinglePass===!0&&(n.forceSinglePass=!0),this.wireframe===!0&&(n.wireframe=!0),this.wireframeLinewidth>1&&(n.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(n.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(n.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(n.flatShading=!0),this.visible===!1&&(n.visible=!1),this.toneMapped===!1&&(n.toneMapped=!1),this.fog===!1&&(n.fog=!1),Object.keys(this.userData).length>0&&(n.userData=this.userData);function i(r){let o=[];for(let a in r){let c=r[a];delete c.metadata,o.push(c)}return o}if(t){let r=i(e.textures),o=i(e.images);r.length>0&&(n.textures=r),o.length>0&&(n.images=o)}return n}clone(){return new this.constructor().copy(this)}copy(e){this.name=e.name,this.blending=e.blending,this.side=e.side,this.vertexColors=e.vertexColors,this.opacity=e.opacity,this.transparent=e.transparent,this.blendSrc=e.blendSrc,this.blendDst=e.blendDst,this.blendEquation=e.blendEquation,this.blendSrcAlpha=e.blendSrcAlpha,this.blendDstAlpha=e.blendDstAlpha,this.blendEquationAlpha=e.blendEquationAlpha,this.blendColor.copy(e.blendColor),this.blendAlpha=e.blendAlpha,this.depthFunc=e.depthFunc,this.depthTest=e.depthTest,this.depthWrite=e.depthWrite,this.stencilWriteMask=e.stencilWriteMask,this.stencilFunc=e.stencilFunc,this.stencilRef=e.stencilRef,this.stencilFuncMask=e.stencilFuncMask,this.stencilFail=e.stencilFail,this.stencilZFail=e.stencilZFail,this.stencilZPass=e.stencilZPass,this.stencilWrite=e.stencilWrite;let t=e.clippingPlanes,n=null;if(t!==null){let i=t.length;n=new Array(i);for(let r=0;r!==i;++r)n[r]=t[r].clone()}return this.clippingPlanes=n,this.clipIntersection=e.clipIntersection,this.clipShadows=e.clipShadows,this.shadowSide=e.shadowSide,this.colorWrite=e.colorWrite,this.precision=e.precision,this.polygonOffset=e.polygonOffset,this.polygonOffsetFactor=e.polygonOffsetFactor,this.polygonOffsetUnits=e.polygonOffsetUnits,this.dithering=e.dithering,this.alphaTest=e.alphaTest,this.alphaHash=e.alphaHash,this.alphaToCoverage=e.alphaToCoverage,this.premultipliedAlpha=e.premultipliedAlpha,this.forceSinglePass=e.forceSinglePass,this.visible=e.visible,this.toneMapped=e.toneMapped,this.userData=JSON.parse(JSON.stringify(e.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(e){e===!0&&this.version++}},jt=class extends Yt{constructor(e){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new Re(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Fn,this.combine=ml,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.specularMap=e.specularMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.combine=e.combine,this.reflectivity=e.reflectivity,this.refractionRatio=e.refractionRatio,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.fog=e.fog,this}};var Tt=new A,Co=new Ee,Gp=0,at=class{constructor(e,t,n=!1){if(Array.isArray(e))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,Object.defineProperty(this,"id",{value:Gp++}),this.name="",this.array=e,this.itemSize=t,this.count=e!==void 0?e.length/t:0,this.normalized=n,this.usage=Jo,this.updateRanges=[],this.gpuType=wn,this.version=0}onUploadCallback(){}set needsUpdate(e){e===!0&&this.version++}setUsage(e){return this.usage=e,this}addUpdateRange(e,t){this.updateRanges.push({start:e,count:t})}clearUpdateRanges(){this.updateRanges.length=0}copy(e){return this.name=e.name,this.array=new e.array.constructor(e.array),this.itemSize=e.itemSize,this.count=e.count,this.normalized=e.normalized,this.usage=e.usage,this.gpuType=e.gpuType,this}copyAt(e,t,n){e*=this.itemSize,n*=t.itemSize;for(let i=0,r=this.itemSize;i<r;i++)this.array[e+i]=t.array[n+i];return this}copyArray(e){return this.array.set(e),this}applyMatrix3(e){if(this.itemSize===2)for(let t=0,n=this.count;t<n;t++)Co.fromBufferAttribute(this,t),Co.applyMatrix3(e),this.setXY(t,Co.x,Co.y);else if(this.itemSize===3)for(let t=0,n=this.count;t<n;t++)Tt.fromBufferAttribute(this,t),Tt.applyMatrix3(e),this.setXYZ(t,Tt.x,Tt.y,Tt.z);return this}applyMatrix4(e){for(let t=0,n=this.count;t<n;t++)Tt.fromBufferAttribute(this,t),Tt.applyMatrix4(e),this.setXYZ(t,Tt.x,Tt.y,Tt.z);return this}applyNormalMatrix(e){for(let t=0,n=this.count;t<n;t++)Tt.fromBufferAttribute(this,t),Tt.applyNormalMatrix(e),this.setXYZ(t,Tt.x,Tt.y,Tt.z);return this}transformDirection(e){for(let t=0,n=this.count;t<n;t++)Tt.fromBufferAttribute(this,t),Tt.transformDirection(e),this.setXYZ(t,Tt.x,Tt.y,Tt.z);return this}set(e,t=0){return this.array.set(e,t),this}getComponent(e,t){let n=this.array[e*this.itemSize+t];return this.normalized&&(n=Dn(n,this.array)),n}setComponent(e,t,n){return this.normalized&&(n=rt(n,this.array)),this.array[e*this.itemSize+t]=n,this}getX(e){let t=this.array[e*this.itemSize];return this.normalized&&(t=Dn(t,this.array)),t}setX(e,t){return this.normalized&&(t=rt(t,this.array)),this.array[e*this.itemSize]=t,this}getY(e){let t=this.array[e*this.itemSize+1];return this.normalized&&(t=Dn(t,this.array)),t}setY(e,t){return this.normalized&&(t=rt(t,this.array)),this.array[e*this.itemSize+1]=t,this}getZ(e){let t=this.array[e*this.itemSize+2];return this.normalized&&(t=Dn(t,this.array)),t}setZ(e,t){return this.normalized&&(t=rt(t,this.array)),this.array[e*this.itemSize+2]=t,this}getW(e){let t=this.array[e*this.itemSize+3];return this.normalized&&(t=Dn(t,this.array)),t}setW(e,t){return this.normalized&&(t=rt(t,this.array)),this.array[e*this.itemSize+3]=t,this}setXY(e,t,n){return e*=this.itemSize,this.normalized&&(t=rt(t,this.array),n=rt(n,this.array)),this.array[e+0]=t,this.array[e+1]=n,this}setXYZ(e,t,n,i){return e*=this.itemSize,this.normalized&&(t=rt(t,this.array),n=rt(n,this.array),i=rt(i,this.array)),this.array[e+0]=t,this.array[e+1]=n,this.array[e+2]=i,this}setXYZW(e,t,n,i,r){return e*=this.itemSize,this.normalized&&(t=rt(t,this.array),n=rt(n,this.array),i=rt(i,this.array),r=rt(r,this.array)),this.array[e+0]=t,this.array[e+1]=n,this.array[e+2]=i,this.array[e+3]=r,this}onUpload(e){return this.onUploadCallback=e,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){let e={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(e.name=this.name),this.usage!==Jo&&(e.usage=this.usage),e}};var Dr=class extends at{constructor(e,t,n){super(new Uint16Array(e),t,n)}};var Ur=class extends at{constructor(e,t,n){super(new Uint32Array(e),t,n)}};var bt=class extends at{constructor(e,t,n){super(new Float32Array(e),t,n)}},Wp=0,Mn=new De,Vc=new pt,Ds=new A,fn=new Rt,br=new Rt,zt=new A,xt=class s extends Kn{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:Wp++}),this.uuid=On(),this.name="",this.type="BufferGeometry",this.index=null,this.indirect=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(e){return Array.isArray(e)?this.index=new(Cl(e)?Ur:Dr)(e,1):this.index=e,this}setIndirect(e){return this.indirect=e,this}getIndirect(){return this.indirect}getAttribute(e){return this.attributes[e]}setAttribute(e,t){return this.attributes[e]=t,this}deleteAttribute(e){return delete this.attributes[e],this}hasAttribute(e){return this.attributes[e]!==void 0}addGroup(e,t,n=0){this.groups.push({start:e,count:t,materialIndex:n})}clearGroups(){this.groups=[]}setDrawRange(e,t){this.drawRange.start=e,this.drawRange.count=t}applyMatrix4(e){let t=this.attributes.position;t!==void 0&&(t.applyMatrix4(e),t.needsUpdate=!0);let n=this.attributes.normal;if(n!==void 0){let r=new Ve().getNormalMatrix(e);n.applyNormalMatrix(r),n.needsUpdate=!0}let i=this.attributes.tangent;return i!==void 0&&(i.transformDirection(e),i.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(e){return Mn.makeRotationFromQuaternion(e),this.applyMatrix4(Mn),this}rotateX(e){return Mn.makeRotationX(e),this.applyMatrix4(Mn),this}rotateY(e){return Mn.makeRotationY(e),this.applyMatrix4(Mn),this}rotateZ(e){return Mn.makeRotationZ(e),this.applyMatrix4(Mn),this}translate(e,t,n){return Mn.makeTranslation(e,t,n),this.applyMatrix4(Mn),this}scale(e,t,n){return Mn.makeScale(e,t,n),this.applyMatrix4(Mn),this}lookAt(e){return Vc.lookAt(e),Vc.updateMatrix(),this.applyMatrix4(Vc.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(Ds).negate(),this.translate(Ds.x,Ds.y,Ds.z),this}setFromPoints(e){let t=this.getAttribute("position");if(t===void 0){let n=[];for(let i=0,r=e.length;i<r;i++){let o=e[i];n.push(o.x,o.y,o.z||0)}this.setAttribute("position",new bt(n,3))}else{let n=Math.min(e.length,t.count);for(let i=0;i<n;i++){let r=e[i];t.setXYZ(i,r.x,r.y,r.z||0)}e.length>t.count&&console.warn("THREE.BufferGeometry: Buffer size too small for points data. Use .dispose() and create a new geometry."),t.needsUpdate=!0}return this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new Rt);let e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box.",this),this.boundingBox.set(new A(-1/0,-1/0,-1/0),new A(1/0,1/0,1/0));return}if(e!==void 0){if(this.boundingBox.setFromBufferAttribute(e),t)for(let n=0,i=t.length;n<i;n++){let r=t[n];fn.setFromBufferAttribute(r),this.morphTargetsRelative?(zt.addVectors(this.boundingBox.min,fn.min),this.boundingBox.expandByPoint(zt),zt.addVectors(this.boundingBox.max,fn.max),this.boundingBox.expandByPoint(zt)):(this.boundingBox.expandByPoint(fn.min),this.boundingBox.expandByPoint(fn.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new sn);let e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere.",this),this.boundingSphere.set(new A,1/0);return}if(e){let n=this.boundingSphere.center;if(fn.setFromBufferAttribute(e),t)for(let r=0,o=t.length;r<o;r++){let a=t[r];br.setFromBufferAttribute(a),this.morphTargetsRelative?(zt.addVectors(fn.min,br.min),fn.expandByPoint(zt),zt.addVectors(fn.max,br.max),fn.expandByPoint(zt)):(fn.expandByPoint(br.min),fn.expandByPoint(br.max))}fn.getCenter(n);let i=0;for(let r=0,o=e.count;r<o;r++)zt.fromBufferAttribute(e,r),i=Math.max(i,n.distanceToSquared(zt));if(t)for(let r=0,o=t.length;r<o;r++){let a=t[r],c=this.morphTargetsRelative;for(let l=0,h=a.count;l<h;l++)zt.fromBufferAttribute(a,l),c&&(Ds.fromBufferAttribute(e,l),zt.add(Ds)),i=Math.max(i,n.distanceToSquared(zt))}this.boundingSphere.radius=Math.sqrt(i),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){let e=this.index,t=this.attributes;if(e===null||t.position===void 0||t.normal===void 0||t.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}let n=t.position,i=t.normal,r=t.uv;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new at(new Float32Array(4*n.count),4));let o=this.getAttribute("tangent"),a=[],c=[];for(let N=0;N<n.count;N++)a[N]=new A,c[N]=new A;let l=new A,h=new A,u=new A,d=new Ee,f=new Ee,g=new Ee,_=new A,m=new A;function p(N,M,b){l.fromBufferAttribute(n,N),h.fromBufferAttribute(n,M),u.fromBufferAttribute(n,b),d.fromBufferAttribute(r,N),f.fromBufferAttribute(r,M),g.fromBufferAttribute(r,b),h.sub(l),u.sub(l),f.sub(d),g.sub(d);let P=1/(f.x*g.y-g.x*f.y);isFinite(P)&&(_.copy(h).multiplyScalar(g.y).addScaledVector(u,-f.y).multiplyScalar(P),m.copy(u).multiplyScalar(f.x).addScaledVector(h,-g.x).multiplyScalar(P),a[N].add(_),a[M].add(_),a[b].add(_),c[N].add(m),c[M].add(m),c[b].add(m))}let S=this.groups;S.length===0&&(S=[{start:0,count:e.count}]);for(let N=0,M=S.length;N<M;++N){let b=S[N],P=b.start,H=b.count;for(let G=P,q=P+H;G<q;G+=3)p(e.getX(G+0),e.getX(G+1),e.getX(G+2))}let E=new A,y=new A,w=new A,C=new A;function R(N){w.fromBufferAttribute(i,N),C.copy(w);let M=a[N];E.copy(M),E.sub(w.multiplyScalar(w.dot(M))).normalize(),y.crossVectors(C,M);let P=y.dot(c[N])<0?-1:1;o.setXYZW(N,E.x,E.y,E.z,P)}for(let N=0,M=S.length;N<M;++N){let b=S[N],P=b.start,H=b.count;for(let G=P,q=P+H;G<q;G+=3)R(e.getX(G+0)),R(e.getX(G+1)),R(e.getX(G+2))}}computeVertexNormals(){let e=this.index,t=this.getAttribute("position");if(t!==void 0){let n=this.getAttribute("normal");if(n===void 0)n=new at(new Float32Array(t.count*3),3),this.setAttribute("normal",n);else for(let d=0,f=n.count;d<f;d++)n.setXYZ(d,0,0,0);let i=new A,r=new A,o=new A,a=new A,c=new A,l=new A,h=new A,u=new A;if(e)for(let d=0,f=e.count;d<f;d+=3){let g=e.getX(d+0),_=e.getX(d+1),m=e.getX(d+2);i.fromBufferAttribute(t,g),r.fromBufferAttribute(t,_),o.fromBufferAttribute(t,m),h.subVectors(o,r),u.subVectors(i,r),h.cross(u),a.fromBufferAttribute(n,g),c.fromBufferAttribute(n,_),l.fromBufferAttribute(n,m),a.add(h),c.add(h),l.add(h),n.setXYZ(g,a.x,a.y,a.z),n.setXYZ(_,c.x,c.y,c.z),n.setXYZ(m,l.x,l.y,l.z)}else for(let d=0,f=t.count;d<f;d+=3)i.fromBufferAttribute(t,d+0),r.fromBufferAttribute(t,d+1),o.fromBufferAttribute(t,d+2),h.subVectors(o,r),u.subVectors(i,r),h.cross(u),n.setXYZ(d+0,h.x,h.y,h.z),n.setXYZ(d+1,h.x,h.y,h.z),n.setXYZ(d+2,h.x,h.y,h.z);this.normalizeNormals(),n.needsUpdate=!0}}normalizeNormals(){let e=this.attributes.normal;for(let t=0,n=e.count;t<n;t++)zt.fromBufferAttribute(e,t),zt.normalize(),e.setXYZ(t,zt.x,zt.y,zt.z)}toNonIndexed(){function e(a,c){let l=a.array,h=a.itemSize,u=a.normalized,d=new l.constructor(c.length*h),f=0,g=0;for(let _=0,m=c.length;_<m;_++){a.isInterleavedBufferAttribute?f=c[_]*a.data.stride+a.offset:f=c[_]*h;for(let p=0;p<h;p++)d[g++]=l[f++]}return new at(d,h,u)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;let t=new s,n=this.index.array,i=this.attributes;for(let a in i){let c=i[a],l=e(c,n);t.setAttribute(a,l)}let r=this.morphAttributes;for(let a in r){let c=[],l=r[a];for(let h=0,u=l.length;h<u;h++){let d=l[h],f=e(d,n);c.push(f)}t.morphAttributes[a]=c}t.morphTargetsRelative=this.morphTargetsRelative;let o=this.groups;for(let a=0,c=o.length;a<c;a++){let l=o[a];t.addGroup(l.start,l.count,l.materialIndex)}return t}toJSON(){let e={metadata:{version:4.7,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(e.uuid=this.uuid,e.type=this.type,this.name!==""&&(e.name=this.name),Object.keys(this.userData).length>0&&(e.userData=this.userData),this.parameters!==void 0){let c=this.parameters;for(let l in c)c[l]!==void 0&&(e[l]=c[l]);return e}e.data={attributes:{}};let t=this.index;t!==null&&(e.data.index={type:t.array.constructor.name,array:Array.prototype.slice.call(t.array)});let n=this.attributes;for(let c in n){let l=n[c];e.data.attributes[c]=l.toJSON(e.data)}let i={},r=!1;for(let c in this.morphAttributes){let l=this.morphAttributes[c],h=[];for(let u=0,d=l.length;u<d;u++){let f=l[u];h.push(f.toJSON(e.data))}h.length>0&&(i[c]=h,r=!0)}r&&(e.data.morphAttributes=i,e.data.morphTargetsRelative=this.morphTargetsRelative);let o=this.groups;o.length>0&&(e.data.groups=JSON.parse(JSON.stringify(o)));let a=this.boundingSphere;return a!==null&&(e.data.boundingSphere=a.toJSON()),e}clone(){return new this.constructor().copy(this)}copy(e){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;let t={};this.name=e.name;let n=e.index;n!==null&&this.setIndex(n.clone());let i=e.attributes;for(let l in i){let h=i[l];this.setAttribute(l,h.clone(t))}let r=e.morphAttributes;for(let l in r){let h=[],u=r[l];for(let d=0,f=u.length;d<f;d++)h.push(u[d].clone(t));this.morphAttributes[l]=h}this.morphTargetsRelative=e.morphTargetsRelative;let o=e.groups;for(let l=0,h=o.length;l<h;l++){let u=o[l];this.addGroup(u.start,u.count,u.materialIndex)}let a=e.boundingBox;a!==null&&(this.boundingBox=a.clone());let c=e.boundingSphere;return c!==null&&(this.boundingSphere=c.clone()),this.drawRange.start=e.drawRange.start,this.drawRange.count=e.drawRange.count,this.userData=e.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}},Ru=new De,Qi=new Jn,Io=new sn,Cu=new A,Po=new A,Lo=new A,No=new A,Gc=new A,Do=new A,Iu=new A,Uo=new A,lt=class extends pt{constructor(e=new xt,t=new jt){super(),this.isMesh=!0,this.type="Mesh",this.geometry=e,this.material=t,this.morphTargetDictionary=void 0,this.morphTargetInfluences=void 0,this.count=1,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),e.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=e.morphTargetInfluences.slice()),e.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},e.morphTargetDictionary)),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}updateMorphTargets(){let t=this.geometry.morphAttributes,n=Object.keys(t);if(n.length>0){let i=t[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,o=i.length;r<o;r++){let a=i[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=r}}}}getVertexPosition(e,t){let n=this.geometry,i=n.attributes.position,r=n.morphAttributes.position,o=n.morphTargetsRelative;t.fromBufferAttribute(i,e);let a=this.morphTargetInfluences;if(r&&a){Do.set(0,0,0);for(let c=0,l=r.length;c<l;c++){let h=a[c],u=r[c];h!==0&&(Gc.fromBufferAttribute(u,e),o?Do.addScaledVector(Gc,h):Do.addScaledVector(Gc.sub(t),h))}t.add(Do)}return t}raycast(e,t){let n=this.geometry,i=this.material,r=this.matrixWorld;i!==void 0&&(n.boundingSphere===null&&n.computeBoundingSphere(),Io.copy(n.boundingSphere),Io.applyMatrix4(r),Qi.copy(e.ray).recast(e.near),!(Io.containsPoint(Qi.origin)===!1&&(Qi.intersectSphere(Io,Cu)===null||Qi.origin.distanceToSquared(Cu)>(e.far-e.near)**2))&&(Ru.copy(r).invert(),Qi.copy(e.ray).applyMatrix4(Ru),!(n.boundingBox!==null&&Qi.intersectsBox(n.boundingBox)===!1)&&this._computeIntersections(e,t,Qi)))}_computeIntersections(e,t,n){let i,r=this.geometry,o=this.material,a=r.index,c=r.attributes.position,l=r.attributes.uv,h=r.attributes.uv1,u=r.attributes.normal,d=r.groups,f=r.drawRange;if(a!==null)if(Array.isArray(o))for(let g=0,_=d.length;g<_;g++){let m=d[g],p=o[m.materialIndex],S=Math.max(m.start,f.start),E=Math.min(a.count,Math.min(m.start+m.count,f.start+f.count));for(let y=S,w=E;y<w;y+=3){let C=a.getX(y),R=a.getX(y+1),N=a.getX(y+2);i=Oo(this,p,e,n,l,h,u,C,R,N),i&&(i.faceIndex=Math.floor(y/3),i.face.materialIndex=m.materialIndex,t.push(i))}}else{let g=Math.max(0,f.start),_=Math.min(a.count,f.start+f.count);for(let m=g,p=_;m<p;m+=3){let S=a.getX(m),E=a.getX(m+1),y=a.getX(m+2);i=Oo(this,o,e,n,l,h,u,S,E,y),i&&(i.faceIndex=Math.floor(m/3),t.push(i))}}else if(c!==void 0)if(Array.isArray(o))for(let g=0,_=d.length;g<_;g++){let m=d[g],p=o[m.materialIndex],S=Math.max(m.start,f.start),E=Math.min(c.count,Math.min(m.start+m.count,f.start+f.count));for(let y=S,w=E;y<w;y+=3){let C=y,R=y+1,N=y+2;i=Oo(this,p,e,n,l,h,u,C,R,N),i&&(i.faceIndex=Math.floor(y/3),i.face.materialIndex=m.materialIndex,t.push(i))}}else{let g=Math.max(0,f.start),_=Math.min(c.count,f.start+f.count);for(let m=g,p=_;m<p;m+=3){let S=m,E=m+1,y=m+2;i=Oo(this,o,e,n,l,h,u,S,E,y),i&&(i.faceIndex=Math.floor(m/3),t.push(i))}}}};function Xp(s,e,t,n,i,r,o,a){let c;if(e.side===en?c=n.intersectTriangle(o,r,i,!0,a):c=n.intersectTriangle(i,r,o,e.side===mn,a),c===null)return null;Uo.copy(a),Uo.applyMatrix4(s.matrixWorld);let l=t.ray.origin.distanceTo(Uo);return l<t.near||l>t.far?null:{distance:l,point:Uo.clone(),object:s}}function Oo(s,e,t,n,i,r,o,a,c,l){s.getVertexPosition(a,Po),s.getVertexPosition(c,Lo),s.getVertexPosition(l,No);let h=Xp(s,e,t,n,Po,Lo,No,Iu);if(h){let u=new A;pn.getBarycoord(Iu,Po,Lo,No,u),i&&(h.uv=pn.getInterpolatedAttribute(i,a,c,l,u,new Ee)),r&&(h.uv1=pn.getInterpolatedAttribute(r,a,c,l,u,new Ee)),o&&(h.normal=pn.getInterpolatedAttribute(o,a,c,l,u,new A),h.normal.dot(n.direction)>0&&h.normal.multiplyScalar(-1));let d={a,b:c,c:l,normal:new A,materialIndex:0};pn.getNormal(Po,Lo,No,d.normal),h.face=d,h.barycoord=u}return h}var Ks=class s extends xt{constructor(e=1,t=1,n=1,i=1,r=1,o=1){super(),this.type="BoxGeometry",this.parameters={width:e,height:t,depth:n,widthSegments:i,heightSegments:r,depthSegments:o};let a=this;i=Math.floor(i),r=Math.floor(r),o=Math.floor(o);let c=[],l=[],h=[],u=[],d=0,f=0;g("z","y","x",-1,-1,n,t,e,o,r,0),g("z","y","x",1,-1,n,t,-e,o,r,1),g("x","z","y",1,1,e,n,t,i,o,2),g("x","z","y",1,-1,e,n,-t,i,o,3),g("x","y","z",1,-1,e,t,n,i,r,4),g("x","y","z",-1,-1,e,t,-n,i,r,5),this.setIndex(c),this.setAttribute("position",new bt(l,3)),this.setAttribute("normal",new bt(h,3)),this.setAttribute("uv",new bt(u,2));function g(_,m,p,S,E,y,w,C,R,N,M){let b=y/R,P=w/N,H=y/2,G=w/2,q=C/2,W=R+1,Y=N+1,B=0,D=0,Q=new A;for(let ae=0;ae<Y;ae++){let ce=ae*P-G;for(let ve=0;ve<W;ve++){let Ze=ve*b-H;Q[_]=Ze*S,Q[m]=ce*E,Q[p]=q,l.push(Q.x,Q.y,Q.z),Q[_]=0,Q[m]=0,Q[p]=C>0?1:-1,h.push(Q.x,Q.y,Q.z),u.push(ve/R),u.push(1-ae/N),B+=1}}for(let ae=0;ae<N;ae++)for(let ce=0;ce<R;ce++){let ve=d+ce+W*ae,Ze=d+ce+W*(ae+1),it=d+(ce+1)+W*(ae+1),Ye=d+(ce+1)+W*ae;c.push(ve,Ze,Ye),c.push(Ze,it,Ye),D+=6}a.addGroup(f,D,M),f+=D,d+=B}}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new s(e.width,e.height,e.depth,e.widthSegments,e.heightSegments,e.depthSegments)}};function fs(s){let e={};for(let t in s){e[t]={};for(let n in s[t]){let i=s[t][n];i&&(i.isColor||i.isMatrix3||i.isMatrix4||i.isVector2||i.isVector3||i.isVector4||i.isTexture||i.isQuaternion)?i.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),e[t][n]=null):e[t][n]=i.clone():Array.isArray(i)?e[t][n]=i.slice():e[t][n]=i}}return e}function Zt(s){let e={};for(let t=0;t<s.length;t++){let n=fs(s[t]);for(let i in n)e[i]=n[i]}return e}function qp(s){let e=[];for(let t=0;t<s.length;t++)e.push(s[t].clone());return e}function Il(s){let e=s.getRenderTarget();return e===null?s.outputColorSpace:e.isXRRenderTarget===!0?e.texture.colorSpace:$e.workingColorSpace}var Wd={clone:fs,merge:Zt},Yp=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,jp=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`,Bn=class extends Yt{constructor(e){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=Yp,this.fragmentShader=jp,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={clipCullDistance:!1,multiDraw:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,e!==void 0&&this.setValues(e)}copy(e){return super.copy(e),this.fragmentShader=e.fragmentShader,this.vertexShader=e.vertexShader,this.uniforms=fs(e.uniforms),this.uniformsGroups=qp(e.uniformsGroups),this.defines=Object.assign({},e.defines),this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.fog=e.fog,this.lights=e.lights,this.clipping=e.clipping,this.extensions=Object.assign({},e.extensions),this.glslVersion=e.glslVersion,this}toJSON(e){let t=super.toJSON(e);t.glslVersion=this.glslVersion,t.uniforms={};for(let i in this.uniforms){let o=this.uniforms[i].value;o&&o.isTexture?t.uniforms[i]={type:"t",value:o.toJSON(e).uuid}:o&&o.isColor?t.uniforms[i]={type:"c",value:o.getHex()}:o&&o.isVector2?t.uniforms[i]={type:"v2",value:o.toArray()}:o&&o.isVector3?t.uniforms[i]={type:"v3",value:o.toArray()}:o&&o.isVector4?t.uniforms[i]={type:"v4",value:o.toArray()}:o&&o.isMatrix3?t.uniforms[i]={type:"m3",value:o.toArray()}:o&&o.isMatrix4?t.uniforms[i]={type:"m4",value:o.toArray()}:t.uniforms[i]={value:o}}Object.keys(this.defines).length>0&&(t.defines=this.defines),t.vertexShader=this.vertexShader,t.fragmentShader=this.fragmentShader,t.lights=this.lights,t.clipping=this.clipping;let n={};for(let i in this.extensions)this.extensions[i]===!0&&(n[i]=!0);return Object.keys(n).length>0&&(t.extensions=n),t}},Or=class extends pt{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new De,this.projectionMatrix=new De,this.projectionMatrixInverse=new De,this.coordinateSystem=Un,this._reversedDepth=!1}get reversedDepth(){return this._reversedDepth}copy(e,t){return super.copy(e,t),this.matrixWorldInverse.copy(e.matrixWorldInverse),this.projectionMatrix.copy(e.projectionMatrix),this.projectionMatrixInverse.copy(e.projectionMatrixInverse),this.coordinateSystem=e.coordinateSystem,this}getWorldDirection(e){return super.getWorldDirection(e).negate()}updateMatrixWorld(e){super.updateMatrixWorld(e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(e,t){super.updateWorldMatrix(e,t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}},Ni=new A,Pu=new Ee,Lu=new Ee,wt=class extends Or{constructor(e=50,t=1,n=.1,i=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=e,this.zoom=1,this.near=n,this.far=i,this.focus=10,this.aspect=t,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.fov=e.fov,this.zoom=e.zoom,this.near=e.near,this.far=e.far,this.focus=e.focus,this.aspect=e.aspect,this.view=e.view===null?null:Object.assign({},e.view),this.filmGauge=e.filmGauge,this.filmOffset=e.filmOffset,this}setFocalLength(e){let t=.5*this.getFilmHeight()/e;this.fov=ss*2*Math.atan(t),this.updateProjectionMatrix()}getFocalLength(){let e=Math.tan(Cr*.5*this.fov);return .5*this.getFilmHeight()/e}getEffectiveFOV(){return ss*2*Math.atan(Math.tan(Cr*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}getViewBounds(e,t,n){Ni.set(-1,-1,.5).applyMatrix4(this.projectionMatrixInverse),t.set(Ni.x,Ni.y).multiplyScalar(-e/Ni.z),Ni.set(1,1,.5).applyMatrix4(this.projectionMatrixInverse),n.set(Ni.x,Ni.y).multiplyScalar(-e/Ni.z)}getViewSize(e,t){return this.getViewBounds(e,Pu,Lu),t.subVectors(Lu,Pu)}setViewOffset(e,t,n,i,r,o){this.aspect=e/t,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=n,this.view.offsetY=i,this.view.width=r,this.view.height=o,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){let e=this.near,t=e*Math.tan(Cr*.5*this.fov)/this.zoom,n=2*t,i=this.aspect*n,r=-.5*i,o=this.view;if(this.view!==null&&this.view.enabled){let c=o.fullWidth,l=o.fullHeight;r+=o.offsetX*i/c,t-=o.offsetY*n/l,i*=o.width/c,n*=o.height/l}let a=this.filmOffset;a!==0&&(r+=e*a/this.getFilmWidth()),this.projectionMatrix.makePerspective(r,r+i,t,t-n,e,this.far,this.coordinateSystem,this.reversedDepth),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){let t=super.toJSON(e);return t.object.fov=this.fov,t.object.zoom=this.zoom,t.object.near=this.near,t.object.far=this.far,t.object.focus=this.focus,t.object.aspect=this.aspect,this.view!==null&&(t.object.view=Object.assign({},this.view)),t.object.filmGauge=this.filmGauge,t.object.filmOffset=this.filmOffset,t}},Us=-90,Os=1,ea=class extends pt{constructor(e,t,n){super(),this.type="CubeCamera",this.renderTarget=n,this.coordinateSystem=null,this.activeMipmapLevel=0;let i=new wt(Us,Os,e,t);i.layers=this.layers,this.add(i);let r=new wt(Us,Os,e,t);r.layers=this.layers,this.add(r);let o=new wt(Us,Os,e,t);o.layers=this.layers,this.add(o);let a=new wt(Us,Os,e,t);a.layers=this.layers,this.add(a);let c=new wt(Us,Os,e,t);c.layers=this.layers,this.add(c);let l=new wt(Us,Os,e,t);l.layers=this.layers,this.add(l)}updateCoordinateSystem(){let e=this.coordinateSystem,t=this.children.concat(),[n,i,r,o,a,c]=t;for(let l of t)this.remove(l);if(e===Un)n.up.set(0,1,0),n.lookAt(1,0,0),i.up.set(0,1,0),i.lookAt(-1,0,0),r.up.set(0,0,-1),r.lookAt(0,1,0),o.up.set(0,0,1),o.lookAt(0,-1,0),a.up.set(0,1,0),a.lookAt(0,0,1),c.up.set(0,1,0),c.lookAt(0,0,-1);else if(e===Lr)n.up.set(0,-1,0),n.lookAt(-1,0,0),i.up.set(0,-1,0),i.lookAt(1,0,0),r.up.set(0,0,1),r.lookAt(0,1,0),o.up.set(0,0,-1),o.lookAt(0,-1,0),a.up.set(0,-1,0),a.lookAt(0,0,1),c.up.set(0,-1,0),c.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+e);for(let l of t)this.add(l),l.updateMatrixWorld()}update(e,t){this.parent===null&&this.updateMatrixWorld();let{renderTarget:n,activeMipmapLevel:i}=this;this.coordinateSystem!==e.coordinateSystem&&(this.coordinateSystem=e.coordinateSystem,this.updateCoordinateSystem());let[r,o,a,c,l,h]=this.children,u=e.getRenderTarget(),d=e.getActiveCubeFace(),f=e.getActiveMipmapLevel(),g=e.xr.enabled;e.xr.enabled=!1;let _=n.texture.generateMipmaps;n.texture.generateMipmaps=!1,e.setRenderTarget(n,0,i),e.render(t,r),e.setRenderTarget(n,1,i),e.render(t,o),e.setRenderTarget(n,2,i),e.render(t,a),e.setRenderTarget(n,3,i),e.render(t,c),e.setRenderTarget(n,4,i),e.render(t,l),n.texture.generateMipmaps=_,e.setRenderTarget(n,5,i),e.render(t,h),e.setRenderTarget(u,d,f),e.xr.enabled=g,n.texture.needsPMREMUpdate=!0}},Fr=class extends At{constructor(e=[],t=us,n,i,r,o,a,c,l,h){super(e,t,n,i,r,o,a,c,l,h),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(e){this.image=e}},ta=class extends Zn{constructor(e=1,t={}){super(e,e,t),this.isWebGLCubeRenderTarget=!0;let n={width:e,height:e,depth:1},i=[n,n,n,n,n,n];this.texture=new Fr(i),this._setTextureOptions(t),this.texture.isRenderTargetTexture=!0}fromEquirectangularTexture(e,t){this.texture.type=t.type,this.texture.colorSpace=t.colorSpace,this.texture.generateMipmaps=t.generateMipmaps,this.texture.minFilter=t.minFilter,this.texture.magFilter=t.magFilter;let n={uniforms:{tEquirect:{value:null}},vertexShader:`

				varying vec3 vWorldDirection;

				vec3 transformDirection( in vec3 dir, in mat4 matrix ) {

					return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );

				}

				void main() {

					vWorldDirection = transformDirection( position, modelMatrix );

					#include <begin_vertex>
					#include <project_vertex>

				}
			`,fragmentShader:`

				uniform sampler2D tEquirect;

				varying vec3 vWorldDirection;

				#include <common>

				void main() {

					vec3 direction = normalize( vWorldDirection );

					vec2 sampleUV = equirectUv( direction );

					gl_FragColor = texture2D( tEquirect, sampleUV );

				}
			`},i=new Ks(5,5,5),r=new Bn({name:"CubemapFromEquirect",uniforms:fs(n.uniforms),vertexShader:n.vertexShader,fragmentShader:n.fragmentShader,side:en,blending:Mi});r.uniforms.tEquirect.value=t;let o=new lt(i,r),a=t.minFilter;return t.minFilter===gn&&(t.minFilter=Ht),new ea(1,10,this).update(e,o),t.minFilter=a,o.geometry.dispose(),o.material.dispose(),this}clear(e,t=!0,n=!0,i=!0){let r=e.getRenderTarget();for(let o=0;o<6;o++)e.setRenderTarget(this,o),e.clear(t,n,i);e.setRenderTarget(r)}},Et=class extends pt{constructor(){super(),this.isGroup=!0,this.type="Group"}},Kp={type:"move"},Zs=class{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new Et,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new Et,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new A,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new A),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new Et,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new A,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new A),this._grip}dispatchEvent(e){return this._targetRay!==null&&this._targetRay.dispatchEvent(e),this._grip!==null&&this._grip.dispatchEvent(e),this._hand!==null&&this._hand.dispatchEvent(e),this}connect(e){if(e&&e.hand){let t=this._hand;if(t)for(let n of e.hand.values())this._getHandJoint(t,n)}return this.dispatchEvent({type:"connected",data:e}),this}disconnect(e){return this.dispatchEvent({type:"disconnected",data:e}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(e,t,n){let i=null,r=null,o=null,a=this._targetRay,c=this._grip,l=this._hand;if(e&&t.session.visibilityState!=="visible-blurred"){if(l&&e.hand){o=!0;for(let _ of e.hand.values()){let m=t.getJointPose(_,n),p=this._getHandJoint(l,_);m!==null&&(p.matrix.fromArray(m.transform.matrix),p.matrix.decompose(p.position,p.rotation,p.scale),p.matrixWorldNeedsUpdate=!0,p.jointRadius=m.radius),p.visible=m!==null}let h=l.joints["index-finger-tip"],u=l.joints["thumb-tip"],d=h.position.distanceTo(u.position),f=.02,g=.005;l.inputState.pinching&&d>f+g?(l.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:e.handedness,target:this})):!l.inputState.pinching&&d<=f-g&&(l.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:e.handedness,target:this}))}else c!==null&&e.gripSpace&&(r=t.getPose(e.gripSpace,n),r!==null&&(c.matrix.fromArray(r.transform.matrix),c.matrix.decompose(c.position,c.rotation,c.scale),c.matrixWorldNeedsUpdate=!0,r.linearVelocity?(c.hasLinearVelocity=!0,c.linearVelocity.copy(r.linearVelocity)):c.hasLinearVelocity=!1,r.angularVelocity?(c.hasAngularVelocity=!0,c.angularVelocity.copy(r.angularVelocity)):c.hasAngularVelocity=!1));a!==null&&(i=t.getPose(e.targetRaySpace,n),i===null&&r!==null&&(i=r),i!==null&&(a.matrix.fromArray(i.transform.matrix),a.matrix.decompose(a.position,a.rotation,a.scale),a.matrixWorldNeedsUpdate=!0,i.linearVelocity?(a.hasLinearVelocity=!0,a.linearVelocity.copy(i.linearVelocity)):a.hasLinearVelocity=!1,i.angularVelocity?(a.hasAngularVelocity=!0,a.angularVelocity.copy(i.angularVelocity)):a.hasAngularVelocity=!1,this.dispatchEvent(Kp)))}return a!==null&&(a.visible=i!==null),c!==null&&(c.visible=r!==null),l!==null&&(l.visible=o!==null),this}_getHandJoint(e,t){if(e.joints[t.jointName]===void 0){let n=new Et;n.matrixAutoUpdate=!1,n.visible=!1,e.joints[t.jointName]=n,e.add(n)}return e.joints[t.jointName]}};var Fi=class extends pt{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.backgroundRotation=new Fn,this.environmentIntensity=1,this.environmentRotation=new Fn,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(e,t){return super.copy(e,t),e.background!==null&&(this.background=e.background.clone()),e.environment!==null&&(this.environment=e.environment.clone()),e.fog!==null&&(this.fog=e.fog.clone()),this.backgroundBlurriness=e.backgroundBlurriness,this.backgroundIntensity=e.backgroundIntensity,this.backgroundRotation.copy(e.backgroundRotation),this.environmentIntensity=e.environmentIntensity,this.environmentRotation.copy(e.environmentRotation),e.overrideMaterial!==null&&(this.overrideMaterial=e.overrideMaterial.clone()),this.matrixAutoUpdate=e.matrixAutoUpdate,this}toJSON(e){let t=super.toJSON(e);return this.fog!==null&&(t.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(t.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(t.object.backgroundIntensity=this.backgroundIntensity),t.object.backgroundRotation=this.backgroundRotation.toArray(),this.environmentIntensity!==1&&(t.object.environmentIntensity=this.environmentIntensity),t.object.environmentRotation=this.environmentRotation.toArray(),t}},rs=class{constructor(e,t){this.isInterleavedBuffer=!0,this.array=e,this.stride=t,this.count=e!==void 0?e.length/t:0,this.usage=Jo,this.updateRanges=[],this.version=0,this.uuid=On()}onUploadCallback(){}set needsUpdate(e){e===!0&&this.version++}setUsage(e){return this.usage=e,this}addUpdateRange(e,t){this.updateRanges.push({start:e,count:t})}clearUpdateRanges(){this.updateRanges.length=0}copy(e){return this.array=new e.array.constructor(e.array),this.count=e.count,this.stride=e.stride,this.usage=e.usage,this}copyAt(e,t,n){e*=this.stride,n*=t.stride;for(let i=0,r=this.stride;i<r;i++)this.array[e+i]=t.array[n+i];return this}set(e,t=0){return this.array.set(e,t),this}clone(e){e.arrayBuffers===void 0&&(e.arrayBuffers={}),this.array.buffer._uuid===void 0&&(this.array.buffer._uuid=On()),e.arrayBuffers[this.array.buffer._uuid]===void 0&&(e.arrayBuffers[this.array.buffer._uuid]=this.array.slice(0).buffer);let t=new this.array.constructor(e.arrayBuffers[this.array.buffer._uuid]),n=new this.constructor(t,this.stride);return n.setUsage(this.usage),n}onUpload(e){return this.onUploadCallback=e,this}toJSON(e){return e.arrayBuffers===void 0&&(e.arrayBuffers={}),this.array.buffer._uuid===void 0&&(this.array.buffer._uuid=On()),e.arrayBuffers[this.array.buffer._uuid]===void 0&&(e.arrayBuffers[this.array.buffer._uuid]=Array.from(new Uint32Array(this.array.buffer))),{uuid:this.uuid,buffer:this.array.buffer._uuid,type:this.array.constructor.name,stride:this.stride}}},Qt=new A,Bi=class s{constructor(e,t,n,i=!1){this.isInterleavedBufferAttribute=!0,this.name="",this.data=e,this.itemSize=t,this.offset=n,this.normalized=i}get count(){return this.data.count}get array(){return this.data.array}set needsUpdate(e){this.data.needsUpdate=e}applyMatrix4(e){for(let t=0,n=this.data.count;t<n;t++)Qt.fromBufferAttribute(this,t),Qt.applyMatrix4(e),this.setXYZ(t,Qt.x,Qt.y,Qt.z);return this}applyNormalMatrix(e){for(let t=0,n=this.count;t<n;t++)Qt.fromBufferAttribute(this,t),Qt.applyNormalMatrix(e),this.setXYZ(t,Qt.x,Qt.y,Qt.z);return this}transformDirection(e){for(let t=0,n=this.count;t<n;t++)Qt.fromBufferAttribute(this,t),Qt.transformDirection(e),this.setXYZ(t,Qt.x,Qt.y,Qt.z);return this}getComponent(e,t){let n=this.array[e*this.data.stride+this.offset+t];return this.normalized&&(n=Dn(n,this.array)),n}setComponent(e,t,n){return this.normalized&&(n=rt(n,this.array)),this.data.array[e*this.data.stride+this.offset+t]=n,this}setX(e,t){return this.normalized&&(t=rt(t,this.array)),this.data.array[e*this.data.stride+this.offset]=t,this}setY(e,t){return this.normalized&&(t=rt(t,this.array)),this.data.array[e*this.data.stride+this.offset+1]=t,this}setZ(e,t){return this.normalized&&(t=rt(t,this.array)),this.data.array[e*this.data.stride+this.offset+2]=t,this}setW(e,t){return this.normalized&&(t=rt(t,this.array)),this.data.array[e*this.data.stride+this.offset+3]=t,this}getX(e){let t=this.data.array[e*this.data.stride+this.offset];return this.normalized&&(t=Dn(t,this.array)),t}getY(e){let t=this.data.array[e*this.data.stride+this.offset+1];return this.normalized&&(t=Dn(t,this.array)),t}getZ(e){let t=this.data.array[e*this.data.stride+this.offset+2];return this.normalized&&(t=Dn(t,this.array)),t}getW(e){let t=this.data.array[e*this.data.stride+this.offset+3];return this.normalized&&(t=Dn(t,this.array)),t}setXY(e,t,n){return e=e*this.data.stride+this.offset,this.normalized&&(t=rt(t,this.array),n=rt(n,this.array)),this.data.array[e+0]=t,this.data.array[e+1]=n,this}setXYZ(e,t,n,i){return e=e*this.data.stride+this.offset,this.normalized&&(t=rt(t,this.array),n=rt(n,this.array),i=rt(i,this.array)),this.data.array[e+0]=t,this.data.array[e+1]=n,this.data.array[e+2]=i,this}setXYZW(e,t,n,i,r){return e=e*this.data.stride+this.offset,this.normalized&&(t=rt(t,this.array),n=rt(n,this.array),i=rt(i,this.array),r=rt(r,this.array)),this.data.array[e+0]=t,this.data.array[e+1]=n,this.data.array[e+2]=i,this.data.array[e+3]=r,this}clone(e){if(e===void 0){console.log("THREE.InterleavedBufferAttribute.clone(): Cloning an interleaved buffer attribute will de-interleave buffer data.");let t=[];for(let n=0;n<this.count;n++){let i=n*this.data.stride+this.offset;for(let r=0;r<this.itemSize;r++)t.push(this.data.array[i+r])}return new at(new this.array.constructor(t),this.itemSize,this.normalized)}else return e.interleavedBuffers===void 0&&(e.interleavedBuffers={}),e.interleavedBuffers[this.data.uuid]===void 0&&(e.interleavedBuffers[this.data.uuid]=this.data.clone(e)),new s(e.interleavedBuffers[this.data.uuid],this.itemSize,this.offset,this.normalized)}toJSON(e){if(e===void 0){console.log("THREE.InterleavedBufferAttribute.toJSON(): Serializing an interleaved buffer attribute will de-interleave buffer data.");let t=[];for(let n=0;n<this.count;n++){let i=n*this.data.stride+this.offset;for(let r=0;r<this.itemSize;r++)t.push(this.data.array[i+r])}return{itemSize:this.itemSize,type:this.array.constructor.name,array:t,normalized:this.normalized}}else return e.interleavedBuffers===void 0&&(e.interleavedBuffers={}),e.interleavedBuffers[this.data.uuid]===void 0&&(e.interleavedBuffers[this.data.uuid]=this.data.toJSON(e)),{isInterleavedBufferAttribute:!0,itemSize:this.itemSize,data:this.data.uuid,offset:this.offset,normalized:this.normalized}}},Js=class extends Yt{constructor(e){super(),this.isSpriteMaterial=!0,this.type="SpriteMaterial",this.color=new Re(16777215),this.map=null,this.alphaMap=null,this.rotation=0,this.sizeAttenuation=!0,this.transparent=!0,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.alphaMap=e.alphaMap,this.rotation=e.rotation,this.sizeAttenuation=e.sizeAttenuation,this.fog=e.fog,this}},Fs,Mr=new A,Bs=new A,ks=new A,zs=new Ee,Sr=new Ee,Xd=new De,Fo=new A,Er=new A,Bo=new A,Nu=new Ee,Wc=new Ee,Du=new Ee,Br=class extends pt{constructor(e=new Js){if(super(),this.isSprite=!0,this.type="Sprite",Fs===void 0){Fs=new xt;let t=new Float32Array([-.5,-.5,0,0,0,.5,-.5,0,1,0,.5,.5,0,1,1,-.5,.5,0,0,1]),n=new rs(t,5);Fs.setIndex([0,1,2,0,2,3]),Fs.setAttribute("position",new Bi(n,3,0,!1)),Fs.setAttribute("uv",new Bi(n,2,3,!1))}this.geometry=Fs,this.material=e,this.center=new Ee(.5,.5),this.count=1}raycast(e,t){e.camera===null&&console.error('THREE.Sprite: "Raycaster.camera" needs to be set in order to raycast against sprites.'),Bs.setFromMatrixScale(this.matrixWorld),Xd.copy(e.camera.matrixWorld),this.modelViewMatrix.multiplyMatrices(e.camera.matrixWorldInverse,this.matrixWorld),ks.setFromMatrixPosition(this.modelViewMatrix),e.camera.isPerspectiveCamera&&this.material.sizeAttenuation===!1&&Bs.multiplyScalar(-ks.z);let n=this.material.rotation,i,r;n!==0&&(r=Math.cos(n),i=Math.sin(n));let o=this.center;ko(Fo.set(-.5,-.5,0),ks,o,Bs,i,r),ko(Er.set(.5,-.5,0),ks,o,Bs,i,r),ko(Bo.set(.5,.5,0),ks,o,Bs,i,r),Nu.set(0,0),Wc.set(1,0),Du.set(1,1);let a=e.ray.intersectTriangle(Fo,Er,Bo,!1,Mr);if(a===null&&(ko(Er.set(-.5,.5,0),ks,o,Bs,i,r),Wc.set(0,1),a=e.ray.intersectTriangle(Fo,Bo,Er,!1,Mr),a===null))return;let c=e.ray.origin.distanceTo(Mr);c<e.near||c>e.far||t.push({distance:c,point:Mr.clone(),uv:pn.getInterpolation(Mr,Fo,Er,Bo,Nu,Wc,Du,new Ee),face:null,object:this})}copy(e,t){return super.copy(e,t),e.center!==void 0&&this.center.copy(e.center),this.material=e.material,this}};function ko(s,e,t,n,i,r){zs.subVectors(s,t).addScalar(.5).multiply(n),i!==void 0?(Sr.x=r*zs.x-i*zs.y,Sr.y=i*zs.x+r*zs.y):Sr.copy(zs),s.copy(e),s.x+=Sr.x,s.y+=Sr.y,s.applyMatrix4(Xd)}var Uu=new A,Ou=new nt,Fu=new nt,Zp=new A,Bu=new De,zo=new A,Xc=new sn,ku=new De,qc=new Jn,kr=class extends lt{constructor(e,t){super(e,t),this.isSkinnedMesh=!0,this.type="SkinnedMesh",this.bindMode=$c,this.bindMatrix=new De,this.bindMatrixInverse=new De,this.boundingBox=null,this.boundingSphere=null}computeBoundingBox(){let e=this.geometry;this.boundingBox===null&&(this.boundingBox=new Rt),this.boundingBox.makeEmpty();let t=e.getAttribute("position");for(let n=0;n<t.count;n++)this.getVertexPosition(n,zo),this.boundingBox.expandByPoint(zo)}computeBoundingSphere(){let e=this.geometry;this.boundingSphere===null&&(this.boundingSphere=new sn),this.boundingSphere.makeEmpty();let t=e.getAttribute("position");for(let n=0;n<t.count;n++)this.getVertexPosition(n,zo),this.boundingSphere.expandByPoint(zo)}copy(e,t){return super.copy(e,t),this.bindMode=e.bindMode,this.bindMatrix.copy(e.bindMatrix),this.bindMatrixInverse.copy(e.bindMatrixInverse),this.skeleton=e.skeleton,e.boundingBox!==null&&(this.boundingBox=e.boundingBox.clone()),e.boundingSphere!==null&&(this.boundingSphere=e.boundingSphere.clone()),this}raycast(e,t){let n=this.material,i=this.matrixWorld;n!==void 0&&(this.boundingSphere===null&&this.computeBoundingSphere(),Xc.copy(this.boundingSphere),Xc.applyMatrix4(i),e.ray.intersectsSphere(Xc)!==!1&&(ku.copy(i).invert(),qc.copy(e.ray).applyMatrix4(ku),!(this.boundingBox!==null&&qc.intersectsBox(this.boundingBox)===!1)&&this._computeIntersections(e,t,qc)))}getVertexPosition(e,t){return super.getVertexPosition(e,t),this.applyBoneTransform(e,t),t}bind(e,t){this.skeleton=e,t===void 0&&(this.updateMatrixWorld(!0),this.skeleton.calculateInverses(),t=this.matrixWorld),this.bindMatrix.copy(t),this.bindMatrixInverse.copy(t).invert()}pose(){this.skeleton.pose()}normalizeSkinWeights(){let e=new nt,t=this.geometry.attributes.skinWeight;for(let n=0,i=t.count;n<i;n++){e.fromBufferAttribute(t,n);let r=1/e.manhattanLength();r!==1/0?e.multiplyScalar(r):e.set(1,0,0,0),t.setXYZW(n,e.x,e.y,e.z,e.w)}}updateMatrixWorld(e){super.updateMatrixWorld(e),this.bindMode===$c?this.bindMatrixInverse.copy(this.matrixWorld).invert():this.bindMode===Cd?this.bindMatrixInverse.copy(this.bindMatrix).invert():console.warn("THREE.SkinnedMesh: Unrecognized bindMode: "+this.bindMode)}applyBoneTransform(e,t){let n=this.skeleton,i=this.geometry;Ou.fromBufferAttribute(i.attributes.skinIndex,e),Fu.fromBufferAttribute(i.attributes.skinWeight,e),Uu.copy(t).applyMatrix4(this.bindMatrix),t.set(0,0,0);for(let r=0;r<4;r++){let o=Fu.getComponent(r);if(o!==0){let a=Ou.getComponent(r);Bu.multiplyMatrices(n.bones[a].matrixWorld,n.boneInverses[a]),t.addScaledVector(Zp.copy(Uu).applyMatrix4(Bu),o)}}return t.applyMatrix4(this.bindMatrixInverse)}},$s=class extends pt{constructor(){super(),this.isBone=!0,this.type="Bone"}},zr=class extends At{constructor(e=null,t=1,n=1,i,r,o,a,c,l=Ct,h=Ct,u,d){super(null,o,a,c,l,h,i,r,u,d),this.isDataTexture=!0,this.image={data:e,width:t,height:n},this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}},zu=new De,Jp=new De,Hr=class s{constructor(e=[],t=[]){this.uuid=On(),this.bones=e.slice(0),this.boneInverses=t,this.boneMatrices=null,this.boneTexture=null,this.init()}init(){let e=this.bones,t=this.boneInverses;if(this.boneMatrices=new Float32Array(e.length*16),t.length===0)this.calculateInverses();else if(e.length!==t.length){console.warn("THREE.Skeleton: Number of inverse bone matrices does not match amount of bones."),this.boneInverses=[];for(let n=0,i=this.bones.length;n<i;n++)this.boneInverses.push(new De)}}calculateInverses(){this.boneInverses.length=0;for(let e=0,t=this.bones.length;e<t;e++){let n=new De;this.bones[e]&&n.copy(this.bones[e].matrixWorld).invert(),this.boneInverses.push(n)}}pose(){for(let e=0,t=this.bones.length;e<t;e++){let n=this.bones[e];n&&n.matrixWorld.copy(this.boneInverses[e]).invert()}for(let e=0,t=this.bones.length;e<t;e++){let n=this.bones[e];n&&(n.parent&&n.parent.isBone?(n.matrix.copy(n.parent.matrixWorld).invert(),n.matrix.multiply(n.matrixWorld)):n.matrix.copy(n.matrixWorld),n.matrix.decompose(n.position,n.quaternion,n.scale))}}update(){let e=this.bones,t=this.boneInverses,n=this.boneMatrices,i=this.boneTexture;for(let r=0,o=e.length;r<o;r++){let a=e[r]?e[r].matrixWorld:Jp;zu.multiplyMatrices(a,t[r]),zu.toArray(n,r*16)}i!==null&&(i.needsUpdate=!0)}clone(){return new s(this.bones,this.boneInverses)}computeBoneTexture(){let e=Math.sqrt(this.bones.length*4);e=Math.ceil(e/4)*4,e=Math.max(e,4);let t=new Float32Array(e*e*4);t.set(this.boneMatrices);let n=new zr(t,e,e,tn,wn);return n.needsUpdate=!0,this.boneMatrices=t,this.boneTexture=n,this}getBoneByName(e){for(let t=0,n=this.bones.length;t<n;t++){let i=this.bones[t];if(i.name===e)return i}}dispose(){this.boneTexture!==null&&(this.boneTexture.dispose(),this.boneTexture=null)}fromJSON(e,t){this.uuid=e.uuid;for(let n=0,i=e.bones.length;n<i;n++){let r=e.bones[n],o=t[r];o===void 0&&(console.warn("THREE.Skeleton: No bone found with UUID:",r),o=new $s),this.bones.push(o),this.boneInverses.push(new De().fromArray(e.boneInverses[n]))}return this.init(),this}toJSON(){let e={metadata:{version:4.7,type:"Skeleton",generator:"Skeleton.toJSON"},bones:[],boneInverses:[]};e.uuid=this.uuid;let t=this.bones,n=this.boneInverses;for(let i=0,r=t.length;i<r;i++){let o=t[i];e.bones.push(o.uuid);let a=n[i];e.boneInverses.push(a.toArray())}return e}},ki=class extends at{constructor(e,t,n,i=1){super(e,t,n),this.isInstancedBufferAttribute=!0,this.meshPerAttribute=i}copy(e){return super.copy(e),this.meshPerAttribute=e.meshPerAttribute,this}toJSON(){let e=super.toJSON();return e.meshPerAttribute=this.meshPerAttribute,e.isInstancedBufferAttribute=!0,e}},Hs=new De,Hu=new De,Ho=[],Vu=new Rt,$p=new De,Tr=new lt,wr=new sn,Vr=class extends lt{constructor(e,t,n){super(e,t),this.isInstancedMesh=!0,this.instanceMatrix=new ki(new Float32Array(n*16),16),this.instanceColor=null,this.morphTexture=null,this.count=n,this.boundingBox=null,this.boundingSphere=null;for(let i=0;i<n;i++)this.setMatrixAt(i,$p)}computeBoundingBox(){let e=this.geometry,t=this.count;this.boundingBox===null&&(this.boundingBox=new Rt),e.boundingBox===null&&e.computeBoundingBox(),this.boundingBox.makeEmpty();for(let n=0;n<t;n++)this.getMatrixAt(n,Hs),Vu.copy(e.boundingBox).applyMatrix4(Hs),this.boundingBox.union(Vu)}computeBoundingSphere(){let e=this.geometry,t=this.count;this.boundingSphere===null&&(this.boundingSphere=new sn),e.boundingSphere===null&&e.computeBoundingSphere(),this.boundingSphere.makeEmpty();for(let n=0;n<t;n++)this.getMatrixAt(n,Hs),wr.copy(e.boundingSphere).applyMatrix4(Hs),this.boundingSphere.union(wr)}copy(e,t){return super.copy(e,t),this.instanceMatrix.copy(e.instanceMatrix),e.morphTexture!==null&&(this.morphTexture=e.morphTexture.clone()),e.instanceColor!==null&&(this.instanceColor=e.instanceColor.clone()),this.count=e.count,e.boundingBox!==null&&(this.boundingBox=e.boundingBox.clone()),e.boundingSphere!==null&&(this.boundingSphere=e.boundingSphere.clone()),this}getColorAt(e,t){t.fromArray(this.instanceColor.array,e*3)}getMatrixAt(e,t){t.fromArray(this.instanceMatrix.array,e*16)}getMorphAt(e,t){let n=t.morphTargetInfluences,i=this.morphTexture.source.data.data,r=n.length+1,o=e*r+1;for(let a=0;a<n.length;a++)n[a]=i[o+a]}raycast(e,t){let n=this.matrixWorld,i=this.count;if(Tr.geometry=this.geometry,Tr.material=this.material,Tr.material!==void 0&&(this.boundingSphere===null&&this.computeBoundingSphere(),wr.copy(this.boundingSphere),wr.applyMatrix4(n),e.ray.intersectsSphere(wr)!==!1))for(let r=0;r<i;r++){this.getMatrixAt(r,Hs),Hu.multiplyMatrices(n,Hs),Tr.matrixWorld=Hu,Tr.raycast(e,Ho);for(let o=0,a=Ho.length;o<a;o++){let c=Ho[o];c.instanceId=r,c.object=this,t.push(c)}Ho.length=0}}setColorAt(e,t){this.instanceColor===null&&(this.instanceColor=new ki(new Float32Array(this.instanceMatrix.count*3).fill(1),3)),t.toArray(this.instanceColor.array,e*3)}setMatrixAt(e,t){t.toArray(this.instanceMatrix.array,e*16)}setMorphAt(e,t){let n=t.morphTargetInfluences,i=n.length+1;this.morphTexture===null&&(this.morphTexture=new zr(new Float32Array(i*this.count),i,this.count,Aa,wn));let r=this.morphTexture.source.data.data,o=0;for(let l=0;l<n.length;l++)o+=n[l];let a=this.geometry.morphTargetsRelative?1:1-o,c=i*e;r[c]=a,r.set(n,c+1)}updateMorphTargets(){}dispose(){this.dispatchEvent({type:"dispose"}),this.morphTexture!==null&&(this.morphTexture.dispose(),this.morphTexture=null)}},Yc=new A,Qp=new A,em=new Ve,Sn=class{constructor(e=new A(1,0,0),t=0){this.isPlane=!0,this.normal=e,this.constant=t}set(e,t){return this.normal.copy(e),this.constant=t,this}setComponents(e,t,n,i){return this.normal.set(e,t,n),this.constant=i,this}setFromNormalAndCoplanarPoint(e,t){return this.normal.copy(e),this.constant=-t.dot(this.normal),this}setFromCoplanarPoints(e,t,n){let i=Yc.subVectors(n,t).cross(Qp.subVectors(e,t)).normalize();return this.setFromNormalAndCoplanarPoint(i,e),this}copy(e){return this.normal.copy(e.normal),this.constant=e.constant,this}normalize(){let e=1/this.normal.length();return this.normal.multiplyScalar(e),this.constant*=e,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(e){return this.normal.dot(e)+this.constant}distanceToSphere(e){return this.distanceToPoint(e.center)-e.radius}projectPoint(e,t){return t.copy(e).addScaledVector(this.normal,-this.distanceToPoint(e))}intersectLine(e,t){let n=e.delta(Yc),i=this.normal.dot(n);if(i===0)return this.distanceToPoint(e.start)===0?t.copy(e.start):null;let r=-(e.start.dot(this.normal)+this.constant)/i;return r<0||r>1?null:t.copy(e.start).addScaledVector(n,r)}intersectsLine(e){let t=this.distanceToPoint(e.start),n=this.distanceToPoint(e.end);return t<0&&n>0||n<0&&t>0}intersectsBox(e){return e.intersectsPlane(this)}intersectsSphere(e){return e.intersectsPlane(this)}coplanarPoint(e){return e.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(e,t){let n=t||em.getNormalMatrix(e),i=this.coplanarPoint(Yc).applyMatrix4(e),r=this.normal.applyMatrix3(n).normalize();return this.constant=-i.dot(r),this}translate(e){return this.constant-=e.dot(this.normal),this}equals(e){return e.normal.equals(this.normal)&&e.constant===this.constant}clone(){return new this.constructor().copy(this)}},es=new sn,tm=new Ee(.5,.5),Vo=new A,Qs=class{constructor(e=new Sn,t=new Sn,n=new Sn,i=new Sn,r=new Sn,o=new Sn){this.planes=[e,t,n,i,r,o]}set(e,t,n,i,r,o){let a=this.planes;return a[0].copy(e),a[1].copy(t),a[2].copy(n),a[3].copy(i),a[4].copy(r),a[5].copy(o),this}copy(e){let t=this.planes;for(let n=0;n<6;n++)t[n].copy(e.planes[n]);return this}setFromProjectionMatrix(e,t=Un,n=!1){let i=this.planes,r=e.elements,o=r[0],a=r[1],c=r[2],l=r[3],h=r[4],u=r[5],d=r[6],f=r[7],g=r[8],_=r[9],m=r[10],p=r[11],S=r[12],E=r[13],y=r[14],w=r[15];if(i[0].setComponents(l-o,f-h,p-g,w-S).normalize(),i[1].setComponents(l+o,f+h,p+g,w+S).normalize(),i[2].setComponents(l+a,f+u,p+_,w+E).normalize(),i[3].setComponents(l-a,f-u,p-_,w-E).normalize(),n)i[4].setComponents(c,d,m,y).normalize(),i[5].setComponents(l-c,f-d,p-m,w-y).normalize();else if(i[4].setComponents(l-c,f-d,p-m,w-y).normalize(),t===Un)i[5].setComponents(l+c,f+d,p+m,w+y).normalize();else if(t===Lr)i[5].setComponents(c,d,m,y).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+t);return this}intersectsObject(e){if(e.boundingSphere!==void 0)e.boundingSphere===null&&e.computeBoundingSphere(),es.copy(e.boundingSphere).applyMatrix4(e.matrixWorld);else{let t=e.geometry;t.boundingSphere===null&&t.computeBoundingSphere(),es.copy(t.boundingSphere).applyMatrix4(e.matrixWorld)}return this.intersectsSphere(es)}intersectsSprite(e){es.center.set(0,0,0);let t=tm.distanceTo(e.center);return es.radius=.7071067811865476+t,es.applyMatrix4(e.matrixWorld),this.intersectsSphere(es)}intersectsSphere(e){let t=this.planes,n=e.center,i=-e.radius;for(let r=0;r<6;r++)if(t[r].distanceToPoint(n)<i)return!1;return!0}intersectsBox(e){let t=this.planes;for(let n=0;n<6;n++){let i=t[n];if(Vo.x=i.normal.x>0?e.max.x:e.min.x,Vo.y=i.normal.y>0?e.max.y:e.min.y,Vo.z=i.normal.z>0?e.max.z:e.min.z,i.distanceToPoint(Vo)<0)return!1}return!0}containsPoint(e){let t=this.planes;for(let n=0;n<6;n++)if(t[n].distanceToPoint(e)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}};var Tn=class extends Yt{constructor(e){super(),this.isLineBasicMaterial=!0,this.type="LineBasicMaterial",this.color=new Re(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.linewidth=e.linewidth,this.linecap=e.linecap,this.linejoin=e.linejoin,this.fog=e.fog,this}},na=new A,ia=new A,Gu=new De,Ar=new Jn,Go=new sn,jc=new A,Wu=new A,gi=class extends pt{constructor(e=new xt,t=new Tn){super(),this.isLine=!0,this.type="Line",this.geometry=e,this.material=t,this.morphTargetDictionary=void 0,this.morphTargetInfluences=void 0,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}computeLineDistances(){let e=this.geometry;if(e.index===null){let t=e.attributes.position,n=[0];for(let i=1,r=t.count;i<r;i++)na.fromBufferAttribute(t,i-1),ia.fromBufferAttribute(t,i),n[i]=n[i-1],n[i]+=na.distanceTo(ia);e.setAttribute("lineDistance",new bt(n,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(e,t){let n=this.geometry,i=this.matrixWorld,r=e.params.Line.threshold,o=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),Go.copy(n.boundingSphere),Go.applyMatrix4(i),Go.radius+=r,e.ray.intersectsSphere(Go)===!1)return;Gu.copy(i).invert(),Ar.copy(e.ray).applyMatrix4(Gu);let a=r/((this.scale.x+this.scale.y+this.scale.z)/3),c=a*a,l=this.isLineSegments?2:1,h=n.index,d=n.attributes.position;if(h!==null){let f=Math.max(0,o.start),g=Math.min(h.count,o.start+o.count);for(let _=f,m=g-1;_<m;_+=l){let p=h.getX(_),S=h.getX(_+1),E=Wo(this,e,Ar,c,p,S,_);E&&t.push(E)}if(this.isLineLoop){let _=h.getX(g-1),m=h.getX(f),p=Wo(this,e,Ar,c,_,m,g-1);p&&t.push(p)}}else{let f=Math.max(0,o.start),g=Math.min(d.count,o.start+o.count);for(let _=f,m=g-1;_<m;_+=l){let p=Wo(this,e,Ar,c,_,_+1,_);p&&t.push(p)}if(this.isLineLoop){let _=Wo(this,e,Ar,c,g-1,f,g-1);_&&t.push(_)}}}updateMorphTargets(){let t=this.geometry.morphAttributes,n=Object.keys(t);if(n.length>0){let i=t[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,o=i.length;r<o;r++){let a=i[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=r}}}}};function Wo(s,e,t,n,i,r,o){let a=s.geometry.attributes.position;if(na.fromBufferAttribute(a,i),ia.fromBufferAttribute(a,r),t.distanceSqToSegment(na,ia,jc,Wu)>n)return;jc.applyMatrix4(s.matrixWorld);let l=e.ray.origin.distanceTo(jc);if(!(l<e.near||l>e.far))return{distance:l,point:Wu.clone().applyMatrix4(s.matrixWorld),index:o,face:null,faceIndex:null,barycoord:null,object:s}}var Xu=new A,qu=new A,zi=class extends gi{constructor(e,t){super(e,t),this.isLineSegments=!0,this.type="LineSegments"}computeLineDistances(){let e=this.geometry;if(e.index===null){let t=e.attributes.position,n=[];for(let i=0,r=t.count;i<r;i+=2)Xu.fromBufferAttribute(t,i),qu.fromBufferAttribute(t,i+1),n[i]=i===0?0:n[i-1],n[i+1]=n[i]+Xu.distanceTo(qu);e.setAttribute("lineDistance",new bt(n,1))}else console.warn("THREE.LineSegments.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}},os=class extends gi{constructor(e,t){super(e,t),this.isLineLoop=!0,this.type="LineLoop"}},er=class extends Yt{constructor(e){super(),this.isPointsMaterial=!0,this.type="PointsMaterial",this.color=new Re(16777215),this.map=null,this.alphaMap=null,this.size=1,this.sizeAttenuation=!0,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.alphaMap=e.alphaMap,this.size=e.size,this.sizeAttenuation=e.sizeAttenuation,this.fog=e.fog,this}},Yu=new De,il=new Jn,Xo=new sn,qo=new A,Gr=class extends pt{constructor(e=new xt,t=new er){super(),this.isPoints=!0,this.type="Points",this.geometry=e,this.material=t,this.morphTargetDictionary=void 0,this.morphTargetInfluences=void 0,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}raycast(e,t){let n=this.geometry,i=this.matrixWorld,r=e.params.Points.threshold,o=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),Xo.copy(n.boundingSphere),Xo.applyMatrix4(i),Xo.radius+=r,e.ray.intersectsSphere(Xo)===!1)return;Yu.copy(i).invert(),il.copy(e.ray).applyMatrix4(Yu);let a=r/((this.scale.x+this.scale.y+this.scale.z)/3),c=a*a,l=n.index,u=n.attributes.position;if(l!==null){let d=Math.max(0,o.start),f=Math.min(l.count,o.start+o.count);for(let g=d,_=f;g<_;g++){let m=l.getX(g);qo.fromBufferAttribute(u,m),ju(qo,m,c,i,e,t,this)}}else{let d=Math.max(0,o.start),f=Math.min(u.count,o.start+o.count);for(let g=d,_=f;g<_;g++)qo.fromBufferAttribute(u,g),ju(qo,g,c,i,e,t,this)}}updateMorphTargets(){let t=this.geometry.morphAttributes,n=Object.keys(t);if(n.length>0){let i=t[n[0]];if(i!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,o=i.length;r<o;r++){let a=i[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=r}}}}};function ju(s,e,t,n,i,r,o){let a=il.distanceSqToPoint(s);if(a<t){let c=new A;il.closestPointToPoint(s,c),c.applyMatrix4(n);let l=i.ray.origin.distanceTo(c);if(l<i.near||l>i.far)return;r.push({distance:l,distanceToRay:Math.sqrt(a),point:c,index:e,face:null,faceIndex:null,barycoord:null,object:o})}}var as=class extends At{constructor(e,t,n,i,r,o,a,c,l,h,u,d){super(null,o,a,c,l,h,i,r,u,d),this.isCompressedTexture=!0,this.image={width:t,height:n},this.mipmaps=e,this.flipY=!1,this.generateMipmaps=!1}};var Wr=class extends At{constructor(e,t,n,i,r,o,a,c,l){super(e,t,n,i,r,o,a,c,l),this.isCanvasTexture=!0,this.needsUpdate=!0}},Xr=class extends At{constructor(e,t,n=Xi,i,r,o,a=Ct,c=Ct,l,h=Ws,u=1){if(h!==Ws&&h!==ar)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");let d={width:e,height:t,depth:u};super(d,i,r,o,a,c,h,n,l),this.isDepthTexture=!0,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(e){return super.copy(e),this.source=new Oi(Object.assign({},e.image)),this.compareFunction=e.compareFunction,this}toJSON(e){let t=super.toJSON(e);return this.compareFunction!==null&&(t.compareFunction=this.compareFunction),t}},qr=class extends At{constructor(e=null){super(),this.sourceTexture=e,this.isExternalTexture=!0}copy(e){return super.copy(e),this.sourceTexture=e.sourceTexture,this}};var cs=class s extends xt{constructor(e=1,t=1,n=1,i=1){super(),this.type="PlaneGeometry",this.parameters={width:e,height:t,widthSegments:n,heightSegments:i};let r=e/2,o=t/2,a=Math.floor(n),c=Math.floor(i),l=a+1,h=c+1,u=e/a,d=t/c,f=[],g=[],_=[],m=[];for(let p=0;p<h;p++){let S=p*d-o;for(let E=0;E<l;E++){let y=E*u-r;g.push(y,-S,0),_.push(0,0,1),m.push(E/a),m.push(1-p/c)}}for(let p=0;p<c;p++)for(let S=0;S<a;S++){let E=S+l*p,y=S+l*(p+1),w=S+1+l*(p+1),C=S+1+l*p;f.push(E,y,C),f.push(y,w,C)}this.setIndex(f),this.setAttribute("position",new bt(g,3)),this.setAttribute("normal",new bt(_,3)),this.setAttribute("uv",new bt(m,2))}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new s(e.width,e.height,e.widthSegments,e.heightSegments)}};var Hi=class s extends xt{constructor(e=1,t=32,n=16,i=0,r=Math.PI*2,o=0,a=Math.PI){super(),this.type="SphereGeometry",this.parameters={radius:e,widthSegments:t,heightSegments:n,phiStart:i,phiLength:r,thetaStart:o,thetaLength:a},t=Math.max(3,Math.floor(t)),n=Math.max(2,Math.floor(n));let c=Math.min(o+a,Math.PI),l=0,h=[],u=new A,d=new A,f=[],g=[],_=[],m=[];for(let p=0;p<=n;p++){let S=[],E=p/n,y=0;p===0&&o===0?y=.5/t:p===n&&c===Math.PI&&(y=-.5/t);for(let w=0;w<=t;w++){let C=w/t;u.x=-e*Math.cos(i+C*r)*Math.sin(o+E*a),u.y=e*Math.cos(o+E*a),u.z=e*Math.sin(i+C*r)*Math.sin(o+E*a),g.push(u.x,u.y,u.z),d.copy(u).normalize(),_.push(d.x,d.y,d.z),m.push(C+y,1-E),S.push(l++)}h.push(S)}for(let p=0;p<n;p++)for(let S=0;S<t;S++){let E=h[p][S+1],y=h[p][S],w=h[p+1][S],C=h[p+1][S+1];(p!==0||o>0)&&f.push(E,y,C),(p!==n-1||c<Math.PI)&&f.push(y,w,C)}this.setIndex(f),this.setAttribute("position",new bt(g,3)),this.setAttribute("normal",new bt(_,3)),this.setAttribute("uv",new bt(m,2))}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new s(e.radius,e.widthSegments,e.heightSegments,e.phiStart,e.phiLength,e.thetaStart,e.thetaLength)}};var Yr=class extends Yt{constructor(e){super(),this.isShadowMaterial=!0,this.type="ShadowMaterial",this.color=new Re(0),this.transparent=!0,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.fog=e.fog,this}};var kn=class extends Yt{constructor(e){super(),this.isMeshStandardMaterial=!0,this.type="MeshStandardMaterial",this.defines={STANDARD:""},this.color=new Re(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new Re(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=Tl,this.normalScale=new Ee(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Fn,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.defines={STANDARD:""},this.color.copy(e.color),this.roughness=e.roughness,this.metalness=e.metalness,this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.emissive.copy(e.emissive),this.emissiveMap=e.emissiveMap,this.emissiveIntensity=e.emissiveIntensity,this.bumpMap=e.bumpMap,this.bumpScale=e.bumpScale,this.normalMap=e.normalMap,this.normalMapType=e.normalMapType,this.normalScale.copy(e.normalScale),this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.roughnessMap=e.roughnessMap,this.metalnessMap=e.metalnessMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.envMapIntensity=e.envMapIntensity,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.flatShading=e.flatShading,this.fog=e.fog,this}},rn=class extends kn{constructor(e){super(),this.isMeshPhysicalMaterial=!0,this.defines={STANDARD:"",PHYSICAL:""},this.type="MeshPhysicalMaterial",this.anisotropyRotation=0,this.anisotropyMap=null,this.clearcoatMap=null,this.clearcoatRoughness=0,this.clearcoatRoughnessMap=null,this.clearcoatNormalScale=new Ee(1,1),this.clearcoatNormalMap=null,this.ior=1.5,Object.defineProperty(this,"reflectivity",{get:function(){return qe(2.5*(this.ior-1)/(this.ior+1),0,1)},set:function(t){this.ior=(1+.4*t)/(1-.4*t)}}),this.iridescenceMap=null,this.iridescenceIOR=1.3,this.iridescenceThicknessRange=[100,400],this.iridescenceThicknessMap=null,this.sheenColor=new Re(0),this.sheenColorMap=null,this.sheenRoughness=1,this.sheenRoughnessMap=null,this.transmissionMap=null,this.thickness=0,this.thicknessMap=null,this.attenuationDistance=1/0,this.attenuationColor=new Re(1,1,1),this.specularIntensity=1,this.specularIntensityMap=null,this.specularColor=new Re(1,1,1),this.specularColorMap=null,this._anisotropy=0,this._clearcoat=0,this._dispersion=0,this._iridescence=0,this._sheen=0,this._transmission=0,this.setValues(e)}get anisotropy(){return this._anisotropy}set anisotropy(e){this._anisotropy>0!=e>0&&this.version++,this._anisotropy=e}get clearcoat(){return this._clearcoat}set clearcoat(e){this._clearcoat>0!=e>0&&this.version++,this._clearcoat=e}get iridescence(){return this._iridescence}set iridescence(e){this._iridescence>0!=e>0&&this.version++,this._iridescence=e}get dispersion(){return this._dispersion}set dispersion(e){this._dispersion>0!=e>0&&this.version++,this._dispersion=e}get sheen(){return this._sheen}set sheen(e){this._sheen>0!=e>0&&this.version++,this._sheen=e}get transmission(){return this._transmission}set transmission(e){this._transmission>0!=e>0&&this.version++,this._transmission=e}copy(e){return super.copy(e),this.defines={STANDARD:"",PHYSICAL:""},this.anisotropy=e.anisotropy,this.anisotropyRotation=e.anisotropyRotation,this.anisotropyMap=e.anisotropyMap,this.clearcoat=e.clearcoat,this.clearcoatMap=e.clearcoatMap,this.clearcoatRoughness=e.clearcoatRoughness,this.clearcoatRoughnessMap=e.clearcoatRoughnessMap,this.clearcoatNormalMap=e.clearcoatNormalMap,this.clearcoatNormalScale.copy(e.clearcoatNormalScale),this.dispersion=e.dispersion,this.ior=e.ior,this.iridescence=e.iridescence,this.iridescenceMap=e.iridescenceMap,this.iridescenceIOR=e.iridescenceIOR,this.iridescenceThicknessRange=[...e.iridescenceThicknessRange],this.iridescenceThicknessMap=e.iridescenceThicknessMap,this.sheen=e.sheen,this.sheenColor.copy(e.sheenColor),this.sheenColorMap=e.sheenColorMap,this.sheenRoughness=e.sheenRoughness,this.sheenRoughnessMap=e.sheenRoughnessMap,this.transmission=e.transmission,this.transmissionMap=e.transmissionMap,this.thickness=e.thickness,this.thicknessMap=e.thicknessMap,this.attenuationDistance=e.attenuationDistance,this.attenuationColor.copy(e.attenuationColor),this.specularIntensity=e.specularIntensity,this.specularIntensityMap=e.specularIntensityMap,this.specularColor.copy(e.specularColor),this.specularColorMap=e.specularColorMap,this}};var sa=class extends Yt{constructor(e){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=Pd,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(e)}copy(e){return super.copy(e),this.depthPacking=e.depthPacking,this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this}},ra=class extends Yt{constructor(e){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(e)}copy(e){return super.copy(e),this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this}};function Yo(s,e){return!s||s.constructor===e?s:typeof e.BYTES_PER_ELEMENT=="number"?new e(s):Array.prototype.slice.call(s)}function nm(s){return ArrayBuffer.isView(s)&&!(s instanceof DataView)}function im(s){function e(i,r){return s[i]-s[r]}let t=s.length,n=new Array(t);for(let i=0;i!==t;++i)n[i]=i;return n.sort(e),n}function Ku(s,e,t){let n=s.length,i=new s.constructor(n);for(let r=0,o=0;o!==n;++r){let a=t[r]*e;for(let c=0;c!==e;++c)i[o++]=s[a+c]}return i}function qd(s,e,t,n){let i=1,r=s[0];for(;r!==void 0&&r[n]===void 0;)r=s[i++];if(r===void 0)return;let o=r[n];if(o!==void 0)if(Array.isArray(o))do o=r[n],o!==void 0&&(e.push(r.time),t.push(...o)),r=s[i++];while(r!==void 0);else if(o.toArray!==void 0)do o=r[n],o!==void 0&&(e.push(r.time),o.toArray(t,t.length)),r=s[i++];while(r!==void 0);else do o=r[n],o!==void 0&&(e.push(r.time),t.push(o)),r=s[i++];while(r!==void 0)}var _i=class{constructor(e,t,n,i){this.parameterPositions=e,this._cachedIndex=0,this.resultBuffer=i!==void 0?i:new t.constructor(n),this.sampleValues=t,this.valueSize=n,this.settings=null,this.DefaultSettings_={}}evaluate(e){let t=this.parameterPositions,n=this._cachedIndex,i=t[n],r=t[n-1];n:{e:{let o;t:{i:if(!(e<i)){for(let a=n+2;;){if(i===void 0){if(e<r)break i;return n=t.length,this._cachedIndex=n,this.copySampleValue_(n-1)}if(n===a)break;if(r=i,i=t[++n],e<i)break e}o=t.length;break t}if(!(e>=r)){let a=t[1];e<a&&(n=2,r=a);for(let c=n-2;;){if(r===void 0)return this._cachedIndex=0,this.copySampleValue_(0);if(n===c)break;if(i=r,r=t[--n-1],e>=r)break e}o=n,n=0;break t}break n}for(;n<o;){let a=n+o>>>1;e<t[a]?o=a:n=a+1}if(i=t[n],r=t[n-1],r===void 0)return this._cachedIndex=0,this.copySampleValue_(0);if(i===void 0)return n=t.length,this._cachedIndex=n,this.copySampleValue_(n-1)}this._cachedIndex=n,this.intervalChanged_(n,r,i)}return this.interpolate_(n,r,e,i)}getSettings_(){return this.settings||this.DefaultSettings_}copySampleValue_(e){let t=this.resultBuffer,n=this.sampleValues,i=this.valueSize,r=e*i;for(let o=0;o!==i;++o)t[o]=n[r+o];return t}interpolate_(){throw new Error("call to abstract method")}intervalChanged_(){}},oa=class extends _i{constructor(e,t,n,i){super(e,t,n,i),this._weightPrev=-0,this._offsetPrev=-0,this._weightNext=-0,this._offsetNext=-0,this.DefaultSettings_={endingStart:Qc,endingEnd:Qc}}intervalChanged_(e,t,n){let i=this.parameterPositions,r=e-2,o=e+1,a=i[r],c=i[o];if(a===void 0)switch(this.getSettings_().endingStart){case el:r=e,a=2*t-n;break;case tl:r=i.length-2,a=t+i[r]-i[r+1];break;default:r=e,a=n}if(c===void 0)switch(this.getSettings_().endingEnd){case el:o=e,c=2*n-t;break;case tl:o=1,c=n+i[1]-i[0];break;default:o=e-1,c=t}let l=(n-t)*.5,h=this.valueSize;this._weightPrev=l/(t-a),this._weightNext=l/(c-n),this._offsetPrev=r*h,this._offsetNext=o*h}interpolate_(e,t,n,i){let r=this.resultBuffer,o=this.sampleValues,a=this.valueSize,c=e*a,l=c-a,h=this._offsetPrev,u=this._offsetNext,d=this._weightPrev,f=this._weightNext,g=(n-t)/(i-t),_=g*g,m=_*g,p=-d*m+2*d*_-d*g,S=(1+d)*m+(-1.5-2*d)*_+(-.5+d)*g+1,E=(-1-f)*m+(1.5+f)*_+.5*g,y=f*m-f*_;for(let w=0;w!==a;++w)r[w]=p*o[h+w]+S*o[l+w]+E*o[c+w]+y*o[u+w];return r}},aa=class extends _i{constructor(e,t,n,i){super(e,t,n,i)}interpolate_(e,t,n,i){let r=this.resultBuffer,o=this.sampleValues,a=this.valueSize,c=e*a,l=c-a,h=(n-t)/(i-t),u=1-h;for(let d=0;d!==a;++d)r[d]=o[l+d]*u+o[c+d]*h;return r}},ca=class extends _i{constructor(e,t,n,i){super(e,t,n,i)}interpolate_(e){return this.copySampleValue_(e-1)}},on=class{constructor(e,t,n,i){if(e===void 0)throw new Error("THREE.KeyframeTrack: track name is undefined");if(t===void 0||t.length===0)throw new Error("THREE.KeyframeTrack: no keyframes in track named "+e);this.name=e,this.times=Yo(t,this.TimeBufferType),this.values=Yo(n,this.ValueBufferType),this.setInterpolation(i||this.DefaultInterpolation)}static toJSON(e){let t=e.constructor,n;if(t.toJSON!==this.toJSON)n=t.toJSON(e);else{n={name:e.name,times:Yo(e.times,Array),values:Yo(e.values,Array)};let i=e.getInterpolation();i!==e.DefaultInterpolation&&(n.interpolation=i)}return n.type=e.ValueTypeName,n}InterpolantFactoryMethodDiscrete(e){return new ca(this.times,this.values,this.getValueSize(),e)}InterpolantFactoryMethodLinear(e){return new aa(this.times,this.values,this.getValueSize(),e)}InterpolantFactoryMethodSmooth(e){return new oa(this.times,this.values,this.getValueSize(),e)}setInterpolation(e){let t;switch(e){case pi:t=this.InterpolantFactoryMethodDiscrete;break;case mi:t=this.InterpolantFactoryMethodLinear;break;case jo:t=this.InterpolantFactoryMethodSmooth;break}if(t===void 0){let n="unsupported interpolation for "+this.ValueTypeName+" keyframe track named "+this.name;if(this.createInterpolant===void 0)if(e!==this.DefaultInterpolation)this.setInterpolation(this.DefaultInterpolation);else throw new Error(n);return console.warn("THREE.KeyframeTrack:",n),this}return this.createInterpolant=t,this}getInterpolation(){switch(this.createInterpolant){case this.InterpolantFactoryMethodDiscrete:return pi;case this.InterpolantFactoryMethodLinear:return mi;case this.InterpolantFactoryMethodSmooth:return jo}}getValueSize(){return this.values.length/this.times.length}shift(e){if(e!==0){let t=this.times;for(let n=0,i=t.length;n!==i;++n)t[n]+=e}return this}scale(e){if(e!==1){let t=this.times;for(let n=0,i=t.length;n!==i;++n)t[n]*=e}return this}trim(e,t){let n=this.times,i=n.length,r=0,o=i-1;for(;r!==i&&n[r]<e;)++r;for(;o!==-1&&n[o]>t;)--o;if(++o,r!==0||o!==i){r>=o&&(o=Math.max(o,1),r=o-1);let a=this.getValueSize();this.times=n.slice(r,o),this.values=this.values.slice(r*a,o*a)}return this}validate(){let e=!0,t=this.getValueSize();t-Math.floor(t)!==0&&(console.error("THREE.KeyframeTrack: Invalid value size in track.",this),e=!1);let n=this.times,i=this.values,r=n.length;r===0&&(console.error("THREE.KeyframeTrack: Track is empty.",this),e=!1);let o=null;for(let a=0;a!==r;a++){let c=n[a];if(typeof c=="number"&&isNaN(c)){console.error("THREE.KeyframeTrack: Time is not a valid number.",this,a,c),e=!1;break}if(o!==null&&o>c){console.error("THREE.KeyframeTrack: Out of order keys.",this,a,c,o),e=!1;break}o=c}if(i!==void 0&&nm(i))for(let a=0,c=i.length;a!==c;++a){let l=i[a];if(isNaN(l)){console.error("THREE.KeyframeTrack: Value is not a valid number.",this,a,l),e=!1;break}}return e}optimize(){let e=this.times.slice(),t=this.values.slice(),n=this.getValueSize(),i=this.getInterpolation()===jo,r=e.length-1,o=1;for(let a=1;a<r;++a){let c=!1,l=e[a],h=e[a+1];if(l!==h&&(a!==1||l!==e[0]))if(i)c=!0;else{let u=a*n,d=u-n,f=u+n;for(let g=0;g!==n;++g){let _=t[u+g];if(_!==t[d+g]||_!==t[f+g]){c=!0;break}}}if(c){if(a!==o){e[o]=e[a];let u=a*n,d=o*n;for(let f=0;f!==n;++f)t[d+f]=t[u+f]}++o}}if(r>0){e[o]=e[r];for(let a=r*n,c=o*n,l=0;l!==n;++l)t[c+l]=t[a+l];++o}return o!==e.length?(this.times=e.slice(0,o),this.values=t.slice(0,o*n)):(this.times=e,this.values=t),this}clone(){let e=this.times.slice(),t=this.values.slice(),n=this.constructor,i=new n(this.name,e,t);return i.createInterpolant=this.createInterpolant,i}};on.prototype.ValueTypeName="";on.prototype.TimeBufferType=Float32Array;on.prototype.ValueBufferType=Float32Array;on.prototype.DefaultInterpolation=mi;var xi=class extends on{constructor(e,t,n){super(e,t,n)}};xi.prototype.ValueTypeName="bool";xi.prototype.ValueBufferType=Array;xi.prototype.DefaultInterpolation=pi;xi.prototype.InterpolantFactoryMethodLinear=void 0;xi.prototype.InterpolantFactoryMethodSmooth=void 0;var jr=class extends on{constructor(e,t,n,i){super(e,t,n,i)}};jr.prototype.ValueTypeName="color";var $n=class extends on{constructor(e,t,n,i){super(e,t,n,i)}};$n.prototype.ValueTypeName="number";var la=class extends _i{constructor(e,t,n,i){super(e,t,n,i)}interpolate_(e,t,n,i){let r=this.resultBuffer,o=this.sampleValues,a=this.valueSize,c=(n-t)/(i-t),l=e*a;for(let h=l+a;l!==h;l+=4)It.slerpFlat(r,0,o,l-a,o,l,c);return r}},Qn=class extends on{constructor(e,t,n,i){super(e,t,n,i)}InterpolantFactoryMethodLinear(e){return new la(this.times,this.values,this.getValueSize(),e)}};Qn.prototype.ValueTypeName="quaternion";Qn.prototype.InterpolantFactoryMethodSmooth=void 0;var yi=class extends on{constructor(e,t,n){super(e,t,n)}};yi.prototype.ValueTypeName="string";yi.prototype.ValueBufferType=Array;yi.prototype.DefaultInterpolation=pi;yi.prototype.InterpolantFactoryMethodLinear=void 0;yi.prototype.InterpolantFactoryMethodSmooth=void 0;var ei=class extends on{constructor(e,t,n,i){super(e,t,n,i)}};ei.prototype.ValueTypeName="vector";var Kr=class{constructor(e="",t=-1,n=[],i=Id){this.name=e,this.tracks=n,this.duration=t,this.blendMode=i,this.uuid=On(),this.userData={},this.duration<0&&this.resetDuration()}static parse(e){let t=[],n=e.tracks,i=1/(e.fps||1);for(let o=0,a=n.length;o!==a;++o)t.push(rm(n[o]).scale(i));let r=new this(e.name,e.duration,t,e.blendMode);return r.uuid=e.uuid,r.userData=JSON.parse(e.userData||"{}"),r}static toJSON(e){let t=[],n=e.tracks,i={name:e.name,duration:e.duration,tracks:t,uuid:e.uuid,blendMode:e.blendMode,userData:JSON.stringify(e.userData)};for(let r=0,o=n.length;r!==o;++r)t.push(on.toJSON(n[r]));return i}static CreateFromMorphTargetSequence(e,t,n,i){let r=t.length,o=[];for(let a=0;a<r;a++){let c=[],l=[];c.push((a+r-1)%r,a,(a+1)%r),l.push(0,1,0);let h=im(c);c=Ku(c,1,h),l=Ku(l,1,h),!i&&c[0]===0&&(c.push(r),l.push(l[0])),o.push(new $n(".morphTargetInfluences["+t[a].name+"]",c,l).scale(1/n))}return new this(e,-1,o)}static findByName(e,t){let n=e;if(!Array.isArray(e)){let i=e;n=i.geometry&&i.geometry.animations||i.animations}for(let i=0;i<n.length;i++)if(n[i].name===t)return n[i];return null}static CreateClipsFromMorphTargetSequences(e,t,n){let i={},r=/^([\w-]*?)([\d]+)$/;for(let a=0,c=e.length;a<c;a++){let l=e[a],h=l.name.match(r);if(h&&h.length>1){let u=h[1],d=i[u];d||(i[u]=d=[]),d.push(l)}}let o=[];for(let a in i)o.push(this.CreateFromMorphTargetSequence(a,i[a],t,n));return o}static parseAnimation(e,t){if(console.warn("THREE.AnimationClip: parseAnimation() is deprecated and will be removed with r185"),!e)return console.error("THREE.AnimationClip: No animation in JSONLoader data."),null;let n=function(u,d,f,g,_){if(f.length!==0){let m=[],p=[];qd(f,m,p,g),m.length!==0&&_.push(new u(d,m,p))}},i=[],r=e.name||"default",o=e.fps||30,a=e.blendMode,c=e.length||-1,l=e.hierarchy||[];for(let u=0;u<l.length;u++){let d=l[u].keys;if(!(!d||d.length===0))if(d[0].morphTargets){let f={},g;for(g=0;g<d.length;g++)if(d[g].morphTargets)for(let _=0;_<d[g].morphTargets.length;_++)f[d[g].morphTargets[_]]=-1;for(let _ in f){let m=[],p=[];for(let S=0;S!==d[g].morphTargets.length;++S){let E=d[g];m.push(E.time),p.push(E.morphTarget===_?1:0)}i.push(new $n(".morphTargetInfluence["+_+"]",m,p))}c=f.length*o}else{let f=".bones["+t[u].name+"]";n(ei,f+".position",d,"pos",i),n(Qn,f+".quaternion",d,"rot",i),n(ei,f+".scale",d,"scl",i)}}return i.length===0?null:new this(r,c,i,a)}resetDuration(){let e=this.tracks,t=0;for(let n=0,i=e.length;n!==i;++n){let r=this.tracks[n];t=Math.max(t,r.times[r.times.length-1])}return this.duration=t,this}trim(){for(let e=0;e<this.tracks.length;e++)this.tracks[e].trim(0,this.duration);return this}validate(){let e=!0;for(let t=0;t<this.tracks.length;t++)e=e&&this.tracks[t].validate();return e}optimize(){for(let e=0;e<this.tracks.length;e++)this.tracks[e].optimize();return this}clone(){let e=[];for(let n=0;n<this.tracks.length;n++)e.push(this.tracks[n].clone());let t=new this.constructor(this.name,this.duration,e,this.blendMode);return t.userData=JSON.parse(JSON.stringify(this.userData)),t}toJSON(){return this.constructor.toJSON(this)}};function sm(s){switch(s.toLowerCase()){case"scalar":case"double":case"float":case"number":case"integer":return $n;case"vector":case"vector2":case"vector3":case"vector4":return ei;case"color":return jr;case"quaternion":return Qn;case"bool":case"boolean":return xi;case"string":return yi}throw new Error("THREE.KeyframeTrack: Unsupported typeName: "+s)}function rm(s){if(s.type===void 0)throw new Error("THREE.KeyframeTrack: track type undefined, can not parse");let e=sm(s.type);if(s.times===void 0){let t=[],n=[];qd(s.keys,t,n,"value"),s.times=t,s.values=n}return e.parse!==void 0?e.parse(s):new e(s.name,s.times,s.values,s.interpolation)}var Yn={enabled:!1,files:{},add:function(s,e){this.enabled!==!1&&(this.files[s]=e)},get:function(s){if(this.enabled!==!1)return this.files[s]},remove:function(s){delete this.files[s]},clear:function(){this.files={}}},ha=class{constructor(e,t,n){let i=this,r=!1,o=0,a=0,c,l=[];this.onStart=void 0,this.onLoad=e,this.onProgress=t,this.onError=n,this.abortController=new AbortController,this.itemStart=function(h){a++,r===!1&&i.onStart!==void 0&&i.onStart(h,o,a),r=!0},this.itemEnd=function(h){o++,i.onProgress!==void 0&&i.onProgress(h,o,a),o===a&&(r=!1,i.onLoad!==void 0&&i.onLoad())},this.itemError=function(h){i.onError!==void 0&&i.onError(h)},this.resolveURL=function(h){return c?c(h):h},this.setURLModifier=function(h){return c=h,this},this.addHandler=function(h,u){return l.push(h,u),this},this.removeHandler=function(h){let u=l.indexOf(h);return u!==-1&&l.splice(u,2),this},this.getHandler=function(h){for(let u=0,d=l.length;u<d;u+=2){let f=l[u],g=l[u+1];if(f.global&&(f.lastIndex=0),f.test(h))return g}return null},this.abort=function(){return this.abortController.abort(),this.abortController=new AbortController,this}}},Yd=new ha,ti=class{constructor(e){this.manager=e!==void 0?e:Yd,this.crossOrigin="anonymous",this.withCredentials=!1,this.path="",this.resourcePath="",this.requestHeader={}}load(){}loadAsync(e,t){let n=this;return new Promise(function(i,r){n.load(e,i,t,r)})}parse(){}setCrossOrigin(e){return this.crossOrigin=e,this}setWithCredentials(e){return this.withCredentials=e,this}setPath(e){return this.path=e,this}setResourcePath(e){return this.resourcePath=e,this}setRequestHeader(e){return this.requestHeader=e,this}abort(){return this}};ti.DEFAULT_MATERIAL_NAME="__DEFAULT";var di={},sl=class extends Error{constructor(e,t){super(e),this.response=t}},tr=class extends ti{constructor(e){super(e),this.mimeType="",this.responseType="",this._abortController=new AbortController}load(e,t,n,i){e===void 0&&(e=""),this.path!==void 0&&(e=this.path+e),e=this.manager.resolveURL(e);let r=Yn.get(`file:${e}`);if(r!==void 0)return this.manager.itemStart(e),setTimeout(()=>{t&&t(r),this.manager.itemEnd(e)},0),r;if(di[e]!==void 0){di[e].push({onLoad:t,onProgress:n,onError:i});return}di[e]=[],di[e].push({onLoad:t,onProgress:n,onError:i});let o=new Request(e,{headers:new Headers(this.requestHeader),credentials:this.withCredentials?"include":"same-origin",signal:typeof AbortSignal.any=="function"?AbortSignal.any([this._abortController.signal,this.manager.abortController.signal]):this._abortController.signal}),a=this.mimeType,c=this.responseType;fetch(o).then(l=>{if(l.status===200||l.status===0){if(l.status===0&&console.warn("THREE.FileLoader: HTTP Status 0 received."),typeof ReadableStream>"u"||l.body===void 0||l.body.getReader===void 0)return l;let h=di[e],u=l.body.getReader(),d=l.headers.get("X-File-Size")||l.headers.get("Content-Length"),f=d?parseInt(d):0,g=f!==0,_=0,m=new ReadableStream({start(p){S();function S(){u.read().then(({done:E,value:y})=>{if(E)p.close();else{_+=y.byteLength;let w=new ProgressEvent("progress",{lengthComputable:g,loaded:_,total:f});for(let C=0,R=h.length;C<R;C++){let N=h[C];N.onProgress&&N.onProgress(w)}p.enqueue(y),S()}},E=>{p.error(E)})}}});return new Response(m)}else throw new sl(`fetch for "${l.url}" responded with ${l.status}: ${l.statusText}`,l)}).then(l=>{switch(c){case"arraybuffer":return l.arrayBuffer();case"blob":return l.blob();case"document":return l.text().then(h=>new DOMParser().parseFromString(h,a));case"json":return l.json();default:if(a==="")return l.text();{let u=/charset="?([^;"\s]*)"?/i.exec(a),d=u&&u[1]?u[1].toLowerCase():void 0,f=new TextDecoder(d);return l.arrayBuffer().then(g=>f.decode(g))}}}).then(l=>{Yn.add(`file:${e}`,l);let h=di[e];delete di[e];for(let u=0,d=h.length;u<d;u++){let f=h[u];f.onLoad&&f.onLoad(l)}}).catch(l=>{let h=di[e];if(h===void 0)throw this.manager.itemError(e),l;delete di[e];for(let u=0,d=h.length;u<d;u++){let f=h[u];f.onError&&f.onError(l)}this.manager.itemError(e)}).finally(()=>{this.manager.itemEnd(e)}),this.manager.itemStart(e)}setResponseType(e){return this.responseType=e,this}setMimeType(e){return this.mimeType=e,this}abort(){return this._abortController.abort(),this._abortController=new AbortController,this}};var Vs=new WeakMap,ua=class extends ti{constructor(e){super(e)}load(e,t,n,i){this.path!==void 0&&(e=this.path+e),e=this.manager.resolveURL(e);let r=this,o=Yn.get(`image:${e}`);if(o!==void 0){if(o.complete===!0)r.manager.itemStart(e),setTimeout(function(){t&&t(o),r.manager.itemEnd(e)},0);else{let u=Vs.get(o);u===void 0&&(u=[],Vs.set(o,u)),u.push({onLoad:t,onError:i})}return o}let a=Xs("img");function c(){h(),t&&t(this);let u=Vs.get(this)||[];for(let d=0;d<u.length;d++){let f=u[d];f.onLoad&&f.onLoad(this)}Vs.delete(this),r.manager.itemEnd(e)}function l(u){h(),i&&i(u),Yn.remove(`image:${e}`);let d=Vs.get(this)||[];for(let f=0;f<d.length;f++){let g=d[f];g.onError&&g.onError(u)}Vs.delete(this),r.manager.itemError(e),r.manager.itemEnd(e)}function h(){a.removeEventListener("load",c,!1),a.removeEventListener("error",l,!1)}return a.addEventListener("load",c,!1),a.addEventListener("error",l,!1),e.slice(0,5)!=="data:"&&this.crossOrigin!==void 0&&(a.crossOrigin=this.crossOrigin),Yn.add(`image:${e}`,a),r.manager.itemStart(e),a.src=e,a}};var Zr=class extends ti{constructor(e){super(e)}load(e,t,n,i){let r=new At,o=new ua(this.manager);return o.setCrossOrigin(this.crossOrigin),o.setPath(this.path),o.load(e,function(a){r.image=a,r.needsUpdate=!0,t!==void 0&&t(r)},n,i),r}},ls=class extends pt{constructor(e,t=1){super(),this.isLight=!0,this.type="Light",this.color=new Re(e),this.intensity=t}dispose(){}copy(e,t){return super.copy(e,t),this.color.copy(e.color),this.intensity=e.intensity,this}toJSON(e){let t=super.toJSON(e);return t.object.color=this.color.getHex(),t.object.intensity=this.intensity,this.groundColor!==void 0&&(t.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(t.object.distance=this.distance),this.angle!==void 0&&(t.object.angle=this.angle),this.decay!==void 0&&(t.object.decay=this.decay),this.penumbra!==void 0&&(t.object.penumbra=this.penumbra),this.shadow!==void 0&&(t.object.shadow=this.shadow.toJSON()),this.target!==void 0&&(t.object.target=this.target.uuid),t}},Jr=class extends ls{constructor(e,t,n){super(e,n),this.isHemisphereLight=!0,this.type="HemisphereLight",this.position.copy(pt.DEFAULT_UP),this.updateMatrix(),this.groundColor=new Re(t)}copy(e,t){return super.copy(e,t),this.groundColor.copy(e.groundColor),this}},Kc=new De,Zu=new A,Ju=new A,$r=class{constructor(e){this.camera=e,this.intensity=1,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new Ee(512,512),this.mapType=zn,this.map=null,this.mapPass=null,this.matrix=new De,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new Qs,this._frameExtents=new Ee(1,1),this._viewportCount=1,this._viewports=[new nt(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(e){let t=this.camera,n=this.matrix;Zu.setFromMatrixPosition(e.matrixWorld),t.position.copy(Zu),Ju.setFromMatrixPosition(e.target.matrixWorld),t.lookAt(Ju),t.updateMatrixWorld(),Kc.multiplyMatrices(t.projectionMatrix,t.matrixWorldInverse),this._frustum.setFromProjectionMatrix(Kc,t.coordinateSystem,t.reversedDepth),t.reversedDepth?n.set(.5,0,0,.5,0,.5,0,.5,0,0,1,0,0,0,0,1):n.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),n.multiply(Kc)}getViewport(e){return this._viewports[e]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(e){return this.camera=e.camera.clone(),this.intensity=e.intensity,this.bias=e.bias,this.radius=e.radius,this.autoUpdate=e.autoUpdate,this.needsUpdate=e.needsUpdate,this.normalBias=e.normalBias,this.blurSamples=e.blurSamples,this.mapSize.copy(e.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){let e={};return this.intensity!==1&&(e.intensity=this.intensity),this.bias!==0&&(e.bias=this.bias),this.normalBias!==0&&(e.normalBias=this.normalBias),this.radius!==1&&(e.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(e.mapSize=this.mapSize.toArray()),e.camera=this.camera.toJSON(!1).object,delete e.camera.matrix,e}},rl=class extends $r{constructor(){super(new wt(50,1,.5,500)),this.isSpotLightShadow=!0,this.focus=1,this.aspect=1}updateMatrices(e){let t=this.camera,n=ss*2*e.angle*this.focus,i=this.mapSize.width/this.mapSize.height*this.aspect,r=e.distance||t.far;(n!==t.fov||i!==t.aspect||r!==t.far)&&(t.fov=n,t.aspect=i,t.far=r,t.updateProjectionMatrix()),super.updateMatrices(e)}copy(e){return super.copy(e),this.focus=e.focus,this}},Qr=class extends ls{constructor(e,t,n=0,i=Math.PI/3,r=0,o=2){super(e,t),this.isSpotLight=!0,this.type="SpotLight",this.position.copy(pt.DEFAULT_UP),this.updateMatrix(),this.target=new pt,this.distance=n,this.angle=i,this.penumbra=r,this.decay=o,this.map=null,this.shadow=new rl}get power(){return this.intensity*Math.PI}set power(e){this.intensity=e/Math.PI}dispose(){this.shadow.dispose()}copy(e,t){return super.copy(e,t),this.distance=e.distance,this.angle=e.angle,this.penumbra=e.penumbra,this.decay=e.decay,this.target=e.target.clone(),this.shadow=e.shadow.clone(),this}},$u=new De,Rr=new A,Zc=new A,ol=class extends $r{constructor(){super(new wt(90,1,.5,500)),this.isPointLightShadow=!0,this._frameExtents=new Ee(4,2),this._viewportCount=6,this._viewports=[new nt(2,1,1,1),new nt(0,1,1,1),new nt(3,1,1,1),new nt(1,1,1,1),new nt(3,0,1,1),new nt(1,0,1,1)],this._cubeDirections=[new A(1,0,0),new A(-1,0,0),new A(0,0,1),new A(0,0,-1),new A(0,1,0),new A(0,-1,0)],this._cubeUps=[new A(0,1,0),new A(0,1,0),new A(0,1,0),new A(0,1,0),new A(0,0,1),new A(0,0,-1)]}updateMatrices(e,t=0){let n=this.camera,i=this.matrix,r=e.distance||n.far;r!==n.far&&(n.far=r,n.updateProjectionMatrix()),Rr.setFromMatrixPosition(e.matrixWorld),n.position.copy(Rr),Zc.copy(n.position),Zc.add(this._cubeDirections[t]),n.up.copy(this._cubeUps[t]),n.lookAt(Zc),n.updateMatrixWorld(),i.makeTranslation(-Rr.x,-Rr.y,-Rr.z),$u.multiplyMatrices(n.projectionMatrix,n.matrixWorldInverse),this._frustum.setFromProjectionMatrix($u,n.coordinateSystem,n.reversedDepth)}},eo=class extends ls{constructor(e,t,n=0,i=2){super(e,t),this.isPointLight=!0,this.type="PointLight",this.distance=n,this.decay=i,this.shadow=new ol}get power(){return this.intensity*4*Math.PI}set power(e){this.intensity=e/(4*Math.PI)}dispose(){this.shadow.dispose()}copy(e,t){return super.copy(e,t),this.distance=e.distance,this.decay=e.decay,this.shadow=e.shadow.clone(),this}},hs=class extends Or{constructor(e=-1,t=1,n=1,i=-1,r=.1,o=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=e,this.right=t,this.top=n,this.bottom=i,this.near=r,this.far=o,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.left=e.left,this.right=e.right,this.top=e.top,this.bottom=e.bottom,this.near=e.near,this.far=e.far,this.zoom=e.zoom,this.view=e.view===null?null:Object.assign({},e.view),this}setViewOffset(e,t,n,i,r,o){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=n,this.view.offsetY=i,this.view.width=r,this.view.height=o,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){let e=(this.right-this.left)/(2*this.zoom),t=(this.top-this.bottom)/(2*this.zoom),n=(this.right+this.left)/2,i=(this.top+this.bottom)/2,r=n-e,o=n+e,a=i+t,c=i-t;if(this.view!==null&&this.view.enabled){let l=(this.right-this.left)/this.view.fullWidth/this.zoom,h=(this.top-this.bottom)/this.view.fullHeight/this.zoom;r+=l*this.view.offsetX,o=r+l*this.view.width,a-=h*this.view.offsetY,c=a-h*this.view.height}this.projectionMatrix.makeOrthographic(r,o,a,c,this.near,this.far,this.coordinateSystem,this.reversedDepth),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){let t=super.toJSON(e);return t.object.zoom=this.zoom,t.object.left=this.left,t.object.right=this.right,t.object.top=this.top,t.object.bottom=this.bottom,t.object.near=this.near,t.object.far=this.far,this.view!==null&&(t.object.view=Object.assign({},this.view)),t}},al=class extends $r{constructor(){super(new hs(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}},vi=class extends ls{constructor(e,t){super(e,t),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(pt.DEFAULT_UP),this.updateMatrix(),this.target=new pt,this.shadow=new al}dispose(){this.shadow.dispose()}copy(e){return super.copy(e),this.target=e.target.clone(),this.shadow=e.shadow.clone(),this}};var bi=class{static extractUrlBase(e){let t=e.lastIndexOf("/");return t===-1?"./":e.slice(0,t+1)}static resolveURL(e,t){return typeof e!="string"||e===""?"":(/^https?:\/\//i.test(t)&&/^\//.test(e)&&(t=t.replace(/(^https?:\/\/[^\/]+).*/i,"$1")),/^(https?:)?\/\//i.test(e)||/^data:.*,.*$/i.test(e)||/^blob:.*$/i.test(e)?e:t+e)}};var Jc=new WeakMap,to=class extends ti{constructor(e){super(e),this.isImageBitmapLoader=!0,typeof createImageBitmap>"u"&&console.warn("THREE.ImageBitmapLoader: createImageBitmap() not supported."),typeof fetch>"u"&&console.warn("THREE.ImageBitmapLoader: fetch() not supported."),this.options={premultiplyAlpha:"none"},this._abortController=new AbortController}setOptions(e){return this.options=e,this}load(e,t,n,i){e===void 0&&(e=""),this.path!==void 0&&(e=this.path+e),e=this.manager.resolveURL(e);let r=this,o=Yn.get(`image-bitmap:${e}`);if(o!==void 0){if(r.manager.itemStart(e),o.then){o.then(l=>{if(Jc.has(o)===!0)i&&i(Jc.get(o)),r.manager.itemError(e),r.manager.itemEnd(e);else return t&&t(l),r.manager.itemEnd(e),l});return}return setTimeout(function(){t&&t(o),r.manager.itemEnd(e)},0),o}let a={};a.credentials=this.crossOrigin==="anonymous"?"same-origin":"include",a.headers=this.requestHeader,a.signal=typeof AbortSignal.any=="function"?AbortSignal.any([this._abortController.signal,this.manager.abortController.signal]):this._abortController.signal;let c=fetch(e,a).then(function(l){return l.blob()}).then(function(l){return createImageBitmap(l,Object.assign(r.options,{colorSpaceConversion:"none"}))}).then(function(l){return Yn.add(`image-bitmap:${e}`,l),t&&t(l),r.manager.itemEnd(e),l}).catch(function(l){i&&i(l),Jc.set(c,l),Yn.remove(`image-bitmap:${e}`),r.manager.itemError(e),r.manager.itemEnd(e)});Yn.add(`image-bitmap:${e}`,c),r.manager.itemStart(e)}abort(){return this._abortController.abort(),this._abortController=new AbortController,this}};var da=class extends wt{constructor(e=[]){super(),this.isArrayCamera=!0,this.isMultiViewCamera=!1,this.cameras=e}};var Pl="\\[\\]\\.:\\/",om=new RegExp("["+Pl+"]","g"),Ll="[^"+Pl+"]",am="[^"+Pl.replace("\\.","")+"]",cm=/((?:WC+[\/:])*)/.source.replace("WC",Ll),lm=/(WCOD+)?/.source.replace("WCOD",am),hm=/(?:\.(WC+)(?:\[(.+)\])?)?/.source.replace("WC",Ll),um=/\.(WC+)(?:\[(.+)\])?/.source.replace("WC",Ll),dm=new RegExp("^"+cm+lm+hm+um+"$"),fm=["material","materials","bones","map"],cl=class{constructor(e,t,n){let i=n||tt.parseTrackName(t);this._targetGroup=e,this._bindings=e.subscribe_(t,i)}getValue(e,t){this.bind();let n=this._targetGroup.nCachedObjects_,i=this._bindings[n];i!==void 0&&i.getValue(e,t)}setValue(e,t){let n=this._bindings;for(let i=this._targetGroup.nCachedObjects_,r=n.length;i!==r;++i)n[i].setValue(e,t)}bind(){let e=this._bindings;for(let t=this._targetGroup.nCachedObjects_,n=e.length;t!==n;++t)e[t].bind()}unbind(){let e=this._bindings;for(let t=this._targetGroup.nCachedObjects_,n=e.length;t!==n;++t)e[t].unbind()}},tt=class s{constructor(e,t,n){this.path=t,this.parsedPath=n||s.parseTrackName(t),this.node=s.findNode(e,this.parsedPath.nodeName),this.rootNode=e,this.getValue=this._getValue_unbound,this.setValue=this._setValue_unbound}static create(e,t,n){return e&&e.isAnimationObjectGroup?new s.Composite(e,t,n):new s(e,t,n)}static sanitizeNodeName(e){return e.replace(/\s/g,"_").replace(om,"")}static parseTrackName(e){let t=dm.exec(e);if(t===null)throw new Error("PropertyBinding: Cannot parse trackName: "+e);let n={nodeName:t[2],objectName:t[3],objectIndex:t[4],propertyName:t[5],propertyIndex:t[6]},i=n.nodeName&&n.nodeName.lastIndexOf(".");if(i!==void 0&&i!==-1){let r=n.nodeName.substring(i+1);fm.indexOf(r)!==-1&&(n.nodeName=n.nodeName.substring(0,i),n.objectName=r)}if(n.propertyName===null||n.propertyName.length===0)throw new Error("PropertyBinding: can not parse propertyName from trackName: "+e);return n}static findNode(e,t){if(t===void 0||t===""||t==="."||t===-1||t===e.name||t===e.uuid)return e;if(e.skeleton){let n=e.skeleton.getBoneByName(t);if(n!==void 0)return n}if(e.children){let n=function(r){for(let o=0;o<r.length;o++){let a=r[o];if(a.name===t||a.uuid===t)return a;let c=n(a.children);if(c)return c}return null},i=n(e.children);if(i)return i}return null}_getValue_unavailable(){}_setValue_unavailable(){}_getValue_direct(e,t){e[t]=this.targetObject[this.propertyName]}_getValue_array(e,t){let n=this.resolvedProperty;for(let i=0,r=n.length;i!==r;++i)e[t++]=n[i]}_getValue_arrayElement(e,t){e[t]=this.resolvedProperty[this.propertyIndex]}_getValue_toArray(e,t){this.resolvedProperty.toArray(e,t)}_setValue_direct(e,t){this.targetObject[this.propertyName]=e[t]}_setValue_direct_setNeedsUpdate(e,t){this.targetObject[this.propertyName]=e[t],this.targetObject.needsUpdate=!0}_setValue_direct_setMatrixWorldNeedsUpdate(e,t){this.targetObject[this.propertyName]=e[t],this.targetObject.matrixWorldNeedsUpdate=!0}_setValue_array(e,t){let n=this.resolvedProperty;for(let i=0,r=n.length;i!==r;++i)n[i]=e[t++]}_setValue_array_setNeedsUpdate(e,t){let n=this.resolvedProperty;for(let i=0,r=n.length;i!==r;++i)n[i]=e[t++];this.targetObject.needsUpdate=!0}_setValue_array_setMatrixWorldNeedsUpdate(e,t){let n=this.resolvedProperty;for(let i=0,r=n.length;i!==r;++i)n[i]=e[t++];this.targetObject.matrixWorldNeedsUpdate=!0}_setValue_arrayElement(e,t){this.resolvedProperty[this.propertyIndex]=e[t]}_setValue_arrayElement_setNeedsUpdate(e,t){this.resolvedProperty[this.propertyIndex]=e[t],this.targetObject.needsUpdate=!0}_setValue_arrayElement_setMatrixWorldNeedsUpdate(e,t){this.resolvedProperty[this.propertyIndex]=e[t],this.targetObject.matrixWorldNeedsUpdate=!0}_setValue_fromArray(e,t){this.resolvedProperty.fromArray(e,t)}_setValue_fromArray_setNeedsUpdate(e,t){this.resolvedProperty.fromArray(e,t),this.targetObject.needsUpdate=!0}_setValue_fromArray_setMatrixWorldNeedsUpdate(e,t){this.resolvedProperty.fromArray(e,t),this.targetObject.matrixWorldNeedsUpdate=!0}_getValue_unbound(e,t){this.bind(),this.getValue(e,t)}_setValue_unbound(e,t){this.bind(),this.setValue(e,t)}bind(){let e=this.node,t=this.parsedPath,n=t.objectName,i=t.propertyName,r=t.propertyIndex;if(e||(e=s.findNode(this.rootNode,t.nodeName),this.node=e),this.getValue=this._getValue_unavailable,this.setValue=this._setValue_unavailable,!e){console.warn("THREE.PropertyBinding: No target node found for track: "+this.path+".");return}if(n){let l=t.objectIndex;switch(n){case"materials":if(!e.material){console.error("THREE.PropertyBinding: Can not bind to material as node does not have a material.",this);return}if(!e.material.materials){console.error("THREE.PropertyBinding: Can not bind to material.materials as node.material does not have a materials array.",this);return}e=e.material.materials;break;case"bones":if(!e.skeleton){console.error("THREE.PropertyBinding: Can not bind to bones as node does not have a skeleton.",this);return}e=e.skeleton.bones;for(let h=0;h<e.length;h++)if(e[h].name===l){l=h;break}break;case"map":if("map"in e){e=e.map;break}if(!e.material){console.error("THREE.PropertyBinding: Can not bind to material as node does not have a material.",this);return}if(!e.material.map){console.error("THREE.PropertyBinding: Can not bind to material.map as node.material does not have a map.",this);return}e=e.material.map;break;default:if(e[n]===void 0){console.error("THREE.PropertyBinding: Can not bind to objectName of node undefined.",this);return}e=e[n]}if(l!==void 0){if(e[l]===void 0){console.error("THREE.PropertyBinding: Trying to bind to objectIndex of objectName, but is undefined.",this,e);return}e=e[l]}}let o=e[i];if(o===void 0){let l=t.nodeName;console.error("THREE.PropertyBinding: Trying to update property for track: "+l+"."+i+" but it wasn't found.",e);return}let a=this.Versioning.None;this.targetObject=e,e.isMaterial===!0?a=this.Versioning.NeedsUpdate:e.isObject3D===!0&&(a=this.Versioning.MatrixWorldNeedsUpdate);let c=this.BindingType.Direct;if(r!==void 0){if(i==="morphTargetInfluences"){if(!e.geometry){console.error("THREE.PropertyBinding: Can not bind to morphTargetInfluences because node does not have a geometry.",this);return}if(!e.geometry.morphAttributes){console.error("THREE.PropertyBinding: Can not bind to morphTargetInfluences because node does not have a geometry.morphAttributes.",this);return}e.morphTargetDictionary[r]!==void 0&&(r=e.morphTargetDictionary[r])}c=this.BindingType.ArrayElement,this.resolvedProperty=o,this.propertyIndex=r}else o.fromArray!==void 0&&o.toArray!==void 0?(c=this.BindingType.HasFromToArray,this.resolvedProperty=o):Array.isArray(o)?(c=this.BindingType.EntireArray,this.resolvedProperty=o):this.propertyName=i;this.getValue=this.GetterByBindingType[c],this.setValue=this.SetterByBindingTypeAndVersioning[c][a]}unbind(){this.node=null,this.getValue=this._getValue_unbound,this.setValue=this._setValue_unbound}};tt.Composite=cl;tt.prototype.BindingType={Direct:0,EntireArray:1,ArrayElement:2,HasFromToArray:3};tt.prototype.Versioning={None:0,NeedsUpdate:1,MatrixWorldNeedsUpdate:2};tt.prototype.GetterByBindingType=[tt.prototype._getValue_direct,tt.prototype._getValue_array,tt.prototype._getValue_arrayElement,tt.prototype._getValue_toArray];tt.prototype.SetterByBindingTypeAndVersioning=[[tt.prototype._setValue_direct,tt.prototype._setValue_direct_setNeedsUpdate,tt.prototype._setValue_direct_setMatrixWorldNeedsUpdate],[tt.prototype._setValue_array,tt.prototype._setValue_array_setNeedsUpdate,tt.prototype._setValue_array_setMatrixWorldNeedsUpdate],[tt.prototype._setValue_arrayElement,tt.prototype._setValue_arrayElement_setNeedsUpdate,tt.prototype._setValue_arrayElement_setMatrixWorldNeedsUpdate],[tt.prototype._setValue_fromArray,tt.prototype._setValue_fromArray_setNeedsUpdate,tt.prototype._setValue_fromArray_setMatrixWorldNeedsUpdate]];var Dy=new Float32Array(1);var Qu=new De,no=class{constructor(e,t,n=0,i=1/0){this.ray=new Jn(e,t),this.near=n,this.far=i,this.camera=null,this.layers=new js,this.params={Mesh:{},Line:{threshold:1},LOD:{},Points:{threshold:1},Sprite:{}}}set(e,t){this.ray.set(e,t)}setFromCamera(e,t){t.isPerspectiveCamera?(this.ray.origin.setFromMatrixPosition(t.matrixWorld),this.ray.direction.set(e.x,e.y,.5).unproject(t).sub(this.ray.origin).normalize(),this.camera=t):t.isOrthographicCamera?(this.ray.origin.set(e.x,e.y,(t.near+t.far)/(t.near-t.far)).unproject(t),this.ray.direction.set(0,0,-1).transformDirection(t.matrixWorld),this.camera=t):console.error("THREE.Raycaster: Unsupported camera type: "+t.type)}setFromXRController(e){return Qu.identity().extractRotation(e.matrixWorld),this.ray.origin.setFromMatrixPosition(e.matrixWorld),this.ray.direction.set(0,0,-1).applyMatrix4(Qu),this}intersectObject(e,t=!0,n=[]){return ll(e,this,n,t),n.sort(ed),n}intersectObjects(e,t=!0,n=[]){for(let i=0,r=e.length;i<r;i++)ll(e[i],this,n,t);return n.sort(ed),n}};function ed(s,e){return s.distance-e.distance}function ll(s,e,t,n){let i=!0;if(s.layers.test(e.layers)&&s.raycast(e,t)===!1&&(i=!1),i===!0&&n===!0){let r=s.children;for(let o=0,a=r.length;o<a;o++)ll(r[o],e,t,!0)}}var nr=class{constructor(e=1,t=0,n=0){this.radius=e,this.phi=t,this.theta=n}set(e,t,n){return this.radius=e,this.phi=t,this.theta=n,this}copy(e){return this.radius=e.radius,this.phi=e.phi,this.theta=e.theta,this}makeSafe(){return this.phi=qe(this.phi,1e-6,Math.PI-1e-6),this}setFromVector3(e){return this.setFromCartesianCoords(e.x,e.y,e.z)}setFromCartesianCoords(e,t,n){return this.radius=Math.sqrt(e*e+t*t+n*n),this.radius===0?(this.theta=0,this.phi=0):(this.theta=Math.atan2(e,n),this.phi=Math.acos(qe(t/this.radius,-1,1))),this}clone(){return new this.constructor().copy(this)}};var io=class extends zi{constructor(e=10,t=10,n=4473924,i=8947848){n=new Re(n),i=new Re(i);let r=t/2,o=e/t,a=e/2,c=[],l=[];for(let d=0,f=0,g=-a;d<=t;d++,g+=o){c.push(-a,0,g,a,0,g),c.push(g,0,-a,g,0,a);let _=d===r?n:i;_.toArray(l,f),f+=3,_.toArray(l,f),f+=3,_.toArray(l,f),f+=3,_.toArray(l,f),f+=3}let h=new xt;h.setAttribute("position",new bt(c,3)),h.setAttribute("color",new bt(l,3));let u=new Tn({vertexColors:!0,toneMapped:!1});super(h,u),this.type="GridHelper"}dispose(){this.geometry.dispose(),this.material.dispose()}};var so=class extends Kn{constructor(e,t=null){super(),this.object=e,this.domElement=t,this.enabled=!0,this.state=-1,this.keys={},this.mouseButtons={LEFT:null,MIDDLE:null,RIGHT:null},this.touches={ONE:null,TWO:null}}connect(e){if(e===void 0){console.warn("THREE.Controls: connect() now requires an element.");return}this.domElement!==null&&this.disconnect(),this.domElement=e}disconnect(){}dispose(){}update(){}};function Nl(s,e,t,n){let i=pm(n);switch(t){case bl:return s*e;case Aa:return s*e/i.components*i.byteLength;case Ra:return s*e/i.components*i.byteLength;case Sl:return s*e*2/i.components*i.byteLength;case Ca:return s*e*2/i.components*i.byteLength;case Ml:return s*e*3/i.components*i.byteLength;case tn:return s*e*4/i.components*i.byteLength;case Ia:return s*e*4/i.components*i.byteLength;case oo:case ao:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*8;case co:case lo:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*16;case La:case Da:return Math.max(s,16)*Math.max(e,8)/4;case Pa:case Na:return Math.max(s,8)*Math.max(e,8)/2;case Ua:case Oa:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*8;case Fa:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*16;case Ba:return Math.floor((s+3)/4)*Math.floor((e+3)/4)*16;case ka:return Math.floor((s+4)/5)*Math.floor((e+3)/4)*16;case za:return Math.floor((s+4)/5)*Math.floor((e+4)/5)*16;case Ha:return Math.floor((s+5)/6)*Math.floor((e+4)/5)*16;case Va:return Math.floor((s+5)/6)*Math.floor((e+5)/6)*16;case Ga:return Math.floor((s+7)/8)*Math.floor((e+4)/5)*16;case Wa:return Math.floor((s+7)/8)*Math.floor((e+5)/6)*16;case Xa:return Math.floor((s+7)/8)*Math.floor((e+7)/8)*16;case qa:return Math.floor((s+9)/10)*Math.floor((e+4)/5)*16;case Ya:return Math.floor((s+9)/10)*Math.floor((e+5)/6)*16;case ja:return Math.floor((s+9)/10)*Math.floor((e+7)/8)*16;case Ka:return Math.floor((s+9)/10)*Math.floor((e+9)/10)*16;case Za:return Math.floor((s+11)/12)*Math.floor((e+9)/10)*16;case Ja:return Math.floor((s+11)/12)*Math.floor((e+11)/12)*16;case $a:case Qa:case ec:return Math.ceil(s/4)*Math.ceil(e/4)*16;case tc:case nc:return Math.ceil(s/4)*Math.ceil(e/4)*8;case ic:case sc:return Math.ceil(s/4)*Math.ceil(e/4)*16}throw new Error(`Unable to determine texture byte length for ${t} format.`)}function pm(s){switch(s){case zn:case _l:return{byteLength:1,components:1};case sr:case xl:case rr:return{byteLength:2,components:1};case Ta:case wa:return{byteLength:2,components:4};case Xi:case Ea:case wn:return{byteLength:4,components:1};case yl:case vl:return{byteLength:4,components:3}}throw new Error(`Unknown texture type ${s}.`)}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:"180"}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__="180");/**
 * @license
 * Copyright 2010-2025 Three.js Authors
 * SPDX-License-Identifier: MIT
 */function xf(){let s=null,e=!1,t=null,n=null;function i(r,o){t(r,o),n=s.requestAnimationFrame(i)}return{start:function(){e!==!0&&t!==null&&(n=s.requestAnimationFrame(i),e=!0)},stop:function(){s.cancelAnimationFrame(n),e=!1},setAnimationLoop:function(r){t=r},setContext:function(r){s=r}}}function mm(s){let e=new WeakMap;function t(a,c){let l=a.array,h=a.usage,u=l.byteLength,d=s.createBuffer();s.bindBuffer(c,d),s.bufferData(c,l,h),a.onUploadCallback();let f;if(l instanceof Float32Array)f=s.FLOAT;else if(typeof Float16Array<"u"&&l instanceof Float16Array)f=s.HALF_FLOAT;else if(l instanceof Uint16Array)a.isFloat16BufferAttribute?f=s.HALF_FLOAT:f=s.UNSIGNED_SHORT;else if(l instanceof Int16Array)f=s.SHORT;else if(l instanceof Uint32Array)f=s.UNSIGNED_INT;else if(l instanceof Int32Array)f=s.INT;else if(l instanceof Int8Array)f=s.BYTE;else if(l instanceof Uint8Array)f=s.UNSIGNED_BYTE;else if(l instanceof Uint8ClampedArray)f=s.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+l);return{buffer:d,type:f,bytesPerElement:l.BYTES_PER_ELEMENT,version:a.version,size:u}}function n(a,c,l){let h=c.array,u=c.updateRanges;if(s.bindBuffer(l,a),u.length===0)s.bufferSubData(l,0,h);else{u.sort((f,g)=>f.start-g.start);let d=0;for(let f=1;f<u.length;f++){let g=u[d],_=u[f];_.start<=g.start+g.count+1?g.count=Math.max(g.count,_.start+_.count-g.start):(++d,u[d]=_)}u.length=d+1;for(let f=0,g=u.length;f<g;f++){let _=u[f];s.bufferSubData(l,_.start*h.BYTES_PER_ELEMENT,h,_.start,_.count)}c.clearUpdateRanges()}c.onUploadCallback()}function i(a){return a.isInterleavedBufferAttribute&&(a=a.data),e.get(a)}function r(a){a.isInterleavedBufferAttribute&&(a=a.data);let c=e.get(a);c&&(s.deleteBuffer(c.buffer),e.delete(a))}function o(a,c){if(a.isInterleavedBufferAttribute&&(a=a.data),a.isGLBufferAttribute){let h=e.get(a);(!h||h.version<a.version)&&e.set(a,{buffer:a.buffer,type:a.type,bytesPerElement:a.elementSize,version:a.version});return}let l=e.get(a);if(l===void 0)e.set(a,t(a,c));else if(l.version<a.version){if(l.size!==a.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");n(l.buffer,a,c),l.version=a.version}}return{get:i,remove:r,update:o}}var gm=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,_m=`#ifdef USE_ALPHAHASH
	const float ALPHA_HASH_SCALE = 0.05;
	float hash2D( vec2 value ) {
		return fract( 1.0e4 * sin( 17.0 * value.x + 0.1 * value.y ) * ( 0.1 + abs( sin( 13.0 * value.y + value.x ) ) ) );
	}
	float hash3D( vec3 value ) {
		return hash2D( vec2( hash2D( value.xy ), value.z ) );
	}
	float getAlphaHashThreshold( vec3 position ) {
		float maxDeriv = max(
			length( dFdx( position.xyz ) ),
			length( dFdy( position.xyz ) )
		);
		float pixScale = 1.0 / ( ALPHA_HASH_SCALE * maxDeriv );
		vec2 pixScales = vec2(
			exp2( floor( log2( pixScale ) ) ),
			exp2( ceil( log2( pixScale ) ) )
		);
		vec2 alpha = vec2(
			hash3D( floor( pixScales.x * position.xyz ) ),
			hash3D( floor( pixScales.y * position.xyz ) )
		);
		float lerpFactor = fract( log2( pixScale ) );
		float x = ( 1.0 - lerpFactor ) * alpha.x + lerpFactor * alpha.y;
		float a = min( lerpFactor, 1.0 - lerpFactor );
		vec3 cases = vec3(
			x * x / ( 2.0 * a * ( 1.0 - a ) ),
			( x - 0.5 * a ) / ( 1.0 - a ),
			1.0 - ( ( 1.0 - x ) * ( 1.0 - x ) / ( 2.0 * a * ( 1.0 - a ) ) )
		);
		float threshold = ( x < ( 1.0 - a ) )
			? ( ( x < a ) ? cases.x : cases.y )
			: cases.z;
		return clamp( threshold , 1.0e-6, 1.0 );
	}
#endif`,xm=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,ym=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,vm=`#ifdef USE_ALPHATEST
	#ifdef ALPHA_TO_COVERAGE
	diffuseColor.a = smoothstep( alphaTest, alphaTest + fwidth( diffuseColor.a ), diffuseColor.a );
	if ( diffuseColor.a == 0.0 ) discard;
	#else
	if ( diffuseColor.a < alphaTest ) discard;
	#endif
#endif`,bm=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,Mm=`#ifdef USE_AOMAP
	float ambientOcclusion = ( texture2D( aoMap, vAoMapUv ).r - 1.0 ) * aoMapIntensity + 1.0;
	reflectedLight.indirectDiffuse *= ambientOcclusion;
	#if defined( USE_CLEARCOAT )
		clearcoatSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_SHEEN )
		sheenSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD )
		float dotNV = saturate( dot( geometryNormal, geometryViewDir ) );
		reflectedLight.indirectSpecular *= computeSpecularOcclusion( dotNV, ambientOcclusion, material.roughness );
	#endif
#endif`,Sm=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,Em=`#ifdef USE_BATCHING
	#if ! defined( GL_ANGLE_multi_draw )
	#define gl_DrawID _gl_DrawID
	uniform int _gl_DrawID;
	#endif
	uniform highp sampler2D batchingTexture;
	uniform highp usampler2D batchingIdTexture;
	mat4 getBatchingMatrix( const in float i ) {
		int size = textureSize( batchingTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( batchingTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( batchingTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( batchingTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( batchingTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
	float getIndirectIndex( const in int i ) {
		int size = textureSize( batchingIdTexture, 0 ).x;
		int x = i % size;
		int y = i / size;
		return float( texelFetch( batchingIdTexture, ivec2( x, y ), 0 ).r );
	}
#endif
#ifdef USE_BATCHING_COLOR
	uniform sampler2D batchingColorTexture;
	vec3 getBatchingColor( const in float i ) {
		int size = textureSize( batchingColorTexture, 0 ).x;
		int j = int( i );
		int x = j % size;
		int y = j / size;
		return texelFetch( batchingColorTexture, ivec2( x, y ), 0 ).rgb;
	}
#endif`,Tm=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( getIndirectIndex( gl_DrawID ) );
#endif`,wm=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,Am=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,Rm=`float G_BlinnPhong_Implicit( ) {
	return 0.25;
}
float D_BlinnPhong( const in float shininess, const in float dotNH ) {
	return RECIPROCAL_PI * ( shininess * 0.5 + 1.0 ) * pow( dotNH, shininess );
}
vec3 BRDF_BlinnPhong( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in vec3 specularColor, const in float shininess ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( specularColor, 1.0, dotVH );
	float G = G_BlinnPhong_Implicit( );
	float D = D_BlinnPhong( shininess, dotNH );
	return F * ( G * D );
} // validated`,Cm=`#ifdef USE_IRIDESCENCE
	const mat3 XYZ_TO_REC709 = mat3(
		 3.2404542, -0.9692660,  0.0556434,
		-1.5371385,  1.8760108, -0.2040259,
		-0.4985314,  0.0415560,  1.0572252
	);
	vec3 Fresnel0ToIor( vec3 fresnel0 ) {
		vec3 sqrtF0 = sqrt( fresnel0 );
		return ( vec3( 1.0 ) + sqrtF0 ) / ( vec3( 1.0 ) - sqrtF0 );
	}
	vec3 IorToFresnel0( vec3 transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - vec3( incidentIor ) ) / ( transmittedIor + vec3( incidentIor ) ) );
	}
	float IorToFresnel0( float transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - incidentIor ) / ( transmittedIor + incidentIor ));
	}
	vec3 evalSensitivity( float OPD, vec3 shift ) {
		float phase = 2.0 * PI * OPD * 1.0e-9;
		vec3 val = vec3( 5.4856e-13, 4.4201e-13, 5.2481e-13 );
		vec3 pos = vec3( 1.6810e+06, 1.7953e+06, 2.2084e+06 );
		vec3 var = vec3( 4.3278e+09, 9.3046e+09, 6.6121e+09 );
		vec3 xyz = val * sqrt( 2.0 * PI * var ) * cos( pos * phase + shift ) * exp( - pow2( phase ) * var );
		xyz.x += 9.7470e-14 * sqrt( 2.0 * PI * 4.5282e+09 ) * cos( 2.2399e+06 * phase + shift[ 0 ] ) * exp( - 4.5282e+09 * pow2( phase ) );
		xyz /= 1.0685e-7;
		vec3 rgb = XYZ_TO_REC709 * xyz;
		return rgb;
	}
	vec3 evalIridescence( float outsideIOR, float eta2, float cosTheta1, float thinFilmThickness, vec3 baseF0 ) {
		vec3 I;
		float iridescenceIOR = mix( outsideIOR, eta2, smoothstep( 0.0, 0.03, thinFilmThickness ) );
		float sinTheta2Sq = pow2( outsideIOR / iridescenceIOR ) * ( 1.0 - pow2( cosTheta1 ) );
		float cosTheta2Sq = 1.0 - sinTheta2Sq;
		if ( cosTheta2Sq < 0.0 ) {
			return vec3( 1.0 );
		}
		float cosTheta2 = sqrt( cosTheta2Sq );
		float R0 = IorToFresnel0( iridescenceIOR, outsideIOR );
		float R12 = F_Schlick( R0, 1.0, cosTheta1 );
		float T121 = 1.0 - R12;
		float phi12 = 0.0;
		if ( iridescenceIOR < outsideIOR ) phi12 = PI;
		float phi21 = PI - phi12;
		vec3 baseIOR = Fresnel0ToIor( clamp( baseF0, 0.0, 0.9999 ) );		vec3 R1 = IorToFresnel0( baseIOR, iridescenceIOR );
		vec3 R23 = F_Schlick( R1, 1.0, cosTheta2 );
		vec3 phi23 = vec3( 0.0 );
		if ( baseIOR[ 0 ] < iridescenceIOR ) phi23[ 0 ] = PI;
		if ( baseIOR[ 1 ] < iridescenceIOR ) phi23[ 1 ] = PI;
		if ( baseIOR[ 2 ] < iridescenceIOR ) phi23[ 2 ] = PI;
		float OPD = 2.0 * iridescenceIOR * thinFilmThickness * cosTheta2;
		vec3 phi = vec3( phi21 ) + phi23;
		vec3 R123 = clamp( R12 * R23, 1e-5, 0.9999 );
		vec3 r123 = sqrt( R123 );
		vec3 Rs = pow2( T121 ) * R23 / ( vec3( 1.0 ) - R123 );
		vec3 C0 = R12 + Rs;
		I = C0;
		vec3 Cm = Rs - T121;
		for ( int m = 1; m <= 2; ++ m ) {
			Cm *= r123;
			vec3 Sm = 2.0 * evalSensitivity( float( m ) * OPD, float( m ) * phi );
			I += Cm * Sm;
		}
		return max( I, vec3( 0.0 ) );
	}
#endif`,Im=`#ifdef USE_BUMPMAP
	uniform sampler2D bumpMap;
	uniform float bumpScale;
	vec2 dHdxy_fwd() {
		vec2 dSTdx = dFdx( vBumpMapUv );
		vec2 dSTdy = dFdy( vBumpMapUv );
		float Hll = bumpScale * texture2D( bumpMap, vBumpMapUv ).x;
		float dBx = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdx ).x - Hll;
		float dBy = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdy ).x - Hll;
		return vec2( dBx, dBy );
	}
	vec3 perturbNormalArb( vec3 surf_pos, vec3 surf_norm, vec2 dHdxy, float faceDirection ) {
		vec3 vSigmaX = normalize( dFdx( surf_pos.xyz ) );
		vec3 vSigmaY = normalize( dFdy( surf_pos.xyz ) );
		vec3 vN = surf_norm;
		vec3 R1 = cross( vSigmaY, vN );
		vec3 R2 = cross( vN, vSigmaX );
		float fDet = dot( vSigmaX, R1 ) * faceDirection;
		vec3 vGrad = sign( fDet ) * ( dHdxy.x * R1 + dHdxy.y * R2 );
		return normalize( abs( fDet ) * surf_norm - vGrad );
	}
#endif`,Pm=`#if NUM_CLIPPING_PLANES > 0
	vec4 plane;
	#ifdef ALPHA_TO_COVERAGE
		float distanceToPlane, distanceGradient;
		float clipOpacity = 1.0;
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
			distanceGradient = fwidth( distanceToPlane ) / 2.0;
			clipOpacity *= smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			if ( clipOpacity == 0.0 ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			float unionClipOpacity = 1.0;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
				distanceGradient = fwidth( distanceToPlane ) / 2.0;
				unionClipOpacity *= 1.0 - smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			}
			#pragma unroll_loop_end
			clipOpacity *= 1.0 - unionClipOpacity;
		#endif
		diffuseColor.a *= clipOpacity;
		if ( diffuseColor.a == 0.0 ) discard;
	#else
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			if ( dot( vClipPosition, plane.xyz ) > plane.w ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			bool clipped = true;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				clipped = ( dot( vClipPosition, plane.xyz ) > plane.w ) && clipped;
			}
			#pragma unroll_loop_end
			if ( clipped ) discard;
		#endif
	#endif
#endif`,Lm=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,Nm=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,Dm=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,Um=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,Om=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,Fm=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	varying vec3 vColor;
#endif`,Bm=`#if defined( USE_COLOR_ALPHA )
	vColor = vec4( 1.0 );
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	vColor = vec3( 1.0 );
#endif
#ifdef USE_COLOR
	vColor *= color;
#endif
#ifdef USE_INSTANCING_COLOR
	vColor.xyz *= instanceColor.xyz;
#endif
#ifdef USE_BATCHING_COLOR
	vec3 batchingColor = getBatchingColor( getIndirectIndex( gl_DrawID ) );
	vColor.xyz *= batchingColor.xyz;
#endif`,km=`#define PI 3.141592653589793
#define PI2 6.283185307179586
#define PI_HALF 1.5707963267948966
#define RECIPROCAL_PI 0.3183098861837907
#define RECIPROCAL_PI2 0.15915494309189535
#define EPSILON 1e-6
#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
#define whiteComplement( a ) ( 1.0 - saturate( a ) )
float pow2( const in float x ) { return x*x; }
vec3 pow2( const in vec3 x ) { return x*x; }
float pow3( const in float x ) { return x*x*x; }
float pow4( const in float x ) { float x2 = x*x; return x2*x2; }
float max3( const in vec3 v ) { return max( max( v.x, v.y ), v.z ); }
float average( const in vec3 v ) { return dot( v, vec3( 0.3333333 ) ); }
highp float rand( const in vec2 uv ) {
	const highp float a = 12.9898, b = 78.233, c = 43758.5453;
	highp float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	return fract( sin( sn ) * c );
}
#ifdef HIGH_PRECISION
	float precisionSafeLength( vec3 v ) { return length( v ); }
#else
	float precisionSafeLength( vec3 v ) {
		float maxComponent = max3( abs( v ) );
		return length( v / maxComponent ) * maxComponent;
	}
#endif
struct IncidentLight {
	vec3 color;
	vec3 direction;
	bool visible;
};
struct ReflectedLight {
	vec3 directDiffuse;
	vec3 directSpecular;
	vec3 indirectDiffuse;
	vec3 indirectSpecular;
};
#ifdef USE_ALPHAHASH
	varying vec3 vPosition;
#endif
vec3 transformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );
}
vec3 inverseTransformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( vec4( dir, 0.0 ) * matrix ).xyz );
}
mat3 transposeMat3( const in mat3 m ) {
	mat3 tmp;
	tmp[ 0 ] = vec3( m[ 0 ].x, m[ 1 ].x, m[ 2 ].x );
	tmp[ 1 ] = vec3( m[ 0 ].y, m[ 1 ].y, m[ 2 ].y );
	tmp[ 2 ] = vec3( m[ 0 ].z, m[ 1 ].z, m[ 2 ].z );
	return tmp;
}
bool isPerspectiveMatrix( mat4 m ) {
	return m[ 2 ][ 3 ] == - 1.0;
}
vec2 equirectUv( in vec3 dir ) {
	float u = atan( dir.z, dir.x ) * RECIPROCAL_PI2 + 0.5;
	float v = asin( clamp( dir.y, - 1.0, 1.0 ) ) * RECIPROCAL_PI + 0.5;
	return vec2( u, v );
}
vec3 BRDF_Lambert( const in vec3 diffuseColor ) {
	return RECIPROCAL_PI * diffuseColor;
}
vec3 F_Schlick( const in vec3 f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
}
float F_Schlick( const in float f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
} // validated`,zm=`#ifdef ENVMAP_TYPE_CUBE_UV
	#define cubeUV_minMipLevel 4.0
	#define cubeUV_minTileSize 16.0
	float getFace( vec3 direction ) {
		vec3 absDirection = abs( direction );
		float face = - 1.0;
		if ( absDirection.x > absDirection.z ) {
			if ( absDirection.x > absDirection.y )
				face = direction.x > 0.0 ? 0.0 : 3.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		} else {
			if ( absDirection.z > absDirection.y )
				face = direction.z > 0.0 ? 2.0 : 5.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		}
		return face;
	}
	vec2 getUV( vec3 direction, float face ) {
		vec2 uv;
		if ( face == 0.0 ) {
			uv = vec2( direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 1.0 ) {
			uv = vec2( - direction.x, - direction.z ) / abs( direction.y );
		} else if ( face == 2.0 ) {
			uv = vec2( - direction.x, direction.y ) / abs( direction.z );
		} else if ( face == 3.0 ) {
			uv = vec2( - direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 4.0 ) {
			uv = vec2( - direction.x, direction.z ) / abs( direction.y );
		} else {
			uv = vec2( direction.x, direction.y ) / abs( direction.z );
		}
		return 0.5 * ( uv + 1.0 );
	}
	vec3 bilinearCubeUV( sampler2D envMap, vec3 direction, float mipInt ) {
		float face = getFace( direction );
		float filterInt = max( cubeUV_minMipLevel - mipInt, 0.0 );
		mipInt = max( mipInt, cubeUV_minMipLevel );
		float faceSize = exp2( mipInt );
		highp vec2 uv = getUV( direction, face ) * ( faceSize - 2.0 ) + 1.0;
		if ( face > 2.0 ) {
			uv.y += faceSize;
			face -= 3.0;
		}
		uv.x += face * faceSize;
		uv.x += filterInt * 3.0 * cubeUV_minTileSize;
		uv.y += 4.0 * ( exp2( CUBEUV_MAX_MIP ) - faceSize );
		uv.x *= CUBEUV_TEXEL_WIDTH;
		uv.y *= CUBEUV_TEXEL_HEIGHT;
		#ifdef texture2DGradEXT
			return texture2DGradEXT( envMap, uv, vec2( 0.0 ), vec2( 0.0 ) ).rgb;
		#else
			return texture2D( envMap, uv ).rgb;
		#endif
	}
	#define cubeUV_r0 1.0
	#define cubeUV_m0 - 2.0
	#define cubeUV_r1 0.8
	#define cubeUV_m1 - 1.0
	#define cubeUV_r4 0.4
	#define cubeUV_m4 2.0
	#define cubeUV_r5 0.305
	#define cubeUV_m5 3.0
	#define cubeUV_r6 0.21
	#define cubeUV_m6 4.0
	float roughnessToMip( float roughness ) {
		float mip = 0.0;
		if ( roughness >= cubeUV_r1 ) {
			mip = ( cubeUV_r0 - roughness ) * ( cubeUV_m1 - cubeUV_m0 ) / ( cubeUV_r0 - cubeUV_r1 ) + cubeUV_m0;
		} else if ( roughness >= cubeUV_r4 ) {
			mip = ( cubeUV_r1 - roughness ) * ( cubeUV_m4 - cubeUV_m1 ) / ( cubeUV_r1 - cubeUV_r4 ) + cubeUV_m1;
		} else if ( roughness >= cubeUV_r5 ) {
			mip = ( cubeUV_r4 - roughness ) * ( cubeUV_m5 - cubeUV_m4 ) / ( cubeUV_r4 - cubeUV_r5 ) + cubeUV_m4;
		} else if ( roughness >= cubeUV_r6 ) {
			mip = ( cubeUV_r5 - roughness ) * ( cubeUV_m6 - cubeUV_m5 ) / ( cubeUV_r5 - cubeUV_r6 ) + cubeUV_m5;
		} else {
			mip = - 2.0 * log2( 1.16 * roughness );		}
		return mip;
	}
	vec4 textureCubeUV( sampler2D envMap, vec3 sampleDir, float roughness ) {
		float mip = clamp( roughnessToMip( roughness ), cubeUV_m0, CUBEUV_MAX_MIP );
		float mipF = fract( mip );
		float mipInt = floor( mip );
		vec3 color0 = bilinearCubeUV( envMap, sampleDir, mipInt );
		if ( mipF == 0.0 ) {
			return vec4( color0, 1.0 );
		} else {
			vec3 color1 = bilinearCubeUV( envMap, sampleDir, mipInt + 1.0 );
			return vec4( mix( color0, color1, mipF ), 1.0 );
		}
	}
#endif`,Hm=`vec3 transformedNormal = objectNormal;
#ifdef USE_TANGENT
	vec3 transformedTangent = objectTangent;
#endif
#ifdef USE_BATCHING
	mat3 bm = mat3( batchingMatrix );
	transformedNormal /= vec3( dot( bm[ 0 ], bm[ 0 ] ), dot( bm[ 1 ], bm[ 1 ] ), dot( bm[ 2 ], bm[ 2 ] ) );
	transformedNormal = bm * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = bm * transformedTangent;
	#endif
#endif
#ifdef USE_INSTANCING
	mat3 im = mat3( instanceMatrix );
	transformedNormal /= vec3( dot( im[ 0 ], im[ 0 ] ), dot( im[ 1 ], im[ 1 ] ), dot( im[ 2 ], im[ 2 ] ) );
	transformedNormal = im * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = im * transformedTangent;
	#endif
#endif
transformedNormal = normalMatrix * transformedNormal;
#ifdef FLIP_SIDED
	transformedNormal = - transformedNormal;
#endif
#ifdef USE_TANGENT
	transformedTangent = ( modelViewMatrix * vec4( transformedTangent, 0.0 ) ).xyz;
	#ifdef FLIP_SIDED
		transformedTangent = - transformedTangent;
	#endif
#endif`,Vm=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,Gm=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,Wm=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	#ifdef DECODE_VIDEO_TEXTURE_EMISSIVE
		emissiveColor = sRGBTransferEOTF( emissiveColor );
	#endif
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,Xm=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,qm="gl_FragColor = linearToOutputTexel( gl_FragColor );",Ym=`vec4 LinearTransferOETF( in vec4 value ) {
	return value;
}
vec4 sRGBTransferEOTF( in vec4 value ) {
	return vec4( mix( pow( value.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), value.rgb * 0.0773993808, vec3( lessThanEqual( value.rgb, vec3( 0.04045 ) ) ) ), value.a );
}
vec4 sRGBTransferOETF( in vec4 value ) {
	return vec4( mix( pow( value.rgb, vec3( 0.41666 ) ) * 1.055 - vec3( 0.055 ), value.rgb * 12.92, vec3( lessThanEqual( value.rgb, vec3( 0.0031308 ) ) ) ), value.a );
}`,jm=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vec3 cameraToFrag;
		if ( isOrthographic ) {
			cameraToFrag = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToFrag = normalize( vWorldPosition - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vec3 reflectVec = reflect( cameraToFrag, worldNormal );
		#else
			vec3 reflectVec = refract( cameraToFrag, worldNormal, refractionRatio );
		#endif
	#else
		vec3 reflectVec = vReflect;
	#endif
	#ifdef ENVMAP_TYPE_CUBE
		vec4 envColor = textureCube( envMap, envMapRotation * vec3( flipEnvMap * reflectVec.x, reflectVec.yz ) );
	#else
		vec4 envColor = vec4( 0.0 );
	#endif
	#ifdef ENVMAP_BLENDING_MULTIPLY
		outgoingLight = mix( outgoingLight, outgoingLight * envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_MIX )
		outgoingLight = mix( outgoingLight, envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_ADD )
		outgoingLight += envColor.xyz * specularStrength * reflectivity;
	#endif
#endif`,Km=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	uniform mat3 envMapRotation;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif

#endif`,Zm=`#ifdef USE_ENVMAP
	uniform float reflectivity;
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		varying vec3 vWorldPosition;
		uniform float refractionRatio;
	#else
		varying vec3 vReflect;
	#endif
#endif`,Jm=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS

		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,$m=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vWorldPosition = worldPosition.xyz;
	#else
		vec3 cameraToVertex;
		if ( isOrthographic ) {
			cameraToVertex = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToVertex = normalize( worldPosition.xyz - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vReflect = reflect( cameraToVertex, worldNormal );
		#else
			vReflect = refract( cameraToVertex, worldNormal, refractionRatio );
		#endif
	#endif
#endif`,Qm=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,eg=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,tg=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,ng=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,ig=`#ifdef USE_GRADIENTMAP
	uniform sampler2D gradientMap;
#endif
vec3 getGradientIrradiance( vec3 normal, vec3 lightDirection ) {
	float dotNL = dot( normal, lightDirection );
	vec2 coord = vec2( dotNL * 0.5 + 0.5, 0.0 );
	#ifdef USE_GRADIENTMAP
		return vec3( texture2D( gradientMap, coord ).r );
	#else
		vec2 fw = fwidth( coord ) * 0.5;
		return mix( vec3( 0.7 ), vec3( 1.0 ), smoothstep( 0.7 - fw.x, 0.7 + fw.x, coord.x ) );
	#endif
}`,sg=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,rg=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,og=`varying vec3 vViewPosition;
struct LambertMaterial {
	vec3 diffuseColor;
	float specularStrength;
};
void RE_Direct_Lambert( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Lambert( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Lambert
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,ag=`uniform bool receiveShadow;
uniform vec3 ambientLightColor;
#if defined( USE_LIGHT_PROBES )
	uniform vec3 lightProbe[ 9 ];
#endif
vec3 shGetIrradianceAt( in vec3 normal, in vec3 shCoefficients[ 9 ] ) {
	float x = normal.x, y = normal.y, z = normal.z;
	vec3 result = shCoefficients[ 0 ] * 0.886227;
	result += shCoefficients[ 1 ] * 2.0 * 0.511664 * y;
	result += shCoefficients[ 2 ] * 2.0 * 0.511664 * z;
	result += shCoefficients[ 3 ] * 2.0 * 0.511664 * x;
	result += shCoefficients[ 4 ] * 2.0 * 0.429043 * x * y;
	result += shCoefficients[ 5 ] * 2.0 * 0.429043 * y * z;
	result += shCoefficients[ 6 ] * ( 0.743125 * z * z - 0.247708 );
	result += shCoefficients[ 7 ] * 2.0 * 0.429043 * x * z;
	result += shCoefficients[ 8 ] * 0.429043 * ( x * x - y * y );
	return result;
}
vec3 getLightProbeIrradiance( const in vec3 lightProbe[ 9 ], const in vec3 normal ) {
	vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
	vec3 irradiance = shGetIrradianceAt( worldNormal, lightProbe );
	return irradiance;
}
vec3 getAmbientLightIrradiance( const in vec3 ambientLightColor ) {
	vec3 irradiance = ambientLightColor;
	return irradiance;
}
float getDistanceAttenuation( const in float lightDistance, const in float cutoffDistance, const in float decayExponent ) {
	float distanceFalloff = 1.0 / max( pow( lightDistance, decayExponent ), 0.01 );
	if ( cutoffDistance > 0.0 ) {
		distanceFalloff *= pow2( saturate( 1.0 - pow4( lightDistance / cutoffDistance ) ) );
	}
	return distanceFalloff;
}
float getSpotAttenuation( const in float coneCosine, const in float penumbraCosine, const in float angleCosine ) {
	return smoothstep( coneCosine, penumbraCosine, angleCosine );
}
#if NUM_DIR_LIGHTS > 0
	struct DirectionalLight {
		vec3 direction;
		vec3 color;
	};
	uniform DirectionalLight directionalLights[ NUM_DIR_LIGHTS ];
	void getDirectionalLightInfo( const in DirectionalLight directionalLight, out IncidentLight light ) {
		light.color = directionalLight.color;
		light.direction = directionalLight.direction;
		light.visible = true;
	}
#endif
#if NUM_POINT_LIGHTS > 0
	struct PointLight {
		vec3 position;
		vec3 color;
		float distance;
		float decay;
	};
	uniform PointLight pointLights[ NUM_POINT_LIGHTS ];
	void getPointLightInfo( const in PointLight pointLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = pointLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float lightDistance = length( lVector );
		light.color = pointLight.color;
		light.color *= getDistanceAttenuation( lightDistance, pointLight.distance, pointLight.decay );
		light.visible = ( light.color != vec3( 0.0 ) );
	}
#endif
#if NUM_SPOT_LIGHTS > 0
	struct SpotLight {
		vec3 position;
		vec3 direction;
		vec3 color;
		float distance;
		float decay;
		float coneCos;
		float penumbraCos;
	};
	uniform SpotLight spotLights[ NUM_SPOT_LIGHTS ];
	void getSpotLightInfo( const in SpotLight spotLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = spotLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float angleCos = dot( light.direction, spotLight.direction );
		float spotAttenuation = getSpotAttenuation( spotLight.coneCos, spotLight.penumbraCos, angleCos );
		if ( spotAttenuation > 0.0 ) {
			float lightDistance = length( lVector );
			light.color = spotLight.color * spotAttenuation;
			light.color *= getDistanceAttenuation( lightDistance, spotLight.distance, spotLight.decay );
			light.visible = ( light.color != vec3( 0.0 ) );
		} else {
			light.color = vec3( 0.0 );
			light.visible = false;
		}
	}
#endif
#if NUM_RECT_AREA_LIGHTS > 0
	struct RectAreaLight {
		vec3 color;
		vec3 position;
		vec3 halfWidth;
		vec3 halfHeight;
	};
	uniform sampler2D ltc_1;	uniform sampler2D ltc_2;
	uniform RectAreaLight rectAreaLights[ NUM_RECT_AREA_LIGHTS ];
#endif
#if NUM_HEMI_LIGHTS > 0
	struct HemisphereLight {
		vec3 direction;
		vec3 skyColor;
		vec3 groundColor;
	};
	uniform HemisphereLight hemisphereLights[ NUM_HEMI_LIGHTS ];
	vec3 getHemisphereLightIrradiance( const in HemisphereLight hemiLight, const in vec3 normal ) {
		float dotNL = dot( normal, hemiLight.direction );
		float hemiDiffuseWeight = 0.5 * dotNL + 0.5;
		vec3 irradiance = mix( hemiLight.groundColor, hemiLight.skyColor, hemiDiffuseWeight );
		return irradiance;
	}
#endif`,cg=`#ifdef USE_ENVMAP
	vec3 getIBLIrradiance( const in vec3 normal ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * worldNormal, 1.0 );
			return PI * envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	vec3 getIBLRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 reflectVec = reflect( - viewDir, normal );
			reflectVec = normalize( mix( reflectVec, normal, roughness * roughness) );
			reflectVec = inverseTransformDirection( reflectVec, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * reflectVec, roughness );
			return envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	#ifdef USE_ANISOTROPY
		vec3 getIBLAnisotropyRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness, const in vec3 bitangent, const in float anisotropy ) {
			#ifdef ENVMAP_TYPE_CUBE_UV
				vec3 bentNormal = cross( bitangent, viewDir );
				bentNormal = normalize( cross( bentNormal, bitangent ) );
				bentNormal = normalize( mix( bentNormal, normal, pow2( pow2( 1.0 - anisotropy * ( 1.0 - roughness ) ) ) ) );
				return getIBLRadiance( viewDir, bentNormal, roughness );
			#else
				return vec3( 0.0 );
			#endif
		}
	#endif
#endif`,lg=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,hg=`varying vec3 vViewPosition;
struct ToonMaterial {
	vec3 diffuseColor;
};
void RE_Direct_Toon( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	vec3 irradiance = getGradientIrradiance( geometryNormal, directLight.direction ) * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Toon( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Toon
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,ug=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,dg=`varying vec3 vViewPosition;
struct BlinnPhongMaterial {
	vec3 diffuseColor;
	vec3 specularColor;
	float specularShininess;
	float specularStrength;
};
void RE_Direct_BlinnPhong( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
	reflectedLight.directSpecular += irradiance * BRDF_BlinnPhong( directLight.direction, geometryViewDir, geometryNormal, material.specularColor, material.specularShininess ) * material.specularStrength;
}
void RE_IndirectDiffuse_BlinnPhong( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_BlinnPhong
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,fg=`PhysicalMaterial material;
material.diffuseColor = diffuseColor.rgb * ( 1.0 - metalnessFactor );
vec3 dxy = max( abs( dFdx( nonPerturbedNormal ) ), abs( dFdy( nonPerturbedNormal ) ) );
float geometryRoughness = max( max( dxy.x, dxy.y ), dxy.z );
material.roughness = max( roughnessFactor, 0.0525 );material.roughness += geometryRoughness;
material.roughness = min( material.roughness, 1.0 );
#ifdef IOR
	material.ior = ior;
	#ifdef USE_SPECULAR
		float specularIntensityFactor = specularIntensity;
		vec3 specularColorFactor = specularColor;
		#ifdef USE_SPECULAR_COLORMAP
			specularColorFactor *= texture2D( specularColorMap, vSpecularColorMapUv ).rgb;
		#endif
		#ifdef USE_SPECULAR_INTENSITYMAP
			specularIntensityFactor *= texture2D( specularIntensityMap, vSpecularIntensityMapUv ).a;
		#endif
		material.specularF90 = mix( specularIntensityFactor, 1.0, metalnessFactor );
	#else
		float specularIntensityFactor = 1.0;
		vec3 specularColorFactor = vec3( 1.0 );
		material.specularF90 = 1.0;
	#endif
	material.specularColor = mix( min( pow2( ( material.ior - 1.0 ) / ( material.ior + 1.0 ) ) * specularColorFactor, vec3( 1.0 ) ) * specularIntensityFactor, diffuseColor.rgb, metalnessFactor );
#else
	material.specularColor = mix( vec3( 0.04 ), diffuseColor.rgb, metalnessFactor );
	material.specularF90 = 1.0;
#endif
#ifdef USE_CLEARCOAT
	material.clearcoat = clearcoat;
	material.clearcoatRoughness = clearcoatRoughness;
	material.clearcoatF0 = vec3( 0.04 );
	material.clearcoatF90 = 1.0;
	#ifdef USE_CLEARCOATMAP
		material.clearcoat *= texture2D( clearcoatMap, vClearcoatMapUv ).x;
	#endif
	#ifdef USE_CLEARCOAT_ROUGHNESSMAP
		material.clearcoatRoughness *= texture2D( clearcoatRoughnessMap, vClearcoatRoughnessMapUv ).y;
	#endif
	material.clearcoat = saturate( material.clearcoat );	material.clearcoatRoughness = max( material.clearcoatRoughness, 0.0525 );
	material.clearcoatRoughness += geometryRoughness;
	material.clearcoatRoughness = min( material.clearcoatRoughness, 1.0 );
#endif
#ifdef USE_DISPERSION
	material.dispersion = dispersion;
#endif
#ifdef USE_IRIDESCENCE
	material.iridescence = iridescence;
	material.iridescenceIOR = iridescenceIOR;
	#ifdef USE_IRIDESCENCEMAP
		material.iridescence *= texture2D( iridescenceMap, vIridescenceMapUv ).r;
	#endif
	#ifdef USE_IRIDESCENCE_THICKNESSMAP
		material.iridescenceThickness = (iridescenceThicknessMaximum - iridescenceThicknessMinimum) * texture2D( iridescenceThicknessMap, vIridescenceThicknessMapUv ).g + iridescenceThicknessMinimum;
	#else
		material.iridescenceThickness = iridescenceThicknessMaximum;
	#endif
#endif
#ifdef USE_SHEEN
	material.sheenColor = sheenColor;
	#ifdef USE_SHEEN_COLORMAP
		material.sheenColor *= texture2D( sheenColorMap, vSheenColorMapUv ).rgb;
	#endif
	material.sheenRoughness = clamp( sheenRoughness, 0.07, 1.0 );
	#ifdef USE_SHEEN_ROUGHNESSMAP
		material.sheenRoughness *= texture2D( sheenRoughnessMap, vSheenRoughnessMapUv ).a;
	#endif
#endif
#ifdef USE_ANISOTROPY
	#ifdef USE_ANISOTROPYMAP
		mat2 anisotropyMat = mat2( anisotropyVector.x, anisotropyVector.y, - anisotropyVector.y, anisotropyVector.x );
		vec3 anisotropyPolar = texture2D( anisotropyMap, vAnisotropyMapUv ).rgb;
		vec2 anisotropyV = anisotropyMat * normalize( 2.0 * anisotropyPolar.rg - vec2( 1.0 ) ) * anisotropyPolar.b;
	#else
		vec2 anisotropyV = anisotropyVector;
	#endif
	material.anisotropy = length( anisotropyV );
	if( material.anisotropy == 0.0 ) {
		anisotropyV = vec2( 1.0, 0.0 );
	} else {
		anisotropyV /= material.anisotropy;
		material.anisotropy = saturate( material.anisotropy );
	}
	material.alphaT = mix( pow2( material.roughness ), 1.0, pow2( material.anisotropy ) );
	material.anisotropyT = tbn[ 0 ] * anisotropyV.x + tbn[ 1 ] * anisotropyV.y;
	material.anisotropyB = tbn[ 1 ] * anisotropyV.x - tbn[ 0 ] * anisotropyV.y;
#endif`,pg=`struct PhysicalMaterial {
	vec3 diffuseColor;
	float roughness;
	vec3 specularColor;
	float specularF90;
	float dispersion;
	#ifdef USE_CLEARCOAT
		float clearcoat;
		float clearcoatRoughness;
		vec3 clearcoatF0;
		float clearcoatF90;
	#endif
	#ifdef USE_IRIDESCENCE
		float iridescence;
		float iridescenceIOR;
		float iridescenceThickness;
		vec3 iridescenceFresnel;
		vec3 iridescenceF0;
	#endif
	#ifdef USE_SHEEN
		vec3 sheenColor;
		float sheenRoughness;
	#endif
	#ifdef IOR
		float ior;
	#endif
	#ifdef USE_TRANSMISSION
		float transmission;
		float transmissionAlpha;
		float thickness;
		float attenuationDistance;
		vec3 attenuationColor;
	#endif
	#ifdef USE_ANISOTROPY
		float anisotropy;
		float alphaT;
		vec3 anisotropyT;
		vec3 anisotropyB;
	#endif
};
vec3 clearcoatSpecularDirect = vec3( 0.0 );
vec3 clearcoatSpecularIndirect = vec3( 0.0 );
vec3 sheenSpecularDirect = vec3( 0.0 );
vec3 sheenSpecularIndirect = vec3(0.0 );
vec3 Schlick_to_F0( const in vec3 f, const in float f90, const in float dotVH ) {
    float x = clamp( 1.0 - dotVH, 0.0, 1.0 );
    float x2 = x * x;
    float x5 = clamp( x * x2 * x2, 0.0, 0.9999 );
    return ( f - vec3( f90 ) * x5 ) / ( 1.0 - x5 );
}
float V_GGX_SmithCorrelated( const in float alpha, const in float dotNL, const in float dotNV ) {
	float a2 = pow2( alpha );
	float gv = dotNL * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNV ) );
	float gl = dotNV * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNL ) );
	return 0.5 / max( gv + gl, EPSILON );
}
float D_GGX( const in float alpha, const in float dotNH ) {
	float a2 = pow2( alpha );
	float denom = pow2( dotNH ) * ( a2 - 1.0 ) + 1.0;
	return RECIPROCAL_PI * a2 / pow2( denom );
}
#ifdef USE_ANISOTROPY
	float V_GGX_SmithCorrelated_Anisotropic( const in float alphaT, const in float alphaB, const in float dotTV, const in float dotBV, const in float dotTL, const in float dotBL, const in float dotNV, const in float dotNL ) {
		float gv = dotNL * length( vec3( alphaT * dotTV, alphaB * dotBV, dotNV ) );
		float gl = dotNV * length( vec3( alphaT * dotTL, alphaB * dotBL, dotNL ) );
		float v = 0.5 / ( gv + gl );
		return saturate(v);
	}
	float D_GGX_Anisotropic( const in float alphaT, const in float alphaB, const in float dotNH, const in float dotTH, const in float dotBH ) {
		float a2 = alphaT * alphaB;
		highp vec3 v = vec3( alphaB * dotTH, alphaT * dotBH, a2 * dotNH );
		highp float v2 = dot( v, v );
		float w2 = a2 / v2;
		return RECIPROCAL_PI * a2 * pow2 ( w2 );
	}
#endif
#ifdef USE_CLEARCOAT
	vec3 BRDF_GGX_Clearcoat( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material) {
		vec3 f0 = material.clearcoatF0;
		float f90 = material.clearcoatF90;
		float roughness = material.clearcoatRoughness;
		float alpha = pow2( roughness );
		vec3 halfDir = normalize( lightDir + viewDir );
		float dotNL = saturate( dot( normal, lightDir ) );
		float dotNV = saturate( dot( normal, viewDir ) );
		float dotNH = saturate( dot( normal, halfDir ) );
		float dotVH = saturate( dot( viewDir, halfDir ) );
		vec3 F = F_Schlick( f0, f90, dotVH );
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
		return F * ( V * D );
	}
#endif
vec3 BRDF_GGX( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material ) {
	vec3 f0 = material.specularColor;
	float f90 = material.specularF90;
	float roughness = material.roughness;
	float alpha = pow2( roughness );
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( f0, f90, dotVH );
	#ifdef USE_IRIDESCENCE
		F = mix( F, material.iridescenceFresnel, material.iridescence );
	#endif
	#ifdef USE_ANISOTROPY
		float dotTL = dot( material.anisotropyT, lightDir );
		float dotTV = dot( material.anisotropyT, viewDir );
		float dotTH = dot( material.anisotropyT, halfDir );
		float dotBL = dot( material.anisotropyB, lightDir );
		float dotBV = dot( material.anisotropyB, viewDir );
		float dotBH = dot( material.anisotropyB, halfDir );
		float V = V_GGX_SmithCorrelated_Anisotropic( material.alphaT, alpha, dotTV, dotBV, dotTL, dotBL, dotNV, dotNL );
		float D = D_GGX_Anisotropic( material.alphaT, alpha, dotNH, dotTH, dotBH );
	#else
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
	#endif
	return F * ( V * D );
}
vec2 LTC_Uv( const in vec3 N, const in vec3 V, const in float roughness ) {
	const float LUT_SIZE = 64.0;
	const float LUT_SCALE = ( LUT_SIZE - 1.0 ) / LUT_SIZE;
	const float LUT_BIAS = 0.5 / LUT_SIZE;
	float dotNV = saturate( dot( N, V ) );
	vec2 uv = vec2( roughness, sqrt( 1.0 - dotNV ) );
	uv = uv * LUT_SCALE + LUT_BIAS;
	return uv;
}
float LTC_ClippedSphereFormFactor( const in vec3 f ) {
	float l = length( f );
	return max( ( l * l + f.z ) / ( l + 1.0 ), 0.0 );
}
vec3 LTC_EdgeVectorFormFactor( const in vec3 v1, const in vec3 v2 ) {
	float x = dot( v1, v2 );
	float y = abs( x );
	float a = 0.8543985 + ( 0.4965155 + 0.0145206 * y ) * y;
	float b = 3.4175940 + ( 4.1616724 + y ) * y;
	float v = a / b;
	float theta_sintheta = ( x > 0.0 ) ? v : 0.5 * inversesqrt( max( 1.0 - x * x, 1e-7 ) ) - v;
	return cross( v1, v2 ) * theta_sintheta;
}
vec3 LTC_Evaluate( const in vec3 N, const in vec3 V, const in vec3 P, const in mat3 mInv, const in vec3 rectCoords[ 4 ] ) {
	vec3 v1 = rectCoords[ 1 ] - rectCoords[ 0 ];
	vec3 v2 = rectCoords[ 3 ] - rectCoords[ 0 ];
	vec3 lightNormal = cross( v1, v2 );
	if( dot( lightNormal, P - rectCoords[ 0 ] ) < 0.0 ) return vec3( 0.0 );
	vec3 T1, T2;
	T1 = normalize( V - N * dot( V, N ) );
	T2 = - cross( N, T1 );
	mat3 mat = mInv * transposeMat3( mat3( T1, T2, N ) );
	vec3 coords[ 4 ];
	coords[ 0 ] = mat * ( rectCoords[ 0 ] - P );
	coords[ 1 ] = mat * ( rectCoords[ 1 ] - P );
	coords[ 2 ] = mat * ( rectCoords[ 2 ] - P );
	coords[ 3 ] = mat * ( rectCoords[ 3 ] - P );
	coords[ 0 ] = normalize( coords[ 0 ] );
	coords[ 1 ] = normalize( coords[ 1 ] );
	coords[ 2 ] = normalize( coords[ 2 ] );
	coords[ 3 ] = normalize( coords[ 3 ] );
	vec3 vectorFormFactor = vec3( 0.0 );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 0 ], coords[ 1 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 1 ], coords[ 2 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 2 ], coords[ 3 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 3 ], coords[ 0 ] );
	float result = LTC_ClippedSphereFormFactor( vectorFormFactor );
	return vec3( result );
}
#if defined( USE_SHEEN )
float D_Charlie( float roughness, float dotNH ) {
	float alpha = pow2( roughness );
	float invAlpha = 1.0 / alpha;
	float cos2h = dotNH * dotNH;
	float sin2h = max( 1.0 - cos2h, 0.0078125 );
	return ( 2.0 + invAlpha ) * pow( sin2h, invAlpha * 0.5 ) / ( 2.0 * PI );
}
float V_Neubelt( float dotNV, float dotNL ) {
	return saturate( 1.0 / ( 4.0 * ( dotNL + dotNV - dotNL * dotNV ) ) );
}
vec3 BRDF_Sheen( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, vec3 sheenColor, const in float sheenRoughness ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float D = D_Charlie( sheenRoughness, dotNH );
	float V = V_Neubelt( dotNV, dotNL );
	return sheenColor * ( D * V );
}
#endif
float IBLSheenBRDF( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	float r2 = roughness * roughness;
	float a = roughness < 0.25 ? -339.2 * r2 + 161.4 * roughness - 25.9 : -8.48 * r2 + 14.3 * roughness - 9.95;
	float b = roughness < 0.25 ? 44.0 * r2 - 23.7 * roughness + 3.26 : 1.97 * r2 - 3.27 * roughness + 0.72;
	float DG = exp( a * dotNV + b ) + ( roughness < 0.25 ? 0.0 : 0.1 * ( roughness - 0.25 ) );
	return saturate( DG * RECIPROCAL_PI );
}
vec2 DFGApprox( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	const vec4 c0 = vec4( - 1, - 0.0275, - 0.572, 0.022 );
	const vec4 c1 = vec4( 1, 0.0425, 1.04, - 0.04 );
	vec4 r = roughness * c0 + c1;
	float a004 = min( r.x * r.x, exp2( - 9.28 * dotNV ) ) * r.x + r.y;
	vec2 fab = vec2( - 1.04, 1.04 ) * a004 + r.zw;
	return fab;
}
vec3 EnvironmentBRDF( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness ) {
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	return specularColor * fab.x + specularF90 * fab.y;
}
#ifdef USE_IRIDESCENCE
void computeMultiscatteringIridescence( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float iridescence, const in vec3 iridescenceF0, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#else
void computeMultiscattering( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#endif
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	#ifdef USE_IRIDESCENCE
		vec3 Fr = mix( specularColor, iridescenceF0, iridescence );
	#else
		vec3 Fr = specularColor;
	#endif
	vec3 FssEss = Fr * fab.x + specularF90 * fab.y;
	float Ess = fab.x + fab.y;
	float Ems = 1.0 - Ess;
	vec3 Favg = Fr + ( 1.0 - Fr ) * 0.047619;	vec3 Fms = FssEss * Favg / ( 1.0 - Ems * Favg );
	singleScatter += FssEss;
	multiScatter += Fms * Ems;
}
#if NUM_RECT_AREA_LIGHTS > 0
	void RE_Direct_RectArea_Physical( const in RectAreaLight rectAreaLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
		vec3 normal = geometryNormal;
		vec3 viewDir = geometryViewDir;
		vec3 position = geometryPosition;
		vec3 lightPos = rectAreaLight.position;
		vec3 halfWidth = rectAreaLight.halfWidth;
		vec3 halfHeight = rectAreaLight.halfHeight;
		vec3 lightColor = rectAreaLight.color;
		float roughness = material.roughness;
		vec3 rectCoords[ 4 ];
		rectCoords[ 0 ] = lightPos + halfWidth - halfHeight;		rectCoords[ 1 ] = lightPos - halfWidth - halfHeight;
		rectCoords[ 2 ] = lightPos - halfWidth + halfHeight;
		rectCoords[ 3 ] = lightPos + halfWidth + halfHeight;
		vec2 uv = LTC_Uv( normal, viewDir, roughness );
		vec4 t1 = texture2D( ltc_1, uv );
		vec4 t2 = texture2D( ltc_2, uv );
		mat3 mInv = mat3(
			vec3( t1.x, 0, t1.y ),
			vec3(    0, 1,    0 ),
			vec3( t1.z, 0, t1.w )
		);
		vec3 fresnel = ( material.specularColor * t2.x + ( vec3( 1.0 ) - material.specularColor ) * t2.y );
		reflectedLight.directSpecular += lightColor * fresnel * LTC_Evaluate( normal, viewDir, position, mInv, rectCoords );
		reflectedLight.directDiffuse += lightColor * material.diffuseColor * LTC_Evaluate( normal, viewDir, position, mat3( 1.0 ), rectCoords );
	}
#endif
void RE_Direct_Physical( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	#ifdef USE_CLEARCOAT
		float dotNLcc = saturate( dot( geometryClearcoatNormal, directLight.direction ) );
		vec3 ccIrradiance = dotNLcc * directLight.color;
		clearcoatSpecularDirect += ccIrradiance * BRDF_GGX_Clearcoat( directLight.direction, geometryViewDir, geometryClearcoatNormal, material );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularDirect += irradiance * BRDF_Sheen( directLight.direction, geometryViewDir, geometryNormal, material.sheenColor, material.sheenRoughness );
	#endif
	reflectedLight.directSpecular += irradiance * BRDF_GGX( directLight.direction, geometryViewDir, geometryNormal, material );
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Physical( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectSpecular_Physical( const in vec3 radiance, const in vec3 irradiance, const in vec3 clearcoatRadiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight) {
	#ifdef USE_CLEARCOAT
		clearcoatSpecularIndirect += clearcoatRadiance * EnvironmentBRDF( geometryClearcoatNormal, geometryViewDir, material.clearcoatF0, material.clearcoatF90, material.clearcoatRoughness );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularIndirect += irradiance * material.sheenColor * IBLSheenBRDF( geometryNormal, geometryViewDir, material.sheenRoughness );
	#endif
	vec3 singleScattering = vec3( 0.0 );
	vec3 multiScattering = vec3( 0.0 );
	vec3 cosineWeightedIrradiance = irradiance * RECIPROCAL_PI;
	#ifdef USE_IRIDESCENCE
		computeMultiscatteringIridescence( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.iridescence, material.iridescenceFresnel, material.roughness, singleScattering, multiScattering );
	#else
		computeMultiscattering( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.roughness, singleScattering, multiScattering );
	#endif
	vec3 totalScattering = singleScattering + multiScattering;
	vec3 diffuse = material.diffuseColor * ( 1.0 - max( max( totalScattering.r, totalScattering.g ), totalScattering.b ) );
	reflectedLight.indirectSpecular += radiance * singleScattering;
	reflectedLight.indirectSpecular += multiScattering * cosineWeightedIrradiance;
	reflectedLight.indirectDiffuse += diffuse * cosineWeightedIrradiance;
}
#define RE_Direct				RE_Direct_Physical
#define RE_Direct_RectArea		RE_Direct_RectArea_Physical
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Physical
#define RE_IndirectSpecular		RE_IndirectSpecular_Physical
float computeSpecularOcclusion( const in float dotNV, const in float ambientOcclusion, const in float roughness ) {
	return saturate( pow( dotNV + ambientOcclusion, exp2( - 16.0 * roughness - 1.0 ) ) - 1.0 + ambientOcclusion );
}`,mg=`
vec3 geometryPosition = - vViewPosition;
vec3 geometryNormal = normal;
vec3 geometryViewDir = ( isOrthographic ) ? vec3( 0, 0, 1 ) : normalize( vViewPosition );
vec3 geometryClearcoatNormal = vec3( 0.0 );
#ifdef USE_CLEARCOAT
	geometryClearcoatNormal = clearcoatNormal;
#endif
#ifdef USE_IRIDESCENCE
	float dotNVi = saturate( dot( normal, geometryViewDir ) );
	if ( material.iridescenceThickness == 0.0 ) {
		material.iridescence = 0.0;
	} else {
		material.iridescence = saturate( material.iridescence );
	}
	if ( material.iridescence > 0.0 ) {
		material.iridescenceFresnel = evalIridescence( 1.0, material.iridescenceIOR, dotNVi, material.iridescenceThickness, material.specularColor );
		material.iridescenceF0 = Schlick_to_F0( material.iridescenceFresnel, 1.0, dotNVi );
	}
#endif
IncidentLight directLight;
#if ( NUM_POINT_LIGHTS > 0 ) && defined( RE_Direct )
	PointLight pointLight;
	#if defined( USE_SHADOWMAP ) && NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHTS; i ++ ) {
		pointLight = pointLights[ i ];
		getPointLightInfo( pointLight, geometryPosition, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_POINT_LIGHT_SHADOWS )
		pointLightShadow = pointLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getPointShadow( pointShadowMap[ i ], pointLightShadow.shadowMapSize, pointLightShadow.shadowIntensity, pointLightShadow.shadowBias, pointLightShadow.shadowRadius, vPointShadowCoord[ i ], pointLightShadow.shadowCameraNear, pointLightShadow.shadowCameraFar ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_SPOT_LIGHTS > 0 ) && defined( RE_Direct )
	SpotLight spotLight;
	vec4 spotColor;
	vec3 spotLightCoord;
	bool inSpotLightMap;
	#if defined( USE_SHADOWMAP ) && NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHTS; i ++ ) {
		spotLight = spotLights[ i ];
		getSpotLightInfo( spotLight, geometryPosition, directLight );
		#if ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#define SPOT_LIGHT_MAP_INDEX UNROLLED_LOOP_INDEX
		#elif ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		#define SPOT_LIGHT_MAP_INDEX NUM_SPOT_LIGHT_MAPS
		#else
		#define SPOT_LIGHT_MAP_INDEX ( UNROLLED_LOOP_INDEX - NUM_SPOT_LIGHT_SHADOWS + NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#endif
		#if ( SPOT_LIGHT_MAP_INDEX < NUM_SPOT_LIGHT_MAPS )
			spotLightCoord = vSpotLightCoord[ i ].xyz / vSpotLightCoord[ i ].w;
			inSpotLightMap = all( lessThan( abs( spotLightCoord * 2. - 1. ), vec3( 1.0 ) ) );
			spotColor = texture2D( spotLightMap[ SPOT_LIGHT_MAP_INDEX ], spotLightCoord.xy );
			directLight.color = inSpotLightMap ? directLight.color * spotColor.rgb : directLight.color;
		#endif
		#undef SPOT_LIGHT_MAP_INDEX
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		spotLightShadow = spotLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( spotShadowMap[ i ], spotLightShadow.shadowMapSize, spotLightShadow.shadowIntensity, spotLightShadow.shadowBias, spotLightShadow.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_DIR_LIGHTS > 0 ) && defined( RE_Direct )
	DirectionalLight directionalLight;
	#if defined( USE_SHADOWMAP ) && NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHTS; i ++ ) {
		directionalLight = directionalLights[ i ];
		getDirectionalLightInfo( directionalLight, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_DIR_LIGHT_SHADOWS )
		directionalLightShadow = directionalLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( directionalShadowMap[ i ], directionalLightShadow.shadowMapSize, directionalLightShadow.shadowIntensity, directionalLightShadow.shadowBias, directionalLightShadow.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_RECT_AREA_LIGHTS > 0 ) && defined( RE_Direct_RectArea )
	RectAreaLight rectAreaLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_RECT_AREA_LIGHTS; i ++ ) {
		rectAreaLight = rectAreaLights[ i ];
		RE_Direct_RectArea( rectAreaLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if defined( RE_IndirectDiffuse )
	vec3 iblIrradiance = vec3( 0.0 );
	vec3 irradiance = getAmbientLightIrradiance( ambientLightColor );
	#if defined( USE_LIGHT_PROBES )
		irradiance += getLightProbeIrradiance( lightProbe, geometryNormal );
	#endif
	#if ( NUM_HEMI_LIGHTS > 0 )
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_HEMI_LIGHTS; i ++ ) {
			irradiance += getHemisphereLightIrradiance( hemisphereLights[ i ], geometryNormal );
		}
		#pragma unroll_loop_end
	#endif
#endif
#if defined( RE_IndirectSpecular )
	vec3 radiance = vec3( 0.0 );
	vec3 clearcoatRadiance = vec3( 0.0 );
#endif`,gg=`#if defined( RE_IndirectDiffuse )
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
		irradiance += lightMapIrradiance;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD ) && defined( ENVMAP_TYPE_CUBE_UV )
		iblIrradiance += getIBLIrradiance( geometryNormal );
	#endif
#endif
#if defined( USE_ENVMAP ) && defined( RE_IndirectSpecular )
	#ifdef USE_ANISOTROPY
		radiance += getIBLAnisotropyRadiance( geometryViewDir, geometryNormal, material.roughness, material.anisotropyB, material.anisotropy );
	#else
		radiance += getIBLRadiance( geometryViewDir, geometryNormal, material.roughness );
	#endif
	#ifdef USE_CLEARCOAT
		clearcoatRadiance += getIBLRadiance( geometryViewDir, geometryClearcoatNormal, material.clearcoatRoughness );
	#endif
#endif`,_g=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,xg=`#if defined( USE_LOGARITHMIC_DEPTH_BUFFER )
	gl_FragDepth = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,yg=`#if defined( USE_LOGARITHMIC_DEPTH_BUFFER )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,vg=`#ifdef USE_LOGARITHMIC_DEPTH_BUFFER
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,bg=`#ifdef USE_LOGARITHMIC_DEPTH_BUFFER
	vFragDepth = 1.0 + gl_Position.w;
	vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
#endif`,Mg=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = sRGBTransferEOTF( sampledDiffuseColor );
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,Sg=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,Eg=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
	#if defined( USE_POINTS_UV )
		vec2 uv = vUv;
	#else
		vec2 uv = ( uvTransform * vec3( gl_PointCoord.x, 1.0 - gl_PointCoord.y, 1 ) ).xy;
	#endif
#endif
#ifdef USE_MAP
	diffuseColor *= texture2D( map, uv );
#endif
#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, uv ).g;
#endif`,Tg=`#if defined( USE_POINTS_UV )
	varying vec2 vUv;
#else
	#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
		uniform mat3 uvTransform;
	#endif
#endif
#ifdef USE_MAP
	uniform sampler2D map;
#endif
#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,wg=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,Ag=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,Rg=`#ifdef USE_INSTANCING_MORPH
	float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	float morphTargetBaseInfluence = texelFetch( morphTexture, ivec2( 0, gl_InstanceID ), 0 ).r;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		morphTargetInfluences[i] =  texelFetch( morphTexture, ivec2( i + 1, gl_InstanceID ), 0 ).r;
	}
#endif`,Cg=`#if defined( USE_MORPHCOLORS )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,Ig=`#ifdef USE_MORPHNORMALS
	objectNormal *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) objectNormal += getMorph( gl_VertexID, i, 1 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,Pg=`#ifdef USE_MORPHTARGETS
	#ifndef USE_INSTANCING_MORPH
		uniform float morphTargetBaseInfluence;
		uniform float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	#endif
	uniform sampler2DArray morphTargetsTexture;
	uniform ivec2 morphTargetsTextureSize;
	vec4 getMorph( const in int vertexIndex, const in int morphTargetIndex, const in int offset ) {
		int texelIndex = vertexIndex * MORPHTARGETS_TEXTURE_STRIDE + offset;
		int y = texelIndex / morphTargetsTextureSize.x;
		int x = texelIndex - y * morphTargetsTextureSize.x;
		ivec3 morphUV = ivec3( x, y, morphTargetIndex );
		return texelFetch( morphTargetsTexture, morphUV, 0 );
	}
#endif`,Lg=`#ifdef USE_MORPHTARGETS
	transformed *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) transformed += getMorph( gl_VertexID, i, 0 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,Ng=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
#ifdef FLAT_SHADED
	vec3 fdx = dFdx( vViewPosition );
	vec3 fdy = dFdy( vViewPosition );
	vec3 normal = normalize( cross( fdx, fdy ) );
#else
	vec3 normal = normalize( vNormal );
	#ifdef DOUBLE_SIDED
		normal *= faceDirection;
	#endif
#endif
#if defined( USE_NORMALMAP_TANGENTSPACE ) || defined( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY )
	#ifdef USE_TANGENT
		mat3 tbn = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn = getTangentFrame( - vViewPosition, normal,
		#if defined( USE_NORMALMAP )
			vNormalMapUv
		#elif defined( USE_CLEARCOAT_NORMALMAP )
			vClearcoatNormalMapUv
		#else
			vUv
		#endif
		);
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn[0] *= faceDirection;
		tbn[1] *= faceDirection;
	#endif
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	#ifdef USE_TANGENT
		mat3 tbn2 = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn2 = getTangentFrame( - vViewPosition, normal, vClearcoatNormalMapUv );
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn2[0] *= faceDirection;
		tbn2[1] *= faceDirection;
	#endif
#endif
vec3 nonPerturbedNormal = normal;`,Dg=`#ifdef USE_NORMALMAP_OBJECTSPACE
	normal = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	#ifdef FLIP_SIDED
		normal = - normal;
	#endif
	#ifdef DOUBLE_SIDED
		normal = normal * faceDirection;
	#endif
	normal = normalize( normalMatrix * normal );
#elif defined( USE_NORMALMAP_TANGENTSPACE )
	vec3 mapN = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	mapN.xy *= normalScale;
	normal = normalize( tbn * mapN );
#elif defined( USE_BUMPMAP )
	normal = perturbNormalArb( - vViewPosition, normal, dHdxy_fwd(), faceDirection );
#endif`,Ug=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,Og=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,Fg=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,Bg=`#ifdef USE_NORMALMAP
	uniform sampler2D normalMap;
	uniform vec2 normalScale;
#endif
#ifdef USE_NORMALMAP_OBJECTSPACE
	uniform mat3 normalMatrix;
#endif
#if ! defined ( USE_TANGENT ) && ( defined ( USE_NORMALMAP_TANGENTSPACE ) || defined ( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY ) )
	mat3 getTangentFrame( vec3 eye_pos, vec3 surf_norm, vec2 uv ) {
		vec3 q0 = dFdx( eye_pos.xyz );
		vec3 q1 = dFdy( eye_pos.xyz );
		vec2 st0 = dFdx( uv.st );
		vec2 st1 = dFdy( uv.st );
		vec3 N = surf_norm;
		vec3 q1perp = cross( q1, N );
		vec3 q0perp = cross( N, q0 );
		vec3 T = q1perp * st0.x + q0perp * st1.x;
		vec3 B = q1perp * st0.y + q0perp * st1.y;
		float det = max( dot( T, T ), dot( B, B ) );
		float scale = ( det == 0.0 ) ? 0.0 : inversesqrt( det );
		return mat3( T * scale, B * scale, N );
	}
#endif`,kg=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,zg=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,Hg=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,Vg=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,Gg=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,Wg=`vec3 packNormalToRGB( const in vec3 normal ) {
	return normalize( normal ) * 0.5 + 0.5;
}
vec3 unpackRGBToNormal( const in vec3 rgb ) {
	return 2.0 * rgb.xyz - 1.0;
}
const float PackUpscale = 256. / 255.;const float UnpackDownscale = 255. / 256.;const float ShiftRight8 = 1. / 256.;
const float Inv255 = 1. / 255.;
const vec4 PackFactors = vec4( 1.0, 256.0, 256.0 * 256.0, 256.0 * 256.0 * 256.0 );
const vec2 UnpackFactors2 = vec2( UnpackDownscale, 1.0 / PackFactors.g );
const vec3 UnpackFactors3 = vec3( UnpackDownscale / PackFactors.rg, 1.0 / PackFactors.b );
const vec4 UnpackFactors4 = vec4( UnpackDownscale / PackFactors.rgb, 1.0 / PackFactors.a );
vec4 packDepthToRGBA( const in float v ) {
	if( v <= 0.0 )
		return vec4( 0., 0., 0., 0. );
	if( v >= 1.0 )
		return vec4( 1., 1., 1., 1. );
	float vuf;
	float af = modf( v * PackFactors.a, vuf );
	float bf = modf( vuf * ShiftRight8, vuf );
	float gf = modf( vuf * ShiftRight8, vuf );
	return vec4( vuf * Inv255, gf * PackUpscale, bf * PackUpscale, af );
}
vec3 packDepthToRGB( const in float v ) {
	if( v <= 0.0 )
		return vec3( 0., 0., 0. );
	if( v >= 1.0 )
		return vec3( 1., 1., 1. );
	float vuf;
	float bf = modf( v * PackFactors.b, vuf );
	float gf = modf( vuf * ShiftRight8, vuf );
	return vec3( vuf * Inv255, gf * PackUpscale, bf );
}
vec2 packDepthToRG( const in float v ) {
	if( v <= 0.0 )
		return vec2( 0., 0. );
	if( v >= 1.0 )
		return vec2( 1., 1. );
	float vuf;
	float gf = modf( v * 256., vuf );
	return vec2( vuf * Inv255, gf );
}
float unpackRGBAToDepth( const in vec4 v ) {
	return dot( v, UnpackFactors4 );
}
float unpackRGBToDepth( const in vec3 v ) {
	return dot( v, UnpackFactors3 );
}
float unpackRGToDepth( const in vec2 v ) {
	return v.r * UnpackFactors2.r + v.g * UnpackFactors2.g;
}
vec4 pack2HalfToRGBA( const in vec2 v ) {
	vec4 r = vec4( v.x, fract( v.x * 255.0 ), v.y, fract( v.y * 255.0 ) );
	return vec4( r.x - r.y / 255.0, r.y, r.z - r.w / 255.0, r.w );
}
vec2 unpackRGBATo2Half( const in vec4 v ) {
	return vec2( v.x + ( v.y / 255.0 ), v.z + ( v.w / 255.0 ) );
}
float viewZToOrthographicDepth( const in float viewZ, const in float near, const in float far ) {
	return ( viewZ + near ) / ( near - far );
}
float orthographicDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return depth * ( near - far ) - near;
}
float viewZToPerspectiveDepth( const in float viewZ, const in float near, const in float far ) {
	return ( ( near + viewZ ) * far ) / ( ( far - near ) * viewZ );
}
float perspectiveDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return ( near * far ) / ( ( far - near ) * depth - far );
}`,Xg=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,qg=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,Yg=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,jg=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,Kg=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,Zg=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,Jg=`#if NUM_SPOT_LIGHT_COORDS > 0
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#if NUM_SPOT_LIGHT_MAPS > 0
	uniform sampler2D spotLightMap[ NUM_SPOT_LIGHT_MAPS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform sampler2D directionalShadowMap[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		uniform sampler2D spotShadowMap[ NUM_SPOT_LIGHT_SHADOWS ];
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform sampler2D pointShadowMap[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
	float texture2DCompare( sampler2D depths, vec2 uv, float compare ) {
		float depth = unpackRGBAToDepth( texture2D( depths, uv ) );
		#ifdef USE_REVERSED_DEPTH_BUFFER
			return step( depth, compare );
		#else
			return step( compare, depth );
		#endif
	}
	vec2 texture2DDistribution( sampler2D shadow, vec2 uv ) {
		return unpackRGBATo2Half( texture2D( shadow, uv ) );
	}
	float VSMShadow( sampler2D shadow, vec2 uv, float compare ) {
		float occlusion = 1.0;
		vec2 distribution = texture2DDistribution( shadow, uv );
		#ifdef USE_REVERSED_DEPTH_BUFFER
			float hard_shadow = step( distribution.x, compare );
		#else
			float hard_shadow = step( compare, distribution.x );
		#endif
		if ( hard_shadow != 1.0 ) {
			float distance = compare - distribution.x;
			float variance = max( 0.00000, distribution.y * distribution.y );
			float softness_probability = variance / (variance + distance * distance );			softness_probability = clamp( ( softness_probability - 0.3 ) / ( 0.95 - 0.3 ), 0.0, 1.0 );			occlusion = clamp( max( hard_shadow, softness_probability ), 0.0, 1.0 );
		}
		return occlusion;
	}
	float getShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord ) {
		float shadow = 1.0;
		shadowCoord.xyz /= shadowCoord.w;
		shadowCoord.z += shadowBias;
		bool inFrustum = shadowCoord.x >= 0.0 && shadowCoord.x <= 1.0 && shadowCoord.y >= 0.0 && shadowCoord.y <= 1.0;
		bool frustumTest = inFrustum && shadowCoord.z <= 1.0;
		if ( frustumTest ) {
		#if defined( SHADOWMAP_TYPE_PCF )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx0 = - texelSize.x * shadowRadius;
			float dy0 = - texelSize.y * shadowRadius;
			float dx1 = + texelSize.x * shadowRadius;
			float dy1 = + texelSize.y * shadowRadius;
			float dx2 = dx0 / 2.0;
			float dy2 = dy0 / 2.0;
			float dx3 = dx1 / 2.0;
			float dy3 = dy1 / 2.0;
			shadow = (
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy1 ), shadowCoord.z )
			) * ( 1.0 / 17.0 );
		#elif defined( SHADOWMAP_TYPE_PCF_SOFT )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx = texelSize.x;
			float dy = texelSize.y;
			vec2 uv = shadowCoord.xy;
			vec2 f = fract( uv * shadowMapSize + 0.5 );
			uv -= f * texelSize;
			shadow = (
				texture2DCompare( shadowMap, uv, shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( dx, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( 0.0, dy ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + texelSize, shadowCoord.z ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, 0.0 ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 0.0 ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, dy ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( 0.0, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 0.0, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( texture2DCompare( shadowMap, uv + vec2( dx, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( dx, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( mix( texture2DCompare( shadowMap, uv + vec2( -dx, -dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, -dy ), shadowCoord.z ),
						  f.x ),
					 mix( texture2DCompare( shadowMap, uv + vec2( -dx, 2.0 * dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 2.0 * dy ), shadowCoord.z ),
						  f.x ),
					 f.y )
			) * ( 1.0 / 9.0 );
		#elif defined( SHADOWMAP_TYPE_VSM )
			shadow = VSMShadow( shadowMap, shadowCoord.xy, shadowCoord.z );
		#else
			shadow = texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z );
		#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
	vec2 cubeToUV( vec3 v, float texelSizeY ) {
		vec3 absV = abs( v );
		float scaleToCube = 1.0 / max( absV.x, max( absV.y, absV.z ) );
		absV *= scaleToCube;
		v *= scaleToCube * ( 1.0 - 2.0 * texelSizeY );
		vec2 planar = v.xy;
		float almostATexel = 1.5 * texelSizeY;
		float almostOne = 1.0 - almostATexel;
		if ( absV.z >= almostOne ) {
			if ( v.z > 0.0 )
				planar.x = 4.0 - v.x;
		} else if ( absV.x >= almostOne ) {
			float signX = sign( v.x );
			planar.x = v.z * signX + 2.0 * signX;
		} else if ( absV.y >= almostOne ) {
			float signY = sign( v.y );
			planar.x = v.x + 2.0 * signY + 2.0;
			planar.y = v.z * signY - 2.0;
		}
		return vec2( 0.125, 0.25 ) * planar + vec2( 0.375, 0.75 );
	}
	float getPointShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord, float shadowCameraNear, float shadowCameraFar ) {
		float shadow = 1.0;
		vec3 lightToPosition = shadowCoord.xyz;

		float lightToPositionLength = length( lightToPosition );
		if ( lightToPositionLength - shadowCameraFar <= 0.0 && lightToPositionLength - shadowCameraNear >= 0.0 ) {
			float dp = ( lightToPositionLength - shadowCameraNear ) / ( shadowCameraFar - shadowCameraNear );			dp += shadowBias;
			vec3 bd3D = normalize( lightToPosition );
			vec2 texelSize = vec2( 1.0 ) / ( shadowMapSize * vec2( 4.0, 2.0 ) );
			#if defined( SHADOWMAP_TYPE_PCF ) || defined( SHADOWMAP_TYPE_PCF_SOFT ) || defined( SHADOWMAP_TYPE_VSM )
				vec2 offset = vec2( - 1, 1 ) * shadowRadius * texelSize.y;
				shadow = (
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxx, texelSize.y ), dp )
				) * ( 1.0 / 9.0 );
			#else
				shadow = texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp );
			#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
#endif`,$g=`#if NUM_SPOT_LIGHT_COORDS > 0
	uniform mat4 spotLightMatrix[ NUM_SPOT_LIGHT_COORDS ];
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform mat4 directionalShadowMatrix[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform mat4 pointShadowMatrix[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
#endif`,Qg=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
	vec3 shadowWorldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
	vec4 shadowWorldPosition;
#endif
#if defined( USE_SHADOWMAP )
	#if NUM_DIR_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * directionalLightShadows[ i ].shadowNormalBias, 0 );
			vDirectionalShadowCoord[ i ] = directionalShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * pointLightShadows[ i ].shadowNormalBias, 0 );
			vPointShadowCoord[ i ] = pointShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
#endif
#if NUM_SPOT_LIGHT_COORDS > 0
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_COORDS; i ++ ) {
		shadowWorldPosition = worldPosition;
		#if ( defined( USE_SHADOWMAP ) && UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
			shadowWorldPosition.xyz += shadowWorldNormal * spotLightShadows[ i ].shadowNormalBias;
		#endif
		vSpotLightCoord[ i ] = spotLightMatrix[ i ] * shadowWorldPosition;
	}
	#pragma unroll_loop_end
#endif`,e_=`float getShadowMask() {
	float shadow = 1.0;
	#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
		directionalLight = directionalLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( directionalShadowMap[ i ], directionalLight.shadowMapSize, directionalLight.shadowIntensity, directionalLight.shadowBias, directionalLight.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_SHADOWS; i ++ ) {
		spotLight = spotLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( spotShadowMap[ i ], spotLight.shadowMapSize, spotLight.shadowIntensity, spotLight.shadowBias, spotLight.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
		pointLight = pointLightShadows[ i ];
		shadow *= receiveShadow ? getPointShadow( pointShadowMap[ i ], pointLight.shadowMapSize, pointLight.shadowIntensity, pointLight.shadowBias, pointLight.shadowRadius, vPointShadowCoord[ i ], pointLight.shadowCameraNear, pointLight.shadowCameraFar ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#endif
	return shadow;
}`,t_=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,n_=`#ifdef USE_SKINNING
	uniform mat4 bindMatrix;
	uniform mat4 bindMatrixInverse;
	uniform highp sampler2D boneTexture;
	mat4 getBoneMatrix( const in float i ) {
		int size = textureSize( boneTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( boneTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( boneTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( boneTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( boneTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
#endif`,i_=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,s_=`#ifdef USE_SKINNING
	mat4 skinMatrix = mat4( 0.0 );
	skinMatrix += skinWeight.x * boneMatX;
	skinMatrix += skinWeight.y * boneMatY;
	skinMatrix += skinWeight.z * boneMatZ;
	skinMatrix += skinWeight.w * boneMatW;
	skinMatrix = bindMatrixInverse * skinMatrix * bindMatrix;
	objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;
	#ifdef USE_TANGENT
		objectTangent = vec4( skinMatrix * vec4( objectTangent, 0.0 ) ).xyz;
	#endif
#endif`,r_=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,o_=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,a_=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,c_=`#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
uniform float toneMappingExposure;
vec3 LinearToneMapping( vec3 color ) {
	return saturate( toneMappingExposure * color );
}
vec3 ReinhardToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	return saturate( color / ( vec3( 1.0 ) + color ) );
}
vec3 CineonToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	color = max( vec3( 0.0 ), color - 0.004 );
	return pow( ( color * ( 6.2 * color + 0.5 ) ) / ( color * ( 6.2 * color + 1.7 ) + 0.06 ), vec3( 2.2 ) );
}
vec3 RRTAndODTFit( vec3 v ) {
	vec3 a = v * ( v + 0.0245786 ) - 0.000090537;
	vec3 b = v * ( 0.983729 * v + 0.4329510 ) + 0.238081;
	return a / b;
}
vec3 ACESFilmicToneMapping( vec3 color ) {
	const mat3 ACESInputMat = mat3(
		vec3( 0.59719, 0.07600, 0.02840 ),		vec3( 0.35458, 0.90834, 0.13383 ),
		vec3( 0.04823, 0.01566, 0.83777 )
	);
	const mat3 ACESOutputMat = mat3(
		vec3(  1.60475, -0.10208, -0.00327 ),		vec3( -0.53108,  1.10813, -0.07276 ),
		vec3( -0.07367, -0.00605,  1.07602 )
	);
	color *= toneMappingExposure / 0.6;
	color = ACESInputMat * color;
	color = RRTAndODTFit( color );
	color = ACESOutputMat * color;
	return saturate( color );
}
const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(
	vec3( 1.6605, - 0.1246, - 0.0182 ),
	vec3( - 0.5876, 1.1329, - 0.1006 ),
	vec3( - 0.0728, - 0.0083, 1.1187 )
);
const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(
	vec3( 0.6274, 0.0691, 0.0164 ),
	vec3( 0.3293, 0.9195, 0.0880 ),
	vec3( 0.0433, 0.0113, 0.8956 )
);
vec3 agxDefaultContrastApprox( vec3 x ) {
	vec3 x2 = x * x;
	vec3 x4 = x2 * x2;
	return + 15.5 * x4 * x2
		- 40.14 * x4 * x
		+ 31.96 * x4
		- 6.868 * x2 * x
		+ 0.4298 * x2
		+ 0.1191 * x
		- 0.00232;
}
vec3 AgXToneMapping( vec3 color ) {
	const mat3 AgXInsetMatrix = mat3(
		vec3( 0.856627153315983, 0.137318972929847, 0.11189821299995 ),
		vec3( 0.0951212405381588, 0.761241990602591, 0.0767994186031903 ),
		vec3( 0.0482516061458583, 0.101439036467562, 0.811302368396859 )
	);
	const mat3 AgXOutsetMatrix = mat3(
		vec3( 1.1271005818144368, - 0.1413297634984383, - 0.14132976349843826 ),
		vec3( - 0.11060664309660323, 1.157823702216272, - 0.11060664309660294 ),
		vec3( - 0.016493938717834573, - 0.016493938717834257, 1.2519364065950405 )
	);
	const float AgxMinEv = - 12.47393;	const float AgxMaxEv = 4.026069;
	color *= toneMappingExposure;
	color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
	color = AgXInsetMatrix * color;
	color = max( color, 1e-10 );	color = log2( color );
	color = ( color - AgxMinEv ) / ( AgxMaxEv - AgxMinEv );
	color = clamp( color, 0.0, 1.0 );
	color = agxDefaultContrastApprox( color );
	color = AgXOutsetMatrix * color;
	color = pow( max( vec3( 0.0 ), color ), vec3( 2.2 ) );
	color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
	color = clamp( color, 0.0, 1.0 );
	return color;
}
vec3 NeutralToneMapping( vec3 color ) {
	const float StartCompression = 0.8 - 0.04;
	const float Desaturation = 0.15;
	color *= toneMappingExposure;
	float x = min( color.r, min( color.g, color.b ) );
	float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
	color -= offset;
	float peak = max( color.r, max( color.g, color.b ) );
	if ( peak < StartCompression ) return color;
	float d = 1. - StartCompression;
	float newPeak = 1. - d * d / ( peak + d - StartCompression );
	color *= newPeak / peak;
	float g = 1. - 1. / ( Desaturation * ( peak - newPeak ) + 1. );
	return mix( color, vec3( newPeak ), g );
}
vec3 CustomToneMapping( vec3 color ) { return color; }`,l_=`#ifdef USE_TRANSMISSION
	material.transmission = transmission;
	material.transmissionAlpha = 1.0;
	material.thickness = thickness;
	material.attenuationDistance = attenuationDistance;
	material.attenuationColor = attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		material.transmission *= texture2D( transmissionMap, vTransmissionMapUv ).r;
	#endif
	#ifdef USE_THICKNESSMAP
		material.thickness *= texture2D( thicknessMap, vThicknessMapUv ).g;
	#endif
	vec3 pos = vWorldPosition;
	vec3 v = normalize( cameraPosition - pos );
	vec3 n = inverseTransformDirection( normal, viewMatrix );
	vec4 transmitted = getIBLVolumeRefraction(
		n, v, material.roughness, material.diffuseColor, material.specularColor, material.specularF90,
		pos, modelMatrix, viewMatrix, projectionMatrix, material.dispersion, material.ior, material.thickness,
		material.attenuationColor, material.attenuationDistance );
	material.transmissionAlpha = mix( material.transmissionAlpha, transmitted.a, material.transmission );
	totalDiffuse = mix( totalDiffuse, transmitted.rgb, material.transmission );
#endif`,h_=`#ifdef USE_TRANSMISSION
	uniform float transmission;
	uniform float thickness;
	uniform float attenuationDistance;
	uniform vec3 attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		uniform sampler2D transmissionMap;
	#endif
	#ifdef USE_THICKNESSMAP
		uniform sampler2D thicknessMap;
	#endif
	uniform vec2 transmissionSamplerSize;
	uniform sampler2D transmissionSamplerMap;
	uniform mat4 modelMatrix;
	uniform mat4 projectionMatrix;
	varying vec3 vWorldPosition;
	float w0( float a ) {
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - a + 3.0 ) - 3.0 ) + 1.0 );
	}
	float w1( float a ) {
		return ( 1.0 / 6.0 ) * ( a *  a * ( 3.0 * a - 6.0 ) + 4.0 );
	}
	float w2( float a ){
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - 3.0 * a + 3.0 ) + 3.0 ) + 1.0 );
	}
	float w3( float a ) {
		return ( 1.0 / 6.0 ) * ( a * a * a );
	}
	float g0( float a ) {
		return w0( a ) + w1( a );
	}
	float g1( float a ) {
		return w2( a ) + w3( a );
	}
	float h0( float a ) {
		return - 1.0 + w1( a ) / ( w0( a ) + w1( a ) );
	}
	float h1( float a ) {
		return 1.0 + w3( a ) / ( w2( a ) + w3( a ) );
	}
	vec4 bicubic( sampler2D tex, vec2 uv, vec4 texelSize, float lod ) {
		uv = uv * texelSize.zw + 0.5;
		vec2 iuv = floor( uv );
		vec2 fuv = fract( uv );
		float g0x = g0( fuv.x );
		float g1x = g1( fuv.x );
		float h0x = h0( fuv.x );
		float h1x = h1( fuv.x );
		float h0y = h0( fuv.y );
		float h1y = h1( fuv.y );
		vec2 p0 = ( vec2( iuv.x + h0x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p1 = ( vec2( iuv.x + h1x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p2 = ( vec2( iuv.x + h0x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		vec2 p3 = ( vec2( iuv.x + h1x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		return g0( fuv.y ) * ( g0x * textureLod( tex, p0, lod ) + g1x * textureLod( tex, p1, lod ) ) +
			g1( fuv.y ) * ( g0x * textureLod( tex, p2, lod ) + g1x * textureLod( tex, p3, lod ) );
	}
	vec4 textureBicubic( sampler2D sampler, vec2 uv, float lod ) {
		vec2 fLodSize = vec2( textureSize( sampler, int( lod ) ) );
		vec2 cLodSize = vec2( textureSize( sampler, int( lod + 1.0 ) ) );
		vec2 fLodSizeInv = 1.0 / fLodSize;
		vec2 cLodSizeInv = 1.0 / cLodSize;
		vec4 fSample = bicubic( sampler, uv, vec4( fLodSizeInv, fLodSize ), floor( lod ) );
		vec4 cSample = bicubic( sampler, uv, vec4( cLodSizeInv, cLodSize ), ceil( lod ) );
		return mix( fSample, cSample, fract( lod ) );
	}
	vec3 getVolumeTransmissionRay( const in vec3 n, const in vec3 v, const in float thickness, const in float ior, const in mat4 modelMatrix ) {
		vec3 refractionVector = refract( - v, normalize( n ), 1.0 / ior );
		vec3 modelScale;
		modelScale.x = length( vec3( modelMatrix[ 0 ].xyz ) );
		modelScale.y = length( vec3( modelMatrix[ 1 ].xyz ) );
		modelScale.z = length( vec3( modelMatrix[ 2 ].xyz ) );
		return normalize( refractionVector ) * thickness * modelScale;
	}
	float applyIorToRoughness( const in float roughness, const in float ior ) {
		return roughness * clamp( ior * 2.0 - 2.0, 0.0, 1.0 );
	}
	vec4 getTransmissionSample( const in vec2 fragCoord, const in float roughness, const in float ior ) {
		float lod = log2( transmissionSamplerSize.x ) * applyIorToRoughness( roughness, ior );
		return textureBicubic( transmissionSamplerMap, fragCoord.xy, lod );
	}
	vec3 volumeAttenuation( const in float transmissionDistance, const in vec3 attenuationColor, const in float attenuationDistance ) {
		if ( isinf( attenuationDistance ) ) {
			return vec3( 1.0 );
		} else {
			vec3 attenuationCoefficient = -log( attenuationColor ) / attenuationDistance;
			vec3 transmittance = exp( - attenuationCoefficient * transmissionDistance );			return transmittance;
		}
	}
	vec4 getIBLVolumeRefraction( const in vec3 n, const in vec3 v, const in float roughness, const in vec3 diffuseColor,
		const in vec3 specularColor, const in float specularF90, const in vec3 position, const in mat4 modelMatrix,
		const in mat4 viewMatrix, const in mat4 projMatrix, const in float dispersion, const in float ior, const in float thickness,
		const in vec3 attenuationColor, const in float attenuationDistance ) {
		vec4 transmittedLight;
		vec3 transmittance;
		#ifdef USE_DISPERSION
			float halfSpread = ( ior - 1.0 ) * 0.025 * dispersion;
			vec3 iors = vec3( ior - halfSpread, ior, ior + halfSpread );
			for ( int i = 0; i < 3; i ++ ) {
				vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, iors[ i ], modelMatrix );
				vec3 refractedRayExit = position + transmissionRay;
				vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
				vec2 refractionCoords = ndcPos.xy / ndcPos.w;
				refractionCoords += 1.0;
				refractionCoords /= 2.0;
				vec4 transmissionSample = getTransmissionSample( refractionCoords, roughness, iors[ i ] );
				transmittedLight[ i ] = transmissionSample[ i ];
				transmittedLight.a += transmissionSample.a;
				transmittance[ i ] = diffuseColor[ i ] * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance )[ i ];
			}
			transmittedLight.a /= 3.0;
		#else
			vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, ior, modelMatrix );
			vec3 refractedRayExit = position + transmissionRay;
			vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
			vec2 refractionCoords = ndcPos.xy / ndcPos.w;
			refractionCoords += 1.0;
			refractionCoords /= 2.0;
			transmittedLight = getTransmissionSample( refractionCoords, roughness, ior );
			transmittance = diffuseColor * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance );
		#endif
		vec3 attenuatedColor = transmittance * transmittedLight.rgb;
		vec3 F = EnvironmentBRDF( n, v, specularColor, specularF90, roughness );
		float transmittanceFactor = ( transmittance.r + transmittance.g + transmittance.b ) / 3.0;
		return vec4( ( 1.0 - F ) * attenuatedColor, 1.0 - ( 1.0 - transmittedLight.a ) * transmittanceFactor );
	}
#endif`,u_=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_SPECULARMAP
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,d_=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	uniform mat3 mapTransform;
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	uniform mat3 alphaMapTransform;
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	uniform mat3 lightMapTransform;
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	uniform mat3 aoMapTransform;
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	uniform mat3 bumpMapTransform;
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	uniform mat3 normalMapTransform;
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_DISPLACEMENTMAP
	uniform mat3 displacementMapTransform;
	varying vec2 vDisplacementMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	uniform mat3 emissiveMapTransform;
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	uniform mat3 metalnessMapTransform;
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	uniform mat3 roughnessMapTransform;
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	uniform mat3 anisotropyMapTransform;
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	uniform mat3 clearcoatMapTransform;
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform mat3 clearcoatNormalMapTransform;
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform mat3 clearcoatRoughnessMapTransform;
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	uniform mat3 sheenColorMapTransform;
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	uniform mat3 sheenRoughnessMapTransform;
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	uniform mat3 iridescenceMapTransform;
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform mat3 iridescenceThicknessMapTransform;
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SPECULARMAP
	uniform mat3 specularMapTransform;
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	uniform mat3 specularColorMapTransform;
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	uniform mat3 specularIntensityMapTransform;
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,f_=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	vUv = vec3( uv, 1 ).xy;
#endif
#ifdef USE_MAP
	vMapUv = ( mapTransform * vec3( MAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ALPHAMAP
	vAlphaMapUv = ( alphaMapTransform * vec3( ALPHAMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_LIGHTMAP
	vLightMapUv = ( lightMapTransform * vec3( LIGHTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_AOMAP
	vAoMapUv = ( aoMapTransform * vec3( AOMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_BUMPMAP
	vBumpMapUv = ( bumpMapTransform * vec3( BUMPMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_NORMALMAP
	vNormalMapUv = ( normalMapTransform * vec3( NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_DISPLACEMENTMAP
	vDisplacementMapUv = ( displacementMapTransform * vec3( DISPLACEMENTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_EMISSIVEMAP
	vEmissiveMapUv = ( emissiveMapTransform * vec3( EMISSIVEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_METALNESSMAP
	vMetalnessMapUv = ( metalnessMapTransform * vec3( METALNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ROUGHNESSMAP
	vRoughnessMapUv = ( roughnessMapTransform * vec3( ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ANISOTROPYMAP
	vAnisotropyMapUv = ( anisotropyMapTransform * vec3( ANISOTROPYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOATMAP
	vClearcoatMapUv = ( clearcoatMapTransform * vec3( CLEARCOATMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	vClearcoatNormalMapUv = ( clearcoatNormalMapTransform * vec3( CLEARCOAT_NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	vClearcoatRoughnessMapUv = ( clearcoatRoughnessMapTransform * vec3( CLEARCOAT_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCEMAP
	vIridescenceMapUv = ( iridescenceMapTransform * vec3( IRIDESCENCEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	vIridescenceThicknessMapUv = ( iridescenceThicknessMapTransform * vec3( IRIDESCENCE_THICKNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_COLORMAP
	vSheenColorMapUv = ( sheenColorMapTransform * vec3( SHEEN_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	vSheenRoughnessMapUv = ( sheenRoughnessMapTransform * vec3( SHEEN_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULARMAP
	vSpecularMapUv = ( specularMapTransform * vec3( SPECULARMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_COLORMAP
	vSpecularColorMapUv = ( specularColorMapTransform * vec3( SPECULAR_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	vSpecularIntensityMapUv = ( specularIntensityMapTransform * vec3( SPECULAR_INTENSITYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_TRANSMISSIONMAP
	vTransmissionMapUv = ( transmissionMapTransform * vec3( TRANSMISSIONMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_THICKNESSMAP
	vThicknessMapUv = ( thicknessMapTransform * vec3( THICKNESSMAP_UV, 1 ) ).xy;
#endif`,p_=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`,m_=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,g_=`uniform sampler2D t2D;
uniform float backgroundIntensity;
varying vec2 vUv;
void main() {
	vec4 texColor = texture2D( t2D, vUv );
	#ifdef DECODE_VIDEO_TEXTURE
		texColor = vec4( mix( pow( texColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), texColor.rgb * 0.0773993808, vec3( lessThanEqual( texColor.rgb, vec3( 0.04045 ) ) ) ), texColor.w );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,__=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,x_=`#ifdef ENVMAP_TYPE_CUBE
	uniform samplerCube envMap;
#elif defined( ENVMAP_TYPE_CUBE_UV )
	uniform sampler2D envMap;
#endif
uniform float flipEnvMap;
uniform float backgroundBlurriness;
uniform float backgroundIntensity;
uniform mat3 backgroundRotation;
varying vec3 vWorldDirection;
#include <cube_uv_reflection_fragment>
void main() {
	#ifdef ENVMAP_TYPE_CUBE
		vec4 texColor = textureCube( envMap, backgroundRotation * vec3( flipEnvMap * vWorldDirection.x, vWorldDirection.yz ) );
	#elif defined( ENVMAP_TYPE_CUBE_UV )
		vec4 texColor = textureCubeUV( envMap, backgroundRotation * vWorldDirection, backgroundBlurriness );
	#else
		vec4 texColor = vec4( 0.0, 0.0, 0.0, 1.0 );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,y_=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,v_=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,b_=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
varying vec2 vHighPrecisionZW;
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vHighPrecisionZW = gl_Position.zw;
}`,M_=`#if DEPTH_PACKING == 3200
	uniform float opacity;
#endif
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
varying vec2 vHighPrecisionZW;
void main() {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#if DEPTH_PACKING == 3200
		diffuseColor.a = opacity;
	#endif
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <logdepthbuf_fragment>
	#ifdef USE_REVERSED_DEPTH_BUFFER
		float fragCoordZ = vHighPrecisionZW[ 0 ] / vHighPrecisionZW[ 1 ];
	#else
		float fragCoordZ = 0.5 * vHighPrecisionZW[ 0 ] / vHighPrecisionZW[ 1 ] + 0.5;
	#endif
	#if DEPTH_PACKING == 3200
		gl_FragColor = vec4( vec3( 1.0 - fragCoordZ ), opacity );
	#elif DEPTH_PACKING == 3201
		gl_FragColor = packDepthToRGBA( fragCoordZ );
	#elif DEPTH_PACKING == 3202
		gl_FragColor = vec4( packDepthToRGB( fragCoordZ ), 1.0 );
	#elif DEPTH_PACKING == 3203
		gl_FragColor = vec4( packDepthToRG( fragCoordZ ), 0.0, 1.0 );
	#endif
}`,S_=`#define DISTANCE
varying vec3 vWorldPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <worldpos_vertex>
	#include <clipping_planes_vertex>
	vWorldPosition = worldPosition.xyz;
}`,E_=`#define DISTANCE
uniform vec3 referencePosition;
uniform float nearDistance;
uniform float farDistance;
varying vec3 vWorldPosition;
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <clipping_planes_pars_fragment>
void main () {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	float dist = length( vWorldPosition - referencePosition );
	dist = ( dist - nearDistance ) / ( farDistance - nearDistance );
	dist = saturate( dist );
	gl_FragColor = packDepthToRGBA( dist );
}`,T_=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,w_=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,A_=`uniform float scale;
attribute float lineDistance;
varying float vLineDistance;
#include <common>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	vLineDistance = scale * lineDistance;
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,R_=`uniform vec3 diffuse;
uniform float opacity;
uniform float dashSize;
uniform float totalSize;
varying float vLineDistance;
#include <common>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	if ( mod( vLineDistance, totalSize ) > dashSize ) {
		discard;
	}
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,C_=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#if defined ( USE_ENVMAP ) || defined ( USE_SKINNING )
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinbase_vertex>
		#include <skinnormal_vertex>
		#include <defaultnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <fog_vertex>
}`,I_=`uniform vec3 diffuse;
uniform float opacity;
#ifndef FLAT_SHADED
	varying vec3 vNormal;
#endif
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		reflectedLight.indirectDiffuse += lightMapTexel.rgb * lightMapIntensity * RECIPROCAL_PI;
	#else
		reflectedLight.indirectDiffuse += vec3( 1.0 );
	#endif
	#include <aomap_fragment>
	reflectedLight.indirectDiffuse *= diffuseColor.rgb;
	vec3 outgoingLight = reflectedLight.indirectDiffuse;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,P_=`#define LAMBERT
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,L_=`#define LAMBERT
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_lambert_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_lambert_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,N_=`#define MATCAP
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <displacementmap_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
	vViewPosition = - mvPosition.xyz;
}`,D_=`#define MATCAP
uniform vec3 diffuse;
uniform float opacity;
uniform sampler2D matcap;
varying vec3 vViewPosition;
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	vec3 viewDir = normalize( vViewPosition );
	vec3 x = normalize( vec3( viewDir.z, 0.0, - viewDir.x ) );
	vec3 y = cross( viewDir, x );
	vec2 uv = vec2( dot( x, normal ), dot( y, normal ) ) * 0.495 + 0.5;
	#ifdef USE_MATCAP
		vec4 matcapColor = texture2D( matcap, uv );
	#else
		vec4 matcapColor = vec4( vec3( mix( 0.2, 0.8, uv.y ) ), 1.0 );
	#endif
	vec3 outgoingLight = diffuseColor.rgb * matcapColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,U_=`#define NORMAL
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	vViewPosition = - mvPosition.xyz;
#endif
}`,O_=`#define NORMAL
uniform float opacity;
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <packing>
#include <uv_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( 0.0, 0.0, 0.0, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	gl_FragColor = vec4( packNormalToRGB( normal ), diffuseColor.a );
	#ifdef OPAQUE
		gl_FragColor.a = 1.0;
	#endif
}`,F_=`#define PHONG
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,B_=`#define PHONG
uniform vec3 diffuse;
uniform vec3 emissive;
uniform vec3 specular;
uniform float shininess;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_phong_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_phong_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + reflectedLight.directSpecular + reflectedLight.indirectSpecular + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,k_=`#define STANDARD
varying vec3 vViewPosition;
#ifdef USE_TRANSMISSION
	varying vec3 vWorldPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
#ifdef USE_TRANSMISSION
	vWorldPosition = worldPosition.xyz;
#endif
}`,z_=`#define STANDARD
#ifdef PHYSICAL
	#define IOR
	#define USE_SPECULAR
#endif
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float roughness;
uniform float metalness;
uniform float opacity;
#ifdef IOR
	uniform float ior;
#endif
#ifdef USE_SPECULAR
	uniform float specularIntensity;
	uniform vec3 specularColor;
	#ifdef USE_SPECULAR_COLORMAP
		uniform sampler2D specularColorMap;
	#endif
	#ifdef USE_SPECULAR_INTENSITYMAP
		uniform sampler2D specularIntensityMap;
	#endif
#endif
#ifdef USE_CLEARCOAT
	uniform float clearcoat;
	uniform float clearcoatRoughness;
#endif
#ifdef USE_DISPERSION
	uniform float dispersion;
#endif
#ifdef USE_IRIDESCENCE
	uniform float iridescence;
	uniform float iridescenceIOR;
	uniform float iridescenceThicknessMinimum;
	uniform float iridescenceThicknessMaximum;
#endif
#ifdef USE_SHEEN
	uniform vec3 sheenColor;
	uniform float sheenRoughness;
	#ifdef USE_SHEEN_COLORMAP
		uniform sampler2D sheenColorMap;
	#endif
	#ifdef USE_SHEEN_ROUGHNESSMAP
		uniform sampler2D sheenRoughnessMap;
	#endif
#endif
#ifdef USE_ANISOTROPY
	uniform vec2 anisotropyVector;
	#ifdef USE_ANISOTROPYMAP
		uniform sampler2D anisotropyMap;
	#endif
#endif
varying vec3 vViewPosition;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <iridescence_fragment>
#include <cube_uv_reflection_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_physical_pars_fragment>
#include <fog_pars_fragment>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_physical_pars_fragment>
#include <transmission_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <clearcoat_pars_fragment>
#include <iridescence_pars_fragment>
#include <roughnessmap_pars_fragment>
#include <metalnessmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <roughnessmap_fragment>
	#include <metalnessmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <clearcoat_normal_fragment_begin>
	#include <clearcoat_normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_physical_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 totalDiffuse = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse;
	vec3 totalSpecular = reflectedLight.directSpecular + reflectedLight.indirectSpecular;
	#include <transmission_fragment>
	vec3 outgoingLight = totalDiffuse + totalSpecular + totalEmissiveRadiance;
	#ifdef USE_SHEEN
		float sheenEnergyComp = 1.0 - 0.157 * max3( material.sheenColor );
		outgoingLight = outgoingLight * sheenEnergyComp + sheenSpecularDirect + sheenSpecularIndirect;
	#endif
	#ifdef USE_CLEARCOAT
		float dotNVcc = saturate( dot( geometryClearcoatNormal, geometryViewDir ) );
		vec3 Fcc = F_Schlick( material.clearcoatF0, material.clearcoatF90, dotNVcc );
		outgoingLight = outgoingLight * ( 1.0 - material.clearcoat * Fcc ) + ( clearcoatSpecularDirect + clearcoatSpecularIndirect ) * material.clearcoat;
	#endif
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,H_=`#define TOON
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,V_=`#define TOON
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <gradientmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_toon_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_toon_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,G_=`uniform float size;
uniform float scale;
#include <common>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
#ifdef USE_POINTS_UV
	varying vec2 vUv;
	uniform mat3 uvTransform;
#endif
void main() {
	#ifdef USE_POINTS_UV
		vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	#endif
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	gl_PointSize = size;
	#ifdef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) gl_PointSize *= ( scale / - mvPosition.z );
	#endif
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <fog_vertex>
}`,W_=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <color_pars_fragment>
#include <map_particle_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_particle_fragment>
	#include <color_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,X_=`#include <common>
#include <batching_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <shadowmap_pars_vertex>
void main() {
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,q_=`uniform vec3 color;
uniform float opacity;
#include <common>
#include <packing>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <logdepthbuf_pars_fragment>
#include <shadowmap_pars_fragment>
#include <shadowmask_pars_fragment>
void main() {
	#include <logdepthbuf_fragment>
	gl_FragColor = vec4( color, opacity * ( 1.0 - getShadowMask() ) );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,Y_=`uniform float rotation;
uniform vec2 center;
#include <common>
#include <uv_pars_vertex>
#include <fog_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	vec4 mvPosition = modelViewMatrix[ 3 ];
	vec2 scale = vec2( length( modelMatrix[ 0 ].xyz ), length( modelMatrix[ 1 ].xyz ) );
	#ifndef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) scale *= - mvPosition.z;
	#endif
	vec2 alignedPosition = ( position.xy - ( center - vec2( 0.5 ) ) ) * scale;
	vec2 rotatedPosition;
	rotatedPosition.x = cos( rotation ) * alignedPosition.x - sin( rotation ) * alignedPosition.y;
	rotatedPosition.y = sin( rotation ) * alignedPosition.x + cos( rotation ) * alignedPosition.y;
	mvPosition.xy += rotatedPosition;
	gl_Position = projectionMatrix * mvPosition;
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,j_=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,Xe={alphahash_fragment:gm,alphahash_pars_fragment:_m,alphamap_fragment:xm,alphamap_pars_fragment:ym,alphatest_fragment:vm,alphatest_pars_fragment:bm,aomap_fragment:Mm,aomap_pars_fragment:Sm,batching_pars_vertex:Em,batching_vertex:Tm,begin_vertex:wm,beginnormal_vertex:Am,bsdfs:Rm,iridescence_fragment:Cm,bumpmap_pars_fragment:Im,clipping_planes_fragment:Pm,clipping_planes_pars_fragment:Lm,clipping_planes_pars_vertex:Nm,clipping_planes_vertex:Dm,color_fragment:Um,color_pars_fragment:Om,color_pars_vertex:Fm,color_vertex:Bm,common:km,cube_uv_reflection_fragment:zm,defaultnormal_vertex:Hm,displacementmap_pars_vertex:Vm,displacementmap_vertex:Gm,emissivemap_fragment:Wm,emissivemap_pars_fragment:Xm,colorspace_fragment:qm,colorspace_pars_fragment:Ym,envmap_fragment:jm,envmap_common_pars_fragment:Km,envmap_pars_fragment:Zm,envmap_pars_vertex:Jm,envmap_physical_pars_fragment:cg,envmap_vertex:$m,fog_vertex:Qm,fog_pars_vertex:eg,fog_fragment:tg,fog_pars_fragment:ng,gradientmap_pars_fragment:ig,lightmap_pars_fragment:sg,lights_lambert_fragment:rg,lights_lambert_pars_fragment:og,lights_pars_begin:ag,lights_toon_fragment:lg,lights_toon_pars_fragment:hg,lights_phong_fragment:ug,lights_phong_pars_fragment:dg,lights_physical_fragment:fg,lights_physical_pars_fragment:pg,lights_fragment_begin:mg,lights_fragment_maps:gg,lights_fragment_end:_g,logdepthbuf_fragment:xg,logdepthbuf_pars_fragment:yg,logdepthbuf_pars_vertex:vg,logdepthbuf_vertex:bg,map_fragment:Mg,map_pars_fragment:Sg,map_particle_fragment:Eg,map_particle_pars_fragment:Tg,metalnessmap_fragment:wg,metalnessmap_pars_fragment:Ag,morphinstance_vertex:Rg,morphcolor_vertex:Cg,morphnormal_vertex:Ig,morphtarget_pars_vertex:Pg,morphtarget_vertex:Lg,normal_fragment_begin:Ng,normal_fragment_maps:Dg,normal_pars_fragment:Ug,normal_pars_vertex:Og,normal_vertex:Fg,normalmap_pars_fragment:Bg,clearcoat_normal_fragment_begin:kg,clearcoat_normal_fragment_maps:zg,clearcoat_pars_fragment:Hg,iridescence_pars_fragment:Vg,opaque_fragment:Gg,packing:Wg,premultiplied_alpha_fragment:Xg,project_vertex:qg,dithering_fragment:Yg,dithering_pars_fragment:jg,roughnessmap_fragment:Kg,roughnessmap_pars_fragment:Zg,shadowmap_pars_fragment:Jg,shadowmap_pars_vertex:$g,shadowmap_vertex:Qg,shadowmask_pars_fragment:e_,skinbase_vertex:t_,skinning_pars_vertex:n_,skinning_vertex:i_,skinnormal_vertex:s_,specularmap_fragment:r_,specularmap_pars_fragment:o_,tonemapping_fragment:a_,tonemapping_pars_fragment:c_,transmission_fragment:l_,transmission_pars_fragment:h_,uv_pars_fragment:u_,uv_pars_vertex:d_,uv_vertex:f_,worldpos_vertex:p_,background_vert:m_,background_frag:g_,backgroundCube_vert:__,backgroundCube_frag:x_,cube_vert:y_,cube_frag:v_,depth_vert:b_,depth_frag:M_,distanceRGBA_vert:S_,distanceRGBA_frag:E_,equirect_vert:T_,equirect_frag:w_,linedashed_vert:A_,linedashed_frag:R_,meshbasic_vert:C_,meshbasic_frag:I_,meshlambert_vert:P_,meshlambert_frag:L_,meshmatcap_vert:N_,meshmatcap_frag:D_,meshnormal_vert:U_,meshnormal_frag:O_,meshphong_vert:F_,meshphong_frag:B_,meshphysical_vert:k_,meshphysical_frag:z_,meshtoon_vert:H_,meshtoon_frag:V_,points_vert:G_,points_frag:W_,shadow_vert:X_,shadow_frag:q_,sprite_vert:Y_,sprite_frag:j_},he={common:{diffuse:{value:new Re(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new Ve},alphaMap:{value:null},alphaMapTransform:{value:new Ve},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new Ve}},envmap:{envMap:{value:null},envMapRotation:{value:new Ve},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new Ve}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new Ve}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new Ve},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new Ve},normalScale:{value:new Ee(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new Ve},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new Ve}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new Ve}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new Ve}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new Re(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new Re(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new Ve},alphaTest:{value:0},uvTransform:{value:new Ve}},sprite:{diffuse:{value:new Re(16777215)},opacity:{value:1},center:{value:new Ee(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new Ve},alphaMap:{value:null},alphaMapTransform:{value:new Ve},alphaTest:{value:0}}},ii={basic:{uniforms:Zt([he.common,he.specularmap,he.envmap,he.aomap,he.lightmap,he.fog]),vertexShader:Xe.meshbasic_vert,fragmentShader:Xe.meshbasic_frag},lambert:{uniforms:Zt([he.common,he.specularmap,he.envmap,he.aomap,he.lightmap,he.emissivemap,he.bumpmap,he.normalmap,he.displacementmap,he.fog,he.lights,{emissive:{value:new Re(0)}}]),vertexShader:Xe.meshlambert_vert,fragmentShader:Xe.meshlambert_frag},phong:{uniforms:Zt([he.common,he.specularmap,he.envmap,he.aomap,he.lightmap,he.emissivemap,he.bumpmap,he.normalmap,he.displacementmap,he.fog,he.lights,{emissive:{value:new Re(0)},specular:{value:new Re(1118481)},shininess:{value:30}}]),vertexShader:Xe.meshphong_vert,fragmentShader:Xe.meshphong_frag},standard:{uniforms:Zt([he.common,he.envmap,he.aomap,he.lightmap,he.emissivemap,he.bumpmap,he.normalmap,he.displacementmap,he.roughnessmap,he.metalnessmap,he.fog,he.lights,{emissive:{value:new Re(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:Xe.meshphysical_vert,fragmentShader:Xe.meshphysical_frag},toon:{uniforms:Zt([he.common,he.aomap,he.lightmap,he.emissivemap,he.bumpmap,he.normalmap,he.displacementmap,he.gradientmap,he.fog,he.lights,{emissive:{value:new Re(0)}}]),vertexShader:Xe.meshtoon_vert,fragmentShader:Xe.meshtoon_frag},matcap:{uniforms:Zt([he.common,he.bumpmap,he.normalmap,he.displacementmap,he.fog,{matcap:{value:null}}]),vertexShader:Xe.meshmatcap_vert,fragmentShader:Xe.meshmatcap_frag},points:{uniforms:Zt([he.points,he.fog]),vertexShader:Xe.points_vert,fragmentShader:Xe.points_frag},dashed:{uniforms:Zt([he.common,he.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:Xe.linedashed_vert,fragmentShader:Xe.linedashed_frag},depth:{uniforms:Zt([he.common,he.displacementmap]),vertexShader:Xe.depth_vert,fragmentShader:Xe.depth_frag},normal:{uniforms:Zt([he.common,he.bumpmap,he.normalmap,he.displacementmap,{opacity:{value:1}}]),vertexShader:Xe.meshnormal_vert,fragmentShader:Xe.meshnormal_frag},sprite:{uniforms:Zt([he.sprite,he.fog]),vertexShader:Xe.sprite_vert,fragmentShader:Xe.sprite_frag},background:{uniforms:{uvTransform:{value:new Ve},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:Xe.background_vert,fragmentShader:Xe.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1},backgroundRotation:{value:new Ve}},vertexShader:Xe.backgroundCube_vert,fragmentShader:Xe.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:Xe.cube_vert,fragmentShader:Xe.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:Xe.equirect_vert,fragmentShader:Xe.equirect_frag},distanceRGBA:{uniforms:Zt([he.common,he.displacementmap,{referencePosition:{value:new A},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:Xe.distanceRGBA_vert,fragmentShader:Xe.distanceRGBA_frag},shadow:{uniforms:Zt([he.lights,he.fog,{color:{value:new Re(0)},opacity:{value:1}}]),vertexShader:Xe.shadow_vert,fragmentShader:Xe.shadow_frag}};ii.physical={uniforms:Zt([ii.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new Ve},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new Ve},clearcoatNormalScale:{value:new Ee(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new Ve},dispersion:{value:0},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new Ve},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new Ve},sheen:{value:0},sheenColor:{value:new Re(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new Ve},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new Ve},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new Ve},transmissionSamplerSize:{value:new Ee},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new Ve},attenuationDistance:{value:0},attenuationColor:{value:new Re(0)},specularColor:{value:new Re(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new Ve},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new Ve},anisotropyVector:{value:new Ee},anisotropyMap:{value:null},anisotropyMapTransform:{value:new Ve}}]),vertexShader:Xe.meshphysical_vert,fragmentShader:Xe.meshphysical_frag};var rc={r:0,b:0,g:0},ps=new Fn,K_=new De;function Z_(s,e,t,n,i,r,o){let a=new Re(0),c=r===!0?0:1,l,h,u=null,d=0,f=null;function g(E){let y=E.isScene===!0?E.background:null;return y&&y.isTexture&&(y=(E.backgroundBlurriness>0?t:e).get(y)),y}function _(E){let y=!1,w=g(E);w===null?p(a,c):w&&w.isColor&&(p(w,1),y=!0);let C=s.xr.getEnvironmentBlendMode();C==="additive"?n.buffers.color.setClear(0,0,0,1,o):C==="alpha-blend"&&n.buffers.color.setClear(0,0,0,0,o),(s.autoClear||y)&&(n.buffers.depth.setTest(!0),n.buffers.depth.setMask(!0),n.buffers.color.setMask(!0),s.clear(s.autoClearColor,s.autoClearDepth,s.autoClearStencil))}function m(E,y){let w=g(y);w&&(w.isCubeTexture||w.mapping===ro)?(h===void 0&&(h=new lt(new Ks(1,1,1),new Bn({name:"BackgroundCubeMaterial",uniforms:fs(ii.backgroundCube.uniforms),vertexShader:ii.backgroundCube.vertexShader,fragmentShader:ii.backgroundCube.fragmentShader,side:en,depthTest:!1,depthWrite:!1,fog:!1,allowOverride:!1})),h.geometry.deleteAttribute("normal"),h.geometry.deleteAttribute("uv"),h.onBeforeRender=function(C,R,N){this.matrixWorld.copyPosition(N.matrixWorld)},Object.defineProperty(h.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),i.update(h)),ps.copy(y.backgroundRotation),ps.x*=-1,ps.y*=-1,ps.z*=-1,w.isCubeTexture&&w.isRenderTargetTexture===!1&&(ps.y*=-1,ps.z*=-1),h.material.uniforms.envMap.value=w,h.material.uniforms.flipEnvMap.value=w.isCubeTexture&&w.isRenderTargetTexture===!1?-1:1,h.material.uniforms.backgroundBlurriness.value=y.backgroundBlurriness,h.material.uniforms.backgroundIntensity.value=y.backgroundIntensity,h.material.uniforms.backgroundRotation.value.setFromMatrix4(K_.makeRotationFromEuler(ps)),h.material.toneMapped=$e.getTransfer(w.colorSpace)!==ot,(u!==w||d!==w.version||f!==s.toneMapping)&&(h.material.needsUpdate=!0,u=w,d=w.version,f=s.toneMapping),h.layers.enableAll(),E.unshift(h,h.geometry,h.material,0,0,null)):w&&w.isTexture&&(l===void 0&&(l=new lt(new cs(2,2),new Bn({name:"BackgroundMaterial",uniforms:fs(ii.background.uniforms),vertexShader:ii.background.vertexShader,fragmentShader:ii.background.fragmentShader,side:mn,depthTest:!1,depthWrite:!1,fog:!1,allowOverride:!1})),l.geometry.deleteAttribute("normal"),Object.defineProperty(l.material,"map",{get:function(){return this.uniforms.t2D.value}}),i.update(l)),l.material.uniforms.t2D.value=w,l.material.uniforms.backgroundIntensity.value=y.backgroundIntensity,l.material.toneMapped=$e.getTransfer(w.colorSpace)!==ot,w.matrixAutoUpdate===!0&&w.updateMatrix(),l.material.uniforms.uvTransform.value.copy(w.matrix),(u!==w||d!==w.version||f!==s.toneMapping)&&(l.material.needsUpdate=!0,u=w,d=w.version,f=s.toneMapping),l.layers.enableAll(),E.unshift(l,l.geometry,l.material,0,0,null))}function p(E,y){E.getRGB(rc,Il(s)),n.buffers.color.setClear(rc.r,rc.g,rc.b,y,o)}function S(){h!==void 0&&(h.geometry.dispose(),h.material.dispose(),h=void 0),l!==void 0&&(l.geometry.dispose(),l.material.dispose(),l=void 0)}return{getClearColor:function(){return a},setClearColor:function(E,y=1){a.set(E),c=y,p(a,c)},getClearAlpha:function(){return c},setClearAlpha:function(E){c=E,p(a,c)},render:_,addToRenderList:m,dispose:S}}function J_(s,e){let t=s.getParameter(s.MAX_VERTEX_ATTRIBS),n={},i=d(null),r=i,o=!1;function a(b,P,H,G,q){let W=!1,Y=u(G,H,P);r!==Y&&(r=Y,l(r.object)),W=f(b,G,H,q),W&&g(b,G,H,q),q!==null&&e.update(q,s.ELEMENT_ARRAY_BUFFER),(W||o)&&(o=!1,y(b,P,H,G),q!==null&&s.bindBuffer(s.ELEMENT_ARRAY_BUFFER,e.get(q).buffer))}function c(){return s.createVertexArray()}function l(b){return s.bindVertexArray(b)}function h(b){return s.deleteVertexArray(b)}function u(b,P,H){let G=H.wireframe===!0,q=n[b.id];q===void 0&&(q={},n[b.id]=q);let W=q[P.id];W===void 0&&(W={},q[P.id]=W);let Y=W[G];return Y===void 0&&(Y=d(c()),W[G]=Y),Y}function d(b){let P=[],H=[],G=[];for(let q=0;q<t;q++)P[q]=0,H[q]=0,G[q]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:P,enabledAttributes:H,attributeDivisors:G,object:b,attributes:{},index:null}}function f(b,P,H,G){let q=r.attributes,W=P.attributes,Y=0,B=H.getAttributes();for(let D in B)if(B[D].location>=0){let ae=q[D],ce=W[D];if(ce===void 0&&(D==="instanceMatrix"&&b.instanceMatrix&&(ce=b.instanceMatrix),D==="instanceColor"&&b.instanceColor&&(ce=b.instanceColor)),ae===void 0||ae.attribute!==ce||ce&&ae.data!==ce.data)return!0;Y++}return r.attributesNum!==Y||r.index!==G}function g(b,P,H,G){let q={},W=P.attributes,Y=0,B=H.getAttributes();for(let D in B)if(B[D].location>=0){let ae=W[D];ae===void 0&&(D==="instanceMatrix"&&b.instanceMatrix&&(ae=b.instanceMatrix),D==="instanceColor"&&b.instanceColor&&(ae=b.instanceColor));let ce={};ce.attribute=ae,ae&&ae.data&&(ce.data=ae.data),q[D]=ce,Y++}r.attributes=q,r.attributesNum=Y,r.index=G}function _(){let b=r.newAttributes;for(let P=0,H=b.length;P<H;P++)b[P]=0}function m(b){p(b,0)}function p(b,P){let H=r.newAttributes,G=r.enabledAttributes,q=r.attributeDivisors;H[b]=1,G[b]===0&&(s.enableVertexAttribArray(b),G[b]=1),q[b]!==P&&(s.vertexAttribDivisor(b,P),q[b]=P)}function S(){let b=r.newAttributes,P=r.enabledAttributes;for(let H=0,G=P.length;H<G;H++)P[H]!==b[H]&&(s.disableVertexAttribArray(H),P[H]=0)}function E(b,P,H,G,q,W,Y){Y===!0?s.vertexAttribIPointer(b,P,H,q,W):s.vertexAttribPointer(b,P,H,G,q,W)}function y(b,P,H,G){_();let q=G.attributes,W=H.getAttributes(),Y=P.defaultAttributeValues;for(let B in W){let D=W[B];if(D.location>=0){let Q=q[B];if(Q===void 0&&(B==="instanceMatrix"&&b.instanceMatrix&&(Q=b.instanceMatrix),B==="instanceColor"&&b.instanceColor&&(Q=b.instanceColor)),Q!==void 0){let ae=Q.normalized,ce=Q.itemSize,ve=e.get(Q);if(ve===void 0)continue;let Ze=ve.buffer,it=ve.type,Ye=ve.bytesPerElement,K=it===s.INT||it===s.UNSIGNED_INT||Q.gpuType===Ea;if(Q.isInterleavedBufferAttribute){let te=Q.data,_e=te.stride,ke=Q.offset;if(te.isInstancedInterleavedBuffer){for(let Ce=0;Ce<D.locationSize;Ce++)p(D.location+Ce,te.meshPerAttribute);b.isInstancedMesh!==!0&&G._maxInstanceCount===void 0&&(G._maxInstanceCount=te.meshPerAttribute*te.count)}else for(let Ce=0;Ce<D.locationSize;Ce++)m(D.location+Ce);s.bindBuffer(s.ARRAY_BUFFER,Ze);for(let Ce=0;Ce<D.locationSize;Ce++)E(D.location+Ce,ce/D.locationSize,it,ae,_e*Ye,(ke+ce/D.locationSize*Ce)*Ye,K)}else{if(Q.isInstancedBufferAttribute){for(let te=0;te<D.locationSize;te++)p(D.location+te,Q.meshPerAttribute);b.isInstancedMesh!==!0&&G._maxInstanceCount===void 0&&(G._maxInstanceCount=Q.meshPerAttribute*Q.count)}else for(let te=0;te<D.locationSize;te++)m(D.location+te);s.bindBuffer(s.ARRAY_BUFFER,Ze);for(let te=0;te<D.locationSize;te++)E(D.location+te,ce/D.locationSize,it,ae,ce*Ye,ce/D.locationSize*te*Ye,K)}}else if(Y!==void 0){let ae=Y[B];if(ae!==void 0)switch(ae.length){case 2:s.vertexAttrib2fv(D.location,ae);break;case 3:s.vertexAttrib3fv(D.location,ae);break;case 4:s.vertexAttrib4fv(D.location,ae);break;default:s.vertexAttrib1fv(D.location,ae)}}}}S()}function w(){N();for(let b in n){let P=n[b];for(let H in P){let G=P[H];for(let q in G)h(G[q].object),delete G[q];delete P[H]}delete n[b]}}function C(b){if(n[b.id]===void 0)return;let P=n[b.id];for(let H in P){let G=P[H];for(let q in G)h(G[q].object),delete G[q];delete P[H]}delete n[b.id]}function R(b){for(let P in n){let H=n[P];if(H[b.id]===void 0)continue;let G=H[b.id];for(let q in G)h(G[q].object),delete G[q];delete H[b.id]}}function N(){M(),o=!0,r!==i&&(r=i,l(r.object))}function M(){i.geometry=null,i.program=null,i.wireframe=!1}return{setup:a,reset:N,resetDefaultState:M,dispose:w,releaseStatesOfGeometry:C,releaseStatesOfProgram:R,initAttributes:_,enableAttribute:m,disableUnusedAttributes:S}}function $_(s,e,t){let n;function i(l){n=l}function r(l,h){s.drawArrays(n,l,h),t.update(h,n,1)}function o(l,h,u){u!==0&&(s.drawArraysInstanced(n,l,h,u),t.update(h,n,u))}function a(l,h,u){if(u===0)return;e.get("WEBGL_multi_draw").multiDrawArraysWEBGL(n,l,0,h,0,u);let f=0;for(let g=0;g<u;g++)f+=h[g];t.update(f,n,1)}function c(l,h,u,d){if(u===0)return;let f=e.get("WEBGL_multi_draw");if(f===null)for(let g=0;g<l.length;g++)o(l[g],h[g],d[g]);else{f.multiDrawArraysInstancedWEBGL(n,l,0,h,0,d,0,u);let g=0;for(let _=0;_<u;_++)g+=h[_]*d[_];t.update(g,n,1)}}this.setMode=i,this.render=r,this.renderInstances=o,this.renderMultiDraw=a,this.renderMultiDrawInstances=c}function Q_(s,e,t,n){let i;function r(){if(i!==void 0)return i;if(e.has("EXT_texture_filter_anisotropic")===!0){let R=e.get("EXT_texture_filter_anisotropic");i=s.getParameter(R.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else i=0;return i}function o(R){return!(R!==tn&&n.convert(R)!==s.getParameter(s.IMPLEMENTATION_COLOR_READ_FORMAT))}function a(R){let N=R===rr&&(e.has("EXT_color_buffer_half_float")||e.has("EXT_color_buffer_float"));return!(R!==zn&&n.convert(R)!==s.getParameter(s.IMPLEMENTATION_COLOR_READ_TYPE)&&R!==wn&&!N)}function c(R){if(R==="highp"){if(s.getShaderPrecisionFormat(s.VERTEX_SHADER,s.HIGH_FLOAT).precision>0&&s.getShaderPrecisionFormat(s.FRAGMENT_SHADER,s.HIGH_FLOAT).precision>0)return"highp";R="mediump"}return R==="mediump"&&s.getShaderPrecisionFormat(s.VERTEX_SHADER,s.MEDIUM_FLOAT).precision>0&&s.getShaderPrecisionFormat(s.FRAGMENT_SHADER,s.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}let l=t.precision!==void 0?t.precision:"highp",h=c(l);h!==l&&(console.warn("THREE.WebGLRenderer:",l,"not supported, using",h,"instead."),l=h);let u=t.logarithmicDepthBuffer===!0,d=t.reversedDepthBuffer===!0&&e.has("EXT_clip_control"),f=s.getParameter(s.MAX_TEXTURE_IMAGE_UNITS),g=s.getParameter(s.MAX_VERTEX_TEXTURE_IMAGE_UNITS),_=s.getParameter(s.MAX_TEXTURE_SIZE),m=s.getParameter(s.MAX_CUBE_MAP_TEXTURE_SIZE),p=s.getParameter(s.MAX_VERTEX_ATTRIBS),S=s.getParameter(s.MAX_VERTEX_UNIFORM_VECTORS),E=s.getParameter(s.MAX_VARYING_VECTORS),y=s.getParameter(s.MAX_FRAGMENT_UNIFORM_VECTORS),w=g>0,C=s.getParameter(s.MAX_SAMPLES);return{isWebGL2:!0,getMaxAnisotropy:r,getMaxPrecision:c,textureFormatReadable:o,textureTypeReadable:a,precision:l,logarithmicDepthBuffer:u,reversedDepthBuffer:d,maxTextures:f,maxVertexTextures:g,maxTextureSize:_,maxCubemapSize:m,maxAttributes:p,maxVertexUniforms:S,maxVaryings:E,maxFragmentUniforms:y,vertexTextures:w,maxSamples:C}}function ex(s){let e=this,t=null,n=0,i=!1,r=!1,o=new Sn,a=new Ve,c={value:null,needsUpdate:!1};this.uniform=c,this.numPlanes=0,this.numIntersection=0,this.init=function(u,d){let f=u.length!==0||d||n!==0||i;return i=d,n=u.length,f},this.beginShadows=function(){r=!0,h(null)},this.endShadows=function(){r=!1},this.setGlobalState=function(u,d){t=h(u,d,0)},this.setState=function(u,d,f){let g=u.clippingPlanes,_=u.clipIntersection,m=u.clipShadows,p=s.get(u);if(!i||g===null||g.length===0||r&&!m)r?h(null):l();else{let S=r?0:n,E=S*4,y=p.clippingState||null;c.value=y,y=h(g,d,E,f);for(let w=0;w!==E;++w)y[w]=t[w];p.clippingState=y,this.numIntersection=_?this.numPlanes:0,this.numPlanes+=S}};function l(){c.value!==t&&(c.value=t,c.needsUpdate=n>0),e.numPlanes=n,e.numIntersection=0}function h(u,d,f,g){let _=u!==null?u.length:0,m=null;if(_!==0){if(m=c.value,g!==!0||m===null){let p=f+_*4,S=d.matrixWorldInverse;a.getNormalMatrix(S),(m===null||m.length<p)&&(m=new Float32Array(p));for(let E=0,y=f;E!==_;++E,y+=4)o.copy(u[E]).applyMatrix4(S,a),o.normal.toArray(m,y),m[y+3]=o.constant}c.value=m,c.needsUpdate=!0}return e.numPlanes=_,e.numIntersection=0,m}}function tx(s){let e=new WeakMap;function t(o,a){return a===Ma?o.mapping=us:a===Sa&&(o.mapping=ds),o}function n(o){if(o&&o.isTexture){let a=o.mapping;if(a===Ma||a===Sa)if(e.has(o)){let c=e.get(o).texture;return t(c,o.mapping)}else{let c=o.image;if(c&&c.height>0){let l=new ta(c.height);return l.fromEquirectangularTexture(s,o),e.set(o,l),o.addEventListener("dispose",i),t(l.texture,o.mapping)}else return null}}return o}function i(o){let a=o.target;a.removeEventListener("dispose",i);let c=e.get(a);c!==void 0&&(e.delete(a),c.dispose())}function r(){e=new WeakMap}return{get:n,dispose:r}}var hr=4,jd=[.125,.215,.35,.446,.526,.582],_s=20,Dl=new hs,Kd=new Re,Ul=null,Ol=0,Fl=0,Bl=!1,gs=(1+Math.sqrt(5))/2,lr=1/gs,Zd=[new A(-gs,lr,0),new A(gs,lr,0),new A(-lr,0,gs),new A(lr,0,gs),new A(0,gs,-lr),new A(0,gs,lr),new A(-1,1,-1),new A(1,1,-1),new A(-1,1,1),new A(1,1,1)],nx=new A,cc=class{constructor(e){this._renderer=e,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(e,t=0,n=.1,i=100,r={}){let{size:o=256,position:a=nx}=r;Ul=this._renderer.getRenderTarget(),Ol=this._renderer.getActiveCubeFace(),Fl=this._renderer.getActiveMipmapLevel(),Bl=this._renderer.xr.enabled,this._renderer.xr.enabled=!1,this._setSize(o);let c=this._allocateTargets();return c.depthBuffer=!0,this._sceneToCubeUV(e,n,i,c,a),t>0&&this._blur(c,0,0,t),this._applyPMREM(c),this._cleanup(c),c}fromEquirectangular(e,t=null){return this._fromTexture(e,t)}fromCubemap(e,t=null){return this._fromTexture(e,t)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=Qd(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=$d(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(e){this._lodMax=Math.floor(Math.log2(e)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let e=0;e<this._lodPlanes.length;e++)this._lodPlanes[e].dispose()}_cleanup(e){this._renderer.setRenderTarget(Ul,Ol,Fl),this._renderer.xr.enabled=Bl,e.scissorTest=!1,oc(e,0,0,e.width,e.height)}_fromTexture(e,t){e.mapping===us||e.mapping===ds?this._setSize(e.image.length===0?16:e.image[0].width||e.image[0].image.width):this._setSize(e.image.width/4),Ul=this._renderer.getRenderTarget(),Ol=this._renderer.getActiveCubeFace(),Fl=this._renderer.getActiveMipmapLevel(),Bl=this._renderer.xr.enabled,this._renderer.xr.enabled=!1;let n=t||this._allocateTargets();return this._textureToCubeUV(e,n),this._applyPMREM(n),this._cleanup(n),n}_allocateTargets(){let e=3*Math.max(this._cubeSize,112),t=4*this._cubeSize,n={magFilter:Ht,minFilter:Ht,generateMipmaps:!1,type:rr,format:tn,colorSpace:Vt,depthBuffer:!1},i=Jd(e,t,n);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==e||this._pingPongRenderTarget.height!==t){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=Jd(e,t,n);let{_lodMax:r}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=ix(r)),this._blurMaterial=sx(r,e,t)}return i}_compileMaterial(e){let t=new lt(this._lodPlanes[0],e);this._renderer.compile(t,Dl)}_sceneToCubeUV(e,t,n,i,r){let c=new wt(90,1,t,n),l=[1,-1,1,1,1,1],h=[1,1,1,-1,-1,-1],u=this._renderer,d=u.autoClear,f=u.toneMapping;u.getClearColor(Kd),u.toneMapping=Si,u.autoClear=!1,u.state.buffers.depth.getReversed()&&(u.setRenderTarget(i),u.clearDepth(),u.setRenderTarget(null));let _=new jt({name:"PMREM.Background",side:en,depthWrite:!1,depthTest:!1}),m=new lt(new Ks,_),p=!1,S=e.background;S?S.isColor&&(_.color.copy(S),e.background=null,p=!0):(_.color.copy(Kd),p=!0);for(let E=0;E<6;E++){let y=E%3;y===0?(c.up.set(0,l[E],0),c.position.set(r.x,r.y,r.z),c.lookAt(r.x+h[E],r.y,r.z)):y===1?(c.up.set(0,0,l[E]),c.position.set(r.x,r.y,r.z),c.lookAt(r.x,r.y+h[E],r.z)):(c.up.set(0,l[E],0),c.position.set(r.x,r.y,r.z),c.lookAt(r.x,r.y,r.z+h[E]));let w=this._cubeSize;oc(i,y*w,E>2?w:0,w,w),u.setRenderTarget(i),p&&u.render(m,c),u.render(e,c)}m.geometry.dispose(),m.material.dispose(),u.toneMapping=f,u.autoClear=d,e.background=S}_textureToCubeUV(e,t){let n=this._renderer,i=e.mapping===us||e.mapping===ds;i?(this._cubemapMaterial===null&&(this._cubemapMaterial=Qd()),this._cubemapMaterial.uniforms.flipEnvMap.value=e.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=$d());let r=i?this._cubemapMaterial:this._equirectMaterial,o=new lt(this._lodPlanes[0],r),a=r.uniforms;a.envMap.value=e;let c=this._cubeSize;oc(t,0,0,3*c,2*c),n.setRenderTarget(t),n.render(o,Dl)}_applyPMREM(e){let t=this._renderer,n=t.autoClear;t.autoClear=!1;let i=this._lodPlanes.length;for(let r=1;r<i;r++){let o=Math.sqrt(this._sigmas[r]*this._sigmas[r]-this._sigmas[r-1]*this._sigmas[r-1]),a=Zd[(i-r-1)%Zd.length];this._blur(e,r-1,r,o,a)}t.autoClear=n}_blur(e,t,n,i,r){let o=this._pingPongRenderTarget;this._halfBlur(e,o,t,n,i,"latitudinal",r),this._halfBlur(o,e,n,n,i,"longitudinal",r)}_halfBlur(e,t,n,i,r,o,a){let c=this._renderer,l=this._blurMaterial;o!=="latitudinal"&&o!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");let h=3,u=new lt(this._lodPlanes[i],l),d=l.uniforms,f=this._sizeLods[n]-1,g=isFinite(r)?Math.PI/(2*f):2*Math.PI/(2*_s-1),_=r/g,m=isFinite(r)?1+Math.floor(h*_):_s;m>_s&&console.warn(`sigmaRadians, ${r}, is too large and will clip, as it requested ${m} samples when the maximum is set to ${_s}`);let p=[],S=0;for(let R=0;R<_s;++R){let N=R/_,M=Math.exp(-N*N/2);p.push(M),R===0?S+=M:R<m&&(S+=2*M)}for(let R=0;R<p.length;R++)p[R]=p[R]/S;d.envMap.value=e.texture,d.samples.value=m,d.weights.value=p,d.latitudinal.value=o==="latitudinal",a&&(d.poleAxis.value=a);let{_lodMax:E}=this;d.dTheta.value=g,d.mipInt.value=E-n;let y=this._sizeLods[i],w=3*y*(i>E-hr?i-E+hr:0),C=4*(this._cubeSize-y);oc(t,w,C,3*y,2*y),c.setRenderTarget(t),c.render(u,Dl)}};function ix(s){let e=[],t=[],n=[],i=s,r=s-hr+1+jd.length;for(let o=0;o<r;o++){let a=Math.pow(2,i);t.push(a);let c=1/a;o>s-hr?c=jd[o-s+hr-1]:o===0&&(c=0),n.push(c);let l=1/(a-2),h=-l,u=1+l,d=[h,h,u,h,u,u,h,h,u,u,h,u],f=6,g=6,_=3,m=2,p=1,S=new Float32Array(_*g*f),E=new Float32Array(m*g*f),y=new Float32Array(p*g*f);for(let C=0;C<f;C++){let R=C%3*2/3-1,N=C>2?0:-1,M=[R,N,0,R+2/3,N,0,R+2/3,N+1,0,R,N,0,R+2/3,N+1,0,R,N+1,0];S.set(M,_*g*C),E.set(d,m*g*C);let b=[C,C,C,C,C,C];y.set(b,p*g*C)}let w=new xt;w.setAttribute("position",new at(S,_)),w.setAttribute("uv",new at(E,m)),w.setAttribute("faceIndex",new at(y,p)),e.push(w),i>hr&&i--}return{lodPlanes:e,sizeLods:t,sigmas:n}}function Jd(s,e,t){let n=new Zn(s,e,t);return n.texture.mapping=ro,n.texture.name="PMREM.cubeUv",n.scissorTest=!0,n}function oc(s,e,t,n,i){s.viewport.set(e,t,n,i),s.scissor.set(e,t,n,i)}function sx(s,e,t){let n=new Float32Array(_s),i=new A(0,1,0);return new Bn({name:"SphericalGaussianBlur",defines:{n:_s,CUBEUV_TEXEL_WIDTH:1/e,CUBEUV_TEXEL_HEIGHT:1/t,CUBEUV_MAX_MIP:`${s}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:n},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:i}},vertexShader:Kl(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;
			uniform int samples;
			uniform float weights[ n ];
			uniform bool latitudinal;
			uniform float dTheta;
			uniform float mipInt;
			uniform vec3 poleAxis;

			#define ENVMAP_TYPE_CUBE_UV
			#include <cube_uv_reflection_fragment>

			vec3 getSample( float theta, vec3 axis ) {

				float cosTheta = cos( theta );
				// Rodrigues' axis-angle rotation
				vec3 sampleDirection = vOutputDirection * cosTheta
					+ cross( axis, vOutputDirection ) * sin( theta )
					+ axis * dot( axis, vOutputDirection ) * ( 1.0 - cosTheta );

				return bilinearCubeUV( envMap, sampleDirection, mipInt );

			}

			void main() {

				vec3 axis = latitudinal ? poleAxis : cross( poleAxis, vOutputDirection );

				if ( all( equal( axis, vec3( 0.0 ) ) ) ) {

					axis = vec3( vOutputDirection.z, 0.0, - vOutputDirection.x );

				}

				axis = normalize( axis );

				gl_FragColor = vec4( 0.0, 0.0, 0.0, 1.0 );
				gl_FragColor.rgb += weights[ 0 ] * getSample( 0.0, axis );

				for ( int i = 1; i < n; i++ ) {

					if ( i >= samples ) {

						break;

					}

					float theta = dTheta * float( i );
					gl_FragColor.rgb += weights[ i ] * getSample( -1.0 * theta, axis );
					gl_FragColor.rgb += weights[ i ] * getSample( theta, axis );

				}

			}
		`,blending:Mi,depthTest:!1,depthWrite:!1})}function $d(){return new Bn({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:Kl(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;

			#include <common>

			void main() {

				vec3 outputDirection = normalize( vOutputDirection );
				vec2 uv = equirectUv( outputDirection );

				gl_FragColor = vec4( texture2D ( envMap, uv ).rgb, 1.0 );

			}
		`,blending:Mi,depthTest:!1,depthWrite:!1})}function Qd(){return new Bn({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:Kl(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:Mi,depthTest:!1,depthWrite:!1})}function Kl(){return`

		precision mediump float;
		precision mediump int;

		attribute float faceIndex;

		varying vec3 vOutputDirection;

		// RH coordinate system; PMREM face-indexing convention
		vec3 getDirection( vec2 uv, float face ) {

			uv = 2.0 * uv - 1.0;

			vec3 direction = vec3( uv, 1.0 );

			if ( face == 0.0 ) {

				direction = direction.zyx; // ( 1, v, u ) pos x

			} else if ( face == 1.0 ) {

				direction = direction.xzy;
				direction.xz *= -1.0; // ( -u, 1, -v ) pos y

			} else if ( face == 2.0 ) {

				direction.x *= -1.0; // ( -u, v, 1 ) pos z

			} else if ( face == 3.0 ) {

				direction = direction.zyx;
				direction.xz *= -1.0; // ( -1, v, -u ) neg x

			} else if ( face == 4.0 ) {

				direction = direction.xzy;
				direction.xy *= -1.0; // ( -u, -1, v ) neg y

			} else if ( face == 5.0 ) {

				direction.z *= -1.0; // ( u, v, -1 ) neg z

			}

			return direction;

		}

		void main() {

			vOutputDirection = getDirection( uv, faceIndex );
			gl_Position = vec4( position, 1.0 );

		}
	`}function rx(s){let e=new WeakMap,t=null;function n(a){if(a&&a.isTexture){let c=a.mapping,l=c===Ma||c===Sa,h=c===us||c===ds;if(l||h){let u=e.get(a),d=u!==void 0?u.texture.pmremVersion:0;if(a.isRenderTargetTexture&&a.pmremVersion!==d)return t===null&&(t=new cc(s)),u=l?t.fromEquirectangular(a,u):t.fromCubemap(a,u),u.texture.pmremVersion=a.pmremVersion,e.set(a,u),u.texture;if(u!==void 0)return u.texture;{let f=a.image;return l&&f&&f.height>0||h&&f&&i(f)?(t===null&&(t=new cc(s)),u=l?t.fromEquirectangular(a):t.fromCubemap(a),u.texture.pmremVersion=a.pmremVersion,e.set(a,u),a.addEventListener("dispose",r),u.texture):null}}}return a}function i(a){let c=0,l=6;for(let h=0;h<l;h++)a[h]!==void 0&&c++;return c===l}function r(a){let c=a.target;c.removeEventListener("dispose",r);let l=e.get(c);l!==void 0&&(e.delete(c),l.dispose())}function o(){e=new WeakMap,t!==null&&(t.dispose(),t=null)}return{get:n,dispose:o}}function ox(s){let e={};function t(n){if(e[n]!==void 0)return e[n];let i;switch(n){case"WEBGL_depth_texture":i=s.getExtension("WEBGL_depth_texture")||s.getExtension("MOZ_WEBGL_depth_texture")||s.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":i=s.getExtension("EXT_texture_filter_anisotropic")||s.getExtension("MOZ_EXT_texture_filter_anisotropic")||s.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":i=s.getExtension("WEBGL_compressed_texture_s3tc")||s.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||s.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":i=s.getExtension("WEBGL_compressed_texture_pvrtc")||s.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:i=s.getExtension(n)}return e[n]=i,i}return{has:function(n){return t(n)!==null},init:function(){t("EXT_color_buffer_float"),t("WEBGL_clip_cull_distance"),t("OES_texture_float_linear"),t("EXT_color_buffer_half_float"),t("WEBGL_multisampled_render_to_texture"),t("WEBGL_render_shared_exponent")},get:function(n){let i=t(n);return i===null&&qs("THREE.WebGLRenderer: "+n+" extension not supported."),i}}}function ax(s,e,t,n){let i={},r=new WeakMap;function o(u){let d=u.target;d.index!==null&&e.remove(d.index);for(let g in d.attributes)e.remove(d.attributes[g]);d.removeEventListener("dispose",o),delete i[d.id];let f=r.get(d);f&&(e.remove(f),r.delete(d)),n.releaseStatesOfGeometry(d),d.isInstancedBufferGeometry===!0&&delete d._maxInstanceCount,t.memory.geometries--}function a(u,d){return i[d.id]===!0||(d.addEventListener("dispose",o),i[d.id]=!0,t.memory.geometries++),d}function c(u){let d=u.attributes;for(let f in d)e.update(d[f],s.ARRAY_BUFFER)}function l(u){let d=[],f=u.index,g=u.attributes.position,_=0;if(f!==null){let S=f.array;_=f.version;for(let E=0,y=S.length;E<y;E+=3){let w=S[E+0],C=S[E+1],R=S[E+2];d.push(w,C,C,R,R,w)}}else if(g!==void 0){let S=g.array;_=g.version;for(let E=0,y=S.length/3-1;E<y;E+=3){let w=E+0,C=E+1,R=E+2;d.push(w,C,C,R,R,w)}}else return;let m=new(Cl(d)?Ur:Dr)(d,1);m.version=_;let p=r.get(u);p&&e.remove(p),r.set(u,m)}function h(u){let d=r.get(u);if(d){let f=u.index;f!==null&&d.version<f.version&&l(u)}else l(u);return r.get(u)}return{get:a,update:c,getWireframeAttribute:h}}function cx(s,e,t){let n;function i(d){n=d}let r,o;function a(d){r=d.type,o=d.bytesPerElement}function c(d,f){s.drawElements(n,f,r,d*o),t.update(f,n,1)}function l(d,f,g){g!==0&&(s.drawElementsInstanced(n,f,r,d*o,g),t.update(f,n,g))}function h(d,f,g){if(g===0)return;e.get("WEBGL_multi_draw").multiDrawElementsWEBGL(n,f,0,r,d,0,g);let m=0;for(let p=0;p<g;p++)m+=f[p];t.update(m,n,1)}function u(d,f,g,_){if(g===0)return;let m=e.get("WEBGL_multi_draw");if(m===null)for(let p=0;p<d.length;p++)l(d[p]/o,f[p],_[p]);else{m.multiDrawElementsInstancedWEBGL(n,f,0,r,d,0,_,0,g);let p=0;for(let S=0;S<g;S++)p+=f[S]*_[S];t.update(p,n,1)}}this.setMode=i,this.setIndex=a,this.render=c,this.renderInstances=l,this.renderMultiDraw=h,this.renderMultiDrawInstances=u}function lx(s){let e={geometries:0,textures:0},t={frame:0,calls:0,triangles:0,points:0,lines:0};function n(r,o,a){switch(t.calls++,o){case s.TRIANGLES:t.triangles+=a*(r/3);break;case s.LINES:t.lines+=a*(r/2);break;case s.LINE_STRIP:t.lines+=a*(r-1);break;case s.LINE_LOOP:t.lines+=a*r;break;case s.POINTS:t.points+=a*r;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",o);break}}function i(){t.calls=0,t.triangles=0,t.points=0,t.lines=0}return{memory:e,render:t,programs:null,autoReset:!0,reset:i,update:n}}function hx(s,e,t){let n=new WeakMap,i=new nt;function r(o,a,c){let l=o.morphTargetInfluences,h=a.morphAttributes.position||a.morphAttributes.normal||a.morphAttributes.color,u=h!==void 0?h.length:0,d=n.get(a);if(d===void 0||d.count!==u){let M=function(){R.dispose(),n.delete(a),a.removeEventListener("dispose",M)};d!==void 0&&d.texture.dispose();let f=a.morphAttributes.position!==void 0,g=a.morphAttributes.normal!==void 0,_=a.morphAttributes.color!==void 0,m=a.morphAttributes.position||[],p=a.morphAttributes.normal||[],S=a.morphAttributes.color||[],E=0;f===!0&&(E=1),g===!0&&(E=2),_===!0&&(E=3);let y=a.attributes.position.count*E,w=1;y>e.maxTextureSize&&(w=Math.ceil(y/e.maxTextureSize),y=e.maxTextureSize);let C=new Float32Array(y*w*4*u),R=new Nr(C,y,w,u);R.type=wn,R.needsUpdate=!0;let N=E*4;for(let b=0;b<u;b++){let P=m[b],H=p[b],G=S[b],q=y*w*4*b;for(let W=0;W<P.count;W++){let Y=W*N;f===!0&&(i.fromBufferAttribute(P,W),C[q+Y+0]=i.x,C[q+Y+1]=i.y,C[q+Y+2]=i.z,C[q+Y+3]=0),g===!0&&(i.fromBufferAttribute(H,W),C[q+Y+4]=i.x,C[q+Y+5]=i.y,C[q+Y+6]=i.z,C[q+Y+7]=0),_===!0&&(i.fromBufferAttribute(G,W),C[q+Y+8]=i.x,C[q+Y+9]=i.y,C[q+Y+10]=i.z,C[q+Y+11]=G.itemSize===4?i.w:1)}}d={count:u,texture:R,size:new Ee(y,w)},n.set(a,d),a.addEventListener("dispose",M)}if(o.isInstancedMesh===!0&&o.morphTexture!==null)c.getUniforms().setValue(s,"morphTexture",o.morphTexture,t);else{let f=0;for(let _=0;_<l.length;_++)f+=l[_];let g=a.morphTargetsRelative?1:1-f;c.getUniforms().setValue(s,"morphTargetBaseInfluence",g),c.getUniforms().setValue(s,"morphTargetInfluences",l)}c.getUniforms().setValue(s,"morphTargetsTexture",d.texture,t),c.getUniforms().setValue(s,"morphTargetsTextureSize",d.size)}return{update:r}}function ux(s,e,t,n){let i=new WeakMap;function r(c){let l=n.render.frame,h=c.geometry,u=e.get(c,h);if(i.get(u)!==l&&(e.update(u),i.set(u,l)),c.isInstancedMesh&&(c.hasEventListener("dispose",a)===!1&&c.addEventListener("dispose",a),i.get(c)!==l&&(t.update(c.instanceMatrix,s.ARRAY_BUFFER),c.instanceColor!==null&&t.update(c.instanceColor,s.ARRAY_BUFFER),i.set(c,l))),c.isSkinnedMesh){let d=c.skeleton;i.get(d)!==l&&(d.update(),i.set(d,l))}return u}function o(){i=new WeakMap}function a(c){let l=c.target;l.removeEventListener("dispose",a),t.remove(l.instanceMatrix),l.instanceColor!==null&&t.remove(l.instanceColor)}return{update:r,dispose:o}}var yf=new At,ef=new Xr(1,1),vf=new Nr,bf=new Qo,Mf=new Fr,tf=[],nf=[],sf=new Float32Array(16),rf=new Float32Array(9),of=new Float32Array(4);function dr(s,e,t){let n=s[0];if(n<=0||n>0)return s;let i=e*t,r=tf[i];if(r===void 0&&(r=new Float32Array(i),tf[i]=r),e!==0){n.toArray(r,0);for(let o=1,a=0;o!==e;++o)a+=t,s[o].toArray(r,a)}return r}function Pt(s,e){if(s.length!==e.length)return!1;for(let t=0,n=s.length;t<n;t++)if(s[t]!==e[t])return!1;return!0}function Lt(s,e){for(let t=0,n=e.length;t<n;t++)s[t]=e[t]}function hc(s,e){let t=nf[e];t===void 0&&(t=new Int32Array(e),nf[e]=t);for(let n=0;n!==e;++n)t[n]=s.allocateTextureUnit();return t}function dx(s,e){let t=this.cache;t[0]!==e&&(s.uniform1f(this.addr,e),t[0]=e)}function fx(s,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(s.uniform2f(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Pt(t,e))return;s.uniform2fv(this.addr,e),Lt(t,e)}}function px(s,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(s.uniform3f(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else if(e.r!==void 0)(t[0]!==e.r||t[1]!==e.g||t[2]!==e.b)&&(s.uniform3f(this.addr,e.r,e.g,e.b),t[0]=e.r,t[1]=e.g,t[2]=e.b);else{if(Pt(t,e))return;s.uniform3fv(this.addr,e),Lt(t,e)}}function mx(s,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(s.uniform4f(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Pt(t,e))return;s.uniform4fv(this.addr,e),Lt(t,e)}}function gx(s,e){let t=this.cache,n=e.elements;if(n===void 0){if(Pt(t,e))return;s.uniformMatrix2fv(this.addr,!1,e),Lt(t,e)}else{if(Pt(t,n))return;of.set(n),s.uniformMatrix2fv(this.addr,!1,of),Lt(t,n)}}function _x(s,e){let t=this.cache,n=e.elements;if(n===void 0){if(Pt(t,e))return;s.uniformMatrix3fv(this.addr,!1,e),Lt(t,e)}else{if(Pt(t,n))return;rf.set(n),s.uniformMatrix3fv(this.addr,!1,rf),Lt(t,n)}}function xx(s,e){let t=this.cache,n=e.elements;if(n===void 0){if(Pt(t,e))return;s.uniformMatrix4fv(this.addr,!1,e),Lt(t,e)}else{if(Pt(t,n))return;sf.set(n),s.uniformMatrix4fv(this.addr,!1,sf),Lt(t,n)}}function yx(s,e){let t=this.cache;t[0]!==e&&(s.uniform1i(this.addr,e),t[0]=e)}function vx(s,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(s.uniform2i(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Pt(t,e))return;s.uniform2iv(this.addr,e),Lt(t,e)}}function bx(s,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(s.uniform3i(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(Pt(t,e))return;s.uniform3iv(this.addr,e),Lt(t,e)}}function Mx(s,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(s.uniform4i(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Pt(t,e))return;s.uniform4iv(this.addr,e),Lt(t,e)}}function Sx(s,e){let t=this.cache;t[0]!==e&&(s.uniform1ui(this.addr,e),t[0]=e)}function Ex(s,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(s.uniform2ui(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(Pt(t,e))return;s.uniform2uiv(this.addr,e),Lt(t,e)}}function Tx(s,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(s.uniform3ui(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(Pt(t,e))return;s.uniform3uiv(this.addr,e),Lt(t,e)}}function wx(s,e){let t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(s.uniform4ui(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(Pt(t,e))return;s.uniform4uiv(this.addr,e),Lt(t,e)}}function Ax(s,e,t){let n=this.cache,i=t.allocateTextureUnit();n[0]!==i&&(s.uniform1i(this.addr,i),n[0]=i);let r;this.type===s.SAMPLER_2D_SHADOW?(ef.compareFunction=wl,r=ef):r=yf,t.setTexture2D(e||r,i)}function Rx(s,e,t){let n=this.cache,i=t.allocateTextureUnit();n[0]!==i&&(s.uniform1i(this.addr,i),n[0]=i),t.setTexture3D(e||bf,i)}function Cx(s,e,t){let n=this.cache,i=t.allocateTextureUnit();n[0]!==i&&(s.uniform1i(this.addr,i),n[0]=i),t.setTextureCube(e||Mf,i)}function Ix(s,e,t){let n=this.cache,i=t.allocateTextureUnit();n[0]!==i&&(s.uniform1i(this.addr,i),n[0]=i),t.setTexture2DArray(e||vf,i)}function Px(s){switch(s){case 5126:return dx;case 35664:return fx;case 35665:return px;case 35666:return mx;case 35674:return gx;case 35675:return _x;case 35676:return xx;case 5124:case 35670:return yx;case 35667:case 35671:return vx;case 35668:case 35672:return bx;case 35669:case 35673:return Mx;case 5125:return Sx;case 36294:return Ex;case 36295:return Tx;case 36296:return wx;case 35678:case 36198:case 36298:case 36306:case 35682:return Ax;case 35679:case 36299:case 36307:return Rx;case 35680:case 36300:case 36308:case 36293:return Cx;case 36289:case 36303:case 36311:case 36292:return Ix}}function Lx(s,e){s.uniform1fv(this.addr,e)}function Nx(s,e){let t=dr(e,this.size,2);s.uniform2fv(this.addr,t)}function Dx(s,e){let t=dr(e,this.size,3);s.uniform3fv(this.addr,t)}function Ux(s,e){let t=dr(e,this.size,4);s.uniform4fv(this.addr,t)}function Ox(s,e){let t=dr(e,this.size,4);s.uniformMatrix2fv(this.addr,!1,t)}function Fx(s,e){let t=dr(e,this.size,9);s.uniformMatrix3fv(this.addr,!1,t)}function Bx(s,e){let t=dr(e,this.size,16);s.uniformMatrix4fv(this.addr,!1,t)}function kx(s,e){s.uniform1iv(this.addr,e)}function zx(s,e){s.uniform2iv(this.addr,e)}function Hx(s,e){s.uniform3iv(this.addr,e)}function Vx(s,e){s.uniform4iv(this.addr,e)}function Gx(s,e){s.uniform1uiv(this.addr,e)}function Wx(s,e){s.uniform2uiv(this.addr,e)}function Xx(s,e){s.uniform3uiv(this.addr,e)}function qx(s,e){s.uniform4uiv(this.addr,e)}function Yx(s,e,t){let n=this.cache,i=e.length,r=hc(t,i);Pt(n,r)||(s.uniform1iv(this.addr,r),Lt(n,r));for(let o=0;o!==i;++o)t.setTexture2D(e[o]||yf,r[o])}function jx(s,e,t){let n=this.cache,i=e.length,r=hc(t,i);Pt(n,r)||(s.uniform1iv(this.addr,r),Lt(n,r));for(let o=0;o!==i;++o)t.setTexture3D(e[o]||bf,r[o])}function Kx(s,e,t){let n=this.cache,i=e.length,r=hc(t,i);Pt(n,r)||(s.uniform1iv(this.addr,r),Lt(n,r));for(let o=0;o!==i;++o)t.setTextureCube(e[o]||Mf,r[o])}function Zx(s,e,t){let n=this.cache,i=e.length,r=hc(t,i);Pt(n,r)||(s.uniform1iv(this.addr,r),Lt(n,r));for(let o=0;o!==i;++o)t.setTexture2DArray(e[o]||vf,r[o])}function Jx(s){switch(s){case 5126:return Lx;case 35664:return Nx;case 35665:return Dx;case 35666:return Ux;case 35674:return Ox;case 35675:return Fx;case 35676:return Bx;case 5124:case 35670:return kx;case 35667:case 35671:return zx;case 35668:case 35672:return Hx;case 35669:case 35673:return Vx;case 5125:return Gx;case 36294:return Wx;case 36295:return Xx;case 36296:return qx;case 35678:case 36198:case 36298:case 36306:case 35682:return Yx;case 35679:case 36299:case 36307:return jx;case 35680:case 36300:case 36308:case 36293:return Kx;case 36289:case 36303:case 36311:case 36292:return Zx}}var zl=class{constructor(e,t,n){this.id=e,this.addr=n,this.cache=[],this.type=t.type,this.setValue=Px(t.type)}},Hl=class{constructor(e,t,n){this.id=e,this.addr=n,this.cache=[],this.type=t.type,this.size=t.size,this.setValue=Jx(t.type)}},Vl=class{constructor(e){this.id=e,this.seq=[],this.map={}}setValue(e,t,n){let i=this.seq;for(let r=0,o=i.length;r!==o;++r){let a=i[r];a.setValue(e,t[a.id],n)}}},kl=/(\w+)(\])?(\[|\.)?/g;function af(s,e){s.seq.push(e),s.map[e.id]=e}function $x(s,e,t){let n=s.name,i=n.length;for(kl.lastIndex=0;;){let r=kl.exec(n),o=kl.lastIndex,a=r[1],c=r[2]==="]",l=r[3];if(c&&(a=a|0),l===void 0||l==="["&&o+2===i){af(t,l===void 0?new zl(a,s,e):new Hl(a,s,e));break}else{let u=t.map[a];u===void 0&&(u=new Vl(a),af(t,u)),t=u}}}var ur=class{constructor(e,t){this.seq=[],this.map={};let n=e.getProgramParameter(t,e.ACTIVE_UNIFORMS);for(let i=0;i<n;++i){let r=e.getActiveUniform(t,i),o=e.getUniformLocation(t,r.name);$x(r,o,this)}}setValue(e,t,n,i){let r=this.map[t];r!==void 0&&r.setValue(e,n,i)}setOptional(e,t,n){let i=t[n];i!==void 0&&this.setValue(e,n,i)}static upload(e,t,n,i){for(let r=0,o=t.length;r!==o;++r){let a=t[r],c=n[a.id];c.needsUpdate!==!1&&a.setValue(e,c.value,i)}}static seqWithValue(e,t){let n=[];for(let i=0,r=e.length;i!==r;++i){let o=e[i];o.id in t&&n.push(o)}return n}};function cf(s,e,t){let n=s.createShader(e);return s.shaderSource(n,t),s.compileShader(n),n}var Qx=37297,e0=0;function t0(s,e){let t=s.split(`
`),n=[],i=Math.max(e-6,0),r=Math.min(e+6,t.length);for(let o=i;o<r;o++){let a=o+1;n.push(`${a===e?">":" "} ${a}: ${t[o]}`)}return n.join(`
`)}var lf=new Ve;function n0(s){$e._getMatrix(lf,$e.workingColorSpace,s);let e=`mat3( ${lf.elements.map(t=>t.toFixed(4))} )`;switch($e.getTransfer(s)){case Pr:return[e,"LinearTransferOETF"];case ot:return[e,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space: ",s),[e,"LinearTransferOETF"]}}function hf(s,e,t){let n=s.getShaderParameter(e,s.COMPILE_STATUS),r=(s.getShaderInfoLog(e)||"").trim();if(n&&r==="")return"";let o=/ERROR: 0:(\d+)/.exec(r);if(o){let a=parseInt(o[1]);return t.toUpperCase()+`

`+r+`

`+t0(s.getShaderSource(e),a)}else return r}function i0(s,e){let t=n0(e);return[`vec4 ${s}( vec4 value ) {`,`	return ${t[1]}( vec4( value.rgb * ${t[0]}, value.a ) );`,"}"].join(`
`)}function s0(s,e){let t;switch(e){case Sd:t="Linear";break;case Ed:t="Reinhard";break;case Td:t="Cineon";break;case ba:t="ACESFilmic";break;case Ad:t="AgX";break;case Rd:t="Neutral";break;case wd:t="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",e),t="Linear"}return"vec3 "+s+"( vec3 color ) { return "+t+"ToneMapping( color ); }"}var ac=new A;function r0(){$e.getLuminanceCoefficients(ac);let s=ac.x.toFixed(4),e=ac.y.toFixed(4),t=ac.z.toFixed(4);return["float luminance( const in vec3 rgb ) {",`	const vec3 weights = vec3( ${s}, ${e}, ${t} );`,"	return dot( weights, rgb );","}"].join(`
`)}function o0(s){return[s.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":"",s.extensionMultiDraw?"#extension GL_ANGLE_multi_draw : require":""].filter(uo).join(`
`)}function a0(s){let e=[];for(let t in s){let n=s[t];n!==!1&&e.push("#define "+t+" "+n)}return e.join(`
`)}function c0(s,e){let t={},n=s.getProgramParameter(e,s.ACTIVE_ATTRIBUTES);for(let i=0;i<n;i++){let r=s.getActiveAttrib(e,i),o=r.name,a=1;r.type===s.FLOAT_MAT2&&(a=2),r.type===s.FLOAT_MAT3&&(a=3),r.type===s.FLOAT_MAT4&&(a=4),t[o]={type:r.type,location:s.getAttribLocation(e,o),locationSize:a}}return t}function uo(s){return s!==""}function uf(s,e){let t=e.numSpotLightShadows+e.numSpotLightMaps-e.numSpotLightShadowsWithMaps;return s.replace(/NUM_DIR_LIGHTS/g,e.numDirLights).replace(/NUM_SPOT_LIGHTS/g,e.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,e.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,t).replace(/NUM_RECT_AREA_LIGHTS/g,e.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,e.numPointLights).replace(/NUM_HEMI_LIGHTS/g,e.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,e.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,e.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,e.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,e.numPointLightShadows)}function df(s,e){return s.replace(/NUM_CLIPPING_PLANES/g,e.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,e.numClippingPlanes-e.numClipIntersection)}var l0=/^[ \t]*#include +<([\w\d./]+)>/gm;function Gl(s){return s.replace(l0,u0)}var h0=new Map;function u0(s,e){let t=Xe[e];if(t===void 0){let n=h0.get(e);if(n!==void 0)t=Xe[n],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',e,n);else throw new Error("Can not resolve #include <"+e+">")}return Gl(t)}var d0=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function ff(s){return s.replace(d0,f0)}function f0(s,e,t,n){let i="";for(let r=parseInt(e);r<parseInt(t);r++)i+=n.replace(/\[\s*i\s*\]/g,"[ "+r+" ]").replace(/UNROLLED_LOOP_INDEX/g,r);return i}function pf(s){let e=`precision ${s.precision} float;
	precision ${s.precision} int;
	precision ${s.precision} sampler2D;
	precision ${s.precision} samplerCube;
	precision ${s.precision} sampler3D;
	precision ${s.precision} sampler2DArray;
	precision ${s.precision} sampler2DShadow;
	precision ${s.precision} samplerCubeShadow;
	precision ${s.precision} sampler2DArrayShadow;
	precision ${s.precision} isampler2D;
	precision ${s.precision} isampler3D;
	precision ${s.precision} isamplerCube;
	precision ${s.precision} isampler2DArray;
	precision ${s.precision} usampler2D;
	precision ${s.precision} usampler3D;
	precision ${s.precision} usamplerCube;
	precision ${s.precision} usampler2DArray;
	`;return s.precision==="highp"?e+=`
#define HIGH_PRECISION`:s.precision==="mediump"?e+=`
#define MEDIUM_PRECISION`:s.precision==="lowp"&&(e+=`
#define LOW_PRECISION`),e}function p0(s){let e="SHADOWMAP_TYPE_BASIC";return s.shadowMapType===ul?e="SHADOWMAP_TYPE_PCF":s.shadowMapType===fa?e="SHADOWMAP_TYPE_PCF_SOFT":s.shadowMapType===ni&&(e="SHADOWMAP_TYPE_VSM"),e}function m0(s){let e="ENVMAP_TYPE_CUBE";if(s.envMap)switch(s.envMapMode){case us:case ds:e="ENVMAP_TYPE_CUBE";break;case ro:e="ENVMAP_TYPE_CUBE_UV";break}return e}function g0(s){let e="ENVMAP_MODE_REFLECTION";if(s.envMap)switch(s.envMapMode){case ds:e="ENVMAP_MODE_REFRACTION";break}return e}function _0(s){let e="ENVMAP_BLENDING_NONE";if(s.envMap)switch(s.combine){case ml:e="ENVMAP_BLENDING_MULTIPLY";break;case bd:e="ENVMAP_BLENDING_MIX";break;case Md:e="ENVMAP_BLENDING_ADD";break}return e}function x0(s){let e=s.envMapCubeUVHeight;if(e===null)return null;let t=Math.log2(e)-2,n=1/e;return{texelWidth:1/(3*Math.max(Math.pow(2,t),112)),texelHeight:n,maxMip:t}}function y0(s,e,t,n){let i=s.getContext(),r=t.defines,o=t.vertexShader,a=t.fragmentShader,c=p0(t),l=m0(t),h=g0(t),u=_0(t),d=x0(t),f=o0(t),g=a0(r),_=i.createProgram(),m,p,S=t.glslVersion?"#version "+t.glslVersion+`
`:"";t.isRawShaderMaterial?(m=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g].filter(uo).join(`
`),m.length>0&&(m+=`
`),p=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g].filter(uo).join(`
`),p.length>0&&(p+=`
`)):(m=[pf(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g,t.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",t.batching?"#define USE_BATCHING":"",t.batchingColor?"#define USE_BATCHING_COLOR":"",t.instancing?"#define USE_INSTANCING":"",t.instancingColor?"#define USE_INSTANCING_COLOR":"",t.instancingMorph?"#define USE_INSTANCING_MORPH":"",t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.map?"#define USE_MAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+h:"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.displacementMap?"#define USE_DISPLACEMENTMAP":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.mapUv?"#define MAP_UV "+t.mapUv:"",t.alphaMapUv?"#define ALPHAMAP_UV "+t.alphaMapUv:"",t.lightMapUv?"#define LIGHTMAP_UV "+t.lightMapUv:"",t.aoMapUv?"#define AOMAP_UV "+t.aoMapUv:"",t.emissiveMapUv?"#define EMISSIVEMAP_UV "+t.emissiveMapUv:"",t.bumpMapUv?"#define BUMPMAP_UV "+t.bumpMapUv:"",t.normalMapUv?"#define NORMALMAP_UV "+t.normalMapUv:"",t.displacementMapUv?"#define DISPLACEMENTMAP_UV "+t.displacementMapUv:"",t.metalnessMapUv?"#define METALNESSMAP_UV "+t.metalnessMapUv:"",t.roughnessMapUv?"#define ROUGHNESSMAP_UV "+t.roughnessMapUv:"",t.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+t.anisotropyMapUv:"",t.clearcoatMapUv?"#define CLEARCOATMAP_UV "+t.clearcoatMapUv:"",t.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+t.clearcoatNormalMapUv:"",t.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+t.clearcoatRoughnessMapUv:"",t.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+t.iridescenceMapUv:"",t.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+t.iridescenceThicknessMapUv:"",t.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+t.sheenColorMapUv:"",t.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+t.sheenRoughnessMapUv:"",t.specularMapUv?"#define SPECULARMAP_UV "+t.specularMapUv:"",t.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+t.specularColorMapUv:"",t.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+t.specularIntensityMapUv:"",t.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+t.transmissionMapUv:"",t.thicknessMapUv?"#define THICKNESSMAP_UV "+t.thicknessMapUv:"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.flatShading?"#define FLAT_SHADED":"",t.skinning?"#define USE_SKINNING":"",t.morphTargets?"#define USE_MORPHTARGETS":"",t.morphNormals&&t.flatShading===!1?"#define USE_MORPHNORMALS":"",t.morphColors?"#define USE_MORPHCOLORS":"",t.morphTargetsCount>0?"#define MORPHTARGETS_TEXTURE_STRIDE "+t.morphTextureStride:"",t.morphTargetsCount>0?"#define MORPHTARGETS_COUNT "+t.morphTargetsCount:"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+c:"",t.sizeAttenuation?"#define USE_SIZEATTENUATION":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.logarithmicDepthBuffer?"#define USE_LOGARITHMIC_DEPTH_BUFFER":"",t.reversedDepthBuffer?"#define USE_REVERSED_DEPTH_BUFFER":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","#ifdef USE_INSTANCING_MORPH","	uniform sampler2D morphTexture;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(uo).join(`
`),p=[pf(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g,t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.alphaToCoverage?"#define ALPHA_TO_COVERAGE":"",t.map?"#define USE_MAP":"",t.matcap?"#define USE_MATCAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+l:"",t.envMap?"#define "+h:"",t.envMap?"#define "+u:"",d?"#define CUBEUV_TEXEL_WIDTH "+d.texelWidth:"",d?"#define CUBEUV_TEXEL_HEIGHT "+d.texelHeight:"",d?"#define CUBEUV_MAX_MIP "+d.maxMip+".0":"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoat?"#define USE_CLEARCOAT":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.dispersion?"#define USE_DISPERSION":"",t.iridescence?"#define USE_IRIDESCENCE":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaTest?"#define USE_ALPHATEST":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.sheen?"#define USE_SHEEN":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors||t.instancingColor||t.batchingColor?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.gradientMap?"#define USE_GRADIENTMAP":"",t.flatShading?"#define FLAT_SHADED":"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+c:"",t.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",t.decodeVideoTextureEmissive?"#define DECODE_VIDEO_TEXTURE_EMISSIVE":"",t.logarithmicDepthBuffer?"#define USE_LOGARITHMIC_DEPTH_BUFFER":"",t.reversedDepthBuffer?"#define USE_REVERSED_DEPTH_BUFFER":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",t.toneMapping!==Si?"#define TONE_MAPPING":"",t.toneMapping!==Si?Xe.tonemapping_pars_fragment:"",t.toneMapping!==Si?s0("toneMapping",t.toneMapping):"",t.dithering?"#define DITHERING":"",t.opaque?"#define OPAQUE":"",Xe.colorspace_pars_fragment,i0("linearToOutputTexel",t.outputColorSpace),r0(),t.useDepthPacking?"#define DEPTH_PACKING "+t.depthPacking:"",`
`].filter(uo).join(`
`)),o=Gl(o),o=uf(o,t),o=df(o,t),a=Gl(a),a=uf(a,t),a=df(a,t),o=ff(o),a=ff(a),t.isRawShaderMaterial!==!0&&(S=`#version 300 es
`,m=[f,"#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+m,p=["#define varying in",t.glslVersion===Al?"":"layout(location = 0) out highp vec4 pc_fragColor;",t.glslVersion===Al?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+p);let E=S+m+o,y=S+p+a,w=cf(i,i.VERTEX_SHADER,E),C=cf(i,i.FRAGMENT_SHADER,y);i.attachShader(_,w),i.attachShader(_,C),t.index0AttributeName!==void 0?i.bindAttribLocation(_,0,t.index0AttributeName):t.morphTargets===!0&&i.bindAttribLocation(_,0,"position"),i.linkProgram(_);function R(P){if(s.debug.checkShaderErrors){let H=i.getProgramInfoLog(_)||"",G=i.getShaderInfoLog(w)||"",q=i.getShaderInfoLog(C)||"",W=H.trim(),Y=G.trim(),B=q.trim(),D=!0,Q=!0;if(i.getProgramParameter(_,i.LINK_STATUS)===!1)if(D=!1,typeof s.debug.onShaderError=="function")s.debug.onShaderError(i,_,w,C);else{let ae=hf(i,w,"vertex"),ce=hf(i,C,"fragment");console.error("THREE.WebGLProgram: Shader Error "+i.getError()+" - VALIDATE_STATUS "+i.getProgramParameter(_,i.VALIDATE_STATUS)+`

Material Name: `+P.name+`
Material Type: `+P.type+`

Program Info Log: `+W+`
`+ae+`
`+ce)}else W!==""?console.warn("THREE.WebGLProgram: Program Info Log:",W):(Y===""||B==="")&&(Q=!1);Q&&(P.diagnostics={runnable:D,programLog:W,vertexShader:{log:Y,prefix:m},fragmentShader:{log:B,prefix:p}})}i.deleteShader(w),i.deleteShader(C),N=new ur(i,_),M=c0(i,_)}let N;this.getUniforms=function(){return N===void 0&&R(this),N};let M;this.getAttributes=function(){return M===void 0&&R(this),M};let b=t.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return b===!1&&(b=i.getProgramParameter(_,Qx)),b},this.destroy=function(){n.releaseStatesOfProgram(this),i.deleteProgram(_),this.program=void 0},this.type=t.shaderType,this.name=t.shaderName,this.id=e0++,this.cacheKey=e,this.usedTimes=1,this.program=_,this.vertexShader=w,this.fragmentShader=C,this}var v0=0,Wl=class{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(e){let t=e.vertexShader,n=e.fragmentShader,i=this._getShaderStage(t),r=this._getShaderStage(n),o=this._getShaderCacheForMaterial(e);return o.has(i)===!1&&(o.add(i),i.usedTimes++),o.has(r)===!1&&(o.add(r),r.usedTimes++),this}remove(e){let t=this.materialCache.get(e);for(let n of t)n.usedTimes--,n.usedTimes===0&&this.shaderCache.delete(n.code);return this.materialCache.delete(e),this}getVertexShaderID(e){return this._getShaderStage(e.vertexShader).id}getFragmentShaderID(e){return this._getShaderStage(e.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(e){let t=this.materialCache,n=t.get(e);return n===void 0&&(n=new Set,t.set(e,n)),n}_getShaderStage(e){let t=this.shaderCache,n=t.get(e);return n===void 0&&(n=new Xl(e),t.set(e,n)),n}},Xl=class{constructor(e){this.id=v0++,this.code=e,this.usedTimes=0}};function b0(s,e,t,n,i,r,o){let a=new js,c=new Wl,l=new Set,h=[],u=i.logarithmicDepthBuffer,d=i.vertexTextures,f=i.precision,g={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function _(M){return l.add(M),M===0?"uv":`uv${M}`}function m(M,b,P,H,G){let q=H.fog,W=G.geometry,Y=M.isMeshStandardMaterial?H.environment:null,B=(M.isMeshStandardMaterial?t:e).get(M.envMap||Y),D=B&&B.mapping===ro?B.image.height:null,Q=g[M.type];M.precision!==null&&(f=i.getMaxPrecision(M.precision),f!==M.precision&&console.warn("THREE.WebGLProgram.getParameters:",M.precision,"not supported, using",f,"instead."));let ae=W.morphAttributes.position||W.morphAttributes.normal||W.morphAttributes.color,ce=ae!==void 0?ae.length:0,ve=0;W.morphAttributes.position!==void 0&&(ve=1),W.morphAttributes.normal!==void 0&&(ve=2),W.morphAttributes.color!==void 0&&(ve=3);let Ze,it,Ye,K;if(Q){let st=ii[Q];Ze=st.vertexShader,it=st.fragmentShader}else Ze=M.vertexShader,it=M.fragmentShader,c.update(M),Ye=c.getVertexShaderID(M),K=c.getFragmentShaderID(M);let te=s.getRenderTarget(),_e=s.state.buffers.depth.getReversed(),ke=G.isInstancedMesh===!0,Ce=G.isBatchedMesh===!0,Qe=!!M.map,Wt=!!M.matcap,I=!!B,mt=!!M.aoMap,He=!!M.lightMap,Ue=!!M.bumpMap,be=!!M.normalMap,gt=!!M.displacementMap,Me=!!M.emissiveMap,We=!!M.metalnessMap,kt=!!M.roughnessMap,St=M.anisotropy>0,T=M.clearcoat>0,x=M.dispersion>0,k=M.iridescence>0,j=M.sheen>0,ee=M.transmission>0,X=St&&!!M.anisotropyMap,Ae=T&&!!M.clearcoatMap,oe=T&&!!M.clearcoatNormalMap,Se=T&&!!M.clearcoatRoughnessMap,Te=k&&!!M.iridescenceMap,se=k&&!!M.iridescenceThicknessMap,pe=j&&!!M.sheenColorMap,Ne=j&&!!M.sheenRoughnessMap,we=!!M.specularMap,de=!!M.specularColorMap,Ge=!!M.specularIntensityMap,L=ee&&!!M.transmissionMap,re=ee&&!!M.thicknessMap,le=!!M.gradientMap,ge=!!M.alphaMap,ne=M.alphaTest>0,Z=!!M.alphaHash,ye=!!M.extensions,ze=Si;M.toneMapped&&(te===null||te.isXRRenderTarget===!0)&&(ze=s.toneMapping);let dt={shaderID:Q,shaderType:M.type,shaderName:M.name,vertexShader:Ze,fragmentShader:it,defines:M.defines,customVertexShaderID:Ye,customFragmentShaderID:K,isRawShaderMaterial:M.isRawShaderMaterial===!0,glslVersion:M.glslVersion,precision:f,batching:Ce,batchingColor:Ce&&G._colorsTexture!==null,instancing:ke,instancingColor:ke&&G.instanceColor!==null,instancingMorph:ke&&G.morphTexture!==null,supportsVertexTextures:d,outputColorSpace:te===null?s.outputColorSpace:te.isXRRenderTarget===!0?te.texture.colorSpace:Vt,alphaToCoverage:!!M.alphaToCoverage,map:Qe,matcap:Wt,envMap:I,envMapMode:I&&B.mapping,envMapCubeUVHeight:D,aoMap:mt,lightMap:He,bumpMap:Ue,normalMap:be,displacementMap:d&&gt,emissiveMap:Me,normalMapObjectSpace:be&&M.normalMapType===Nd,normalMapTangentSpace:be&&M.normalMapType===Tl,metalnessMap:We,roughnessMap:kt,anisotropy:St,anisotropyMap:X,clearcoat:T,clearcoatMap:Ae,clearcoatNormalMap:oe,clearcoatRoughnessMap:Se,dispersion:x,iridescence:k,iridescenceMap:Te,iridescenceThicknessMap:se,sheen:j,sheenColorMap:pe,sheenRoughnessMap:Ne,specularMap:we,specularColorMap:de,specularIntensityMap:Ge,transmission:ee,transmissionMap:L,thicknessMap:re,gradientMap:le,opaque:M.transparent===!1&&M.blending===ns&&M.alphaToCoverage===!1,alphaMap:ge,alphaTest:ne,alphaHash:Z,combine:M.combine,mapUv:Qe&&_(M.map.channel),aoMapUv:mt&&_(M.aoMap.channel),lightMapUv:He&&_(M.lightMap.channel),bumpMapUv:Ue&&_(M.bumpMap.channel),normalMapUv:be&&_(M.normalMap.channel),displacementMapUv:gt&&_(M.displacementMap.channel),emissiveMapUv:Me&&_(M.emissiveMap.channel),metalnessMapUv:We&&_(M.metalnessMap.channel),roughnessMapUv:kt&&_(M.roughnessMap.channel),anisotropyMapUv:X&&_(M.anisotropyMap.channel),clearcoatMapUv:Ae&&_(M.clearcoatMap.channel),clearcoatNormalMapUv:oe&&_(M.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:Se&&_(M.clearcoatRoughnessMap.channel),iridescenceMapUv:Te&&_(M.iridescenceMap.channel),iridescenceThicknessMapUv:se&&_(M.iridescenceThicknessMap.channel),sheenColorMapUv:pe&&_(M.sheenColorMap.channel),sheenRoughnessMapUv:Ne&&_(M.sheenRoughnessMap.channel),specularMapUv:we&&_(M.specularMap.channel),specularColorMapUv:de&&_(M.specularColorMap.channel),specularIntensityMapUv:Ge&&_(M.specularIntensityMap.channel),transmissionMapUv:L&&_(M.transmissionMap.channel),thicknessMapUv:re&&_(M.thicknessMap.channel),alphaMapUv:ge&&_(M.alphaMap.channel),vertexTangents:!!W.attributes.tangent&&(be||St),vertexColors:M.vertexColors,vertexAlphas:M.vertexColors===!0&&!!W.attributes.color&&W.attributes.color.itemSize===4,pointsUvs:G.isPoints===!0&&!!W.attributes.uv&&(Qe||ge),fog:!!q,useFog:M.fog===!0,fogExp2:!!q&&q.isFogExp2,flatShading:M.flatShading===!0&&M.wireframe===!1,sizeAttenuation:M.sizeAttenuation===!0,logarithmicDepthBuffer:u,reversedDepthBuffer:_e,skinning:G.isSkinnedMesh===!0,morphTargets:W.morphAttributes.position!==void 0,morphNormals:W.morphAttributes.normal!==void 0,morphColors:W.morphAttributes.color!==void 0,morphTargetsCount:ce,morphTextureStride:ve,numDirLights:b.directional.length,numPointLights:b.point.length,numSpotLights:b.spot.length,numSpotLightMaps:b.spotLightMap.length,numRectAreaLights:b.rectArea.length,numHemiLights:b.hemi.length,numDirLightShadows:b.directionalShadowMap.length,numPointLightShadows:b.pointShadowMap.length,numSpotLightShadows:b.spotShadowMap.length,numSpotLightShadowsWithMaps:b.numSpotLightShadowsWithMaps,numLightProbes:b.numLightProbes,numClippingPlanes:o.numPlanes,numClipIntersection:o.numIntersection,dithering:M.dithering,shadowMapEnabled:s.shadowMap.enabled&&P.length>0,shadowMapType:s.shadowMap.type,toneMapping:ze,decodeVideoTexture:Qe&&M.map.isVideoTexture===!0&&$e.getTransfer(M.map.colorSpace)===ot,decodeVideoTextureEmissive:Me&&M.emissiveMap.isVideoTexture===!0&&$e.getTransfer(M.emissiveMap.colorSpace)===ot,premultipliedAlpha:M.premultipliedAlpha,doubleSided:M.side===Kt,flipSided:M.side===en,useDepthPacking:M.depthPacking>=0,depthPacking:M.depthPacking||0,index0AttributeName:M.index0AttributeName,extensionClipCullDistance:ye&&M.extensions.clipCullDistance===!0&&n.has("WEBGL_clip_cull_distance"),extensionMultiDraw:(ye&&M.extensions.multiDraw===!0||Ce)&&n.has("WEBGL_multi_draw"),rendererExtensionParallelShaderCompile:n.has("KHR_parallel_shader_compile"),customProgramCacheKey:M.customProgramCacheKey()};return dt.vertexUv1s=l.has(1),dt.vertexUv2s=l.has(2),dt.vertexUv3s=l.has(3),l.clear(),dt}function p(M){let b=[];if(M.shaderID?b.push(M.shaderID):(b.push(M.customVertexShaderID),b.push(M.customFragmentShaderID)),M.defines!==void 0)for(let P in M.defines)b.push(P),b.push(M.defines[P]);return M.isRawShaderMaterial===!1&&(S(b,M),E(b,M),b.push(s.outputColorSpace)),b.push(M.customProgramCacheKey),b.join()}function S(M,b){M.push(b.precision),M.push(b.outputColorSpace),M.push(b.envMapMode),M.push(b.envMapCubeUVHeight),M.push(b.mapUv),M.push(b.alphaMapUv),M.push(b.lightMapUv),M.push(b.aoMapUv),M.push(b.bumpMapUv),M.push(b.normalMapUv),M.push(b.displacementMapUv),M.push(b.emissiveMapUv),M.push(b.metalnessMapUv),M.push(b.roughnessMapUv),M.push(b.anisotropyMapUv),M.push(b.clearcoatMapUv),M.push(b.clearcoatNormalMapUv),M.push(b.clearcoatRoughnessMapUv),M.push(b.iridescenceMapUv),M.push(b.iridescenceThicknessMapUv),M.push(b.sheenColorMapUv),M.push(b.sheenRoughnessMapUv),M.push(b.specularMapUv),M.push(b.specularColorMapUv),M.push(b.specularIntensityMapUv),M.push(b.transmissionMapUv),M.push(b.thicknessMapUv),M.push(b.combine),M.push(b.fogExp2),M.push(b.sizeAttenuation),M.push(b.morphTargetsCount),M.push(b.morphAttributeCount),M.push(b.numDirLights),M.push(b.numPointLights),M.push(b.numSpotLights),M.push(b.numSpotLightMaps),M.push(b.numHemiLights),M.push(b.numRectAreaLights),M.push(b.numDirLightShadows),M.push(b.numPointLightShadows),M.push(b.numSpotLightShadows),M.push(b.numSpotLightShadowsWithMaps),M.push(b.numLightProbes),M.push(b.shadowMapType),M.push(b.toneMapping),M.push(b.numClippingPlanes),M.push(b.numClipIntersection),M.push(b.depthPacking)}function E(M,b){a.disableAll(),b.supportsVertexTextures&&a.enable(0),b.instancing&&a.enable(1),b.instancingColor&&a.enable(2),b.instancingMorph&&a.enable(3),b.matcap&&a.enable(4),b.envMap&&a.enable(5),b.normalMapObjectSpace&&a.enable(6),b.normalMapTangentSpace&&a.enable(7),b.clearcoat&&a.enable(8),b.iridescence&&a.enable(9),b.alphaTest&&a.enable(10),b.vertexColors&&a.enable(11),b.vertexAlphas&&a.enable(12),b.vertexUv1s&&a.enable(13),b.vertexUv2s&&a.enable(14),b.vertexUv3s&&a.enable(15),b.vertexTangents&&a.enable(16),b.anisotropy&&a.enable(17),b.alphaHash&&a.enable(18),b.batching&&a.enable(19),b.dispersion&&a.enable(20),b.batchingColor&&a.enable(21),b.gradientMap&&a.enable(22),M.push(a.mask),a.disableAll(),b.fog&&a.enable(0),b.useFog&&a.enable(1),b.flatShading&&a.enable(2),b.logarithmicDepthBuffer&&a.enable(3),b.reversedDepthBuffer&&a.enable(4),b.skinning&&a.enable(5),b.morphTargets&&a.enable(6),b.morphNormals&&a.enable(7),b.morphColors&&a.enable(8),b.premultipliedAlpha&&a.enable(9),b.shadowMapEnabled&&a.enable(10),b.doubleSided&&a.enable(11),b.flipSided&&a.enable(12),b.useDepthPacking&&a.enable(13),b.dithering&&a.enable(14),b.transmission&&a.enable(15),b.sheen&&a.enable(16),b.opaque&&a.enable(17),b.pointsUvs&&a.enable(18),b.decodeVideoTexture&&a.enable(19),b.decodeVideoTextureEmissive&&a.enable(20),b.alphaToCoverage&&a.enable(21),M.push(a.mask)}function y(M){let b=g[M.type],P;if(b){let H=ii[b];P=Wd.clone(H.uniforms)}else P=M.uniforms;return P}function w(M,b){let P;for(let H=0,G=h.length;H<G;H++){let q=h[H];if(q.cacheKey===b){P=q,++P.usedTimes;break}}return P===void 0&&(P=new y0(s,b,M,r),h.push(P)),P}function C(M){if(--M.usedTimes===0){let b=h.indexOf(M);h[b]=h[h.length-1],h.pop(),M.destroy()}}function R(M){c.remove(M)}function N(){c.dispose()}return{getParameters:m,getProgramCacheKey:p,getUniforms:y,acquireProgram:w,releaseProgram:C,releaseShaderCache:R,programs:h,dispose:N}}function M0(){let s=new WeakMap;function e(o){return s.has(o)}function t(o){let a=s.get(o);return a===void 0&&(a={},s.set(o,a)),a}function n(o){s.delete(o)}function i(o,a,c){s.get(o)[a]=c}function r(){s=new WeakMap}return{has:e,get:t,remove:n,update:i,dispose:r}}function S0(s,e){return s.groupOrder!==e.groupOrder?s.groupOrder-e.groupOrder:s.renderOrder!==e.renderOrder?s.renderOrder-e.renderOrder:s.material.id!==e.material.id?s.material.id-e.material.id:s.z!==e.z?s.z-e.z:s.id-e.id}function mf(s,e){return s.groupOrder!==e.groupOrder?s.groupOrder-e.groupOrder:s.renderOrder!==e.renderOrder?s.renderOrder-e.renderOrder:s.z!==e.z?e.z-s.z:s.id-e.id}function gf(){let s=[],e=0,t=[],n=[],i=[];function r(){e=0,t.length=0,n.length=0,i.length=0}function o(u,d,f,g,_,m){let p=s[e];return p===void 0?(p={id:u.id,object:u,geometry:d,material:f,groupOrder:g,renderOrder:u.renderOrder,z:_,group:m},s[e]=p):(p.id=u.id,p.object=u,p.geometry=d,p.material=f,p.groupOrder=g,p.renderOrder=u.renderOrder,p.z=_,p.group=m),e++,p}function a(u,d,f,g,_,m){let p=o(u,d,f,g,_,m);f.transmission>0?n.push(p):f.transparent===!0?i.push(p):t.push(p)}function c(u,d,f,g,_,m){let p=o(u,d,f,g,_,m);f.transmission>0?n.unshift(p):f.transparent===!0?i.unshift(p):t.unshift(p)}function l(u,d){t.length>1&&t.sort(u||S0),n.length>1&&n.sort(d||mf),i.length>1&&i.sort(d||mf)}function h(){for(let u=e,d=s.length;u<d;u++){let f=s[u];if(f.id===null)break;f.id=null,f.object=null,f.geometry=null,f.material=null,f.group=null}}return{opaque:t,transmissive:n,transparent:i,init:r,push:a,unshift:c,finish:h,sort:l}}function E0(){let s=new WeakMap;function e(n,i){let r=s.get(n),o;return r===void 0?(o=new gf,s.set(n,[o])):i>=r.length?(o=new gf,r.push(o)):o=r[i],o}function t(){s=new WeakMap}return{get:e,dispose:t}}function T0(){let s={};return{get:function(e){if(s[e.id]!==void 0)return s[e.id];let t;switch(e.type){case"DirectionalLight":t={direction:new A,color:new Re};break;case"SpotLight":t={position:new A,direction:new A,color:new Re,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":t={position:new A,color:new Re,distance:0,decay:0};break;case"HemisphereLight":t={direction:new A,skyColor:new Re,groundColor:new Re};break;case"RectAreaLight":t={color:new Re,position:new A,halfWidth:new A,halfHeight:new A};break}return s[e.id]=t,t}}}function w0(){let s={};return{get:function(e){if(s[e.id]!==void 0)return s[e.id];let t;switch(e.type){case"DirectionalLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Ee};break;case"SpotLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Ee};break;case"PointLight":t={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Ee,shadowCameraNear:1,shadowCameraFar:1e3};break}return s[e.id]=t,t}}}var A0=0;function R0(s,e){return(e.castShadow?2:0)-(s.castShadow?2:0)+(e.map?1:0)-(s.map?1:0)}function C0(s){let e=new T0,t=w0(),n={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let l=0;l<9;l++)n.probe.push(new A);let i=new A,r=new De,o=new De;function a(l){let h=0,u=0,d=0;for(let M=0;M<9;M++)n.probe[M].set(0,0,0);let f=0,g=0,_=0,m=0,p=0,S=0,E=0,y=0,w=0,C=0,R=0;l.sort(R0);for(let M=0,b=l.length;M<b;M++){let P=l[M],H=P.color,G=P.intensity,q=P.distance,W=P.shadow&&P.shadow.map?P.shadow.map.texture:null;if(P.isAmbientLight)h+=H.r*G,u+=H.g*G,d+=H.b*G;else if(P.isLightProbe){for(let Y=0;Y<9;Y++)n.probe[Y].addScaledVector(P.sh.coefficients[Y],G);R++}else if(P.isDirectionalLight){let Y=e.get(P);if(Y.color.copy(P.color).multiplyScalar(P.intensity),P.castShadow){let B=P.shadow,D=t.get(P);D.shadowIntensity=B.intensity,D.shadowBias=B.bias,D.shadowNormalBias=B.normalBias,D.shadowRadius=B.radius,D.shadowMapSize=B.mapSize,n.directionalShadow[f]=D,n.directionalShadowMap[f]=W,n.directionalShadowMatrix[f]=P.shadow.matrix,S++}n.directional[f]=Y,f++}else if(P.isSpotLight){let Y=e.get(P);Y.position.setFromMatrixPosition(P.matrixWorld),Y.color.copy(H).multiplyScalar(G),Y.distance=q,Y.coneCos=Math.cos(P.angle),Y.penumbraCos=Math.cos(P.angle*(1-P.penumbra)),Y.decay=P.decay,n.spot[_]=Y;let B=P.shadow;if(P.map&&(n.spotLightMap[w]=P.map,w++,B.updateMatrices(P),P.castShadow&&C++),n.spotLightMatrix[_]=B.matrix,P.castShadow){let D=t.get(P);D.shadowIntensity=B.intensity,D.shadowBias=B.bias,D.shadowNormalBias=B.normalBias,D.shadowRadius=B.radius,D.shadowMapSize=B.mapSize,n.spotShadow[_]=D,n.spotShadowMap[_]=W,y++}_++}else if(P.isRectAreaLight){let Y=e.get(P);Y.color.copy(H).multiplyScalar(G),Y.halfWidth.set(P.width*.5,0,0),Y.halfHeight.set(0,P.height*.5,0),n.rectArea[m]=Y,m++}else if(P.isPointLight){let Y=e.get(P);if(Y.color.copy(P.color).multiplyScalar(P.intensity),Y.distance=P.distance,Y.decay=P.decay,P.castShadow){let B=P.shadow,D=t.get(P);D.shadowIntensity=B.intensity,D.shadowBias=B.bias,D.shadowNormalBias=B.normalBias,D.shadowRadius=B.radius,D.shadowMapSize=B.mapSize,D.shadowCameraNear=B.camera.near,D.shadowCameraFar=B.camera.far,n.pointShadow[g]=D,n.pointShadowMap[g]=W,n.pointShadowMatrix[g]=P.shadow.matrix,E++}n.point[g]=Y,g++}else if(P.isHemisphereLight){let Y=e.get(P);Y.skyColor.copy(P.color).multiplyScalar(G),Y.groundColor.copy(P.groundColor).multiplyScalar(G),n.hemi[p]=Y,p++}}m>0&&(s.has("OES_texture_float_linear")===!0?(n.rectAreaLTC1=he.LTC_FLOAT_1,n.rectAreaLTC2=he.LTC_FLOAT_2):(n.rectAreaLTC1=he.LTC_HALF_1,n.rectAreaLTC2=he.LTC_HALF_2)),n.ambient[0]=h,n.ambient[1]=u,n.ambient[2]=d;let N=n.hash;(N.directionalLength!==f||N.pointLength!==g||N.spotLength!==_||N.rectAreaLength!==m||N.hemiLength!==p||N.numDirectionalShadows!==S||N.numPointShadows!==E||N.numSpotShadows!==y||N.numSpotMaps!==w||N.numLightProbes!==R)&&(n.directional.length=f,n.spot.length=_,n.rectArea.length=m,n.point.length=g,n.hemi.length=p,n.directionalShadow.length=S,n.directionalShadowMap.length=S,n.pointShadow.length=E,n.pointShadowMap.length=E,n.spotShadow.length=y,n.spotShadowMap.length=y,n.directionalShadowMatrix.length=S,n.pointShadowMatrix.length=E,n.spotLightMatrix.length=y+w-C,n.spotLightMap.length=w,n.numSpotLightShadowsWithMaps=C,n.numLightProbes=R,N.directionalLength=f,N.pointLength=g,N.spotLength=_,N.rectAreaLength=m,N.hemiLength=p,N.numDirectionalShadows=S,N.numPointShadows=E,N.numSpotShadows=y,N.numSpotMaps=w,N.numLightProbes=R,n.version=A0++)}function c(l,h){let u=0,d=0,f=0,g=0,_=0,m=h.matrixWorldInverse;for(let p=0,S=l.length;p<S;p++){let E=l[p];if(E.isDirectionalLight){let y=n.directional[u];y.direction.setFromMatrixPosition(E.matrixWorld),i.setFromMatrixPosition(E.target.matrixWorld),y.direction.sub(i),y.direction.transformDirection(m),u++}else if(E.isSpotLight){let y=n.spot[f];y.position.setFromMatrixPosition(E.matrixWorld),y.position.applyMatrix4(m),y.direction.setFromMatrixPosition(E.matrixWorld),i.setFromMatrixPosition(E.target.matrixWorld),y.direction.sub(i),y.direction.transformDirection(m),f++}else if(E.isRectAreaLight){let y=n.rectArea[g];y.position.setFromMatrixPosition(E.matrixWorld),y.position.applyMatrix4(m),o.identity(),r.copy(E.matrixWorld),r.premultiply(m),o.extractRotation(r),y.halfWidth.set(E.width*.5,0,0),y.halfHeight.set(0,E.height*.5,0),y.halfWidth.applyMatrix4(o),y.halfHeight.applyMatrix4(o),g++}else if(E.isPointLight){let y=n.point[d];y.position.setFromMatrixPosition(E.matrixWorld),y.position.applyMatrix4(m),d++}else if(E.isHemisphereLight){let y=n.hemi[_];y.direction.setFromMatrixPosition(E.matrixWorld),y.direction.transformDirection(m),_++}}}return{setup:a,setupView:c,state:n}}function _f(s){let e=new C0(s),t=[],n=[];function i(h){l.camera=h,t.length=0,n.length=0}function r(h){t.push(h)}function o(h){n.push(h)}function a(){e.setup(t)}function c(h){e.setupView(t,h)}let l={lightsArray:t,shadowsArray:n,camera:null,lights:e,transmissionRenderTarget:{}};return{init:i,state:l,setupLights:a,setupLightsView:c,pushLight:r,pushShadow:o}}function I0(s){let e=new WeakMap;function t(i,r=0){let o=e.get(i),a;return o===void 0?(a=new _f(s),e.set(i,[a])):r>=o.length?(a=new _f(s),o.push(a)):a=o[r],a}function n(){e=new WeakMap}return{get:t,dispose:n}}var P0=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,L0=`uniform sampler2D shadow_pass;
uniform vec2 resolution;
uniform float radius;
#include <packing>
void main() {
	const float samples = float( VSM_SAMPLES );
	float mean = 0.0;
	float squared_mean = 0.0;
	float uvStride = samples <= 1.0 ? 0.0 : 2.0 / ( samples - 1.0 );
	float uvStart = samples <= 1.0 ? 0.0 : - 1.0;
	for ( float i = 0.0; i < samples; i ++ ) {
		float uvOffset = uvStart + i * uvStride;
		#ifdef HORIZONTAL_PASS
			vec2 distribution = unpackRGBATo2Half( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( uvOffset, 0.0 ) * radius ) / resolution ) );
			mean += distribution.x;
			squared_mean += distribution.y * distribution.y + distribution.x * distribution.x;
		#else
			float depth = unpackRGBAToDepth( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( 0.0, uvOffset ) * radius ) / resolution ) );
			mean += depth;
			squared_mean += depth * depth;
		#endif
	}
	mean = mean / samples;
	squared_mean = squared_mean / samples;
	float std_dev = sqrt( squared_mean - mean * mean );
	gl_FragColor = pack2HalfToRGBA( vec2( mean, std_dev ) );
}`;function N0(s,e,t){let n=new Qs,i=new Ee,r=new Ee,o=new nt,a=new sa({depthPacking:Ld}),c=new ra,l={},h=t.maxTextureSize,u={[mn]:en,[en]:mn,[Kt]:Kt},d=new Bn({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new Ee},radius:{value:4}},vertexShader:P0,fragmentShader:L0}),f=d.clone();f.defines.HORIZONTAL_PASS=1;let g=new xt;g.setAttribute("position",new at(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));let _=new lt(g,d),m=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=ul;let p=this.type;this.render=function(C,R,N){if(m.enabled===!1||m.autoUpdate===!1&&m.needsUpdate===!1||C.length===0)return;let M=s.getRenderTarget(),b=s.getActiveCubeFace(),P=s.getActiveMipmapLevel(),H=s.state;H.setBlending(Mi),H.buffers.depth.getReversed()===!0?H.buffers.color.setClear(0,0,0,0):H.buffers.color.setClear(1,1,1,1),H.buffers.depth.setTest(!0),H.setScissorTest(!1);let G=p!==ni&&this.type===ni,q=p===ni&&this.type!==ni;for(let W=0,Y=C.length;W<Y;W++){let B=C[W],D=B.shadow;if(D===void 0){console.warn("THREE.WebGLShadowMap:",B,"has no shadow.");continue}if(D.autoUpdate===!1&&D.needsUpdate===!1)continue;i.copy(D.mapSize);let Q=D.getFrameExtents();if(i.multiply(Q),r.copy(D.mapSize),(i.x>h||i.y>h)&&(i.x>h&&(r.x=Math.floor(h/Q.x),i.x=r.x*Q.x,D.mapSize.x=r.x),i.y>h&&(r.y=Math.floor(h/Q.y),i.y=r.y*Q.y,D.mapSize.y=r.y)),D.map===null||G===!0||q===!0){let ce=this.type!==ni?{minFilter:Ct,magFilter:Ct}:{};D.map!==null&&D.map.dispose(),D.map=new Zn(i.x,i.y,ce),D.map.texture.name=B.name+".shadowMap",D.camera.updateProjectionMatrix()}s.setRenderTarget(D.map),s.clear();let ae=D.getViewportCount();for(let ce=0;ce<ae;ce++){let ve=D.getViewport(ce);o.set(r.x*ve.x,r.y*ve.y,r.x*ve.z,r.y*ve.w),H.viewport(o),D.updateMatrices(B,ce),n=D.getFrustum(),y(R,N,D.camera,B,this.type)}D.isPointLightShadow!==!0&&this.type===ni&&S(D,N),D.needsUpdate=!1}p=this.type,m.needsUpdate=!1,s.setRenderTarget(M,b,P)};function S(C,R){let N=e.update(_);d.defines.VSM_SAMPLES!==C.blurSamples&&(d.defines.VSM_SAMPLES=C.blurSamples,f.defines.VSM_SAMPLES=C.blurSamples,d.needsUpdate=!0,f.needsUpdate=!0),C.mapPass===null&&(C.mapPass=new Zn(i.x,i.y)),d.uniforms.shadow_pass.value=C.map.texture,d.uniforms.resolution.value=C.mapSize,d.uniforms.radius.value=C.radius,s.setRenderTarget(C.mapPass),s.clear(),s.renderBufferDirect(R,null,N,d,_,null),f.uniforms.shadow_pass.value=C.mapPass.texture,f.uniforms.resolution.value=C.mapSize,f.uniforms.radius.value=C.radius,s.setRenderTarget(C.map),s.clear(),s.renderBufferDirect(R,null,N,f,_,null)}function E(C,R,N,M){let b=null,P=N.isPointLight===!0?C.customDistanceMaterial:C.customDepthMaterial;if(P!==void 0)b=P;else if(b=N.isPointLight===!0?c:a,s.localClippingEnabled&&R.clipShadows===!0&&Array.isArray(R.clippingPlanes)&&R.clippingPlanes.length!==0||R.displacementMap&&R.displacementScale!==0||R.alphaMap&&R.alphaTest>0||R.map&&R.alphaTest>0||R.alphaToCoverage===!0){let H=b.uuid,G=R.uuid,q=l[H];q===void 0&&(q={},l[H]=q);let W=q[G];W===void 0&&(W=b.clone(),q[G]=W,R.addEventListener("dispose",w)),b=W}if(b.visible=R.visible,b.wireframe=R.wireframe,M===ni?b.side=R.shadowSide!==null?R.shadowSide:R.side:b.side=R.shadowSide!==null?R.shadowSide:u[R.side],b.alphaMap=R.alphaMap,b.alphaTest=R.alphaToCoverage===!0?.5:R.alphaTest,b.map=R.map,b.clipShadows=R.clipShadows,b.clippingPlanes=R.clippingPlanes,b.clipIntersection=R.clipIntersection,b.displacementMap=R.displacementMap,b.displacementScale=R.displacementScale,b.displacementBias=R.displacementBias,b.wireframeLinewidth=R.wireframeLinewidth,b.linewidth=R.linewidth,N.isPointLight===!0&&b.isMeshDistanceMaterial===!0){let H=s.properties.get(b);H.light=N}return b}function y(C,R,N,M,b){if(C.visible===!1)return;if(C.layers.test(R.layers)&&(C.isMesh||C.isLine||C.isPoints)&&(C.castShadow||C.receiveShadow&&b===ni)&&(!C.frustumCulled||n.intersectsObject(C))){C.modelViewMatrix.multiplyMatrices(N.matrixWorldInverse,C.matrixWorld);let G=e.update(C),q=C.material;if(Array.isArray(q)){let W=G.groups;for(let Y=0,B=W.length;Y<B;Y++){let D=W[Y],Q=q[D.materialIndex];if(Q&&Q.visible){let ae=E(C,Q,M,b);C.onBeforeShadow(s,C,R,N,G,ae,D),s.renderBufferDirect(N,null,G,ae,C,D),C.onAfterShadow(s,C,R,N,G,ae,D)}}}else if(q.visible){let W=E(C,q,M,b);C.onBeforeShadow(s,C,R,N,G,W,null),s.renderBufferDirect(N,null,G,W,C,null),C.onAfterShadow(s,C,R,N,G,W,null)}}let H=C.children;for(let G=0,q=H.length;G<q;G++)y(H[G],R,N,M,b)}function w(C){C.target.removeEventListener("dispose",w);for(let N in l){let M=l[N],b=C.target.uuid;b in M&&(M[b].dispose(),delete M[b])}}}var D0={[pa]:ma,[ga]:ya,[_a]:va,[is]:xa,[ma]:pa,[ya]:ga,[va]:_a,[xa]:is};function U0(s,e){function t(){let L=!1,re=new nt,le=null,ge=new nt(0,0,0,0);return{setMask:function(ne){le!==ne&&!L&&(s.colorMask(ne,ne,ne,ne),le=ne)},setLocked:function(ne){L=ne},setClear:function(ne,Z,ye,ze,dt){dt===!0&&(ne*=ze,Z*=ze,ye*=ze),re.set(ne,Z,ye,ze),ge.equals(re)===!1&&(s.clearColor(ne,Z,ye,ze),ge.copy(re))},reset:function(){L=!1,le=null,ge.set(-1,0,0,0)}}}function n(){let L=!1,re=!1,le=null,ge=null,ne=null;return{setReversed:function(Z){if(re!==Z){let ye=e.get("EXT_clip_control");Z?ye.clipControlEXT(ye.LOWER_LEFT_EXT,ye.ZERO_TO_ONE_EXT):ye.clipControlEXT(ye.LOWER_LEFT_EXT,ye.NEGATIVE_ONE_TO_ONE_EXT),re=Z;let ze=ne;ne=null,this.setClear(ze)}},getReversed:function(){return re},setTest:function(Z){Z?te(s.DEPTH_TEST):_e(s.DEPTH_TEST)},setMask:function(Z){le!==Z&&!L&&(s.depthMask(Z),le=Z)},setFunc:function(Z){if(re&&(Z=D0[Z]),ge!==Z){switch(Z){case pa:s.depthFunc(s.NEVER);break;case ma:s.depthFunc(s.ALWAYS);break;case ga:s.depthFunc(s.LESS);break;case is:s.depthFunc(s.LEQUAL);break;case _a:s.depthFunc(s.EQUAL);break;case xa:s.depthFunc(s.GEQUAL);break;case ya:s.depthFunc(s.GREATER);break;case va:s.depthFunc(s.NOTEQUAL);break;default:s.depthFunc(s.LEQUAL)}ge=Z}},setLocked:function(Z){L=Z},setClear:function(Z){ne!==Z&&(re&&(Z=1-Z),s.clearDepth(Z),ne=Z)},reset:function(){L=!1,le=null,ge=null,ne=null,re=!1}}}function i(){let L=!1,re=null,le=null,ge=null,ne=null,Z=null,ye=null,ze=null,dt=null;return{setTest:function(st){L||(st?te(s.STENCIL_TEST):_e(s.STENCIL_TEST))},setMask:function(st){re!==st&&!L&&(s.stencilMask(st),re=st)},setFunc:function(st,oi,qn){(le!==st||ge!==oi||ne!==qn)&&(s.stencilFunc(st,oi,qn),le=st,ge=oi,ne=qn)},setOp:function(st,oi,qn){(Z!==st||ye!==oi||ze!==qn)&&(s.stencilOp(st,oi,qn),Z=st,ye=oi,ze=qn)},setLocked:function(st){L=st},setClear:function(st){dt!==st&&(s.clearStencil(st),dt=st)},reset:function(){L=!1,re=null,le=null,ge=null,ne=null,Z=null,ye=null,ze=null,dt=null}}}let r=new t,o=new n,a=new i,c=new WeakMap,l=new WeakMap,h={},u={},d=new WeakMap,f=[],g=null,_=!1,m=null,p=null,S=null,E=null,y=null,w=null,C=null,R=new Re(0,0,0),N=0,M=!1,b=null,P=null,H=null,G=null,q=null,W=s.getParameter(s.MAX_COMBINED_TEXTURE_IMAGE_UNITS),Y=!1,B=0,D=s.getParameter(s.VERSION);D.indexOf("WebGL")!==-1?(B=parseFloat(/^WebGL (\d)/.exec(D)[1]),Y=B>=1):D.indexOf("OpenGL ES")!==-1&&(B=parseFloat(/^OpenGL ES (\d)/.exec(D)[1]),Y=B>=2);let Q=null,ae={},ce=s.getParameter(s.SCISSOR_BOX),ve=s.getParameter(s.VIEWPORT),Ze=new nt().fromArray(ce),it=new nt().fromArray(ve);function Ye(L,re,le,ge){let ne=new Uint8Array(4),Z=s.createTexture();s.bindTexture(L,Z),s.texParameteri(L,s.TEXTURE_MIN_FILTER,s.NEAREST),s.texParameteri(L,s.TEXTURE_MAG_FILTER,s.NEAREST);for(let ye=0;ye<le;ye++)L===s.TEXTURE_3D||L===s.TEXTURE_2D_ARRAY?s.texImage3D(re,0,s.RGBA,1,1,ge,0,s.RGBA,s.UNSIGNED_BYTE,ne):s.texImage2D(re+ye,0,s.RGBA,1,1,0,s.RGBA,s.UNSIGNED_BYTE,ne);return Z}let K={};K[s.TEXTURE_2D]=Ye(s.TEXTURE_2D,s.TEXTURE_2D,1),K[s.TEXTURE_CUBE_MAP]=Ye(s.TEXTURE_CUBE_MAP,s.TEXTURE_CUBE_MAP_POSITIVE_X,6),K[s.TEXTURE_2D_ARRAY]=Ye(s.TEXTURE_2D_ARRAY,s.TEXTURE_2D_ARRAY,1,1),K[s.TEXTURE_3D]=Ye(s.TEXTURE_3D,s.TEXTURE_3D,1,1),r.setClear(0,0,0,1),o.setClear(1),a.setClear(0),te(s.DEPTH_TEST),o.setFunc(is),Ue(!1),be(hl),te(s.CULL_FACE),mt(Mi);function te(L){h[L]!==!0&&(s.enable(L),h[L]=!0)}function _e(L){h[L]!==!1&&(s.disable(L),h[L]=!1)}function ke(L,re){return u[L]!==re?(s.bindFramebuffer(L,re),u[L]=re,L===s.DRAW_FRAMEBUFFER&&(u[s.FRAMEBUFFER]=re),L===s.FRAMEBUFFER&&(u[s.DRAW_FRAMEBUFFER]=re),!0):!1}function Ce(L,re){let le=f,ge=!1;if(L){le=d.get(re),le===void 0&&(le=[],d.set(re,le));let ne=L.textures;if(le.length!==ne.length||le[0]!==s.COLOR_ATTACHMENT0){for(let Z=0,ye=ne.length;Z<ye;Z++)le[Z]=s.COLOR_ATTACHMENT0+Z;le.length=ne.length,ge=!0}}else le[0]!==s.BACK&&(le[0]=s.BACK,ge=!0);ge&&s.drawBuffers(le)}function Qe(L){return g!==L?(s.useProgram(L),g=L,!0):!1}let Wt={[Di]:s.FUNC_ADD,[sd]:s.FUNC_SUBTRACT,[rd]:s.FUNC_REVERSE_SUBTRACT};Wt[od]=s.MIN,Wt[ad]=s.MAX;let I={[cd]:s.ZERO,[ld]:s.ONE,[hd]:s.SRC_COLOR,[Ko]:s.SRC_ALPHA,[gd]:s.SRC_ALPHA_SATURATE,[pd]:s.DST_COLOR,[dd]:s.DST_ALPHA,[ud]:s.ONE_MINUS_SRC_COLOR,[Zo]:s.ONE_MINUS_SRC_ALPHA,[md]:s.ONE_MINUS_DST_COLOR,[fd]:s.ONE_MINUS_DST_ALPHA,[_d]:s.CONSTANT_COLOR,[xd]:s.ONE_MINUS_CONSTANT_COLOR,[yd]:s.CONSTANT_ALPHA,[vd]:s.ONE_MINUS_CONSTANT_ALPHA};function mt(L,re,le,ge,ne,Z,ye,ze,dt,st){if(L===Mi){_===!0&&(_e(s.BLEND),_=!1);return}if(_===!1&&(te(s.BLEND),_=!0),L!==id){if(L!==m||st!==M){if((p!==Di||y!==Di)&&(s.blendEquation(s.FUNC_ADD),p=Di,y=Di),st)switch(L){case ns:s.blendFuncSeparate(s.ONE,s.ONE_MINUS_SRC_ALPHA,s.ONE,s.ONE_MINUS_SRC_ALPHA);break;case dl:s.blendFunc(s.ONE,s.ONE);break;case fl:s.blendFuncSeparate(s.ZERO,s.ONE_MINUS_SRC_COLOR,s.ZERO,s.ONE);break;case pl:s.blendFuncSeparate(s.DST_COLOR,s.ONE_MINUS_SRC_ALPHA,s.ZERO,s.ONE);break;default:console.error("THREE.WebGLState: Invalid blending: ",L);break}else switch(L){case ns:s.blendFuncSeparate(s.SRC_ALPHA,s.ONE_MINUS_SRC_ALPHA,s.ONE,s.ONE_MINUS_SRC_ALPHA);break;case dl:s.blendFuncSeparate(s.SRC_ALPHA,s.ONE,s.ONE,s.ONE);break;case fl:console.error("THREE.WebGLState: SubtractiveBlending requires material.premultipliedAlpha = true");break;case pl:console.error("THREE.WebGLState: MultiplyBlending requires material.premultipliedAlpha = true");break;default:console.error("THREE.WebGLState: Invalid blending: ",L);break}S=null,E=null,w=null,C=null,R.set(0,0,0),N=0,m=L,M=st}return}ne=ne||re,Z=Z||le,ye=ye||ge,(re!==p||ne!==y)&&(s.blendEquationSeparate(Wt[re],Wt[ne]),p=re,y=ne),(le!==S||ge!==E||Z!==w||ye!==C)&&(s.blendFuncSeparate(I[le],I[ge],I[Z],I[ye]),S=le,E=ge,w=Z,C=ye),(ze.equals(R)===!1||dt!==N)&&(s.blendColor(ze.r,ze.g,ze.b,dt),R.copy(ze),N=dt),m=L,M=!1}function He(L,re){L.side===Kt?_e(s.CULL_FACE):te(s.CULL_FACE);let le=L.side===en;re&&(le=!le),Ue(le),L.blending===ns&&L.transparent===!1?mt(Mi):mt(L.blending,L.blendEquation,L.blendSrc,L.blendDst,L.blendEquationAlpha,L.blendSrcAlpha,L.blendDstAlpha,L.blendColor,L.blendAlpha,L.premultipliedAlpha),o.setFunc(L.depthFunc),o.setTest(L.depthTest),o.setMask(L.depthWrite),r.setMask(L.colorWrite);let ge=L.stencilWrite;a.setTest(ge),ge&&(a.setMask(L.stencilWriteMask),a.setFunc(L.stencilFunc,L.stencilRef,L.stencilFuncMask),a.setOp(L.stencilFail,L.stencilZFail,L.stencilZPass)),Me(L.polygonOffset,L.polygonOffsetFactor,L.polygonOffsetUnits),L.alphaToCoverage===!0?te(s.SAMPLE_ALPHA_TO_COVERAGE):_e(s.SAMPLE_ALPHA_TO_COVERAGE)}function Ue(L){b!==L&&(L?s.frontFace(s.CW):s.frontFace(s.CCW),b=L)}function be(L){L!==td?(te(s.CULL_FACE),L!==P&&(L===hl?s.cullFace(s.BACK):L===nd?s.cullFace(s.FRONT):s.cullFace(s.FRONT_AND_BACK))):_e(s.CULL_FACE),P=L}function gt(L){L!==H&&(Y&&s.lineWidth(L),H=L)}function Me(L,re,le){L?(te(s.POLYGON_OFFSET_FILL),(G!==re||q!==le)&&(s.polygonOffset(re,le),G=re,q=le)):_e(s.POLYGON_OFFSET_FILL)}function We(L){L?te(s.SCISSOR_TEST):_e(s.SCISSOR_TEST)}function kt(L){L===void 0&&(L=s.TEXTURE0+W-1),Q!==L&&(s.activeTexture(L),Q=L)}function St(L,re,le){le===void 0&&(Q===null?le=s.TEXTURE0+W-1:le=Q);let ge=ae[le];ge===void 0&&(ge={type:void 0,texture:void 0},ae[le]=ge),(ge.type!==L||ge.texture!==re)&&(Q!==le&&(s.activeTexture(le),Q=le),s.bindTexture(L,re||K[L]),ge.type=L,ge.texture=re)}function T(){let L=ae[Q];L!==void 0&&L.type!==void 0&&(s.bindTexture(L.type,null),L.type=void 0,L.texture=void 0)}function x(){try{s.compressedTexImage2D(...arguments)}catch(L){console.error("THREE.WebGLState:",L)}}function k(){try{s.compressedTexImage3D(...arguments)}catch(L){console.error("THREE.WebGLState:",L)}}function j(){try{s.texSubImage2D(...arguments)}catch(L){console.error("THREE.WebGLState:",L)}}function ee(){try{s.texSubImage3D(...arguments)}catch(L){console.error("THREE.WebGLState:",L)}}function X(){try{s.compressedTexSubImage2D(...arguments)}catch(L){console.error("THREE.WebGLState:",L)}}function Ae(){try{s.compressedTexSubImage3D(...arguments)}catch(L){console.error("THREE.WebGLState:",L)}}function oe(){try{s.texStorage2D(...arguments)}catch(L){console.error("THREE.WebGLState:",L)}}function Se(){try{s.texStorage3D(...arguments)}catch(L){console.error("THREE.WebGLState:",L)}}function Te(){try{s.texImage2D(...arguments)}catch(L){console.error("THREE.WebGLState:",L)}}function se(){try{s.texImage3D(...arguments)}catch(L){console.error("THREE.WebGLState:",L)}}function pe(L){Ze.equals(L)===!1&&(s.scissor(L.x,L.y,L.z,L.w),Ze.copy(L))}function Ne(L){it.equals(L)===!1&&(s.viewport(L.x,L.y,L.z,L.w),it.copy(L))}function we(L,re){let le=l.get(re);le===void 0&&(le=new WeakMap,l.set(re,le));let ge=le.get(L);ge===void 0&&(ge=s.getUniformBlockIndex(re,L.name),le.set(L,ge))}function de(L,re){let ge=l.get(re).get(L);c.get(re)!==ge&&(s.uniformBlockBinding(re,ge,L.__bindingPointIndex),c.set(re,ge))}function Ge(){s.disable(s.BLEND),s.disable(s.CULL_FACE),s.disable(s.DEPTH_TEST),s.disable(s.POLYGON_OFFSET_FILL),s.disable(s.SCISSOR_TEST),s.disable(s.STENCIL_TEST),s.disable(s.SAMPLE_ALPHA_TO_COVERAGE),s.blendEquation(s.FUNC_ADD),s.blendFunc(s.ONE,s.ZERO),s.blendFuncSeparate(s.ONE,s.ZERO,s.ONE,s.ZERO),s.blendColor(0,0,0,0),s.colorMask(!0,!0,!0,!0),s.clearColor(0,0,0,0),s.depthMask(!0),s.depthFunc(s.LESS),o.setReversed(!1),s.clearDepth(1),s.stencilMask(4294967295),s.stencilFunc(s.ALWAYS,0,4294967295),s.stencilOp(s.KEEP,s.KEEP,s.KEEP),s.clearStencil(0),s.cullFace(s.BACK),s.frontFace(s.CCW),s.polygonOffset(0,0),s.activeTexture(s.TEXTURE0),s.bindFramebuffer(s.FRAMEBUFFER,null),s.bindFramebuffer(s.DRAW_FRAMEBUFFER,null),s.bindFramebuffer(s.READ_FRAMEBUFFER,null),s.useProgram(null),s.lineWidth(1),s.scissor(0,0,s.canvas.width,s.canvas.height),s.viewport(0,0,s.canvas.width,s.canvas.height),h={},Q=null,ae={},u={},d=new WeakMap,f=[],g=null,_=!1,m=null,p=null,S=null,E=null,y=null,w=null,C=null,R=new Re(0,0,0),N=0,M=!1,b=null,P=null,H=null,G=null,q=null,Ze.set(0,0,s.canvas.width,s.canvas.height),it.set(0,0,s.canvas.width,s.canvas.height),r.reset(),o.reset(),a.reset()}return{buffers:{color:r,depth:o,stencil:a},enable:te,disable:_e,bindFramebuffer:ke,drawBuffers:Ce,useProgram:Qe,setBlending:mt,setMaterial:He,setFlipSided:Ue,setCullFace:be,setLineWidth:gt,setPolygonOffset:Me,setScissorTest:We,activeTexture:kt,bindTexture:St,unbindTexture:T,compressedTexImage2D:x,compressedTexImage3D:k,texImage2D:Te,texImage3D:se,updateUBOMapping:we,uniformBlockBinding:de,texStorage2D:oe,texStorage3D:Se,texSubImage2D:j,texSubImage3D:ee,compressedTexSubImage2D:X,compressedTexSubImage3D:Ae,scissor:pe,viewport:Ne,reset:Ge}}function O0(s,e,t,n,i,r,o){let a=e.has("WEBGL_multisampled_render_to_texture")?e.get("WEBGL_multisampled_render_to_texture"):null,c=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),l=new Ee,h=new WeakMap,u,d=new WeakMap,f=!1;try{f=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function g(T,x){return f?new OffscreenCanvas(T,x):Xs("canvas")}function _(T,x,k){let j=1,ee=St(T);if((ee.width>k||ee.height>k)&&(j=k/Math.max(ee.width,ee.height)),j<1)if(typeof HTMLImageElement<"u"&&T instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&T instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&T instanceof ImageBitmap||typeof VideoFrame<"u"&&T instanceof VideoFrame){let X=Math.floor(j*ee.width),Ae=Math.floor(j*ee.height);u===void 0&&(u=g(X,Ae));let oe=x?g(X,Ae):u;return oe.width=X,oe.height=Ae,oe.getContext("2d").drawImage(T,0,0,X,Ae),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+ee.width+"x"+ee.height+") to ("+X+"x"+Ae+")."),oe}else return"data"in T&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+ee.width+"x"+ee.height+")."),T;return T}function m(T){return T.generateMipmaps}function p(T){s.generateMipmap(T)}function S(T){return T.isWebGLCubeRenderTarget?s.TEXTURE_CUBE_MAP:T.isWebGL3DRenderTarget?s.TEXTURE_3D:T.isWebGLArrayRenderTarget||T.isCompressedArrayTexture?s.TEXTURE_2D_ARRAY:s.TEXTURE_2D}function E(T,x,k,j,ee=!1){if(T!==null){if(s[T]!==void 0)return s[T];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+T+"'")}let X=x;if(x===s.RED&&(k===s.FLOAT&&(X=s.R32F),k===s.HALF_FLOAT&&(X=s.R16F),k===s.UNSIGNED_BYTE&&(X=s.R8)),x===s.RED_INTEGER&&(k===s.UNSIGNED_BYTE&&(X=s.R8UI),k===s.UNSIGNED_SHORT&&(X=s.R16UI),k===s.UNSIGNED_INT&&(X=s.R32UI),k===s.BYTE&&(X=s.R8I),k===s.SHORT&&(X=s.R16I),k===s.INT&&(X=s.R32I)),x===s.RG&&(k===s.FLOAT&&(X=s.RG32F),k===s.HALF_FLOAT&&(X=s.RG16F),k===s.UNSIGNED_BYTE&&(X=s.RG8)),x===s.RG_INTEGER&&(k===s.UNSIGNED_BYTE&&(X=s.RG8UI),k===s.UNSIGNED_SHORT&&(X=s.RG16UI),k===s.UNSIGNED_INT&&(X=s.RG32UI),k===s.BYTE&&(X=s.RG8I),k===s.SHORT&&(X=s.RG16I),k===s.INT&&(X=s.RG32I)),x===s.RGB_INTEGER&&(k===s.UNSIGNED_BYTE&&(X=s.RGB8UI),k===s.UNSIGNED_SHORT&&(X=s.RGB16UI),k===s.UNSIGNED_INT&&(X=s.RGB32UI),k===s.BYTE&&(X=s.RGB8I),k===s.SHORT&&(X=s.RGB16I),k===s.INT&&(X=s.RGB32I)),x===s.RGBA_INTEGER&&(k===s.UNSIGNED_BYTE&&(X=s.RGBA8UI),k===s.UNSIGNED_SHORT&&(X=s.RGBA16UI),k===s.UNSIGNED_INT&&(X=s.RGBA32UI),k===s.BYTE&&(X=s.RGBA8I),k===s.SHORT&&(X=s.RGBA16I),k===s.INT&&(X=s.RGBA32I)),x===s.RGB&&(k===s.UNSIGNED_INT_5_9_9_9_REV&&(X=s.RGB9_E5),k===s.UNSIGNED_INT_10F_11F_11F_REV&&(X=s.R11F_G11F_B10F)),x===s.RGBA){let Ae=ee?Pr:$e.getTransfer(j);k===s.FLOAT&&(X=s.RGBA32F),k===s.HALF_FLOAT&&(X=s.RGBA16F),k===s.UNSIGNED_BYTE&&(X=Ae===ot?s.SRGB8_ALPHA8:s.RGBA8),k===s.UNSIGNED_SHORT_4_4_4_4&&(X=s.RGBA4),k===s.UNSIGNED_SHORT_5_5_5_1&&(X=s.RGB5_A1)}return(X===s.R16F||X===s.R32F||X===s.RG16F||X===s.RG32F||X===s.RGBA16F||X===s.RGBA32F)&&e.get("EXT_color_buffer_float"),X}function y(T,x){let k;return T?x===null||x===Xi||x===or?k=s.DEPTH24_STENCIL8:x===wn?k=s.DEPTH32F_STENCIL8:x===sr&&(k=s.DEPTH24_STENCIL8,console.warn("DepthTexture: 16 bit depth attachment is not supported with stencil. Using 24-bit attachment.")):x===null||x===Xi||x===or?k=s.DEPTH_COMPONENT24:x===wn?k=s.DEPTH_COMPONENT32F:x===sr&&(k=s.DEPTH_COMPONENT16),k}function w(T,x){return m(T)===!0||T.isFramebufferTexture&&T.minFilter!==Ct&&T.minFilter!==Ht?Math.log2(Math.max(x.width,x.height))+1:T.mipmaps!==void 0&&T.mipmaps.length>0?T.mipmaps.length:T.isCompressedTexture&&Array.isArray(T.image)?x.mipmaps.length:1}function C(T){let x=T.target;x.removeEventListener("dispose",C),N(x),x.isVideoTexture&&h.delete(x)}function R(T){let x=T.target;x.removeEventListener("dispose",R),b(x)}function N(T){let x=n.get(T);if(x.__webglInit===void 0)return;let k=T.source,j=d.get(k);if(j){let ee=j[x.__cacheKey];ee.usedTimes--,ee.usedTimes===0&&M(T),Object.keys(j).length===0&&d.delete(k)}n.remove(T)}function M(T){let x=n.get(T);s.deleteTexture(x.__webglTexture);let k=T.source,j=d.get(k);delete j[x.__cacheKey],o.memory.textures--}function b(T){let x=n.get(T);if(T.depthTexture&&(T.depthTexture.dispose(),n.remove(T.depthTexture)),T.isWebGLCubeRenderTarget)for(let j=0;j<6;j++){if(Array.isArray(x.__webglFramebuffer[j]))for(let ee=0;ee<x.__webglFramebuffer[j].length;ee++)s.deleteFramebuffer(x.__webglFramebuffer[j][ee]);else s.deleteFramebuffer(x.__webglFramebuffer[j]);x.__webglDepthbuffer&&s.deleteRenderbuffer(x.__webglDepthbuffer[j])}else{if(Array.isArray(x.__webglFramebuffer))for(let j=0;j<x.__webglFramebuffer.length;j++)s.deleteFramebuffer(x.__webglFramebuffer[j]);else s.deleteFramebuffer(x.__webglFramebuffer);if(x.__webglDepthbuffer&&s.deleteRenderbuffer(x.__webglDepthbuffer),x.__webglMultisampledFramebuffer&&s.deleteFramebuffer(x.__webglMultisampledFramebuffer),x.__webglColorRenderbuffer)for(let j=0;j<x.__webglColorRenderbuffer.length;j++)x.__webglColorRenderbuffer[j]&&s.deleteRenderbuffer(x.__webglColorRenderbuffer[j]);x.__webglDepthRenderbuffer&&s.deleteRenderbuffer(x.__webglDepthRenderbuffer)}let k=T.textures;for(let j=0,ee=k.length;j<ee;j++){let X=n.get(k[j]);X.__webglTexture&&(s.deleteTexture(X.__webglTexture),o.memory.textures--),n.remove(k[j])}n.remove(T)}let P=0;function H(){P=0}function G(){let T=P;return T>=i.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+T+" texture units while this GPU supports only "+i.maxTextures),P+=1,T}function q(T){let x=[];return x.push(T.wrapS),x.push(T.wrapT),x.push(T.wrapR||0),x.push(T.magFilter),x.push(T.minFilter),x.push(T.anisotropy),x.push(T.internalFormat),x.push(T.format),x.push(T.type),x.push(T.generateMipmaps),x.push(T.premultiplyAlpha),x.push(T.flipY),x.push(T.unpackAlignment),x.push(T.colorSpace),x.join()}function W(T,x){let k=n.get(T);if(T.isVideoTexture&&We(T),T.isRenderTargetTexture===!1&&T.isExternalTexture!==!0&&T.version>0&&k.__version!==T.version){let j=T.image;if(j===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(j.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{K(k,T,x);return}}else T.isExternalTexture&&(k.__webglTexture=T.sourceTexture?T.sourceTexture:null);t.bindTexture(s.TEXTURE_2D,k.__webglTexture,s.TEXTURE0+x)}function Y(T,x){let k=n.get(T);if(T.isRenderTargetTexture===!1&&T.version>0&&k.__version!==T.version){K(k,T,x);return}t.bindTexture(s.TEXTURE_2D_ARRAY,k.__webglTexture,s.TEXTURE0+x)}function B(T,x){let k=n.get(T);if(T.isRenderTargetTexture===!1&&T.version>0&&k.__version!==T.version){K(k,T,x);return}t.bindTexture(s.TEXTURE_3D,k.__webglTexture,s.TEXTURE0+x)}function D(T,x){let k=n.get(T);if(T.version>0&&k.__version!==T.version){te(k,T,x);return}t.bindTexture(s.TEXTURE_CUBE_MAP,k.__webglTexture,s.TEXTURE0+x)}let Q={[jn]:s.REPEAT,[En]:s.CLAMP_TO_EDGE,[Ui]:s.MIRRORED_REPEAT},ae={[Ct]:s.NEAREST,[ir]:s.NEAREST_MIPMAP_NEAREST,[Ei]:s.NEAREST_MIPMAP_LINEAR,[Ht]:s.LINEAR,[Wi]:s.LINEAR_MIPMAP_NEAREST,[gn]:s.LINEAR_MIPMAP_LINEAR},ce={[Dd]:s.NEVER,[zd]:s.ALWAYS,[Ud]:s.LESS,[wl]:s.LEQUAL,[Od]:s.EQUAL,[kd]:s.GEQUAL,[Fd]:s.GREATER,[Bd]:s.NOTEQUAL};function ve(T,x){if(x.type===wn&&e.has("OES_texture_float_linear")===!1&&(x.magFilter===Ht||x.magFilter===Wi||x.magFilter===Ei||x.magFilter===gn||x.minFilter===Ht||x.minFilter===Wi||x.minFilter===Ei||x.minFilter===gn)&&console.warn("THREE.WebGLRenderer: Unable to use linear filtering with floating point textures. OES_texture_float_linear not supported on this device."),s.texParameteri(T,s.TEXTURE_WRAP_S,Q[x.wrapS]),s.texParameteri(T,s.TEXTURE_WRAP_T,Q[x.wrapT]),(T===s.TEXTURE_3D||T===s.TEXTURE_2D_ARRAY)&&s.texParameteri(T,s.TEXTURE_WRAP_R,Q[x.wrapR]),s.texParameteri(T,s.TEXTURE_MAG_FILTER,ae[x.magFilter]),s.texParameteri(T,s.TEXTURE_MIN_FILTER,ae[x.minFilter]),x.compareFunction&&(s.texParameteri(T,s.TEXTURE_COMPARE_MODE,s.COMPARE_REF_TO_TEXTURE),s.texParameteri(T,s.TEXTURE_COMPARE_FUNC,ce[x.compareFunction])),e.has("EXT_texture_filter_anisotropic")===!0){if(x.magFilter===Ct||x.minFilter!==Ei&&x.minFilter!==gn||x.type===wn&&e.has("OES_texture_float_linear")===!1)return;if(x.anisotropy>1||n.get(x).__currentAnisotropy){let k=e.get("EXT_texture_filter_anisotropic");s.texParameterf(T,k.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(x.anisotropy,i.getMaxAnisotropy())),n.get(x).__currentAnisotropy=x.anisotropy}}}function Ze(T,x){let k=!1;T.__webglInit===void 0&&(T.__webglInit=!0,x.addEventListener("dispose",C));let j=x.source,ee=d.get(j);ee===void 0&&(ee={},d.set(j,ee));let X=q(x);if(X!==T.__cacheKey){ee[X]===void 0&&(ee[X]={texture:s.createTexture(),usedTimes:0},o.memory.textures++,k=!0),ee[X].usedTimes++;let Ae=ee[T.__cacheKey];Ae!==void 0&&(ee[T.__cacheKey].usedTimes--,Ae.usedTimes===0&&M(x)),T.__cacheKey=X,T.__webglTexture=ee[X].texture}return k}function it(T,x,k){return Math.floor(Math.floor(T/k)/x)}function Ye(T,x,k,j){let X=T.updateRanges;if(X.length===0)t.texSubImage2D(s.TEXTURE_2D,0,0,0,x.width,x.height,k,j,x.data);else{X.sort((se,pe)=>se.start-pe.start);let Ae=0;for(let se=1;se<X.length;se++){let pe=X[Ae],Ne=X[se],we=pe.start+pe.count,de=it(Ne.start,x.width,4),Ge=it(pe.start,x.width,4);Ne.start<=we+1&&de===Ge&&it(Ne.start+Ne.count-1,x.width,4)===de?pe.count=Math.max(pe.count,Ne.start+Ne.count-pe.start):(++Ae,X[Ae]=Ne)}X.length=Ae+1;let oe=s.getParameter(s.UNPACK_ROW_LENGTH),Se=s.getParameter(s.UNPACK_SKIP_PIXELS),Te=s.getParameter(s.UNPACK_SKIP_ROWS);s.pixelStorei(s.UNPACK_ROW_LENGTH,x.width);for(let se=0,pe=X.length;se<pe;se++){let Ne=X[se],we=Math.floor(Ne.start/4),de=Math.ceil(Ne.count/4),Ge=we%x.width,L=Math.floor(we/x.width),re=de,le=1;s.pixelStorei(s.UNPACK_SKIP_PIXELS,Ge),s.pixelStorei(s.UNPACK_SKIP_ROWS,L),t.texSubImage2D(s.TEXTURE_2D,0,Ge,L,re,le,k,j,x.data)}T.clearUpdateRanges(),s.pixelStorei(s.UNPACK_ROW_LENGTH,oe),s.pixelStorei(s.UNPACK_SKIP_PIXELS,Se),s.pixelStorei(s.UNPACK_SKIP_ROWS,Te)}}function K(T,x,k){let j=s.TEXTURE_2D;(x.isDataArrayTexture||x.isCompressedArrayTexture)&&(j=s.TEXTURE_2D_ARRAY),x.isData3DTexture&&(j=s.TEXTURE_3D);let ee=Ze(T,x),X=x.source;t.bindTexture(j,T.__webglTexture,s.TEXTURE0+k);let Ae=n.get(X);if(X.version!==Ae.__version||ee===!0){t.activeTexture(s.TEXTURE0+k);let oe=$e.getPrimaries($e.workingColorSpace),Se=x.colorSpace===Hn?null:$e.getPrimaries(x.colorSpace),Te=x.colorSpace===Hn||oe===Se?s.NONE:s.BROWSER_DEFAULT_WEBGL;s.pixelStorei(s.UNPACK_FLIP_Y_WEBGL,x.flipY),s.pixelStorei(s.UNPACK_PREMULTIPLY_ALPHA_WEBGL,x.premultiplyAlpha),s.pixelStorei(s.UNPACK_ALIGNMENT,x.unpackAlignment),s.pixelStorei(s.UNPACK_COLORSPACE_CONVERSION_WEBGL,Te);let se=_(x.image,!1,i.maxTextureSize);se=kt(x,se);let pe=r.convert(x.format,x.colorSpace),Ne=r.convert(x.type),we=E(x.internalFormat,pe,Ne,x.colorSpace,x.isVideoTexture);ve(j,x);let de,Ge=x.mipmaps,L=x.isVideoTexture!==!0,re=Ae.__version===void 0||ee===!0,le=X.dataReady,ge=w(x,se);if(x.isDepthTexture)we=y(x.format===ar,x.type),re&&(L?t.texStorage2D(s.TEXTURE_2D,1,we,se.width,se.height):t.texImage2D(s.TEXTURE_2D,0,we,se.width,se.height,0,pe,Ne,null));else if(x.isDataTexture)if(Ge.length>0){L&&re&&t.texStorage2D(s.TEXTURE_2D,ge,we,Ge[0].width,Ge[0].height);for(let ne=0,Z=Ge.length;ne<Z;ne++)de=Ge[ne],L?le&&t.texSubImage2D(s.TEXTURE_2D,ne,0,0,de.width,de.height,pe,Ne,de.data):t.texImage2D(s.TEXTURE_2D,ne,we,de.width,de.height,0,pe,Ne,de.data);x.generateMipmaps=!1}else L?(re&&t.texStorage2D(s.TEXTURE_2D,ge,we,se.width,se.height),le&&Ye(x,se,pe,Ne)):t.texImage2D(s.TEXTURE_2D,0,we,se.width,se.height,0,pe,Ne,se.data);else if(x.isCompressedTexture)if(x.isCompressedArrayTexture){L&&re&&t.texStorage3D(s.TEXTURE_2D_ARRAY,ge,we,Ge[0].width,Ge[0].height,se.depth);for(let ne=0,Z=Ge.length;ne<Z;ne++)if(de=Ge[ne],x.format!==tn)if(pe!==null)if(L){if(le)if(x.layerUpdates.size>0){let ye=Nl(de.width,de.height,x.format,x.type);for(let ze of x.layerUpdates){let dt=de.data.subarray(ze*ye/de.data.BYTES_PER_ELEMENT,(ze+1)*ye/de.data.BYTES_PER_ELEMENT);t.compressedTexSubImage3D(s.TEXTURE_2D_ARRAY,ne,0,0,ze,de.width,de.height,1,pe,dt)}x.clearLayerUpdates()}else t.compressedTexSubImage3D(s.TEXTURE_2D_ARRAY,ne,0,0,0,de.width,de.height,se.depth,pe,de.data)}else t.compressedTexImage3D(s.TEXTURE_2D_ARRAY,ne,we,de.width,de.height,se.depth,0,de.data,0,0);else console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()");else L?le&&t.texSubImage3D(s.TEXTURE_2D_ARRAY,ne,0,0,0,de.width,de.height,se.depth,pe,Ne,de.data):t.texImage3D(s.TEXTURE_2D_ARRAY,ne,we,de.width,de.height,se.depth,0,pe,Ne,de.data)}else{L&&re&&t.texStorage2D(s.TEXTURE_2D,ge,we,Ge[0].width,Ge[0].height);for(let ne=0,Z=Ge.length;ne<Z;ne++)de=Ge[ne],x.format!==tn?pe!==null?L?le&&t.compressedTexSubImage2D(s.TEXTURE_2D,ne,0,0,de.width,de.height,pe,de.data):t.compressedTexImage2D(s.TEXTURE_2D,ne,we,de.width,de.height,0,de.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):L?le&&t.texSubImage2D(s.TEXTURE_2D,ne,0,0,de.width,de.height,pe,Ne,de.data):t.texImage2D(s.TEXTURE_2D,ne,we,de.width,de.height,0,pe,Ne,de.data)}else if(x.isDataArrayTexture)if(L){if(re&&t.texStorage3D(s.TEXTURE_2D_ARRAY,ge,we,se.width,se.height,se.depth),le)if(x.layerUpdates.size>0){let ne=Nl(se.width,se.height,x.format,x.type);for(let Z of x.layerUpdates){let ye=se.data.subarray(Z*ne/se.data.BYTES_PER_ELEMENT,(Z+1)*ne/se.data.BYTES_PER_ELEMENT);t.texSubImage3D(s.TEXTURE_2D_ARRAY,0,0,0,Z,se.width,se.height,1,pe,Ne,ye)}x.clearLayerUpdates()}else t.texSubImage3D(s.TEXTURE_2D_ARRAY,0,0,0,0,se.width,se.height,se.depth,pe,Ne,se.data)}else t.texImage3D(s.TEXTURE_2D_ARRAY,0,we,se.width,se.height,se.depth,0,pe,Ne,se.data);else if(x.isData3DTexture)L?(re&&t.texStorage3D(s.TEXTURE_3D,ge,we,se.width,se.height,se.depth),le&&t.texSubImage3D(s.TEXTURE_3D,0,0,0,0,se.width,se.height,se.depth,pe,Ne,se.data)):t.texImage3D(s.TEXTURE_3D,0,we,se.width,se.height,se.depth,0,pe,Ne,se.data);else if(x.isFramebufferTexture){if(re)if(L)t.texStorage2D(s.TEXTURE_2D,ge,we,se.width,se.height);else{let ne=se.width,Z=se.height;for(let ye=0;ye<ge;ye++)t.texImage2D(s.TEXTURE_2D,ye,we,ne,Z,0,pe,Ne,null),ne>>=1,Z>>=1}}else if(Ge.length>0){if(L&&re){let ne=St(Ge[0]);t.texStorage2D(s.TEXTURE_2D,ge,we,ne.width,ne.height)}for(let ne=0,Z=Ge.length;ne<Z;ne++)de=Ge[ne],L?le&&t.texSubImage2D(s.TEXTURE_2D,ne,0,0,pe,Ne,de):t.texImage2D(s.TEXTURE_2D,ne,we,pe,Ne,de);x.generateMipmaps=!1}else if(L){if(re){let ne=St(se);t.texStorage2D(s.TEXTURE_2D,ge,we,ne.width,ne.height)}le&&t.texSubImage2D(s.TEXTURE_2D,0,0,0,pe,Ne,se)}else t.texImage2D(s.TEXTURE_2D,0,we,pe,Ne,se);m(x)&&p(j),Ae.__version=X.version,x.onUpdate&&x.onUpdate(x)}T.__version=x.version}function te(T,x,k){if(x.image.length!==6)return;let j=Ze(T,x),ee=x.source;t.bindTexture(s.TEXTURE_CUBE_MAP,T.__webglTexture,s.TEXTURE0+k);let X=n.get(ee);if(ee.version!==X.__version||j===!0){t.activeTexture(s.TEXTURE0+k);let Ae=$e.getPrimaries($e.workingColorSpace),oe=x.colorSpace===Hn?null:$e.getPrimaries(x.colorSpace),Se=x.colorSpace===Hn||Ae===oe?s.NONE:s.BROWSER_DEFAULT_WEBGL;s.pixelStorei(s.UNPACK_FLIP_Y_WEBGL,x.flipY),s.pixelStorei(s.UNPACK_PREMULTIPLY_ALPHA_WEBGL,x.premultiplyAlpha),s.pixelStorei(s.UNPACK_ALIGNMENT,x.unpackAlignment),s.pixelStorei(s.UNPACK_COLORSPACE_CONVERSION_WEBGL,Se);let Te=x.isCompressedTexture||x.image[0].isCompressedTexture,se=x.image[0]&&x.image[0].isDataTexture,pe=[];for(let Z=0;Z<6;Z++)!Te&&!se?pe[Z]=_(x.image[Z],!0,i.maxCubemapSize):pe[Z]=se?x.image[Z].image:x.image[Z],pe[Z]=kt(x,pe[Z]);let Ne=pe[0],we=r.convert(x.format,x.colorSpace),de=r.convert(x.type),Ge=E(x.internalFormat,we,de,x.colorSpace),L=x.isVideoTexture!==!0,re=X.__version===void 0||j===!0,le=ee.dataReady,ge=w(x,Ne);ve(s.TEXTURE_CUBE_MAP,x);let ne;if(Te){L&&re&&t.texStorage2D(s.TEXTURE_CUBE_MAP,ge,Ge,Ne.width,Ne.height);for(let Z=0;Z<6;Z++){ne=pe[Z].mipmaps;for(let ye=0;ye<ne.length;ye++){let ze=ne[ye];x.format!==tn?we!==null?L?le&&t.compressedTexSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,ye,0,0,ze.width,ze.height,we,ze.data):t.compressedTexImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,ye,Ge,ze.width,ze.height,0,ze.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):L?le&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,ye,0,0,ze.width,ze.height,we,de,ze.data):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,ye,Ge,ze.width,ze.height,0,we,de,ze.data)}}}else{if(ne=x.mipmaps,L&&re){ne.length>0&&ge++;let Z=St(pe[0]);t.texStorage2D(s.TEXTURE_CUBE_MAP,ge,Ge,Z.width,Z.height)}for(let Z=0;Z<6;Z++)if(se){L?le&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,0,0,0,pe[Z].width,pe[Z].height,we,de,pe[Z].data):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,0,Ge,pe[Z].width,pe[Z].height,0,we,de,pe[Z].data);for(let ye=0;ye<ne.length;ye++){let dt=ne[ye].image[Z].image;L?le&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,ye+1,0,0,dt.width,dt.height,we,de,dt.data):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,ye+1,Ge,dt.width,dt.height,0,we,de,dt.data)}}else{L?le&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,0,0,0,we,de,pe[Z]):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,0,Ge,we,de,pe[Z]);for(let ye=0;ye<ne.length;ye++){let ze=ne[ye];L?le&&t.texSubImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,ye+1,0,0,we,de,ze.image[Z]):t.texImage2D(s.TEXTURE_CUBE_MAP_POSITIVE_X+Z,ye+1,Ge,we,de,ze.image[Z])}}}m(x)&&p(s.TEXTURE_CUBE_MAP),X.__version=ee.version,x.onUpdate&&x.onUpdate(x)}T.__version=x.version}function _e(T,x,k,j,ee,X){let Ae=r.convert(k.format,k.colorSpace),oe=r.convert(k.type),Se=E(k.internalFormat,Ae,oe,k.colorSpace),Te=n.get(x),se=n.get(k);if(se.__renderTarget=x,!Te.__hasExternalTextures){let pe=Math.max(1,x.width>>X),Ne=Math.max(1,x.height>>X);ee===s.TEXTURE_3D||ee===s.TEXTURE_2D_ARRAY?t.texImage3D(ee,X,Se,pe,Ne,x.depth,0,Ae,oe,null):t.texImage2D(ee,X,Se,pe,Ne,0,Ae,oe,null)}t.bindFramebuffer(s.FRAMEBUFFER,T),Me(x)?a.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,j,ee,se.__webglTexture,0,gt(x)):(ee===s.TEXTURE_2D||ee>=s.TEXTURE_CUBE_MAP_POSITIVE_X&&ee<=s.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&s.framebufferTexture2D(s.FRAMEBUFFER,j,ee,se.__webglTexture,X),t.bindFramebuffer(s.FRAMEBUFFER,null)}function ke(T,x,k){if(s.bindRenderbuffer(s.RENDERBUFFER,T),x.depthBuffer){let j=x.depthTexture,ee=j&&j.isDepthTexture?j.type:null,X=y(x.stencilBuffer,ee),Ae=x.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,oe=gt(x);Me(x)?a.renderbufferStorageMultisampleEXT(s.RENDERBUFFER,oe,X,x.width,x.height):k?s.renderbufferStorageMultisample(s.RENDERBUFFER,oe,X,x.width,x.height):s.renderbufferStorage(s.RENDERBUFFER,X,x.width,x.height),s.framebufferRenderbuffer(s.FRAMEBUFFER,Ae,s.RENDERBUFFER,T)}else{let j=x.textures;for(let ee=0;ee<j.length;ee++){let X=j[ee],Ae=r.convert(X.format,X.colorSpace),oe=r.convert(X.type),Se=E(X.internalFormat,Ae,oe,X.colorSpace),Te=gt(x);k&&Me(x)===!1?s.renderbufferStorageMultisample(s.RENDERBUFFER,Te,Se,x.width,x.height):Me(x)?a.renderbufferStorageMultisampleEXT(s.RENDERBUFFER,Te,Se,x.width,x.height):s.renderbufferStorage(s.RENDERBUFFER,Se,x.width,x.height)}}s.bindRenderbuffer(s.RENDERBUFFER,null)}function Ce(T,x){if(x&&x.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(t.bindFramebuffer(s.FRAMEBUFFER,T),!(x.depthTexture&&x.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");let j=n.get(x.depthTexture);j.__renderTarget=x,(!j.__webglTexture||x.depthTexture.image.width!==x.width||x.depthTexture.image.height!==x.height)&&(x.depthTexture.image.width=x.width,x.depthTexture.image.height=x.height,x.depthTexture.needsUpdate=!0),W(x.depthTexture,0);let ee=j.__webglTexture,X=gt(x);if(x.depthTexture.format===Ws)Me(x)?a.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,s.DEPTH_ATTACHMENT,s.TEXTURE_2D,ee,0,X):s.framebufferTexture2D(s.FRAMEBUFFER,s.DEPTH_ATTACHMENT,s.TEXTURE_2D,ee,0);else if(x.depthTexture.format===ar)Me(x)?a.framebufferTexture2DMultisampleEXT(s.FRAMEBUFFER,s.DEPTH_STENCIL_ATTACHMENT,s.TEXTURE_2D,ee,0,X):s.framebufferTexture2D(s.FRAMEBUFFER,s.DEPTH_STENCIL_ATTACHMENT,s.TEXTURE_2D,ee,0);else throw new Error("Unknown depthTexture format")}function Qe(T){let x=n.get(T),k=T.isWebGLCubeRenderTarget===!0;if(x.__boundDepthTexture!==T.depthTexture){let j=T.depthTexture;if(x.__depthDisposeCallback&&x.__depthDisposeCallback(),j){let ee=()=>{delete x.__boundDepthTexture,delete x.__depthDisposeCallback,j.removeEventListener("dispose",ee)};j.addEventListener("dispose",ee),x.__depthDisposeCallback=ee}x.__boundDepthTexture=j}if(T.depthTexture&&!x.__autoAllocateDepthBuffer){if(k)throw new Error("target.depthTexture not supported in Cube render targets");let j=T.texture.mipmaps;j&&j.length>0?Ce(x.__webglFramebuffer[0],T):Ce(x.__webglFramebuffer,T)}else if(k){x.__webglDepthbuffer=[];for(let j=0;j<6;j++)if(t.bindFramebuffer(s.FRAMEBUFFER,x.__webglFramebuffer[j]),x.__webglDepthbuffer[j]===void 0)x.__webglDepthbuffer[j]=s.createRenderbuffer(),ke(x.__webglDepthbuffer[j],T,!1);else{let ee=T.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,X=x.__webglDepthbuffer[j];s.bindRenderbuffer(s.RENDERBUFFER,X),s.framebufferRenderbuffer(s.FRAMEBUFFER,ee,s.RENDERBUFFER,X)}}else{let j=T.texture.mipmaps;if(j&&j.length>0?t.bindFramebuffer(s.FRAMEBUFFER,x.__webglFramebuffer[0]):t.bindFramebuffer(s.FRAMEBUFFER,x.__webglFramebuffer),x.__webglDepthbuffer===void 0)x.__webglDepthbuffer=s.createRenderbuffer(),ke(x.__webglDepthbuffer,T,!1);else{let ee=T.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,X=x.__webglDepthbuffer;s.bindRenderbuffer(s.RENDERBUFFER,X),s.framebufferRenderbuffer(s.FRAMEBUFFER,ee,s.RENDERBUFFER,X)}}t.bindFramebuffer(s.FRAMEBUFFER,null)}function Wt(T,x,k){let j=n.get(T);x!==void 0&&_e(j.__webglFramebuffer,T,T.texture,s.COLOR_ATTACHMENT0,s.TEXTURE_2D,0),k!==void 0&&Qe(T)}function I(T){let x=T.texture,k=n.get(T),j=n.get(x);T.addEventListener("dispose",R);let ee=T.textures,X=T.isWebGLCubeRenderTarget===!0,Ae=ee.length>1;if(Ae||(j.__webglTexture===void 0&&(j.__webglTexture=s.createTexture()),j.__version=x.version,o.memory.textures++),X){k.__webglFramebuffer=[];for(let oe=0;oe<6;oe++)if(x.mipmaps&&x.mipmaps.length>0){k.__webglFramebuffer[oe]=[];for(let Se=0;Se<x.mipmaps.length;Se++)k.__webglFramebuffer[oe][Se]=s.createFramebuffer()}else k.__webglFramebuffer[oe]=s.createFramebuffer()}else{if(x.mipmaps&&x.mipmaps.length>0){k.__webglFramebuffer=[];for(let oe=0;oe<x.mipmaps.length;oe++)k.__webglFramebuffer[oe]=s.createFramebuffer()}else k.__webglFramebuffer=s.createFramebuffer();if(Ae)for(let oe=0,Se=ee.length;oe<Se;oe++){let Te=n.get(ee[oe]);Te.__webglTexture===void 0&&(Te.__webglTexture=s.createTexture(),o.memory.textures++)}if(T.samples>0&&Me(T)===!1){k.__webglMultisampledFramebuffer=s.createFramebuffer(),k.__webglColorRenderbuffer=[],t.bindFramebuffer(s.FRAMEBUFFER,k.__webglMultisampledFramebuffer);for(let oe=0;oe<ee.length;oe++){let Se=ee[oe];k.__webglColorRenderbuffer[oe]=s.createRenderbuffer(),s.bindRenderbuffer(s.RENDERBUFFER,k.__webglColorRenderbuffer[oe]);let Te=r.convert(Se.format,Se.colorSpace),se=r.convert(Se.type),pe=E(Se.internalFormat,Te,se,Se.colorSpace,T.isXRRenderTarget===!0),Ne=gt(T);s.renderbufferStorageMultisample(s.RENDERBUFFER,Ne,pe,T.width,T.height),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+oe,s.RENDERBUFFER,k.__webglColorRenderbuffer[oe])}s.bindRenderbuffer(s.RENDERBUFFER,null),T.depthBuffer&&(k.__webglDepthRenderbuffer=s.createRenderbuffer(),ke(k.__webglDepthRenderbuffer,T,!0)),t.bindFramebuffer(s.FRAMEBUFFER,null)}}if(X){t.bindTexture(s.TEXTURE_CUBE_MAP,j.__webglTexture),ve(s.TEXTURE_CUBE_MAP,x);for(let oe=0;oe<6;oe++)if(x.mipmaps&&x.mipmaps.length>0)for(let Se=0;Se<x.mipmaps.length;Se++)_e(k.__webglFramebuffer[oe][Se],T,x,s.COLOR_ATTACHMENT0,s.TEXTURE_CUBE_MAP_POSITIVE_X+oe,Se);else _e(k.__webglFramebuffer[oe],T,x,s.COLOR_ATTACHMENT0,s.TEXTURE_CUBE_MAP_POSITIVE_X+oe,0);m(x)&&p(s.TEXTURE_CUBE_MAP),t.unbindTexture()}else if(Ae){for(let oe=0,Se=ee.length;oe<Se;oe++){let Te=ee[oe],se=n.get(Te),pe=s.TEXTURE_2D;(T.isWebGL3DRenderTarget||T.isWebGLArrayRenderTarget)&&(pe=T.isWebGL3DRenderTarget?s.TEXTURE_3D:s.TEXTURE_2D_ARRAY),t.bindTexture(pe,se.__webglTexture),ve(pe,Te),_e(k.__webglFramebuffer,T,Te,s.COLOR_ATTACHMENT0+oe,pe,0),m(Te)&&p(pe)}t.unbindTexture()}else{let oe=s.TEXTURE_2D;if((T.isWebGL3DRenderTarget||T.isWebGLArrayRenderTarget)&&(oe=T.isWebGL3DRenderTarget?s.TEXTURE_3D:s.TEXTURE_2D_ARRAY),t.bindTexture(oe,j.__webglTexture),ve(oe,x),x.mipmaps&&x.mipmaps.length>0)for(let Se=0;Se<x.mipmaps.length;Se++)_e(k.__webglFramebuffer[Se],T,x,s.COLOR_ATTACHMENT0,oe,Se);else _e(k.__webglFramebuffer,T,x,s.COLOR_ATTACHMENT0,oe,0);m(x)&&p(oe),t.unbindTexture()}T.depthBuffer&&Qe(T)}function mt(T){let x=T.textures;for(let k=0,j=x.length;k<j;k++){let ee=x[k];if(m(ee)){let X=S(T),Ae=n.get(ee).__webglTexture;t.bindTexture(X,Ae),p(X),t.unbindTexture()}}}let He=[],Ue=[];function be(T){if(T.samples>0){if(Me(T)===!1){let x=T.textures,k=T.width,j=T.height,ee=s.COLOR_BUFFER_BIT,X=T.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT,Ae=n.get(T),oe=x.length>1;if(oe)for(let Te=0;Te<x.length;Te++)t.bindFramebuffer(s.FRAMEBUFFER,Ae.__webglMultisampledFramebuffer),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+Te,s.RENDERBUFFER,null),t.bindFramebuffer(s.FRAMEBUFFER,Ae.__webglFramebuffer),s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0+Te,s.TEXTURE_2D,null,0);t.bindFramebuffer(s.READ_FRAMEBUFFER,Ae.__webglMultisampledFramebuffer);let Se=T.texture.mipmaps;Se&&Se.length>0?t.bindFramebuffer(s.DRAW_FRAMEBUFFER,Ae.__webglFramebuffer[0]):t.bindFramebuffer(s.DRAW_FRAMEBUFFER,Ae.__webglFramebuffer);for(let Te=0;Te<x.length;Te++){if(T.resolveDepthBuffer&&(T.depthBuffer&&(ee|=s.DEPTH_BUFFER_BIT),T.stencilBuffer&&T.resolveStencilBuffer&&(ee|=s.STENCIL_BUFFER_BIT)),oe){s.framebufferRenderbuffer(s.READ_FRAMEBUFFER,s.COLOR_ATTACHMENT0,s.RENDERBUFFER,Ae.__webglColorRenderbuffer[Te]);let se=n.get(x[Te]).__webglTexture;s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0,s.TEXTURE_2D,se,0)}s.blitFramebuffer(0,0,k,j,0,0,k,j,ee,s.NEAREST),c===!0&&(He.length=0,Ue.length=0,He.push(s.COLOR_ATTACHMENT0+Te),T.depthBuffer&&T.resolveDepthBuffer===!1&&(He.push(X),Ue.push(X),s.invalidateFramebuffer(s.DRAW_FRAMEBUFFER,Ue)),s.invalidateFramebuffer(s.READ_FRAMEBUFFER,He))}if(t.bindFramebuffer(s.READ_FRAMEBUFFER,null),t.bindFramebuffer(s.DRAW_FRAMEBUFFER,null),oe)for(let Te=0;Te<x.length;Te++){t.bindFramebuffer(s.FRAMEBUFFER,Ae.__webglMultisampledFramebuffer),s.framebufferRenderbuffer(s.FRAMEBUFFER,s.COLOR_ATTACHMENT0+Te,s.RENDERBUFFER,Ae.__webglColorRenderbuffer[Te]);let se=n.get(x[Te]).__webglTexture;t.bindFramebuffer(s.FRAMEBUFFER,Ae.__webglFramebuffer),s.framebufferTexture2D(s.DRAW_FRAMEBUFFER,s.COLOR_ATTACHMENT0+Te,s.TEXTURE_2D,se,0)}t.bindFramebuffer(s.DRAW_FRAMEBUFFER,Ae.__webglMultisampledFramebuffer)}else if(T.depthBuffer&&T.resolveDepthBuffer===!1&&c){let x=T.stencilBuffer?s.DEPTH_STENCIL_ATTACHMENT:s.DEPTH_ATTACHMENT;s.invalidateFramebuffer(s.DRAW_FRAMEBUFFER,[x])}}}function gt(T){return Math.min(i.maxSamples,T.samples)}function Me(T){let x=n.get(T);return T.samples>0&&e.has("WEBGL_multisampled_render_to_texture")===!0&&x.__useRenderToTexture!==!1}function We(T){let x=o.render.frame;h.get(T)!==x&&(h.set(T,x),T.update())}function kt(T,x){let k=T.colorSpace,j=T.format,ee=T.type;return T.isCompressedTexture===!0||T.isVideoTexture===!0||k!==Vt&&k!==Hn&&($e.getTransfer(k)===ot?(j!==tn||ee!==zn)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",k)),x}function St(T){return typeof HTMLImageElement<"u"&&T instanceof HTMLImageElement?(l.width=T.naturalWidth||T.width,l.height=T.naturalHeight||T.height):typeof VideoFrame<"u"&&T instanceof VideoFrame?(l.width=T.displayWidth,l.height=T.displayHeight):(l.width=T.width,l.height=T.height),l}this.allocateTextureUnit=G,this.resetTextureUnits=H,this.setTexture2D=W,this.setTexture2DArray=Y,this.setTexture3D=B,this.setTextureCube=D,this.rebindTextures=Wt,this.setupRenderTarget=I,this.updateRenderTargetMipmap=mt,this.updateMultisampleRenderTarget=be,this.setupDepthRenderbuffer=Qe,this.setupFrameBufferTexture=_e,this.useMultisampledRTT=Me}function F0(s,e){function t(n,i=Hn){let r,o=$e.getTransfer(i);if(n===zn)return s.UNSIGNED_BYTE;if(n===Ta)return s.UNSIGNED_SHORT_4_4_4_4;if(n===wa)return s.UNSIGNED_SHORT_5_5_5_1;if(n===yl)return s.UNSIGNED_INT_5_9_9_9_REV;if(n===vl)return s.UNSIGNED_INT_10F_11F_11F_REV;if(n===_l)return s.BYTE;if(n===xl)return s.SHORT;if(n===sr)return s.UNSIGNED_SHORT;if(n===Ea)return s.INT;if(n===Xi)return s.UNSIGNED_INT;if(n===wn)return s.FLOAT;if(n===rr)return s.HALF_FLOAT;if(n===bl)return s.ALPHA;if(n===Ml)return s.RGB;if(n===tn)return s.RGBA;if(n===Ws)return s.DEPTH_COMPONENT;if(n===ar)return s.DEPTH_STENCIL;if(n===Aa)return s.RED;if(n===Ra)return s.RED_INTEGER;if(n===Sl)return s.RG;if(n===Ca)return s.RG_INTEGER;if(n===Ia)return s.RGBA_INTEGER;if(n===oo||n===ao||n===co||n===lo)if(o===ot)if(r=e.get("WEBGL_compressed_texture_s3tc_srgb"),r!==null){if(n===oo)return r.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(n===ao)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(n===co)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(n===lo)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(r=e.get("WEBGL_compressed_texture_s3tc"),r!==null){if(n===oo)return r.COMPRESSED_RGB_S3TC_DXT1_EXT;if(n===ao)return r.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(n===co)return r.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(n===lo)return r.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(n===Pa||n===La||n===Na||n===Da)if(r=e.get("WEBGL_compressed_texture_pvrtc"),r!==null){if(n===Pa)return r.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(n===La)return r.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(n===Na)return r.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(n===Da)return r.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(n===Ua||n===Oa||n===Fa)if(r=e.get("WEBGL_compressed_texture_etc"),r!==null){if(n===Ua||n===Oa)return o===ot?r.COMPRESSED_SRGB8_ETC2:r.COMPRESSED_RGB8_ETC2;if(n===Fa)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:r.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(n===Ba||n===ka||n===za||n===Ha||n===Va||n===Ga||n===Wa||n===Xa||n===qa||n===Ya||n===ja||n===Ka||n===Za||n===Ja)if(r=e.get("WEBGL_compressed_texture_astc"),r!==null){if(n===Ba)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:r.COMPRESSED_RGBA_ASTC_4x4_KHR;if(n===ka)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:r.COMPRESSED_RGBA_ASTC_5x4_KHR;if(n===za)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:r.COMPRESSED_RGBA_ASTC_5x5_KHR;if(n===Ha)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:r.COMPRESSED_RGBA_ASTC_6x5_KHR;if(n===Va)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:r.COMPRESSED_RGBA_ASTC_6x6_KHR;if(n===Ga)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:r.COMPRESSED_RGBA_ASTC_8x5_KHR;if(n===Wa)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:r.COMPRESSED_RGBA_ASTC_8x6_KHR;if(n===Xa)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:r.COMPRESSED_RGBA_ASTC_8x8_KHR;if(n===qa)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:r.COMPRESSED_RGBA_ASTC_10x5_KHR;if(n===Ya)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:r.COMPRESSED_RGBA_ASTC_10x6_KHR;if(n===ja)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:r.COMPRESSED_RGBA_ASTC_10x8_KHR;if(n===Ka)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:r.COMPRESSED_RGBA_ASTC_10x10_KHR;if(n===Za)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:r.COMPRESSED_RGBA_ASTC_12x10_KHR;if(n===Ja)return o===ot?r.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:r.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(n===$a||n===Qa||n===ec)if(r=e.get("EXT_texture_compression_bptc"),r!==null){if(n===$a)return o===ot?r.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:r.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(n===Qa)return r.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(n===ec)return r.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(n===tc||n===nc||n===ic||n===sc)if(r=e.get("EXT_texture_compression_rgtc"),r!==null){if(n===tc)return r.COMPRESSED_RED_RGTC1_EXT;if(n===nc)return r.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(n===ic)return r.COMPRESSED_RED_GREEN_RGTC2_EXT;if(n===sc)return r.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return n===or?s.UNSIGNED_INT_24_8:s[n]!==void 0?s[n]:null}return{convert:t}}var B0=`
void main() {

	gl_Position = vec4( position, 1.0 );

}`,k0=`
uniform sampler2DArray depthColor;
uniform float depthWidth;
uniform float depthHeight;

void main() {

	vec2 coord = vec2( gl_FragCoord.x / depthWidth, gl_FragCoord.y / depthHeight );

	if ( coord.x >= 1.0 ) {

		gl_FragDepth = texture( depthColor, vec3( coord.x - 1.0, coord.y, 1 ) ).r;

	} else {

		gl_FragDepth = texture( depthColor, vec3( coord.x, coord.y, 0 ) ).r;

	}

}`,ql=class{constructor(){this.texture=null,this.mesh=null,this.depthNear=0,this.depthFar=0}init(e,t){if(this.texture===null){let n=new qr(e.texture);(e.depthNear!==t.depthNear||e.depthFar!==t.depthFar)&&(this.depthNear=e.depthNear,this.depthFar=e.depthFar),this.texture=n}}getMesh(e){if(this.texture!==null&&this.mesh===null){let t=e.cameras[0].viewport,n=new Bn({vertexShader:B0,fragmentShader:k0,uniforms:{depthColor:{value:this.texture},depthWidth:{value:t.z},depthHeight:{value:t.w}}});this.mesh=new lt(new cs(20,20),n)}return this.mesh}reset(){this.texture=null,this.mesh=null}getDepthTexture(){return this.texture}},Yl=class extends Kn{constructor(e,t){super();let n=this,i=null,r=1,o=null,a="local-floor",c=1,l=null,h=null,u=null,d=null,f=null,g=null,_=typeof XRWebGLBinding<"u",m=new ql,p={},S=t.getContextAttributes(),E=null,y=null,w=[],C=[],R=new Ee,N=null,M=new wt;M.viewport=new nt;let b=new wt;b.viewport=new nt;let P=[M,b],H=new da,G=null,q=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(K){let te=w[K];return te===void 0&&(te=new Zs,w[K]=te),te.getTargetRaySpace()},this.getControllerGrip=function(K){let te=w[K];return te===void 0&&(te=new Zs,w[K]=te),te.getGripSpace()},this.getHand=function(K){let te=w[K];return te===void 0&&(te=new Zs,w[K]=te),te.getHandSpace()};function W(K){let te=C.indexOf(K.inputSource);if(te===-1)return;let _e=w[te];_e!==void 0&&(_e.update(K.inputSource,K.frame,l||o),_e.dispatchEvent({type:K.type,data:K.inputSource}))}function Y(){i.removeEventListener("select",W),i.removeEventListener("selectstart",W),i.removeEventListener("selectend",W),i.removeEventListener("squeeze",W),i.removeEventListener("squeezestart",W),i.removeEventListener("squeezeend",W),i.removeEventListener("end",Y),i.removeEventListener("inputsourceschange",B);for(let K=0;K<w.length;K++){let te=C[K];te!==null&&(C[K]=null,w[K].disconnect(te))}G=null,q=null,m.reset();for(let K in p)delete p[K];e.setRenderTarget(E),f=null,d=null,u=null,i=null,y=null,Ye.stop(),n.isPresenting=!1,e.setPixelRatio(N),e.setSize(R.width,R.height,!1),n.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(K){r=K,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(K){a=K,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return l||o},this.setReferenceSpace=function(K){l=K},this.getBaseLayer=function(){return d!==null?d:f},this.getBinding=function(){return u===null&&_&&(u=new XRWebGLBinding(i,t)),u},this.getFrame=function(){return g},this.getSession=function(){return i},this.setSession=async function(K){if(i=K,i!==null){if(E=e.getRenderTarget(),i.addEventListener("select",W),i.addEventListener("selectstart",W),i.addEventListener("selectend",W),i.addEventListener("squeeze",W),i.addEventListener("squeezestart",W),i.addEventListener("squeezeend",W),i.addEventListener("end",Y),i.addEventListener("inputsourceschange",B),S.xrCompatible!==!0&&await t.makeXRCompatible(),N=e.getPixelRatio(),e.getSize(R),_&&"createProjectionLayer"in XRWebGLBinding.prototype){let _e=null,ke=null,Ce=null;S.depth&&(Ce=S.stencil?t.DEPTH24_STENCIL8:t.DEPTH_COMPONENT24,_e=S.stencil?ar:Ws,ke=S.stencil?or:Xi);let Qe={colorFormat:t.RGBA8,depthFormat:Ce,scaleFactor:r};u=this.getBinding(),d=u.createProjectionLayer(Qe),i.updateRenderState({layers:[d]}),e.setPixelRatio(1),e.setSize(d.textureWidth,d.textureHeight,!1),y=new Zn(d.textureWidth,d.textureHeight,{format:tn,type:zn,depthTexture:new Xr(d.textureWidth,d.textureHeight,ke,void 0,void 0,void 0,void 0,void 0,void 0,_e),stencilBuffer:S.stencil,colorSpace:e.outputColorSpace,samples:S.antialias?4:0,resolveDepthBuffer:d.ignoreDepthValues===!1,resolveStencilBuffer:d.ignoreDepthValues===!1})}else{let _e={antialias:S.antialias,alpha:!0,depth:S.depth,stencil:S.stencil,framebufferScaleFactor:r};f=new XRWebGLLayer(i,t,_e),i.updateRenderState({baseLayer:f}),e.setPixelRatio(1),e.setSize(f.framebufferWidth,f.framebufferHeight,!1),y=new Zn(f.framebufferWidth,f.framebufferHeight,{format:tn,type:zn,colorSpace:e.outputColorSpace,stencilBuffer:S.stencil,resolveDepthBuffer:f.ignoreDepthValues===!1,resolveStencilBuffer:f.ignoreDepthValues===!1})}y.isXRRenderTarget=!0,this.setFoveation(c),l=null,o=await i.requestReferenceSpace(a),Ye.setContext(i),Ye.start(),n.isPresenting=!0,n.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(i!==null)return i.environmentBlendMode},this.getDepthTexture=function(){return m.getDepthTexture()};function B(K){for(let te=0;te<K.removed.length;te++){let _e=K.removed[te],ke=C.indexOf(_e);ke>=0&&(C[ke]=null,w[ke].disconnect(_e))}for(let te=0;te<K.added.length;te++){let _e=K.added[te],ke=C.indexOf(_e);if(ke===-1){for(let Qe=0;Qe<w.length;Qe++)if(Qe>=C.length){C.push(_e),ke=Qe;break}else if(C[Qe]===null){C[Qe]=_e,ke=Qe;break}if(ke===-1)break}let Ce=w[ke];Ce&&Ce.connect(_e)}}let D=new A,Q=new A;function ae(K,te,_e){D.setFromMatrixPosition(te.matrixWorld),Q.setFromMatrixPosition(_e.matrixWorld);let ke=D.distanceTo(Q),Ce=te.projectionMatrix.elements,Qe=_e.projectionMatrix.elements,Wt=Ce[14]/(Ce[10]-1),I=Ce[14]/(Ce[10]+1),mt=(Ce[9]+1)/Ce[5],He=(Ce[9]-1)/Ce[5],Ue=(Ce[8]-1)/Ce[0],be=(Qe[8]+1)/Qe[0],gt=Wt*Ue,Me=Wt*be,We=ke/(-Ue+be),kt=We*-Ue;if(te.matrixWorld.decompose(K.position,K.quaternion,K.scale),K.translateX(kt),K.translateZ(We),K.matrixWorld.compose(K.position,K.quaternion,K.scale),K.matrixWorldInverse.copy(K.matrixWorld).invert(),Ce[10]===-1)K.projectionMatrix.copy(te.projectionMatrix),K.projectionMatrixInverse.copy(te.projectionMatrixInverse);else{let St=Wt+We,T=I+We,x=gt-kt,k=Me+(ke-kt),j=mt*I/T*St,ee=He*I/T*St;K.projectionMatrix.makePerspective(x,k,j,ee,St,T),K.projectionMatrixInverse.copy(K.projectionMatrix).invert()}}function ce(K,te){te===null?K.matrixWorld.copy(K.matrix):K.matrixWorld.multiplyMatrices(te.matrixWorld,K.matrix),K.matrixWorldInverse.copy(K.matrixWorld).invert()}this.updateCamera=function(K){if(i===null)return;let te=K.near,_e=K.far;m.texture!==null&&(m.depthNear>0&&(te=m.depthNear),m.depthFar>0&&(_e=m.depthFar)),H.near=b.near=M.near=te,H.far=b.far=M.far=_e,(G!==H.near||q!==H.far)&&(i.updateRenderState({depthNear:H.near,depthFar:H.far}),G=H.near,q=H.far),H.layers.mask=K.layers.mask|6,M.layers.mask=H.layers.mask&3,b.layers.mask=H.layers.mask&5;let ke=K.parent,Ce=H.cameras;ce(H,ke);for(let Qe=0;Qe<Ce.length;Qe++)ce(Ce[Qe],ke);Ce.length===2?ae(H,M,b):H.projectionMatrix.copy(M.projectionMatrix),ve(K,H,ke)};function ve(K,te,_e){_e===null?K.matrix.copy(te.matrixWorld):(K.matrix.copy(_e.matrixWorld),K.matrix.invert(),K.matrix.multiply(te.matrixWorld)),K.matrix.decompose(K.position,K.quaternion,K.scale),K.updateMatrixWorld(!0),K.projectionMatrix.copy(te.projectionMatrix),K.projectionMatrixInverse.copy(te.projectionMatrixInverse),K.isPerspectiveCamera&&(K.fov=ss*2*Math.atan(1/K.projectionMatrix.elements[5]),K.zoom=1)}this.getCamera=function(){return H},this.getFoveation=function(){if(!(d===null&&f===null))return c},this.setFoveation=function(K){c=K,d!==null&&(d.fixedFoveation=K),f!==null&&f.fixedFoveation!==void 0&&(f.fixedFoveation=K)},this.hasDepthSensing=function(){return m.texture!==null},this.getDepthSensingMesh=function(){return m.getMesh(H)},this.getCameraTexture=function(K){return p[K]};let Ze=null;function it(K,te){if(h=te.getViewerPose(l||o),g=te,h!==null){let _e=h.views;f!==null&&(e.setRenderTargetFramebuffer(y,f.framebuffer),e.setRenderTarget(y));let ke=!1;_e.length!==H.cameras.length&&(H.cameras.length=0,ke=!0);for(let I=0;I<_e.length;I++){let mt=_e[I],He=null;if(f!==null)He=f.getViewport(mt);else{let be=u.getViewSubImage(d,mt);He=be.viewport,I===0&&(e.setRenderTargetTextures(y,be.colorTexture,be.depthStencilTexture),e.setRenderTarget(y))}let Ue=P[I];Ue===void 0&&(Ue=new wt,Ue.layers.enable(I),Ue.viewport=new nt,P[I]=Ue),Ue.matrix.fromArray(mt.transform.matrix),Ue.matrix.decompose(Ue.position,Ue.quaternion,Ue.scale),Ue.projectionMatrix.fromArray(mt.projectionMatrix),Ue.projectionMatrixInverse.copy(Ue.projectionMatrix).invert(),Ue.viewport.set(He.x,He.y,He.width,He.height),I===0&&(H.matrix.copy(Ue.matrix),H.matrix.decompose(H.position,H.quaternion,H.scale)),ke===!0&&H.cameras.push(Ue)}let Ce=i.enabledFeatures;if(Ce&&Ce.includes("depth-sensing")&&i.depthUsage=="gpu-optimized"&&_){u=n.getBinding();let I=u.getDepthInformation(_e[0]);I&&I.isValid&&I.texture&&m.init(I,i.renderState)}if(Ce&&Ce.includes("camera-access")&&_){e.state.unbindTexture(),u=n.getBinding();for(let I=0;I<_e.length;I++){let mt=_e[I].camera;if(mt){let He=p[mt];He||(He=new qr,p[mt]=He);let Ue=u.getCameraImage(mt);He.sourceTexture=Ue}}}}for(let _e=0;_e<w.length;_e++){let ke=C[_e],Ce=w[_e];ke!==null&&Ce!==void 0&&Ce.update(ke,te,l||o)}Ze&&Ze(K,te),te.detectedPlanes&&n.dispatchEvent({type:"planesdetected",data:te}),g=null}let Ye=new xf;Ye.setAnimationLoop(it),this.setAnimationLoop=function(K){Ze=K},this.dispose=function(){}}},ms=new Fn,z0=new De;function H0(s,e){function t(m,p){m.matrixAutoUpdate===!0&&m.updateMatrix(),p.value.copy(m.matrix)}function n(m,p){p.color.getRGB(m.fogColor.value,Il(s)),p.isFog?(m.fogNear.value=p.near,m.fogFar.value=p.far):p.isFogExp2&&(m.fogDensity.value=p.density)}function i(m,p,S,E,y){p.isMeshBasicMaterial||p.isMeshLambertMaterial?r(m,p):p.isMeshToonMaterial?(r(m,p),u(m,p)):p.isMeshPhongMaterial?(r(m,p),h(m,p)):p.isMeshStandardMaterial?(r(m,p),d(m,p),p.isMeshPhysicalMaterial&&f(m,p,y)):p.isMeshMatcapMaterial?(r(m,p),g(m,p)):p.isMeshDepthMaterial?r(m,p):p.isMeshDistanceMaterial?(r(m,p),_(m,p)):p.isMeshNormalMaterial?r(m,p):p.isLineBasicMaterial?(o(m,p),p.isLineDashedMaterial&&a(m,p)):p.isPointsMaterial?c(m,p,S,E):p.isSpriteMaterial?l(m,p):p.isShadowMaterial?(m.color.value.copy(p.color),m.opacity.value=p.opacity):p.isShaderMaterial&&(p.uniformsNeedUpdate=!1)}function r(m,p){m.opacity.value=p.opacity,p.color&&m.diffuse.value.copy(p.color),p.emissive&&m.emissive.value.copy(p.emissive).multiplyScalar(p.emissiveIntensity),p.map&&(m.map.value=p.map,t(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,t(p.alphaMap,m.alphaMapTransform)),p.bumpMap&&(m.bumpMap.value=p.bumpMap,t(p.bumpMap,m.bumpMapTransform),m.bumpScale.value=p.bumpScale,p.side===en&&(m.bumpScale.value*=-1)),p.normalMap&&(m.normalMap.value=p.normalMap,t(p.normalMap,m.normalMapTransform),m.normalScale.value.copy(p.normalScale),p.side===en&&m.normalScale.value.negate()),p.displacementMap&&(m.displacementMap.value=p.displacementMap,t(p.displacementMap,m.displacementMapTransform),m.displacementScale.value=p.displacementScale,m.displacementBias.value=p.displacementBias),p.emissiveMap&&(m.emissiveMap.value=p.emissiveMap,t(p.emissiveMap,m.emissiveMapTransform)),p.specularMap&&(m.specularMap.value=p.specularMap,t(p.specularMap,m.specularMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest);let S=e.get(p),E=S.envMap,y=S.envMapRotation;E&&(m.envMap.value=E,ms.copy(y),ms.x*=-1,ms.y*=-1,ms.z*=-1,E.isCubeTexture&&E.isRenderTargetTexture===!1&&(ms.y*=-1,ms.z*=-1),m.envMapRotation.value.setFromMatrix4(z0.makeRotationFromEuler(ms)),m.flipEnvMap.value=E.isCubeTexture&&E.isRenderTargetTexture===!1?-1:1,m.reflectivity.value=p.reflectivity,m.ior.value=p.ior,m.refractionRatio.value=p.refractionRatio),p.lightMap&&(m.lightMap.value=p.lightMap,m.lightMapIntensity.value=p.lightMapIntensity,t(p.lightMap,m.lightMapTransform)),p.aoMap&&(m.aoMap.value=p.aoMap,m.aoMapIntensity.value=p.aoMapIntensity,t(p.aoMap,m.aoMapTransform))}function o(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,p.map&&(m.map.value=p.map,t(p.map,m.mapTransform))}function a(m,p){m.dashSize.value=p.dashSize,m.totalSize.value=p.dashSize+p.gapSize,m.scale.value=p.scale}function c(m,p,S,E){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.size.value=p.size*S,m.scale.value=E*.5,p.map&&(m.map.value=p.map,t(p.map,m.uvTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,t(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function l(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.rotation.value=p.rotation,p.map&&(m.map.value=p.map,t(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,t(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function h(m,p){m.specular.value.copy(p.specular),m.shininess.value=Math.max(p.shininess,1e-4)}function u(m,p){p.gradientMap&&(m.gradientMap.value=p.gradientMap)}function d(m,p){m.metalness.value=p.metalness,p.metalnessMap&&(m.metalnessMap.value=p.metalnessMap,t(p.metalnessMap,m.metalnessMapTransform)),m.roughness.value=p.roughness,p.roughnessMap&&(m.roughnessMap.value=p.roughnessMap,t(p.roughnessMap,m.roughnessMapTransform)),p.envMap&&(m.envMapIntensity.value=p.envMapIntensity)}function f(m,p,S){m.ior.value=p.ior,p.sheen>0&&(m.sheenColor.value.copy(p.sheenColor).multiplyScalar(p.sheen),m.sheenRoughness.value=p.sheenRoughness,p.sheenColorMap&&(m.sheenColorMap.value=p.sheenColorMap,t(p.sheenColorMap,m.sheenColorMapTransform)),p.sheenRoughnessMap&&(m.sheenRoughnessMap.value=p.sheenRoughnessMap,t(p.sheenRoughnessMap,m.sheenRoughnessMapTransform))),p.clearcoat>0&&(m.clearcoat.value=p.clearcoat,m.clearcoatRoughness.value=p.clearcoatRoughness,p.clearcoatMap&&(m.clearcoatMap.value=p.clearcoatMap,t(p.clearcoatMap,m.clearcoatMapTransform)),p.clearcoatRoughnessMap&&(m.clearcoatRoughnessMap.value=p.clearcoatRoughnessMap,t(p.clearcoatRoughnessMap,m.clearcoatRoughnessMapTransform)),p.clearcoatNormalMap&&(m.clearcoatNormalMap.value=p.clearcoatNormalMap,t(p.clearcoatNormalMap,m.clearcoatNormalMapTransform),m.clearcoatNormalScale.value.copy(p.clearcoatNormalScale),p.side===en&&m.clearcoatNormalScale.value.negate())),p.dispersion>0&&(m.dispersion.value=p.dispersion),p.iridescence>0&&(m.iridescence.value=p.iridescence,m.iridescenceIOR.value=p.iridescenceIOR,m.iridescenceThicknessMinimum.value=p.iridescenceThicknessRange[0],m.iridescenceThicknessMaximum.value=p.iridescenceThicknessRange[1],p.iridescenceMap&&(m.iridescenceMap.value=p.iridescenceMap,t(p.iridescenceMap,m.iridescenceMapTransform)),p.iridescenceThicknessMap&&(m.iridescenceThicknessMap.value=p.iridescenceThicknessMap,t(p.iridescenceThicknessMap,m.iridescenceThicknessMapTransform))),p.transmission>0&&(m.transmission.value=p.transmission,m.transmissionSamplerMap.value=S.texture,m.transmissionSamplerSize.value.set(S.width,S.height),p.transmissionMap&&(m.transmissionMap.value=p.transmissionMap,t(p.transmissionMap,m.transmissionMapTransform)),m.thickness.value=p.thickness,p.thicknessMap&&(m.thicknessMap.value=p.thicknessMap,t(p.thicknessMap,m.thicknessMapTransform)),m.attenuationDistance.value=p.attenuationDistance,m.attenuationColor.value.copy(p.attenuationColor)),p.anisotropy>0&&(m.anisotropyVector.value.set(p.anisotropy*Math.cos(p.anisotropyRotation),p.anisotropy*Math.sin(p.anisotropyRotation)),p.anisotropyMap&&(m.anisotropyMap.value=p.anisotropyMap,t(p.anisotropyMap,m.anisotropyMapTransform))),m.specularIntensity.value=p.specularIntensity,m.specularColor.value.copy(p.specularColor),p.specularColorMap&&(m.specularColorMap.value=p.specularColorMap,t(p.specularColorMap,m.specularColorMapTransform)),p.specularIntensityMap&&(m.specularIntensityMap.value=p.specularIntensityMap,t(p.specularIntensityMap,m.specularIntensityMapTransform))}function g(m,p){p.matcap&&(m.matcap.value=p.matcap)}function _(m,p){let S=e.get(p).light;m.referencePosition.value.setFromMatrixPosition(S.matrixWorld),m.nearDistance.value=S.shadow.camera.near,m.farDistance.value=S.shadow.camera.far}return{refreshFogUniforms:n,refreshMaterialUniforms:i}}function V0(s,e,t,n){let i={},r={},o=[],a=s.getParameter(s.MAX_UNIFORM_BUFFER_BINDINGS);function c(S,E){let y=E.program;n.uniformBlockBinding(S,y)}function l(S,E){let y=i[S.id];y===void 0&&(g(S),y=h(S),i[S.id]=y,S.addEventListener("dispose",m));let w=E.program;n.updateUBOMapping(S,w);let C=e.render.frame;r[S.id]!==C&&(d(S),r[S.id]=C)}function h(S){let E=u();S.__bindingPointIndex=E;let y=s.createBuffer(),w=S.__size,C=S.usage;return s.bindBuffer(s.UNIFORM_BUFFER,y),s.bufferData(s.UNIFORM_BUFFER,w,C),s.bindBuffer(s.UNIFORM_BUFFER,null),s.bindBufferBase(s.UNIFORM_BUFFER,E,y),y}function u(){for(let S=0;S<a;S++)if(o.indexOf(S)===-1)return o.push(S),S;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function d(S){let E=i[S.id],y=S.uniforms,w=S.__cache;s.bindBuffer(s.UNIFORM_BUFFER,E);for(let C=0,R=y.length;C<R;C++){let N=Array.isArray(y[C])?y[C]:[y[C]];for(let M=0,b=N.length;M<b;M++){let P=N[M];if(f(P,C,M,w)===!0){let H=P.__offset,G=Array.isArray(P.value)?P.value:[P.value],q=0;for(let W=0;W<G.length;W++){let Y=G[W],B=_(Y);typeof Y=="number"||typeof Y=="boolean"?(P.__data[0]=Y,s.bufferSubData(s.UNIFORM_BUFFER,H+q,P.__data)):Y.isMatrix3?(P.__data[0]=Y.elements[0],P.__data[1]=Y.elements[1],P.__data[2]=Y.elements[2],P.__data[3]=0,P.__data[4]=Y.elements[3],P.__data[5]=Y.elements[4],P.__data[6]=Y.elements[5],P.__data[7]=0,P.__data[8]=Y.elements[6],P.__data[9]=Y.elements[7],P.__data[10]=Y.elements[8],P.__data[11]=0):(Y.toArray(P.__data,q),q+=B.storage/Float32Array.BYTES_PER_ELEMENT)}s.bufferSubData(s.UNIFORM_BUFFER,H,P.__data)}}}s.bindBuffer(s.UNIFORM_BUFFER,null)}function f(S,E,y,w){let C=S.value,R=E+"_"+y;if(w[R]===void 0)return typeof C=="number"||typeof C=="boolean"?w[R]=C:w[R]=C.clone(),!0;{let N=w[R];if(typeof C=="number"||typeof C=="boolean"){if(N!==C)return w[R]=C,!0}else if(N.equals(C)===!1)return N.copy(C),!0}return!1}function g(S){let E=S.uniforms,y=0,w=16;for(let R=0,N=E.length;R<N;R++){let M=Array.isArray(E[R])?E[R]:[E[R]];for(let b=0,P=M.length;b<P;b++){let H=M[b],G=Array.isArray(H.value)?H.value:[H.value];for(let q=0,W=G.length;q<W;q++){let Y=G[q],B=_(Y),D=y%w,Q=D%B.boundary,ae=D+Q;y+=Q,ae!==0&&w-ae<B.storage&&(y+=w-ae),H.__data=new Float32Array(B.storage/Float32Array.BYTES_PER_ELEMENT),H.__offset=y,y+=B.storage}}}let C=y%w;return C>0&&(y+=w-C),S.__size=y,S.__cache={},this}function _(S){let E={boundary:0,storage:0};return typeof S=="number"||typeof S=="boolean"?(E.boundary=4,E.storage=4):S.isVector2?(E.boundary=8,E.storage=8):S.isVector3||S.isColor?(E.boundary=16,E.storage=12):S.isVector4?(E.boundary=16,E.storage=16):S.isMatrix3?(E.boundary=48,E.storage=48):S.isMatrix4?(E.boundary=64,E.storage=64):S.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",S),E}function m(S){let E=S.target;E.removeEventListener("dispose",m);let y=o.indexOf(E.__bindingPointIndex);o.splice(y,1),s.deleteBuffer(i[E.id]),delete i[E.id],delete r[E.id]}function p(){for(let S in i)s.deleteBuffer(i[S]);o=[],i={},r={}}return{bind:c,update:l,dispose:p}}var lc=class{constructor(e={}){let{canvas:t=Hd(),context:n=null,depth:i=!0,stencil:r=!1,alpha:o=!1,antialias:a=!1,premultipliedAlpha:c=!0,preserveDrawingBuffer:l=!1,powerPreference:h="default",failIfMajorPerformanceCaveat:u=!1,reversedDepthBuffer:d=!1}=e;this.isWebGLRenderer=!0;let f;if(n!==null){if(typeof WebGLRenderingContext<"u"&&n instanceof WebGLRenderingContext)throw new Error("THREE.WebGLRenderer: WebGL 1 is not supported since r163.");f=n.getContextAttributes().alpha}else f=o;let g=new Uint32Array(4),_=new Int32Array(4),m=null,p=null,S=[],E=[];this.domElement=t,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this.toneMapping=Si,this.toneMappingExposure=1,this.transmissionResolutionScale=1;let y=this,w=!1;this._outputColorSpace=_t;let C=0,R=0,N=null,M=-1,b=null,P=new nt,H=new nt,G=null,q=new Re(0),W=0,Y=t.width,B=t.height,D=1,Q=null,ae=null,ce=new nt(0,0,Y,B),ve=new nt(0,0,Y,B),Ze=!1,it=new Qs,Ye=!1,K=!1,te=new De,_e=new A,ke=new nt,Ce={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0},Qe=!1;function Wt(){return N===null?D:1}let I=n;function mt(v,U){return t.getContext(v,U)}try{let v={alpha:!0,depth:i,stencil:r,antialias:a,premultipliedAlpha:c,preserveDrawingBuffer:l,powerPreference:h,failIfMajorPerformanceCaveat:u};if("setAttribute"in t&&t.setAttribute("data-engine",`three.js r${"180"}`),t.addEventListener("webglcontextlost",le,!1),t.addEventListener("webglcontextrestored",ge,!1),t.addEventListener("webglcontextcreationerror",ne,!1),I===null){let U="webgl2";if(I=mt(U,v),I===null)throw mt(U)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}}catch(v){throw console.error("THREE.WebGLRenderer: "+v.message),v}let He,Ue,be,gt,Me,We,kt,St,T,x,k,j,ee,X,Ae,oe,Se,Te,se,pe,Ne,we,de,Ge;function L(){He=new ox(I),He.init(),we=new F0(I,He),Ue=new Q_(I,He,e,we),be=new U0(I,He),Ue.reversedDepthBuffer&&d&&be.buffers.depth.setReversed(!0),gt=new lx(I),Me=new M0,We=new O0(I,He,be,Me,Ue,we,gt),kt=new tx(y),St=new rx(y),T=new mm(I),de=new J_(I,T),x=new ax(I,T,gt,de),k=new ux(I,x,T,gt),se=new hx(I,Ue,We),oe=new ex(Me),j=new b0(y,kt,St,He,Ue,de,oe),ee=new H0(y,Me),X=new E0,Ae=new I0(He),Te=new Z_(y,kt,St,be,k,f,c),Se=new N0(y,k,Ue),Ge=new V0(I,gt,Ue,be),pe=new $_(I,He,gt),Ne=new cx(I,He,gt),gt.programs=j.programs,y.capabilities=Ue,y.extensions=He,y.properties=Me,y.renderLists=X,y.shadowMap=Se,y.state=be,y.info=gt}L();let re=new Yl(y,I);this.xr=re,this.getContext=function(){return I},this.getContextAttributes=function(){return I.getContextAttributes()},this.forceContextLoss=function(){let v=He.get("WEBGL_lose_context");v&&v.loseContext()},this.forceContextRestore=function(){let v=He.get("WEBGL_lose_context");v&&v.restoreContext()},this.getPixelRatio=function(){return D},this.setPixelRatio=function(v){v!==void 0&&(D=v,this.setSize(Y,B,!1))},this.getSize=function(v){return v.set(Y,B)},this.setSize=function(v,U,z=!0){if(re.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}Y=v,B=U,t.width=Math.floor(v*D),t.height=Math.floor(U*D),z===!0&&(t.style.width=v+"px",t.style.height=U+"px"),this.setViewport(0,0,v,U)},this.getDrawingBufferSize=function(v){return v.set(Y*D,B*D).floor()},this.setDrawingBufferSize=function(v,U,z){Y=v,B=U,D=z,t.width=Math.floor(v*z),t.height=Math.floor(U*z),this.setViewport(0,0,v,U)},this.getCurrentViewport=function(v){return v.copy(P)},this.getViewport=function(v){return v.copy(ce)},this.setViewport=function(v,U,z,V){v.isVector4?ce.set(v.x,v.y,v.z,v.w):ce.set(v,U,z,V),be.viewport(P.copy(ce).multiplyScalar(D).round())},this.getScissor=function(v){return v.copy(ve)},this.setScissor=function(v,U,z,V){v.isVector4?ve.set(v.x,v.y,v.z,v.w):ve.set(v,U,z,V),be.scissor(H.copy(ve).multiplyScalar(D).round())},this.getScissorTest=function(){return Ze},this.setScissorTest=function(v){be.setScissorTest(Ze=v)},this.setOpaqueSort=function(v){Q=v},this.setTransparentSort=function(v){ae=v},this.getClearColor=function(v){return v.copy(Te.getClearColor())},this.setClearColor=function(){Te.setClearColor(...arguments)},this.getClearAlpha=function(){return Te.getClearAlpha()},this.setClearAlpha=function(){Te.setClearAlpha(...arguments)},this.clear=function(v=!0,U=!0,z=!0){let V=0;if(v){let O=!1;if(N!==null){let ie=N.texture.format;O=ie===Ia||ie===Ca||ie===Ra}if(O){let ie=N.texture.type,fe=ie===zn||ie===Xi||ie===sr||ie===or||ie===Ta||ie===wa,xe=Te.getClearColor(),me=Te.getClearAlpha(),Le=xe.r,Oe=xe.g,Ie=xe.b;fe?(g[0]=Le,g[1]=Oe,g[2]=Ie,g[3]=me,I.clearBufferuiv(I.COLOR,0,g)):(_[0]=Le,_[1]=Oe,_[2]=Ie,_[3]=me,I.clearBufferiv(I.COLOR,0,_))}else V|=I.COLOR_BUFFER_BIT}U&&(V|=I.DEPTH_BUFFER_BIT),z&&(V|=I.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),I.clear(V)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){t.removeEventListener("webglcontextlost",le,!1),t.removeEventListener("webglcontextrestored",ge,!1),t.removeEventListener("webglcontextcreationerror",ne,!1),Te.dispose(),X.dispose(),Ae.dispose(),Me.dispose(),kt.dispose(),St.dispose(),k.dispose(),de.dispose(),Ge.dispose(),j.dispose(),re.dispose(),re.removeEventListener("sessionstart",qn),re.removeEventListener("sessionend",lu),Ki.stop()};function le(v){v.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),w=!0}function ge(){console.log("THREE.WebGLRenderer: Context Restored."),w=!1;let v=gt.autoReset,U=Se.enabled,z=Se.autoUpdate,V=Se.needsUpdate,O=Se.type;L(),gt.autoReset=v,Se.enabled=U,Se.autoUpdate=z,Se.needsUpdate=V,Se.type=O}function ne(v){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",v.statusMessage)}function Z(v){let U=v.target;U.removeEventListener("dispose",Z),ye(U)}function ye(v){ze(v),Me.remove(v)}function ze(v){let U=Me.get(v).programs;U!==void 0&&(U.forEach(function(z){j.releaseProgram(z)}),v.isShaderMaterial&&j.releaseShaderCache(v))}this.renderBufferDirect=function(v,U,z,V,O,ie){U===null&&(U=Ce);let fe=O.isMesh&&O.matrixWorld.determinant()<0,xe=hp(v,U,z,V,O);be.setMaterial(V,fe);let me=z.index,Le=1;if(V.wireframe===!0){if(me=x.getWireframeAttribute(z),me===void 0)return;Le=2}let Oe=z.drawRange,Ie=z.attributes.position,Je=Oe.start*Le,ct=(Oe.start+Oe.count)*Le;ie!==null&&(Je=Math.max(Je,ie.start*Le),ct=Math.min(ct,(ie.start+ie.count)*Le)),me!==null?(Je=Math.max(Je,0),ct=Math.min(ct,me.count)):Ie!=null&&(Je=Math.max(Je,0),ct=Math.min(ct,Ie.count));let Mt=ct-Je;if(Mt<0||Mt===1/0)return;de.setup(O,V,xe,z,me);let ft,ut=pe;if(me!==null&&(ft=T.get(me),ut=Ne,ut.setIndex(ft)),O.isMesh)V.wireframe===!0?(be.setLineWidth(V.wireframeLinewidth*Wt()),ut.setMode(I.LINES)):ut.setMode(I.TRIANGLES);else if(O.isLine){let Pe=V.linewidth;Pe===void 0&&(Pe=1),be.setLineWidth(Pe*Wt()),O.isLineSegments?ut.setMode(I.LINES):O.isLineLoop?ut.setMode(I.LINE_LOOP):ut.setMode(I.LINE_STRIP)}else O.isPoints?ut.setMode(I.POINTS):O.isSprite&&ut.setMode(I.TRIANGLES);if(O.isBatchedMesh)if(O._multiDrawInstances!==null)qs("THREE.WebGLRenderer: renderMultiDrawInstances has been deprecated and will be removed in r184. Append to renderMultiDraw arguments and use indirection."),ut.renderMultiDrawInstances(O._multiDrawStarts,O._multiDrawCounts,O._multiDrawCount,O._multiDrawInstances);else if(He.get("WEBGL_multi_draw"))ut.renderMultiDraw(O._multiDrawStarts,O._multiDrawCounts,O._multiDrawCount);else{let Pe=O._multiDrawStarts,yt=O._multiDrawCounts,et=O._multiDrawCount,hn=me?T.get(me).bytesPerElement:1,Es=Me.get(V).currentProgram.getUniforms();for(let un=0;un<et;un++)Es.setValue(I,"_gl_DrawID",un),ut.render(Pe[un]/hn,yt[un])}else if(O.isInstancedMesh)ut.renderInstances(Je,Mt,O.count);else if(z.isInstancedBufferGeometry){let Pe=z._maxInstanceCount!==void 0?z._maxInstanceCount:1/0,yt=Math.min(z.instanceCount,Pe);ut.renderInstances(Je,Mt,yt)}else ut.render(Je,Mt)};function dt(v,U,z){v.transparent===!0&&v.side===Kt&&v.forceSinglePass===!1?(v.side=en,v.needsUpdate=!0,vo(v,U,z),v.side=mn,v.needsUpdate=!0,vo(v,U,z),v.side=Kt):vo(v,U,z)}this.compile=function(v,U,z=null){z===null&&(z=v),p=Ae.get(z),p.init(U),E.push(p),z.traverseVisible(function(O){O.isLight&&O.layers.test(U.layers)&&(p.pushLight(O),O.castShadow&&p.pushShadow(O))}),v!==z&&v.traverseVisible(function(O){O.isLight&&O.layers.test(U.layers)&&(p.pushLight(O),O.castShadow&&p.pushShadow(O))}),p.setupLights();let V=new Set;return v.traverse(function(O){if(!(O.isMesh||O.isPoints||O.isLine||O.isSprite))return;let ie=O.material;if(ie)if(Array.isArray(ie))for(let fe=0;fe<ie.length;fe++){let xe=ie[fe];dt(xe,z,O),V.add(xe)}else dt(ie,z,O),V.add(ie)}),p=E.pop(),V},this.compileAsync=function(v,U,z=null){let V=this.compile(v,U,z);return new Promise(O=>{function ie(){if(V.forEach(function(fe){Me.get(fe).currentProgram.isReady()&&V.delete(fe)}),V.size===0){O(v);return}setTimeout(ie,10)}He.get("KHR_parallel_shader_compile")!==null?ie():setTimeout(ie,10)})};let st=null;function oi(v){st&&st(v)}function qn(){Ki.stop()}function lu(){Ki.start()}let Ki=new xf;Ki.setAnimationLoop(oi),typeof self<"u"&&Ki.setContext(self),this.setAnimationLoop=function(v){st=v,re.setAnimationLoop(v),v===null?Ki.stop():Ki.start()},re.addEventListener("sessionstart",qn),re.addEventListener("sessionend",lu),this.render=function(v,U){if(U!==void 0&&U.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(w===!0)return;if(v.matrixWorldAutoUpdate===!0&&v.updateMatrixWorld(),U.parent===null&&U.matrixWorldAutoUpdate===!0&&U.updateMatrixWorld(),re.enabled===!0&&re.isPresenting===!0&&(re.cameraAutoUpdate===!0&&re.updateCamera(U),U=re.getCamera()),v.isScene===!0&&v.onBeforeRender(y,v,U,N),p=Ae.get(v,E.length),p.init(U),E.push(p),te.multiplyMatrices(U.projectionMatrix,U.matrixWorldInverse),it.setFromProjectionMatrix(te,Un,U.reversedDepth),K=this.localClippingEnabled,Ye=oe.init(this.clippingPlanes,K),m=X.get(v,S.length),m.init(),S.push(m),re.enabled===!0&&re.isPresenting===!0){let ie=y.xr.getDepthSensingMesh();ie!==null&&Mc(ie,U,-1/0,y.sortObjects)}Mc(v,U,0,y.sortObjects),m.finish(),y.sortObjects===!0&&m.sort(Q,ae),Qe=re.enabled===!1||re.isPresenting===!1||re.hasDepthSensing()===!1,Qe&&Te.addToRenderList(m,v),this.info.render.frame++,Ye===!0&&oe.beginShadows();let z=p.state.shadowsArray;Se.render(z,v,U),Ye===!0&&oe.endShadows(),this.info.autoReset===!0&&this.info.reset();let V=m.opaque,O=m.transmissive;if(p.setupLights(),U.isArrayCamera){let ie=U.cameras;if(O.length>0)for(let fe=0,xe=ie.length;fe<xe;fe++){let me=ie[fe];uu(V,O,v,me)}Qe&&Te.render(v);for(let fe=0,xe=ie.length;fe<xe;fe++){let me=ie[fe];hu(m,v,me,me.viewport)}}else O.length>0&&uu(V,O,v,U),Qe&&Te.render(v),hu(m,v,U);N!==null&&R===0&&(We.updateMultisampleRenderTarget(N),We.updateRenderTargetMipmap(N)),v.isScene===!0&&v.onAfterRender(y,v,U),de.resetDefaultState(),M=-1,b=null,E.pop(),E.length>0?(p=E[E.length-1],Ye===!0&&oe.setGlobalState(y.clippingPlanes,p.state.camera)):p=null,S.pop(),S.length>0?m=S[S.length-1]:m=null};function Mc(v,U,z,V){if(v.visible===!1)return;if(v.layers.test(U.layers)){if(v.isGroup)z=v.renderOrder;else if(v.isLOD)v.autoUpdate===!0&&v.update(U);else if(v.isLight)p.pushLight(v),v.castShadow&&p.pushShadow(v);else if(v.isSprite){if(!v.frustumCulled||it.intersectsSprite(v)){V&&ke.setFromMatrixPosition(v.matrixWorld).applyMatrix4(te);let fe=k.update(v),xe=v.material;xe.visible&&m.push(v,fe,xe,z,ke.z,null)}}else if((v.isMesh||v.isLine||v.isPoints)&&(!v.frustumCulled||it.intersectsObject(v))){let fe=k.update(v),xe=v.material;if(V&&(v.boundingSphere!==void 0?(v.boundingSphere===null&&v.computeBoundingSphere(),ke.copy(v.boundingSphere.center)):(fe.boundingSphere===null&&fe.computeBoundingSphere(),ke.copy(fe.boundingSphere.center)),ke.applyMatrix4(v.matrixWorld).applyMatrix4(te)),Array.isArray(xe)){let me=fe.groups;for(let Le=0,Oe=me.length;Le<Oe;Le++){let Ie=me[Le],Je=xe[Ie.materialIndex];Je&&Je.visible&&m.push(v,fe,Je,z,ke.z,Ie)}}else xe.visible&&m.push(v,fe,xe,z,ke.z,null)}}let ie=v.children;for(let fe=0,xe=ie.length;fe<xe;fe++)Mc(ie[fe],U,z,V)}function hu(v,U,z,V){let O=v.opaque,ie=v.transmissive,fe=v.transparent;p.setupLightsView(z),Ye===!0&&oe.setGlobalState(y.clippingPlanes,z),V&&be.viewport(P.copy(V)),O.length>0&&yo(O,U,z),ie.length>0&&yo(ie,U,z),fe.length>0&&yo(fe,U,z),be.buffers.depth.setTest(!0),be.buffers.depth.setMask(!0),be.buffers.color.setMask(!0),be.setPolygonOffset(!1)}function uu(v,U,z,V){if((z.isScene===!0?z.overrideMaterial:null)!==null)return;p.state.transmissionRenderTarget[V.id]===void 0&&(p.state.transmissionRenderTarget[V.id]=new Zn(1,1,{generateMipmaps:!0,type:He.has("EXT_color_buffer_half_float")||He.has("EXT_color_buffer_float")?rr:zn,minFilter:gn,samples:4,stencilBuffer:r,resolveDepthBuffer:!1,resolveStencilBuffer:!1,colorSpace:$e.workingColorSpace}));let ie=p.state.transmissionRenderTarget[V.id],fe=V.viewport||P;ie.setSize(fe.z*y.transmissionResolutionScale,fe.w*y.transmissionResolutionScale);let xe=y.getRenderTarget(),me=y.getActiveCubeFace(),Le=y.getActiveMipmapLevel();y.setRenderTarget(ie),y.getClearColor(q),W=y.getClearAlpha(),W<1&&y.setClearColor(16777215,.5),y.clear(),Qe&&Te.render(z);let Oe=y.toneMapping;y.toneMapping=Si;let Ie=V.viewport;if(V.viewport!==void 0&&(V.viewport=void 0),p.setupLightsView(V),Ye===!0&&oe.setGlobalState(y.clippingPlanes,V),yo(v,z,V),We.updateMultisampleRenderTarget(ie),We.updateRenderTargetMipmap(ie),He.has("WEBGL_multisampled_render_to_texture")===!1){let Je=!1;for(let ct=0,Mt=U.length;ct<Mt;ct++){let ft=U[ct],ut=ft.object,Pe=ft.geometry,yt=ft.material,et=ft.group;if(yt.side===Kt&&ut.layers.test(V.layers)){let hn=yt.side;yt.side=en,yt.needsUpdate=!0,du(ut,z,V,Pe,yt,et),yt.side=hn,yt.needsUpdate=!0,Je=!0}}Je===!0&&(We.updateMultisampleRenderTarget(ie),We.updateRenderTargetMipmap(ie))}y.setRenderTarget(xe,me,Le),y.setClearColor(q,W),Ie!==void 0&&(V.viewport=Ie),y.toneMapping=Oe}function yo(v,U,z){let V=U.isScene===!0?U.overrideMaterial:null;for(let O=0,ie=v.length;O<ie;O++){let fe=v[O],xe=fe.object,me=fe.geometry,Le=fe.group,Oe=fe.material;Oe.allowOverride===!0&&V!==null&&(Oe=V),xe.layers.test(z.layers)&&du(xe,U,z,me,Oe,Le)}}function du(v,U,z,V,O,ie){v.onBeforeRender(y,U,z,V,O,ie),v.modelViewMatrix.multiplyMatrices(z.matrixWorldInverse,v.matrixWorld),v.normalMatrix.getNormalMatrix(v.modelViewMatrix),O.onBeforeRender(y,U,z,V,v,ie),O.transparent===!0&&O.side===Kt&&O.forceSinglePass===!1?(O.side=en,O.needsUpdate=!0,y.renderBufferDirect(z,U,V,O,v,ie),O.side=mn,O.needsUpdate=!0,y.renderBufferDirect(z,U,V,O,v,ie),O.side=Kt):y.renderBufferDirect(z,U,V,O,v,ie),v.onAfterRender(y,U,z,V,O,ie)}function vo(v,U,z){U.isScene!==!0&&(U=Ce);let V=Me.get(v),O=p.state.lights,ie=p.state.shadowsArray,fe=O.state.version,xe=j.getParameters(v,O.state,ie,U,z),me=j.getProgramCacheKey(xe),Le=V.programs;V.environment=v.isMeshStandardMaterial?U.environment:null,V.fog=U.fog,V.envMap=(v.isMeshStandardMaterial?St:kt).get(v.envMap||V.environment),V.envMapRotation=V.environment!==null&&v.envMap===null?U.environmentRotation:v.envMapRotation,Le===void 0&&(v.addEventListener("dispose",Z),Le=new Map,V.programs=Le);let Oe=Le.get(me);if(Oe!==void 0){if(V.currentProgram===Oe&&V.lightsStateVersion===fe)return pu(v,xe),Oe}else xe.uniforms=j.getUniforms(v),v.onBeforeCompile(xe,y),Oe=j.acquireProgram(xe,me),Le.set(me,Oe),V.uniforms=xe.uniforms;let Ie=V.uniforms;return(!v.isShaderMaterial&&!v.isRawShaderMaterial||v.clipping===!0)&&(Ie.clippingPlanes=oe.uniform),pu(v,xe),V.needsLights=dp(v),V.lightsStateVersion=fe,V.needsLights&&(Ie.ambientLightColor.value=O.state.ambient,Ie.lightProbe.value=O.state.probe,Ie.directionalLights.value=O.state.directional,Ie.directionalLightShadows.value=O.state.directionalShadow,Ie.spotLights.value=O.state.spot,Ie.spotLightShadows.value=O.state.spotShadow,Ie.rectAreaLights.value=O.state.rectArea,Ie.ltc_1.value=O.state.rectAreaLTC1,Ie.ltc_2.value=O.state.rectAreaLTC2,Ie.pointLights.value=O.state.point,Ie.pointLightShadows.value=O.state.pointShadow,Ie.hemisphereLights.value=O.state.hemi,Ie.directionalShadowMap.value=O.state.directionalShadowMap,Ie.directionalShadowMatrix.value=O.state.directionalShadowMatrix,Ie.spotShadowMap.value=O.state.spotShadowMap,Ie.spotLightMatrix.value=O.state.spotLightMatrix,Ie.spotLightMap.value=O.state.spotLightMap,Ie.pointShadowMap.value=O.state.pointShadowMap,Ie.pointShadowMatrix.value=O.state.pointShadowMatrix),V.currentProgram=Oe,V.uniformsList=null,Oe}function fu(v){if(v.uniformsList===null){let U=v.currentProgram.getUniforms();v.uniformsList=ur.seqWithValue(U.seq,v.uniforms)}return v.uniformsList}function pu(v,U){let z=Me.get(v);z.outputColorSpace=U.outputColorSpace,z.batching=U.batching,z.batchingColor=U.batchingColor,z.instancing=U.instancing,z.instancingColor=U.instancingColor,z.instancingMorph=U.instancingMorph,z.skinning=U.skinning,z.morphTargets=U.morphTargets,z.morphNormals=U.morphNormals,z.morphColors=U.morphColors,z.morphTargetsCount=U.morphTargetsCount,z.numClippingPlanes=U.numClippingPlanes,z.numIntersection=U.numClipIntersection,z.vertexAlphas=U.vertexAlphas,z.vertexTangents=U.vertexTangents,z.toneMapping=U.toneMapping}function hp(v,U,z,V,O){U.isScene!==!0&&(U=Ce),We.resetTextureUnits();let ie=U.fog,fe=V.isMeshStandardMaterial?U.environment:null,xe=N===null?y.outputColorSpace:N.isXRRenderTarget===!0?N.texture.colorSpace:Vt,me=(V.isMeshStandardMaterial?St:kt).get(V.envMap||fe),Le=V.vertexColors===!0&&!!z.attributes.color&&z.attributes.color.itemSize===4,Oe=!!z.attributes.tangent&&(!!V.normalMap||V.anisotropy>0),Ie=!!z.morphAttributes.position,Je=!!z.morphAttributes.normal,ct=!!z.morphAttributes.color,Mt=Si;V.toneMapped&&(N===null||N.isXRRenderTarget===!0)&&(Mt=y.toneMapping);let ft=z.morphAttributes.position||z.morphAttributes.normal||z.morphAttributes.color,ut=ft!==void 0?ft.length:0,Pe=Me.get(V),yt=p.state.lights;if(Ye===!0&&(K===!0||v!==b)){let $t=v===b&&V.id===M;oe.setState(V,v,$t)}let et=!1;V.version===Pe.__version?(Pe.needsLights&&Pe.lightsStateVersion!==yt.state.version||Pe.outputColorSpace!==xe||O.isBatchedMesh&&Pe.batching===!1||!O.isBatchedMesh&&Pe.batching===!0||O.isBatchedMesh&&Pe.batchingColor===!0&&O.colorTexture===null||O.isBatchedMesh&&Pe.batchingColor===!1&&O.colorTexture!==null||O.isInstancedMesh&&Pe.instancing===!1||!O.isInstancedMesh&&Pe.instancing===!0||O.isSkinnedMesh&&Pe.skinning===!1||!O.isSkinnedMesh&&Pe.skinning===!0||O.isInstancedMesh&&Pe.instancingColor===!0&&O.instanceColor===null||O.isInstancedMesh&&Pe.instancingColor===!1&&O.instanceColor!==null||O.isInstancedMesh&&Pe.instancingMorph===!0&&O.morphTexture===null||O.isInstancedMesh&&Pe.instancingMorph===!1&&O.morphTexture!==null||Pe.envMap!==me||V.fog===!0&&Pe.fog!==ie||Pe.numClippingPlanes!==void 0&&(Pe.numClippingPlanes!==oe.numPlanes||Pe.numIntersection!==oe.numIntersection)||Pe.vertexAlphas!==Le||Pe.vertexTangents!==Oe||Pe.morphTargets!==Ie||Pe.morphNormals!==Je||Pe.morphColors!==ct||Pe.toneMapping!==Mt||Pe.morphTargetsCount!==ut)&&(et=!0):(et=!0,Pe.__version=V.version);let hn=Pe.currentProgram;et===!0&&(hn=vo(V,U,O));let Es=!1,un=!1,_r=!1,vt=hn.getUniforms(),vn=Pe.uniforms;if(be.useProgram(hn.program)&&(Es=!0,un=!0,_r=!0),V.id!==M&&(M=V.id,un=!0),Es||b!==v){be.buffers.depth.getReversed()&&v.reversedDepth!==!0&&(v._reversedDepth=!0,v.updateProjectionMatrix()),vt.setValue(I,"projectionMatrix",v.projectionMatrix),vt.setValue(I,"viewMatrix",v.matrixWorldInverse);let nn=vt.map.cameraPosition;nn!==void 0&&nn.setValue(I,_e.setFromMatrixPosition(v.matrixWorld)),Ue.logarithmicDepthBuffer&&vt.setValue(I,"logDepthBufFC",2/(Math.log(v.far+1)/Math.LN2)),(V.isMeshPhongMaterial||V.isMeshToonMaterial||V.isMeshLambertMaterial||V.isMeshBasicMaterial||V.isMeshStandardMaterial||V.isShaderMaterial)&&vt.setValue(I,"isOrthographic",v.isOrthographicCamera===!0),b!==v&&(b=v,un=!0,_r=!0)}if(O.isSkinnedMesh){vt.setOptional(I,O,"bindMatrix"),vt.setOptional(I,O,"bindMatrixInverse");let $t=O.skeleton;$t&&($t.boneTexture===null&&$t.computeBoneTexture(),vt.setValue(I,"boneTexture",$t.boneTexture,We))}O.isBatchedMesh&&(vt.setOptional(I,O,"batchingTexture"),vt.setValue(I,"batchingTexture",O._matricesTexture,We),vt.setOptional(I,O,"batchingIdTexture"),vt.setValue(I,"batchingIdTexture",O._indirectTexture,We),vt.setOptional(I,O,"batchingColorTexture"),O._colorsTexture!==null&&vt.setValue(I,"batchingColorTexture",O._colorsTexture,We));let bn=z.morphAttributes;if((bn.position!==void 0||bn.normal!==void 0||bn.color!==void 0)&&se.update(O,z,hn),(un||Pe.receiveShadow!==O.receiveShadow)&&(Pe.receiveShadow=O.receiveShadow,vt.setValue(I,"receiveShadow",O.receiveShadow)),V.isMeshGouraudMaterial&&V.envMap!==null&&(vn.envMap.value=me,vn.flipEnvMap.value=me.isCubeTexture&&me.isRenderTargetTexture===!1?-1:1),V.isMeshStandardMaterial&&V.envMap===null&&U.environment!==null&&(vn.envMapIntensity.value=U.environmentIntensity),un&&(vt.setValue(I,"toneMappingExposure",y.toneMappingExposure),Pe.needsLights&&up(vn,_r),ie&&V.fog===!0&&ee.refreshFogUniforms(vn,ie),ee.refreshMaterialUniforms(vn,V,D,B,p.state.transmissionRenderTarget[v.id]),ur.upload(I,fu(Pe),vn,We)),V.isShaderMaterial&&V.uniformsNeedUpdate===!0&&(ur.upload(I,fu(Pe),vn,We),V.uniformsNeedUpdate=!1),V.isSpriteMaterial&&vt.setValue(I,"center",O.center),vt.setValue(I,"modelViewMatrix",O.modelViewMatrix),vt.setValue(I,"normalMatrix",O.normalMatrix),vt.setValue(I,"modelMatrix",O.matrixWorld),V.isShaderMaterial||V.isRawShaderMaterial){let $t=V.uniformsGroups;for(let nn=0,Sc=$t.length;nn<Sc;nn++){let Zi=$t[nn];Ge.update(Zi,hn),Ge.bind(Zi,hn)}}return hn}function up(v,U){v.ambientLightColor.needsUpdate=U,v.lightProbe.needsUpdate=U,v.directionalLights.needsUpdate=U,v.directionalLightShadows.needsUpdate=U,v.pointLights.needsUpdate=U,v.pointLightShadows.needsUpdate=U,v.spotLights.needsUpdate=U,v.spotLightShadows.needsUpdate=U,v.rectAreaLights.needsUpdate=U,v.hemisphereLights.needsUpdate=U}function dp(v){return v.isMeshLambertMaterial||v.isMeshToonMaterial||v.isMeshPhongMaterial||v.isMeshStandardMaterial||v.isShadowMaterial||v.isShaderMaterial&&v.lights===!0}this.getActiveCubeFace=function(){return C},this.getActiveMipmapLevel=function(){return R},this.getRenderTarget=function(){return N},this.setRenderTargetTextures=function(v,U,z){let V=Me.get(v);V.__autoAllocateDepthBuffer=v.resolveDepthBuffer===!1,V.__autoAllocateDepthBuffer===!1&&(V.__useRenderToTexture=!1),Me.get(v.texture).__webglTexture=U,Me.get(v.depthTexture).__webglTexture=V.__autoAllocateDepthBuffer?void 0:z,V.__hasExternalTextures=!0},this.setRenderTargetFramebuffer=function(v,U){let z=Me.get(v);z.__webglFramebuffer=U,z.__useDefaultFramebuffer=U===void 0};let fp=I.createFramebuffer();this.setRenderTarget=function(v,U=0,z=0){N=v,C=U,R=z;let V=!0,O=null,ie=!1,fe=!1;if(v){let me=Me.get(v);if(me.__useDefaultFramebuffer!==void 0)be.bindFramebuffer(I.FRAMEBUFFER,null),V=!1;else if(me.__webglFramebuffer===void 0)We.setupRenderTarget(v);else if(me.__hasExternalTextures)We.rebindTextures(v,Me.get(v.texture).__webglTexture,Me.get(v.depthTexture).__webglTexture);else if(v.depthBuffer){let Ie=v.depthTexture;if(me.__boundDepthTexture!==Ie){if(Ie!==null&&Me.has(Ie)&&(v.width!==Ie.image.width||v.height!==Ie.image.height))throw new Error("WebGLRenderTarget: Attached DepthTexture is initialized to the incorrect size.");We.setupDepthRenderbuffer(v)}}let Le=v.texture;(Le.isData3DTexture||Le.isDataArrayTexture||Le.isCompressedArrayTexture)&&(fe=!0);let Oe=Me.get(v).__webglFramebuffer;v.isWebGLCubeRenderTarget?(Array.isArray(Oe[U])?O=Oe[U][z]:O=Oe[U],ie=!0):v.samples>0&&We.useMultisampledRTT(v)===!1?O=Me.get(v).__webglMultisampledFramebuffer:Array.isArray(Oe)?O=Oe[z]:O=Oe,P.copy(v.viewport),H.copy(v.scissor),G=v.scissorTest}else P.copy(ce).multiplyScalar(D).floor(),H.copy(ve).multiplyScalar(D).floor(),G=Ze;if(z!==0&&(O=fp),be.bindFramebuffer(I.FRAMEBUFFER,O)&&V&&be.drawBuffers(v,O),be.viewport(P),be.scissor(H),be.setScissorTest(G),ie){let me=Me.get(v.texture);I.framebufferTexture2D(I.FRAMEBUFFER,I.COLOR_ATTACHMENT0,I.TEXTURE_CUBE_MAP_POSITIVE_X+U,me.__webglTexture,z)}else if(fe){let me=U;for(let Le=0;Le<v.textures.length;Le++){let Oe=Me.get(v.textures[Le]);I.framebufferTextureLayer(I.FRAMEBUFFER,I.COLOR_ATTACHMENT0+Le,Oe.__webglTexture,z,me)}}else if(v!==null&&z!==0){let me=Me.get(v.texture);I.framebufferTexture2D(I.FRAMEBUFFER,I.COLOR_ATTACHMENT0,I.TEXTURE_2D,me.__webglTexture,z)}M=-1},this.readRenderTargetPixels=function(v,U,z,V,O,ie,fe,xe=0){if(!(v&&v.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let me=Me.get(v).__webglFramebuffer;if(v.isWebGLCubeRenderTarget&&fe!==void 0&&(me=me[fe]),me){be.bindFramebuffer(I.FRAMEBUFFER,me);try{let Le=v.textures[xe],Oe=Le.format,Ie=Le.type;if(!Ue.textureFormatReadable(Oe)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}if(!Ue.textureTypeReadable(Ie)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}U>=0&&U<=v.width-V&&z>=0&&z<=v.height-O&&(v.textures.length>1&&I.readBuffer(I.COLOR_ATTACHMENT0+xe),I.readPixels(U,z,V,O,we.convert(Oe),we.convert(Ie),ie))}finally{let Le=N!==null?Me.get(N).__webglFramebuffer:null;be.bindFramebuffer(I.FRAMEBUFFER,Le)}}},this.readRenderTargetPixelsAsync=async function(v,U,z,V,O,ie,fe,xe=0){if(!(v&&v.isWebGLRenderTarget))throw new Error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");let me=Me.get(v).__webglFramebuffer;if(v.isWebGLCubeRenderTarget&&fe!==void 0&&(me=me[fe]),me)if(U>=0&&U<=v.width-V&&z>=0&&z<=v.height-O){be.bindFramebuffer(I.FRAMEBUFFER,me);let Le=v.textures[xe],Oe=Le.format,Ie=Le.type;if(!Ue.textureFormatReadable(Oe))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in RGBA or implementation defined format.");if(!Ue.textureTypeReadable(Ie))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in UnsignedByteType or implementation defined type.");let Je=I.createBuffer();I.bindBuffer(I.PIXEL_PACK_BUFFER,Je),I.bufferData(I.PIXEL_PACK_BUFFER,ie.byteLength,I.STREAM_READ),v.textures.length>1&&I.readBuffer(I.COLOR_ATTACHMENT0+xe),I.readPixels(U,z,V,O,we.convert(Oe),we.convert(Ie),0);let ct=N!==null?Me.get(N).__webglFramebuffer:null;be.bindFramebuffer(I.FRAMEBUFFER,ct);let Mt=I.fenceSync(I.SYNC_GPU_COMMANDS_COMPLETE,0);return I.flush(),await Vd(I,Mt,4),I.bindBuffer(I.PIXEL_PACK_BUFFER,Je),I.getBufferSubData(I.PIXEL_PACK_BUFFER,0,ie),I.deleteBuffer(Je),I.deleteSync(Mt),ie}else throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: requested read bounds are out of range.")},this.copyFramebufferToTexture=function(v,U=null,z=0){let V=Math.pow(2,-z),O=Math.floor(v.image.width*V),ie=Math.floor(v.image.height*V),fe=U!==null?U.x:0,xe=U!==null?U.y:0;We.setTexture2D(v,0),I.copyTexSubImage2D(I.TEXTURE_2D,z,0,0,fe,xe,O,ie),be.unbindTexture()};let pp=I.createFramebuffer(),mp=I.createFramebuffer();this.copyTextureToTexture=function(v,U,z=null,V=null,O=0,ie=null){ie===null&&(O!==0?(qs("WebGLRenderer: copyTextureToTexture function signature has changed to support src and dst mipmap levels."),ie=O,O=0):ie=0);let fe,xe,me,Le,Oe,Ie,Je,ct,Mt,ft=v.isCompressedTexture?v.mipmaps[ie]:v.image;if(z!==null)fe=z.max.x-z.min.x,xe=z.max.y-z.min.y,me=z.isBox3?z.max.z-z.min.z:1,Le=z.min.x,Oe=z.min.y,Ie=z.isBox3?z.min.z:0;else{let bn=Math.pow(2,-O);fe=Math.floor(ft.width*bn),xe=Math.floor(ft.height*bn),v.isDataArrayTexture?me=ft.depth:v.isData3DTexture?me=Math.floor(ft.depth*bn):me=1,Le=0,Oe=0,Ie=0}V!==null?(Je=V.x,ct=V.y,Mt=V.z):(Je=0,ct=0,Mt=0);let ut=we.convert(U.format),Pe=we.convert(U.type),yt;U.isData3DTexture?(We.setTexture3D(U,0),yt=I.TEXTURE_3D):U.isDataArrayTexture||U.isCompressedArrayTexture?(We.setTexture2DArray(U,0),yt=I.TEXTURE_2D_ARRAY):(We.setTexture2D(U,0),yt=I.TEXTURE_2D),I.pixelStorei(I.UNPACK_FLIP_Y_WEBGL,U.flipY),I.pixelStorei(I.UNPACK_PREMULTIPLY_ALPHA_WEBGL,U.premultiplyAlpha),I.pixelStorei(I.UNPACK_ALIGNMENT,U.unpackAlignment);let et=I.getParameter(I.UNPACK_ROW_LENGTH),hn=I.getParameter(I.UNPACK_IMAGE_HEIGHT),Es=I.getParameter(I.UNPACK_SKIP_PIXELS),un=I.getParameter(I.UNPACK_SKIP_ROWS),_r=I.getParameter(I.UNPACK_SKIP_IMAGES);I.pixelStorei(I.UNPACK_ROW_LENGTH,ft.width),I.pixelStorei(I.UNPACK_IMAGE_HEIGHT,ft.height),I.pixelStorei(I.UNPACK_SKIP_PIXELS,Le),I.pixelStorei(I.UNPACK_SKIP_ROWS,Oe),I.pixelStorei(I.UNPACK_SKIP_IMAGES,Ie);let vt=v.isDataArrayTexture||v.isData3DTexture,vn=U.isDataArrayTexture||U.isData3DTexture;if(v.isDepthTexture){let bn=Me.get(v),$t=Me.get(U),nn=Me.get(bn.__renderTarget),Sc=Me.get($t.__renderTarget);be.bindFramebuffer(I.READ_FRAMEBUFFER,nn.__webglFramebuffer),be.bindFramebuffer(I.DRAW_FRAMEBUFFER,Sc.__webglFramebuffer);for(let Zi=0;Zi<me;Zi++)vt&&(I.framebufferTextureLayer(I.READ_FRAMEBUFFER,I.COLOR_ATTACHMENT0,Me.get(v).__webglTexture,O,Ie+Zi),I.framebufferTextureLayer(I.DRAW_FRAMEBUFFER,I.COLOR_ATTACHMENT0,Me.get(U).__webglTexture,ie,Mt+Zi)),I.blitFramebuffer(Le,Oe,fe,xe,Je,ct,fe,xe,I.DEPTH_BUFFER_BIT,I.NEAREST);be.bindFramebuffer(I.READ_FRAMEBUFFER,null),be.bindFramebuffer(I.DRAW_FRAMEBUFFER,null)}else if(O!==0||v.isRenderTargetTexture||Me.has(v)){let bn=Me.get(v),$t=Me.get(U);be.bindFramebuffer(I.READ_FRAMEBUFFER,pp),be.bindFramebuffer(I.DRAW_FRAMEBUFFER,mp);for(let nn=0;nn<me;nn++)vt?I.framebufferTextureLayer(I.READ_FRAMEBUFFER,I.COLOR_ATTACHMENT0,bn.__webglTexture,O,Ie+nn):I.framebufferTexture2D(I.READ_FRAMEBUFFER,I.COLOR_ATTACHMENT0,I.TEXTURE_2D,bn.__webglTexture,O),vn?I.framebufferTextureLayer(I.DRAW_FRAMEBUFFER,I.COLOR_ATTACHMENT0,$t.__webglTexture,ie,Mt+nn):I.framebufferTexture2D(I.DRAW_FRAMEBUFFER,I.COLOR_ATTACHMENT0,I.TEXTURE_2D,$t.__webglTexture,ie),O!==0?I.blitFramebuffer(Le,Oe,fe,xe,Je,ct,fe,xe,I.COLOR_BUFFER_BIT,I.NEAREST):vn?I.copyTexSubImage3D(yt,ie,Je,ct,Mt+nn,Le,Oe,fe,xe):I.copyTexSubImage2D(yt,ie,Je,ct,Le,Oe,fe,xe);be.bindFramebuffer(I.READ_FRAMEBUFFER,null),be.bindFramebuffer(I.DRAW_FRAMEBUFFER,null)}else vn?v.isDataTexture||v.isData3DTexture?I.texSubImage3D(yt,ie,Je,ct,Mt,fe,xe,me,ut,Pe,ft.data):U.isCompressedArrayTexture?I.compressedTexSubImage3D(yt,ie,Je,ct,Mt,fe,xe,me,ut,ft.data):I.texSubImage3D(yt,ie,Je,ct,Mt,fe,xe,me,ut,Pe,ft):v.isDataTexture?I.texSubImage2D(I.TEXTURE_2D,ie,Je,ct,fe,xe,ut,Pe,ft.data):v.isCompressedTexture?I.compressedTexSubImage2D(I.TEXTURE_2D,ie,Je,ct,ft.width,ft.height,ut,ft.data):I.texSubImage2D(I.TEXTURE_2D,ie,Je,ct,fe,xe,ut,Pe,ft);I.pixelStorei(I.UNPACK_ROW_LENGTH,et),I.pixelStorei(I.UNPACK_IMAGE_HEIGHT,hn),I.pixelStorei(I.UNPACK_SKIP_PIXELS,Es),I.pixelStorei(I.UNPACK_SKIP_ROWS,un),I.pixelStorei(I.UNPACK_SKIP_IMAGES,_r),ie===0&&U.generateMipmaps&&I.generateMipmap(yt),be.unbindTexture()},this.initRenderTarget=function(v){Me.get(v).__webglFramebuffer===void 0&&We.setupRenderTarget(v)},this.initTexture=function(v){v.isCubeTexture?We.setTextureCube(v,0):v.isData3DTexture?We.setTexture3D(v,0):v.isDataArrayTexture||v.isCompressedArrayTexture?We.setTexture2DArray(v,0):We.setTexture2D(v,0),be.unbindTexture()},this.resetState=function(){C=0,R=0,N=null,be.reset(),de.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return Un}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(e){this._outputColorSpace=e;let t=this.getContext();t.drawingBufferColorSpace=$e._getDrawingBufferColorSpace(e),t.unpackColorSpace=$e._getUnpackColorSpace()}};var Sf={type:"change"},Jl={type:"start"},Tf={type:"end"},uc=new Jn,Ef=new Sn,G0=Math.cos(70*Ti.DEG2RAD),Nt=new A,an=2*Math.PI,ht={NONE:-1,ROTATE:0,DOLLY:1,PAN:2,TOUCH_ROTATE:3,TOUCH_PAN:4,TOUCH_DOLLY_PAN:5,TOUCH_DOLLY_ROTATE:6},Zl=1e-6,dc=class extends so{constructor(e,t=null){super(e,t),this.state=ht.NONE,this.target=new A,this.cursor=new A,this.minDistance=0,this.maxDistance=1/0,this.minZoom=0,this.maxZoom=1/0,this.minTargetRadius=0,this.maxTargetRadius=1/0,this.minPolarAngle=0,this.maxPolarAngle=Math.PI,this.minAzimuthAngle=-1/0,this.maxAzimuthAngle=1/0,this.enableDamping=!1,this.dampingFactor=.05,this.enableZoom=!0,this.zoomSpeed=1,this.enableRotate=!0,this.rotateSpeed=1,this.keyRotateSpeed=1,this.enablePan=!0,this.panSpeed=1,this.screenSpacePanning=!0,this.keyPanSpeed=7,this.zoomToCursor=!1,this.autoRotate=!1,this.autoRotateSpeed=2,this.keys={LEFT:"ArrowLeft",UP:"ArrowUp",RIGHT:"ArrowRight",BOTTOM:"ArrowDown"},this.mouseButtons={LEFT:Vi.ROTATE,MIDDLE:Vi.DOLLY,RIGHT:Vi.PAN},this.touches={ONE:Gi.ROTATE,TWO:Gi.DOLLY_PAN},this.target0=this.target.clone(),this.position0=this.object.position.clone(),this.zoom0=this.object.zoom,this._domElementKeyEvents=null,this._lastPosition=new A,this._lastQuaternion=new It,this._lastTargetPosition=new A,this._quat=new It().setFromUnitVectors(e.up,new A(0,1,0)),this._quatInverse=this._quat.clone().invert(),this._spherical=new nr,this._sphericalDelta=new nr,this._scale=1,this._panOffset=new A,this._rotateStart=new Ee,this._rotateEnd=new Ee,this._rotateDelta=new Ee,this._panStart=new Ee,this._panEnd=new Ee,this._panDelta=new Ee,this._dollyStart=new Ee,this._dollyEnd=new Ee,this._dollyDelta=new Ee,this._dollyDirection=new A,this._mouse=new Ee,this._performCursorZoom=!1,this._pointers=[],this._pointerPositions={},this._controlActive=!1,this._onPointerMove=X0.bind(this),this._onPointerDown=W0.bind(this),this._onPointerUp=q0.bind(this),this._onContextMenu=Q0.bind(this),this._onMouseWheel=K0.bind(this),this._onKeyDown=Z0.bind(this),this._onTouchStart=J0.bind(this),this._onTouchMove=$0.bind(this),this._onMouseDown=Y0.bind(this),this._onMouseMove=j0.bind(this),this._interceptControlDown=ey.bind(this),this._interceptControlUp=ty.bind(this),this.domElement!==null&&this.connect(this.domElement),this.update()}connect(e){super.connect(e),this.domElement.addEventListener("pointerdown",this._onPointerDown),this.domElement.addEventListener("pointercancel",this._onPointerUp),this.domElement.addEventListener("contextmenu",this._onContextMenu),this.domElement.addEventListener("wheel",this._onMouseWheel,{passive:!1}),this.domElement.getRootNode().addEventListener("keydown",this._interceptControlDown,{passive:!0,capture:!0}),this.domElement.style.touchAction="none"}disconnect(){this.domElement.removeEventListener("pointerdown",this._onPointerDown),this.domElement.removeEventListener("pointermove",this._onPointerMove),this.domElement.removeEventListener("pointerup",this._onPointerUp),this.domElement.removeEventListener("pointercancel",this._onPointerUp),this.domElement.removeEventListener("wheel",this._onMouseWheel),this.domElement.removeEventListener("contextmenu",this._onContextMenu),this.stopListenToKeyEvents(),this.domElement.getRootNode().removeEventListener("keydown",this._interceptControlDown,{capture:!0}),this.domElement.style.touchAction="auto"}dispose(){this.disconnect()}getPolarAngle(){return this._spherical.phi}getAzimuthalAngle(){return this._spherical.theta}getDistance(){return this.object.position.distanceTo(this.target)}listenToKeyEvents(e){e.addEventListener("keydown",this._onKeyDown),this._domElementKeyEvents=e}stopListenToKeyEvents(){this._domElementKeyEvents!==null&&(this._domElementKeyEvents.removeEventListener("keydown",this._onKeyDown),this._domElementKeyEvents=null)}saveState(){this.target0.copy(this.target),this.position0.copy(this.object.position),this.zoom0=this.object.zoom}reset(){this.target.copy(this.target0),this.object.position.copy(this.position0),this.object.zoom=this.zoom0,this.object.updateProjectionMatrix(),this.dispatchEvent(Sf),this.update(),this.state=ht.NONE}update(e=null){let t=this.object.position;Nt.copy(t).sub(this.target),Nt.applyQuaternion(this._quat),this._spherical.setFromVector3(Nt),this.autoRotate&&this.state===ht.NONE&&this._rotateLeft(this._getAutoRotationAngle(e)),this.enableDamping?(this._spherical.theta+=this._sphericalDelta.theta*this.dampingFactor,this._spherical.phi+=this._sphericalDelta.phi*this.dampingFactor):(this._spherical.theta+=this._sphericalDelta.theta,this._spherical.phi+=this._sphericalDelta.phi);let n=this.minAzimuthAngle,i=this.maxAzimuthAngle;isFinite(n)&&isFinite(i)&&(n<-Math.PI?n+=an:n>Math.PI&&(n-=an),i<-Math.PI?i+=an:i>Math.PI&&(i-=an),n<=i?this._spherical.theta=Math.max(n,Math.min(i,this._spherical.theta)):this._spherical.theta=this._spherical.theta>(n+i)/2?Math.max(n,this._spherical.theta):Math.min(i,this._spherical.theta)),this._spherical.phi=Math.max(this.minPolarAngle,Math.min(this.maxPolarAngle,this._spherical.phi)),this._spherical.makeSafe(),this.enableDamping===!0?this.target.addScaledVector(this._panOffset,this.dampingFactor):this.target.add(this._panOffset),this.target.sub(this.cursor),this.target.clampLength(this.minTargetRadius,this.maxTargetRadius),this.target.add(this.cursor);let r=!1;if(this.zoomToCursor&&this._performCursorZoom||this.object.isOrthographicCamera)this._spherical.radius=this._clampDistance(this._spherical.radius);else{let o=this._spherical.radius;this._spherical.radius=this._clampDistance(this._spherical.radius*this._scale),r=o!=this._spherical.radius}if(Nt.setFromSpherical(this._spherical),Nt.applyQuaternion(this._quatInverse),t.copy(this.target).add(Nt),this.object.lookAt(this.target),this.enableDamping===!0?(this._sphericalDelta.theta*=1-this.dampingFactor,this._sphericalDelta.phi*=1-this.dampingFactor,this._panOffset.multiplyScalar(1-this.dampingFactor)):(this._sphericalDelta.set(0,0,0),this._panOffset.set(0,0,0)),this.zoomToCursor&&this._performCursorZoom){let o=null;if(this.object.isPerspectiveCamera){let a=Nt.length();o=this._clampDistance(a*this._scale);let c=a-o;this.object.position.addScaledVector(this._dollyDirection,c),this.object.updateMatrixWorld(),r=!!c}else if(this.object.isOrthographicCamera){let a=new A(this._mouse.x,this._mouse.y,0);a.unproject(this.object);let c=this.object.zoom;this.object.zoom=Math.max(this.minZoom,Math.min(this.maxZoom,this.object.zoom/this._scale)),this.object.updateProjectionMatrix(),r=c!==this.object.zoom;let l=new A(this._mouse.x,this._mouse.y,0);l.unproject(this.object),this.object.position.sub(l).add(a),this.object.updateMatrixWorld(),o=Nt.length()}else console.warn("WARNING: OrbitControls.js encountered an unknown camera type - zoom to cursor disabled."),this.zoomToCursor=!1;o!==null&&(this.screenSpacePanning?this.target.set(0,0,-1).transformDirection(this.object.matrix).multiplyScalar(o).add(this.object.position):(uc.origin.copy(this.object.position),uc.direction.set(0,0,-1).transformDirection(this.object.matrix),Math.abs(this.object.up.dot(uc.direction))<G0?this.object.lookAt(this.target):(Ef.setFromNormalAndCoplanarPoint(this.object.up,this.target),uc.intersectPlane(Ef,this.target))))}else if(this.object.isOrthographicCamera){let o=this.object.zoom;this.object.zoom=Math.max(this.minZoom,Math.min(this.maxZoom,this.object.zoom/this._scale)),o!==this.object.zoom&&(this.object.updateProjectionMatrix(),r=!0)}return this._scale=1,this._performCursorZoom=!1,r||this._lastPosition.distanceToSquared(this.object.position)>Zl||8*(1-this._lastQuaternion.dot(this.object.quaternion))>Zl||this._lastTargetPosition.distanceToSquared(this.target)>Zl?(this.dispatchEvent(Sf),this._lastPosition.copy(this.object.position),this._lastQuaternion.copy(this.object.quaternion),this._lastTargetPosition.copy(this.target),!0):!1}_getAutoRotationAngle(e){return e!==null?an/60*this.autoRotateSpeed*e:an/60/60*this.autoRotateSpeed}_getZoomScale(e){let t=Math.abs(e*.01);return Math.pow(.95,this.zoomSpeed*t)}_rotateLeft(e){this._sphericalDelta.theta-=e}_rotateUp(e){this._sphericalDelta.phi-=e}_panLeft(e,t){Nt.setFromMatrixColumn(t,0),Nt.multiplyScalar(-e),this._panOffset.add(Nt)}_panUp(e,t){this.screenSpacePanning===!0?Nt.setFromMatrixColumn(t,1):(Nt.setFromMatrixColumn(t,0),Nt.crossVectors(this.object.up,Nt)),Nt.multiplyScalar(e),this._panOffset.add(Nt)}_pan(e,t){let n=this.domElement;if(this.object.isPerspectiveCamera){let i=this.object.position;Nt.copy(i).sub(this.target);let r=Nt.length();r*=Math.tan(this.object.fov/2*Math.PI/180),this._panLeft(2*e*r/n.clientHeight,this.object.matrix),this._panUp(2*t*r/n.clientHeight,this.object.matrix)}else this.object.isOrthographicCamera?(this._panLeft(e*(this.object.right-this.object.left)/this.object.zoom/n.clientWidth,this.object.matrix),this._panUp(t*(this.object.top-this.object.bottom)/this.object.zoom/n.clientHeight,this.object.matrix)):(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - pan disabled."),this.enablePan=!1)}_dollyOut(e){this.object.isPerspectiveCamera||this.object.isOrthographicCamera?this._scale/=e:(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),this.enableZoom=!1)}_dollyIn(e){this.object.isPerspectiveCamera||this.object.isOrthographicCamera?this._scale*=e:(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),this.enableZoom=!1)}_updateZoomParameters(e,t){if(!this.zoomToCursor)return;this._performCursorZoom=!0;let n=this.domElement.getBoundingClientRect(),i=e-n.left,r=t-n.top,o=n.width,a=n.height;this._mouse.x=i/o*2-1,this._mouse.y=-(r/a)*2+1,this._dollyDirection.set(this._mouse.x,this._mouse.y,1).unproject(this.object).sub(this.object.position).normalize()}_clampDistance(e){return Math.max(this.minDistance,Math.min(this.maxDistance,e))}_handleMouseDownRotate(e){this._rotateStart.set(e.clientX,e.clientY)}_handleMouseDownDolly(e){this._updateZoomParameters(e.clientX,e.clientX),this._dollyStart.set(e.clientX,e.clientY)}_handleMouseDownPan(e){this._panStart.set(e.clientX,e.clientY)}_handleMouseMoveRotate(e){this._rotateEnd.set(e.clientX,e.clientY),this._rotateDelta.subVectors(this._rotateEnd,this._rotateStart).multiplyScalar(this.rotateSpeed);let t=this.domElement;this._rotateLeft(an*this._rotateDelta.x/t.clientHeight),this._rotateUp(an*this._rotateDelta.y/t.clientHeight),this._rotateStart.copy(this._rotateEnd),this.update()}_handleMouseMoveDolly(e){this._dollyEnd.set(e.clientX,e.clientY),this._dollyDelta.subVectors(this._dollyEnd,this._dollyStart),this._dollyDelta.y>0?this._dollyOut(this._getZoomScale(this._dollyDelta.y)):this._dollyDelta.y<0&&this._dollyIn(this._getZoomScale(this._dollyDelta.y)),this._dollyStart.copy(this._dollyEnd),this.update()}_handleMouseMovePan(e){this._panEnd.set(e.clientX,e.clientY),this._panDelta.subVectors(this._panEnd,this._panStart).multiplyScalar(this.panSpeed),this._pan(this._panDelta.x,this._panDelta.y),this._panStart.copy(this._panEnd),this.update()}_handleMouseWheel(e){this._updateZoomParameters(e.clientX,e.clientY),e.deltaY<0?this._dollyIn(this._getZoomScale(e.deltaY)):e.deltaY>0&&this._dollyOut(this._getZoomScale(e.deltaY)),this.update()}_handleKeyDown(e){let t=!1;switch(e.code){case this.keys.UP:e.ctrlKey||e.metaKey||e.shiftKey?this.enableRotate&&this._rotateUp(an*this.keyRotateSpeed/this.domElement.clientHeight):this.enablePan&&this._pan(0,this.keyPanSpeed),t=!0;break;case this.keys.BOTTOM:e.ctrlKey||e.metaKey||e.shiftKey?this.enableRotate&&this._rotateUp(-an*this.keyRotateSpeed/this.domElement.clientHeight):this.enablePan&&this._pan(0,-this.keyPanSpeed),t=!0;break;case this.keys.LEFT:e.ctrlKey||e.metaKey||e.shiftKey?this.enableRotate&&this._rotateLeft(an*this.keyRotateSpeed/this.domElement.clientHeight):this.enablePan&&this._pan(this.keyPanSpeed,0),t=!0;break;case this.keys.RIGHT:e.ctrlKey||e.metaKey||e.shiftKey?this.enableRotate&&this._rotateLeft(-an*this.keyRotateSpeed/this.domElement.clientHeight):this.enablePan&&this._pan(-this.keyPanSpeed,0),t=!0;break}t&&(e.preventDefault(),this.update())}_handleTouchStartRotate(e){if(this._pointers.length===1)this._rotateStart.set(e.pageX,e.pageY);else{let t=this._getSecondPointerPosition(e),n=.5*(e.pageX+t.x),i=.5*(e.pageY+t.y);this._rotateStart.set(n,i)}}_handleTouchStartPan(e){if(this._pointers.length===1)this._panStart.set(e.pageX,e.pageY);else{let t=this._getSecondPointerPosition(e),n=.5*(e.pageX+t.x),i=.5*(e.pageY+t.y);this._panStart.set(n,i)}}_handleTouchStartDolly(e){let t=this._getSecondPointerPosition(e),n=e.pageX-t.x,i=e.pageY-t.y,r=Math.sqrt(n*n+i*i);this._dollyStart.set(0,r)}_handleTouchStartDollyPan(e){this.enableZoom&&this._handleTouchStartDolly(e),this.enablePan&&this._handleTouchStartPan(e)}_handleTouchStartDollyRotate(e){this.enableZoom&&this._handleTouchStartDolly(e),this.enableRotate&&this._handleTouchStartRotate(e)}_handleTouchMoveRotate(e){if(this._pointers.length==1)this._rotateEnd.set(e.pageX,e.pageY);else{let n=this._getSecondPointerPosition(e),i=.5*(e.pageX+n.x),r=.5*(e.pageY+n.y);this._rotateEnd.set(i,r)}this._rotateDelta.subVectors(this._rotateEnd,this._rotateStart).multiplyScalar(this.rotateSpeed);let t=this.domElement;this._rotateLeft(an*this._rotateDelta.x/t.clientHeight),this._rotateUp(an*this._rotateDelta.y/t.clientHeight),this._rotateStart.copy(this._rotateEnd)}_handleTouchMovePan(e){if(this._pointers.length===1)this._panEnd.set(e.pageX,e.pageY);else{let t=this._getSecondPointerPosition(e),n=.5*(e.pageX+t.x),i=.5*(e.pageY+t.y);this._panEnd.set(n,i)}this._panDelta.subVectors(this._panEnd,this._panStart).multiplyScalar(this.panSpeed),this._pan(this._panDelta.x,this._panDelta.y),this._panStart.copy(this._panEnd)}_handleTouchMoveDolly(e){let t=this._getSecondPointerPosition(e),n=e.pageX-t.x,i=e.pageY-t.y,r=Math.sqrt(n*n+i*i);this._dollyEnd.set(0,r),this._dollyDelta.set(0,Math.pow(this._dollyEnd.y/this._dollyStart.y,this.zoomSpeed)),this._dollyOut(this._dollyDelta.y),this._dollyStart.copy(this._dollyEnd);let o=(e.pageX+t.x)*.5,a=(e.pageY+t.y)*.5;this._updateZoomParameters(o,a)}_handleTouchMoveDollyPan(e){this.enableZoom&&this._handleTouchMoveDolly(e),this.enablePan&&this._handleTouchMovePan(e)}_handleTouchMoveDollyRotate(e){this.enableZoom&&this._handleTouchMoveDolly(e),this.enableRotate&&this._handleTouchMoveRotate(e)}_addPointer(e){this._pointers.push(e.pointerId)}_removePointer(e){delete this._pointerPositions[e.pointerId];for(let t=0;t<this._pointers.length;t++)if(this._pointers[t]==e.pointerId){this._pointers.splice(t,1);return}}_isTrackingPointer(e){for(let t=0;t<this._pointers.length;t++)if(this._pointers[t]==e.pointerId)return!0;return!1}_trackPointer(e){let t=this._pointerPositions[e.pointerId];t===void 0&&(t=new Ee,this._pointerPositions[e.pointerId]=t),t.set(e.pageX,e.pageY)}_getSecondPointerPosition(e){let t=e.pointerId===this._pointers[0]?this._pointers[1]:this._pointers[0];return this._pointerPositions[t]}_customWheelEvent(e){let t=e.deltaMode,n={clientX:e.clientX,clientY:e.clientY,deltaY:e.deltaY};switch(t){case 1:n.deltaY*=16;break;case 2:n.deltaY*=100;break}return e.ctrlKey&&!this._controlActive&&(n.deltaY*=10),n}};function W0(s){this.enabled!==!1&&(this._pointers.length===0&&(this.domElement.setPointerCapture(s.pointerId),this.domElement.addEventListener("pointermove",this._onPointerMove),this.domElement.addEventListener("pointerup",this._onPointerUp)),!this._isTrackingPointer(s)&&(this._addPointer(s),s.pointerType==="touch"?this._onTouchStart(s):this._onMouseDown(s)))}function X0(s){this.enabled!==!1&&(s.pointerType==="touch"?this._onTouchMove(s):this._onMouseMove(s))}function q0(s){switch(this._removePointer(s),this._pointers.length){case 0:this.domElement.releasePointerCapture(s.pointerId),this.domElement.removeEventListener("pointermove",this._onPointerMove),this.domElement.removeEventListener("pointerup",this._onPointerUp),this.dispatchEvent(Tf),this.state=ht.NONE;break;case 1:let e=this._pointers[0],t=this._pointerPositions[e];this._onTouchStart({pointerId:e,pageX:t.x,pageY:t.y});break}}function Y0(s){let e;switch(s.button){case 0:e=this.mouseButtons.LEFT;break;case 1:e=this.mouseButtons.MIDDLE;break;case 2:e=this.mouseButtons.RIGHT;break;default:e=-1}switch(e){case Vi.DOLLY:if(this.enableZoom===!1)return;this._handleMouseDownDolly(s),this.state=ht.DOLLY;break;case Vi.ROTATE:if(s.ctrlKey||s.metaKey||s.shiftKey){if(this.enablePan===!1)return;this._handleMouseDownPan(s),this.state=ht.PAN}else{if(this.enableRotate===!1)return;this._handleMouseDownRotate(s),this.state=ht.ROTATE}break;case Vi.PAN:if(s.ctrlKey||s.metaKey||s.shiftKey){if(this.enableRotate===!1)return;this._handleMouseDownRotate(s),this.state=ht.ROTATE}else{if(this.enablePan===!1)return;this._handleMouseDownPan(s),this.state=ht.PAN}break;default:this.state=ht.NONE}this.state!==ht.NONE&&this.dispatchEvent(Jl)}function j0(s){switch(this.state){case ht.ROTATE:if(this.enableRotate===!1)return;this._handleMouseMoveRotate(s);break;case ht.DOLLY:if(this.enableZoom===!1)return;this._handleMouseMoveDolly(s);break;case ht.PAN:if(this.enablePan===!1)return;this._handleMouseMovePan(s);break}}function K0(s){this.enabled===!1||this.enableZoom===!1||this.state!==ht.NONE||(s.preventDefault(),this.dispatchEvent(Jl),this._handleMouseWheel(this._customWheelEvent(s)),this.dispatchEvent(Tf))}function Z0(s){this.enabled!==!1&&this._handleKeyDown(s)}function J0(s){switch(this._trackPointer(s),this._pointers.length){case 1:switch(this.touches.ONE){case Gi.ROTATE:if(this.enableRotate===!1)return;this._handleTouchStartRotate(s),this.state=ht.TOUCH_ROTATE;break;case Gi.PAN:if(this.enablePan===!1)return;this._handleTouchStartPan(s),this.state=ht.TOUCH_PAN;break;default:this.state=ht.NONE}break;case 2:switch(this.touches.TWO){case Gi.DOLLY_PAN:if(this.enableZoom===!1&&this.enablePan===!1)return;this._handleTouchStartDollyPan(s),this.state=ht.TOUCH_DOLLY_PAN;break;case Gi.DOLLY_ROTATE:if(this.enableZoom===!1&&this.enableRotate===!1)return;this._handleTouchStartDollyRotate(s),this.state=ht.TOUCH_DOLLY_ROTATE;break;default:this.state=ht.NONE}break;default:this.state=ht.NONE}this.state!==ht.NONE&&this.dispatchEvent(Jl)}function $0(s){switch(this._trackPointer(s),this.state){case ht.TOUCH_ROTATE:if(this.enableRotate===!1)return;this._handleTouchMoveRotate(s),this.update();break;case ht.TOUCH_PAN:if(this.enablePan===!1)return;this._handleTouchMovePan(s),this.update();break;case ht.TOUCH_DOLLY_PAN:if(this.enableZoom===!1&&this.enablePan===!1)return;this._handleTouchMoveDollyPan(s),this.update();break;case ht.TOUCH_DOLLY_ROTATE:if(this.enableZoom===!1&&this.enableRotate===!1)return;this._handleTouchMoveDollyRotate(s),this.update();break;default:this.state=ht.NONE}}function Q0(s){this.enabled!==!1&&s.preventDefault()}function ey(s){s.key==="Control"&&(this._controlActive=!0,this.domElement.getRootNode().addEventListener("keyup",this._interceptControlUp,{passive:!0,capture:!0}))}function ty(s){s.key==="Control"&&(this._controlActive=!1,this.domElement.getRootNode().removeEventListener("keyup",this._interceptControlUp,{passive:!0,capture:!0}))}var wf={POSITION:["byte","byte normalized","unsigned byte","unsigned byte normalized","short","short normalized","unsigned short","unsigned short normalized"],NORMAL:["byte normalized","short normalized"],TANGENT:["byte normalized","short normalized"],TEXCOORD:["byte","byte normalized","unsigned byte","short","short normalized","unsigned short"]},xs=class{constructor(){this.textureUtils=null,this.pluginCallbacks=[],this.register(function(e){return new th(e)}),this.register(function(e){return new nh(e)}),this.register(function(e){return new oh(e)}),this.register(function(e){return new ah(e)}),this.register(function(e){return new ch(e)}),this.register(function(e){return new lh(e)}),this.register(function(e){return new ih(e)}),this.register(function(e){return new sh(e)}),this.register(function(e){return new rh(e)}),this.register(function(e){return new hh(e)}),this.register(function(e){return new uh(e)}),this.register(function(e){return new dh(e)}),this.register(function(e){return new fh(e)}),this.register(function(e){return new ph(e)})}register(e){return this.pluginCallbacks.indexOf(e)===-1&&this.pluginCallbacks.push(e),this}unregister(e){return this.pluginCallbacks.indexOf(e)!==-1&&this.pluginCallbacks.splice(this.pluginCallbacks.indexOf(e),1),this}setTextureUtils(e){return this.textureUtils=e,this}parse(e,t,n,i){let r=new eh,o=[];for(let a=0,c=this.pluginCallbacks.length;a<c;a++)o.push(this.pluginCallbacks[a](r));r.setPlugins(o),r.setTextureUtils(this.textureUtils),r.writeAsync(e,t,i).catch(n)}parseAsync(e,t){let n=this;return new Promise(function(i,r){n.parse(e,i,r,t)})}},je={POINTS:0,LINES:1,LINE_LOOP:2,LINE_STRIP:3,TRIANGLES:4,TRIANGLE_STRIP:5,TRIANGLE_FAN:6,BYTE:5120,UNSIGNED_BYTE:5121,SHORT:5122,UNSIGNED_SHORT:5123,INT:5124,UNSIGNED_INT:5125,FLOAT:5126,ARRAY_BUFFER:34962,ELEMENT_ARRAY_BUFFER:34963,NEAREST:9728,LINEAR:9729,NEAREST_MIPMAP_NEAREST:9984,LINEAR_MIPMAP_NEAREST:9985,NEAREST_MIPMAP_LINEAR:9986,LINEAR_MIPMAP_LINEAR:9987,CLAMP_TO_EDGE:33071,MIRRORED_REPEAT:33648,REPEAT:10497},$l="KHR_mesh_quantization",_n={};_n[Ct]=je.NEAREST;_n[ir]=je.NEAREST_MIPMAP_NEAREST;_n[Ei]=je.NEAREST_MIPMAP_LINEAR;_n[Ht]=je.LINEAR;_n[Wi]=je.LINEAR_MIPMAP_NEAREST;_n[gn]=je.LINEAR_MIPMAP_LINEAR;_n[En]=je.CLAMP_TO_EDGE;_n[jn]=je.REPEAT;_n[Ui]=je.MIRRORED_REPEAT;var Af={scale:"scale",position:"translation",quaternion:"rotation",morphTargetInfluences:"weights"},ny=new Re,Rf=12,iy=1179937895,sy=2,Cf=8,ry=1313821514,oy=5130562;function po(s,e){return s.length===e.length&&s.every(function(t,n){return t===e[n]})}function ay(s){return new TextEncoder().encode(s).buffer}function cy(s){return po(s.elements,[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1])}function ly(s,e,t){let n={min:new Array(s.itemSize).fill(Number.POSITIVE_INFINITY),max:new Array(s.itemSize).fill(Number.NEGATIVE_INFINITY)};for(let i=e;i<e+t;i++)for(let r=0;r<s.itemSize;r++){let o;s.itemSize>4?o=s.array[i*s.itemSize+r]:(r===0?o=s.getX(i):r===1?o=s.getY(i):r===2?o=s.getZ(i):r===3&&(o=s.getW(i)),s.normalized===!0&&(o=Ti.normalize(o,s.array))),n.min[r]=Math.min(n.min[r],o),n.max[r]=Math.max(n.max[r],o)}return n}function Pf(s){return Math.ceil(s/4)*4}function Ql(s,e=0){let t=Pf(s.byteLength);if(t!==s.byteLength){let n=new Uint8Array(t);if(n.set(new Uint8Array(s)),e!==0)for(let i=s.byteLength;i<t;i++)n[i]=e;return n.buffer}return s}function If(){return typeof document>"u"&&typeof OffscreenCanvas<"u"?new OffscreenCanvas(1,1):document.createElement("canvas")}function hy(s,e){if(typeof OffscreenCanvas<"u"&&s instanceof OffscreenCanvas){let t;return e==="image/jpeg"?t=.92:e==="image/webp"&&(t=.8),s.convertToBlob({type:e,quality:t})}else return new Promise(t=>s.toBlob(t,e))}var eh=class{constructor(){this.plugins=[],this.options={},this.pending=[],this.buffers=[],this.byteOffset=0,this.buffers=[],this.nodeMap=new Map,this.skins=[],this.extensionsUsed={},this.extensionsRequired={},this.uids=new Map,this.uid=0,this.json={asset:{version:"2.0",generator:"THREE.GLTFExporter r180"}},this.cache={meshes:new Map,attributes:new Map,attributesNormalized:new Map,materials:new Map,textures:new Map,images:new Map},this.textureUtils=null}setPlugins(e){this.plugins=e}setTextureUtils(e){this.textureUtils=e}async writeAsync(e,t,n={}){this.options=Object.assign({binary:!1,trs:!1,onlyVisible:!0,maxTextureSize:1/0,animations:[],includeCustomExtensions:!1},n),this.options.animations.length>0&&(this.options.trs=!0),await this.processInputAsync(e),await Promise.all(this.pending);let i=this,r=i.buffers,o=i.json;n=i.options;let a=i.extensionsUsed,c=i.extensionsRequired,l=new Blob(r,{type:"application/octet-stream"}),h=Object.keys(a),u=Object.keys(c);if(h.length>0&&(o.extensionsUsed=h),u.length>0&&(o.extensionsRequired=u),o.buffers&&o.buffers.length>0&&(o.buffers[0].byteLength=l.size),n.binary===!0){let d=new FileReader;d.readAsArrayBuffer(l),d.onloadend=function(){let f=Ql(d.result),g=new DataView(new ArrayBuffer(Cf));g.setUint32(0,f.byteLength,!0),g.setUint32(4,oy,!0);let _=Ql(ay(JSON.stringify(o)),32),m=new DataView(new ArrayBuffer(Cf));m.setUint32(0,_.byteLength,!0),m.setUint32(4,ry,!0);let p=new ArrayBuffer(Rf),S=new DataView(p);S.setUint32(0,iy,!0),S.setUint32(4,sy,!0);let E=Rf+m.byteLength+_.byteLength+g.byteLength+f.byteLength;S.setUint32(8,E,!0);let y=new Blob([p,m,_,g,f],{type:"application/octet-stream"}),w=new FileReader;w.readAsArrayBuffer(y),w.onloadend=function(){t(w.result)}}}else if(o.buffers&&o.buffers.length>0){let d=new FileReader;d.readAsDataURL(l),d.onloadend=function(){let f=d.result;o.buffers[0].uri=f,t(o)}}else t(o)}serializeUserData(e,t){if(Object.keys(e.userData).length===0)return;let n=this.options,i=this.extensionsUsed;try{let r=JSON.parse(JSON.stringify(e.userData));if(n.includeCustomExtensions&&r.gltfExtensions){t.extensions===void 0&&(t.extensions={});for(let o in r.gltfExtensions)t.extensions[o]=r.gltfExtensions[o],i[o]=!0;delete r.gltfExtensions}Object.keys(r).length>0&&(t.extras=r)}catch(r){console.warn("THREE.GLTFExporter: userData of '"+e.name+"' won't be serialized because of JSON.stringify error - "+r.message)}}getUID(e,t=!1){if(this.uids.has(e)===!1){let i=new Map;i.set(!0,this.uid++),i.set(!1,this.uid++),this.uids.set(e,i)}return this.uids.get(e).get(t)}isNormalizedNormalAttribute(e){if(this.cache.attributesNormalized.has(e))return!1;let n=new A;for(let i=0,r=e.count;i<r;i++)if(Math.abs(n.fromBufferAttribute(e,i).length()-1)>5e-4)return!1;return!0}createNormalizedNormalAttribute(e){let t=this.cache;if(t.attributesNormalized.has(e))return t.attributesNormalized.get(e);let n=e.clone(),i=new A;for(let r=0,o=n.count;r<o;r++)i.fromBufferAttribute(n,r),i.x===0&&i.y===0&&i.z===0?i.setX(1):i.normalize(),n.setXYZ(r,i.x,i.y,i.z);return t.attributesNormalized.set(e,n),n}applyTextureTransform(e,t){let n=!1,i={};(t.offset.x!==0||t.offset.y!==0)&&(i.offset=t.offset.toArray(),n=!0),t.rotation!==0&&(i.rotation=t.rotation,n=!0),(t.repeat.x!==1||t.repeat.y!==1)&&(i.scale=t.repeat.toArray(),n=!0),n&&(e.extensions=e.extensions||{},e.extensions.KHR_texture_transform=i,this.extensionsUsed.KHR_texture_transform=!0)}async buildMetalRoughTextureAsync(e,t){if(e===t)return e;function n(f){return f.colorSpace===_t?function(_){return _<.04045?_*.0773993808:Math.pow(_*.9478672986+.0521327014,2.4)}:function(_){return _}}e instanceof as&&(e=await this.decompressTextureAsync(e)),t instanceof as&&(t=await this.decompressTextureAsync(t));let i=e?e.image:null,r=t?t.image:null,o=Math.max(i?i.width:0,r?r.width:0),a=Math.max(i?i.height:0,r?r.height:0),c=If();c.width=o,c.height=a;let l=c.getContext("2d",{willReadFrequently:!0});l.fillStyle="#00ffff",l.fillRect(0,0,o,a);let h=l.getImageData(0,0,o,a);if(i){l.drawImage(i,0,0,o,a);let f=n(e),g=l.getImageData(0,0,o,a).data;for(let _=2;_<g.length;_+=4)h.data[_]=f(g[_]/256)*256}if(r){l.drawImage(r,0,0,o,a);let f=n(t),g=l.getImageData(0,0,o,a).data;for(let _=1;_<g.length;_+=4)h.data[_]=f(g[_]/256)*256}l.putImageData(h,0,0);let d=(e||t).clone();return d.source=new Oi(c),d.colorSpace=Hn,d.channel=(e||t).channel,e&&t&&e.channel!==t.channel&&console.warn("THREE.GLTFExporter: UV channels for metalnessMap and roughnessMap textures must match."),console.warn("THREE.GLTFExporter: Merged metalnessMap and roughnessMap textures."),d}async decompressTextureAsync(e,t=1/0){if(this.textureUtils===null)throw new Error("THREE.GLTFExporter: setTextureUtils() must be called to process compressed textures.");return await this.textureUtils.decompress(e,t)}processBuffer(e){let t=this.json,n=this.buffers;return t.buffers||(t.buffers=[{byteLength:0}]),n.push(e),0}processBufferView(e,t,n,i,r){let o=this.json;o.bufferViews||(o.bufferViews=[]);let a;switch(t){case je.BYTE:case je.UNSIGNED_BYTE:a=1;break;case je.SHORT:case je.UNSIGNED_SHORT:a=2;break;default:a=4}let c=e.itemSize*a;r===je.ARRAY_BUFFER&&(c=Math.ceil(c/4)*4);let l=Pf(i*c),h=new DataView(new ArrayBuffer(l)),u=0;for(let g=n;g<n+i;g++){for(let _=0;_<e.itemSize;_++){let m;e.itemSize>4?m=e.array[g*e.itemSize+_]:(_===0?m=e.getX(g):_===1?m=e.getY(g):_===2?m=e.getZ(g):_===3&&(m=e.getW(g)),e.normalized===!0&&(m=Ti.normalize(m,e.array))),t===je.FLOAT?h.setFloat32(u,m,!0):t===je.INT?h.setInt32(u,m,!0):t===je.UNSIGNED_INT?h.setUint32(u,m,!0):t===je.SHORT?h.setInt16(u,m,!0):t===je.UNSIGNED_SHORT?h.setUint16(u,m,!0):t===je.BYTE?h.setInt8(u,m):t===je.UNSIGNED_BYTE&&h.setUint8(u,m),u+=a}u%c!==0&&(u+=c-u%c)}let d={buffer:this.processBuffer(h.buffer),byteOffset:this.byteOffset,byteLength:l};return r!==void 0&&(d.target=r),r===je.ARRAY_BUFFER&&(d.byteStride=c),this.byteOffset+=l,o.bufferViews.push(d),{id:o.bufferViews.length-1,byteLength:0}}processBufferViewImage(e){let t=this,n=t.json;return n.bufferViews||(n.bufferViews=[]),new Promise(function(i){let r=new FileReader;r.readAsArrayBuffer(e),r.onloadend=function(){let o=Ql(r.result),a={buffer:t.processBuffer(o),byteOffset:t.byteOffset,byteLength:o.byteLength};t.byteOffset+=o.byteLength,i(n.bufferViews.push(a)-1)}})}processAccessor(e,t,n,i){let r=this.json,o={1:"SCALAR",2:"VEC2",3:"VEC3",4:"VEC4",9:"MAT3",16:"MAT4"},a;if(e.array.constructor===Float32Array)a=je.FLOAT;else if(e.array.constructor===Int32Array)a=je.INT;else if(e.array.constructor===Uint32Array)a=je.UNSIGNED_INT;else if(e.array.constructor===Int16Array)a=je.SHORT;else if(e.array.constructor===Uint16Array)a=je.UNSIGNED_SHORT;else if(e.array.constructor===Int8Array)a=je.BYTE;else if(e.array.constructor===Uint8Array)a=je.UNSIGNED_BYTE;else throw new Error("THREE.GLTFExporter: Unsupported bufferAttribute component type: "+e.array.constructor.name);if(n===void 0&&(n=0),(i===void 0||i===1/0)&&(i=e.count),i===0)return null;let c=ly(e,n,i),l;t!==void 0&&(l=e===t.index?je.ELEMENT_ARRAY_BUFFER:je.ARRAY_BUFFER);let h=this.processBufferView(e,a,n,i,l),u={bufferView:h.id,byteOffset:h.byteOffset,componentType:a,count:i,max:c.max,min:c.min,type:o[e.itemSize]};return e.normalized===!0&&(u.normalized=!0),r.accessors||(r.accessors=[]),r.accessors.push(u)-1}processImage(e,t,n,i="image/png"){if(e!==null){let r=this,o=r.cache,a=r.json,c=r.options,l=r.pending;o.images.has(e)||o.images.set(e,{});let h=o.images.get(e),u=i+":flipY/"+n.toString();if(h[u]!==void 0)return h[u];a.images||(a.images=[]);let d={mimeType:i},f=If();f.width=Math.min(e.width,c.maxTextureSize),f.height=Math.min(e.height,c.maxTextureSize);let g=f.getContext("2d",{willReadFrequently:!0});if(n===!0&&(g.translate(0,f.height),g.scale(1,-1)),e.data!==void 0){t!==tn&&console.error("GLTFExporter: Only RGBAFormat is supported.",t),(e.width>c.maxTextureSize||e.height>c.maxTextureSize)&&console.warn("GLTFExporter: Image size is bigger than maxTextureSize",e);let m=new Uint8ClampedArray(e.height*e.width*4);for(let p=0;p<m.length;p+=4)m[p+0]=e.data[p+0],m[p+1]=e.data[p+1],m[p+2]=e.data[p+2],m[p+3]=e.data[p+3];g.putImageData(new ImageData(m,e.width,e.height),0,0)}else if(typeof HTMLImageElement<"u"&&e instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&e instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&e instanceof ImageBitmap||typeof OffscreenCanvas<"u"&&e instanceof OffscreenCanvas)g.drawImage(e,0,0,f.width,f.height);else throw new Error("THREE.GLTFExporter: Invalid image type. Use HTMLImageElement, HTMLCanvasElement, ImageBitmap or OffscreenCanvas.");c.binary===!0?l.push(hy(f,i).then(m=>r.processBufferViewImage(m)).then(m=>{d.bufferView=m})):d.uri=Ys.getDataURL(f,i);let _=a.images.push(d)-1;return h[u]=_,_}else throw new Error("THREE.GLTFExporter: No valid image data found. Unable to process texture.")}processSampler(e){let t=this.json;t.samplers||(t.samplers=[]);let n={magFilter:_n[e.magFilter],minFilter:_n[e.minFilter],wrapS:_n[e.wrapS],wrapT:_n[e.wrapT]};return t.samplers.push(n)-1}async processTextureAsync(e){let n=this.options,i=this.cache,r=this.json;if(i.textures.has(e))return i.textures.get(e);r.textures||(r.textures=[]),e instanceof as&&(e=await this.decompressTextureAsync(e,n.maxTextureSize));let o=e.userData.mimeType;o==="image/webp"&&(o="image/png");let a={sampler:this.processSampler(e),source:this.processImage(e.image,e.format,e.flipY,o)};e.name&&(a.name=e.name),await this._invokeAllAsync(async function(l){l.writeTexture&&await l.writeTexture(e,a)});let c=r.textures.push(a)-1;return i.textures.set(e,c),c}async processMaterialAsync(e){let t=this.cache,n=this.json;if(t.materials.has(e))return t.materials.get(e);if(e.isShaderMaterial)return console.warn("GLTFExporter: THREE.ShaderMaterial not supported."),null;n.materials||(n.materials=[]);let i={pbrMetallicRoughness:{}};e.isMeshStandardMaterial!==!0&&e.isMeshBasicMaterial!==!0&&console.warn("GLTFExporter: Use MeshStandardMaterial or MeshBasicMaterial for best results.");let r=e.color.toArray().concat([e.opacity]);if(po(r,[1,1,1,1])||(i.pbrMetallicRoughness.baseColorFactor=r),e.isMeshStandardMaterial?(i.pbrMetallicRoughness.metallicFactor=e.metalness,i.pbrMetallicRoughness.roughnessFactor=e.roughness):(i.pbrMetallicRoughness.metallicFactor=0,i.pbrMetallicRoughness.roughnessFactor=1),e.metalnessMap||e.roughnessMap){let a=await this.buildMetalRoughTextureAsync(e.metalnessMap,e.roughnessMap),c={index:await this.processTextureAsync(a),texCoord:a.channel};this.applyTextureTransform(c,a),i.pbrMetallicRoughness.metallicRoughnessTexture=c}if(e.map){let a={index:await this.processTextureAsync(e.map),texCoord:e.map.channel};this.applyTextureTransform(a,e.map),i.pbrMetallicRoughness.baseColorTexture=a}if(e.emissive){let a=e.emissive;if(Math.max(a.r,a.g,a.b)>0&&(i.emissiveFactor=e.emissive.toArray()),e.emissiveMap){let l={index:await this.processTextureAsync(e.emissiveMap),texCoord:e.emissiveMap.channel};this.applyTextureTransform(l,e.emissiveMap),i.emissiveTexture=l}}if(e.normalMap){let a={index:await this.processTextureAsync(e.normalMap),texCoord:e.normalMap.channel};e.normalScale&&e.normalScale.x!==1&&(a.scale=e.normalScale.x),this.applyTextureTransform(a,e.normalMap),i.normalTexture=a}if(e.aoMap){let a={index:await this.processTextureAsync(e.aoMap),texCoord:e.aoMap.channel};e.aoMapIntensity!==1&&(a.strength=e.aoMapIntensity),this.applyTextureTransform(a,e.aoMap),i.occlusionTexture=a}e.transparent?i.alphaMode="BLEND":e.alphaTest>0&&(i.alphaMode="MASK",i.alphaCutoff=e.alphaTest),e.side===Kt&&(i.doubleSided=!0),e.name!==""&&(i.name=e.name),this.serializeUserData(e,i),await this._invokeAllAsync(async function(a){a.writeMaterialAsync&&await a.writeMaterialAsync(e,i)});let o=n.materials.push(i)-1;return t.materials.set(e,o),o}async processMeshAsync(e){let t=this.cache,n=this.json,i=[e.geometry.uuid];if(Array.isArray(e.material))for(let y=0,w=e.material.length;y<w;y++)i.push(e.material[y].uuid);else i.push(e.material.uuid);let r=i.join(":");if(t.meshes.has(r))return t.meshes.get(r);let o=e.geometry,a;e.isLineSegments?a=je.LINES:e.isLineLoop?a=je.LINE_LOOP:e.isLine?a=je.LINE_STRIP:e.isPoints?a=je.POINTS:a=e.material.wireframe?je.LINES:je.TRIANGLES;let c={},l={},h=[],u=[],d={uv:"TEXCOORD_0",uv1:"TEXCOORD_1",uv2:"TEXCOORD_2",uv3:"TEXCOORD_3",color:"COLOR_0",skinWeight:"WEIGHTS_0",skinIndex:"JOINTS_0"},f=o.getAttribute("normal");f!==void 0&&!this.isNormalizedNormalAttribute(f)&&(console.warn("THREE.GLTFExporter: Creating normalized normal attribute from the non-normalized one."),o.setAttribute("normal",this.createNormalizedNormalAttribute(f)));let g=null;for(let y in o.attributes){if(y.slice(0,5)==="morph")continue;let w=o.attributes[y];if(y=d[y]||y.toUpperCase(),/^(POSITION|NORMAL|TANGENT|TEXCOORD_\d+|COLOR_\d+|JOINTS_\d+|WEIGHTS_\d+)$/.test(y)||(y="_"+y),t.attributes.has(this.getUID(w))){l[y]=t.attributes.get(this.getUID(w));continue}g=null;let R=w.array;y==="JOINTS_0"&&!(R instanceof Uint16Array)&&!(R instanceof Uint8Array)?(console.warn('GLTFExporter: Attribute "skinIndex" converted to type UNSIGNED_SHORT.'),g=new at(new Uint16Array(R),w.itemSize,w.normalized)):(R instanceof Uint32Array||R instanceof Int32Array)&&!y.startsWith("_")&&(console.warn(`GLTFExporter: Attribute "${y}" converted to type FLOAT.`),g=xs.Utils.toFloat32BufferAttribute(w));let N=this.processAccessor(g||w,o);N!==null&&(y.startsWith("_")||this.detectMeshQuantization(y,w),l[y]=N,t.attributes.set(this.getUID(w),N))}if(f!==void 0&&o.setAttribute("normal",f),Object.keys(l).length===0)return null;if(e.morphTargetInfluences!==void 0&&e.morphTargetInfluences.length>0){let y=[],w=[],C={};if(e.morphTargetDictionary!==void 0)for(let R in e.morphTargetDictionary)C[e.morphTargetDictionary[R]]=R;for(let R=0;R<e.morphTargetInfluences.length;++R){let N={},M=!1;for(let b in o.morphAttributes){if(b!=="position"&&b!=="normal"){M||(console.warn("GLTFExporter: Only POSITION and NORMAL morph are supported."),M=!0);continue}let P=o.morphAttributes[b][R],H=b.toUpperCase(),G=o.attributes[b];if(t.attributes.has(this.getUID(P,!0))){N[H]=t.attributes.get(this.getUID(P,!0));continue}let q=P.clone();if(!o.morphTargetsRelative)for(let W=0,Y=P.count;W<Y;W++)for(let B=0;B<P.itemSize;B++)B===0&&q.setX(W,P.getX(W)-G.getX(W)),B===1&&q.setY(W,P.getY(W)-G.getY(W)),B===2&&q.setZ(W,P.getZ(W)-G.getZ(W)),B===3&&q.setW(W,P.getW(W)-G.getW(W));N[H]=this.processAccessor(q,o),t.attributes.set(this.getUID(G,!0),N[H])}u.push(N),y.push(e.morphTargetInfluences[R]),e.morphTargetDictionary!==void 0&&w.push(C[R])}c.weights=y,w.length>0&&(c.extras={},c.extras.targetNames=w)}let _=Array.isArray(e.material);if(_&&o.groups.length===0)return null;let m=!1;if(_&&o.index===null){let y=[];for(let w=0,C=o.attributes.position.count;w<C;w++)y[w]=w;o.setIndex(y),m=!0}let p=_?e.material:[e.material],S=_?o.groups:[{materialIndex:0,start:void 0,count:void 0}];for(let y=0,w=S.length;y<w;y++){let C={mode:a,attributes:l};if(this.serializeUserData(o,C),u.length>0&&(C.targets=u),o.index!==null){let N=this.getUID(o.index);(S[y].start!==void 0||S[y].count!==void 0)&&(N+=":"+S[y].start+":"+S[y].count),t.attributes.has(N)?C.indices=t.attributes.get(N):(C.indices=this.processAccessor(o.index,o,S[y].start,S[y].count),t.attributes.set(N,C.indices)),C.indices===null&&delete C.indices}let R=await this.processMaterialAsync(p[S[y].materialIndex]);R!==null&&(C.material=R),h.push(C)}m===!0&&o.setIndex(null),c.primitives=h,n.meshes||(n.meshes=[]),await this._invokeAllAsync(function(y){y.writeMesh&&y.writeMesh(e,c)});let E=n.meshes.push(c)-1;return t.meshes.set(r,E),E}detectMeshQuantization(e,t){if(this.extensionsUsed[$l])return;let n;switch(t.array.constructor){case Int8Array:n="byte";break;case Uint8Array:n="unsigned byte";break;case Int16Array:n="short";break;case Uint16Array:n="unsigned short";break;default:return}t.normalized&&(n+=" normalized");let i=e.split("_",1)[0];wf[i]&&wf[i].includes(n)&&(this.extensionsUsed[$l]=!0,this.extensionsRequired[$l]=!0)}processCamera(e){let t=this.json;t.cameras||(t.cameras=[]);let n=e.isOrthographicCamera,i={type:n?"orthographic":"perspective"};return n?i.orthographic={xmag:e.right*2,ymag:e.top*2,zfar:e.far<=0?.001:e.far,znear:e.near<0?0:e.near}:i.perspective={aspectRatio:e.aspect,yfov:Ti.degToRad(e.fov),zfar:e.far<=0?.001:e.far,znear:e.near<0?0:e.near},e.name!==""&&(i.name=e.type),t.cameras.push(i)-1}processAnimation(e,t){let n=this.json,i=this.nodeMap;n.animations||(n.animations=[]),e=xs.Utils.mergeMorphTargetTracks(e.clone(),t);let r=e.tracks,o=[],a=[];for(let l=0;l<r.length;++l){let h=r[l],u=tt.parseTrackName(h.name),d=tt.findNode(t,u.nodeName),f=Af[u.propertyName];if(u.objectName==="bones"&&(d.isSkinnedMesh===!0?d=d.skeleton.getBoneByName(u.objectIndex):d=void 0),!d||!f){console.warn('THREE.GLTFExporter: Could not export animation track "%s".',h.name);continue}let g=1,_=h.values.length/h.times.length;f===Af.morphTargetInfluences&&(_/=d.morphTargetInfluences.length);let m;h.createInterpolant.isInterpolantFactoryMethodGLTFCubicSpline===!0?(m="CUBICSPLINE",_/=3):h.getInterpolation()===pi?m="STEP":m="LINEAR",a.push({input:this.processAccessor(new at(h.times,g)),output:this.processAccessor(new at(h.values,_)),interpolation:m}),o.push({sampler:a.length-1,target:{node:i.get(d),path:f}})}let c={name:e.name||"clip_"+n.animations.length,samplers:a,channels:o};return this.serializeUserData(e,c),n.animations.push(c),n.animations.length-1}processSkin(e){let t=this.json,n=this.nodeMap,i=t.nodes[n.get(e)],r=e.skeleton;if(r===void 0)return null;let o=e.skeleton.bones[0];if(o===void 0)return null;let a=[],c=new Float32Array(r.bones.length*16),l=new De;for(let u=0;u<r.bones.length;++u)a.push(n.get(r.bones[u])),l.copy(r.boneInverses[u]),l.multiply(e.bindMatrix).toArray(c,u*16);return t.skins===void 0&&(t.skins=[]),t.skins.push({inverseBindMatrices:this.processAccessor(new at(c,16)),joints:a,skeleton:n.get(o)}),i.skin=t.skins.length-1}async processNodeAsync(e){let t=this.json,n=this.options,i=this.nodeMap;t.nodes||(t.nodes=[]);let r={};if(n.trs){let a=e.quaternion.toArray(),c=e.position.toArray(),l=e.scale.toArray();po(a,[0,0,0,1])||(r.rotation=a),po(c,[0,0,0])||(r.translation=c),po(l,[1,1,1])||(r.scale=l)}else e.matrixAutoUpdate&&e.updateMatrix(),cy(e.matrix)===!1&&(r.matrix=e.matrix.elements);if(e.name!==""&&(r.name=String(e.name)),this.serializeUserData(e,r),e.isMesh||e.isLine||e.isPoints){let a=await this.processMeshAsync(e);a!==null&&(r.mesh=a)}else e.isCamera&&(r.camera=this.processCamera(e));e.isSkinnedMesh&&this.skins.push(e);let o=t.nodes.push(r)-1;if(i.set(e,o),e.children.length>0){let a=[];for(let c=0,l=e.children.length;c<l;c++){let h=e.children[c];if(h.visible||n.onlyVisible===!1){let u=await this.processNodeAsync(h);u!==null&&a.push(u)}}a.length>0&&(r.children=a)}return await this._invokeAllAsync(function(a){a.writeNode&&a.writeNode(e,r)}),o}async processSceneAsync(e){let t=this.json,n=this.options;t.scenes||(t.scenes=[],t.scene=0);let i={};e.name!==""&&(i.name=e.name),t.scenes.push(i);let r=[];for(let o=0,a=e.children.length;o<a;o++){let c=e.children[o];if(c.visible||n.onlyVisible===!1){let l=await this.processNodeAsync(c);l!==null&&r.push(l)}}r.length>0&&(i.nodes=r),this.serializeUserData(e,i)}async processObjectsAsync(e){let t=new Fi;t.name="AuxScene";for(let n=0;n<e.length;n++)t.children.push(e[n]);await this.processSceneAsync(t)}async processInputAsync(e){let t=this.options;e=e instanceof Array?e:[e],await this._invokeAllAsync(function(i){i.beforeParse&&i.beforeParse(e)});let n=[];for(let i=0;i<e.length;i++)e[i]instanceof Fi?await this.processSceneAsync(e[i]):n.push(e[i]);n.length>0&&await this.processObjectsAsync(n);for(let i=0;i<this.skins.length;++i)this.processSkin(this.skins[i]);for(let i=0;i<t.animations.length;++i)this.processAnimation(t.animations[i],e[0]);await this._invokeAllAsync(function(i){i.afterParse&&i.afterParse(e)})}async _invokeAllAsync(e){for(let t=0,n=this.plugins.length;t<n;t++)await e(this.plugins[t])}},th=class{constructor(e){this.writer=e,this.name="KHR_lights_punctual"}writeNode(e,t){if(!e.isLight)return;if(!e.isDirectionalLight&&!e.isPointLight&&!e.isSpotLight){console.warn("THREE.GLTFExporter: Only directional, point, and spot lights are supported.",e);return}let n=this.writer,i=n.json,r=n.extensionsUsed,o={};e.name&&(o.name=e.name),o.color=e.color.toArray(),o.intensity=e.intensity,e.isDirectionalLight?o.type="directional":e.isPointLight?(o.type="point",e.distance>0&&(o.range=e.distance)):e.isSpotLight&&(o.type="spot",e.distance>0&&(o.range=e.distance),o.spot={},o.spot.innerConeAngle=(1-e.penumbra)*e.angle,o.spot.outerConeAngle=e.angle),e.decay!==void 0&&e.decay!==2&&console.warn("THREE.GLTFExporter: Light decay may be lost. glTF is physically-based, and expects light.decay=2."),e.target&&(e.target.parent!==e||e.target.position.x!==0||e.target.position.y!==0||e.target.position.z!==-1)&&console.warn("THREE.GLTFExporter: Light direction may be lost. For best results, make light.target a child of the light with position 0,0,-1."),r[this.name]||(i.extensions=i.extensions||{},i.extensions[this.name]={lights:[]},r[this.name]=!0);let a=i.extensions[this.name].lights;a.push(o),t.extensions=t.extensions||{},t.extensions[this.name]={light:a.length-1}}},nh=class{constructor(e){this.writer=e,this.name="KHR_materials_unlit"}async writeMaterialAsync(e,t){if(!e.isMeshBasicMaterial)return;let i=this.writer.extensionsUsed;t.extensions=t.extensions||{},t.extensions[this.name]={},i[this.name]=!0,t.pbrMetallicRoughness.metallicFactor=0,t.pbrMetallicRoughness.roughnessFactor=.9}},ih=class{constructor(e){this.writer=e,this.name="KHR_materials_clearcoat"}async writeMaterialAsync(e,t){if(!e.isMeshPhysicalMaterial||e.clearcoat===0)return;let n=this.writer,i=n.extensionsUsed,r={};if(r.clearcoatFactor=e.clearcoat,e.clearcoatMap){let o={index:await n.processTextureAsync(e.clearcoatMap),texCoord:e.clearcoatMap.channel};n.applyTextureTransform(o,e.clearcoatMap),r.clearcoatTexture=o}if(r.clearcoatRoughnessFactor=e.clearcoatRoughness,e.clearcoatRoughnessMap){let o={index:await n.processTextureAsync(e.clearcoatRoughnessMap),texCoord:e.clearcoatRoughnessMap.channel};n.applyTextureTransform(o,e.clearcoatRoughnessMap),r.clearcoatRoughnessTexture=o}if(e.clearcoatNormalMap){let o={index:await n.processTextureAsync(e.clearcoatNormalMap),texCoord:e.clearcoatNormalMap.channel};e.clearcoatNormalScale.x!==1&&(o.scale=e.clearcoatNormalScale.x),n.applyTextureTransform(o,e.clearcoatNormalMap),r.clearcoatNormalTexture=o}t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},sh=class{constructor(e){this.writer=e,this.name="KHR_materials_dispersion"}async writeMaterialAsync(e,t){if(!e.isMeshPhysicalMaterial||e.dispersion===0)return;let i=this.writer.extensionsUsed,r={};r.dispersion=e.dispersion,t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},rh=class{constructor(e){this.writer=e,this.name="KHR_materials_iridescence"}async writeMaterialAsync(e,t){if(!e.isMeshPhysicalMaterial||e.iridescence===0)return;let n=this.writer,i=n.extensionsUsed,r={};if(r.iridescenceFactor=e.iridescence,e.iridescenceMap){let o={index:await n.processTextureAsync(e.iridescenceMap),texCoord:e.iridescenceMap.channel};n.applyTextureTransform(o,e.iridescenceMap),r.iridescenceTexture=o}if(r.iridescenceIor=e.iridescenceIOR,r.iridescenceThicknessMinimum=e.iridescenceThicknessRange[0],r.iridescenceThicknessMaximum=e.iridescenceThicknessRange[1],e.iridescenceThicknessMap){let o={index:await n.processTextureAsync(e.iridescenceThicknessMap),texCoord:e.iridescenceThicknessMap.channel};n.applyTextureTransform(o,e.iridescenceThicknessMap),r.iridescenceThicknessTexture=o}t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},oh=class{constructor(e){this.writer=e,this.name="KHR_materials_transmission"}async writeMaterialAsync(e,t){if(!e.isMeshPhysicalMaterial||e.transmission===0)return;let n=this.writer,i=n.extensionsUsed,r={};if(r.transmissionFactor=e.transmission,e.transmissionMap){let o={index:await n.processTextureAsync(e.transmissionMap),texCoord:e.transmissionMap.channel};n.applyTextureTransform(o,e.transmissionMap),r.transmissionTexture=o}t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},ah=class{constructor(e){this.writer=e,this.name="KHR_materials_volume"}async writeMaterialAsync(e,t){if(!e.isMeshPhysicalMaterial||e.transmission===0)return;let n=this.writer,i=n.extensionsUsed,r={};if(r.thicknessFactor=e.thickness,e.thicknessMap){let o={index:await n.processTextureAsync(e.thicknessMap),texCoord:e.thicknessMap.channel};n.applyTextureTransform(o,e.thicknessMap),r.thicknessTexture=o}e.attenuationDistance!==1/0&&(r.attenuationDistance=e.attenuationDistance),r.attenuationColor=e.attenuationColor.toArray(),t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},ch=class{constructor(e){this.writer=e,this.name="KHR_materials_ior"}async writeMaterialAsync(e,t){if(!e.isMeshPhysicalMaterial||e.ior===1.5)return;let i=this.writer.extensionsUsed,r={};r.ior=e.ior,t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},lh=class{constructor(e){this.writer=e,this.name="KHR_materials_specular"}async writeMaterialAsync(e,t){if(!e.isMeshPhysicalMaterial||e.specularIntensity===1&&e.specularColor.equals(ny)&&!e.specularIntensityMap&&!e.specularColorMap)return;let n=this.writer,i=n.extensionsUsed,r={};if(e.specularIntensityMap){let o={index:await n.processTextureAsync(e.specularIntensityMap),texCoord:e.specularIntensityMap.channel};n.applyTextureTransform(o,e.specularIntensityMap),r.specularTexture=o}if(e.specularColorMap){let o={index:await n.processTextureAsync(e.specularColorMap),texCoord:e.specularColorMap.channel};n.applyTextureTransform(o,e.specularColorMap),r.specularColorTexture=o}r.specularFactor=e.specularIntensity,r.specularColorFactor=e.specularColor.toArray(),t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},hh=class{constructor(e){this.writer=e,this.name="KHR_materials_sheen"}async writeMaterialAsync(e,t){if(!e.isMeshPhysicalMaterial||e.sheen==0)return;let n=this.writer,i=n.extensionsUsed,r={};if(e.sheenRoughnessMap){let o={index:await n.processTextureAsync(e.sheenRoughnessMap),texCoord:e.sheenRoughnessMap.channel};n.applyTextureTransform(o,e.sheenRoughnessMap),r.sheenRoughnessTexture=o}if(e.sheenColorMap){let o={index:await n.processTextureAsync(e.sheenColorMap),texCoord:e.sheenColorMap.channel};n.applyTextureTransform(o,e.sheenColorMap),r.sheenColorTexture=o}r.sheenRoughnessFactor=e.sheenRoughness,r.sheenColorFactor=e.sheenColor.toArray(),t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},uh=class{constructor(e){this.writer=e,this.name="KHR_materials_anisotropy"}async writeMaterialAsync(e,t){if(!e.isMeshPhysicalMaterial||e.anisotropy==0)return;let n=this.writer,i=n.extensionsUsed,r={};if(e.anisotropyMap){let o={index:await n.processTextureAsync(e.anisotropyMap)};n.applyTextureTransform(o,e.anisotropyMap),r.anisotropyTexture=o}r.anisotropyStrength=e.anisotropy,r.anisotropyRotation=e.anisotropyRotation,t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},dh=class{constructor(e){this.writer=e,this.name="KHR_materials_emissive_strength"}async writeMaterialAsync(e,t){if(!e.isMeshStandardMaterial||e.emissiveIntensity===1)return;let i=this.writer.extensionsUsed,r={};r.emissiveStrength=e.emissiveIntensity,t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},fh=class{constructor(e){this.writer=e,this.name="EXT_materials_bump"}async writeMaterialAsync(e,t){if(!e.isMeshStandardMaterial||e.bumpScale===1&&!e.bumpMap)return;let n=this.writer,i=n.extensionsUsed,r={};if(e.bumpMap){let o={index:await n.processTextureAsync(e.bumpMap),texCoord:e.bumpMap.channel};n.applyTextureTransform(o,e.bumpMap),r.bumpTexture=o}r.bumpFactor=e.bumpScale,t.extensions=t.extensions||{},t.extensions[this.name]=r,i[this.name]=!0}},ph=class{constructor(e){this.writer=e,this.name="EXT_mesh_gpu_instancing"}writeNode(e,t){if(!e.isInstancedMesh)return;let n=this.writer,i=e,r=new Float32Array(i.count*3),o=new Float32Array(i.count*4),a=new Float32Array(i.count*3),c=new De,l=new A,h=new It,u=new A;for(let f=0;f<i.count;f++)i.getMatrixAt(f,c),c.decompose(l,h,u),l.toArray(r,f*3),h.toArray(o,f*4),u.toArray(a,f*3);let d={TRANSLATION:n.processAccessor(new at(r,3)),ROTATION:n.processAccessor(new at(o,4)),SCALE:n.processAccessor(new at(a,3))};i.instanceColor&&(d._COLOR_0=n.processAccessor(i.instanceColor)),t.extensions=t.extensions||{},t.extensions[this.name]={attributes:d},n.extensionsUsed[this.name]=!0,n.extensionsRequired[this.name]=!0}};xs.Utils={insertKeyframe:function(s,e){let n=s.getValueSize(),i=new s.TimeBufferType(s.times.length+1),r=new s.ValueBufferType(s.values.length+n),o=s.createInterpolant(new s.ValueBufferType(n)),a;if(s.times.length===0){i[0]=e;for(let c=0;c<n;c++)r[c]=0;a=0}else if(e<s.times[0]){if(Math.abs(s.times[0]-e)<.001)return 0;i[0]=e,i.set(s.times,1),r.set(o.evaluate(e),0),r.set(s.values,n),a=0}else if(e>s.times[s.times.length-1]){if(Math.abs(s.times[s.times.length-1]-e)<.001)return s.times.length-1;i[i.length-1]=e,i.set(s.times,0),r.set(s.values,0),r.set(o.evaluate(e),s.values.length),a=i.length-1}else for(let c=0;c<s.times.length;c++){if(Math.abs(s.times[c]-e)<.001)return c;if(s.times[c]<e&&s.times[c+1]>e){i.set(s.times.slice(0,c+1),0),i[c+1]=e,i.set(s.times.slice(c+1),c+2),r.set(s.values.slice(0,(c+1)*n),0),r.set(o.evaluate(e),(c+1)*n),r.set(s.values.slice((c+1)*n),(c+2)*n),a=c+1;break}}return s.times=i,s.values=r,a},mergeMorphTargetTracks:function(s,e){let t=[],n={},i=s.tracks;for(let r=0;r<i.length;++r){let o=i[r],a=tt.parseTrackName(o.name),c=tt.findNode(e,a.nodeName);if(a.propertyName!=="morphTargetInfluences"||a.propertyIndex===void 0){t.push(o);continue}if(o.createInterpolant!==o.InterpolantFactoryMethodDiscrete&&o.createInterpolant!==o.InterpolantFactoryMethodLinear){if(o.createInterpolant.isInterpolantFactoryMethodGLTFCubicSpline)throw new Error("THREE.GLTFExporter: Cannot merge tracks with glTF CUBICSPLINE interpolation.");console.warn("THREE.GLTFExporter: Morph target interpolation mode not yet supported. Using LINEAR instead."),o=o.clone(),o.setInterpolation(mi)}let l=c.morphTargetInfluences.length,h=c.morphTargetDictionary[a.propertyIndex];if(h===void 0)throw new Error("THREE.GLTFExporter: Morph target name not found: "+a.propertyIndex);let u;if(n[c.uuid]===void 0){u=o.clone();let f=new u.ValueBufferType(l*u.times.length);for(let g=0;g<u.times.length;g++)f[g*l+h]=u.values[g];u.name=(a.nodeName||"")+".morphTargetInfluences",u.values=f,n[c.uuid]=u,t.push(u);continue}let d=o.createInterpolant(new o.ValueBufferType(1));u=n[c.uuid];for(let f=0;f<u.times.length;f++)u.values[f*l+h]=d.evaluate(u.times[f]);for(let f=0;f<o.times.length;f++){let g=this.insertKeyframe(u,o.times[f]);u.values[g*l+h]=o.values[f]}}return s.tracks=t,s},toFloat32BufferAttribute:function(s){let e=new at(new Float32Array(s.count*s.itemSize),s.itemSize,!1);if(!s.normalized&&!s.isInterleavedBufferAttribute)return e.array.set(s.array),e;for(let t=0,n=s.count;t<n;t++)for(let i=0;i<s.itemSize;i++)e.setComponent(t,i,s.getComponent(t,i));return e}};function mh(s,e){if(e===El)return console.warn("THREE.BufferGeometryUtils.toTrianglesDrawMode(): Geometry already defined as triangles."),s;if(e===cr||e===ho){let t=s.getIndex();if(t===null){let o=[],a=s.getAttribute("position");if(a!==void 0){for(let c=0;c<a.count;c++)o.push(c);s.setIndex(o),t=s.getIndex()}else return console.error("THREE.BufferGeometryUtils.toTrianglesDrawMode(): Undefined position attribute. Processing not possible."),s}let n=t.count-2,i=[];if(e===cr)for(let o=1;o<=n;o++)i.push(t.getX(0)),i.push(t.getX(o)),i.push(t.getX(o+1));else for(let o=0;o<n;o++)o%2===0?(i.push(t.getX(o)),i.push(t.getX(o+1)),i.push(t.getX(o+2))):(i.push(t.getX(o+2)),i.push(t.getX(o+1)),i.push(t.getX(o)));i.length/3!==n&&console.error("THREE.BufferGeometryUtils.toTrianglesDrawMode(): Unable to generate correct amount of triangles.");let r=s.clone();return r.setIndex(i),r.clearGroups(),r}else return console.error("THREE.BufferGeometryUtils.toTrianglesDrawMode(): Unknown draw mode:",e),s}var fc=class extends ti{constructor(e){super(e),this.dracoLoader=null,this.ktx2Loader=null,this.meshoptDecoder=null,this.pluginCallbacks=[],this.register(function(t){return new Mh(t)}),this.register(function(t){return new Sh(t)}),this.register(function(t){return new Lh(t)}),this.register(function(t){return new Nh(t)}),this.register(function(t){return new Dh(t)}),this.register(function(t){return new Th(t)}),this.register(function(t){return new wh(t)}),this.register(function(t){return new Ah(t)}),this.register(function(t){return new Rh(t)}),this.register(function(t){return new bh(t)}),this.register(function(t){return new Ch(t)}),this.register(function(t){return new Eh(t)}),this.register(function(t){return new Ph(t)}),this.register(function(t){return new Ih(t)}),this.register(function(t){return new yh(t)}),this.register(function(t){return new Uh(t)}),this.register(function(t){return new Oh(t)})}load(e,t,n,i){let r=this,o;if(this.resourcePath!=="")o=this.resourcePath;else if(this.path!==""){let l=bi.extractUrlBase(e);o=bi.resolveURL(l,this.path)}else o=bi.extractUrlBase(e);this.manager.itemStart(e);let a=function(l){i?i(l):console.error(l),r.manager.itemError(e),r.manager.itemEnd(e)},c=new tr(this.manager);c.setPath(this.path),c.setResponseType("arraybuffer"),c.setRequestHeader(this.requestHeader),c.setWithCredentials(this.withCredentials),c.load(e,function(l){try{r.parse(l,o,function(h){t(h),r.manager.itemEnd(e)},a)}catch(h){a(h)}},n,a)}setDRACOLoader(e){return this.dracoLoader=e,this}setKTX2Loader(e){return this.ktx2Loader=e,this}setMeshoptDecoder(e){return this.meshoptDecoder=e,this}register(e){return this.pluginCallbacks.indexOf(e)===-1&&this.pluginCallbacks.push(e),this}unregister(e){return this.pluginCallbacks.indexOf(e)!==-1&&this.pluginCallbacks.splice(this.pluginCallbacks.indexOf(e),1),this}parse(e,t,n,i){let r,o={},a={},c=new TextDecoder;if(typeof e=="string")r=JSON.parse(e);else if(e instanceof ArrayBuffer)if(c.decode(new Uint8Array(e,0,4))===Of){try{o[Ke.KHR_BINARY_GLTF]=new Fh(e)}catch(u){i&&i(u);return}r=JSON.parse(o[Ke.KHR_BINARY_GLTF].content)}else r=JSON.parse(c.decode(e));else r=e;if(r.asset===void 0||r.asset.version[0]<2){i&&i(new Error("THREE.GLTFLoader: Unsupported asset. glTF versions >=2.0 are supported."));return}let l=new Wh(r,{path:t||this.resourcePath||"",crossOrigin:this.crossOrigin,requestHeader:this.requestHeader,manager:this.manager,ktx2Loader:this.ktx2Loader,meshoptDecoder:this.meshoptDecoder});l.fileLoader.setRequestHeader(this.requestHeader);for(let h=0;h<this.pluginCallbacks.length;h++){let u=this.pluginCallbacks[h](l);u.name||console.error("THREE.GLTFLoader: Invalid plugin found: missing name"),a[u.name]=u,o[u.name]=!0}if(r.extensionsUsed)for(let h=0;h<r.extensionsUsed.length;++h){let u=r.extensionsUsed[h],d=r.extensionsRequired||[];switch(u){case Ke.KHR_MATERIALS_UNLIT:o[u]=new vh;break;case Ke.KHR_DRACO_MESH_COMPRESSION:o[u]=new Bh(r,this.dracoLoader);break;case Ke.KHR_TEXTURE_TRANSFORM:o[u]=new kh;break;case Ke.KHR_MESH_QUANTIZATION:o[u]=new zh;break;default:d.indexOf(u)>=0&&a[u]===void 0&&console.warn('THREE.GLTFLoader: Unknown extension "'+u+'".')}}l.setExtensions(o),l.setPlugins(a),l.parse(n,i)}parseAsync(e,t){let n=this;return new Promise(function(i,r){n.parse(e,t,i,r)})}};function uy(){let s={};return{get:function(e){return s[e]},add:function(e,t){s[e]=t},remove:function(e){delete s[e]},removeAll:function(){s={}}}}var Ke={KHR_BINARY_GLTF:"KHR_binary_glTF",KHR_DRACO_MESH_COMPRESSION:"KHR_draco_mesh_compression",KHR_LIGHTS_PUNCTUAL:"KHR_lights_punctual",KHR_MATERIALS_CLEARCOAT:"KHR_materials_clearcoat",KHR_MATERIALS_DISPERSION:"KHR_materials_dispersion",KHR_MATERIALS_IOR:"KHR_materials_ior",KHR_MATERIALS_SHEEN:"KHR_materials_sheen",KHR_MATERIALS_SPECULAR:"KHR_materials_specular",KHR_MATERIALS_TRANSMISSION:"KHR_materials_transmission",KHR_MATERIALS_IRIDESCENCE:"KHR_materials_iridescence",KHR_MATERIALS_ANISOTROPY:"KHR_materials_anisotropy",KHR_MATERIALS_UNLIT:"KHR_materials_unlit",KHR_MATERIALS_VOLUME:"KHR_materials_volume",KHR_TEXTURE_BASISU:"KHR_texture_basisu",KHR_TEXTURE_TRANSFORM:"KHR_texture_transform",KHR_MESH_QUANTIZATION:"KHR_mesh_quantization",KHR_MATERIALS_EMISSIVE_STRENGTH:"KHR_materials_emissive_strength",EXT_MATERIALS_BUMP:"EXT_materials_bump",EXT_TEXTURE_WEBP:"EXT_texture_webp",EXT_TEXTURE_AVIF:"EXT_texture_avif",EXT_MESHOPT_COMPRESSION:"EXT_meshopt_compression",EXT_MESH_GPU_INSTANCING:"EXT_mesh_gpu_instancing"},yh=class{constructor(e){this.parser=e,this.name=Ke.KHR_LIGHTS_PUNCTUAL,this.cache={refs:{},uses:{}}}_markDefs(){let e=this.parser,t=this.parser.json.nodes||[];for(let n=0,i=t.length;n<i;n++){let r=t[n];r.extensions&&r.extensions[this.name]&&r.extensions[this.name].light!==void 0&&e._addNodeRef(this.cache,r.extensions[this.name].light)}}_loadLight(e){let t=this.parser,n="light:"+e,i=t.cache.get(n);if(i)return i;let r=t.json,c=((r.extensions&&r.extensions[this.name]||{}).lights||[])[e],l,h=new Re(16777215);c.color!==void 0&&h.setRGB(c.color[0],c.color[1],c.color[2],Vt);let u=c.range!==void 0?c.range:0;switch(c.type){case"directional":l=new vi(h),l.target.position.set(0,0,-1),l.add(l.target);break;case"point":l=new eo(h),l.distance=u;break;case"spot":l=new Qr(h),l.distance=u,c.spot=c.spot||{},c.spot.innerConeAngle=c.spot.innerConeAngle!==void 0?c.spot.innerConeAngle:0,c.spot.outerConeAngle=c.spot.outerConeAngle!==void 0?c.spot.outerConeAngle:Math.PI/4,l.angle=c.spot.outerConeAngle,l.penumbra=1-c.spot.innerConeAngle/c.spot.outerConeAngle,l.target.position.set(0,0,-1),l.add(l.target);break;default:throw new Error("THREE.GLTFLoader: Unexpected light type: "+c.type)}return l.position.set(0,0,0),si(l,c),c.intensity!==void 0&&(l.intensity=c.intensity),l.name=t.createUniqueName(c.name||"light_"+e),i=Promise.resolve(l),t.cache.add(n,i),i}getDependency(e,t){if(e==="light")return this._loadLight(t)}createNodeAttachment(e){let t=this,n=this.parser,r=n.json.nodes[e],a=(r.extensions&&r.extensions[this.name]||{}).light;return a===void 0?null:this._loadLight(a).then(function(c){return n._getNodeRef(t.cache,a,c)})}},vh=class{constructor(){this.name=Ke.KHR_MATERIALS_UNLIT}getMaterialType(){return jt}extendParams(e,t,n){let i=[];e.color=new Re(1,1,1),e.opacity=1;let r=t.pbrMetallicRoughness;if(r){if(Array.isArray(r.baseColorFactor)){let o=r.baseColorFactor;e.color.setRGB(o[0],o[1],o[2],Vt),e.opacity=o[3]}r.baseColorTexture!==void 0&&i.push(n.assignTexture(e,"map",r.baseColorTexture,_t))}return Promise.all(i)}},bh=class{constructor(e){this.parser=e,this.name=Ke.KHR_MATERIALS_EMISSIVE_STRENGTH}extendMaterialParams(e,t){let i=this.parser.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=i.extensions[this.name].emissiveStrength;return r!==void 0&&(t.emissiveIntensity=r),Promise.resolve()}},Mh=class{constructor(e){this.parser=e,this.name=Ke.KHR_MATERIALS_CLEARCOAT}getMaterialType(e){let n=this.parser.json.materials[e];return!n.extensions||!n.extensions[this.name]?null:rn}extendMaterialParams(e,t){let n=this.parser,i=n.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=[],o=i.extensions[this.name];if(o.clearcoatFactor!==void 0&&(t.clearcoat=o.clearcoatFactor),o.clearcoatTexture!==void 0&&r.push(n.assignTexture(t,"clearcoatMap",o.clearcoatTexture)),o.clearcoatRoughnessFactor!==void 0&&(t.clearcoatRoughness=o.clearcoatRoughnessFactor),o.clearcoatRoughnessTexture!==void 0&&r.push(n.assignTexture(t,"clearcoatRoughnessMap",o.clearcoatRoughnessTexture)),o.clearcoatNormalTexture!==void 0&&(r.push(n.assignTexture(t,"clearcoatNormalMap",o.clearcoatNormalTexture)),o.clearcoatNormalTexture.scale!==void 0)){let a=o.clearcoatNormalTexture.scale;t.clearcoatNormalScale=new Ee(a,a)}return Promise.all(r)}},Sh=class{constructor(e){this.parser=e,this.name=Ke.KHR_MATERIALS_DISPERSION}getMaterialType(e){let n=this.parser.json.materials[e];return!n.extensions||!n.extensions[this.name]?null:rn}extendMaterialParams(e,t){let i=this.parser.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=i.extensions[this.name];return t.dispersion=r.dispersion!==void 0?r.dispersion:0,Promise.resolve()}},Eh=class{constructor(e){this.parser=e,this.name=Ke.KHR_MATERIALS_IRIDESCENCE}getMaterialType(e){let n=this.parser.json.materials[e];return!n.extensions||!n.extensions[this.name]?null:rn}extendMaterialParams(e,t){let n=this.parser,i=n.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=[],o=i.extensions[this.name];return o.iridescenceFactor!==void 0&&(t.iridescence=o.iridescenceFactor),o.iridescenceTexture!==void 0&&r.push(n.assignTexture(t,"iridescenceMap",o.iridescenceTexture)),o.iridescenceIor!==void 0&&(t.iridescenceIOR=o.iridescenceIor),t.iridescenceThicknessRange===void 0&&(t.iridescenceThicknessRange=[100,400]),o.iridescenceThicknessMinimum!==void 0&&(t.iridescenceThicknessRange[0]=o.iridescenceThicknessMinimum),o.iridescenceThicknessMaximum!==void 0&&(t.iridescenceThicknessRange[1]=o.iridescenceThicknessMaximum),o.iridescenceThicknessTexture!==void 0&&r.push(n.assignTexture(t,"iridescenceThicknessMap",o.iridescenceThicknessTexture)),Promise.all(r)}},Th=class{constructor(e){this.parser=e,this.name=Ke.KHR_MATERIALS_SHEEN}getMaterialType(e){let n=this.parser.json.materials[e];return!n.extensions||!n.extensions[this.name]?null:rn}extendMaterialParams(e,t){let n=this.parser,i=n.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=[];t.sheenColor=new Re(0,0,0),t.sheenRoughness=0,t.sheen=1;let o=i.extensions[this.name];if(o.sheenColorFactor!==void 0){let a=o.sheenColorFactor;t.sheenColor.setRGB(a[0],a[1],a[2],Vt)}return o.sheenRoughnessFactor!==void 0&&(t.sheenRoughness=o.sheenRoughnessFactor),o.sheenColorTexture!==void 0&&r.push(n.assignTexture(t,"sheenColorMap",o.sheenColorTexture,_t)),o.sheenRoughnessTexture!==void 0&&r.push(n.assignTexture(t,"sheenRoughnessMap",o.sheenRoughnessTexture)),Promise.all(r)}},wh=class{constructor(e){this.parser=e,this.name=Ke.KHR_MATERIALS_TRANSMISSION}getMaterialType(e){let n=this.parser.json.materials[e];return!n.extensions||!n.extensions[this.name]?null:rn}extendMaterialParams(e,t){let n=this.parser,i=n.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=[],o=i.extensions[this.name];return o.transmissionFactor!==void 0&&(t.transmission=o.transmissionFactor),o.transmissionTexture!==void 0&&r.push(n.assignTexture(t,"transmissionMap",o.transmissionTexture)),Promise.all(r)}},Ah=class{constructor(e){this.parser=e,this.name=Ke.KHR_MATERIALS_VOLUME}getMaterialType(e){let n=this.parser.json.materials[e];return!n.extensions||!n.extensions[this.name]?null:rn}extendMaterialParams(e,t){let n=this.parser,i=n.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=[],o=i.extensions[this.name];t.thickness=o.thicknessFactor!==void 0?o.thicknessFactor:0,o.thicknessTexture!==void 0&&r.push(n.assignTexture(t,"thicknessMap",o.thicknessTexture)),t.attenuationDistance=o.attenuationDistance||1/0;let a=o.attenuationColor||[1,1,1];return t.attenuationColor=new Re().setRGB(a[0],a[1],a[2],Vt),Promise.all(r)}},Rh=class{constructor(e){this.parser=e,this.name=Ke.KHR_MATERIALS_IOR}getMaterialType(e){let n=this.parser.json.materials[e];return!n.extensions||!n.extensions[this.name]?null:rn}extendMaterialParams(e,t){let i=this.parser.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=i.extensions[this.name];return t.ior=r.ior!==void 0?r.ior:1.5,Promise.resolve()}},Ch=class{constructor(e){this.parser=e,this.name=Ke.KHR_MATERIALS_SPECULAR}getMaterialType(e){let n=this.parser.json.materials[e];return!n.extensions||!n.extensions[this.name]?null:rn}extendMaterialParams(e,t){let n=this.parser,i=n.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=[],o=i.extensions[this.name];t.specularIntensity=o.specularFactor!==void 0?o.specularFactor:1,o.specularTexture!==void 0&&r.push(n.assignTexture(t,"specularIntensityMap",o.specularTexture));let a=o.specularColorFactor||[1,1,1];return t.specularColor=new Re().setRGB(a[0],a[1],a[2],Vt),o.specularColorTexture!==void 0&&r.push(n.assignTexture(t,"specularColorMap",o.specularColorTexture,_t)),Promise.all(r)}},Ih=class{constructor(e){this.parser=e,this.name=Ke.EXT_MATERIALS_BUMP}getMaterialType(e){let n=this.parser.json.materials[e];return!n.extensions||!n.extensions[this.name]?null:rn}extendMaterialParams(e,t){let n=this.parser,i=n.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=[],o=i.extensions[this.name];return t.bumpScale=o.bumpFactor!==void 0?o.bumpFactor:1,o.bumpTexture!==void 0&&r.push(n.assignTexture(t,"bumpMap",o.bumpTexture)),Promise.all(r)}},Ph=class{constructor(e){this.parser=e,this.name=Ke.KHR_MATERIALS_ANISOTROPY}getMaterialType(e){let n=this.parser.json.materials[e];return!n.extensions||!n.extensions[this.name]?null:rn}extendMaterialParams(e,t){let n=this.parser,i=n.json.materials[e];if(!i.extensions||!i.extensions[this.name])return Promise.resolve();let r=[],o=i.extensions[this.name];return o.anisotropyStrength!==void 0&&(t.anisotropy=o.anisotropyStrength),o.anisotropyRotation!==void 0&&(t.anisotropyRotation=o.anisotropyRotation),o.anisotropyTexture!==void 0&&r.push(n.assignTexture(t,"anisotropyMap",o.anisotropyTexture)),Promise.all(r)}},Lh=class{constructor(e){this.parser=e,this.name=Ke.KHR_TEXTURE_BASISU}loadTexture(e){let t=this.parser,n=t.json,i=n.textures[e];if(!i.extensions||!i.extensions[this.name])return null;let r=i.extensions[this.name],o=t.options.ktx2Loader;if(!o){if(n.extensionsRequired&&n.extensionsRequired.indexOf(this.name)>=0)throw new Error("THREE.GLTFLoader: setKTX2Loader must be called before loading KTX2 textures");return null}return t.loadTextureImage(e,r.source,o)}},Nh=class{constructor(e){this.parser=e,this.name=Ke.EXT_TEXTURE_WEBP}loadTexture(e){let t=this.name,n=this.parser,i=n.json,r=i.textures[e];if(!r.extensions||!r.extensions[t])return null;let o=r.extensions[t],a=i.images[o.source],c=n.textureLoader;if(a.uri){let l=n.options.manager.getHandler(a.uri);l!==null&&(c=l)}return n.loadTextureImage(e,o.source,c)}},Dh=class{constructor(e){this.parser=e,this.name=Ke.EXT_TEXTURE_AVIF}loadTexture(e){let t=this.name,n=this.parser,i=n.json,r=i.textures[e];if(!r.extensions||!r.extensions[t])return null;let o=r.extensions[t],a=i.images[o.source],c=n.textureLoader;if(a.uri){let l=n.options.manager.getHandler(a.uri);l!==null&&(c=l)}return n.loadTextureImage(e,o.source,c)}},Uh=class{constructor(e){this.name=Ke.EXT_MESHOPT_COMPRESSION,this.parser=e}loadBufferView(e){let t=this.parser.json,n=t.bufferViews[e];if(n.extensions&&n.extensions[this.name]){let i=n.extensions[this.name],r=this.parser.getDependency("buffer",i.buffer),o=this.parser.options.meshoptDecoder;if(!o||!o.supported){if(t.extensionsRequired&&t.extensionsRequired.indexOf(this.name)>=0)throw new Error("THREE.GLTFLoader: setMeshoptDecoder must be called before loading compressed files");return null}return r.then(function(a){let c=i.byteOffset||0,l=i.byteLength||0,h=i.count,u=i.byteStride,d=new Uint8Array(a,c,l);return o.decodeGltfBufferAsync?o.decodeGltfBufferAsync(h,u,d,i.mode,i.filter).then(function(f){return f.buffer}):o.ready.then(function(){let f=new ArrayBuffer(h*u);return o.decodeGltfBuffer(new Uint8Array(f),h,u,d,i.mode,i.filter),f})})}else return null}},Oh=class{constructor(e){this.name=Ke.EXT_MESH_GPU_INSTANCING,this.parser=e}createNodeMesh(e){let t=this.parser.json,n=t.nodes[e];if(!n.extensions||!n.extensions[this.name]||n.mesh===void 0)return null;let i=t.meshes[n.mesh];for(let l of i.primitives)if(l.mode!==An.TRIANGLES&&l.mode!==An.TRIANGLE_STRIP&&l.mode!==An.TRIANGLE_FAN&&l.mode!==void 0)return null;let o=n.extensions[this.name].attributes,a=[],c={};for(let l in o)a.push(this.parser.getDependency("accessor",o[l]).then(h=>(c[l]=h,c[l])));return a.length<1?null:(a.push(this.parser.createNodeMesh(e)),Promise.all(a).then(l=>{let h=l.pop(),u=h.isGroup?h.children:[h],d=l[0].count,f=[];for(let g of u){let _=new De,m=new A,p=new It,S=new A(1,1,1),E=new Vr(g.geometry,g.material,d);for(let y=0;y<d;y++)c.TRANSLATION&&m.fromBufferAttribute(c.TRANSLATION,y),c.ROTATION&&p.fromBufferAttribute(c.ROTATION,y),c.SCALE&&S.fromBufferAttribute(c.SCALE,y),E.setMatrixAt(y,_.compose(m,p,S));for(let y in c)if(y==="_COLOR_0"){let w=c[y];E.instanceColor=new ki(w.array,w.itemSize,w.normalized)}else y!=="TRANSLATION"&&y!=="ROTATION"&&y!=="SCALE"&&g.geometry.setAttribute(y,c[y]);pt.prototype.copy.call(E,g),this.parser.assignFinalMaterial(E),f.push(E)}return h.isGroup?(h.clear(),h.add(...f),h):f[0]}))}},Of="glTF",mo=12,Lf={JSON:1313821514,BIN:5130562},Fh=class{constructor(e){this.name=Ke.KHR_BINARY_GLTF,this.content=null,this.body=null;let t=new DataView(e,0,mo),n=new TextDecoder;if(this.header={magic:n.decode(new Uint8Array(e.slice(0,4))),version:t.getUint32(4,!0),length:t.getUint32(8,!0)},this.header.magic!==Of)throw new Error("THREE.GLTFLoader: Unsupported glTF-Binary header.");if(this.header.version<2)throw new Error("THREE.GLTFLoader: Legacy binary file detected.");let i=this.header.length-mo,r=new DataView(e,mo),o=0;for(;o<i;){let a=r.getUint32(o,!0);o+=4;let c=r.getUint32(o,!0);if(o+=4,c===Lf.JSON){let l=new Uint8Array(e,mo+o,a);this.content=n.decode(l)}else if(c===Lf.BIN){let l=mo+o;this.body=e.slice(l,l+a)}o+=a}if(this.content===null)throw new Error("THREE.GLTFLoader: JSON content not found.")}},Bh=class{constructor(e,t){if(!t)throw new Error("THREE.GLTFLoader: No DRACOLoader instance provided.");this.name=Ke.KHR_DRACO_MESH_COMPRESSION,this.json=e,this.dracoLoader=t,this.dracoLoader.preload()}decodePrimitive(e,t){let n=this.json,i=this.dracoLoader,r=e.extensions[this.name].bufferView,o=e.extensions[this.name].attributes,a={},c={},l={};for(let h in o){let u=Vh[h]||h.toLowerCase();a[u]=o[h]}for(let h in e.attributes){let u=Vh[h]||h.toLowerCase();if(o[h]!==void 0){let d=n.accessors[e.attributes[h]],f=fr[d.componentType];l[u]=f.name,c[u]=d.normalized===!0}}return t.getDependency("bufferView",r).then(function(h){return new Promise(function(u,d){i.decodeDracoFile(h,function(f){for(let g in f.attributes){let _=f.attributes[g],m=c[g];m!==void 0&&(_.normalized=m)}u(f)},a,l,Vt,d)})})}},kh=class{constructor(){this.name=Ke.KHR_TEXTURE_TRANSFORM}extendTexture(e,t){return(t.texCoord===void 0||t.texCoord===e.channel)&&t.offset===void 0&&t.rotation===void 0&&t.scale===void 0||(e=e.clone(),t.texCoord!==void 0&&(e.channel=t.texCoord),t.offset!==void 0&&e.offset.fromArray(t.offset),t.rotation!==void 0&&(e.rotation=t.rotation),t.scale!==void 0&&e.repeat.fromArray(t.scale),e.needsUpdate=!0),e}},zh=class{constructor(){this.name=Ke.KHR_MESH_QUANTIZATION}},pc=class extends _i{constructor(e,t,n,i){super(e,t,n,i)}copySampleValue_(e){let t=this.resultBuffer,n=this.sampleValues,i=this.valueSize,r=e*i*3+i;for(let o=0;o!==i;o++)t[o]=n[r+o];return t}interpolate_(e,t,n,i){let r=this.resultBuffer,o=this.sampleValues,a=this.valueSize,c=a*2,l=a*3,h=i-t,u=(n-t)/h,d=u*u,f=d*u,g=e*l,_=g-l,m=-2*f+3*d,p=f-d,S=1-m,E=p-d+u;for(let y=0;y!==a;y++){let w=o[_+y+a],C=o[_+y+c]*h,R=o[g+y+a],N=o[g+y]*h;r[y]=S*w+E*C+m*R+p*N}return r}},dy=new It,Hh=class extends pc{interpolate_(e,t,n,i){let r=super.interpolate_(e,t,n,i);return dy.fromArray(r).normalize().toArray(r),r}},An={FLOAT:5126,FLOAT_MAT3:35675,FLOAT_MAT4:35676,FLOAT_VEC2:35664,FLOAT_VEC3:35665,FLOAT_VEC4:35666,LINEAR:9729,REPEAT:10497,SAMPLER_2D:35678,POINTS:0,LINES:1,LINE_LOOP:2,LINE_STRIP:3,TRIANGLES:4,TRIANGLE_STRIP:5,TRIANGLE_FAN:6,UNSIGNED_BYTE:5121,UNSIGNED_SHORT:5123},fr={5120:Int8Array,5121:Uint8Array,5122:Int16Array,5123:Uint16Array,5125:Uint32Array,5126:Float32Array},Nf={9728:Ct,9729:Ht,9984:ir,9985:Wi,9986:Ei,9987:gn},Df={33071:En,33648:Ui,10497:jn},gh={SCALAR:1,VEC2:2,VEC3:3,VEC4:4,MAT2:4,MAT3:9,MAT4:16},Vh={POSITION:"position",NORMAL:"normal",TANGENT:"tangent",TEXCOORD_0:"uv",TEXCOORD_1:"uv1",TEXCOORD_2:"uv2",TEXCOORD_3:"uv3",COLOR_0:"color",WEIGHTS_0:"skinWeight",JOINTS_0:"skinIndex"},qi={scale:"scale",translation:"position",rotation:"quaternion",weights:"morphTargetInfluences"},fy={CUBICSPLINE:void 0,LINEAR:mi,STEP:pi},_h={OPAQUE:"OPAQUE",MASK:"MASK",BLEND:"BLEND"};function py(s){return s.DefaultMaterial===void 0&&(s.DefaultMaterial=new kn({color:16777215,emissive:0,metalness:1,roughness:1,transparent:!1,depthTest:!0,side:mn})),s.DefaultMaterial}function ys(s,e,t){for(let n in t.extensions)s[n]===void 0&&(e.userData.gltfExtensions=e.userData.gltfExtensions||{},e.userData.gltfExtensions[n]=t.extensions[n])}function si(s,e){e.extras!==void 0&&(typeof e.extras=="object"?Object.assign(s.userData,e.extras):console.warn("THREE.GLTFLoader: Ignoring primitive type .extras, "+e.extras))}function my(s,e,t){let n=!1,i=!1,r=!1;for(let l=0,h=e.length;l<h;l++){let u=e[l];if(u.POSITION!==void 0&&(n=!0),u.NORMAL!==void 0&&(i=!0),u.COLOR_0!==void 0&&(r=!0),n&&i&&r)break}if(!n&&!i&&!r)return Promise.resolve(s);let o=[],a=[],c=[];for(let l=0,h=e.length;l<h;l++){let u=e[l];if(n){let d=u.POSITION!==void 0?t.getDependency("accessor",u.POSITION):s.attributes.position;o.push(d)}if(i){let d=u.NORMAL!==void 0?t.getDependency("accessor",u.NORMAL):s.attributes.normal;a.push(d)}if(r){let d=u.COLOR_0!==void 0?t.getDependency("accessor",u.COLOR_0):s.attributes.color;c.push(d)}}return Promise.all([Promise.all(o),Promise.all(a),Promise.all(c)]).then(function(l){let h=l[0],u=l[1],d=l[2];return n&&(s.morphAttributes.position=h),i&&(s.morphAttributes.normal=u),r&&(s.morphAttributes.color=d),s.morphTargetsRelative=!0,s})}function gy(s,e){if(s.updateMorphTargets(),e.weights!==void 0)for(let t=0,n=e.weights.length;t<n;t++)s.morphTargetInfluences[t]=e.weights[t];if(e.extras&&Array.isArray(e.extras.targetNames)){let t=e.extras.targetNames;if(s.morphTargetInfluences.length===t.length){s.morphTargetDictionary={};for(let n=0,i=t.length;n<i;n++)s.morphTargetDictionary[t[n]]=n}else console.warn("THREE.GLTFLoader: Invalid extras.targetNames length. Ignoring names.")}}function _y(s){let e,t=s.extensions&&s.extensions[Ke.KHR_DRACO_MESH_COMPRESSION];if(t?e="draco:"+t.bufferView+":"+t.indices+":"+xh(t.attributes):e=s.indices+":"+xh(s.attributes)+":"+s.mode,s.targets!==void 0)for(let n=0,i=s.targets.length;n<i;n++)e+=":"+xh(s.targets[n]);return e}function xh(s){let e="",t=Object.keys(s).sort();for(let n=0,i=t.length;n<i;n++)e+=t[n]+":"+s[t[n]]+";";return e}function Gh(s){switch(s){case Int8Array:return 1/127;case Uint8Array:return 1/255;case Int16Array:return 1/32767;case Uint16Array:return 1/65535;default:throw new Error("THREE.GLTFLoader: Unsupported normalized accessor component type.")}}function xy(s){return s.search(/\.jpe?g($|\?)/i)>0||s.search(/^data\:image\/jpeg/)===0?"image/jpeg":s.search(/\.webp($|\?)/i)>0||s.search(/^data\:image\/webp/)===0?"image/webp":s.search(/\.ktx2($|\?)/i)>0||s.search(/^data\:image\/ktx2/)===0?"image/ktx2":"image/png"}var yy=new De,Wh=class{constructor(e={},t={}){this.json=e,this.extensions={},this.plugins={},this.options=t,this.cache=new uy,this.associations=new Map,this.primitiveCache={},this.nodeCache={},this.meshCache={refs:{},uses:{}},this.cameraCache={refs:{},uses:{}},this.lightCache={refs:{},uses:{}},this.sourceCache={},this.textureCache={},this.nodeNamesUsed={};let n=!1,i=-1,r=!1,o=-1;if(typeof navigator<"u"){let a=navigator.userAgent;n=/^((?!chrome|android).)*safari/i.test(a)===!0;let c=a.match(/Version\/(\d+)/);i=n&&c?parseInt(c[1],10):-1,r=a.indexOf("Firefox")>-1,o=r?a.match(/Firefox\/([0-9]+)\./)[1]:-1}typeof createImageBitmap>"u"||n&&i<17||r&&o<98?this.textureLoader=new Zr(this.options.manager):this.textureLoader=new to(this.options.manager),this.textureLoader.setCrossOrigin(this.options.crossOrigin),this.textureLoader.setRequestHeader(this.options.requestHeader),this.fileLoader=new tr(this.options.manager),this.fileLoader.setResponseType("arraybuffer"),this.options.crossOrigin==="use-credentials"&&this.fileLoader.setWithCredentials(!0)}setExtensions(e){this.extensions=e}setPlugins(e){this.plugins=e}parse(e,t){let n=this,i=this.json,r=this.extensions;this.cache.removeAll(),this.nodeCache={},this._invokeAll(function(o){return o._markDefs&&o._markDefs()}),Promise.all(this._invokeAll(function(o){return o.beforeRoot&&o.beforeRoot()})).then(function(){return Promise.all([n.getDependencies("scene"),n.getDependencies("animation"),n.getDependencies("camera")])}).then(function(o){let a={scene:o[0][i.scene||0],scenes:o[0],animations:o[1],cameras:o[2],asset:i.asset,parser:n,userData:{}};return ys(r,a,i),si(a,i),Promise.all(n._invokeAll(function(c){return c.afterRoot&&c.afterRoot(a)})).then(function(){for(let c of a.scenes)c.updateMatrixWorld();e(a)})}).catch(t)}_markDefs(){let e=this.json.nodes||[],t=this.json.skins||[],n=this.json.meshes||[];for(let i=0,r=t.length;i<r;i++){let o=t[i].joints;for(let a=0,c=o.length;a<c;a++)e[o[a]].isBone=!0}for(let i=0,r=e.length;i<r;i++){let o=e[i];o.mesh!==void 0&&(this._addNodeRef(this.meshCache,o.mesh),o.skin!==void 0&&(n[o.mesh].isSkinnedMesh=!0)),o.camera!==void 0&&this._addNodeRef(this.cameraCache,o.camera)}}_addNodeRef(e,t){t!==void 0&&(e.refs[t]===void 0&&(e.refs[t]=e.uses[t]=0),e.refs[t]++)}_getNodeRef(e,t,n){if(e.refs[t]<=1)return n;let i=n.clone(),r=(o,a)=>{let c=this.associations.get(o);c!=null&&this.associations.set(a,c);for(let[l,h]of o.children.entries())r(h,a.children[l])};return r(n,i),i.name+="_instance_"+e.uses[t]++,i}_invokeOne(e){let t=Object.values(this.plugins);t.push(this);for(let n=0;n<t.length;n++){let i=e(t[n]);if(i)return i}return null}_invokeAll(e){let t=Object.values(this.plugins);t.unshift(this);let n=[];for(let i=0;i<t.length;i++){let r=e(t[i]);r&&n.push(r)}return n}getDependency(e,t){let n=e+":"+t,i=this.cache.get(n);if(!i){switch(e){case"scene":i=this.loadScene(t);break;case"node":i=this._invokeOne(function(r){return r.loadNode&&r.loadNode(t)});break;case"mesh":i=this._invokeOne(function(r){return r.loadMesh&&r.loadMesh(t)});break;case"accessor":i=this.loadAccessor(t);break;case"bufferView":i=this._invokeOne(function(r){return r.loadBufferView&&r.loadBufferView(t)});break;case"buffer":i=this.loadBuffer(t);break;case"material":i=this._invokeOne(function(r){return r.loadMaterial&&r.loadMaterial(t)});break;case"texture":i=this._invokeOne(function(r){return r.loadTexture&&r.loadTexture(t)});break;case"skin":i=this.loadSkin(t);break;case"animation":i=this._invokeOne(function(r){return r.loadAnimation&&r.loadAnimation(t)});break;case"camera":i=this.loadCamera(t);break;default:if(i=this._invokeOne(function(r){return r!=this&&r.getDependency&&r.getDependency(e,t)}),!i)throw new Error("Unknown type: "+e);break}this.cache.add(n,i)}return i}getDependencies(e){let t=this.cache.get(e);if(!t){let n=this,i=this.json[e+(e==="mesh"?"es":"s")]||[];t=Promise.all(i.map(function(r,o){return n.getDependency(e,o)})),this.cache.add(e,t)}return t}loadBuffer(e){let t=this.json.buffers[e],n=this.fileLoader;if(t.type&&t.type!=="arraybuffer")throw new Error("THREE.GLTFLoader: "+t.type+" buffer type is not supported.");if(t.uri===void 0&&e===0)return Promise.resolve(this.extensions[Ke.KHR_BINARY_GLTF].body);let i=this.options;return new Promise(function(r,o){n.load(bi.resolveURL(t.uri,i.path),r,void 0,function(){o(new Error('THREE.GLTFLoader: Failed to load buffer "'+t.uri+'".'))})})}loadBufferView(e){let t=this.json.bufferViews[e];return this.getDependency("buffer",t.buffer).then(function(n){let i=t.byteLength||0,r=t.byteOffset||0;return n.slice(r,r+i)})}loadAccessor(e){let t=this,n=this.json,i=this.json.accessors[e];if(i.bufferView===void 0&&i.sparse===void 0){let o=gh[i.type],a=fr[i.componentType],c=i.normalized===!0,l=new a(i.count*o);return Promise.resolve(new at(l,o,c))}let r=[];return i.bufferView!==void 0?r.push(this.getDependency("bufferView",i.bufferView)):r.push(null),i.sparse!==void 0&&(r.push(this.getDependency("bufferView",i.sparse.indices.bufferView)),r.push(this.getDependency("bufferView",i.sparse.values.bufferView))),Promise.all(r).then(function(o){let a=o[0],c=gh[i.type],l=fr[i.componentType],h=l.BYTES_PER_ELEMENT,u=h*c,d=i.byteOffset||0,f=i.bufferView!==void 0?n.bufferViews[i.bufferView].byteStride:void 0,g=i.normalized===!0,_,m;if(f&&f!==u){let p=Math.floor(d/f),S="InterleavedBuffer:"+i.bufferView+":"+i.componentType+":"+p+":"+i.count,E=t.cache.get(S);E||(_=new l(a,p*f,i.count*f/h),E=new rs(_,f/h),t.cache.add(S,E)),m=new Bi(E,c,d%f/h,g)}else a===null?_=new l(i.count*c):_=new l(a,d,i.count*c),m=new at(_,c,g);if(i.sparse!==void 0){let p=gh.SCALAR,S=fr[i.sparse.indices.componentType],E=i.sparse.indices.byteOffset||0,y=i.sparse.values.byteOffset||0,w=new S(o[1],E,i.sparse.count*p),C=new l(o[2],y,i.sparse.count*c);a!==null&&(m=new at(m.array.slice(),m.itemSize,m.normalized)),m.normalized=!1;for(let R=0,N=w.length;R<N;R++){let M=w[R];if(m.setX(M,C[R*c]),c>=2&&m.setY(M,C[R*c+1]),c>=3&&m.setZ(M,C[R*c+2]),c>=4&&m.setW(M,C[R*c+3]),c>=5)throw new Error("THREE.GLTFLoader: Unsupported itemSize in sparse BufferAttribute.")}m.normalized=g}return m})}loadTexture(e){let t=this.json,n=this.options,r=t.textures[e].source,o=t.images[r],a=this.textureLoader;if(o.uri){let c=n.manager.getHandler(o.uri);c!==null&&(a=c)}return this.loadTextureImage(e,r,a)}loadTextureImage(e,t,n){let i=this,r=this.json,o=r.textures[e],a=r.images[t],c=(a.uri||a.bufferView)+":"+o.sampler;if(this.textureCache[c])return this.textureCache[c];let l=this.loadImageSource(t,n).then(function(h){h.flipY=!1,h.name=o.name||a.name||"",h.name===""&&typeof a.uri=="string"&&a.uri.startsWith("data:image/")===!1&&(h.name=a.uri);let d=(r.samplers||{})[o.sampler]||{};return h.magFilter=Nf[d.magFilter]||Ht,h.minFilter=Nf[d.minFilter]||gn,h.wrapS=Df[d.wrapS]||jn,h.wrapT=Df[d.wrapT]||jn,h.generateMipmaps=!h.isCompressedTexture&&h.minFilter!==Ct&&h.minFilter!==Ht,i.associations.set(h,{textures:e}),h}).catch(function(){return null});return this.textureCache[c]=l,l}loadImageSource(e,t){let n=this,i=this.json,r=this.options;if(this.sourceCache[e]!==void 0)return this.sourceCache[e].then(u=>u.clone());let o=i.images[e],a=self.URL||self.webkitURL,c=o.uri||"",l=!1;if(o.bufferView!==void 0)c=n.getDependency("bufferView",o.bufferView).then(function(u){l=!0;let d=new Blob([u],{type:o.mimeType});return c=a.createObjectURL(d),c});else if(o.uri===void 0)throw new Error("THREE.GLTFLoader: Image "+e+" is missing URI and bufferView");let h=Promise.resolve(c).then(function(u){return new Promise(function(d,f){let g=d;t.isImageBitmapLoader===!0&&(g=function(_){let m=new At(_);m.needsUpdate=!0,d(m)}),t.load(bi.resolveURL(u,r.path),g,void 0,f)})}).then(function(u){return l===!0&&a.revokeObjectURL(c),si(u,o),u.userData.mimeType=o.mimeType||xy(o.uri),u}).catch(function(u){throw console.error("THREE.GLTFLoader: Couldn't load texture",c),u});return this.sourceCache[e]=h,h}assignTexture(e,t,n,i){let r=this;return this.getDependency("texture",n.index).then(function(o){if(!o)return null;if(n.texCoord!==void 0&&n.texCoord>0&&(o=o.clone(),o.channel=n.texCoord),r.extensions[Ke.KHR_TEXTURE_TRANSFORM]){let a=n.extensions!==void 0?n.extensions[Ke.KHR_TEXTURE_TRANSFORM]:void 0;if(a){let c=r.associations.get(o);o=r.extensions[Ke.KHR_TEXTURE_TRANSFORM].extendTexture(o,a),r.associations.set(o,c)}}return i!==void 0&&(o.colorSpace=i),e[t]=o,o})}assignFinalMaterial(e){let t=e.geometry,n=e.material,i=t.attributes.tangent===void 0,r=t.attributes.color!==void 0,o=t.attributes.normal===void 0;if(e.isPoints){let a="PointsMaterial:"+n.uuid,c=this.cache.get(a);c||(c=new er,Yt.prototype.copy.call(c,n),c.color.copy(n.color),c.map=n.map,c.sizeAttenuation=!1,this.cache.add(a,c)),n=c}else if(e.isLine){let a="LineBasicMaterial:"+n.uuid,c=this.cache.get(a);c||(c=new Tn,Yt.prototype.copy.call(c,n),c.color.copy(n.color),c.map=n.map,this.cache.add(a,c)),n=c}if(i||r||o){let a="ClonedMaterial:"+n.uuid+":";i&&(a+="derivative-tangents:"),r&&(a+="vertex-colors:"),o&&(a+="flat-shading:");let c=this.cache.get(a);c||(c=n.clone(),r&&(c.vertexColors=!0),o&&(c.flatShading=!0),i&&(c.normalScale&&(c.normalScale.y*=-1),c.clearcoatNormalScale&&(c.clearcoatNormalScale.y*=-1)),this.cache.add(a,c),this.associations.set(c,this.associations.get(n))),n=c}e.material=n}getMaterialType(){return kn}loadMaterial(e){let t=this,n=this.json,i=this.extensions,r=n.materials[e],o,a={},c=r.extensions||{},l=[];if(c[Ke.KHR_MATERIALS_UNLIT]){let u=i[Ke.KHR_MATERIALS_UNLIT];o=u.getMaterialType(),l.push(u.extendParams(a,r,t))}else{let u=r.pbrMetallicRoughness||{};if(a.color=new Re(1,1,1),a.opacity=1,Array.isArray(u.baseColorFactor)){let d=u.baseColorFactor;a.color.setRGB(d[0],d[1],d[2],Vt),a.opacity=d[3]}u.baseColorTexture!==void 0&&l.push(t.assignTexture(a,"map",u.baseColorTexture,_t)),a.metalness=u.metallicFactor!==void 0?u.metallicFactor:1,a.roughness=u.roughnessFactor!==void 0?u.roughnessFactor:1,u.metallicRoughnessTexture!==void 0&&(l.push(t.assignTexture(a,"metalnessMap",u.metallicRoughnessTexture)),l.push(t.assignTexture(a,"roughnessMap",u.metallicRoughnessTexture))),o=this._invokeOne(function(d){return d.getMaterialType&&d.getMaterialType(e)}),l.push(Promise.all(this._invokeAll(function(d){return d.extendMaterialParams&&d.extendMaterialParams(e,a)})))}r.doubleSided===!0&&(a.side=Kt);let h=r.alphaMode||_h.OPAQUE;if(h===_h.BLEND?(a.transparent=!0,a.depthWrite=!1):(a.transparent=!1,h===_h.MASK&&(a.alphaTest=r.alphaCutoff!==void 0?r.alphaCutoff:.5)),r.normalTexture!==void 0&&o!==jt&&(l.push(t.assignTexture(a,"normalMap",r.normalTexture)),a.normalScale=new Ee(1,1),r.normalTexture.scale!==void 0)){let u=r.normalTexture.scale;a.normalScale.set(u,u)}if(r.occlusionTexture!==void 0&&o!==jt&&(l.push(t.assignTexture(a,"aoMap",r.occlusionTexture)),r.occlusionTexture.strength!==void 0&&(a.aoMapIntensity=r.occlusionTexture.strength)),r.emissiveFactor!==void 0&&o!==jt){let u=r.emissiveFactor;a.emissive=new Re().setRGB(u[0],u[1],u[2],Vt)}return r.emissiveTexture!==void 0&&o!==jt&&l.push(t.assignTexture(a,"emissiveMap",r.emissiveTexture,_t)),Promise.all(l).then(function(){let u=new o(a);return r.name&&(u.name=r.name),si(u,r),t.associations.set(u,{materials:e}),r.extensions&&ys(i,u,r),u})}createUniqueName(e){let t=tt.sanitizeNodeName(e||"");return t in this.nodeNamesUsed?t+"_"+ ++this.nodeNamesUsed[t]:(this.nodeNamesUsed[t]=0,t)}loadGeometries(e){let t=this,n=this.extensions,i=this.primitiveCache;function r(a){return n[Ke.KHR_DRACO_MESH_COMPRESSION].decodePrimitive(a,t).then(function(c){return Uf(c,a,t)})}let o=[];for(let a=0,c=e.length;a<c;a++){let l=e[a],h=_y(l),u=i[h];if(u)o.push(u.promise);else{let d;l.extensions&&l.extensions[Ke.KHR_DRACO_MESH_COMPRESSION]?d=r(l):d=Uf(new xt,l,t),i[h]={primitive:l,promise:d},o.push(d)}}return Promise.all(o)}loadMesh(e){let t=this,n=this.json,i=this.extensions,r=n.meshes[e],o=r.primitives,a=[];for(let c=0,l=o.length;c<l;c++){let h=o[c].material===void 0?py(this.cache):this.getDependency("material",o[c].material);a.push(h)}return a.push(t.loadGeometries(o)),Promise.all(a).then(function(c){let l=c.slice(0,c.length-1),h=c[c.length-1],u=[];for(let f=0,g=h.length;f<g;f++){let _=h[f],m=o[f],p,S=l[f];if(m.mode===An.TRIANGLES||m.mode===An.TRIANGLE_STRIP||m.mode===An.TRIANGLE_FAN||m.mode===void 0)p=r.isSkinnedMesh===!0?new kr(_,S):new lt(_,S),p.isSkinnedMesh===!0&&p.normalizeSkinWeights(),m.mode===An.TRIANGLE_STRIP?p.geometry=mh(p.geometry,ho):m.mode===An.TRIANGLE_FAN&&(p.geometry=mh(p.geometry,cr));else if(m.mode===An.LINES)p=new zi(_,S);else if(m.mode===An.LINE_STRIP)p=new gi(_,S);else if(m.mode===An.LINE_LOOP)p=new os(_,S);else if(m.mode===An.POINTS)p=new Gr(_,S);else throw new Error("THREE.GLTFLoader: Primitive mode unsupported: "+m.mode);Object.keys(p.geometry.morphAttributes).length>0&&gy(p,r),p.name=t.createUniqueName(r.name||"mesh_"+e),si(p,r),m.extensions&&ys(i,p,m),t.assignFinalMaterial(p),u.push(p)}for(let f=0,g=u.length;f<g;f++)t.associations.set(u[f],{meshes:e,primitives:f});if(u.length===1)return r.extensions&&ys(i,u[0],r),u[0];let d=new Et;r.extensions&&ys(i,d,r),t.associations.set(d,{meshes:e});for(let f=0,g=u.length;f<g;f++)d.add(u[f]);return d})}loadCamera(e){let t,n=this.json.cameras[e],i=n[n.type];if(!i){console.warn("THREE.GLTFLoader: Missing camera parameters.");return}return n.type==="perspective"?t=new wt(Ti.radToDeg(i.yfov),i.aspectRatio||1,i.znear||1,i.zfar||2e6):n.type==="orthographic"&&(t=new hs(-i.xmag,i.xmag,i.ymag,-i.ymag,i.znear,i.zfar)),n.name&&(t.name=this.createUniqueName(n.name)),si(t,n),Promise.resolve(t)}loadSkin(e){let t=this.json.skins[e],n=[];for(let i=0,r=t.joints.length;i<r;i++)n.push(this._loadNodeShallow(t.joints[i]));return t.inverseBindMatrices!==void 0?n.push(this.getDependency("accessor",t.inverseBindMatrices)):n.push(null),Promise.all(n).then(function(i){let r=i.pop(),o=i,a=[],c=[];for(let l=0,h=o.length;l<h;l++){let u=o[l];if(u){a.push(u);let d=new De;r!==null&&d.fromArray(r.array,l*16),c.push(d)}else console.warn('THREE.GLTFLoader: Joint "%s" could not be found.',t.joints[l])}return new Hr(a,c)})}loadAnimation(e){let t=this.json,n=this,i=t.animations[e],r=i.name?i.name:"animation_"+e,o=[],a=[],c=[],l=[],h=[];for(let u=0,d=i.channels.length;u<d;u++){let f=i.channels[u],g=i.samplers[f.sampler],_=f.target,m=_.node,p=i.parameters!==void 0?i.parameters[g.input]:g.input,S=i.parameters!==void 0?i.parameters[g.output]:g.output;_.node!==void 0&&(o.push(this.getDependency("node",m)),a.push(this.getDependency("accessor",p)),c.push(this.getDependency("accessor",S)),l.push(g),h.push(_))}return Promise.all([Promise.all(o),Promise.all(a),Promise.all(c),Promise.all(l),Promise.all(h)]).then(function(u){let d=u[0],f=u[1],g=u[2],_=u[3],m=u[4],p=[];for(let E=0,y=d.length;E<y;E++){let w=d[E],C=f[E],R=g[E],N=_[E],M=m[E];if(w===void 0)continue;w.updateMatrix&&w.updateMatrix();let b=n._createAnimationTracks(w,C,R,N,M);if(b)for(let P=0;P<b.length;P++)p.push(b[P])}let S=new Kr(r,void 0,p);return si(S,i),S})}createNodeMesh(e){let t=this.json,n=this,i=t.nodes[e];return i.mesh===void 0?null:n.getDependency("mesh",i.mesh).then(function(r){let o=n._getNodeRef(n.meshCache,i.mesh,r);return i.weights!==void 0&&o.traverse(function(a){if(a.isMesh)for(let c=0,l=i.weights.length;c<l;c++)a.morphTargetInfluences[c]=i.weights[c]}),o})}loadNode(e){let t=this.json,n=this,i=t.nodes[e],r=n._loadNodeShallow(e),o=[],a=i.children||[];for(let l=0,h=a.length;l<h;l++)o.push(n.getDependency("node",a[l]));let c=i.skin===void 0?Promise.resolve(null):n.getDependency("skin",i.skin);return Promise.all([r,Promise.all(o),c]).then(function(l){let h=l[0],u=l[1],d=l[2];d!==null&&h.traverse(function(f){f.isSkinnedMesh&&f.bind(d,yy)});for(let f=0,g=u.length;f<g;f++)h.add(u[f]);return h})}_loadNodeShallow(e){let t=this.json,n=this.extensions,i=this;if(this.nodeCache[e]!==void 0)return this.nodeCache[e];let r=t.nodes[e],o=r.name?i.createUniqueName(r.name):"",a=[],c=i._invokeOne(function(l){return l.createNodeMesh&&l.createNodeMesh(e)});return c&&a.push(c),r.camera!==void 0&&a.push(i.getDependency("camera",r.camera).then(function(l){return i._getNodeRef(i.cameraCache,r.camera,l)})),i._invokeAll(function(l){return l.createNodeAttachment&&l.createNodeAttachment(e)}).forEach(function(l){a.push(l)}),this.nodeCache[e]=Promise.all(a).then(function(l){let h;if(r.isBone===!0?h=new $s:l.length>1?h=new Et:l.length===1?h=l[0]:h=new pt,h!==l[0])for(let u=0,d=l.length;u<d;u++)h.add(l[u]);if(r.name&&(h.userData.name=r.name,h.name=o),si(h,r),r.extensions&&ys(n,h,r),r.matrix!==void 0){let u=new De;u.fromArray(r.matrix),h.applyMatrix4(u)}else r.translation!==void 0&&h.position.fromArray(r.translation),r.rotation!==void 0&&h.quaternion.fromArray(r.rotation),r.scale!==void 0&&h.scale.fromArray(r.scale);if(!i.associations.has(h))i.associations.set(h,{});else if(r.mesh!==void 0&&i.meshCache.refs[r.mesh]>1){let u=i.associations.get(h);i.associations.set(h,{...u})}return i.associations.get(h).nodes=e,h}),this.nodeCache[e]}loadScene(e){let t=this.extensions,n=this.json.scenes[e],i=this,r=new Et;n.name&&(r.name=i.createUniqueName(n.name)),si(r,n),n.extensions&&ys(t,r,n);let o=n.nodes||[],a=[];for(let c=0,l=o.length;c<l;c++)a.push(i.getDependency("node",o[c]));return Promise.all(a).then(function(c){for(let h=0,u=c.length;h<u;h++)r.add(c[h]);let l=h=>{let u=new Map;for(let[d,f]of i.associations)(d instanceof Yt||d instanceof At)&&u.set(d,f);return h.traverse(d=>{let f=i.associations.get(d);f!=null&&u.set(d,f)}),u};return i.associations=l(r),r})}_createAnimationTracks(e,t,n,i,r){let o=[],a=e.name?e.name:e.uuid,c=[];qi[r.path]===qi.weights?e.traverse(function(d){d.morphTargetInfluences&&c.push(d.name?d.name:d.uuid)}):c.push(a);let l;switch(qi[r.path]){case qi.weights:l=$n;break;case qi.rotation:l=Qn;break;case qi.translation:case qi.scale:l=ei;break;default:switch(n.itemSize){case 1:l=$n;break;case 2:case 3:default:l=ei;break}break}let h=i.interpolation!==void 0?fy[i.interpolation]:mi,u=this._getArrayFromAccessor(n);for(let d=0,f=c.length;d<f;d++){let g=new l(c[d]+"."+qi[r.path],t.array,u,h);i.interpolation==="CUBICSPLINE"&&this._createCubicSplineTrackInterpolant(g),o.push(g)}return o}_getArrayFromAccessor(e){let t=e.array;if(e.normalized){let n=Gh(t.constructor),i=new Float32Array(t.length);for(let r=0,o=t.length;r<o;r++)i[r]=t[r]*n;t=i}return t}_createCubicSplineTrackInterpolant(e){e.createInterpolant=function(n){let i=this instanceof Qn?Hh:pc;return new i(this.times,this.values,this.getValueSize()/3,n)},e.createInterpolant.isInterpolantFactoryMethodGLTFCubicSpline=!0}};function vy(s,e,t){let n=e.attributes,i=new Rt;if(n.POSITION!==void 0){let a=t.json.accessors[n.POSITION],c=a.min,l=a.max;if(c!==void 0&&l!==void 0){if(i.set(new A(c[0],c[1],c[2]),new A(l[0],l[1],l[2])),a.normalized){let h=Gh(fr[a.componentType]);i.min.multiplyScalar(h),i.max.multiplyScalar(h)}}else{console.warn("THREE.GLTFLoader: Missing min/max properties for accessor POSITION.");return}}else return;let r=e.targets;if(r!==void 0){let a=new A,c=new A;for(let l=0,h=r.length;l<h;l++){let u=r[l];if(u.POSITION!==void 0){let d=t.json.accessors[u.POSITION],f=d.min,g=d.max;if(f!==void 0&&g!==void 0){if(c.setX(Math.max(Math.abs(f[0]),Math.abs(g[0]))),c.setY(Math.max(Math.abs(f[1]),Math.abs(g[1]))),c.setZ(Math.max(Math.abs(f[2]),Math.abs(g[2]))),d.normalized){let _=Gh(fr[d.componentType]);c.multiplyScalar(_)}a.max(c)}else console.warn("THREE.GLTFLoader: Missing min/max properties for accessor POSITION.")}}i.expandByVector(a)}s.boundingBox=i;let o=new sn;i.getCenter(o.center),o.radius=i.min.distanceTo(i.max)/2,s.boundingSphere=o}function Uf(s,e,t){let n=e.attributes,i=[];function r(o,a){return t.getDependency("accessor",o).then(function(c){s.setAttribute(a,c)})}for(let o in n){let a=Vh[o]||o.toLowerCase();a in s.attributes||i.push(r(n[o],a))}if(e.indices!==void 0&&!s.index){let o=t.getDependency("accessor",e.indices).then(function(a){s.setIndex(a)});i.push(o)}return $e.workingColorSpace!==Vt&&"COLOR_0"in n&&console.warn(`THREE.GLTFLoader: Converting vertex colors from "srgb-linear" to "${$e.workingColorSpace}" not supported.`),si(s,e),vy(s,e,t),Promise.all(i).then(function(){return e.targets!==void 0?my(s,e.targets,t):s})}var mc=class{parse(e,t={}){t=Object.assign({binary:!1},t);let n=t.binary,i=[],r=0;e.traverse(function(p){if(p.isMesh){let S=p.geometry,E=S.index,y=S.getAttribute("position");r+=E!==null?E.count/3:y.count/3,i.push({object3d:p,geometry:S})}});let o,a=80;if(n===!0){let p=r*2+r*3*4*4+80+4,S=new ArrayBuffer(p);o=new DataView(S),o.setUint32(a,r,!0),a+=4}else o="",o+=`solid exported
`;let c=new A,l=new A,h=new A,u=new A,d=new A,f=new A;for(let p=0,S=i.length;p<S;p++){let E=i[p].object3d,y=i[p].geometry,w=y.index,C=y.getAttribute("position");if(w!==null)for(let R=0;R<w.count;R+=3){let N=w.getX(R+0),M=w.getX(R+1),b=w.getX(R+2);g(N,M,b,C,E)}else for(let R=0;R<C.count;R+=3){let N=R+0,M=R+1,b=R+2;g(N,M,b,C,E)}}return n===!1&&(o+=`endsolid exported
`),o;function g(p,S,E,y,w){c.fromBufferAttribute(y,p),l.fromBufferAttribute(y,S),h.fromBufferAttribute(y,E),w.isSkinnedMesh===!0&&(w.applyBoneTransform(p,c),w.applyBoneTransform(S,l),w.applyBoneTransform(E,h)),c.applyMatrix4(w.matrixWorld),l.applyMatrix4(w.matrixWorld),h.applyMatrix4(w.matrixWorld),_(c,l,h),m(c),m(l),m(h),n===!0?(o.setUint16(a,0,!0),a+=2):(o+=`		endloop
`,o+=`	endfacet
`)}function _(p,S,E){u.subVectors(E,S),d.subVectors(p,S),u.cross(d).normalize(),f.copy(u).normalize(),n===!0?(o.setFloat32(a,f.x,!0),a+=4,o.setFloat32(a,f.y,!0),a+=4,o.setFloat32(a,f.z,!0),a+=4):(o+="	facet normal "+f.x+" "+f.y+" "+f.z+`
`,o+=`		outer loop
`)}function m(p){n===!0?(o.setFloat32(a,p.x,!0),a+=4,o.setFloat32(a,p.y,!0),a+=4,o.setFloat32(a,p.z,!0),a+=4):o+="			vertex "+p.x+" "+p.y+" "+p.z+`
`}}};function Ff(s,e,t){let n=s.role?s.role==="hardware":s.name.toLowerCase().includes("coupling")||["brass","steel"].includes(s.material);return(e.boneOnly?s.role==="bone_proxy":s.region==="envelope"?t.envelope:n?t.joints:t.frame)&&(!e.isolated||!e.selected||s.id===e.selected)}function Bf(s,e,t){let n=[s.name,s.id,s.region,s.bone_id??"",...s.grouped_bone_ids??[]].join(" ").toLowerCase();return(!t||s.role==="bone_proxy")&&n.includes(e.toLowerCase())}function kf(s,e,t){s.updateMatrixWorld(!0);let n=new Et;for(let i of e){let r=i.userData.part;if(r.region==="envelope"||t&&r.role!=="bone_proxy")continue;let o=new lt(i.geometry,i.material);o.name=i.name,o.applyMatrix4(i.matrixWorld),o.userData={part_id:i.name,role:r.role??"separate_CAD",bone_id:r.bone_id??null,grouped_bone_ids:r.grouped_bone_ids??[],dimensional_fidelity:r.dimensional_fidelity??"unverified",physical_evidence:r.physical_evidence??"unmeasured",geometry_inputs:r.geometry_inputs??[],hand_layout_sha256:r.hand_layout_sha256??null,foot_layout_sha256:r.foot_layout_sha256??null,skull_layout_sha256:r.skull_layout_sha256??null,ear_hyoid_layout_sha256:r.ear_hyoid_layout_sha256??null,proxy_shape:r.proxy_shape??null,source_record:r.source_record??null,source_record_status:r.source_record_status??null,topology_neighbors:r.topology_neighbors??[],topology_source:r.topology_source??null,articulates_with:r.articulates_with??[],housing_bone_id:r.housing_bone_id??null,non_bone_connection:r.non_bone_connection??null,non_bone_support:r.non_bone_support??null,motion_implemented:r.motion_implemented??null,topology_parent:r.topology_parent??null,topology_child:r.topology_child??null,status:"reference/unreleased"},n.add(o)}return n}var $=s=>String(s).replace(/[&<>"']/g,e=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"})[e]);function Dt(s,e,t=""){return'<div class="table-scroll"><table class="'+$(t)+'"><thead><tr>'+s.map(n=>"<th>"+$(n)+"</th>").join("")+"</tr></thead><tbody>"+e.map(n=>"<tr>"+n.map(i=>"<td>"+$(i)+"</td>").join("")+"</tr>").join("")+"</tbody></table></div>"}function Xh(s,e){return typeof s=="string"&&s.startsWith("https://")?'<a href="'+$(s)+'" target="_blank" rel="noopener">'+$(e)+"</a>":$(e)}var Yi=s=>"<ul>"+s.map(e=>"<li>"+$(e)+"</li>").join("")+"</ul>",zf=s=>s==null?"unknown":typeof s=="object"?JSON.stringify(s):String(s),by=s=>s===null?"unknown":String(s);function Hf(s,e){let t=s.bones[e];if(!t)return"<p>Canonical engineering evidence unavailable for this component.</p>";let n=t.dimensions,i=n.values_mm??{},r=Object.entries(i).sort(([c],[l])=>c.localeCompare(l)).map(([c,l])=>[c,by(l),t.measurements[c]?"Source-backed tabular field":l===null?"Unresolved":"Declared value; evidence review open"]),o='<div class="bone-evidence"><h3>Canonical dimensions</h3><p>Declared map: <b>'+$(n.status)+"</b>. Declared values in mm; independent of this display\u2019s height and pose.</p>"+Dt(["FIELD","mm","EVIDENCE"],r,"dimension-table");o+="<p>Geometry: "+$(t.canonical_geometry?.status??"unresolved")+". Record validation: "+$(t.record_validation.status)+". Physical evidence unmeasured; fabrication release false.</p>",o+='<p class="small-note">Source record: '+$(t.source.path??"missing")+"<br>SHA-256: "+$(t.source.sha256??"unknown")+"</p>";for(let[c,l]of Object.entries(t.measurements)){let h=s.sources[l.source_id],u=l.native_measurement,d=l.derivation;o+='<details class="measurement-evidence"><summary>'+$(c)+" \xB7 measurement evidence</summary><p>"+Xh(h.locator,h.title)+"</p>"+Dt(["PROVENANCE","VALUE"],[["Source revision",h.revision],["DOI",h.doi],["Native parameter",u.parameter],["Native mean / mm",u.mean],["Published SD / mm",u.sd],["Study cohort",h.cohort],["Population subset",u.population_subset??"not separately recorded"],["Native side",u.side],["Landmark definition",l.landmark_definition],["Method locator",l.method_locator],["Derivation",d.method],["Dimensional meaning",d.meaning],["Unit scale",d.unit_scale],["Canonical registration",zf(d.canonical_registration)],["Side application",d.side_application??"native side"],["Side assumption",d.side_assumption??"none added"],["Evidence maturity",l.evidence_maturity],["Validation scope",l.validation_result],["Native snapshot",h.native_snapshot.path],["Native snapshot SHA-256",h.native_snapshot.sha256],["Measurement protocol SHA-256",s.inputs.measurement_protocols_sha256],["Publisher license",h.publisher_license],["Rights review",h.rights_review]])+"<p>"+$(l.uncertainty)+"</p><p>"+$(u.limitation??"")+"</p>"+Yi(h.limitations)+"</details>"}let a=n.metric_source_declarations??{};return Object.keys(i).some(c=>i[c]!==null&&!t.measurements[c])&&(o+="<details><summary>Other declared values / provenance</summary><p>Field-level measurement protocols are unavailable in this reviewed DIM-072 projection. Declaration completeness does not qualify dimensional fidelity.</p>"+Dt(["FIELD","DECLARED SOURCE"],Object.entries(a).map(([c,l])=>[c,zf(l)]))+"<p>Reference declarations:</p>"+Yi(t.reference_declarations.declarations)+"</details>"),t.embodiment_fit&&(o+="<p>"+$(t.embodiment_fit)+"</p>"),t.candidate_notes.length&&(o+=Yi(t.candidate_notes)),o+="<details><summary>Remaining engineering evidence</summary>"+Yi(t.evidence_needed)+"<p>Record blockers:</p>"+Yi(t.record_validation.blocker_categories)+"</details></div>",o}function Vf(s,e){let t=s.dimension_expansion,n=s.canonical_reference;return"<p><b>"+t.current_complete+"/"+t.inventory_count+" complete declared dimension maps</b>; "+t.improved_partial_ids.length+" improved partial maps and "+t.new_source_backed_field_count+" traced additions from PHY-DIM-072. "+t.remaining_to_target+' maps remain to the 72-map target.</p><p class="warning">Declared completeness counts supplied fields. Dimensional fidelity is unverified, anatomical geometry and articulation are unqualified, and physical evidence is unmeasured. Fabrication release remains false.</p>'+Dt(["ENGINEERING OBSERVATION","COUNT","MEANING"],Object.entries(s.metrics).map(([i,r])=>[i.replaceAll("_"," "),r.count+"/"+r.total,r.meaning]),"metric-table")+"<p>SOPHY canon "+$(n.version)+" \xB7 height "+n.height_mm+" mm \xB7 span "+n.span_mm+' mm. Source measurements have no adopted spatial registration.</p><label for="evidence-bone">Inspect a canonical identity</label><select id="evidence-bone" class="select">'+Object.values(s.bones).map(i=>'<option value="'+$(i.bone_id)+'">'+$(i.bone_id+" \xB7 "+i.name)+"</option>").join("")+'</select><div id="evidence-details"></div><details><summary>Recovery additions</summary><p>New complete declared maps:</p>'+Yi(t.new_complete_ids)+"<p>Improved partial maps:</p>"+Yi(t.improved_partial_ids)+"</details><details><summary>Package provenance</summary><p>Engineering baseline: "+Xh("https://github.com/SYSTEMS-OPERATOR/PHY/commit/"+s.engineering_revision,s.engineering_revision)+" \xB7 "+$(s.engineering_revision_time)+"</p>"+Dt(["INPUT","SHA-256"],[["Evidence inputs",s.input_sha256],["Package inputs",e.package_input_sha256],["Canonical records",s.inputs.canonical_records_sha256],["NASA catalog",s.inputs.nasa_catalog_sha256],["SOPHY canon",s.inputs.canon_sha256],["Relationship graph",s.inputs.relationship_graph_sha256]])+"<p>The revision names the reviewed engineering baseline. Content hashes identify the evidence and viewer shipped in this offline package.</p></details>"}var qh={unsuited:"Unsuited",suited_unpressurized:"Suited / unpressurized",suited_pressurized:"Suited / pressurized"};function Gf(s,e){return'<div id="nasa-motion-rows">'+s.ranges.filter(t=>t.condition===e).map(t=>'<details class="nasa-motion"><summary>'+$(t.joint_label.replaceAll("_"," ")+" \xB7 "+t.motion_label.replaceAll("_"," "))+(t.source_issue?" \xB7 SOURCE ISSUE":"")+"</summary>"+Dt(["PRINTED ENDPOINT","deg"],Object.entries(t.endpoints_as_printed_deg))+"<p>"+$(t.sign_convention)+". "+$(t.sex_scope)+".</p><p>Source "+$(t.source_id)+" \xB7 table "+t.table+" \xB7 PDF pages "+$(t.pdf_pages.join(", "))+" \xB7 printed pages "+$(t.printed_pages.join(", "))+". Mechanical axis, mechanical limit and combined pose feasibility are unresolved.</p>"+(t.source_issue?'<p class="warning">'+$(t.source_issue)+"</p>":"")+"</details>").join("")+"</div>"}function Wf(s){let e=s.nasa,t=s.nasa_counts;return"<p>Human positioning and range references \xB7 "+e.ranges.length+" motion pairs, "+t.anthropometry+" external anthropometric comparisons, "+t.source_issues+' source issues retained.</p><p class="warning">Human reference ranges require their stated conditions and measurement conventions. PHY mechanical limits, joint centers, coupled pose feasibility and flight compliance remain unqualified.</p><h3>Motion conditions</h3><label for="nasa-condition">Reference condition</label><select id="nasa-condition" class="select">'+Object.entries(qh).map(([n,i])=>'<option value="'+n+'">'+i+" \xB7 "+t.conditions[n]+" pairs</option>").join("")+'</select><p id="nasa-condition-count" aria-live="polite"></p><div id="nasa-motion"></div><h3>Positioning and measurement conventions</h3>'+Yi(e.interpretation_rules)+"<p>Studio pose controls remain visual exploration around proposed centers. An external measurement endpoint requires an explicit anatomical definition before it can inform a mechanical datum.</p><h3>External anthropometric comparisons</h3>"+Dt(["SOURCE MEASUREMENT","PRINTED cm","CONVERTED mm","POPULATION / MAPPING"],e.anthropometry.map(n=>[n.source_label,n.values_cm.join(" to "),n.values_mm.join(" to "),n.population_scope+"; "+n.mapping_status]))+"<h3>Sources and applicability</h3>"+e.sources.map(n=>"<details><summary>"+$(n.title)+"</summary><p>"+Xh(n.uri,"Official NASA source")+" \xB7 revision "+$(n.revision)+(n.approved_on?" \xB7 approved "+$(n.approved_on):"")+"</p><p>Reference scope: "+$(n.supports.join(", "))+".</p><p>Excluded qualification claims: "+$(n.does_not_qualify.join(", "))+'.</p><p class="small-note">Recorded source hash: '+$(n.sha256??"not recorded for the web page")+". PDFs are not embedded or automatically acquired.</p></details>").join("")}var My=[["Legacy anatomical declarations",["legacy_parent","legacy_adjacency","legacy_articulation"]],["Source-backed qualitative articulations",["qualitative_articulation"]],["Structural region membership",["structural_membership"]],["Operational signal links \xB7 runtime behavior",["operational_signal"]]];function Sy(s,e){return(s?.relationships??[]).filter(t=>t.participants.some(n=>n.bone_id===e||n.resolution.candidates?.includes(e)||n.runtime_resolution?.candidates?.includes(e)))}function Xf(s){return s.status+(s.candidates?.length?" \xB7 candidates: "+s.candidates.join(", "):"")}function qf(s,e,t=null){if(!s)return"<p>Relationship snapshot unavailable.</p>";let n=Sy(s,e),i=new Map(s.sources.map(a=>[a.source_id,a])),r='<section class="relationships" data-relationship-bone="'+$(e)+'"><h3>Relationships</h3><p class="small-note">Declarations, qualitative anatomy, regional participation and runtime links have different meanings. None supplies a mechanical joint or a fabrication release.</p>';for(let[a,c]of My){let l=n.filter(h=>c.includes(h.kind));r+='<div class="relationship-group"><h4>'+$(a)+' <span class="tag">'+l.length+"</span></h4>",l.length||(r+='<p class="quiet">None declared in this snapshot.</p>');for(let h of l){r+='<details class="relationship-card" data-relationship-id="'+$(h.relationship_id)+'" data-relationship-kind="'+$(h.kind)+'"><summary>'+$(h.name??h.semantic_role.replaceAll("_"," "))+" \xB7 "+$(h.resolution_status)+"</summary>",r+='<p class="small-note">'+$(h.relationship_id)+"<br>Role: "+$(h.semantic_role)+'</p><ul class="relationship-participants">';for(let d of h.participants){let f=d.bone_id!==null&&(t===null||t.has(d.bone_id));r+="<li><span>"+$(d.role.replaceAll("_"," "))+": </span>"+(d.bone_id===null?"<span>unresolved target</span>":'<button type="button" class="related-bone" data-related-bone="'+$(d.bone_id)+'"'+(f?"":' disabled title="No proxy for this identity in the current model"')+">"+$(d.bone_id)+"</button>")+'<p class="small-note">'+$(d.matching_policy)+": "+$(Xf(d.resolution))+(d.runtime_resolution?"<br>Runtime legacy matching: "+$(Xf(d.runtime_resolution)):"")+"</p></li>"}r+="</ul>",h.kind==="qualitative_articulation"&&(r+=Dt(["EVIDENCE","DECLARATION"],[["Biological classification",h.biological_classification],["Qualitative motion",h.qualitative_motion.join(", ")],["Non-bone participation",(h.non_bone_components??[]).join(", ")],["Engineering definition","unresolved (null)"]])),h.kind==="structural_membership"&&(r+='<p class="small-note">Membership describes contributions to a region. It does not declare pairwise contact. Spatial definition: unresolved (null).</p>'),h.kind==="operational_signal"&&(r+='<p class="small-note">Observed runtime link only; no biological or mechanical articulation is inferred.</p>'),h.kind==="legacy_articulation"&&h.original_declaration?.joint_type&&(r+="<p>Legacy joint label: <b>"+$(h.original_declaration.joint_type)+"</b>. Engineering definition: unresolved (null).</p>"),r+='<details class="relationship-original"><summary>Original declaration</summary><pre>'+$(JSON.stringify(h.original_declaration,null,2))+"</pre></details>";let u=h.provenance;r+='<details class="relationship-evidence"><summary>Source and evidence</summary><p class="small-note">'+$(u.source_path??"Supplied snapshot")+" \xB7 "+$(u.field??u.catalog_locator??"")+(u.reference_locator?"<br>"+$(u.reference_locator):"")+"<br>Value SHA-256: "+$(u.value_sha256)+"<br>File SHA-256: "+$(u.source_file_sha256??"not asserted")+"</p>",u.source_correspondence_only&&(r+='<p class="small-note">Source-file correspondence; the field value is fingerprinted independently.</p>');for(let d of Array.isArray(u.source_ids)?u.source_ids:[]){let f=i.get(d);r+='<p class="small-note">'+$(d)+(f?"<br>"+$(f.title)+"<br>"+$(f.review_scope):"")+"</p>",f?.url?.startsWith("https://")&&(r+='<a href="'+$(f.url)+'" target="_blank" rel="noopener">Reference declaration \u2197</a>')}u.references!==void 0&&(r+="<pre>"+$(JSON.stringify(u.references,null,2))+"</pre>"),r+="</details></details>"}r+="</div>"}let o=s.diagnostics.parent_cycles.filter(a=>a.includes(e));return o.length&&(r+='<p class="warning">Declared parent cycle: '+$(JSON.stringify(o))+". Anatomical adjacency loops are checked separately.</p>"),r+"</section>"}var Vn=s=>["nih_native_preview","nih_native_original"].includes(s?.kind);async function Yh(s){let e=Uint8Array.from(atob(s.glb_base64),i=>i.charCodeAt(0)),t=new DataView(e.buffer);if(e.length!==s.derivative.preview_bytes||t.getUint32(0,!0)!==1179937895||t.getUint32(4,!0)!==2||t.getUint32(8,!0)!==e.length)throw new Error("Pinned preview length/header mismatch");if([...new Uint8Array(await crypto.subtle.digest("SHA-256",e))].map(i=>i.toString(16).padStart(2,"0")).join("")!==s.derivative.preview_sha256)throw new Error("Pinned preview checksum mismatch");return e}function jh(s){let e=s.landmark_packet;return{kind:"PHY_PELVIS_PREVIEW_DRAFT_V1",packet_id:e.packet_id,packet_sha256:s.landmark_packet_sha256,reference_revision:s.revision,source_sha256:e.source.sha256,preview_sha256:e.preview.sha256,annotations:[],accepted_landmarks:null,canonical_transform:null,fabrication_released:!1}}function Yf(s,e,t){let n=s.object,i=n.userData.part;if(!e.preview_mesh_names.includes(i.id))throw new Error("Choose the specified cortical surface: "+e.preview_mesh_names.join(", "));let r=n.geometry,o=r.index,a=r.attributes.position,c=s.faceIndex;if(!Number.isInteger(c)||!o)throw new Error("Preview triangle lineage unavailable");let l=[0,1,2].map(g=>o.getX(c*3+g)),h=l.map(g=>new A().fromBufferAttribute(a,g)),u=n.worldToLocal(s.point.clone()),d=new A;if(!pn.getBarycoord(u,...h,d))throw new Error("Degenerate preview triangle");let f=s.point.clone().multiplyScalar(.001).toArray();return{landmark_id:e.id,mesh_name:i.id,preview_node:i.preview_node,source_node:i.source_node,face_index:c,vertex_indices:l,barycentric:d.toArray(),native_point_m:f,original_face_index:null,anatomically_accepted:!1}}function jf(s){let e=s.native_references;if(!e)return"<p>NIH previews were omitted from this build. Rebuild with the explicit pinned PHY-REF checkout using --reference-root.</p>";let t=e.landmark_packet,n=s.models.filter(i=>i.kind==="nih_native_preview").map(i=>{let r=i.source,o=i.derivative;return"<h3>"+$(r.title)+"</h3><p>"+$(r.attribution)+'</p><p><a href="'+$(r.entry_url)+'" target="_blank" rel="noopener">NIH source entry</a> \xB7 <a href="'+$(r.license_url)+'" target="_blank" rel="noopener">CC BY 4.0</a> \xB7 entry '+r.entry_version+" / HRA "+$(r.reference_object_version)+'</p><p class="small-note">Original '+$(r.original_filename)+" \xB7 "+r.bytes+" bytes \xB7 SHA-256 "+$(r.sha256)+"<br>Preview "+o.preview_bytes+" bytes \xB7 "+o.preview_triangles+" triangles \xB7 SHA-256 "+$(o.preview_sha256)+"</p><p>"+$(o.recipe)+". "+$(o.qualification)+". "+$(o.textures_and_vertex_colors)+".</p>"}).join("");return'<p><b>Separate native reference views.</b> Source world Y-up / meters. Display conversion is 1000 mm/m; no centering, mirroring, anatomical axis mapping, stature normalization or registration to PHY is applied. Switch between NIH and SOPHY to compare their separate frames. Camera changes do not move geometry.</p><p class="warning">The skin cannot supply bone dimensions. Pelvic tissue segments represent four adult bones. Bounds, draft points and decimation samples are not qualified osteometry, joint datums or fabrication evidence.</p><p class="small-note">PHY-REF revision '+$(e.revision)+" \xB7 "+$(e.review_status)+"</p>"+n+"<h3>Sacrum\u2013left hemipelvis review</h3><p>Status: awaiting independent review. Accepted points, original-face correspondence, metric calibration, tolerances and canonical transform remain unknown.</p>"+Dt(["CANDIDATE","FEATURE / POINT RULE"],t.landmarks.map(i=>[i.id,i.definition+" "+i.review_rule]))+'<p>Choose a candidate in the pelvis inspector, arm point capture and click its specified cortical surface. Download the draft JSON, then run PHY-REF tools/validate_landmark_review.py --draft /path/to/draft.json. Geometric reconstruction checks the preview only. Two independent original-surface annotations and SYSOP-approved error budgets remain required.</p><p>NASA positioning/range references remain available in the NASA tab as comparison evidence; they do not set these bone points or mechanical limits.</p><p><a href="https://github.com/SYSTEMS-OPERATOR/PHY-REF/blob/'+$(e.revision)+'/reviews/PELVIS_LEFT_REVIEW.md" target="_blank" rel="noopener">Original-surface review gates and stop criteria</a></p>'}var Be=s=>{throw new Error(s)},Zh=s=>structuredClone(s),vs=s=>JSON.stringify(s,(e,t)=>t&&typeof t=="object"&&!Array.isArray(t)?Object.fromEntries(Object.keys(t).sort().map(n=>[n,t[n]])):t,2)+`
`,Ey=async s=>[...new Uint8Array(await crypto.subtle.digest("SHA-256",s))].map(e=>e.toString(16).padStart(2,"0")).join(""),Kf=s=>new TextEncoder().encode(vs(s)).length,go=(s,e,t)=>{(!s||Array.isArray(s)||typeof s!="object"||Object.keys(s).sort().join(" ")!==e.split(" ").sort().join(" "))&&Be("Unsupported fields: "+t)},pr=(s,e)=>{(!Array.isArray(s)||s.length!==3||s.some(t=>typeof t!="number"||!Number.isFinite(t)))&&Be("Invalid finite vector: "+e)},Kh=(s,e,t=8192)=>{(typeof s!="string"||s.length>t||s.includes("\0"))&&Be("Invalid text: "+e)},Gn=(s,e,t)=>((!Number.isInteger(s)||s<0||s>=e)&&Be("Invalid index: "+t),s);function $h(s,e=2097152){new TextEncoder().encode(s).length>e&&Be("JSON exceeds byte budget");let t=[];for(let n=0;n<s.length;n++){let i=s[n];if(i==='"'){let r=n,o=!1;for(;++n<s.length&&!(!o&&s[n]==='"');)!o&&s[n]==="\\"?o=!0:o=!1;let a=JSON.parse(s.slice(r,n+1)),c=n+1;for(;/\s/.test(s[c]??"")&&c<s.length;)c++;s[c]===":"&&t.at(-1)instanceof Set&&(t.at(-1).has(a)&&Be("Duplicate JSON field: "+a),t.at(-1).add(a))}else i==="{"?t.push(new Set):i==="["?t.push(null):(i==="}"||i==="]")&&t.pop();t.length>64&&Be("JSON nesting budget exceeded")}return JSON.parse(s)}var Jh=class{constructor(e){this.raw=e instanceof Uint8Array?e:new Uint8Array(e);let t=new DataView(this.raw.buffer,this.raw.byteOffset,this.raw.byteLength);(this.raw.length<28||t.getUint32(0,!0)!==1179937895||t.getUint32(4,!0)!==2||t.getUint32(8,!0)!==this.raw.length)&&Be("Invalid original GLB header/length");let n=t.getUint32(12,!0),i=20+n;(t.getUint32(16,!0)!==1313821514||i+8>this.raw.length||t.getUint32(i+4,!0)!==5130562||i+8+t.getUint32(i,!0)!==this.raw.length)&&Be("Unsupported original GLB chunks"),this.doc=$h(new TextDecoder().decode(this.raw.subarray(20,i)),this.raw.length),this.binStart=i+8,this.view=t;let r=this.doc;(r.asset?.version!=="2.0"||r.extensionsRequired?.length||r.skins?.length||r.animations?.length||r.buffers?.length!==1||r.buffers[0].uri)&&Be("Only static embedded original geometry is supported"),this.parents=new Map,this.worlds=new Map,this.parts=[],this.geometry=new Map,r.nodes.forEach((h,u)=>(h.children??[]).forEach(d=>{Gn(d,r.nodes.length,"child"),this.parents.has(d)&&Be("Multiple-parent original node"),this.parents.set(d,u)}));let o=(h,u=new Set)=>{if(this.worlds.has(h))return this.worlds.get(h);u.has(h)&&Be("Original node cycle"),u=new Set([...u,h]);let d=r.nodes[Gn(h,r.nodes.length,"node")],f=new De;if(d.matrix)(d.translation||d.rotation||d.scale||d.matrix.length!==16||d.matrix.some(_=>!Number.isFinite(_)))&&Be("Unsupported node transform"),f.fromArray(d.matrix);else{let _=d.translation??[0,0,0],m=d.rotation??[0,0,0,1],p=d.scale??[1,1,1];pr(_,"translation"),pr(p,"scale"),(m.length!==4||m.some(S=>!Number.isFinite(S))||Math.abs(m.reduce((S,E)=>S+E*E,0)-1)>1e-6||p.some(S=>S===0))&&Be("Invalid original TRS"),f.compose(new A(..._),new It(...m),new A(...p))}let g=this.parents.has(h)?o(this.parents.get(h),u).clone().multiply(f):f;return this.worlds.set(h,g),g};r.nodes.forEach((h,u)=>o(u));let a=new Set,c=h=>{Gn(h,r.nodes.length,"scene node"),a.has(h)&&Be("Repeated scene node"),a.add(h),(r.nodes[h].children??[]).forEach(c)};r.scenes[Gn(r.scene??0,r.scenes.length,"scene")].nodes.forEach(c);for(let h of a){let u=r.nodes[h];if(u.mesh===void 0)continue;let d=r.meshes[Gn(u.mesh,r.meshes.length,"mesh")];(d.primitives.length!==1||this.parts.some(f=>f.id===u.name))&&Be("Unsupported original surface grouping"),d.primitives.forEach((f,g)=>{((f.mode??4)!==4||f.extensions||f.targets||f.indices===void 0)&&Be("Indexed static triangles required");let _=this.accessor(f.attributes.POSITION,3,5126),m=this.accessor(f.indices,1);(m.length%3||m.some(S=>S>=_.length/3))&&Be("Invalid original index buffer");let p=new Rt;for(let S=0;S<_.length;S+=3)p.expandByPoint(new A(_[S],_[S+1],_[S+2]).applyMatrix4(o(h)));this.geometry.set(h+":"+g,{positions:_,indices:m,normal:f.attributes.NORMAL===void 0?null:this.accessor(f.attributes.NORMAL,3,5126),material:f.material}),this.parts.push({id:u.name,name:u.name,source_node:h,primitive_index:g,region:"native_reference",triangle_count:m.length/3,bounds_m:{min:p.min.toArray(),max:p.max.toArray()},bounds_mm:{min:p.min.toArray().map(S=>S*1e3),max:p.max.toArray().map(S=>S*1e3)},authority:"Pinned original surface; anatomical and metric qualification unresolved",source:"NIH/HRA original pelvis"})})}}accessor(e,t,n){let i=this.doc,r=i.accessors[Gn(e,i.accessors.length,"accessor")],o=i.bufferViews[Gn(r.bufferView,i.bufferViews.length,"buffer view")],a={5121:[1,"getUint8"],5123:[2,"getUint16"],5125:[4,"getUint32"],5126:[4,"getFloat32"]},c=a[r.componentType];(!c||r.sparse||r.normalized||r.extensions||o.extensions||o.buffer!==0||r.type!==(t===3?"VEC3":"SCALAR")||n&&r.componentType!==n||!n&&r.componentType===5126)&&Be("Unsupported original accessor"),Gn(r.count-1,1e6,"accessor count");let[l,h]=c,u=o.byteStride??l*t,d=(o.byteOffset??0)+(r.byteOffset??0),f=(r.byteOffset??0)+(r.count-1)*u+l*t;(!Number.isInteger(u)||u<l*t||u%l||f>o.byteLength||d<0||d%l||(o.byteOffset??0)+o.byteLength>i.buffers[0].byteLength||this.binStart+d+(r.count-1)*u+l*t>this.raw.length)&&Be("Original accessor outside embedded buffer");let g=[];for(let _=0;_<r.count;_++)for(let m=0;m<t;m++){let p=this.view[h](this.binStart+d+_*u+m*l,!0);Number.isFinite(p)||Be("Nonfinite original position"),g.push(p)}return g}triangle(e,t,n,i){Gn(e,this.doc.nodes.length,"source node"),Gn(t,this.doc.meshes[this.doc.nodes[e].mesh]?.primitives.length??0,"primitive");let r=this.geometry.get(e+":"+t);r||Be("Original node outside selected scene"),Gn(n,r.indices.length/3,"face");let o=r.indices.slice(n*3,n*3+3);i&&(!Array.isArray(i)||i.length!==3||i.some((c,l)=>!Number.isInteger(c)||c!==o[l]))&&Be("Original triangle vertex lineage mismatch");let a=o.map(c=>new A(...r.positions.slice(c*3,c*3+3)).applyMatrix4(this.worlds.get(e)));return new A().subVectors(a[1],a[0]).cross(new A().subVectors(a[2],a[0])).lengthSq()===0&&Be("Degenerate original triangle"),{vertices:o,points:a}}scene(){let e=new Et;for(let t of this.parts){let n=this.geometry.get(t.source_node+":"+t.primitive_index),i=new xt;i.setAttribute("position",new bt(n.positions,3)),i.setIndex(n.indices),n.normal?i.setAttribute("normal",new bt(n.normal,3)):i.computeVertexNormals();let r=this.doc.materials?.[n.material]?.pbrMetallicRoughness?.baseColorFactor??[.8,.8,.8,1],o=new kn({color:new Re().setRGB(...r.slice(0,3)),roughness:.8,metalness:0}),a=new lt(i,o);a.name=t.id,a.matrixAutoUpdate=!1,a.matrix.copy(this.worlds.get(t.source_node)),a.userData.part=t,e.add(a)}return e}};async function Zf(s,e){let t=e.original_review_contract;t||Be("Original review omitted from this build"),s=new Uint8Array(s),s.length!==t.source.bytes&&Be("Original missing, LFS pointer or size mismatch"),await Ey(s)!==t.source.sha256&&Be("Original checksum mismatch");let n=new Jh(s);for(let i of Object.values(t.features))n.parts.some(r=>r.source_node===i.source_node&&r.id===i.mesh_name)||Be("Original source node/label mismatch");return{contract:t,evidence:e,surface:n}}function Qh(s,e){let t=s.contract,n=s.evidence;return{kind:e,contract_sha256:n.original_review_contract_sha256,packet_id:n.landmark_packet.packet_id,packet_sha256:n.landmark_packet_sha256,reference_revision:n.revision,source_sha256:t.source.sha256,units:"m",coordinate_frame:t.coordinate_frame,accepted_landmarks:null,canonical_transform:null,fabrication_released:!1}}function eu(s){return{...Qh(s,s.contract.draft_kind),reviewer:{id:"unassigned",session_id:"session-1",evidence_kind:"review_submission"},annotations:[],views:[],review_notes:""}}function Wn(s,e){let t=s.contract,n=Qh(s,t.draft_kind);go(e,Object.keys(n).join(" ")+" reviewer annotations views review_notes","original draft"),Object.keys(n).some(o=>e[o]!==n[o])&&Be("Original draft custody, frame or qualification mismatch"),Kf(e)>t.max_draft_bytes&&Be("Original draft exceeds byte budget"),go(e.reviewer,"id session_id evidence_kind","reviewer");for(let o of["id","session_id"])(typeof e.reviewer[o]!="string"||!/^[A-Za-z0-9_.:-]{1,80}$/.test(e.reviewer[o]))&&Be("Invalid reviewer/session identifier");["review_submission","software_fixture"].includes(e.reviewer.evidence_kind)||Be("Invalid declared evidence kind"),Kh(e.review_notes,"review notes"),(!Array.isArray(e.views)||e.views.length>t.max_views)&&Be("Invalid view list");for(let o of e.views)go(o,"label camera_position_mm target_mm","view"),Kh(o.label,"view label",80),pr(o.camera_position_mm,"camera"),pr(o.target_mm,"target");(!Array.isArray(e.annotations)||e.annotations.length>Object.keys(t.features).length)&&Be("Invalid original annotation list");let i=new Set,r=[];for(let o of e.annotations){go(o,"landmark_id mesh_name source_node primitive_index face_index vertex_indices barycentric native_point_m selection_rule patch_faces patch_rule construction_note anatomically_accepted","annotation");let a=t.features[o.landmark_id];(!a||i.has(o.landmark_id))&&Be("Unknown or duplicate original landmark"),i.add(o.landmark_id),(o.mesh_name!==a.mesh_name||o.source_node!==a.source_node||o.anatomically_accepted!==!1)&&Be("Wrong original surface or anatomical promotion");let c=s.surface.triangle(o.source_node,o.primitive_index,o.face_index,o.vertex_indices);pr(o.barycentric,"barycentric"),pr(o.native_point_m,"native point");let l=o.barycentric;(Math.min(...l)<-1e-7||Math.max(...l)>1+1e-7||Math.abs(l.reduce((d,f)=>d+f,0)-1)>1e-7)&&Be("Barycentric point outside original triangle");let h=new A;c.points.forEach((d,f)=>h.addScaledVector(d,l[f])),h.distanceTo(new A(...o.native_point_m))>t.reconstruction_epsilon_m&&Be("Original point does not reconstruct");for(let d of["selection_rule","patch_rule","construction_note"])Kh(o[d],d,2e3);(!Array.isArray(o.patch_faces)||o.patch_faces.length>t.max_patch_faces)&&Be("Invalid patch face budget");let u=new Set;for(let d of o.patch_faces){go(d,"primitive_index face_index vertex_indices","patch face"),s.surface.triangle(o.source_node,d.primitive_index,d.face_index,d.vertex_indices);let f=d.primitive_index+":"+d.face_index;u.has(f)&&Be("Duplicate patch face"),u.add(f)}o.selection_rule.trim()||r.push(o.landmark_id+": point-selection rule"),a.patch_required&&(!o.patch_faces.length||!o.patch_rule.trim())&&r.push(o.landmark_id+": patch and representative-point rule"),a.construction_required&&!o.construction_note.trim()&&r.push(o.landmark_id+": median-plane construction")}return{status:"original_geometry_verified_only",annotations:e.annotations.length,missing_features:Object.keys(t.features).filter(o=>!i.has(o)).sort(),unresolved_requirements:r.sort(),anatomically_accepted:!1,fabrication_released:!1}}function gc(s,e,t){Wn(s,e),Wn(s,t);let n=Object.fromEntries(e.annotations.map(o=>[o.landmark_id,o])),i=Object.fromEntries(t.annotations.map(o=>[o.landmark_id,o])),r=o=>o.patch_faces.map(a=>a.primitive_index+":"+a.face_index).sort().join(",");return{kind:s.contract.comparison_kind,status:"unaccepted_coordinate_comparison",reviewer_relation:e.reviewer.id===t.reviewer.id?"same_declared_reviewer":"distinct_declared_ids_independence_unverified",contains_software_fixtures:[e,t].some(o=>o.reviewer.evidence_kind==="software_fixture"),distance_meaning:"Source-coordinate distances converted to mm; metric calibration unresolved; no tolerance verdict.",rows:Object.keys(s.contract.features).sort().map(o=>{let a=n[o],c=i[o];return{landmark_id:o,status:a&&c?"paired":!a&&!c?"missing_both":a?"missing_b":"missing_a",distance_source_mm:a&&c?Math.hypot(...a.native_point_m.map((l,h)=>l-c.native_point_m[h]))*1e3:null,selection_rules_differ:a&&c?["selection_rule","patch_rule","construction_note"].some(l=>a[l]!==c[l]):null,patch_faces_differ:a&&c?r(a)!==r(c):null}}),accepted_landmarks:null,canonical_transform:null,fabrication_released:!1}}function tu(s,e,t=null){return(e.length<1||e.length>2)&&Be("One or two reviews required"),t!==null&&!/^[0-9a-f]{64}$/.test(t)&&Be("Invalid viewer build fingerprint"),{...Qh(s,s.contract.packet_kind),reviews:Zh(e),validation:e.map(n=>Wn(s,n)),comparison:e.length===2?gc(s,...e):null,viewer_build_sha256:t,unresolved_gates:Zh(s.contract.unresolved_gates),attribution:s.evidence.landmark_packet.source.attribution,license:"CC-BY-4.0"}}function Jf(s,e){(!e||!Array.isArray(e.reviews)||Kf(e)>s.contract.max_packet_bytes)&&Be("Invalid review packet or byte budget");let t=tu(s,e.reviews,e.viewer_build_sha256),n=(i,r)=>typeof i=="number"&&typeof r=="number"?Number.isFinite(i)&&Math.abs(i-r)<=Math.max(1e-9,Math.abs(r)*1e-12):Array.isArray(i)&&Array.isArray(r)?i.length===r.length&&i.every((o,a)=>n(o,r[a])):i&&r&&typeof i=="object"&&typeof r=="object"?Object.keys(i).sort().join(" ")===Object.keys(r).sort().join(" ")&&Object.keys(i).every(o=>n(i[o],r[o])):i===r;return n(e,t)||Be("Packet reports, custody or qualification differ from reconstruction"),e.reviews}function $f(s,e,t,n){let i=e.object.userData.part,r=s.contract.features[t];(!r||i.source_node!==r.source_node||i.id!==r.mesh_name)&&Be("Choose the specified original cortical surface");let o=e.object.geometry,a=e.faceIndex,c=[0,1,2].map(d=>o.index.getX(a*3+d));s.surface.triangle(i.source_node,i.primitive_index,a,c);let l=c.map(d=>new A().fromBufferAttribute(o.attributes.position,d)),h=e.object.worldToLocal(e.point.clone()),u=new A;return pn.getBarycoord(h,...l,u)||Be("Degenerate original triangle"),{landmark_id:t,mesh_name:i.id,source_node:i.source_node,primitive_index:i.primitive_index,face_index:a,vertex_indices:c,barycentric:u.toArray(),native_point_m:e.point.clone().multiplyScalar(.001).toArray(),selection_rule:n?.selection_rule??"",patch_faces:Zh(n?.patch_faces??[]),patch_rule:n?.patch_rule??"",construction_note:n?.construction_note??"",anatomically_accepted:!1}}function Qf({data:s,download:e,activate:t,selectModel:n,changed:i,focus:r,getView:o,restoreView:a,snapshot:c,prepareCapture:l,renderAvailable:h}){let u=document.getElementById("original-review"),d=B=>document.getElementById("original-"+B),f=null,g=null,_=null,m=null,p=!1,S=0;u.innerHTML=`<h3>Original-surface review</h3>
    <p class="small-note">Choose the pinned <b>VH_F_Pelvis.glb</b> from your computer. It is checked locally. Reopen this file before resuming a saved review after closing Studio.</p>
    <label class="file-control">Load original pelvis<input id="original-file" type="file" accept=".glb"></label>
    <p id="original-status" class="small-note" role="status">Original not loaded. Preview drafts remain separate.</p>
    <div class="button-row"><button id="original-preview">Reduced preview</button><button id="original-mode" disabled>Verified original</button></div>
    <fieldset id="original-controls" disabled>
      <p class="warning">Proposed points only. Anatomical acceptance, metric calibration and an approved uncertainty budget remain open.</p>
      <label>Reviewer ID<input id="original-reviewer" maxlength="80" value="unassigned"></label>
      <label>Session ID<input id="original-session" maxlength="80" value="session-1"></label>
      <label>Declared evidence<select id="original-evidence"><option value="review_submission">Review submission</option><option value="software_fixture">Software fixture</option></select></label>
      <label>Candidate<select id="original-candidate"></select></label>
      <p id="original-definition" class="small-note"></p>
      <div class="button-row"><button id="original-focus">Focus surface</button><button id="original-point">Capture / replace point</button></div>
      <div class="button-row"><button id="original-patch">Toggle patch faces</button><button id="original-clear-patch">Clear patch</button><button id="original-delete">Delete candidate</button></div>
      <p id="original-selection" class="small-note"></p>
      <label>Point-selection rule<textarea id="original-rule" maxlength="2000" rows="2"></textarea></label>
      <label>Patch selection / representative-point rule<textarea id="original-patch-rule" maxlength="2000" rows="2"></textarea></label>
      <label>Construction / median-plane note<textarea id="original-construction" maxlength="2000" rows="2"></textarea></label>
      <label>Review notes<textarea id="original-notes" maxlength="8192" rows="2"></textarea></label>
      <p id="original-summary" class="small-note"></p>
      <div class="button-row"><button id="original-save">Save draft</button><button id="original-clear">Clear review</button></div>
      <label class="file-control">Resume draft or packet<input id="original-resume" type="file" accept=".json"></label>
      <label class="file-control">Load comparison draft B<input id="original-second" type="file" accept=".json"></label>
      <button id="original-remove-second">Remove comparison B</button>
      <div id="original-comparison" class="small-note"></div>
      <label>View label<input id="original-view-label" maxlength="80" value="Review view"></label>
      <div class="button-row"><button id="original-record-view">Record view</button><button id="original-image">Save annotated view</button></div>
      <label>Recorded context<select id="original-views"><option value="">No recorded views</option></select></label>
      <div class="button-row"><button id="original-restore-view">Restore view</button><button id="original-remove-view">Remove view</button></div>
      <button id="original-export" class="primary">Export review packet \u2197</button>
      <p class="small-note">A = orange, B = blue. Both stay unaccepted. Packet JSON carries original triangle lineage, source/build fingerprints and unresolved gates. No source binary is included.</p>
    </fieldset>`;for(let B of s.native_references.landmark_packet.landmarks)d("candidate").add(new Option(B.label,B.id));let E=(B,D=!1)=>{d("status").textContent=B,d("status").classList.toggle("warning",D)},y=()=>{if(!f)throw new Error("Load the matching original first")},w=()=>g?.annotations.find(B=>B.landmark_id===d("candidate").value),C=()=>({selection_rule:d("rule").value,patch_rule:d("patch-rule").value,construction_note:d("construction").value});function R(){y();let B=structuredClone(g);B.reviewer={id:d("reviewer").value,session_id:d("session").value,evidence_kind:d("evidence").value},B.review_notes=d("notes").value;let D=B.annotations.find(Q=>Q.landmark_id===d("candidate").value);return D&&Object.assign(D,C()),Wn(f,B),B}function N(){m=null,d("point").classList.remove("active"),d("patch").classList.remove("active")}function M(){d("controls").disabled=!f,d("mode").disabled=!f,d("mode").classList.toggle("active",p),d("preview").classList.toggle("active",!p);let B=s.native_references.landmark_packet.landmarks.find(ce=>ce.id===d("candidate").value);d("definition").textContent=B.definition+" "+B.review_rule;for(let ce of["focus","point","patch","record-view","image","restore-view"])d(ce).disabled=!p||!h;if(!g)return;d("reviewer").value=g.reviewer.id,d("session").value=g.reviewer.session_id,d("evidence").value=g.reviewer.evidence_kind,d("notes").value=g.review_notes;let D=w();d("rule").value=D?.selection_rule??"",d("patch-rule").value=D?.patch_rule??"",d("construction").value=D?.construction_note??"",d("selection").textContent=D?"Original node "+D.source_node+" / primitive "+D.primitive_index+" / face "+D.face_index+" \xB7 "+D.patch_faces.length+" patch faces \xB7 ["+D.native_point_m.map(ce=>ce.toFixed(7)).join(", ")+"] m":"No point for this candidate. Capture a point before adding its patch.";let Q=Wn(f,g);d("summary").textContent=Q.annotations+"/5 proposed points. Missing: "+(Q.missing_features.join(", ")||"none")+". Unresolved rules: "+(Q.unresolved_requirements.join("; ")||"none recorded")+". Anatomical adoption remains pending.";let ae=d("views").value;if(d("views").replaceChildren(new Option("Choose recorded view","")),g.views.forEach((ce,ve)=>d("views").add(new Option(ce.label,String(ve)))),d("views").value=ae,_){let ce=gc(f,g,_);d("comparison").innerHTML="<p>A: "+$(g.reviewer.id)+" / "+$(g.reviewer.session_id)+"; B: "+$(_.reviewer.id)+" / "+$(_.reviewer.session_id)+". "+$(ce.reviewer_relation)+". "+(ce.contains_software_fixtures?"Contains software fixtures. ":"")+$(ce.distance_meaning)+"</p>"+Dt(["FEATURE","SOURCE \u0394 / mm","RULES / PATCH"],ce.rows.map(ve=>[ve.landmark_id,ve.status==="paired"?ve.distance_source_mm.toFixed(6):ve.status,ve.status==="paired"?(ve.selection_rules_differ?"different rules":"same rules")+" / "+(ve.patch_faces_differ?"different faces":"same faces"):"\u2014"]))}else d("comparison").textContent="No comparison draft B loaded. Declared identities do not establish reviewer independence.";i()}function b(B){Wn(f,B),g=B,M()}let P=B=>async()=>{try{await B()}catch(D){E("Review blocked: "+D.message+". Current valid work is retained.",!0)}};async function H(B){if(y(),!B||B.size>f.contract.max_packet_bytes)throw new Error("Missing JSON or byte budget exceeded");return E("Checking supplied review\u2026"),$h(await B.text(),f.contract.max_packet_bytes)}async function G(B){if(!B||B.size!==s.native_references.original_review_contract.source.bytes)throw new Error("Original missing, LFS pointer or size mismatch");E("Checking local original\u2026");let D=++S,Q=await Zf(await B.arrayBuffer(),s.native_references);D===S&&(g&&Wn(Q,g),_&&Wn(Q,_),f=Q,g??=eu(f),N(),t(f),await n("NIH_PELVIS_ORIGINAL"),M(),E("Verified original \xB7 "+f.surface.parts.length+" source meshes \xB7 "+f.surface.parts.reduce((ae,ce)=>ae+ce.triangle_count,0)+" original triangles. No anatomical acceptance."))}function q(B){y();let D,Q=null;if(B?.kind===f.contract.packet_kind){let ae=Jf(f,B);[D,Q=null]=ae}else Wn(f,B),D=B;g=structuredClone(D),_=structuredClone(Q),N(),M(),E("Restored original review at unchanged native coordinates.")}function W(B){y(),Wn(f,B);let D=R();gc(f,D,B),g=D,_=structuredClone(B),M(),E("Comparison B loaded; reviewer independence remains unverified.")}let Y=()=>{let B=R(),D=tu(f,_?[B,_]:[B],s.build.package_input_sha256);return g=B,D};d("file").onchange=P(async()=>{let B=d("file").files[0];try{await G(B)}finally{d("file").value=""}}),d("preview").onclick=()=>n("NIH_PELVIS"),d("mode").onclick=()=>n("NIH_PELVIS_ORIGINAL"),d("candidate").onchange=()=>{N(),M()},d("focus").onclick=P(()=>{r(f.contract.features[d("candidate").value].mesh_name)});for(let[B,D]of[["point","point"],["patch","patch"]])d(B).onclick=P(()=>{if(b(R()),D==="patch"&&!w())throw new Error("Capture this candidate point before selecting patch faces");l();let Q=m===D?null:D;N(),m=Q,m&&d(B).classList.add("active"),E(m==="patch"?"Click original faces to add/remove them from this candidate patch.":m==="point"?"Click the specified original cortical surface.":"Capture paused.")});d("delete").onclick=P(()=>{let B=R();B.annotations=B.annotations.filter(D=>D.landmark_id!==d("candidate").value),N(),b(B)}),d("clear-patch").onclick=P(()=>{let B=R(),D=B.annotations.find(Q=>Q.landmark_id===d("candidate").value);D&&(D.patch_faces=[]),b(B)});for(let B of["reviewer","session","evidence"])d(B).onchange=P(()=>b(R()));for(let B of["rule","patch-rule","construction"])d(B).oninput=P(()=>{w()&&b(R())});return d("notes").oninput=P(()=>b(R())),d("save").onclick=P(()=>{b(R()),e(vs(g),"PHY-pelvis-original-draft.json","application/json")}),d("clear").onclick=P(()=>{N(),b(eu(f)),E("Review A cleared; source remains verified.")}),d("resume").onchange=P(async()=>{try{q(await H(d("resume").files[0]))}finally{d("resume").value=""}}),d("second").onchange=P(async()=>{try{W(await H(d("second").files[0]))}finally{d("second").value=""}}),d("remove-second").onclick=()=>{_=null,M()},d("export").onclick=P(()=>{e(vs(Y()),"PHY-pelvis-original-review-packet.json","application/json"),M()}),d("record-view").onclick=P(()=>{let B=R();B.views.push({label:d("view-label").value,...o()}),b(B),d("views").value=String(g.views.length-1)}),d("restore-view").onclick=P(()=>{if(b(R()),d("views").value==="")throw new Error("Choose a recorded view");a(g.views[Number(d("views").value)])}),d("remove-view").onclick=P(()=>{let B=R();d("views").value!==""&&(B.views.splice(Number(d("views").value),1),b(B))}),d("image").onclick=P(()=>{b(R()),c()}),M(),{currentView:o,syncModel(B){p=B.id==="NIH_PELVIS_ORIGINAL",N(),M()},stop:N,draft:()=>g?structuredClone(g):null,second:()=>_?structuredClone(_):null,packet:Y,loadReview:q,loadSecond:W,pick(B){if(!p||!m)return!1;try{let D=R(),Q=d("candidate").value,ae=D.annotations.find(ve=>ve.landmark_id===Q),ce=$f(f,B,Q,ae);if(m==="point")Object.assign(ce,C()),D.annotations=D.annotations.filter(ve=>ve.landmark_id!==Q),D.annotations.push(ce),N();else{let ve={primitive_index:ce.primitive_index,face_index:ce.face_index,vertex_indices:ce.vertex_indices},Ze=ae.patch_faces.findIndex(it=>it.primitive_index===ve.primitive_index&&it.face_index===ve.face_index);Ze<0?ae.patch_faces.push(ve):ae.patch_faces.splice(Ze,1)}b(D),E("Original lineage recorded. Proposed review only.")}catch(D){E(D.message,!0)}return!0},overlays(B){if(!(!p||!f))for(let[D,Q]of[[g,12081705],[_,1469629]])for(let ae of D?.annotations??[]){let ce=new lt(new Hi(1.6,12,8),new jt({color:Q,depthTest:!1}));ce.position.fromArray(ae.native_point_m).multiplyScalar(1e3),ce.renderOrder=2,B.add(ce);let ve=[];for(let Ze of ae.patch_faces){let it=f.surface.triangle(ae.source_node,Ze.primitive_index,Ze.face_index,Ze.vertex_indices).points.map(Ye=>Ye.multiplyScalar(1e3));for(let Ye=0;Ye<3;Ye++)ve.push(it[Ye],it[(Ye+1)%3])}if(ve.length){let Ze=new zi(new xt().setFromPoints(ve),new Tn({color:Q,depthTest:!1}));Ze.renderOrder=1,B.add(Ze)}}}}}var Fe=window.PHY_DATA,F=s=>document.getElementById(s),wi=F("viewport"),ip=F("stage"),Bt=(s,e=1)=>Number(s).toFixed(e),_c=s=>new A(s[0],s[2],-s[1]),sp=Fe.reference.statistics.stature.mean_mm,J={model:"F28_REFINED",height:sp,pose:75,palette:"wood",exploded:0,selected:null,isolated:!1,boneOnly:!1,ruler:!1,points:[],view:"iso",loading:!1,referenceCapture:!1},ue,Ot,Rn,Gt,cn,Ut,ln=[],Ai,_o,xn,Ms,bs,nu=0,rp="NIH_PELVIS",In=Fe.native_references?jh(Fe.native_references):null,Ss,ep,op;var yn=!0,ap={wood:{redwood:11696208,brass:12163934,copper:11890773,ivory:11189163,steel:7042683,aluminum:11712949,acetal:3686205},carbon:{redwood:4345168,brass:13022604,copper:10596016,ivory:14015183,steel:6845047,aluminum:12896966,acetal:3555133}};function Ft(s){F("toast").textContent=s,F("toast").classList.add("show"),clearTimeout(ep),ep=setTimeout(()=>F("toast").classList.remove("show"),3200)}function Cn(s,e,t="application/octet-stream"){let n=s instanceof Blob?s:new Blob([s],{type:t}),i=URL.createObjectURL(n),r=document.createElement("a");r.href=i,r.download=e,document.body.append(r),r.click(),r.remove(),setTimeout(()=>URL.revokeObjectURL(i),5e3)}function Ty(){try{Ot=new lc({canvas:wi,antialias:!0,alpha:!0,preserveDrawingBuffer:!0})}catch{F("loader").hidden=!0,F("fallback").style.display="block",F("fallback").innerHTML="<h2>3D rendering is unavailable here.</h2><p>Open PHY Studio in a browser with WebGL enabled. Proportions, readiness, and the maquette download still work.</p>",F("render-status").textContent="WebGL unavailable",su(),np(),gr("F28_REFINED",!1),F("export").disabled=!0;return}Ot.setPixelRatio(Math.min(devicePixelRatio||1,2)),Ot.setClearColor(15921643,0),Ot.outputColorSpace=_t,Ot.toneMapping=ba,Ot.toneMappingExposure=1.5,Ot.shadowMap.enabled=!0,Ot.shadowMap.type=fa,Rn=new Fi,Gt=new wt(31,1,1,25e3),Gt.up.set(0,1,0),cn=new dc(Gt,wi),cn.enableDamping=!0,cn.dampingFactor=.09,cn.maxPolarAngle=Math.PI*.94,cn.minDistance=150,cn.maxDistance=18e3,cn.addEventListener("change",()=>yn=!0),Rn.add(new Jr(16777215,14210243,2.4));let s=new vi(16777215,4);s.position.set(-1400,2100,-1900),s.castShadow=!0,s.shadow.mapSize.set(2048,2048),Object.assign(s.shadow.camera,{left:-1700,right:1700,top:2200,bottom:-1e3,near:1,far:7e3}),s.shadow.bias=-3e-4,Rn.add(s);let e=new vi(16765603,2.2);e.position.set(1500,650,1100),Rn.add(e);let t=new vi(13166045,3);t.position.set(1e3,2e3,1400),Rn.add(t),Ut=new Et,Rn.add(Ut),Ai=new Et,Rn.add(Ai),xn=new Et,Rn.add(xn),Ms=new Et,Rn.add(Ms),bs=new Et,Rn.add(bs),_o=new lt(new cs(1e4,1e4),new Yr({opacity:.12})),_o.rotation.x=-Math.PI/2,_o.position.y=-3,_o.receiveShadow=!0,bs.add(_o);let n=new io(6e3,60,13094079,14672343);n.position.y=-2,n.material.transparent=!0,n.material.opacity=.3,bs.add(n);let i=new os(new xt().setFromPoints(Array.from({length:120},(a,c)=>new A(Math.cos(c*Math.PI/60)*430,0,Math.sin(c*Math.PI/60)*430))),new Tn({color:13029312,transparent:!0,opacity:.7}));i.position.y=-1,bs.add(i),su(),Iy(),np(),gr("F28_REFINED"),new ResizeObserver(tp).observe(ip),tp();let r=null;wi.addEventListener("pointerdown",a=>r=[a.clientX,a.clientY]),wi.addEventListener("pointerup",a=>{r&&Math.hypot(a.clientX-r[0],a.clientY-r[1])<5&&Cy(a),r=null});let o=()=>{requestAnimationFrame(o),cn.update(),yn&&(Ot.render(Rn,Gt),yn=!1)};o(),F("loader").hidden=!0,F("render-status").textContent="3D ready \xB7 models stored locally"}function tp(){if(!Ot)return;let s=ip.getBoundingClientRect();Ot.setSize(s.width,s.height,!1),Gt.aspect=s.width/s.height,Gt.updateProjectionMatrix(),yn=!0}function su(){let s={F28_MEAN:["01","Female 28","Arithmetic mean"],F28_REFINED:["02","Female 28 \xB7 refined","Slight form adjustments"],SOPHY_SCALE:["03","SOPHY scale","1676.4 mm \xB7 canon reference"],A0_R2:["04","Shoulder \xB7 A0-R2","Located CAD / shop review"],REFERENCE_KIT:["05","Source library","Legacy component envelopes"],NIH_PELVIS:["06","NIH \xB7 female pelvis","Native frame / reduced preview"],NIH_SKIN:["07","NIH \xB7 female skin","Exterior / reduced preview"],NIH_PELVIS_ORIGINAL:["08","NIH \xB7 pelvis original","Verified local file / original faces"]};F("models").replaceChildren();for(let e of Fe.models){let[t,n,i]=s[e.id],r=document.createElement("button");r.className="model-button",r.dataset.model=e.id,r.innerHTML='<span class="model-glyph">'+t+'</span><span><span class="model-label">'+n+'</span><div class="model-note">'+i+"</div></span>",r.onclick=()=>gr(e.id),F("models").append(r)}}function Jt(s){for(;s.children.length;){let e=s.children.pop();e.traverse(t=>{if(t.geometry?.dispose(),t.material)for(let n of Array.isArray(t.material)?t.material:[t.material])n.map?.dispose(),n.dispose()}),e.parent=null}}function wy(s){let e=s.material==="redwood",t=s.region==="envelope";return new kn({color:ap[J.palette][s.material]??11580848,roughness:e?.58:.34,metalness:e?.04:.62,transparent:t,opacity:t?.105:1,depthWrite:!t,side:t?Kt:mn})}async function gr(s,e=!0){if(ue=Fe.models.find(a=>a.id===s),!ue)return;let t=ue,n=++nu,i=Vn(ue);J.loading=!1,J.referenceCapture=!1,F("capture-landmark").classList.remove("active"),Ss?.syncModel(ue),i&&(rp=s),J.model=s,J.height=ue.height_mm??0,J.pose=ue.arm_drop_deg??0,J.selected=null,J.isolated=!1,J.boneOnly=!1,J.exploded=0,F("layer-bones-only").checked=!1,F("layer-bones-only").disabled=!ue.bone_equivalence,J.points=[],J.ruler=!1,F("measure").classList.remove("active"),F("explode").value=0,F("part-search").value="",document.querySelectorAll("[data-model]").forEach(a=>{a.classList.toggle("active",a.dataset.model===s),a.setAttribute("aria-pressed",a.dataset.model===s)});let r=!!ue.height_mm,o=s==="A0_R2";F("model-title").textContent=r?"The female armature":o?"The shoulder article":"A library of parts",F("model-eyebrow").textContent=r?"FEMALE / 28 / REFERENCE ASSEMBLY":o?"A0-R2 / SINGLE-SIDE BENCH ARTICLE":"PHY / SOURCE COMPONENT LIBRARY",F("model-subtitle").textContent=r?"Measured proportions. Considered form.":o?"35 located instances. Exact current CAD geometry.":"Reference envelopes, arranged for inspection.",F("model-badge").textContent=s==="F28_MEAN"?"ARITHMETIC MEAN \xB7 AGE 28":s==="F28_REFINED"?"MEAN-BASED \xB7 GENTLY REFINED":s==="SOPHY_SCALE"?"CANON SCALE / PROPOSED FORM OVERLAY":o?"SHOP REVIEW \xB7 PHYSICAL QUALIFICATION OPEN":"UNASSEMBLED / DISPLAY LAYOUT ONLY",F("profile-note").textContent=r?s==="F28_MEAN"?"Measured reference dimensions; mechanical station mapping is a design proposal.":s==="SOPHY_SCALE"?"H = span = 1676.4 mm. This form overlay does not adopt new canon landmarks.":"92 women aged exactly 28. Shoulder \u22121%, waist \u22123%, hip +2%.":o?"A0-R2 stays in its original bench datum and dimensions.":"Gallery positions are display transforms, not anatomical placement.",F("source-note").textContent=r?"ANSUR II, female US Army cohort (2010\u20132012), age exactly 28; not a civilian/worldwide mean. 92 samples. External dimensions differ from mechanical centers. Arm station closure factor "+Bt(ue.design_datums.arm_chain_closure_scale,4)+". See Reference for every mean and limitation.":ue.limitations.join(" "),F("pose-note").textContent=r?"Visual arm motion around proposed centers.":o?"Neutral A0 assembly. Its physical motion screen remains discrete.":"Unassembled component gallery.",i&&(F("model-title").textContent=s==="NIH_SKIN"?"The female exterior":"The female pelvis",F("model-eyebrow").textContent="NIH / HRA / NATIVE REFERENCE",F("model-subtitle").textContent="Source geometry in a separate native frame.",F("model-badge").textContent=s==="NIH_PELVIS_ORIGINAL"?"VERIFIED ORIGINAL / ANATOMICAL QUALIFICATION OPEN":"DECIMATED PREVIEW / ANATOMICAL QUALIFICATION OPEN",F("profile-note").textContent="Source world coordinates preserved. 1000 mm/m display conversion only; no stature normalization.",F("pose-note").textContent="Native static source pose. No PHY joint axes or anatomical registration."),bs&&(bs.visible=!i),F("axis-note").textContent=i?"SOURCE X \xB7 Y UP \xB7 Z / m \u2192 mm \xD71000":"mm / X RIGHT \xB7 Y FRONT \xB7 Z UP",F("canvas-note").textContent=i?"SEPARATE NATIVE VIEW \xB7 CAMERA ORBIT ONLY \xB7 NO REGISTRATION":"DRAG TO ORBIT \xB7 SCROLL / PINCH TO ZOOM \xB7 CLICK A PART",F("reference-review").hidden=s!=="NIH_PELVIS",F("original-review").hidden=!Fe.native_references?.original_review_contract,F("compare-reference").hidden=!Fe.native_references||!i&&s!=="SOPHY_SCALE",F("compare-reference").textContent=i?"View SOPHY separately \u2197":"Return to NIH native view \u2197";for(let a of["explode","mat-wood","mat-carbon","layer-frame","layer-joints","layer-dimensions"])F(a).disabled=i;for(let a of F("export-format").options)a.disabled=i&&a.value==="stl";i&&F("export-format").value==="stl"&&(F("export-format").value="glb"),F("export-note").textContent=s==="NIH_PELVIS_ORIGINAL"?"GLB saves the verified original bytes. JSON exports the unaccepted original review packet.":i?"GLB downloads the identical pinned preview in meters. JSON retains draft/provenance. No transformed STL or fabrication mesh export.":"Current scale and pose. Reference geometry; full-scale structural release is open.",F("export").disabled=!Ot&&!i,xo();for(let a of["height","height-number","pose","pose-a","pose-t"])F(a).disabled=!r;if(F("layer-envelope").disabled=!r,F("height").value=J.height,F("height-number").value=r?Bt(J.height):"",F("pose").value=J.pose,F("part-count").textContent=ue.parts.length+" parts",vc(),Xn(null),cp(),lp(),!(!Ot||!e)){if(Jt(Ut),Jt(Ai),Jt(xn),Jt(Ms),ln=[],Ut.position.set(0,0,0),Ut.quaternion.identity(),Ut.scale.setScalar(1),i){J.loading=!0,F("render-status").textContent="Checking pinned native geometry\u2026";try{let a=t._originalSurface?{scene:t._originalSurface.scene()}:await new fc().parseAsync((await Yh(t)).buffer,"");if(n!==nu){Jt(a.scene);return}if(a.scene.traverse(c=>{if(!c.isMesh)return;let l=t.parts.find(h=>h.id===c.name);if(!l)throw new Error("Native mesh label mismatch");c.userData.part=l,c.castShadow=!0,c.receiveShadow=!0,ln.push(c)}),ln.length!==t.parts.length)throw new Error("Native mesh count mismatch");Ut.add(a.scene),Ut.scale.setScalar(1e3),Ut.userData={profile:s,units:"m",display_units:"mm",display_scale:1e3,canonical_transform:null},J.loading=!1,F("render-status").textContent=(t._originalSurface?"Verified local original":"Pinned native preview")+" \xB7 anatomy/metric fidelity unqualified",ri(),ji("iso"),xo();return}catch(a){n===nu&&(J.loading=!1,ln=[],Jt(Ut),F("render-status").textContent="Reference load blocked: "+a.message,Ft("Reference load blocked: "+a.message));return}}for(let a of ue.parts){let c=new xt,l=[];for(let u of a.vertices)l.push(u[0],u[2],-u[1]);c.setAttribute("position",new bt(l,3)),c.setIndex(a.faces.flat()),c.computeVertexNormals(),c.computeBoundingSphere();let h=new lt(c,wy(a));h.name=a.id,h.userData={part:a},h.castShadow=a.region!=="envelope",h.receiveShadow=!0,Ut.add(h),ln.push(h)}Ut.userData={profile:s,geometry_status:"reference / unreleased",units:"mm"},F("render-status").textContent="3D ready \xB7 models stored locally",ri(),ji("iso"),yn=!0}}function cp(){let s=!!ue.height_mm;if(Vn(ue)){[ue.parts.length+" <span>source meshes</span>",ue.derivative.preview_triangles.toLocaleString()+" <span>triangles</span>","m <span>native / Y up</span>"].forEach((i,r)=>{F("stat"+(r+1)).innerHTML=i,F("stat"+(r+1)+"-label").textContent=["Segmentation",ue._originalSurface?"Original triangles":"Reduced preview","Source frame"][r]});return}let e=s?[Bt(J.height)+" <span>mm</span>",Bt(ue.t_pose_span_mm*J.height/ue.height_mm)+" <span>mm</span>","92 <span>women / age 28</span>"]:ue.id==="A0_R2"?[ue.part_definition_count+" <span>CAD parts</span>",ue.parts.length+" <span>located instances</span>","317 <span>mm / S\u2013E station</span>"]:["18 <span>source solids</span>","mm <span>canonical units</span>","0 <span>anatomical transforms</span>"],t=s?["Standing height","T-pose span","Source cohort"]:ue.id==="A0_R2"?["Unique geometry","Assembly","Dummy member"]:["Library","Dimensions","Placement"];e.forEach((n,i)=>{F("stat"+(i+1)).innerHTML=n,F("stat"+(i+1)+"-label").textContent=t[i]})}function lp(){F("pose-label").textContent=ue.height_mm?J.pose===0?"T / 0\xB0":"A / "+J.pose+"\xB0":"Neutral",F("pose-a").classList.toggle("active",J.pose!==0),F("pose-t").classList.toggle("active",J.pose===0)}function ri(){if(!Ut)return;if(Vn(ue)){for(let t of ln){t.visible=!J.isolated||t.name===J.selected;for(let n of Array.isArray(t.material)?t.material:[t.material])n.emissive?.setHex(t.name===J.selected?5257750:0),n.emissiveIntensity=t.name===J.selected?.4:0}Jt(Ai),Ai.visible=!1,yn=!0;return}let s=ue.height_mm?J.height/ue.height_mm:1;Ut.scale.setScalar(s);let e=ue.height_mm?new A(0,ue.height_mm*.56,0):_c([-240,0,40]);for(let t of ln){let n=t.userData.part;t.position.set(0,0,0),t.quaternion.identity();let i=n.id.match(/_(R|L)(?:_|$)/);if(i&&["arms","hands"].includes(n.region)&&!n.id.startsWith("shoulder")&&ue.height_mm){let o=i[1],a=_c(ue.landmarks["shoulder_"+o]),c=(ue.arm_drop_deg-J.pose)*Math.PI/180*(o==="R"?1:-1);t.quaternion.setFromAxisAngle(new A(0,0,1),c),t.position.copy(a).sub(a.clone().applyQuaternion(t.quaternion))}if(J.exploded){let a=_c(n.bounds_mm.min).add(_c(n.bounds_mm.max)).multiplyScalar(.5).sub(e);a.length()>1&&t.position.add(a.normalize().multiplyScalar(J.exploded*(ue.height_mm?2.5:1.5)))}t.visible=Ff(n,J,{envelope:F("layer-envelope").checked,joints:F("layer-joints").checked,frame:F("layer-frame").checked}),t.material.wireframe=!1,t.material.emissive.setHex(n.id===J.selected?5257750:0),t.material.emissiveIntensity=n.id===J.selected?.4:0}F("explode-label").textContent=J.exploded+"%",Ay(),yn=!0}function xc(s,e,t=9018506,n=Ai){let i=new gi(new xt().setFromPoints([s,e]),new Tn({color:t,transparent:!0,opacity:.8}));return n.add(i),i}function ru(s,e,t=65,n=Ai){let i=document.createElement("canvas");i.width=400,i.height=88;let r=i.getContext("2d");r.fillStyle="#f2f1ebe8",r.fillRect(0,0,i.width,i.height),r.font="40px system-ui",r.textAlign="center",r.textBaseline="middle",r.fillStyle="#506552",r.fillText(s,200,44);let o=new Wr(i);o.colorSpace=_t;let a=new Br(new Js({map:o,depthTest:!1,transparent:!0,toneMapped:!1}));return a.position.copy(e),a.scale.set(t*4.5,t,1),n.add(a),a}function Ay(){if(Jt(Ai),Ai.visible=F("layer-dimensions").checked,!ue.height_mm)return;let s=J.height,e=-280*s/sp,t=45;xc(new A(e,0,t),new A(e,s,t));for(let n of[0,s])xc(new A(e-13,n,t),new A(e+13,n,t));if(ru(Bt(s)+" mm",new A(e-85,s*.51,t),s*.048),J.pose===0){let n=ue.t_pose_span_mm*s/ue.height_mm,i=ue.design_datums.shoulder_z_mm*s/ue.height_mm+90;xc(new A(-n/2,i,0),new A(n/2,i,0)),ru(Bt(n)+" mm span",new A(0,i+38,0),s*.048)}}function ji(s="iso",e=null,t=null){if(!Gt)return;J.view=s,Ut.updateMatrixWorld(!0);let n=new Rt;for(let c of ln)c.visible&&n.expandByObject(c);n.isEmpty()&&n.setFromObject(Ut);let i=n.getSize(new A),r=e??n.getCenter(new A),o=t??Math.max(i.y,i.x,i.z,180)*(innerWidth<760?2.5:2.18);cn.target.copy(r);let a={iso:new A(-.42,.14,-1),front:new A(0,.035,-1),side:new A(-1,.035,0),back:new A(0,.035,1)};Gt.position.copy(r).add(a[s].clone().normalize().multiplyScalar(o)),Gt.near=Math.max(.1,o/1e3),Gt.far=Math.max(25e3,o*10),Gt.updateProjectionMatrix(),cn.update(),document.querySelectorAll("[data-view]").forEach(c=>{c.classList.toggle("active",c.dataset.view===s),c.dataset.view!=="iso"&&(c.textContent=Vn(ue)?{front:"\u2212Z view",side:"\u2212X view",back:"+Z view"}[c.dataset.view]:{front:"Front",side:"Side",back:"Back"}[c.dataset.view])}),yn=!0}function vc(){let s=F("part-search").value.toLowerCase(),e=F("part-select");e.replaceChildren(new Option("Select in the viewport",""));for(let t of ue.parts)Bf(t,s,J.boneOnly)&&e.add(new Option(t.name+(t.bone_id?" \xB7 "+t.bone_id:""),t.id));e.value=J.selected??""}function Xn(s){J.selected=s;let e=ue.parts.find(n=>n.id===s);if(F("part-select").value=s??"",!e){F("part-details").innerHTML='<p class="quiet">Click the model to inspect a part, its dimensions, and its source.</p>',Ot&&ri();return}let t=e.bounds_mm.max.map((n,i)=>(n-e.bounds_mm.min[i])*(ue.height_mm?J.height/ue.height_mm:1));F("part-details").innerHTML="<p><strong>"+$(e.name)+"</strong><br>"+$(e.id)+"</p>"+(e.role?"<p>Role: "+$(e.role)+(e.bone_id?"<br>"+$(e.bone_id)+" \xB7 dimensional fidelity unverified":e.grouped_bone_ids?"<br>"+e.grouped_bone_ids.length+" grouped identities \xB7 not individual bones":"")+"</p>":"")+(e.source_record_status?'<p class="quiet">Canonical source record '+$(e.source_record_status)+"</p>":"")+"<p>"+t.map(n=>Bt(n)).join(" \xD7 ")+' mm <span class="quiet">/ axis-aligned envelope</span></p><p class="quiet">'+$(e.authority)+"<br>"+$(e.source)+"</p>"+(e.center_distance_mm?"<p>Member center distance: "+Bt(e.center_distance_mm*(ue.height_mm?J.height/ue.height_mm:1))+" mm</p>":""),e.bone_id&&(F("part-details").innerHTML+=bc(e.bone_id)),Vn(ue)&&(F("part-details").innerHTML+='<p class="quiet">Native X/Y/Z display envelope only; not a bone measurement.<br>Original source node '+e.source_node+(ue._originalSurface?" \xB7 original primitive "+e.primitive_index:" \xB7 preview node "+e.preview_node)+" \xB7 "+e.triangle_count+" triangles<br>Source "+$(ue.source.sha256)+(ue._originalSurface?"":"<br>Preview "+$(ue.derivative.preview_sha256))+"</p>"),ri()}function bc(s){let e=new Set(ue.parts.filter(t=>t.bone_id).map(t=>t.bone_id));return Hf(Fe.engineering,s)+qf(Fe.engineering.relationships,s,e)}function Ry(s){let e=ue.parts.find(n=>n.bone_id===s);if(!e)return Ft("No proxy for this identity in the current model."),!1;F("part-search").value="",vc(),Xn(e.id);let t=F("evidence-bone");return t&&(t.value=s,F("evidence-details").innerHTML=bc(s)),!0}function Cy(s){if(J.loading)return;let e=wi.getBoundingClientRect(),t=new Ee((s.clientX-e.left)/e.width*2-1,-(s.clientY-e.top)/e.height*2+1),n=new no;n.setFromCamera(t,Gt);let i=n.intersectObjects(ln.filter(r=>r.visible&&r.userData.part.region!=="envelope"),!1)[0];if(i&&!Ss?.pick(i)){if(J.referenceCapture&&ue.id==="NIH_PELVIS"){try{let r=Fe.native_references.landmark_packet.landmarks.find(a=>a.id===F("landmark-candidate").value),o=Yf(i,r,ue);In.annotations=In.annotations.filter(a=>a.landmark_id!==o.landmark_id),In.annotations.push(o),J.referenceCapture=!1,F("capture-landmark").classList.remove("active"),xo(),Ft("Saved an unaccepted preview candidate. Original-surface review remains required.")}catch(r){Ft(r.message)}return}if(J.ruler){J.points.length===2&&(J.points=[],Jt(xn)),J.points.push(i.point.clone());let r=new lt(new Hi(Math.max(J.height*.004,3),12,8),new jt({color:11954760,depthTest:!1}));if(r.position.copy(i.point),xn.add(r),J.points.length===2){let[o,a]=J.points;xc(o,a,11954760,xn);let c=o.distanceTo(a);ru(Bt(c)+" mm",o.clone().add(a).multiplyScalar(.5),Math.max(J.height*.028,12),xn),Ft((Vn(ue)?"Unqualified preview distance: ":"Surface-to-surface distance: ")+Bt(c)+" mm")}else Ft("Choose the second surface point.");yn=!0}else Xn(i.object.name)}}function xo(){if(In&&(F("draft-count").textContent=In.annotations.length+"/5 preview candidates",F("draft-points").textContent=In.annotations.map(s=>s.landmark_id+": ["+s.native_point_m.map(e=>Bt(e,6)).join(", ")+"] m \xB7 preview face "+s.face_index).join(`
`),!!Ms)){if(Jt(Ms),ue?.id==="NIH_PELVIS"&&!J.loading)for(let s of In.annotations){let e=new lt(new Hi(1.5,12,8),new jt({color:11954760,depthTest:!1}));e.position.fromArray(s.native_point_m).multiplyScalar(1e3),Ms.add(e)}J.loading||Ss?.overlays(Ms),yn=!0}}function Iy(){F("height").oninput=()=>ou(Number(F("height").value)),F("height-number").onchange=()=>ou(Number(F("height-number").value)),F("pose").oninput=()=>yc(Number(F("pose").value)),F("pose-a").onclick=()=>yc(75),F("pose-t").onclick=()=>yc(0);for(let s of["layer-frame","layer-joints","layer-envelope","layer-dimensions"])F(s).onchange=ri;F("layer-bones-only").onchange=()=>{J.boneOnly=F("layer-bones-only").checked,J.isolated=!1,J.boneOnly&&ue.parts.find(s=>s.id===J.selected)?.role!=="bone_proxy"&&(J.selected=null),J.points=[],Jt(xn),vc(),Xn(J.selected),ji(J.view)},F("explode").oninput=()=>{J.exploded=Number(F("explode").value),ri()},F("part-search").oninput=vc,F("part-select").onchange=()=>Xn(F("part-select").value||null),F("isolate").onclick=()=>{if(!J.selected){Ft("Select a component first.");return}J.isolated=!0,ri(),ji(J.view)},F("show-all").onclick=()=>{J.isolated=!1,Xn(null),ji(J.view)},F("focus-part").onclick=()=>{let s=ln.find(t=>t.name===J.selected);if(!s){Ft("Select a component first.");return}let e=new Rt().setFromObject(s);ji(J.view,e.getCenter(new A),Math.max(...e.getSize(new A).toArray(),45)*2.5)},F("mat-wood").onclick=()=>au("wood"),F("mat-carbon").onclick=()=>au("carbon"),document.querySelectorAll("[data-view]").forEach(s=>s.onclick=()=>ji(s.dataset.view)),F("measure").onclick=()=>{Ss?.stop(),J.referenceCapture=!1,F("capture-landmark").classList.remove("active"),J.ruler=!J.ruler,F("measure").classList.toggle("active",J.ruler),J.points=[],Jt(xn),J.ruler&&Ft(Vn(ue)?"Click two preview points. Distances are not qualified osteometry.":"Click two points on the frame to measure."),yn=!0},F("compare-reference").onclick=()=>gr(Vn(ue)?"SOPHY_SCALE":rp),F("capture-landmark").onclick=()=>{J.loading||!Ot||(J.ruler=!1,F("measure").classList.remove("active"),J.points=[],Jt(xn),J.referenceCapture=!J.referenceCapture,F("capture-landmark").classList.toggle("active",J.referenceCapture),J.referenceCapture&&Ft("Click the specified cortical surface for a draft point."))},F("landmark-candidate").onchange=()=>{let s=Fe.native_references.landmark_packet.landmarks.find(e=>e.id===F("landmark-candidate").value);F("landmark-definition").textContent=s.definition+" "+s.review_rule,Xn(s.preview_mesh_names[0])},F("download-draft").onclick=()=>Cn(JSON.stringify(In,null,2),"PHY-pelvis-preview-draft.json","application/json"),F("clear-draft").onclick=()=>{In=jh(Fe.native_references),xo()},F("export").onclick=Ly}function ou(s){if(ue.height_mm){if(!Number.isFinite(s)||s<1400||s>1900){Ft("Use a height from 1400 to 1900 mm."),F("height-number").value=Bt(J.height);return}J.height=s,F("height").value=s,F("height-number").value=Bt(s),J.points=[],Jt(xn),cp(),ri(),J.selected&&Xn(J.selected)}}function yc(s){ue.height_mm&&(J.pose=Math.max(0,Math.min(80,s)),F("pose").value=J.pose,lp(),J.points=[],Jt(xn),ri())}function au(s){if(!Vn(ue)){J.palette=s,F("mat-wood").classList.toggle("active",s==="wood"),F("mat-carbon").classList.toggle("active",s==="carbon");for(let e of ln)e.material.color.setHex(ap[s][e.userData.part.material]??11580848);yn=!0}}function Py(){return kf(Ut,ln,J.boneOnly)}async function Ly(){if(Vn(ue)){let n=ue,i=F("export-format").value;try{if(i==="stl"){Ft("Native preview STL export is unavailable. Use the pinned GLB or draft JSON.");return}if(n._originalSurface){i==="glb"?Cn(n._originalSurface.raw,"VH_F_Pelvis.glb","model/gltf-binary"):Cn(vs(Ss.packet()),"PHY-pelvis-original-review-packet.json","application/json");return}i==="glb"?Cn(await Yh(n),n.derivative.preview_filename,"model/gltf-binary"):Cn(JSON.stringify({reference_model:{...n,glb_base64:void 0},review_state:{units:"m",frame:n.frame,display_conversion_mm_per_m:1e3,canonical_transform:null,fabrication_released:!1,build:Fe.build},landmark_draft:n.id==="NIH_PELVIS"?In:null,native_reference_evidence:Fe.native_references,relationship_graph:Fe.engineering.relationships},null,2),"PHY-"+n.id+"-native-review.json","application/json"),Ft("Exported pinned native reference / unaccepted review data.")}catch(r){Ft("Reference export blocked: "+r.message)}return}if(!Ot){Ft("3D exports need WebGL. The maquette package remains available.");return}let s=F("export-format").value,e="PHY-"+J.model+"-"+Bt(J.height??0,0)+"mm",t={...J,points:[],units:"mm",fabrication_released:!1,source_snapshot_sha256:Fe.reference.snapshot_sha256,build:Fe.build,notes:ue.limitations};try{if(s==="json")Cn(JSON.stringify({model:ue,review_state:t,engineering_evidence:Fe.engineering},null,2),e+".json","application/json");else{let n=Py();if(n.userData=t,s==="stl")n.rotateX(Math.PI/2),n.updateMatrixWorld(!0),Cn(new mc().parse(n,{binary:!0}),e+".stl","model/stl");else{n.scale.setScalar(.001),n.userData.units="meters / glTF standard",n.updateMatrixWorld(!0);let i=await new xs().parseAsync(n,{binary:!0});Cn(i,e+".glb","model/gltf-binary")}}Ft("Exported current scale, pose and separation. Reference geometry.")}catch(n){console.error(n),Ft("Export failed: "+n.message)}}function Ny(s){let e=new TextEncoder,t=[],n=[],i=Array.from({length:256},(u,d)=>{let f=d;for(let g=0;g<8;g++)f=f&1?3988292384^f>>>1:f>>>1;return f>>>0}),r=0,o=u=>new Uint8Array(u),a=(u,d,f,g)=>{for(let _=0;_<g;_++)u[d+_]=f>>>_*8&255};for(let[u,d]of Object.entries(s)){let f=e.encode(u),g=e.encode(d),_=4294967295;for(let S of g)_=i[(_^S)&255]^_>>>8;_=(_^4294967295)>>>0;let m=o(30);a(m,0,67324752,4),a(m,4,20,2),a(m,6,2048,2),a(m,12,33,2),a(m,14,_,4),a(m,18,g.length,4),a(m,22,g.length,4),a(m,26,f.length,2),t.push(m,f,g);let p=o(46);a(p,0,33639248,4),a(p,4,20,2),a(p,6,20,2),a(p,8,2048,2),a(p,14,33,2),a(p,16,_,4),a(p,20,g.length,4),a(p,24,g.length,4),a(p,28,f.length,2),a(p,42,r,4),n.push(p,f),r+=m.length+f.length+g.length}let c=o(22),l=Object.keys(s).length,h=n.reduce((u,d)=>u+d.length,0);return a(c,0,101010256,4),a(c,8,l,2),a(c,10,l,2),a(c,12,h,4),a(c,16,r,4),new Blob([...t,...n,c],{type:"application/zip"})}function cu(){if(!Fe.maquette_files){Ft("Rebuild PHY Studio to embed the maquette cut files.");return}Cn(Ny(Fe.maquette_files),"PHY-F28-quarter-scale-maquette.zip"),Ft("Cut sheets, assembly stencil, BOM and instructions downloaded.")}function mr(s){F("modal-backdrop").classList.contains("open")||(op=document.activeElement),F("modal-backdrop").classList.add("open"),document.querySelector(".modal").focus(),document.querySelectorAll("[data-tab]").forEach(n=>n.classList.toggle("active",n.dataset.tab===s));let e=Fe.reference,t=e.statistics;if(s==="reference")F("modal-title").textContent="Proportion reference",F("modal-content").innerHTML="<p>Arithmetic means of <b>92 women aged exactly 28</b> from ANSUR II. "+$(e.population)+"</p><p>The baseline retains the measured mean stature and span. The refinement uses shoulder \u22121%, waist \u22123%, and hip +2%. Marginal means are a design reference, not one real individual or an objective beauty standard.</p><p>Joint centers, head height, curved formers and the interpolated form envelope are proposed mechanical datums. External segment ratios are mapped to the measured span; they are not osteometric bone lengths.</p>"+Dt(["MEASUREMENT","MEAN / mm","SAMPLE SD / mm","n"],Object.entries(t).map(([n,i])=>[n,Bt(i.mean_mm,3),Bt(i.sd_mm,3),i.n]))+'<p><a href="'+$(e.report_url)+'" target="_blank" rel="noopener">ANSUR II report</a> \xB7 <a href="'+$(e.data_url)+'" target="_blank" rel="noopener">Public data source</a></p><p class="small-note">Projected snapshot SHA-256: '+$(e.snapshot_sha256)+"</p>";else if(s==="nih")F("modal-title").textContent="NIH native references and landmark review",F("modal-content").innerHTML=jf(Fe);else if(s==="engineering"){F("modal-title").textContent="Engineering evidence",F("modal-content").innerHTML=Vf(Fe.engineering,Fe.build);let n=F("evidence-bone"),i=ue.parts.find(r=>r.id===J.selected)?.bone_id;i&&(n.value=i),n.onchange=()=>{let r=n.value;F("evidence-details").innerHTML=bc(r);let o=ue.parts.find(a=>a.bone_id===r);o&&Xn(o.id)},F("evidence-details").innerHTML=bc(n.value)}else if(s==="nasa"){F("modal-title").textContent="NASA positioning and ranges",F("modal-content").innerHTML=Wf(Fe.engineering);let n=F("nasa-condition");n.onchange=()=>{let i=n.value,r=Fe.engineering.nasa_counts.conditions[i];F("nasa-condition-count").textContent=qh[i]+" \xB7 "+r+" motion pairs",F("nasa-motion").innerHTML=Gf(Fe.engineering.nasa,i)},n.onchange()}else if(s==="bones"){let n=ue.bone_equivalence?ue:Fe.models.find(r=>r.bone_equivalence),i=n.bone_equivalence;F("modal-title").textContent="Bone equivalence / distribution",F("modal-content").innerHTML="<p>Reference-body scope only; A0 and the source gallery are separate. "+i.expected_bones+" adult identities: <b>"+i.individual_bone_proxies+" individual project proxies</b>, "+i.grouped_bones+" grouped-form identities, "+i.unrepresented_bones+' unrepresented. Hardware/supports do not count.</p><p class="warning">Dimensions and morphology are unverified project proposals, not osteometry or canon adoption. Coverage is not fabrication readiness.</p>'+Dt(["REGION","EXPECTED","INDIVIDUAL","GROUPED","UNREPRESENTED"],i.regions.map(r=>[r.region,r.expected,r.individual,r.grouped,r.unrepresented]))+"<p>"+i.source_records_present+" canonical source records; "+i.missing_source_records.length+" records absent. Source presence does not qualify dimensions, geometry or mechanics.</p><details><summary>Missing source records</summary><ul>"+i.missing_source_records.map(r=>"<li>"+$(r)+"</li>").join("")+"</ul></details><details><summary>All 206 identities / current disposition</summary>"+Dt(["BONE ID","DISPOSITION","MESH","SOURCE"],i.bones.map(r=>[r.bone_id,r.representation,r.mesh_ids.join(", ")||"\u2014",r.source_record||"MISSING"]))+"</details>"}else if(s==="bom")F("modal-title").textContent="Quarter-scale maquette",F("modal-content").innerHTML="<p>Supported, passive form study \xB7 "+Bt(Fe.maquette.height_mm)+" mm tall \xB7 "+Fe.maquette.cut_parts+" cut parts \xB7 "+Fe.maquette.cut_sheets+" A3 sheets. Nominal 3 mm birch plywood. Verify stock, hole coupons and cutter kerf before cutting.</p>"+Dt(["ID","QTY","DESCRIPTION","STOCK / SIZE","ASSEMBLY NOTE"],Fe.maquette.bill_of_materials.map(n=>[n.id,n.qty,n.description,n.stock+" / "+Bt(n.width_mm)+" \xD7 "+Bt(n.length_mm)+" mm",n.note]))+'<button id="modal-download" class="primary">Download the cut and assembly package \u2197</button>',F("modal-download").onclick=cu;else if(s==="build")F("modal-title").textContent="From reference to physical form",F("modal-content").innerHTML='<div class="build-steps"><div class="build-step"><span class="num">01 / SEE</span><h3>Complete reference assembly</h3><span class="tag">DEMONSTRABLE</span><p>Whole-body bilateral armature, source means, explicit mechanical datums, materials, poses and transferable meshes.</p></div><div class="build-step"><span class="num">02 / MAKE</span><h3>Supported 1:4 maquette</h3><span class="tag">CUT PACKAGE AVAILABLE</span><p>Actual-size SVG cut paths, plywood parts, copper pins, rear support post, assembly stencil and bill of materials.</p></div><div class="build-step"><span class="num">03 / PROVE</span><h3>A0-R2 shoulder article</h3><span class="tag">SHOP REVIEW</span><p>Revised cartridge geometry and manufacturing drawings. Stop compliance, purchased fits, remaining article interfaces and physical measurements remain open.</p></div><div class="build-step"><span class="num">04 / INTEGRATE</span><h3>Full-scale functional armature</h3><span class="tag">ENGINEERING OPEN</span><p>Resolve whole-body interfaces, joint mechanisms, retained motion, loads, support, balance and fabrication inspection. A form study does not close these gates.</p></div></div><p>Start with the quarter-scale physical form. Use its measurements to review the proportions and record actual assembly observations.</p><button id="modal-download" class="primary">Get the maquette package \u2197</button>',F("modal-download").onclick=cu;else{F("modal-title").textContent="Fabrication readiness";let n=Fe.models.find(i=>i.id==="A0_R2");F("modal-content").innerHTML="<p>The Studio and whole-body form-study model are demonstrable. The maquette has reproducible cut geometry and assembly instructions. The full-scale armature has <b>no structural fabrication release or physical qualification</b>.</p>"+Dt(["DELIVERABLE","CURRENT STATE"],[["Female-28 reference assembly","Complete visual reference / explicit design proposals"],["Quarter-scale supported maquette","Cut/assembly package; physical build unmeasured"],["SOPHY canon 1.0.0","Kernel unchanged; scale comparison overlay only"],["A0-R2 shoulder","Shop-review candidate; release and bench evidence open"],["Whole-body functional armature","Mechanism integration and load qualification open"]])+(n?'<h3>A0-R2 release blockers</h3><p class="warning">The revised cartridge remains blocked for release: the assumed 4 mm stop compression is unverified and conflicts with the hard-stop angle limits. Bearing fits and remaining article interfaces also need review.</p><ul>'+n.limitations.slice(2).map(i=>"<li>"+$(i)+"</li>").join("")+"</ul><p>Six load cases and five evidence packages remain unapproved. The 35-pose centerline screen is not a continuous swept-solid clearance proof.</p>":"<p>A0 geometry was omitted in this standard-library build.</p>")+"<h3>Current model assumptions</h3><ul>"+ue.limitations.map(i=>"<li>"+$(i)+"</li>").join("")+"</ul>",F("modal-content").innerHTML+="<p>Canonical evidence: "+Fe.engineering.dimension_expansion.current_complete+"/206 complete declared maps; "+Fe.engineering.metrics.record_validation_pass.count+"/206 records pass the current validator. See Engineering for sources and unresolved fields.</p>"}}function iu(){F("modal-backdrop").classList.remove("open"),op?.focus()}function np(){if(Fe.native_references?.original_review_contract&&(Ss=Qf({data:Fe,download:Cn,selectModel:gr,changed:xo,renderAvailable:!!Ot,prepareCapture(){J.ruler=!1,J.points=[],F("measure").classList.remove("active"),Jt(xn)},activate(s){let e=Fe.models.find(i=>i.id==="NIH_PELVIS"),t={...e,id:"NIH_PELVIS_ORIGINAL",kind:"nih_native_original",parts:s.surface.parts,derivative:{...e.derivative,preview_triangles:s.surface.parts.reduce((i,r)=>i+r.triangle_count,0)},glb_base64:void 0};Object.defineProperty(t,"_originalSurface",{value:s.surface});let n=Fe.models.findIndex(i=>i.id===t.id);n<0?Fe.models.push(t):Fe.models[n]=t,su()},focus(s){Xn(s),J.isolated=!0,ri(),ji(J.view)},getView:()=>({camera_position_mm:Gt.position.toArray(),target_mm:cn.target.toArray()}),restoreView(s){Gt.position.fromArray(s.camera_position_mm),cn.target.fromArray(s.target_mm),cn.update(),yn=!0},snapshot(){Ot.render(Rn,Gt),wi.toBlob(s=>Cn(s,"PHY-original-annotated-view.png","image/png")),Cn(vs({viewer_build_sha256:Fe.build.package_input_sha256,user_agent:navigator.userAgent,viewport_px:[wi.width,wi.height],view:{camera_position_mm:Gt.position.toArray(),target_mm:cn.target.toArray()},note:"Review context only; image bytes are not geometric proof and vary across renderers."}),"PHY-original-annotated-view-context.json","application/json")}})),Fe.native_references){for(let e of Fe.native_references.landmark_packet.landmarks)F("landmark-candidate").add(new Option(e.label,e.id));let s=Fe.native_references.landmark_packet.landmarks[0];F("landmark-definition").textContent=s.definition+" "+s.review_rule}F("download-maquette").onclick=cu,F("open-reference").onclick=()=>mr("reference"),F("open-build").onclick=()=>mr("build"),F("open-readiness").onclick=()=>mr("readiness"),F("open-bones").onclick=()=>mr("bones"),F("close-modal").onclick=iu,F("open-engineering").onclick=()=>mr("engineering");for(let s of[F("part-details"),F("modal-content")])s.addEventListener("click",e=>{let t=e.target.closest("button[data-related-bone]");t&&!t.disabled&&Ry(t.dataset.relatedBone)});F("build-stamp").textContent="Engineering baseline "+Fe.build.engineering_revision.slice(0,8)+" \xB7 package "+Fe.build.package_input_sha256.slice(0,12),document.querySelectorAll("[data-tab]").forEach(s=>s.onclick=()=>mr(s.dataset.tab)),F("modal-backdrop").onclick=s=>{s.target===F("modal-backdrop")&&iu()},document.addEventListener("keydown",s=>{if(F("modal-backdrop").classList.contains("open")&&(s.key==="Escape"&&iu(),s.key==="Tab")){let e=[...document.querySelector(".modal").querySelectorAll('button,a,input,select,[tabindex="0"]')].filter(i=>!i.disabled),t=e[0],n=e.at(-1);s.shiftKey&&document.activeElement===t?(s.preventDefault(),n.focus()):!s.shiftKey&&document.activeElement===n&&(s.preventDefault(),t.focus())}})}window.PHY_STUDIO={getState:()=>({...J,points:J.points.map(s=>s.toArray()),modelParts:ue?.parts.length,webgl:!!Ot}),selectModel:gr,setHeight:ou,setPose:yc,setPalette:au,inspectPart:Xn,referenceDraft:()=>In?structuredClone(In):null,originalReview:()=>Ss,boneAudit:()=>ue?.bone_equivalence??null,visiblePartIds:()=>ln.filter(s=>s.visible).map(s=>s.name),modelBounds:()=>Ut?new Rt().setFromObject(Ut).getSize(new A).toArray():null,projectPart:s=>{let e=ln.find(i=>i.name===s);if(!e)return null;let t=new Rt().setFromObject(e).getCenter(new A).project(Gt),n=wi.getBoundingClientRect();return[n.left+(t.x+1)*n.width/2,n.top+(1-t.y)*n.height/2]}};Ty();})();

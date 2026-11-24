/* PRISM 3D twin. Scene, CAD parts, electronics and SO-101 kinematics: ported from the CAD artifact (134 parts, exported
   from Fusion). Motion: driven by the live cell data, see the LIVE ENGINE section at the end. Needs web/cad_data.js
   (window.CAD) and three.js r128 (web/vendor). */
window.Cell3D = async function (opts) {
const $ = id => document.getElementById(id);
const META = window.CAD.META, B64 = window.CAD.B64;
if (typeof DecompressionStream === 'undefined') throw new Error('this browser has no DecompressionStream: use a current Chrome / Edge / Firefox / Safari');
const raw=Uint8Array.from(atob(B64),c=>c.charCodeAt(0));
const buf=new Uint8Array(await new Response(new Blob([raw]).stream().pipeThrough(new DecompressionStream('deflate-raw'))).arrayBuffer());
const geos=META.meshes.map(m=>{
  const q=new Int16Array(buf.buffer,m.vo,m.nv*3),p=new Float32Array(m.nv*3);
  for(let i=0;i<m.nv;i++)for(let a=0;a<3;a++)p[i*3+a]=q[i*3+a]/32767*m.h[a]+m.c[a];
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(p,3));
  g.setIndex(new THREE.BufferAttribute(m.big?new Uint32Array(buf.buffer,m.io,m.nf*3):new Uint16Array(buf.buffer,m.io,m.nf*3),1));
  g.computeBoundingSphere();return g;});
// ---------- scene ----------
const cv=opts.canvas,R=new THREE.WebGLRenderer({canvas:cv,antialias:true});R.setPixelRatio(Math.min(devicePixelRatio,2));
const scene=new THREE.Scene();scene.background=new THREE.Color(0x0d1116);scene.fog=new THREE.Fog(0x0d1116,1800,4200);
const cam=new THREE.PerspectiveCamera(40,1,5,6000);cam.up.set(0,0,1);
const ctl=new THREE.OrbitControls(cam,cv);ctl.enableDamping=true;ctl.dampingFactor=.08;ctl.maxPolarAngle=Math.PI*0.52;
scene.add(new THREE.AmbientLight(0xffffff,.5));scene.add(new THREE.HemisphereLight(0xcfe0ff,0x1a1f26,.75));
const sun=new THREE.DirectionalLight(0xffffff,.9);sun.position.set(-300,-500,900);scene.add(sun);
const sun2=new THREE.DirectionalLight(0x9db8ff,.35);sun2.position.set(500,300,200);scene.add(sun2);
const grid=new THREE.GridHelper(2400,48,0x2a3440,0x1a222b);grid.rotation.x=Math.PI/2;grid.position.set(100,-60,-100.6);scene.add(grid);
const M4=a=>new THREE.Matrix4().set(...a);
// ---------- materials by part name ----------
function style(n){
  // Colors matched to the real build photos: green SO-101 arm + pink pads, white printed parts, navy plates/guides, dark-green belt.
  const WHITE=0xeceae6;
  if(/^A13[45]/.test(n))return{c:0xff2fa0,m:0,r:.5};                         // pink gripper pads (M18)
  if(/^A12[4-9]|^A13[01]|^A133/.test(n))return{c:0x17181b,m:.3,r:.5};         // black servos + bus adapter
  if(/^A132/.test(n))return{c:0x1c2a52,m:.1,r:.6};                            // navy mounting plate
  if(/^A1/.test(n))return{c:0x3cb24a,m:0,r:.55};                              // green printed arm links
  if(/ENV_2020/.test(n))return{c:0xa7b0ba,m:.55,r:.45};                       // aluminium extrusion
  if(/ENV_bearing|bracket_JGB|motor_JGB|M24_shaft|M25_shaft|M22b|M23/.test(n))return{c:0x9aa3ad,m:.6,r:.4}; // metal hardware
  if(/belt_strip/.test(n))return{c:0x0e4a2a,m:0,r:.9};                        // dark green belt
  if(/M05_magazine/.test(n))return{c:WHITE,m:0,r:.4,o:.3};                    // white magazine (kept see-through so blocks show)
  if(/M17/.test(n))return{c:0x1b2650,m:0,r:.6};                               // navy side guides
  if(/M16a/.test(n))return{c:0xff2fa0,m:0,r:.5};                              // pink accent strips
  if(/M16b/.test(n))return{c:0x1b2650,m:0,r:.6};                              // guide caps, navy like the guides
  if(/M14|M31|M32/.test(n))return{c:/M14/.test(n)?0x1b2547:0x16181c,m:0,r:.7};// navy arm plate, black deck tiles
  if(/reserve_estop_relay_fuse/.test(n))return{c:0xcfd3d8,m:0,r:.3};          // clear fuse box
  if(/rpi/.test(n))return{c:0xe9e9e6,m:0,r:.5};                               // white Pi case
  if(/mega/.test(n))return{c:0x2150a0,m:.1,r:.6};                             // blue Arduino Mega
  if(/usb_hub|esp32|urt1|bts7960|ina226/.test(n))return{c:0x1c5a3a,m:.1,r:.6};// PCBs
  if(/psu/.test(n))return{c:0xb9bfc7,m:.5,r:.45};                             // silver PSU
  if(/MG90S/.test(n))return{c:0x2b5cb8,m:.1,r:.5};                            // blue MG90S servo
  return{c:WHITE,m:0,r:.55};}                                                 // everything else is white PLA
// ---------- instances ----------
const WM=M4(META.W),CH=META.chain.map(j=>({T:M4(j.T),ax:new THREE.Vector3(...j.ax)})),GT=M4(META.gT);
const parts={},armParts=[],extra={leds:[],rollers:[],shafts:[]};
const root=new THREE.Group();scene.add(root);
META.inst.forEach(it=>{
  const s=style(it.name);
  if(/ENV_board_rest|ENV_belt_strip:4|ENV_(rpi4|usb_hub|mega2560|reserve_estop_relay_fuse|psu_12v5a|bts7960|urt1|esp32|ina226)/.test(it.name))return;   // electronics are drawn as detailed models below
  const mat=new THREE.MeshStandardMaterial({color:s.c,metalness:s.m,roughness:s.r,flatShading:true,transparent:!!s.o,opacity:s.o||1,depthWrite:!s.o,side:s.o?THREE.DoubleSide:THREE.FrontSide});
  const me=new THREE.Mesh(geos[it.m],mat);me.matrixAutoUpdate=false;me.matrix.copy(M4(it.M));me.frustumCulled=false;
  if(s.o)me.renderOrder=2;
  root.add(me);
  me.userData={n:it.name,M0:M4(it.M)};
  if(it.arm){me.userData.link=it.arm;armParts.push(me);}
  if(/M06_blade/.test(it.name))parts.blade=me;
  if(/M11/.test(it.name))extra.leds.push(me);
  if(/M01_roller/.test(it.name))extra.rollers.push(me);
  if(/M24_shaft|M25_shaft/.test(it.name))extra.shafts.push(me);
});
// belt top: textured plane that scrolls
const tc=document.createElement('canvas');tc.width=64;tc.height=64;{const g=tc.getContext('2d');g.fillStyle='#0f4a2c';g.fillRect(0,0,64,64);g.fillStyle='#186238';g.fillRect(0,0,6,64);g.fillStyle='#0c3d25';g.fillRect(6,0,3,64);}
const bt=new THREE.CanvasTexture(tc);bt.wrapS=bt.wrapT=THREE.RepeatWrapping;bt.repeat.set(728/40,1);bt.anisotropy=4;
const beltTop=new THREE.Mesh(new THREE.PlaneGeometry(728,80),new THREE.MeshStandardMaterial({map:bt,roughness:.9}));beltTop.position.set(0,0,.2);scene.add(beltTop);
// camera field of view (shown while inspecting)
const LENS=new THREE.Vector3(-232,0,198),FW=138,FH=78;
const fv=new Float32Array([LENS.x,LENS.y,LENS.z,-232-FW,-FH,0,-232+FW,-FH,0,-232+FW,FH,0,-232-FW,FH,0]);
const fg=new THREE.BufferGeometry();fg.setAttribute('position',new THREE.BufferAttribute(fv,3));fg.setIndex([0,1,2,0,2,3,0,3,4,0,4,1]);
const fov=new THREE.Mesh(fg,new THREE.MeshBasicMaterial({color:0xfff2c0,transparent:true,opacity:0,depthWrite:false,side:THREE.DoubleSide}));fov.renderOrder=3;scene.add(fov);
// ---------- Intel RealSense D415 (99 x 20 x 23 mm), mounted under the camera-head plate, lens facing down at the belt ----------
const d415=new THREE.Group();
{
  const W=99,H=20,D=23;                                    // X length, Y height, Z depth (mm)
  // anodised metal body
  const body=new THREE.Mesh(new THREE.BoxGeometry(W,H,D),new THREE.MeshStandardMaterial({color:0x4a4f56,metalness:.7,roughness:.4}));
  body.position.set(0,0,D/2);d415.add(body);               // body spans z=0 (front face) .. D (back)
  // black front glass panel
  const glass=new THREE.Mesh(new THREE.BoxGeometry(W-2,H-2,.6),new THREE.MeshStandardMaterial({color:0x07090c,metalness:.2,roughness:.15}));
  glass.position.set(0,0,-.3);d415.add(glass);
  // lenses along X: left imager, IR projector, right imager, RGB (55 mm stereo baseline between the two imagers)
  const lensMat=new THREE.MeshStandardMaterial({color:0x14181f,metalness:.3,roughness:.1});
  const ringMat=new THREE.MeshStandardMaterial({color:0x8d949c,metalness:.8,roughness:.35});
  [[-38,3.6],[17,3.6],[38,3.6]].forEach(([x,r])=>{
    const ring=new THREE.Mesh(new THREE.CylinderGeometry(r+1.2,r+1.2,.5,24),ringMat);ring.rotation.x=Math.PI/2;ring.position.set(x,0,-.8);d415.add(ring);
    const lens=new THREE.Mesh(new THREE.CylinderGeometry(r,r,.6,24),lensMat);lens.rotation.x=Math.PI/2;lens.position.set(x,0,-1.1);d415.add(lens);
  });
  const proj=new THREE.Mesh(new THREE.BoxGeometry(11,6,.6),new THREE.MeshStandardMaterial({color:0x3a0d10,metalness:.2,roughness:.2}));
  proj.position.set(-8,0,-.9);d415.add(proj);               // dark-red IR projector window
  // USB-C connector at the +X end
  const usb=new THREE.Mesh(new THREE.BoxGeometry(8,9,3.2),new THREE.MeshStandardMaterial({color:0xb9bfc7,metalness:.85,roughness:.35}));
  usb.position.set(W/2+4,0,D/2);usb.rotation.y=Math.PI/2;d415.add(usb);
}
d415.position.set(-232,0,198);                             // front face at z=198 (glass faces down, -Z, toward the belt), body up to z=221 just under the head's top plate
scene.add(d415);
// ---------- detailed electronics on the deck (replace the plain placeholder boxes; same footprints and positions as the CAD) ----------
const DECK=-89;                                              // top surface of the deck tiles (z, mm)
const eM=(c,m,r,o)=>new THREE.MeshStandardMaterial(Object.assign({color:c,metalness:m||0,roughness:r==null?.6:r},o||{}));
// box: w=X, d=Y, h=Z, (x,y)=centre, z=bottom face
const eBox=(g,w,d,h,x,y,z,mat)=>{const m=new THREE.Mesh(new THREE.BoxGeometry(w,d,h),mat);m.position.set(x,y,z+h/2);g.add(m);return m;};
// vertical cylinder, z=bottom face
const eCyl=(g,r,h,x,y,z,mat,seg)=>{const m=new THREE.Mesh(new THREE.CylinderGeometry(r,r,h,seg||20),mat);m.rotation.x=Math.PI/2;m.position.set(x,y,z+h/2);g.add(m);return m;};
// cylinder lying along X, centred on (x,y,z)
const eCylX=(g,r,h,x,y,z,mat,seg)=>{const m=new THREE.Mesh(new THREE.CylinderGeometry(r,r,h,seg||20),mat);m.rotation.z=Math.PI/2;m.position.set(x,y,z);g.add(m);return m;};
// canvas texture (used only on top faces, where canvas-up = world +Y and canvas-right = world +X)
const eTex=(w,h,fn)=>{const c=document.createElement('canvas');c.width=w;c.height=h;fn(c.getContext('2d'),w,h);const t=new THREE.CanvasTexture(c);t.anisotropy=4;return t;};
// box whose top face carries a texture
const eTop=(g,w,d,h,x,y,z,sideMat,topMat)=>{const m=new THREE.Mesh(new THREE.BoxGeometry(w,d,h),[sideMat,sideMat,sideMat,sideMat,topMat,sideMat]);m.position.set(x,y,z+h/2);g.add(m);return m;};
const eList=[];                                              // every electronics group, so the part panel can hide them
const ePut=(g,x,y)=>{g.position.set(x,y,DECK);scene.add(g);eList.push(g);};
const BLK=eM(0x16171a,.2,.6),SIL=eM(0xc3c9d0,.85,.35),GOLD=eM(0xc9a227,.8,.3),WHT=eM(0xf1efea,0,.5),BLU=eM(0x2d6cdf,0,.5);
const eLed=c=>eM(c,0,.4,{emissive:c,emissiveIntensity:.5});
// 2.54 mm pin-header strip of n pins (alongX: strip runs along X, otherwise along Y)
const eHdr=(g,n,x,y,z,alongX)=>eBox(g,alongX?n*2.54:2.54,alongX?2.54:n*2.54,8.5,x,y,z,BLK);

// --- Raspberry Pi 4 in a white vented case with a fan (56 x 85 x 23) ---
{const g=new THREE.Group(),W=56,D=85,H=23,s=4;
 const top=eTex(W*s,D*s,(c,w,h)=>{
  c.fillStyle='#f1efea';c.fillRect(0,0,w,h);
  const fx=w/2,fy=h*.42,fr=25*s;
  c.fillStyle='#2a2d33';                                      // honeycomb vent holes around the fan
  for(let r=0;r<13;r++)for(let q=0;q<8;q++){const x=(4+q*7+(r%2?3.5:0))*s,y=(4+r*6.2)*s;if(Math.hypot(x-fx,y-fy)>fr+4*s&&x<w-3*s){c.beginPath();c.arc(x,y,1.5*s,0,7);c.fill();}}
  c.fillStyle='#16171a';c.beginPath();c.arc(fx,fy,fr,0,7);c.fill();            // fan frame
  c.fillStyle='#3a3e45';for(let k=0;k<7;k++){c.save();c.translate(fx,fy);c.rotate(k*Math.PI*2/7);c.beginPath();c.ellipse(11*s,0,11*s,4.5*s,.5,0,7);c.fill();c.restore();}   // blades
  c.fillStyle='#26282d';c.beginPath();c.arc(fx,fy,6*s,0,7);c.fill();            // hub
  c.strokeStyle='#4a4e56';c.lineWidth=.7*s;for(const rr of[12,18,24]){c.beginPath();c.arc(fx,fy,rr*s,0,7);c.stroke();}   // grille rings
 });
 eTop(g,W,D,H,0,0,0,eM(0xf1efea,0,.5),new THREE.MeshStandardMaterial({map:top,roughness:.5}));
 [-19.5,-2.5].forEach(x=>{eBox(g,14,1,15,x,D/2+.5,3,SIL);eBox(g,11,1.2,4.5,x,D/2+.6,5,BLU);eBox(g,11,1.2,4.5,x,D/2+.6,10.5,BLK);});   // 2 x USB stacks (USB3 blue / USB2 black)
 eBox(g,16,1,14,17,D/2+.5,3,SIL);eBox(g,12,1.2,7,17,D/2+.6,6.5,BLK);          // Ethernet
 eBox(g,10,1,4,0,-D/2-.5,8,SIL);                                                // USB-C power
 ePut(g,-222,224.5);}

// --- USB hub (100 x 40 x 20) ---
{const g=new THREE.Group(),W=100,D=40,H=20;
 eBox(g,W,D,H,0,0,0,eM(0x2a2e34,.5,.4));
 for(let i=0;i<4;i++){const x=-33+i*22;eBox(g,14,1,7,x,D/2+.5,6,SIL);eBox(g,12,1.2,5,x,D/2+.6,7,BLK);eBox(g,10,1.3,1.5,x,D/2+.65,8.5,BLU);}   // 4 x USB-A ports
 eBox(g,4,4,.6,44,-13,H,eLed(0x3ecf8e));
 ePut(g,-130,202);}

// --- Arduino Mega 2560 (101.6 x 53.3), blue PCB on standoffs ---
{const g=new THREE.Group(),L=101.6,Wd=53.3,pz=4,pt=1.6,top=pz+pt;
 eBox(g,L,Wd,pt,0,0,pz,eM(0x1d5fae,0,.5));
 [[-46,-22],[-46,22],[46,-22],[46,22]].forEach(([x,y])=>eCyl(g,2.6,pz,x,y,0,GOLD,12));   // standoffs
 eBox(g,16,12,11,-43,13,top,SIL);                              // USB-B
 eBox(g,14,9,11,-43,-14,top,BLK);                              // DC jack
 eBox(g,14,14,1.6,6,-3,top,BLK);                               // ATmega2560
 eBox(g,8,8,1.2,-24,5,top,BLK);                                // USB controller chip
 eBox(g,11,4.5,4,-14,-9,top,SIL);                              // crystal
 eBox(g,6,6,3.5,-33,-20,top,eM(0xd9d9dc,0,.5));                // reset button
 eHdr(g,10,-11,24,top,true);eHdr(g,8,18,24,top,true);          // top edge: digital pins
 eHdr(g,8,-27,-24,top,true);eHdr(g,8,-4,-24,top,true);eHdr(g,8,19,-24,top,true);   // bottom edge: power + analog
 eBox(g,5.08,45.72,8.5,47,0,top,BLK);                          // 2 x 18 digital block at the right end
 [[-30,14,0xff3b30],[-30,11,0x3ecf8e],[-30,8,0xf5c542]].forEach(([x,y,c])=>eBox(g,1.6,.9,.6,x,y,top,eLed(c)));   // status LEDs
 ePut(g,5.8,208.65);}

// --- 6-way fuse box with clear lid (130 x 53 x 30) ---
{const g=new THREE.Group(),W=130,D=53,H=30;
 eBox(g,W,D,4,0,0,0,eM(0x2a2c30,0,.6));
 for(let i=0;i<4;i++)[-14,14].forEach(y=>{eBox(g,10,10,6,-50+i*13,y,4,SIL);eCyl(g,3,2,-50+i*13,y,10,GOLD,12);});   // bus-bar terminals + screws
 [11,-11].forEach((y,row)=>[0,1,2].forEach(i=>{const x=10+i*22;eBox(g,16,5,16,x,y,4,eM(0x23252a,0,.6));   // fuse slots
  if(row===0)eBox(g,12,3,12,x,y,6,eM([0xe49b2b,0xe49b2b,0x2d5fd0][i],0,.4));}));                         // amber, amber, blue blade fuses
 const lid=eBox(g,W,D,H-4,0,0,4,eM(0xdfe3e8,0,.15,{transparent:true,opacity:.28,depthWrite:false,side:THREE.DoubleSide}));lid.renderOrder=2;
 ePut(g,140,208.5);}

// --- 12 V power supply, silver vented case with screw-terminal strip (110 x 55 x 32) ---
{const g=new THREE.Group(),W=110,D=55,H=32,s=3;
 const top=eTex(W*s,D*s,(c,w,h)=>{
  c.fillStyle='#b9c0c8';c.fillRect(0,0,w,h);
  c.fillStyle='#2b2e33';for(let r=0;r<3;r++)for(let q=0;q<4;q++)c.fillRect((8+q*26)*s,(14+r*8)*s,18*s,3*s);   // vent slots
  c.fillStyle='#e6e8eb';c.fillRect(15*s,37*s,80*s,14*s);                                                      // rating label
  c.fillStyle='#2b2e33';c.font=(9*s)+'px sans-serif';c.textAlign='center';c.fillText('12V  5A',w/2,48*s);
 });
 eTop(g,W,D,H,0,0,0,eM(0xb9c0c8,.6,.45),new THREE.MeshStandardMaterial({map:top,metalness:.5,roughness:.45}));
 eBox(g,98,8,7,0,D/2-5,H,eM(0x15171a,0,.6));                                            // terminal block along the top edge
 for(let i=0;i<8;i++)eCyl(g,2.4,1.2,-42+i*12,D/2-5,H+7,GOLD,14);                       // brass screws
 eBox(g,3,3,1.5,-48,D/2-14,H,eLed(0x22c55e));                                           // power LED
 [[13,.6,0x1f2226],[11,.8,0xb9c0c8],[8,1,0x1f2226],[6,1.2,0xb9c0c8],[2.5,1.4,0x1f2226]].forEach(([r,h,c])=>eCylX(g,r,h,W/2+h/2-.2,0,H/2,eM(c,.4,.5),32));   // fan grille on the +X end
 ePut(g,270,209.5);}

// --- BTS7960 motor driver: finned heat sink, blue screw terminals (50 x 50 x 31) ---
{const g=new THREE.Group(),AL=eM(0xc3c9d0,.8,.4);
 eBox(g,50,50,1.6,0,0,2,eM(0x0e6b3f,0,.5));
 [[-22,-22],[-22,22],[22,-22],[22,22]].forEach(([x,y])=>eCyl(g,2.2,2,x,y,0,GOLD,12));
 eBox(g,40,20,3,0,6,3.6,AL);for(let i=0;i<9;i++)eBox(g,1.6,20,13,-18+i*4.5,6,6.6,AL);   // heat sink base + fins
 [-14,14].forEach(x=>eBox(g,13,10,11,x,-18,3.6,BLU));                                    // power screw terminals
 [-18,18].forEach(x=>eCyl(g,4,10,x,-9,3.6,eM(0x1a1b1f,.2,.5),16));                     // capacitors
 eHdr(g,8,0,22,3.6,true);                                                                // control header
 ePut(g,363,207);}

// --- USB-to-servo bus adapter (58 x 38.4 x 22) ---
{const g=new THREE.Group();
 eBox(g,58,38.4,1.6,0,0,3,eM(0x146b45,0,.5));
 [[-26,-16],[-26,16],[26,-16],[26,16]].forEach(([x,y])=>eCyl(g,2.2,3,x,y,0,GOLD,12));
 eBox(g,9,7.5,3.2,-27,0,4.6,SIL);                                                        // USB-C
 [-6,10].forEach(x=>eBox(g,12,7,6,x,-12,4.6,WHT));                                       // 3-pin servo connectors
 eBox(g,6,4,1.5,8,6,4.6,BLK);eBox(g,5,5,1.5,-12,8,4.6,BLK);                             // chips
 eBox(g,7,3.5,2,20,10,4.6,eM(0xd9d9dc,0,.5));                                            // slide switch
 eBox(g,1.6,.9,.6,-18,10,4.6,eLed(0x3ecf8e));
 ePut(g,244,281.2);}

// --- ESP32 dev board, black PCB with pin headers (55 x 28.5 x 17) ---
{const g=new THREE.Group();
 [-11.4,11.4].forEach(y=>eBox(g,48.3,2.54,6,-1,y,0,BLK));                                // pin headers under the board
 eBox(g,55,28.5,1.6,0,0,6,eM(0x15171a,0,.5));
 eBox(g,25.5,18,3.1,-14.7,0,7.6,eM(0xb9bfc7,.8,.35));                                    // ESP32 module shield
 eBox(g,5.6,7.5,2.8,24.7,0,7.6,SIL);                                                     // micro-USB
 eBox(g,5,5,1,14,0,7.6,BLK);                                                             // USB-UART chip
 [8,-8].forEach(y=>eBox(g,3.6,6,2,21,y,7.6,eM(0x2b2e33,0,.5)));                          // BOOT / EN buttons
 eBox(g,1.6,.9,.6,8,4,7.6,eLed(0x2d6cdf));
 ePut(g,310.5,276.25);}

// --- INA226 current-sensor modules x3 (25 x 21 x 16) ---
[360.5,388.5,416.5].forEach(x=>{const g=new THREE.Group();
 eBox(g,25,21,1.6,0,0,3,eM(0x1d3fa0,0,.5));
 [[-10,-8],[10,-8],[-10,8],[10,8]].forEach(([px,py])=>eCyl(g,1.8,3,px,py,0,GOLD,10));
 eBox(g,10,8,10,-7,0,4.6,BLU);                                                           // 2-pin screw terminal
 eBox(g,4,4,1,2,3,4.6,BLK);eBox(g,6,3,1,2,-5,4.6,BLK);                                   // INA226 chip + shunt
 eHdr(g,6,10.3,0,4.6,false);                                                             // 6-pin header
 ePut(g,x,272.5);});

// --- red mushroom E-stop button in the Ø22 hole of the enclosure (top at z=-40) ---
{const g=new THREE.Group();
 eCyl(g,15,3,0,0,0,eM(0x15171a,0,.5),28);                                                // black locking ring
 eCyl(g,19,2,0,0,3,eM(0xf5c542,0,.5),32);                                                // yellow collar
 const dome=new THREE.Mesh(new THREE.CylinderGeometry(16,20,11,36),eM(0xd32626,.1,.35));dome.rotation.x=Math.PI/2;dome.position.set(0,0,5+5.5);g.add(dome);   // mushroom head
 const lab=eTex(128,128,(c,w,h)=>{c.fillStyle='#d32626';c.fillRect(0,0,w,h);c.fillStyle='#fff';c.font='bold 30px sans-serif';c.textAlign='center';c.fillText('STOP',w/2,h/2+10);});
 const cap=new THREE.Mesh(new THREE.CircleGeometry(16,36),new THREE.MeshStandardMaterial({map:lab,roughness:.35}));cap.position.set(0,0,5+11+.05);g.add(cap);
 g.position.set(-380,-170,-40);scene.add(g);eList.push(g);}
// ---------- block geometry (40x30x20, 3 mm top chamfer) ----------
function blockGeo(){
  const v=[],A=[[-20,-15],[20,-15],[20,15],[-20,15]],Bv=[[-17,-12],[17,-12],[17,12],[-17,12]];
  A.forEach(p=>v.push(p[0],p[1],-10));A.forEach(p=>v.push(p[0],p[1],7));Bv.forEach(p=>v.push(p[0],p[1],10));
  const ix=[];for(let r=0;r<2;r++)for(let k=0;k<4;k++){const a=r*4+k,b=r*4+(k+1)%4;ix.push(a,b,b+4,a,b+4,a+4);}
  ix.push(0,2,1,0,3,2);ix.push(8,9,10,8,10,11);
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(v,3));g.setIndex(ix);return g;}
const BG=blockGeo();
const spotG=new THREE.CylinderGeometry(4,4,.6,16);spotG.rotateX(Math.PI/2);
const blocks=[];
function mkBlock(kind){
  const m=new THREE.Mesh(BG,new THREE.MeshStandardMaterial({color:0xf2f0ec,roughness:.55,flatShading:true,emissive:0x000000,side:THREE.DoubleSide}));
  if(kind==='fail'){const s=new THREE.Mesh(spotG,new THREE.MeshStandardMaterial({color:0x2a2d33,roughness:.9}));s.position.set(6,-3,10.2);m.add(s);}
  scene.add(m);return{m,kind,x:0,y:0,z:0,state:'mag',v:0};}
// ---------- arm FK ----------
const Rz=a=>new THREE.Matrix4().makeRotationZ(a);
function links(q,g){
  const X=new THREE.Matrix4(),L={base:WM.clone()},nm=['shoulder','upper','lower','wrist','grip'];
  for(let i=0;i<5;i++){X.multiply(CH[i].T).multiply(new THREE.Matrix4().makeRotationAxis(CH[i].ax,q[i]));L[nm[i]]=WM.clone().multiply(X);}
  L.jaw=WM.clone().multiply(X).multiply(GT).multiply(Rz(g));return L;}
const TR=META.traj,K=TR.keys,REL=M4(TR.rel),GO=TR.g_open,GC=TR.g_close;
const lerpQ=(a,b,t)=>a.map((v,i)=>v+(b[i]-v)*t);
function pathQ(nodes,s){const n=nodes.length-1,x=Math.min(Math.max(s,0),1)*n,i=Math.min(Math.floor(x),n-1);return lerpQ(nodes[i],nodes[i+1],x-i);}
const sm=t=>{t=Math.min(Math.max(t,0),1);return t*t*t*(t*(6*t-15)+10);};
// ---------- cameras ----------
const PRE={ov:[[170,-820,560],[90,-70,10]],pick:[[440,200,200],[235,-30,35]],top:[[90,-330,880],[90,-60,0]],ins:[[-60,-330,230],[-232,0,90]],tray:[[215,-600,300],[215,-140,-10]]};
let camGoal=null,camT=0;
function goCam(k){camGoal={p:new THREE.Vector3(...PRE[k][0]),t:new THREE.Vector3(...PRE[k][1])};camT=1.6;}
cam.position.set(...PRE.ov[0]);ctl.target.set(...PRE.ov[1]);
// ---------- part visibility panel: toggle groups, or "solo" one group and frame it ----------
const GRP=[
 {k:'arm',t:'แขน SO-101 + ฐาน',re:/^A1|M14_arm_base|M15_arm_rest/},
 {k:'conv',t:'สายพานลำเลียง',re:/belt_strip|M01_roller|M02_bearing|M03|M04_motor|M24|M25|ENV_bearing|M26|JGB|M27|M28|M29|M17|M16|M33/},
 {k:'feed',t:'แมกกาซีน + ใบพุช',re:/M05_magazine|M06_blade|M07_servo|MG90S/},
 {k:'cam',t:'กล้อง D415 + ไฟ',re:/M09_camera|M10_post|M11/},
 {k:'tray',t:'ถาด PASS / FAIL',re:/M12_pass|M13_fail/},
 {k:'frame',t:'โครงอลูมิเนียม',re:/ENV_2020|M22|M23|M08/},
 {k:'elec',t:'วงจร + แผ่นดาดฟ้า',re:/M20|M31|M32|ENV_/},
 {k:'blk',t:'บล็อก',re:null}];
const GM={},PV={};GRP.forEach(g=>{GM[g.k]=[];PV[g.k]=true;});
root.children.forEach(o=>{const n=o.userData.n||'';const g=GRP.find(g=>g.re&&g.re.test(n))||GRP[5];GM[g.k].push(o);});   // classify every CAD part by name
GM.conv.push(beltTop);GM.cam.push(fov,d415);eList.forEach(o=>GM.elec.push(o));                                        // extra non-CAD objects
const panel=$('parts');
panel.innerHTML='<div class="ph">ชิ้นส่วน<span>แตะชื่อ = ซ่อน/แสดง · "เดี่ยว" = ดูเฉพาะชิ้นนั้น</span></div>'
 +GRP.map(g=>'<div class="pr"><button class="pt on" data-k="'+g.k+'">'+g.t+'</button><button class="ps" data-k="'+g.k+'">เดี่ยว</button></div>').join('')
 +'<button class="pa" id="pall">แสดงทั้งหมด</button>';
function applyVis(){
  GRP.forEach(g=>{if(g.k!=='blk')GM[g.k].forEach(o=>{o.visible=PV[g.k];});});                                        // blocks are re-created each cycle, so frame() applies PV.blk
  panel.querySelectorAll('.pt').forEach(b=>b.classList.toggle('on',PV[b.dataset.k]));}
function frameGroup(k){
  if(!GM[k].length){goCam('pick');return;}
  scene.updateMatrixWorld(true);const b=new THREE.Box3();GM[k].forEach(o=>b.expandByObject(o));
  if(b.isEmpty())return;
  const c=b.getCenter(new THREE.Vector3()),r=b.getSize(new THREE.Vector3()).length()/2,d=Math.max(r*2.9,90);
  camGoal={p:c.clone().addScaledVector(new THREE.Vector3(.3,-.8,.5).normalize(),d),t:c};camT=1.6;}
panel.addEventListener('click',e=>{
  const b=e.target.closest('button');if(!b)return;
  if(b.id==='pall'){GRP.forEach(g=>{PV[g.k]=true;});applyVis();goCam('ov');return;}
  const k=b.dataset.k;
  if(b.classList.contains('pt')){PV[k]=!PV[k];applyVis();return;}
  const solo=GRP.every(g=>PV[g.k]===(g.k===k));                                                                       // tapping "solo" again restores everything
  if(solo){GRP.forEach(g=>{PV[g.k]=true;});applyVis();goCam('ov');}
  else{GRP.forEach(g=>{PV[g.k]=g.k===k;});applyVis();frameGroup(k);}});
// ======================================================================================================
// LIVE ENGINE  -  everything below is NEW (the scene, the CAD parts, the electronics and the arm kinematics above come
// unchanged from the CAD-twin artifact). The artifact ran a made-up timeline; here every moving thing is driven by the
// REAL numbers the cell publishes:
//   arm   : arm/telemetry joint ticks  -> five joint angles + jaw angle (map below) -> forward kinematics links(q,g)
//   belt  : belt/telemetry velocity    -> belt texture scroll, roller + shaft rotation, block travel
//   feed  : autoen phase FEED          -> M06 blade stroke, block pushed out of the magazine
//   cam   : phase TRANSPORT start      -> D415 LED + field-of-view flash (inspection)
//   block : phase machine (FEED > TRANSPORT > GRIP > CARRY_* > RELEASE) + IR edge to re-sync the block on the belt
// Whatever is estimated (not measured) is listed in the "what is real" panel on the page.
// ======================================================================================================
const TWO_PI_TICKS = 2 * Math.PI / 4096;
const FEED_X0 = -281, BLK_PICK = TR.block_pick[0];
const ROLLER_R = 23;                                       // belt roller radius (mm): belt travel = revolutions x 2 pi R
const MM_PER_REV = 2 * Math.PI * ROLLER_R;
const PASS_TRAY = TRAYS_XY('pass'), FAIL_TRAY = TRAYS_XY('fail');
function TRAYS_XY(k) { return k === 'pass' ? { x: 410, y: -160, n: 0 } : { x: 20, y: -160, n: 0 }; }
const trays = { pass: PASS_TRAY, fail: FAIL_TRAY };

// ---- real ticks -> model angles. Default fitted from the named poses (see README): pick ('up') <-> model pick_down.
const DEFAULT_MAP = {
  joints: [{ sign: 1, zero: 1988 }, { sign: 1, zero: 1826 }, { sign: 1, zero: 2217 }, { sign: 1, zero: 1961 }, { sign: -1, zero: 2263 }],
  gripper: { zero: 2108 }
};
let MAP = JSON.parse(JSON.stringify(DEFAULT_MAP));
const toQ = ticks => MAP.joints.map((j, i) => ticks[i] == null ? null : j.sign * (ticks[i] - j.zero) * TWO_PI_TICKS);
const toG = t => t == null ? null : GC + (t - MAP.gripper.zero) * TWO_PI_TICKS;     // jaw: closed-on-block ticks <-> g_close

// ---- smoothed pose (telemetry arrives at 10 Hz, the screen runs at 60 Hz)
const S = {
  q: K.park.slice(), g: TR.g_park, qt: null, gt: null,
  beltV: 0, beltX: 0, stroke: 0, ledA: 0,
  ticks: null, armAge: null, beltAge: null,
  phase: 'IDLE', phaseT0: 0, drop: '', status: 'idle', cycle: 0,
  ir: false, irPrev: false, camOn: false,
  live: false, counts: { pass: 0, fail: 0 }, lastPose: null
};
let cur = null;                                            // the block of the current cycle
function newBlock(kind) {
  const b = mkBlock(kind || 'pass'); b.x = FEED_X0; b.y = 0; b.z = 10; b.state = 'push'; b.t = 0;
  b.m.position.set(b.x, b.y, b.z); blocks.push(b); return b; }
function dropBlock(b) { scene.remove(b.m); const i = blocks.indexOf(b); if (i >= 0) blocks.splice(i, 1); }
function clearTrays() { blocks.slice().forEach(dropBlock); trays.pass.n = 0; trays.fail.n = 0; S.counts = { pass: 0, fail: 0 }; cur = null; }

// ---- phase machine: one handler per phase ENTRY. autoen publishes the phase at 1 Hz and arm/state at 10 Hz; the 10 Hz one is used.
function enterPhase(ph, prev) {
  S.phaseT0 = performance.now() / 1000;
  if (ph === 'FEED') {
    if (cur && cur.state !== 'tray' && cur.state !== 'drop') dropBlock(cur);
    cur = newBlock('pass'); S.stroke = 0;
  } else if (ph === 'TRANSPORT') {
    if (!cur) { cur = newBlock('pass'); cur.x = FEED_X0 + 55; }
    cur.state = 'belt'; cur.m.material.emissive.set(0x000000);
  } else if (ph === 'GRIP') {
    if (!cur) { cur = newBlock('pass'); }
    cur.state = 'belt'; cur.x = BLK_PICK; cur.y = TR.block_pick[1];
  } else if (ph.startsWith('CARRY')) {
    if (!cur || cur.state === 'tray' || cur.state === 'drop') { cur = newBlock('pass'); }        // dashboard opened mid-cycle: the block is in the jaw
    if (cur.state !== 'held') cur.state = 'held';
  } else if (ph === 'RELEASE') {
    if (!cur || cur.state === 'tray' || cur.state === 'drop') { cur = newBlock('pass'); cur.state = 'held'; }
    if (cur.state === 'held') startDrop(cur);
  } else if (ph === 'RECOVER') {
    if (cur) { dropBlock(cur); cur = null; }
  }
}
function startDrop(b) {
  // real pose names: drop 'left' = left bin = FAIL tray, 'down' = right bin = PASS tray
  const kind = (S.drop === 'left') ? 'fail' : 'pass';
  if (b.kind !== kind) { scene.remove(b.m); const nb = mkBlock(kind); nb.m.position.copy(b.m.position); nb.m.quaternion.copy(b.m.quaternion); blocks[blocks.indexOf(b)] = nb; b = nb; cur = nb; }
  const T = trays[kind], n = T.n++;
  b.state = 'drop'; b.dt = 0; b.from = b.m.position.clone(); b.fq = b.m.quaternion.clone();
  b.to = new THREE.Vector3(T.x, T.y, -95 + 10 + 20 * (n % 8));
  const yaw = new THREE.Euler().setFromQuaternion(b.m.quaternion, 'ZYX').z;
  b.tq = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.round(yaw / (Math.PI / 2)) * Math.PI / 2);
  b.kindT = kind; S.counts[kind]++;
}

// ---- called by the page for every live snapshot (10 Hz)
function onLive(L) {
  const A = L.arm, B = L.belt;
  S.armAge = A ? A.age : null; S.beltAge = B ? B.age : null;
  S.live = !!(A && A.age != null && A.age < 1.5);
  if (A && A.joints) {
    const ticks = A.joints.map(j => j.pos);
    S.ticks = ticks;
    if (ticks.slice(0, 5).every(v => v != null)) S.qt = toQ(ticks);
    if (ticks[5] != null) S.gt = toG(ticks[5]);
  }
  S.beltV = (B && B.d && B.age < 1.5 && B.d.velocity != null) ? Math.abs(B.d.velocity) / 60 * MM_PER_REV : 0;   // rpm -> mm/s (direction assumed +X)
  // a retained copy from an earlier session is not 'now': only messages that are still arriving count
  const st = L.arm_state && L.arm_state.age != null && L.arm_state.age < 3 ? L.arm_state.d : null;
  const ae = L.autoen && L.autoen.age != null && L.autoen.age < 5 ? L.autoen.d : null;
  const ph = (st && st.phase) || (ae && ae.phase) || 'IDLE';
  if (st && st.drop) S.drop = st.drop; else if (ae && ae.drop) S.drop = ae.drop;
  S.status = (ae && ae.status) || (st && st.status) || 'idle';
  if (ph !== S.phase) { const prev = S.phase; S.phase = ph; enterPhase(ph, prev); }
  const cyc = (ae && ae.cycle != null) ? ae.cycle : (st && st.cycle);
  if (cyc != null) S.cycle = cyc;
  S.ir = !!(L.ir && L.ir.detected);
  if (S.ir && !S.irPrev && cur && cur.state === 'belt') cur.x = BLK_PICK;               // the IR is the truth: re-sync the block on the belt
  S.irPrev = S.ir;
}

// ---- per-frame
const ease2 = t => t < .5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
let prevNow = 0, ledLevel = 0;
function stepLive(now, raw) {
  const t = now / 1000, inPhase = t - S.phaseT0;
  // pose smoothing: first-order lag, fast enough to follow a 10 Hz source without steps
  const k = 1 - Math.exp(-raw * 14);
  if (S.qt) S.q = S.q.map((v, i) => S.qt[i] == null ? v : v + (S.qt[i] - v) * k);
  if (S.gt != null) S.g += (S.gt - S.g) * k;
  // belt
  S.beltX += S.beltV * raw; bt.offset.x -= S.beltV * raw / 40;
  // feeder blade: stroke out in the first 45 % of FEED, back in the rest (autoen gives no blade position; time-based estimate)
  const FEED_T = 2.4;
  if (S.phase === 'FEED' && S.status !== 'error') {
    const u = Math.min(inPhase / FEED_T, 1); S.stroke = u < .45 ? 55 * ease2(u / .45) : 55 * (1 - ease2((u - .45) / .55));
  } else S.stroke += (0 - S.stroke) * Math.min(1, raw * 6);
  if (cur && cur.state === 'push') {
    cur.x = FEED_X0 + Math.min(S.stroke, 55);
    if (cur.x > -250) { cur.state = 'fall'; cur.fz = 0; }
  }
  blocks.forEach(b => {
    if (b === cur && b.state === 'fall') { b.fz += raw; b.z = Math.max(10, b.z - b.fz * 900 * raw - 60 * raw); if (b.z <= 10) b.state = 'belt'; b.m.position.set(b.x, b.y, b.z); }
    else if (b.state === 'belt') { if (S.phase === 'TRANSPORT' || S.beltV > 1) b.x = Math.min(BLK_PICK, b.x + S.beltV * raw); b.m.position.set(b.x, b.y, b.z); b.m.quaternion.identity(); }
    else if (b.state === 'push') b.m.position.set(b.x, b.y, b.z);
  });
  // inspection LED + field of view while the block is first seen
  const ins = (S.phase === 'TRANSPORT' && inPhase < 2.0) ? 1 : 0;
  ledLevel += (ins - ledLevel) * Math.min(1, raw * 8);
  extra.leds.forEach(me => me.material.emissive.setScalar(ledLevel * .9));
  fov.material.opacity = ledLevel * (.12 + .05 * Math.sin(now / 90));
}
function applyLive(raw) {
  const Lk = links(S.q, S.g);
  armParts.forEach(me => { me.matrix.copy(Lk[me.userData.link]).multiply(me.userData.M0); });
  blocks.forEach(b => {
    if (b.state === 'held') { const m = Lk.grip.clone().multiply(REL); m.decompose(b.m.position, b.m.quaternion, b.m.scale); }
    else if (b.state === 'drop') {
      b.dt += raw; const T = .5, s = Math.min(b.dt / T, 1);
      b.m.position.set(b.from.x + (b.to.x - b.from.x) * ease2(s), b.from.y + (b.to.y - b.from.y) * ease2(s), b.from.z + (b.to.z - b.from.z) * s * s);
      b.m.quaternion.copy(b.fq).slerp(b.tq, ease2(s)); if (s >= 1) b.state = 'tray';
    }
    b.m.visible = PV.blk;
  });
  // rollers + shafts turn with the belt
  const th = S.beltX / ROLLER_R;
  const rot = (me, x0) => { const T = new THREE.Matrix4().makeTranslation(x0, 0, -24), Ti = new THREE.Matrix4().makeTranslation(-x0, 0, 24);
    me.matrix.copy(T).multiply(new THREE.Matrix4().makeRotationY(th)).multiply(Ti).multiply(me.userData.M0); };
  extra.rollers.concat(extra.shafts).forEach(me => { const c = me.geometry.boundingSphere.center.clone().applyMatrix4(me.userData.M0); rot(me, Math.sign(c.x) * 363.87); });
  if (parts.blade) parts.blade.matrix.makeTranslation(S.stroke, 0, 0).multiply(parts.blade.userData.M0);
}

// ---- render loop (paused while the page is hidden)
let running = false, camFollow = false;
function frame(now) {
  if (!running) return;
  requestAnimationFrame(frame);
  // Real elapsed time, capped at 0.25 s: pose smoothing, belt travel and drops must follow the clock, not the frame count
  // (a slow machine renders few frames per second; clamping to 50 ms made the model lag behind the real arm).
  const raw = Math.min((now - (prevNow || now)) / 1000, .25); prevNow = now;
  stepLive(now, raw); applyLive(raw);
  if (camGoal && camT > 0) { camT -= raw; const kk = 1 - Math.exp(-raw * 5); cam.position.lerp(camGoal.p, kk); ctl.target.lerp(camGoal.t, kk); }
  ctl.update(); R.render(scene, cam);
}
function fit() {
  const w = cv.parentElement.clientWidth || 800, h = cv.parentElement.clientHeight || 500;
  R.setSize(w, h, false); cam.aspect = w / h; cam.updateProjectionMatrix();
}
new ResizeObserver(fit).observe(cv.parentElement); fit();
cam.position.set(...PRE.ov[0]); ctl.target.set(...PRE.ov[1]);

// ---- align: put the REAL arm into a pose the model knows, press the button -> zero offsets are set from the live ticks
function alignTo(poseKey) {
  if (!S.ticks) throw new Error('no arm telemetry yet');
  const ref = poseKey === 'closed' ? null : K[poseKey];
  if (poseKey === 'closed') { MAP.gripper.zero = S.ticks[5]; return MAP; }
  if (!ref) throw new Error('unknown model pose ' + poseKey);
  MAP.joints.forEach((j, i) => { j.zero = +(S.ticks[i] - j.sign * ref[i] / TWO_PI_TICKS).toFixed(1); });
  return MAP;
}
const api = {
  onLive,
  start() { if (!running) { running = true; prevNow = 0; requestAnimationFrame(frame); fit(); } },
  stop() { running = false; },
  goCam, frameGroup, state: S, poses: Object.keys(K),
  getMap: () => JSON.parse(JSON.stringify(MAP)),
  setMap(m) { if (m && m.joints && m.joints.length === 5 && m.gripper) MAP = JSON.parse(JSON.stringify(m)); },
  defaultMap: () => JSON.parse(JSON.stringify(DEFAULT_MAP)),
  alignTo, clearTrays,
  togglePanel() { panel.hidden = !panel.hidden; return !panel.hidden; },
  modelQ: () => S.q.slice(), counts: () => S.counts,
  stats: () => ({ parts: root.children.length, blocks: blocks.length })
};
return api;

};

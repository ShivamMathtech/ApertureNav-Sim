import {useEffect,useRef} from 'react';
import * as THREE from 'three';
import {OrbitControls} from 'three/examples/jsm/controls/OrbitControls.js';
import type {Telemetry} from './types';
export function Scene({data,view}:{data:Telemetry|null;view:string}){
 const host=useRef<HTMLDivElement>(null),current=useRef(data),mode=useRef(view);
 useEffect(()=>{current.current=data},[data]);useEffect(()=>{mode.current=view},[view]);
 useEffect(()=>{
  const el=host.current!;const scene=new THREE.Scene();scene.background=new THREE.Color('#0b131e');scene.fog=new THREE.Fog('#0b131e',16,35);
  const camera=new THREE.PerspectiveCamera(45,1,.01,100);camera.up.set(0,0,1);camera.position.set(-2,-7,5);
  let renderer:THREE.WebGLRenderer;try{renderer=new THREE.WebGLRenderer({antialias:true})}catch{el.textContent='3D rendering is unavailable. Telemetry and controls remain usable.';return}
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));el.appendChild(renderer.domElement);
  const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(3,0,1.3);controls.enableDamping=true;
  scene.add(new THREE.AmbientLight(0xffffff,2));const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(-3,-4,8);scene.add(light);
  const grid=new THREE.GridHelper(30,60,0x344559,0x1d2c3c);grid.rotation.x=Math.PI/2;scene.add(grid);
  const drone=new THREE.Group();const mat=new THREE.MeshStandardMaterial({color:0x51e8ba,metalness:.5,roughness:.3});
  const body=new THREE.Mesh(new THREE.BoxGeometry(.20,.13,.10),mat);drone.add(body);
  for(const x of [-.12,.12])for(const y of [-.12,.12]){
   const arm=new THREE.Mesh(new THREE.BoxGeometry(.27,.025,.025),mat);arm.rotation.z=Math.atan2(y,x);drone.add(arm);
   const prop=new THREE.Mesh(new THREE.CylinderGeometry(.06,.06,.008,24),new THREE.MeshStandardMaterial({color:0x91adc5}));prop.rotation.x=Math.PI/2;prop.position.set(x,y,.04);drone.add(prop);
  }scene.add(drone);
  const walls=new THREE.Group();scene.add(walls);let signature='';
  const wallMat=new THREE.MeshStandardMaterial({color:0x32475c,roughness:.9,transparent:true,opacity:.8});
  const lineMat=new THREE.LineBasicMaterial({color:0x54e8c1});let trail:number[]=[];let last=-1;
  const path=new THREE.Line(new THREE.BufferGeometry(),lineMat);scene.add(path);
  const planned=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineDashedMaterial({color:0x7c91ff,dashSize:.12,gapSize:.09}));scene.add(planned);
  function box(w:number,h:number,d:number,x:number,y:number,z:number){const b=new THREE.Mesh(new THREE.BoxGeometry(w,h,d),wallMat);b.position.set(x,y,z);walls.add(b)}
  const observer=new ResizeObserver(()=>{const {width,height}=el.getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix()});observer.observe(el);
  let frame=0;function draw(){frame=requestAnimationFrame(draw);const s=current.current;
   if(s?.position){drone.position.set(...s.position as [number,number,number]);drone.rotation.z=s.yaw;
    if(s.time<last){trail=[]}if(s.time!==last){trail.push(...s.position);if(trail.length>9000)trail.splice(0,3);path.geometry.dispose();path.geometry=new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(trail,3));last=s.time}
    const a=s.aperture;const key=JSON.stringify(a);if(key!==signature){signature=key;for(const child of [...walls.children]){(child as THREE.Mesh).geometry.dispose();walls.remove(child)}
     const [x,y,z]=a.center;walls.position.set(x,y,z);walls.rotation.z=a.yaw;
     box(a.thickness,2,3,0,-a.width/2-1,0);box(a.thickness,2,3,0,a.width/2+1,0);
     box(a.thickness,a.width,1.5-a.height/2,0,0,(1.5+a.height/2)/2);box(a.thickness,a.width,1.5-a.height/2,0,0,-(1.5+a.height/2)/2);
     const p=[new THREE.Vector3(x-Math.cos(a.yaw),y-Math.sin(a.yaw),z),new THREE.Vector3(x,y,z),new THREE.Vector3(x+Math.cos(a.yaw),y+Math.sin(a.yaw),z)];planned.geometry.dispose();planned.geometry=new THREE.BufferGeometry().setFromPoints(p);planned.computeLineDistances();
    }
    controls.enabled=mode.current==='Orbit';
    if(mode.current==='Top'){camera.position.set(3,0,10);camera.lookAt(3,0,0)}
    if(mode.current==='Side'){camera.position.set(3,-9,2);camera.lookAt(3,0,1.5)}
    if(mode.current==='FPV'){camera.position.copy(drone.position).add(new THREE.Vector3(.2*Math.cos(s.yaw),.2*Math.sin(s.yaw),.04));camera.lookAt(camera.position.clone().add(new THREE.Vector3(Math.cos(s.yaw),Math.sin(s.yaw),0)))}
   }if(controls.enabled)controls.update();renderer.render(scene,camera);
  }draw();return()=>{cancelAnimationFrame(frame);observer.disconnect();controls.dispose();scene.traverse(obj=>{if(obj instanceof THREE.Mesh||obj instanceof THREE.Line){obj.geometry.dispose();const m=obj.material;for(const v of Array.isArray(m)?m:[m])v.dispose()}});renderer.dispose();el.removeChild(renderer.domElement)};
 },[]);return <div className="scene" ref={host}/>;
}

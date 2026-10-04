"""Generate self-contained SDF worlds from the same YAML as navigation."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

def element(parent, tag, text=None, **attrs):
    out=ET.SubElement(parent,tag,attrs)
    if text is not None:out.text=str(text)
    return out

def box(parent,name,size,pose,color='0.3 0.38 0.48 1',collision=True):
    for kind in (['collision','visual'] if collision else ['visual']):
        part=element(parent,kind,name=name+'_'+kind);element(part,'pose',' '.join(map(str,pose)))
        geo=element(part,'geometry');shape=element(geo,'box');element(shape,'size',' '.join(map(str,size)))
        if kind=='visual':
            material=element(part,'material');element(material,'diffuse',color);element(material,'ambient',color)

def generate(c,path,empty=False,lighting=1.):
    root=ET.Element('sdf',version='1.9');world=element(root,'world',name='aperture')
    physics=element(world,'physics',name='default',type='ignored');element(physics,'max_step_size',.002);element(physics,'real_time_factor',1)
    element(world,'gravity','0 0 -9.81')
    for filename,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('imu','Imu')]:
        p=element(world,'plugin',filename='gz-sim-'+filename+'-system',name='gz::sim::systems::'+name)
        if name=='Sensors':element(p,'render_engine','ogre2')
    scene=element(world,'scene');element(scene,'ambient',f'{.5*lighting} {.5*lighting} {.5*lighting} 1');element(scene,'background','0.06 0.08 0.12 1')
    light=element(world,'light',name='sun',type='directional');element(light,'pose','0 0 8 0 0 0');element(light,'diffuse',f'{lighting} {lighting} {lighting} 1');element(light,'direction','-.3 .2 -1');element(light,'cast_shadows','true')
    ground=element(world,'model',name='room');element(ground,'static','true');link=element(ground,'link',name='structure')
    box(link,'floor',[14,10,.1],[3,0,-.05,0,0,0],'0.12 0.17 0.22 1')
    box(link,'back_wall',[.15,10,4],[9,0,2,0,0,0],'0.2 0.24 0.28 1')
    a=c['aperture'];w,h,t=a['width'],a['height'],a['thickness']
    if not empty:
        wall=element(world,'model',name='window_wall');element(wall,'static','true')
        element(wall,'pose',' '.join(map(str,[*a['center'],0,0,a['yaw']])))
        link=element(wall,'link',name='wall')
        box(link,'left',[t,3,4],[0,-w/2-1.5,0,0,0,0]);box(link,'right',[t,3,4],[0,w/2+1.5,0,0,0,0])
        box(link,'bottom',[t,w,2-h/2],[0,0,-(2+h/2)/2,0,0,0]);box(link,'top',[t,w,2-h/2],[0,0,(2+h/2)/2,0,0,0])
        for name,size,pose in [('left',[t+.02,.025,h],[0,-w/2-.0125,0,0,0,0]),('right',[t+.02,.025,h],[0,w/2+.0125,0,0,0,0]),('top',[t+.02,w,.025],[0,0,h/2+.0125,0,0,0]),('bottom',[t+.02,w,.025],[0,0,-h/2-.0125,0,0,0])]:box(link,'trim_'+name,size,pose,'0.65 0.9 0.78 1')
    drone=element(world,'model',name='drone');element(drone,'pose',' '.join(map(str,[*c['drone']['spawn'],0,0,0])))
    link=element(drone,'link',name='base_link');element(link,'gravity','false')
    inertia=element(link,'inertial');element(inertia,'mass',1.0);tensor=element(inertia,'inertia')
    for k,v in [('ixx',.012),('iyy',.012),('izz',.022),('ixy',0),('ixz',0),('iyz',0)]:element(tensor,k,v)
    box(link,'body',[.2,.13,.10],[0,0,0,0,0,0],'0.15 0.7 0.55 1',False)
    collision=element(link,'collision',name='vehicle_envelope');geo=element(collision,'geometry');shape=element(geo,'box')
    element(shape,'size',f"{c['drone']['length']} {c['drone']['width']} {c['drone']['height']}")
    for x in (-.12,.12):
        for y in (-.12,.12):
            suffix=f'{x}_{y}'
            box(link,'arm_'+suffix,[.30,.025,.025],[0,0,0,0,0,math.atan2(y,x)],'0.2 0.25 0.3 1',False)
            visual=element(link,'visual',name='prop_'+suffix);element(visual,'pose',f'{x} {y} .04 0 0 0')
            geometry=element(visual,'geometry');cylinder=element(geometry,'cylinder');element(cylinder,'radius',.06);element(cylinder,'length',.008)
    sensor=element(link,'sensor',name='rgbd',type='rgbd_camera');element(sensor,'pose','.18 0 .03 0 0 0');element(sensor,'always_on','true');element(sensor,'update_rate',30);element(sensor,'topic','/drone/rgbd');element(sensor,'gz_frame_id','camera_optical_frame')
    camera=element(sensor,'camera');element(camera,'horizontal_fov',1.5707963)
    img=element(camera,'image');element(img,'width',640);element(img,'height',480);element(img,'format','R8G8B8')
    clip=element(camera,'clip');element(clip,'near',.05);element(clip,'far',20)
    depth=element(camera,'depth_camera');clip=element(depth,'clip');element(clip,'near',.05);element(clip,'far',20)
    imu=element(link,'sensor',name='imu',type='imu');element(imu,'always_on','true');element(imu,'update_rate',100);element(imu,'topic','/drone/imu');element(imu,'gz_frame_id','imu_link')
    control=element(drone,'plugin',filename='gz-sim-velocity-control-system',name='gz::sim::systems::VelocityControl');element(control,'topic','/drone/cmd_vel')
    odom=element(drone,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
    for k,v in [('odom_frame','odom'),('robot_base_frame','base_link'),('odom_topic','/drone/odom'),('odom_publish_frequency',50),('dimensions',3)]:element(odom,k,v)
    ET.indent(root);Path(path).write_text(ET.tostring(root,encoding='unicode'))

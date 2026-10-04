"""ROS adapters. Algorithms are in aperturenav; all frames use ENU metres."""
import csv
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Point
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ros_gz_interfaces.srv import ControlWorld
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformException
from cv_bridge import CvBridge
from drone_interfaces.msg import ApertureDetection,AperturePose
from aperturenav.config import load
from aperturenav.geometry import Aperture,clearance,waypoints,wrap_angle,swept_collision
from aperturenav.control import PID
from aperturenav.mission import Mission
from aperturenav.perception import detect_rectangles,localize_frame
from aperturenav.simulation import Simulation


def seconds(stamp):return stamp.sec+stamp.nanosec*1e-9

def yaw_of(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

class Base(Node):
    def __init__(self,name):
        super().__init__(name)
        self.declare_parameter('config_file','config/default.yaml')
        self.c=load(self.get_parameter('config_file').value)
        self.diag=self.create_publisher(DiagnosticArray,'/diagnostics',10)
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def status(self,text,level=0):
        m=DiagnosticArray();m.header.stamp=self.get_clock().now().to_msg()
        d=DiagnosticStatus();d.name=self.get_name();d.level=level;d.message=text;d.hardware_id='simulation'
        m.status=[d];self.diag.publish(m)

class Detector(Base):
    def __init__(self):
        super().__init__('aperture_detector');self.bridge=CvBridge();self.image=None;self.last_stamp=None
        self.declare_parameter('min_area',500);self.declare_parameter('rate',15.)
        self.pub=self.create_publisher(ApertureDetection,'/aperture/detection',10)
        self.debug=self.create_publisher(Image,'/aperture/debug_image',10)
        self.compressed=self.create_publisher(CompressedImage,'/drone/camera/compressed',qos_profile_sensor_data)
        self.create_subscription(Image,'/drone/camera/image_raw',lambda msg:setattr(self,'image',msg),qos_profile_sensor_data)
        self.create_timer(1/self.get_parameter('rate').value,self.process)
    def process(self):
        msg=self.image
        if msg is None or seconds(msg.header.stamp)==self.last_stamp:return
        self.last_stamp=seconds(msg.header.stamp)
        try:
            import cv2
            image=self.bridge.imgmsg_to_cv2(msg,'bgr8')
            candidates=detect_rectangles(image,self.get_parameter('min_area').value)
            # Publish each candidate; localization checks frame depth and open interior.
            for detection in candidates[:10]:
                out=ApertureDetection();out.header=msg.header;out.header.frame_id='camera_optical_frame'
                from geometry_msgs.msg import Point32
                out.corners=[Point32(x=float(x),y=float(y),z=0.) for x,y in detection.corners]
                out.confidence=float(detection.confidence);out.detector_type='classical_rectangle'
                self.pub.publish(out)
                cv2.polylines(image,[detection.corners.astype(np.int32)],True,(90,240,150),2)
            debug=self.bridge.cv2_to_imgmsg(image,'bgr8');debug.header=msg.header;self.debug.publish(debug)
            ok,jpg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])
            if ok:self.compressed.publish(CompressedImage(header=msg.header,format='jpeg',data=jpg.tobytes()))
            self.status(f'{len(candidates)} rectangular candidates')
        except (ValueError,RuntimeError) as error:
            self.get_logger().warning(str(error));self.status(str(error),1)

class Localizer(Base):
    def __init__(self):
        super().__init__('aperture_localizer');self.bridge=CvBridge();self.depth=None;self.info=None
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self);self.tf=TransformBroadcaster(self)
        self.pub=self.create_publisher(AperturePose,'/aperture/pose',10)
        self.create_subscription(CameraInfo,'/drone/camera/camera_info',lambda msg:setattr(self,'info',msg),qos_profile_sensor_data)
        self.create_subscription(Image,'/drone/depth/image_raw',lambda msg:setattr(self,'depth',msg),qos_profile_sensor_data)
        self.create_subscription(ApertureDetection,'/aperture/detection',self.localize,10)
    def localize(self,msg):
        if self.depth is None or self.info is None:return
        if abs(seconds(msg.header.stamp)-seconds(self.depth.header.stamp))>.1:return
        try:
            depth=self.bridge.imgmsg_to_cv2(self.depth,desired_encoding='passthrough').astype(float)
            if self.depth.encoding=='16UC1':depth*=.001
            corners=np.array([[p.x,p.y] for p in msg.corners]);k=self.info.k
            estimate=localize_frame(corners,depth,[k[0],k[4],k[2],k[5]])
            # A wall panel/rectangle is not an opening. Require valid depth behind its rim.
            u,v=np.rint(corners.mean(0)).astype(int)
            interior=depth[max(0,v-3):v+4,max(0,u-3):u+4]
            valid=interior[np.isfinite(interior)&(interior>0)]
            if len(valid)<3 or np.median(valid)<estimate['center'][2]+.3:return
            tr=self.buffer.lookup_transform('world','camera_optical_frame',Time.from_msg(msg.header.stamp))
            rot=rotation(tr.transform.rotation);p=tr.transform.translation
            center=rot@estimate['center']+np.array([p.x,p.y,p.z]);normal=rot@estimate['normal']
            if abs(normal[2])>.25:return  # Baseline supports vertical wall planes only.
            yaw=math.atan2(normal[1],normal[0])
            out=AperturePose();out.header=msg.header;out.header.frame_id='world'
            out.pose.position.x,out.pose.position.y,out.pose.position.z=map(float,center)
            out.pose.orientation.z=math.sin(yaw/2);out.pose.orientation.w=math.cos(yaw/2)
            out.width=float(estimate['width']);out.height=float(estimate['height'])
            out.confidence=float(msg.confidence);out.plane_residual=float(estimate['plane_residual'])
            self.pub.publish(out)
            tf=TransformStamped();tf.header=out.header;tf.child_frame_id='aperture_frame'
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,center)
            tf.transform.rotation=out.pose.orientation;self.tf.sendTransform(tf);self.status('valid frame localized')
        except (ValueError,TransformException) as error:self.status(str(error),1)

class Navigator(Base):
    def __init__(self):
        super().__init__('mission_manager')
        self.model=Simulation(self.c);self.mission=self.model.mission;self.pid=self.model.pid
        self.odom=None;self.odom_received=-1.;self.depth_stamp=-1.;self.last_detection=-1.;self.last_time=None
        self.depth_nearest=None;self.bridge=CvBridge();self.acquired=False;self.saved=False;self.pending_reset=False
        self.declare_parameter('ground_truth_aperture',False)
        self.declare_parameter('results_directory','experiments/results')
        self.command=self.create_publisher(Twist,'/drone/cmd_vel',10)
        self.telemetry=self.create_publisher(String,'/sim/telemetry',10)
        self.state=self.create_publisher(String,'/mission/state',10)
        self.path=self.create_publisher(RosPath,'/planner/trajectory',10)
        self.marker=self.create_publisher(Marker,'/aperture/marker',10)
        self.safe_pub=self.create_publisher(Bool,'/aperture/safe_to_enter',10)
        self.clearance_pub=self.create_publisher(Float32,'/aperture/minimum_clearance',10)
        self.tf=TransformBroadcaster(self)
        self.world=self.create_client(ControlWorld,'/world/aperture/control')
        self.create_subscription(Odometry,'/drone/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Image,'/drone/depth/image_raw',self.on_depth,qos_profile_sensor_data)
        self.create_subscription(AperturePose,'/aperture/pose',self.on_aperture,10)
        self.create_subscription(String,'/ui/command',self.on_command,10)
        self.create_service(Trigger,'/start_mission',lambda q,r:self.service('start',r))
        self.create_service(Trigger,'/abort_mission',lambda q,r:self.service('abort',r))
        self.create_service(Trigger,'/reset_simulation',lambda q,r:self.service('reset',r))
        self.create_timer(self.c['dt'],self.tick)
    def service(self,action,response):
        try:self.action(action);response.success=True;response.message='Request accepted'
        except ValueError as error:response.success=False;response.message=str(error)
        return response
    def on_odom(self,msg):
        self.odom=msg;self.odom_received=self.now()
        t=TransformStamped();t.header.stamp=msg.header.stamp;t.header.frame_id='odom';t.child_frame_id='base_link'
        p=msg.pose.pose.position;t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.translation.z=p.z
        t.transform.rotation=msg.pose.pose.orientation;self.tf.sendTransform(t)
    def on_depth(self,msg):
        self.depth_stamp=seconds(msg.header.stamp)
        try:
            d=self.bridge.imgmsg_to_cv2(msg,'passthrough').astype(float)
            if msg.encoding=='16UC1':d*=.001
            h,w=d.shape;roi=d[h//2-8:h//2+8,w//2-8:w//2+8]
            valid=roi[np.isfinite(roi)&(roi>0)]
            self.depth_nearest=float(np.min(valid)) if len(valid)>8 else None
        except (ValueError,RuntimeError) as error:self.depth_nearest=None;self.status(str(error),1)
    def on_aperture(self,msg):
        if self.mission.state in ('ENTER','PASS_THROUGH','CLEAR_APERTURE','HOVER','COMPLETE'):return
        p=msg.pose.position
        a=Aperture(np.array([p.x,p.y,p.z]),msg.width,msg.height,yaw_of(msg.pose.orientation),self.c['aperture']['thickness'])
        if self.acquired and np.linalg.norm(a.center-self.model.aperture.center)>.5:return
        self.model.aperture=a;self.mission.aperture=a;self.acquired=True;self.last_detection=seconds(msg.header.stamp)
    def on_command(self,msg):
        try:self.action(msg.data)
        except ValueError:return  # action() already publishes diagnostic and warning
    def action(self,action):
        try:
            if action=='start':
                if self.odom is None:raise ValueError('No odometry received')
                self.mission.start(self.now());self.saved=False
            elif action in ('abort','estop','hover'):
                self.command.publish(Twist());self.pid.reset()
                if self.mission.state not in ('COMPLETE','ABORT','EMERGENCY_HOVER'):
                    self.mission.transition('ABORT' if action=='abort' else 'EMERGENCY_HOVER',f'operator {action}',self.now())
            elif action in ('pause','resume','reset'):
                self.command.publish(Twist())
                if not self.world.service_is_ready():raise ValueError('Gazebo world control service unavailable')
                request=ControlWorld.Request();request.world_control.pause=action=='pause'
                if action=='reset':request.world_control.reset.all=True;self.pending_reset=True
                future=self.world.call_async(request)
                def finished(f):
                    response=f.result()
                    if response is None or not response.success:
                        self.pending_reset=False;self.status('Gazebo world control failed',2);return
                    if action=='reset':
                        self.model.reset();self.mission=self.model.mission;self.pid=self.model.pid
                        self.acquired=False;self.saved=False;self.last_time=None;self.odom=None
                        self.last_detection=-1.;self.depth_stamp=-1.;self.pending_reset=False
                future.add_done_callback(finished)
            else:raise ValueError('Unknown command')
        except ValueError as error:
            self.get_logger().warning(str(error));self.status(str(error),1)
            raise
    def tick(self):
        now=self.now()
        if self.odom is None or self.pending_reset:self.command.publish(Twist());return
        dt=self.c['dt'] if self.last_time is None else now-self.last_time;self.last_time=now
        if dt<=0 or dt>.2:self.command.publish(Twist());self.pid.reset();return
        p=self.odom.pose.pose.position;pos=np.array([p.x,p.y,p.z]);yaw=yaw_of(self.odom.pose.pose.orientation)
        ground_truth=self.get_parameter('ground_truth_aperture').value
        crossing=self.mission.state in ('ENTER','PASS_THROUGH','CLEAR_APERTURE','HOVER','COMPLETE')
        detected=ground_truth or (self.acquired and (crossing or now-self.last_detection<self.c['safety']['target_lost_timeout']))
        freshness=min(self.odom_received,self.depth_stamp)
        obstacle=self.depth_nearest is not None and self.depth_nearest<.25
        collision=swept_collision(self.model.aperture,self.c['drone'],self.model.position,pos,yaw)
        target,heading,speed=self.mission.update(pos,yaw,now,freshness,detected,collision or obstacle)
        if self.depth_nearest is None and self.mission.state not in ('IDLE','COMPLETE','ABORT','EMERGENCY_HOVER'):
            self.mission.transition('EMERGENCY_HOVER','invalid forward depth',now);speed=0.
        cmd=Twist()
        if speed:
            velocity=self.pid.step(target-pos,dt,speed)
            # Gazebo VelocityControl expects linear velocity in model coordinates.
            cmd.linear.x=float(math.cos(yaw)*velocity[0]+math.sin(yaw)*velocity[1])
            cmd.linear.y=float(-math.sin(yaw)*velocity[0]+math.cos(yaw)*velocity[1]);cmd.linear.z=float(velocity[2])
            cmd.angular.z=float(np.clip(wrap_angle(heading-yaw),-self.c['controller']['max_yaw_rate'],self.c['controller']['max_yaw_rate']))
        else:self.pid.reset()
        self.command.publish(cmd)
        self.model.path_length+=float(np.linalg.norm(pos-self.model.position))
        self.model.position=pos;self.model.yaw=yaw;self.model.time=now
        v=self.odom.twist.twist.linear;self.model.velocity=np.array([v.x,v.y,v.z])
        if collision:self.model.collisions+=1
        row=self.model.snapshot();a=self.model.aperture
        row['aperture']={'center':a.center.tolist(),'width':a.width,'height':a.height,'yaw':a.yaw,'thickness':a.thickness}
        row.update(source='Gazebo ground-truth aperture' if ground_truth else 'Gazebo RGB-D perception',events=self.mission.events[-16:])
        if self.mission.state not in ('IDLE','COMPLETE','ABORT','EMERGENCY_HOVER'):self.model.history.append(row)
        self.telemetry.publish(String(data=json.dumps(row)));self.state.publish(String(data=self.mission.state))
        self.safe_pub.publish(Bool(data=bool(row['safe'])));self.clearance_pub.publish(Float32(data=float(row['clearance'])))
        self.publish_geometry(a)
        if self.mission.state in ('COMPLETE','ABORT','EMERGENCY_HOVER') and not self.saved and self.model.history:
            out=self.model.save(self.get_parameter('results_directory').value)
            summary=self.model.summary();summary['source']=row['source'];summary['mission_duration']=now-(self.mission.started or now)
            (out/'summary.json').write_text(json.dumps(summary,indent=2));self.saved=True
            self.get_logger().info(f'Results written to {out}')
        self.status(self.mission.state)
    def publish_geometry(self,a):
        msg=RosPath();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
        for point in waypoints(a,self.c['mission']['approach_distance'],self.c['mission']['exit_distance']):
            pose=PoseStamped();pose.header=msg.header;pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,point);pose.pose.orientation.w=1.;msg.poses.append(pose)
        self.path.publish(msg)
        marker=Marker();marker.header=msg.header;marker.ns='aperture';marker.id=0;marker.type=Marker.LINE_STRIP;marker.action=Marker.ADD
        marker.pose.orientation.w=1.;marker.scale.x=.015;marker.color.g=1.;marker.color.a=1.
        for y,z in [(-1,-1),(1,-1),(1,1),(-1,1),(-1,-1)]:
            p=a.center+y*a.width/2*a.lateral+np.array([0,0,z*a.height/2]);marker.points.append(Point(x=float(p[0]),y=float(p[1]),z=float(p[2])))
        self.marker.publish(marker)

def run(cls):
    rclpy.init();node=cls()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if isinstance(node,Navigator):node.command.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
def detector_main():run(Detector)
def localizer_main():run(Localizer)
def mission_main():run(Navigator)

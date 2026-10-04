"""ROS-to-web transport, confined to a dedicated executor thread."""
import json
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class RosAdapter:
    def __init__(self):
        rclpy.init()
        self.node=Node('ui_bridge')
        self.lock=threading.Lock();self.data=None;self.received=0.;self.jpeg=None
        self.pub=self.node.create_publisher(String,'/ui/command',10)
        self.node.create_subscription(String,'/sim/telemetry',self.on_telemetry,10)
        self.node.create_subscription(CompressedImage,'/drone/camera/compressed',self.on_image,qos_profile_sensor_data)
        self.thread=threading.Thread(target=rclpy.spin,args=(self.node,),daemon=True);self.thread.start()
    def on_telemetry(self,msg):
        data=json.loads(msg.data)
        with self.lock:self.data=data;self.received=time.monotonic()
    def on_image(self,msg):self.jpeg=bytes(msg.data)
    def snapshot(self):
        with self.lock:
            if self.data is None:return {'source':'ROS 2','state':'WAITING_FOR_ROS','connected':False}
            return dict(self.data,connected=time.monotonic()-self.received<1.0,source='ROS 2 / Gazebo')
    def command(self,action):self.pub.publish(String(data=action))
    def close(self):
        rclpy.shutdown();self.thread.join(timeout=2);self.node.destroy_node()

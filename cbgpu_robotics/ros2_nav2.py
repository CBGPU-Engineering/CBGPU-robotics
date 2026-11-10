"""Optional ROS 2 adapter. ROS packages are installed by the robot project, not pip.
Read-only until locally commissioned, armed, healthy and started with --allow-navigation.
"""
import math,os,threading,time,uuid
from .controller import Safety
from .remote import pack_grid
from .remote_protocol import free_goal
class Ros2Nav2:
 robot_id=os.environ.get('CBGPU_ROBOT_ID','ros2-lidar-robot')
 commissioned=False
 def __init__(self):
  import rclpy
  from rclpy.node import Node
  from rclpy.executors import MultiThreadedExecutor
  from rclpy.action import ActionClient
  from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
  from nav_msgs.msg import OccupancyGrid
  from sensor_msgs.msg import LaserScan,CompressedImage
  from std_msgs.msg import Bool
  from nav2_msgs.action import NavigateToPose
  from tf2_ros import Buffer,TransformListener
  self.rclpy=rclpy;self.owns_context=not rclpy.ok()
  if self.owns_context:rclpy.init()
  self.node=Node('cbgpu_remote_'+str(os.getpid()));self.executor=MultiThreadedExecutor(num_threads=2);self.executor.add_node(self.node);self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.lock=threading.RLock();self.map=None;self.version=0;self.mapping_session=str(uuid.uuid4());self.scan_at=0;self.scan_stamp=0;self.armed=False;self.estop=True;self.arm_at=0;self.estop_at=0;self.image=None;self.image_at=None;self.handle=None;self.goal_event=None;self.last_error='';self.base_frame=os.environ.get('CBGPU_BASE_FRAME','base_link');self.commissioned=os.environ.get('CBGPU_ROBOT_COMMISSIONED')=='1'
  map_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
  self.node.create_subscription(OccupancyGrid,os.environ.get('CBGPU_MAP_TOPIC','/map'),self.on_map,map_qos);self.node.create_subscription(LaserScan,os.environ.get('CBGPU_SCAN_TOPIC','/scan'),self.on_scan,qos_profile_sensor_data);self.node.create_subscription(Bool,os.environ.get('CBGPU_ARM_TOPIC','/cbgpu/armed'),self.on_arm,10);self.node.create_subscription(Bool,os.environ.get('CBGPU_ESTOP_TOPIC','/cbgpu/emergency_stop'),self.on_estop,10)
  image_topic=os.environ.get('CBGPU_IMAGE_TOPIC')
  if image_topic:self.node.create_subscription(CompressedImage,image_topic,self.on_image,qos_profile_sensor_data)
  self.tf=Buffer();self.listener=TransformListener(self.tf,self.node);self.navigation=ActionClient(self.node,NavigateToPose,os.environ.get('CBGPU_NAV_ACTION','/navigate_to_pose'));self.thread.start()
 def stamp_age(self,stamp):
  age=(self.node.get_clock().now().nanoseconds-(stamp.sec*1000000000+stamp.nanosec))/1e9;return age if age>=-.2 else 999
 def on_map(self,msg):
  try:
   q=msg.info.origin.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));self.version+=1;value=pack_grid(msg.info.width,msg.info.height,msg.info.resolution,{'x':msg.info.origin.position.x,'y':msg.info.origin.position.y,'yaw':yaw},msg.data,msg.header.frame_id,self.mapping_session,self.version)
   with self.lock:self.map=value;self.last_error=''
  except Exception as e:self.last_error='Map rejected: '+str(e)
 def on_scan(self,msg):
  valid=sum(1 for v in msg.ranges if math.isfinite(v) and msg.range_min<=v<=msg.range_max)
  if valid>=5 and math.isfinite(msg.angle_increment) and msg.angle_increment!=0:self.scan_at=time.monotonic();self.scan_stamp=time.time()-self.stamp_age(msg.header.stamp)
  else:self.scan_at=0;self.last_error='LiDAR scan contains insufficient valid ranges'
 def on_arm(self,msg):self.armed=msg.data;self.arm_at=time.monotonic()
 def on_estop(self,msg):self.estop=msg.data;self.estop_at=time.monotonic()
 def on_image(self,msg):
  data=bytes(msg.data)
  if len(data)<=256000 and data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9'):self.image=data;self.image_at=time.time()-self.stamp_age(msg.header.stamp)
 def current_pose(self):
  if not self.map:return None,999
  try:
   t=self.tf.lookup_transform(self.map['frame'],self.base_frame,self.rclpy.time.Time());q=t.transform.rotation;return {'x':t.transform.translation.x,'y':t.transform.translation.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))},self.stamp_age(t.header.stamp)
  except Exception:return None,999
 def safety(self):
  p,tf_age=self.current_pose();age=max(time.monotonic()-self.scan_at,time.time()-self.scan_stamp,tf_age,time.monotonic()-self.arm_at,time.monotonic()-self.estop_at);healthy=bool(self.map and p and age<=1 and self.navigation.server_is_ready());return Safety(self.armed,self.estop,time.time()-age,healthy)
 def remote_snapshot(self):
  p,age=self.current_pose()
  with self.lock:m=self.map
  return {'map':m,'pose':p,'ready':bool(m and p and age<=1 and self.navigation.server_is_ready()),'mode':'hardware','message':self.last_error or 'ROS2 map / TF / LiDAR / Nav2; local arming required','image_jpeg':self.image,'image_at':self.image_at}
 def validate_remote_goal(self,goal):
  try:
   m=self.remote_snapshot()['map'];free_goal(m,goal['x'],goal['y']);return bool(goal['frame']==m['frame'] and goal['map_session']==m['session'] and self.safety().healthy)
  except Exception:return False
 def navigate_to(self,goal,stop_event):
  from nav2_msgs.action import NavigateToPose
  from action_msgs.msg import GoalStatus
  command=NavigateToPose.Goal();command.pose.header.frame_id=goal['frame'];command.pose.header.stamp=self.node.get_clock().now().to_msg();command.pose.pose.position.x=float(goal['x']);command.pose.pose.position.y=float(goal['y']);command.pose.pose.orientation.z=math.sin(goal['yaw']/2);command.pose.pose.orientation.w=math.cos(goal['yaw']/2);done=threading.Event();success=[];self.goal_event=done
  def result(future):
   try:success.append(future.result().status==GoalStatus.STATUS_SUCCEEDED)
   except Exception:success.append(False)
   done.set()
  def accepted(future):
   try:
    self.handle=future.result()
    if not self.handle.accepted:success.append(False);done.set();return
    if stop_event.is_set():self.handle.cancel_goal_async()
    self.handle.get_result_async().add_done_callback(result)
   except Exception:success.append(False);done.set()
  self.navigation.send_goal_async(command).add_done_callback(accepted)
  while not done.wait(.1):
   if stop_event.is_set():self.stop();return False
  self.handle=None;return bool(success and success[0])
 def stop(self):
  handle=self.handle
  if handle:
   try:handle.cancel_goal_async()
   except Exception:pass
 def close(self):
  self.stop();self.executor.shutdown(timeout_sec=2);self.node.destroy_node()
  if self.owns_context:self.rclpy.shutdown()
def create():return Ros2Nav2()

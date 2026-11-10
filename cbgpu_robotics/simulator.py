"""A virtual robot with navigation, gripper and speech capabilities. Never accesses hardware."""
import time
from .controller import Safety
from .protocol import validate
CAPABILITIES=[
 {'name':'navigate_to','description':'Navigate to a known named waypoint using local obstacle avoidance.','parameters':{'type':'object','properties':{'waypoint':{'type':'string','enum':['desk','charging_station'],'maxLength':32}},'required':['waypoint'],'additionalProperties':False},'max_duration_s':10},
 {'name':'set_gripper','description':'Open or close the local gripper.','parameters':{'type':'object','properties':{'closed':{'type':'boolean'}},'required':['closed'],'additionalProperties':False},'max_duration_s':2},
 {'name':'say','description':'Speak a short sentence.','parameters':{'type':'object','properties':{'text':{'type':'string','maxLength':100}},'required':['text'],'additionalProperties':False},'max_duration_s':5},
]
class VirtualRobot:
 robot_id='virtual-robot';capabilities=CAPABILITIES;commissioned=True
 def __init__(self):self.events=[];self.estop=False;self.armed=True;self.stopped=False
 def observe(self):return {'location':'desk','gripper_closed':False,'known_waypoints':['desk','charging_station'],'mode':'simulation'}
 def safety(self):return Safety(armed=self.armed,emergency_stop=self.estop,sensors_at=time.time(),healthy=True)
 def validate_step(self,step):
  schema=next(c['parameters'] for c in self.capabilities if c['name']==step['capability']);validate(step['arguments'],schema);return True
 def execute(self,name,arguments,stop_event):
  if stop_event.wait(.2):return
  self.events.append({'capability':name,'arguments':arguments})
 def stop(self):self.stopped=True

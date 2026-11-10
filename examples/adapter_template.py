"""Copy into your robot project and implement using existing, verified drivers."""
from cbgpu_robotics import Safety
from cbgpu_robotics.simulator import CAPABILITIES
class MyRobot:
 robot_id='my-robot'
 # Replace this example with only the operations your physical robot actually implements.
 capabilities=CAPABILITIES[:1]
 commissioned=False
 def observe(self):
  raise NotImplementedError('Read real local observations; never invent position or objects')
 def safety(self):
  # Replace with the actual local arm, emergency-stop, health and sensor capture state.
  return Safety()
 def validate_step(self,step):
  return False  # Implement real local preconditions before accepting a step.
 def execute(self,capability,arguments,stop_event):
  raise NotImplementedError('Dispatch known local operations, with cooperative cancellation and bounded driver timeouts')
 def stop(self):
  raise NotImplementedError('Implement immediate hardware stop and independent watchdog first')
def create():return MyRobot()

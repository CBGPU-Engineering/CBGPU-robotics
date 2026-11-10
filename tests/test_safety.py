import copy,json,tempfile,threading,time,unittest,uuid
from pathlib import Path
from cbgpu_robotics import Controller,Ledger,Safety,Invalid
from cbgpu_robotics.protocol import digest,envelope,capabilities,proposal
from cbgpu_robotics.simulator import VirtualRobot
class Tests(unittest.TestCase):
 def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.ledger=Ledger(Path(self.tmp.name)/'ledger.sqlite');self.robot=VirtualRobot();self.controller=Controller(self.robot,self.ledger)
 def tearDown(self):self.ledger.close();self.tmp.cleanup()
 def mission(self):
  now=time.time();return {'protocol_version':1,'id':str(uuid.uuid4()),'robot_id':self.robot.robot_id,'capabilities_digest':digest(self.robot.capabilities),'observed_at':now,'issued_at':now,'expires_at':now+60,'requires_local_approval':True,'plan':{'summary':'Move','needs_clarification':False,'question':'','steps':[{'capability':'navigate_to','arguments':{'waypoint':'desk'},'duration_s':2}]}}
 def test_default_never_calls_hardware_and_approval_is_required(self):
  m=self.mission();self.assertEqual(self.controller.run(m)['mode'],'dry-run');self.assertEqual(self.robot.events,[])
  with self.assertRaises(Invalid):self.controller.run(m,execute=True)
  self.assertEqual(self.controller.run(m,execute=True,approve=True)['mode'],'executed');self.assertTrue(self.robot.stopped)
 def test_replay_is_blocked_across_controller_instances(self):
  m=self.mission();self.controller.run(m,True,True)
  with self.assertRaises(Invalid):Controller(self.robot,self.ledger).run(m,True,True)
 def test_robot_binding_expiry_bounds_unknown_calls_and_extra_arguments(self):
  cases=[lambda m:m.update(robot_id='another'),lambda m:m.update(expires_at=0),lambda m:m.update(capabilities_digest='bad'),lambda m:m['plan']['steps'][0].update(capability='shell'),lambda m:m['plan']['steps'][0]['arguments'].update(command='unexpected'),lambda m:m['plan']['steps'][0].update(duration_s=100),lambda m:m['plan']['steps'][0]['arguments'].update(waypoint='hallucinated')]
  for mutate in cases:
   m=self.mission();mutate(m)
   with self.assertRaises(Invalid):self.controller.run(m,True,True)
  self.assertEqual(self.robot.events,[])
 def test_stale_sensors_estop_and_uncommissioned_adapter_fail(self):
  for safety in [Safety(True,False,time.time()-10,True),Safety(True,True,time.time(),True),Safety(False,False,time.time(),True),Safety(True,False,time.time(),False)]:
   self.robot.safety=lambda:safety
   with self.assertRaises(Invalid):self.controller.run(self.mission(),True,True)
  self.robot=VirtualRobot();self.robot.commissioned=False
  with self.assertRaises(Invalid):Controller(self.robot,self.ledger).run(self.mission(),True,True)
 def test_emergency_stop_during_work_stops_driver(self):
  def work(name,args,stop):self.robot.estop=True;stop.wait(1)
  self.robot.execute=work
  with self.assertRaises(Invalid):self.controller.run(self.mission(),True,True)
  self.assertTrue(self.robot.stopped);self.assertEqual(self.robot.events,[])
 def test_timeout_stops_and_noncooperative_driver_keeps_lock(self):
  finished=threading.Event();self.robot.execute=lambda name,args,stop:finished.wait(2);m=self.mission();m['plan']['steps'][0]['duration_s']=.05
  try:
   with self.assertRaises(Invalid):self.controller.run(m,True,True)
   self.assertTrue(self.robot.stopped)
   with self.assertRaises(Invalid):self.ledger.reserve(self.robot.robot_id,str(uuid.uuid4()))
  finally:finished.set();time.sleep(.05)
 def test_strict_schema_rejects_unbounded_or_nonfinite_values(self):
  for value in [float('nan'),float('inf')]:
   caps=copy.deepcopy(self.robot.capabilities);caps[0]['max_duration_s']=value
   with self.assertRaises(Invalid):capabilities(caps)
  caps=copy.deepcopy(self.robot.capabilities);caps[0]['parameters']['additionalProperties']=True
  with self.assertRaises(Invalid):capabilities(caps)
 def test_clarification_never_executes(self):
  m=self.mission();m['plan'].update(needs_clarification=True,question='Where?',steps=[])
  with self.assertRaises(Invalid):self.controller.run(m,True,True)
  self.assertEqual(self.robot.events,[])
if __name__=='__main__':unittest.main()

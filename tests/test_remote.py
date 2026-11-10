import tempfile,time,unittest,uuid,threading
from pathlib import Path
from cbgpu_robotics import Ledger,Invalid
from cbgpu_robotics.remote import MapSimulator,RemoteAgent,pack_grid
from cbgpu_robotics.remote_protocol import cell,free_goal,grid
class RemoteTests(unittest.TestCase):
 def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.ledger=Ledger(Path(self.tmp.name)/'ledger.sqlite');self.robot=MapSimulator();self.agent=RemoteAgent(self.robot,{'device_token':'cbgpu_robot_test'},True,self.ledger)
 def tearDown(self):self.agent.stop();self.ledger.close();self.tmp.cleanup()
 def command(self):
  m=self.robot.remote_snapshot()['map'];return {'id':str(uuid.uuid4()),'expires_at':time.time()+15,'payload':{'x':2.2,'y':2.1,'yaw':0,'frame':m['frame'],'map_session':m['session'],'map_version':m['version']}}
 def activate(self):self.agent.operator_live=True;self.agent.last_sync=time.monotonic();self.agent.motion_stop.clear()
 def test_default_remote_navigation_is_disabled(self):
  a=RemoteAgent(self.robot,{'device_token':'cbgpu_robot_test'},False,self.ledger);a.operator_live=True;a.last_sync=time.monotonic()
  with self.assertRaises(Invalid):a.check()
 def test_grid_coordinate_rotation_and_conservative_downsampling(self):
  m=pack_grid(4,4,.5,{'x':1,'y':2,'yaw':1.5707963267948966},[0]*8+[-1]*4+[100]*4,'map','session',1);self.assertEqual(cell(m,.75,2.25),0)
  with self.assertRaises(ValueError):free_goal(m,-.75,2.25)
  data=[0]*1024;data[0]=100;m=pack_grid(1024,1,.05,{'x':0,'y':0,'yaw':0},data,'map','s',1);self.assertEqual(m['width'],512);self.assertEqual(cell(m,.01,.01),100)
 def test_live_goal_completes_once_and_replay_does_not_move_again(self):
  self.activate();command=self.command();self.agent.accept(command);deadline=time.monotonic()+5
  while self.agent.thread.is_alive() and time.monotonic()<deadline:self.agent.last_sync=time.monotonic();time.sleep(.05)
  self.assertEqual(self.agent.ack['state'],'succeeded');position=dict(self.robot.position);self.agent.accept(command);self.assertEqual(self.robot.position,position);self.assertEqual(self.agent.ack['state'],'succeeded')
 def test_connection_loss_stops_local_navigation(self):
  self.activate();command=self.command();self.agent.accept(command);self.agent.last_sync=time.monotonic()-5;self.agent.thread.join(3);self.assertIn(self.agent.ack['state'],['failed','cancelled']);self.assertNotAlmostEqual(self.robot.position['x'],command['payload']['x'])
 def test_unknown_goal_is_rejected_locally(self):
  self.activate();command=self.command();command['payload']['x']=7;command['payload']['y']=7;self.agent.accept(command);self.agent.thread.join(2);self.assertEqual(self.agent.ack['state'],'failed')
 def test_sync_sends_only_changed_maps_and_valid_scoped_credentials(self):
  calls=[]
  class HTTP:
   def request(self,path,method,body):calls.append((path,body));return {'server_time':time.time(),'operator_live':False,'stop_epoch':0,'command':None}
  self.agent.client=HTTP();self.agent.sync_once();self.agent.sync_once();self.assertIn('map',calls[0][1]);self.assertNotIn('map',calls[1][1]);self.assertNotIn('device_token',calls[0][1]);self.assertEqual(calls[0][0],'/robotics/agent/sync');self.assertEqual(calls[0][1]['telemetry']['mode'],'simulation')

 def test_readonly_map_upload_does_not_stop_unrelated_manual_mapping(self):
  stopped=[];self.robot.stop=lambda:stopped.append(True)
  class HTTP:
   epoch=0
   def request(self,path,method,body):return {'server_time':time.time(),'operator_live':False,'stop_epoch':self.epoch,'command':None}
  self.agent.client=HTTP();self.agent.sync_once();self.agent.sync_once();self.assertEqual(stopped,[]);self.agent.client.epoch=1;self.agent.sync_once();self.assertEqual(stopped,[True])

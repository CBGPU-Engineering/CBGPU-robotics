import unittest,threading,json,time,uuid
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from cbgpu_robotics import Client,Invalid
from cbgpu_robotics.protocol import digest
from cbgpu_robotics.simulator import VirtualRobot
class ClientTests(unittest.TestCase):
 def test_https_by_default_and_explicit_loopback_for_tests(self):
  for url in ['http://example.com','https://user:password@example.com','https://example.com/?token=x']:
   with self.assertRaises(Invalid):Client('test-key',url)
 def test_authenticated_async_transport_roundtrip(self):
  robot=VirtualRobot();now=time.time();mission={'protocol_version':1,'id':str(uuid.uuid4()),'robot_id':robot.robot_id,'capabilities_digest':digest(robot.capabilities),'issued_at':now,'observed_at':now,'expires_at':now+60,'requires_local_approval':True,'plan':{'summary':'Move','needs_clarification':False,'question':'','steps':[{'capability':'navigate_to','arguments':{'waypoint':'desk'},'duration_s':1}]}};calls=[]
  class Handler(BaseHTTPRequestHandler):
   def log_message(self,*a):pass
   def send(self,value):
    raw=json.dumps(value).encode();self.send_response(200);self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
   def do_POST(self):calls.append((self.path,self.headers.get('Authorization'),json.loads(self.rfile.read(int(self.headers['Content-Length'])))));self.send({'id':mission['id'],'state':'queued'})
   def do_GET(self):self.send({'id':mission['id'],'state':'succeeded','mission':mission})
  server=ThreadingHTTPServer(('127.0.0.1',0),Handler);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
  try:
   client=Client('test-key','http://127.0.0.1:'+str(server.server_port));result=client.plan('Move',robot.robot_id,robot.capabilities,robot.observe());self.assertEqual(result,mission);self.assertEqual(calls[0][1],'Bearer test-key');self.assertEqual(calls[0][2]['robot_id'],robot.robot_id)
  finally:server.shutdown();server.server_close();thread.join()

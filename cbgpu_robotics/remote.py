"""Outbound-only remote map agent. Navigation remains local and explicitly enabled."""
import base64,copy,getpass,json,math,os,threading,time,uuid
from pathlib import Path
from .client import Client
from .controller import Ledger,Safety
from .protocol import require,Invalid
from .remote_protocol import grid,free_goal,pose
DEFAULT='https://cbgpu.com/api/ai/v1'
def credentials_dir():return Path.home()/'.cbgpu/robotics/devices'
def connect(base_url=DEFAULT):
 code=getpass.getpass('Paste the one-time device pairing code (hidden): ').strip();client=Client('pairing',base_url);result=client.request('/robotics/agent/enroll','POST',{'pair_code':code,'agent_session':str(uuid.uuid4())});folder=credentials_dir();folder.mkdir(parents=True,exist_ok=True);file=folder/(result['device_id']+'.json');file.write_text(json.dumps({**result,'base_url':base_url}));os.chmod(file,0o600);print('Device connected:',result['name']);print('Saved private credential:',file);print('Run: cbgpu-robotics agent --device '+result['device_id']+' --simulator --allow-navigation');return result

def pack_grid(width,height,resolution,origin,data,frame,session,version):
 require(type(width) is int and type(height) is int and 0<width*height<=16000000 and len(data)==width*height,'Invalid source occupancy map');scale=max(1,math.ceil(max(width,height)/512));w=math.ceil(width/scale);h=math.ceil(height/scale);cells=[]
 for row in range(h):
  for col in range(w):
   block=[int(data[y*width+x]) for y in range(row*scale,min((row+1)*scale,height)) for x in range(col*scale,min((col+1)*scale,width))];require(all(-1<=v<=100 for v in block),'Invalid occupancy value');cells.append(max(block) if any(v>0 for v in block) else -1 if -1 in block else 0)
 rle=[]
 for v in cells:
  if rle and rle[-2]==v:rle[-1]+=1
  else:rle.extend([v,1])
 return grid({'width':w,'height':h,'resolution':resolution*scale,'origin':origin,'rle':rle,'frame':frame,'session':session,'version':version})

class RemoteAgent:
 def __init__(self,adapter,config,allow_navigation=False,ledger=None):
  require(all(callable(getattr(adapter,k,None)) for k in ['remote_snapshot','safety','validate_remote_goal','navigate_to','stop']),'Adapter needs remote_snapshot, safety, validate_remote_goal, navigate_to and stop');self.adapter=adapter;self.config=config;self.client=Client(config['device_token'],config.get('base_url',DEFAULT),timeout=2);self.allow_navigation=allow_navigation;self.ledger=ledger or Ledger();self.session=str(uuid.uuid4());self.quit=threading.Event();self.motion_stop=threading.Event();self.thread=None;self.current=None;self.ack=None;self.last_sync=0.;self.operator_live=False;self.epoch=None;self.sent_map=None;self.last_image=0.;self.guard=threading.RLock();self.failed=False
 def check(self):
  s=self.adapter.safety();require(isinstance(s,Safety) and s.armed is True and s.healthy is True and s.emergency_stop is False,'Robot is not armed/healthy or emergency stop is active');require(type(s.sensors_at) in [int,float] and math.isfinite(s.sensors_at) and -.2<=time.time()-s.sensors_at<=1,'Sensor state is stale');require(self.allow_navigation and getattr(self.adapter,'commissioned',False) is True,'Remote navigation has not been commissioned/enabled');require(self.operator_live and time.monotonic()-self.last_sync<1.5,'Remote operator/connection is stale');require(not self.motion_stop.is_set() and not self.quit.is_set(),'Navigation cancelled')
 def stop(self,force=False):
  self.motion_stop.set()
  if not force and self.current is None and not self.failed:return
  try:self.adapter.stop()
  except Exception:pass
 def run_goal(self,command):
  reserved=False;state='failed';error='';goal=command['payload'];driver=None;outcome=[]
  try:
   self.check();snapshot=self.adapter.remote_snapshot();m=grid(snapshot['map']);require(goal['map_session']==m['session'] and goal['frame']==m['frame'],'Mapping session changed');free_goal(m,goal['x'],goal['y']);require(time.time()<command['expires_at'],'Goal delivery expired');require(self.adapter.validate_remote_goal(goal) is True,'Local navigation rejected this goal');self.ledger.reserve(self.adapter.robot_id,command['id']);reserved=True
   def work():
    try:
     value=self.adapter.navigate_to(goal,self.motion_stop);require(value is True,'Navigation did not reach the goal');outcome.append(None)
    except BaseException as e:outcome.append(e)
   driver=threading.Thread(target=work,daemon=True);driver.start();self.ack={'id':command['id'],'state':'running'};deadline=time.monotonic()+180
   while driver.is_alive():
    driver.join(.1);self.check();require(time.monotonic()<deadline,'Navigation exceeded 180 seconds');snap=self.adapter.remote_snapshot();require(snap.get('map',{}).get('session')==goal['map_session'],'Mapping session changed')
   require(outcome and outcome[0] is None,'Local navigation failed');state='succeeded'
  except BaseException as e:state='cancelled' if self.motion_stop.is_set() else 'failed';error=str(e)[:250]
  finally:
   self.stop()
   if driver and driver.is_alive():driver.join(.5)
   if driver and driver.is_alive():self.failed=True;error='Driver did not stop; local robot lock retained'
   if reserved:self.ledger.finish(self.adapter.robot_id,command['id'],state,release=not(driver and driver.is_alive()))
   with self.guard:self.ack={'id':command['id'],'state':state,'error':error};self.current=None
 def accept(self,command):
  with self.guard:
   if self.current==command['id']:return
   if self.thread and self.thread.is_alive():return
   previous=self.ledger.db.execute('SELECT state FROM missions WHERE id=?',(command['id'],)).fetchone()
   if previous:self.ack={'id':command['id'],'state':previous[0] if previous[0] in ['succeeded','failed','cancelled'] else 'failed','error':'Previous mission will not be replayed'};return
   if self.failed:self.ack={'id':command['id'],'state':'failed','error':'Local controller needs recovery'};return
   self.current=command['id'];self.motion_stop=threading.Event();self.thread=threading.Thread(target=self.run_goal,args=(command,),daemon=True);self.thread.start()
 def sync_once(self):
  snapshot=self.adapter.remote_snapshot();m=grid(snapshot['map']) if snapshot.get('map') else None;safety=self.adapter.safety();age=max(0,time.time()-safety.sensors_at);t={'armed':safety.armed,'emergency_stop':safety.emergency_stop,'ready':bool(snapshot.get('ready') and m and safety.healthy and age<=1),'navigation_enabled':bool(self.allow_navigation and getattr(self.adapter,'commissioned',False) and not self.failed),'sensor_age_s':age,'pose':pose(snapshot['pose']) if snapshot.get('pose') else None,'map_session':m['session'] if m else '', 'map_version':m['version'] if m else 0,'mode':snapshot.get('mode','hardware'),'message':str(snapshot.get('message',''))[:300],'image_at':snapshot.get('image_at')}
  body={'agent_session':self.session,'telemetry':t}
  marker=(m['session'],m['version']) if m else None
  if m and marker!=self.sent_map:body['map']=m
  image=snapshot.get('image_jpeg')
  if image and time.monotonic()-self.last_image>.5:
   if isinstance(image,bytes) and len(image)<=256000 and image.startswith(b'\xff\xd8') and image.endswith(b'\xff\xd9'):body['image']=base64.b64encode(image).decode();self.last_image=time.monotonic()
  sent_ack=self.ack
  if sent_ack:body['ack']=sent_ack
  result=self.client.request('/robotics/agent/sync','POST',body);self.last_sync=time.monotonic();self.operator_live=result['operator_live']
  if self.epoch is not None and self.epoch!=result['stop_epoch']:self.stop(force=True)
  self.epoch=result['stop_epoch'];self.sent_map=marker
  if not self.operator_live:self.stop()
  if sent_ack and self.ack==sent_ack and sent_ack['state']!='running':self.ack=None
  if result.get('command'):self.accept(result['command'])
  return result
 def run(self):
  try:
   while not self.quit.is_set():
    began=time.monotonic()
    try:self.sync_once()
    except Exception as e:
     self.operator_live=False;self.stop();print('Connection/control paused:',str(e),flush=True);self.quit.wait(1)
    self.quit.wait(max(0,.25-(time.monotonic()-began)))
  finally:
   self.quit.set();self.stop()
   if self.thread:self.thread.join(2)
   close=getattr(self.adapter,'close',None)
   if callable(close):close()

class MapSimulator:
 """Virtual occupancy mapping and click navigation. No serial/GPIO/hardware access."""
 robot_id='remote-map-simulator';commissioned=True
 def __init__(self):self.started=time.monotonic();self.session=str(uuid.uuid4());self.position={'x':2.,'y':2.,'yaw':0.};self.lock=threading.RLock()
 def safety(self):return Safety(True,False,time.time(),True)
 def remote_snapshot(self):
  elapsed=int(time.monotonic()-self.started);radius=min(29,9+elapsed//2);data=[]
  for y in range(64):
   for x in range(64):data.append(-1 if abs(x-16)>radius or abs(y-16)>radius else 100 if x in [0,63] or y in [0,63] or x==36 and 8<y<48 else 0)
  m=pack_grid(64,64,.125,{'x':0,'y':0,'yaw':0},data,'map',self.session,min(41,elapsed//2+1))
  with self.lock:p=dict(self.position)
  return {'map':m,'pose':p,'ready':True,'mode':'simulation','message':'Virtual online mapping; no physical robot attached'}
 def validate_remote_goal(self,goal):
  try:free_goal(self.remote_snapshot()['map'],goal['x'],goal['y']);return True
  except ValueError:return False
 def navigate_to(self,goal,stop_event):
  # The simulator demonstrates the transport; real navigation must use a local collision-aware planner.
  with self.lock:start=dict(self.position)
  for i in range(1,41):
   if stop_event.wait(.05):return False
   x=start['x']+(goal['x']-start['x'])*i/40;y=start['y']+(goal['y']-start['y'])*i/40
   free_goal(self.remote_snapshot()['map'],x,y)
   with self.lock:self.position={'x':x,'y':y,'yaw':goal['yaw']}
  return True
 def stop(self):pass

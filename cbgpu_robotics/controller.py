"""Local-only execution. A commissioned adapter owns drivers, watchdogs and stopping."""
from dataclasses import dataclass
from pathlib import Path
import json,math,os,sqlite3,threading,time
from .protocol import envelope,digest,require,Invalid
@dataclass(frozen=True)
class Safety:
 armed:bool=False
 emergency_stop:bool=True
 sensors_at:float=0
 healthy:bool=False
class Ledger:
 def __init__(self,path=None):
  path=Path(path or Path.home()/'.cbgpu/robotics/missions.sqlite');path.parent.mkdir(parents=True,exist_ok=True);self.db=sqlite3.connect(path,timeout=10,check_same_thread=False);self.lock=threading.RLock()
  try:os.chmod(path,0o600)
  except OSError:pass
  self.db.executescript('CREATE TABLE IF NOT EXISTS missions(id TEXT PRIMARY KEY,robot TEXT,state TEXT,at REAL); CREATE TABLE IF NOT EXISTS robot_locks(robot TEXT PRIMARY KEY,mission TEXT);');self.db.commit()
 def reserve(self,robot,id):
  with self.lock:
   try:
    self.db.execute('BEGIN IMMEDIATE');self.db.execute('INSERT INTO robot_locks VALUES(?,?)',(robot,id));self.db.execute('INSERT INTO missions VALUES(?,?,?,?)',(id,robot,'running',time.time()));self.db.commit()
   except sqlite3.IntegrityError:self.db.rollback();raise Invalid('Mission replay or robot already locked; verify the robot is stopped before local recovery')
 def finish(self,robot,id,state,release=True):
  with self.lock:
   self.db.execute('UPDATE missions SET state=? WHERE id=?',(state,id))
   if release:self.db.execute('DELETE FROM robot_locks WHERE robot=? AND mission=?',(robot,id))
   self.db.commit()
 def unlock(self,robot,confirm_stopped=False):
  require(confirm_stopped,'Confirm the physical robot is stopped before clearing its lock')
  with self.lock:self.db.execute('DELETE FROM robot_locks WHERE robot=?',(robot,));self.db.commit()
 def close(self):self.db.close()
class Controller:
 def __init__(self,adapter,ledger=None):self.adapter=adapter;self.ledger=ledger or Ledger();self.lock=threading.Lock()
 def check(self):
  state=self.adapter.safety();now=time.time();require(isinstance(state,Safety),'Adapter must return Safety');require(state.armed is True and state.emergency_stop is False and state.healthy is True,'Robot is not armed and healthy, or emergency stop is active');require(type(state.sensors_at) in [float,int] and math.isfinite(state.sensors_at) and -.2<=now-state.sensors_at<=1,'Robot sensor state is stale')
 def run(self,mission,execute=False,approve=False):
  caps=json.loads(json.dumps(self.adapter.capabilities));plan=envelope(mission,caps,self.adapter.robot_id)
  require(not plan['needs_clarification'],'Clarification required: '+plan['question'])
  if not execute:return {'mode':'dry-run','steps':plan['steps'],'summary':plan['summary']}
  require(approve is True and getattr(self.adapter,'commissioned',False) is True,'Execution needs local approval and a commissioned adapter');require(self.lock.acquire(blocking=False),'This controller is already running');reserved=False;thread=None;stop=threading.Event();state='failed'
  try:
   self.check();self.ledger.reserve(self.adapter.robot_id,mission['id']);reserved=True
   for step in plan['steps']:
    envelope(mission,caps,self.adapter.robot_id);self.check();require(digest(self.adapter.capabilities)==digest(caps),'Local capabilities changed');require(self.adapter.validate_step(step) is True,'Adapter rejected step preconditions')
    outcome=[]
    def work(current=step):
     try:self.adapter.execute(current['capability'],current['arguments'],stop);outcome.append(None)
     except BaseException as e:outcome.append(e)
    thread=threading.Thread(target=work,daemon=True);thread.start();deadline=time.monotonic()+step['duration_s']
    while thread.is_alive():
     thread.join(.1);self.check();require(time.time()<mission['expires_at'],'Mission expired');require(time.monotonic()<deadline or not thread.is_alive(),'Step deadline exceeded')
    require(outcome and outcome[0] is None,'Adapter execution failed')
   state='succeeded';return {'mode':'executed','steps':len(plan['steps'])}
  finally:
   stop.set()
   try:self.adapter.stop()
   finally:
    if thread and thread.is_alive():thread.join(.5)
    if reserved:self.ledger.finish(self.adapter.robot_id,mission['id'],state,release=not(thread and thread.is_alive()))
    self.lock.release()

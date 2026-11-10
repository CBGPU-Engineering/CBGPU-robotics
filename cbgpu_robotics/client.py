"""HTTPS-only planning client; no inbound port or remote command runner."""
import json,os,time,urllib.error,urllib.parse,urllib.request
from .protocol import capabilities,robot_id,require,bounded,envelope,Invalid
class NoRedirect(urllib.request.HTTPRedirectHandler):
 def redirect_request(self,*args,**kwargs):raise Invalid('API redirects are not allowed')
class Client:
 def __init__(self,key=None,base_url='https://cbgpu.com/api/ai/v1',timeout=20):
  self.key=key or os.environ.get('CBGPU_API_KEY','');require(bool(self.key) and '\n' not in self.key and '\r' not in self.key,'Set CBGPU_API_KEY to your private AI API key')
  parsed=urllib.parse.urlsplit(base_url);require(not parsed.username and not parsed.password and not parsed.query and not parsed.fragment and (parsed.scheme=='https' or parsed.scheme=='http' and parsed.hostname in ['localhost','127.0.0.1','::1']),'Use HTTPS (HTTP is permitted only for loopback tests)')
  self.base_url=base_url.rstrip('/');self.timeout=timeout;self.opener=urllib.request.build_opener(NoRedirect)
 def request(self,path,method='GET',data=None):
  request=urllib.request.Request(self.base_url+path,data=json.dumps(data,allow_nan=False).encode() if data is not None else None,method=method,headers={'Authorization':'Bearer '+self.key,'Content-Type':'application/json','User-Agent':'CBGPU-Robotics/0.1'})
  try:
   with self.opener.open(request,timeout=self.timeout) as response:
    raw=response.read(1024*1024+1);require(len(raw)<=1024*1024,'API response is too large');return json.loads(raw)
  except urllib.error.HTTPError as e:
   try:message=json.loads(e.read(8192)).get('message','Planning request failed')
   except Exception:message='Planning request failed'
   raise Invalid('CBGPU API '+str(e.code)+': '+str(message)) from None
 def create(self,instruction,robot,caps,observations,model='cbgpu-coder',effort='medium'):
  capabilities(caps);robot_id(robot);bounded(observations,12000,'Observations')
  return self.request('/robotics/plans','POST',{'instruction':instruction,'robot_id':robot,'capabilities':caps,'observations':observations,'observed_at':time.time(),'model':model,'effort':effort})
 def get(self,id):
  import uuid
  require(str(uuid.UUID(id))==id,'Invalid plan ID');return self.request('/robotics/plans/'+id)
 def cancel(self,id):
  import uuid
  require(str(uuid.UUID(id))==id,'Invalid plan ID');return self.request('/robotics/plans/'+id,'DELETE')
 def plan(self,instruction,robot,caps,observations,model='cbgpu-coder',effort='medium',wait=180):
  job=self.create(instruction,robot,caps,observations,model,effort);id=job['id'];deadline=time.monotonic()+wait
  try:
   while time.monotonic()<deadline:
    result=self.get(id)
    if result['state']=='succeeded':envelope(result['mission'],caps,robot);return result['mission']
    if result['state'] in ['failed','cancelled','deleted']:raise Invalid(result.get('error') or 'Planning stopped')
    time.sleep(1)
   raise Invalid('Planning timed out waiting for an unreserved GPU')
  except BaseException:
   try:self.cancel(id)
   except Exception:pass
   raise

"""Portable CLI. Simulation/dry-run is the default; hardware adapters are explicit local plugins."""
import argparse,importlib,json,sys,time,uuid
from pathlib import Path
from .client import Client
from .controller import Controller,Ledger
from .protocol import digest,Invalid
from .simulator import VirtualRobot

def main():
 parser=argparse.ArgumentParser(prog='cbgpu-robotics');sub=parser.add_subparsers(dest='command',required=True)
 demo=sub.add_parser('demo',help='Offline simulator: no API key or hardware');demo.add_argument('--execute-simulation',action='store_true')
 plan=sub.add_parser('plan',help='Ask CBGPU for a robot plan (dry-run by default)');plan.add_argument('instruction');plan.add_argument('--adapter',help='Trusted local module:factory (defaults to virtual robot)');plan.add_argument('--execute',action='store_true');plan.add_argument('--approve',action='store_true');plan.add_argument('--model',default='cbgpu-coder');plan.add_argument('--effort',choices=['low','medium','high'],default='medium')
 unlock=sub.add_parser('unlock',help='Clear a local crash lock after physically stopping the robot');unlock.add_argument('robot_id');unlock.add_argument('--confirm-stopped',action='store_true')
 args=parser.parse_args();ledger=Ledger()
 try:
  if args.command=='unlock':ledger.unlock(args.robot_id,args.confirm_stopped);print('Local lock cleared. Used mission IDs remain blocked.');return
  if args.command=='demo':
   robot=VirtualRobot();now=time.time();mission={'protocol_version':1,'id':str(uuid.uuid4()),'robot_id':robot.robot_id,'capabilities_digest':digest(robot.capabilities),'issued_at':now,'observed_at':now,'expires_at':now+60,'requires_local_approval':True,'plan':{'summary':'Offline virtual-robot demonstration','needs_clarification':False,'question':'','steps':[{'capability':'navigate_to','arguments':{'waypoint':'charging_station'},'duration_s':5}]}};execute=args.execute_simulation;approve=execute
  else:
   if args.adapter:
    module,separator,factory=args.adapter.partition(':')
    if not separator:raise Invalid('Use --adapter module:factory')
    robot=getattr(importlib.import_module(module),factory)()
   else:robot=VirtualRobot()
   mission=Client().plan(args.instruction,robot.robot_id,robot.capabilities,robot.observe(),args.model,args.effort);print(json.dumps(mission,ensure_ascii=False,indent=2));execute=args.execute;approve=args.approve
  result=Controller(robot,ledger).run(mission,execute=execute,approve=approve);print(json.dumps(result,ensure_ascii=False,indent=2))
 except (Invalid,OSError,ValueError,ImportError) as e:print('CBGPU Robotics: '+str(e),file=sys.stderr);raise SystemExit(1)
 finally:ledger.close()
if __name__=='__main__':main()

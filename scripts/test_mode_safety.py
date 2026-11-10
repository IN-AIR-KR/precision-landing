#!/usr/bin/env python3
"""SITL-only target loss and MAVLink Hold/Land takeover test; owns its simulation."""
import os,signal,subprocess,time
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[1]
os.chdir(root)
out=root/'.run'/('mode-safety-'+time.strftime('%Y%m%d-%H%M%S'));out.mkdir(exist_ok=False)
cfg=yaml.safe_load((root/'src/pl_bringup/config/sim.yaml').read_text())
cfg['control']['descent_vel']=0.1
(out/'sim.yaml').write_text(yaml.safe_dump(cfg))
env=dict(os.environ,HEADLESS='1',RUN_LOG_DIR=str(out),BRINGUP_CONFIG=str(out/'sim.yaml'))
with (out/'runner.log').open('w') as log:
 p=subprocess.Popen(['bash','scripts/run.sh'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  for _ in range(120):
   if p.poll() is not None: raise RuntimeError('SITL exited')
   r=out/'ros.log'
   if r.exists() and 'Got RegisterExtComponentReply' in r.read_text():break
   time.sleep(1)
  else:raise RuntimeError('Registration timeout')
  with (out/'test.log').open('w') as testlog:
   subprocess.run(['python3','scripts/test_flight.py','--speed','0','--exercise-handover','--output',str(out/'flight.jsonl')],env=env,stdout=testlog,stderr=subprocess.STDOUT,check=True,timeout=150)
 finally:
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=20)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
print(f'PASS: target loss, MAVLink Hold takeover and Land; logs: {out}')

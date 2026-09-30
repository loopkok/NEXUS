import json,time
import numpy as np
from scipy.spatial.transform import Rotation
from nero_quest_teleop.ik_solver import IKSolver, fk_all
from nexus_core.profile import Profile
p=Profile.load('src/nexus_core/profiles/nero_dual_xhand.json');rng=np.random.default_rng(77)
s=IKSolver();spec=p.component('left_arm');lo=np.array(spec.lower);hi=np.array(spec.upper)
cases=[]
for i in range(192):
 q=rng.uniform(lo+.001,hi-.001);T=s.fk(q);s.sync_state(q);t=time.perf_counter();out=s.solve(T)
 err=None if out is None else float(np.linalg.norm(s.fk(out)-T))
 cases.append(dict(q=q.tolist(),target=T.tolist(),failed=out is None,error=err,ms=(time.perf_counter()-t)*1000,base_radius=float(np.linalg.norm(T[:3,3]))))
print('FULL_FK_WITNESS',json.dumps(dict(count=len(cases),failed=sum(c['failed'] for c in cases),p95_ms=float(np.percentile([c['ms'] for c in cases],95)),clipped_by_base_sphere=sum(c['base_radius']>.58 for c in cases),max_fk_error=max(c['error'] or 0 for c in cases))),flush=True)
print('FULL_FAILED_WITNESSES',json.dumps([c for c in cases if c['failed']]),flush=True)
for side in ['left','right']:
 home=np.array(p.adapter_config('nero_can')['home_pose'][side]);T0=s.fk(home);samples=[]
 for i in range(80):
  T=T0.copy();T[:3,3]+=rng.uniform(-.25,.25,3);T[:3,:3]=Rotation.from_rotvec(rng.uniform(-.9,.9,3)).as_matrix()@T0[:3,:3]
  wrist=T[:3,3]-.0235*T[:3,2];dist=float(np.linalg.norm(wrist-[0,0,.138]));s.sync_state(home);t=time.perf_counter();q=s.solve(T)
  samples.append(dict(target=T.tolist(),failed=q is None,ms=(time.perf_counter()-t)*1000,wrist_distance=dist))
 print('BROAD_POSES',side,json.dumps(samples),flush=True)

import json,time
import numpy as np
from scipy.spatial.transform import Rotation
from nero_quest_teleop.ik_solver import IKSolver, fk_all
from nero_quest_teleop.safety_filter import SafetyFilter
from nexus_core.profile import Profile
p=Profile.load('src/nexus_core/profiles/nero_dual_xhand.json')
rng=np.random.default_rng(42)
for side in ['left','right']:
 s=IKSolver(); spec=p.component(side+'_arm'); home=np.array(p.adapter_config('nero_can')['home_pose'][side]); T=s.fk(home)
 print('HOME',side,'position',T[:3,3].tolist(),'distance_base',float(np.linalg.norm(T[:3,3])),'limit_margin_rad',np.minimum(home-np.array(spec.lower),np.array(spec.upper)-home).tolist(),flush=True)
 cases=[]
 for i in range(64):
  q=np.clip(home+rng.normal(0,.12,7),np.array(spec.lower)+.001,np.array(spec.upper)-.001)
  target=s.fk(q);s.sync_state(q);start=time.perf_counter();out=s.solve(target);dt=(time.perf_counter()-start)*1e3
  cases.append(dict(q=q.tolist(),target=target.tolist(),failed=out is None,ms=dt,base_radius=float(np.linalg.norm(target[:3,3]))))
 print('FK_WITNESS',side,json.dumps(dict(count=len(cases),failed=sum(x['failed'] for x in cases),p95_ms=float(np.percentile([x['ms'] for x in cases],95)),clipped_by_base_sphere=sum(x['base_radius']>.58 for x in cases))),flush=True)
 print('FAILED_WITNESSES',side,json.dumps([x for x in cases if x['failed']]),flush=True)
 sweep=[]
 for kind in ['position','rotation']:
  for axis in range(3):
   for amount in ([-.10,-.06,-.03,-.01,.01,.03,.06,.10] if kind=='position' else [-30,-20,-10,-5,5,10,20,30]):
    target=T.copy()
    if kind=='position':target[axis,3]+=amount
    else:
     rv=np.zeros(3);rv[axis]=np.deg2rad(amount);target[:3,:3]=Rotation.from_rotvec(rv).as_matrix()@T[:3,:3]
    s.sync_state(home);start=time.perf_counter();q=s.solve(target);dt=(time.perf_counter()-start)*1e3
    wrist=target[:3,3]-.0235*target[:3,2];distance=float(np.linalg.norm(wrist-np.array([0,0,.138])))
    sweep.append(dict(kind=kind,axis=axis,amount=amount,failed=q is None,ms=dt,wrist_distance=distance,target=target.tolist()))
 print('SWEEP',side,json.dumps(sweep),flush=True)

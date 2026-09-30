import os,time,json,math,subprocess,signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from nexus_core.profile import Profile
from nexus_core.nero_ik_adapter import InteractiveNeroIK
p=Profile.load('src/nexus_core/profiles/nero_dual_xhand.json')
assert {c.driver for c in p.components if c.kind=='arm'}=={'nero_can'}
logs=[];processes=[]
for side in ['left','right']:
 f=open('/tmp/nexus_ik_fix_hardware_'+side+'.log','w');logs.append(f)
 processes.append(subprocess.Popen(['ros2','run','nexus_core','nexus_nero_teleop','--ros-args','-p',f'profile_file:={os.path.abspath("src/nexus_core/profiles/nero_dual_xhand.json")}','-p',f'component:={side}_arm','-p',f'side:={side}','-r',f'__node:=offline_{side}'],stdout=f,stderr=f,start_new_session=True))
rclpy.init();n=Node('hardware_profile_virtual_feedback');qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
origin=time.monotonic();q={};candidates={s:[] for s in ['left','right']};wristpub={};statepub={};solver={};active=False
for side in ['left','right']:
 sp=p.component(side+'_arm');q[side]=np.array(p.adapter_config('nero_can')['home_pose'][side]);solver[side]=InteractiveNeroIK(sp.lower,sp.upper)
 wristpub[side]=n.create_publisher(PoseStamped,f'{p.namespace}/input/{side}/wrist_pose',qos)
 statepub[side]=n.create_publisher(JointState,p.topic(sp.name,'joint_states'),qos)
 def on_candidate(msg,side=side):
  q[side]=np.array(msg.position);t=time.monotonic()-origin;candidates[side].append((t,q[side].tolist()))
 n.create_subscription(JointState,p.candidate_topic('teleop',sp.name),on_candidate,qos)
startpub=n.create_publisher(Bool,'/teleop/start',10)
def input_tick():
 if not active:return
 t=time.monotonic()-origin
 amplitude=.8 if 11.<t<16. else .008
 for side in ['left','right']:
  m=PoseStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id=p.input_spec(side,'wrist').get('frame_id','nero_'+side+'_wrist_mapped');m.pose.orientation.w=1.
  m.pose.position.y=amplitude*math.sin(2*math.pi*.5*t)*(1 if side=='left' else -1)
  wristpub[side].publish(m)
def state_tick():
 for side in ['left','right']:
  sp=p.component(side+'_arm');m=JointState();m.header.stamp=n.get_clock().now().to_msg();m.name=list(sp.joints);m.position=q[side].tolist();statepub[side].publish(m)
n.create_timer(1./72.,input_tick);n.create_timer(1./20.,state_tick);active=True
try:
 while time.monotonic()-origin<3.:rclpy.spin_once(n,timeout_sec=.02)
 startpub.publish(Bool(data=True))
 while time.monotonic()-origin<24.:rclpy.spin_once(n,timeout_sec=.02)
 active=False
 until=time.monotonic()+1.
 while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.02)
 for side,trace in candidates.items():
  epochs={}
  for label,lo,hi in [('normal',5,10),('unreachable',12,15),('recovery',18,23)]:
   points=[(t,v) for t,v in trace if lo<=t<hi]
   epochs[label]={'candidate_hz':len(points)/(hi-lo),'motion_rad':float(np.max(np.ptp(np.array([v for t,v in points]),axis=0))) if points else None}
  print('HARDWARE_PROFILE_VIRTUAL_RESULT',side,json.dumps(epochs),flush=True)
  assert epochs['recovery']['candidate_hz']>40 and epochs['recovery']['motion_rad']>.0001
 print('NO_HARDWARE_DRIVERS_STARTED',flush=True)
finally:
 n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 for proc in processes:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=3)
 for f in logs:f.close()

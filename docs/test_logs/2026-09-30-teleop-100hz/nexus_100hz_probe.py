import sys,time,json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import JointState
from nexus_core.profile import Profile
p=Profile.load(sys.argv[1]);rclpy.init();n=Node('teleop_rate_probe');qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT);samples={};origin=time.monotonic()
for c in p.components:
 if c.kind!='arm':continue
 for kind,topic in [('candidate',p.candidate_topic('teleop',c.name)),('final',p.topic(c.name,'joint_commands'))]:
  key=c.name+'/'+kind;samples[key]=[]
  n.create_subscription(JointState,topic,lambda msg,k=key:samples[k].append(time.monotonic()-origin),qos)
try:
 while time.monotonic()-origin<12.:rclpy.spin_once(n,timeout_sec=.01)
 for key,trace in samples.items():
  trace=[t for t in trace if 2.<t<11.]
  print('RATE_RESULT',key,'hz',len(trace)/9.,flush=True)
finally:
 n.destroy_node()
 if rclpy.ok():rclpy.shutdown()

from ikpy.chain import Chain
from ikpy.URDF_utils import get_chain_from_joints
from urdfpy import URDF
import math
import sys


# DARWIN_URDF = 'darwin_orig.urdf'
DARWIN_URDF = 'darwin.urdf'
robot = URDF.load(DARWIN_URDF)
joint_names = list(filter(lambda j: j.joint_type != 'fixed', robot.joints))
default_angle = math.pi * 150 / 180
cfg = dict(zip(joint_names, [default_angle] * len(joint_names)))
robot.show(cfg=cfg)
sys.exit(0)


def print_link(l, indent=''):
    sys.stdout.write(l.name + '\n')
    for j in robot.joints:
        if j.parent == l.name:
            joint_type = '(R)' if j.joint_type == 'revolute' else '(F)'
            sys.stdout.write('{}{} {} -> '.format(indent, j.name, joint_type))
            print_link(robot.link_map[j.child], indent + ' ')


print_link(robot.base_link)

# motors = ['l_shoulder_anchor_joint', 'l_shoulder_servo_joint',
#           'l_shoulder_joint', 'l_biceps_anchor_joint',
#           'l_biceps_axis_joint', 'l_biceps_joint', 'l_biceps_spo_joint']
# chain_elements = get_chain_from_joints(DARWIN_URDF, motors)
#  -> this does not work due to StopIteration!  How does this code even exist???

chain_elements = [
'body_skeleton_link',
'body_torso_joint', 'body_torso_link',
'neck_axis_joint', 'neck_axis_link',
'neck_joint', 'neck_link',
'body_head_joint', 'body_head_link',
'body_back_joint', 'body_back_link',
'body_chest_joint', 'body_chest_link',
'l_shoulder_anchor_joint', 'l_shoulder_anchor_link',
'l_shoulder_servo_joint', 'l_shoulder_servo_link',
'l_shoulder_joint', 'l_shoulder_link',
'l_biceps_anchor_joint', 'l_biceps_anchor_link',
'l_biceps_axis_joint', 'l_biceps_axis_link',
'l_biceps_joint', 'l_biceps_link',
'l_biceps_spo_joint',
]

chain = Chain.from_urdf_file(DARWIN_URDF, base_elements=chain_elements)
print(chain)



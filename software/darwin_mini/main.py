from ikpy.chain import Chain
from ikpy.URDF_utils import get_chain_from_joints
from urdfpy import URDF
import math
import sys


def ang(offset_deg):
    abs_deg = 150 + offset_deg
    abs_rad = math.pi * abs_deg / 180
    return abs_rad


DARWIN_URDF = 'darwin.urdf'
robot = URDF.load(DARWIN_URDF)
joint_names = list(filter(lambda j: j.joint_type != 'fixed', robot.joints))
default_angle = math.pi * 150 / 180
cfg = dict(zip(joint_names, [default_angle] * len(joint_names)))
cfg['neck_joint'] = ang(-60)
cfg['l_shoulder_joint'] = ang(-60)
cfg['l_biceps_joint'] = ang(60)
cfg['l_elbow_joint'] = ang(-120)    # limit: 0-150
cfg['r_shoulder_joint'] = ang(-60)
cfg['r_biceps_joint'] = ang(-60)
cfg['r_elbow_joint'] = ang(120)   # limit:150-300
cfg['l_hip_joint'] = ang(30)
cfg['l_thigh_joint'] = ang(-60)
cfg['l_knee_joint'] = ang(30)
cfg['l_ankle_joint'] = ang(30)
cfg['l_foot_joint'] = ang(-30)
cfg['r_hip_joint'] = ang(-30)
cfg['r_thigh_joint'] = ang(-60)
cfg['r_knee_joint'] = ang(30)
cfg['r_ankle_joint'] = ang(30)
cfg['r_foot_joint'] = ang(30)
# robot.show(cfg=cfg)
# robot.animate()


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


def get_chain(name_templates, side):
    assert side in ['l', 'r']
    element_names = ['body_link'] + list(map(lambda s: s.format(side), name_templates))
    return Chain.from_urdf_file(DARWIN_URDF, base_elements=element_names)


arm_element_names = ['{}_shoulder_joint', '{}_shoulder_link',
                     '{}_biceps_joint', '{}_biceps_link',
                     '{}_elbow_joint', '{}_hand_link']
leg_element_names = ['{}_hip_joint', '{}_hip_link',
                     '{}_thigh_joint', '{}_thigh_link',
                     '{}_knee_joint', '{}_lowerleg_link',
                     '{}_ankle_joint', '{}_ankle_link',
                     '{}_foot_joint', '{}_foot_link']

left_arm = get_chain(arm_element_names, 'l')
right_arm = get_chain(arm_element_names, 'r')
left_leg = get_chain(leg_element_names, 'l')
right_leg = get_chain(leg_element_names, 'r')

# chain_elements = ['body_link', 'l_shoulder_joint', 'l_shoulder_link',
#                   'l_biceps_joint', 'l_biceps_link', 'l_elbow_joint',
#                   'l_hand_link']

# chain = Chain.from_urdf_file(DARWIN_URDF, base_elements=chain_elements)
# print(chain)

print('left_arm', left_arm)
print('right_arm', right_arm)
print('left_leg', left_leg)
print('right_leg', right_leg)



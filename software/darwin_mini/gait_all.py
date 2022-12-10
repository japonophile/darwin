#!/usr/bin/env python
# coding: utf-8

# # Darwin Mini gait generation

# <img src="assets/darwin-mini.png" width="400px">

# ## General approach

# The overall approach should be something like:
# 
# - based on a goal position, define hypothetical steps around the trajectory to the goal
# - have a rough idea of the COM trajectory
# - for each phase of the walking (DSP - SSP - DSP - SSP), define a trajectory for the joints so that the ZMP always stays within the acceptable area
# 
# Actually, the following is a great resource, that explains step by step what we need to do:
# https://scaron.info/teaching/prototyping-a-walking-pattern-generator.html

# ## Forward and inverse kinematics

# First, let's make sure we can compute feet position from joint configuration (forward kinematics), and required joint configuration to achieve a given feet position (inverse kinematics).

# ### DH parameters

# It seems we need to express the kinematic chain(s) of the robot using the D-H parameters.
# 
# Time to remember how to do this.
# 
# I found a great video that explains it: https://www.youtube.com/watch?v=rA9tm0gTln8

# I updated my V-REP scene, and added a frame of reference object to each joint of the scene.  At the same time, I cleaned up the joints, and made sure they follow the D-H conventions:
# 
# - x axis of the next link based at the common normal intersection
# - x axis of the next link pointing away from the previous link

# <img src="assets/darwin-3d-joint-config.png" width="400px">

# Saved the scene as `darwin_mini_ref.ttt`.

# I measured the D-H parameters in V-REP, and (fortunately enough) obtained the same values as reported by the V-REP plugin above.  However, I later changed the hip joint (which caused a difference in theta of the hip), **and** I figured out I had to add a frame of reference in the foot, to obtain the last line of my DH table, which V-REP had not produced.
# 
# *TODO: revise arms.*
# 
# |Link    |  d   |theta| a/r  |alpha|
# |--------|------|-----|------|-----|
# |l_biceps|0.0180| 90.0|0.0120| 90.0|
# |l_elbow |0.0000| -0.7|0.0435|  0.0|
# 
# |Link    |  d   |theta| a/r  |alpha|
# |--------|------|-----|------|-----|
# |r_biceps|0.0180|-90.0|0.0120|-90.0|
# |r_elbow |0.0000|  0.7|0.0435|  0.0|
# 
# |Link    |  d   |theta| a/r  |alpha|
# |--------|------|-----|------|-----|
# |l_hip   |0.0180|-90.0|0.0060| 90.0|
# |l_thigh |0.0000|  0.0|0.0450|  0.0|
# |l_knee  |0.0000|  0.0|0.0420|  0.0|
# |l_ankle |0.0010|180.0|0.0060| 90.0|
# |l_foot  |0.0040|-90.0|0.0090|-90.0|
# 
# |Link    |  d   |theta| a/r  |alpha|
# |--------|------|-----|------|-----|
# |r_hip   |0.0180| 90.0|0.0060|-90.0|
# |r_thigh |0.0000|  0.0|0.0450|  0.0|
# |r_knee  |0.0000|  0.0|0.0420|  0.0|
# |r_ankle |0.0010|180.0|0.0060|-90.0|
# |r_foot  |0.0040| 90.0|0.0090| 90.0|
# 

# OK, now let's see if we can use the `pypot.kinematics` module.

# ### Using the `pypot.kinematics` module

# In[1]:


from pypot.kinematics import Link, Chain
from numpy import deg2rad


# In[2]:


import numpy as np
np.set_printoptions(precision=3, suppress=True)


# Start from the hip joint, and add a last frame of ref in the foot, following the DH convention.  Then, we will need to define the transformation between this last frame of ref and the base of the foot on the floor (= "end effector").

# Note: At first, I had defined an additional "fake joint" between the pelvis and the hip, but this turned out not to be correct.  Instead, I had to define a frame of reference in the foot, to which the last link in the chain would map the foot joint to.

# In[3]:


#l_biceps = Link(d=0.0180, theta=deg2rad(90.0), a=0.0120, alpha=deg2rad(90.0))
#l_elbow  = Link(d=0.0000, theta=deg2rad(-0.7), a=0.0435, alpha=deg2rad(0.0))

#r_biceps = Link(d=0.0180, theta=deg2rad(-90.0), a=0.0120, alpha=deg2rad(-90.0))
#r_elbow  = Link(d=0.0000, theta=deg2rad(0.7), a=0.0435, alpha=0.0)

l_hip    = Link(d=0.0180, theta=deg2rad(-90.0), a=0.0060, alpha=deg2rad(90.0))
l_thigh  = Link(d=0.0000, theta=0.0, a=0.0450, alpha=0.0)
l_knee   = Link(d=0.0000, theta=0.0, a=0.0420, alpha=0.0)
l_ankle  = Link(d=0.0010, theta=deg2rad(180.0), a=0.0060, alpha=deg2rad(90.0))
l_foot   = Link(d=0.0040, theta=deg2rad(-90.0), a=0.0090, alpha=deg2rad(-90.0))

r_hip    = Link(d=0.0180, theta=deg2rad(90.0), a=0.0060, alpha=deg2rad(-90.0))
r_thigh  = Link(d=0.0000, theta=0.0, a=0.0450, alpha=0.0)
r_knee   = Link(d=0.0000, theta=0.0, a=0.0420, alpha=0.0)
r_ankle  = Link(d=0.0010, theta=deg2rad(-180.0), a=0.0060, alpha=deg2rad(-90.0))
r_foot   = Link(d=0.0040, theta=deg2rad(90.0), a=0.0090, alpha=deg2rad(90.0))


# Determine transformation matrices of the first joint of each kinematic chain, wrt the body.

# In[4]:


# > sim.getObjectHandle('l_shoulder_joint')
# 21
# > sim.getObjectHandle('body_respondable')
# 19
# > sim.getObjectMatrix(21, 19)
# {1, -4.144544619e-08, 6.961163308e-06, 0.001000333577, -6.961163308e-06, -1.192092896e-06, 1.000000119, 0.03899861872, -4.14383976e-08, -1.000000119, -1.192092896e-06, 0.0300001204} 

# l_shoulder_T = [[1, 0, 0, 0.001],
#                 [0, 0, 1, 0.039],
#                 [0, -1, 0, 0.030],
#                 [0, 0, 0, 1]]

# {1, 4.067101145e-08, -6.94807477e-06, 0.0009997934103, -6.94807477e-06, 1.192092896e-06, -1, -0.03900143132, -4.06619165e-08, 1, 1.192092896e-06, 0.0300001502} 
# r_shoulder_T = [[1, 0, 0, 0.001],
#                 [0, 0, -1, -0.039],
#                 [0, 1, 0, 0.030],
#                 [0, 0, 0, 1]]

## {1.788139343e-07, 6.794906767e-06, 1, -0.001999982633, 1.171752047e-07, 1, -6.794906767e-06, 0.02399860322, -1, 1.171765689e-07, 1.788139343e-07, -0.04199992865} 
l_hip_T = [[0, 0, 1, -0.002],
          [1, 0, 0, 0.024],
          [0, 1, 0, -0.042],
          [0, 0, 0, 1]]

## {-2.026557922e-06, 6.710644811e-06, 1.000000119, -0.001999982633, -5.143601811e-08, 1, -6.710644811e-06, -0.02400141023, -1.000000119, -5.144852366e-08, -2.026557922e-06, -0.04200158268} 
r_hip_T = [[0, 0, 1, -0.002],
          [-1, 0, 0, -0.024],
          [0, -1, 0, -0.042],
          [0, 0, 0, 1]]


# Determine transformation matrices of each end effector (for now, only feet) wrt to their respective joint.

# In[5]:


l_foot_T = [[0, 1, 0, 0.0],
            [-1, 0, 0, 0.0],
            [0, 0, 1, -0.031],
            [0, 0, 0, 1]]

r_foot_T = [[0, -1, 0, 0.0],
            [1, 0, 0, 0.0],
            [0, 0, 1, -0.031],
            [0, 0, 0, 1]]


# Now, we can define the kinematic chains for the legs.

# In[6]:


l_leg = Chain([l_hip, l_thigh, l_knee, l_ankle, l_foot], l_hip_T, l_foot_T)
r_leg = Chain([r_hip, r_thigh, r_knee, r_ankle, r_foot], r_hip_T, r_foot_T)


# In[7]:


d = 0.


# In[8]:


l_foot_pos = l_leg.forward_kinematics([d, d, d, d, d])
l_foot_pos[0][:3,3]


# In[9]:


r_foot_pos = r_leg.forward_kinematics([d, d, d, d, d])
r_foot_pos[0][:3,3]


# In[10]:


l_foot_pos[0]


# OK, this looks good, since the `body_respondable` is at 160mm above the ground, so at the default joint position of 0, both feet are at -160mm wrt the body, that is, right on the ground.

# ### Closed form for Inverse Kinematics

# Instead of using `pypot.kinematics` for computing inverse kinematics for Darwin Mini, we can create a very simple inverse kinematic closed form (i.e. a python function) that does not need to do optimization, just by computing joint angles based on the leg length to be achieved.

# <img src="assets/darwin-joint-diagram.png" width="600px">

# Assumption: foot is flat againsta the floot (no pitch nor roll)<br>
# => `q0` and `q4` must have same amplitude<br>
# => `q3 = q2 - q1`

# Define `L` as the length of the leg as seen from the front view:<br>
# `L * cos(q0) = (0.118 - 0.031 - dz)`<br>
# `L * sin(q0) = dy`

# `q0 = atan(dy / (0.118 - 0.031 - dz))`<br>
# `L = dy / sin(q0)`<br>
# 

# Find the angle of the leg in the leg plane:<br>
# `alpha = atan(dx / (L - a1 + a4))`<br>
# 
# Define `d` as the distance from thigh to ankle (just 2 links, thigh and lower leg):<br>
# `d = dx / sin(alpha)`<br>
# 
# Compute the angles of the thigh-knee-ankle triangle:<br>
# `beta1 = acos((a2**2 + d**2 - a3**2) / (2 * a2 * d))`<br>
# `beta3 = acos((a3**2 + d**2 - a2**2) / (2 * a3 * d))`<br>
# (`beta2 = pi - beta1 - beta3`)<br>
# 
# Find the angles of the thigh, knee and ankle joints:<br>
# `q1 = beta1 + alpha`<br>
# `q3 = beta3 - alpha`<br>
# `q2 = -(q1 + q3)`<br>

# Finally, find the angle of the foot:<br>
# `q4 = -q0`

# In[7]:


from math import sin, acos, atan


EPS = 1e-6
# PELVIS_HEIGHT = 0.118
# FOOT_JOINT_HEIGHT = 0.031
a1 = 0.006
a2 = 0.045
a3 = 0.042
a4 = 0.006


def darwin_leg_ik(dx, dy, dz):
    assert dz >= 0  # dx and dy are allowed to be negative
    q = [.0] * 5
    #q[0] = atan(dy / (PELVIS_HEIGHT - FOOT_JOINT_HEIGHT - dz))
    q[0] = atan(dy / (a1 + a2 + a3 - a4 - dz))
    # length of the leg as seen from the front view
    L = dy / sin(q[0]) if abs(dy) > EPS else a1 + a2 + a3 - a4 - dz
    assert L <= a1 + a2 + a3 - a4, "Leg cannot be extended beyond limit. Use larger dz and/or smaller dy."
    # angle of the leg in the leg plane
    alpha = atan(dx / (L - a1 + a4))
    # distance from thigh to ankle (just 2 links, thigh and lower leg)
    d = dx / sin(alpha) if abs(dx) > EPS else L - a1 + a4
    assert d <= a2 + a3, "Leg cannot be extended beyond limit. Use larger dz, smaller dy and/or smaller dx."
    # angles of the thigh-knee-ankle triangle
    beta1 = acos((a2**2 + d**2 - a3**2) / (2 * a2 * d))
    beta3 = acos((a3**2 + d**2 - a2**2) / (2 * a3 * d))
    # angles of the thigh, knee and ankle joints
    q[1] = beta1 + alpha
    q[3] = beta3 - alpha
    q[2] = -(q[1] + q[3])
    # angle of the foot
    q[4] = -q[0] if abs(q[0]) > EPS else 0.0
    return q


# In[8]:


import numpy as np
from numpy import deg2rad


# In[15]:


for p in [(.0, .0, .0),
          (.0, .0, .0001),
          (.01, .0, .015),
          (.01, .0, .025),
          (.0, .0, .025),
          (-.01, .0, .025),
          (-.06, .0, .025),
          (.0, .01, .025),
          (.01, .01, .025)]:
    q = darwin_leg_ik(*p)
    print(p, q)
    print(l_leg.forward_kinematics(q)[1][-1][:3,3].flatten())


# OK, now we have solved the forward and inverse kinematics problems, let's go back to gait generation.

# ## Gait generation

# The following video was helpful to understand what needs to be done:<br>
# https://www.youtube.com/watch?v=EUZLMSqyd9Q

# ### Single Support Phase
# 

# #### Cartesian Trajectory
# 
# 1. Define the COM (**pelvis**) trajectory wrt the **stable leg** foot
# 1. Define the **swing leg** foot trajectory wrt the **pelvis**
# 
# #### Joint Trajectory
# 
# 1. Apply IK on **stable leg** chain (with foot fixed) to determine stable leg joint positions
# 1. Apply IK on **swing leg** chain (with pelvis fixed) to determine swing leg joint positions

# ### Double Support Phase

# ...

# <img src="assets/darwin-footstep-diagram.png" width="400px">

# For a step of length `Lstep`, we:
# - move the COM of `0.5 * Lstep` in the `x` direction (from `x - 0.25 * Lstep` to `x + 0.25 * Lstep`)
# - move the swing leg of `Lstep` in the `x` direction (from `x - 0.5 * Lstep` to `x + 0.5 * Lstep`)
# 
# where `x` is the position of the stable foot
# 
# At the end of the step, we are back to DSP (= Double Support Phase), and
# - the COM is between both legs, at `x - 0.25 * Lstep`
# - the next swing leg (which used to be the stable leg) is now at `x - 0.5 * Lstep`
# 
# where `x` = the position of the next stable foot
# 
# Note: the first step is a half step, so we:
# - move the COM of `0.25 * Lstep` in the `x` direction (from `x` to `x + 0.25 * Lstep`)
# - move the swing leg of `0.5 * Lstep` in the `x` direction (from `x` to `x + 0.5 * Lstep`)

# In[10]:


from math import sqrt


# com_height_rest = 0.160
pelvis_height_rest = 0.118
joint_names = ['hip', 'thigh', 'knee', 'ankle', 'foot']


class FootstepTrajectory(object):
    def __init__(self, step_length, step_height, x_stable, x_swing_start, pelvis_height, swing_side, duration,
                 pelvis_start_vel=[0., 0.], pelvis_end_vel=[0., 0.]):
        assert swing_side in ['l', 'r'], "swing_side must be either 'l' or 'r'"
        assert pelvis_height < pelvis_height_rest, "pelvis must be lower than {}".format(pelvis_height_rest)
        self.step_length = step_length
        self.step_height = step_height
        self.x_stable = x_stable
        self.x_swing_start = x_swing_start
        self.pelvis_height = pelvis_height
        self.swing_side = swing_side
        self.duration = duration
        self.pelvis_start_vel = pelvis_start_vel
        self.pelvis_end_vel = pelvis_end_vel
        # assume (for now):
        # - both feet are on the ground before starting the trajectory
        # - pelvis is (and stays) at y = 0
        # - pelvis start and end velocities are given as [vx, vz]
    
    def get_pelvis_pos(self, t):
        L = self.step_length
        xs = 0.5 * (self.x_stable + self.x_swing_start)
        vs = self.pelvis_start_vel[0]
        ve = self.pelvis_end_vel[0]
        tf = self.duration
        r4 = -2 * (L / (2 * tf**3) - (vs + ve) / (2 * tf**2))
        x_pel = (xs + vs * t + ((ve - vs) / (2 * tf) - r4 * 1.5 * tf) * t**2
                 - 2 * (L / (2 * tf**3) - (vs + ve) / (2 * tf**2)) * t**3)
        h = self.pelvis_height
        z_pel = sqrt(h**2 - (x_pel - self.x_stable)**2)
        return x_pel, z_pel

    def get_swing_foot_pos(self, t):
        L = self.step_length
        h = self.step_height
        xs = self.x_swing_start
        tf = self.duration
        x_foot = xs + (3 * L / tf**2) * t**2 - (2 * L / tf**3) * t**3
        z_foot = (4 * h * t / tf) * (1 - t / tf)
        return x_foot, z_foot
    
    def get_stable_foot_disp_vs_pelvis(self, pelvis_pos):
        x_pel, z_pel = pelvis_pos
        dx = self.x_stable - x_pel
        dz = pelvis_height_rest - z_pel
        return dx, dz
    
    def get_swing_foot_disp_vs_pelvis(self, swing_foot_pos, pelvis_pos):
        x_foot, z_foot = swing_foot_pos
        x_pel, z_pel = pelvis_pos
        dx = x_foot - x_pel
        dz = pelvis_height_rest - z_pel - z_foot
        return dx, dz

    def __call__(self, t):
        """
        Returns a map of joint positions at time t.
        """
        
        # Pelvis motion
        pelvis_pos = self.get_pelvis_pos(t)
        dx, dz = self.get_stable_foot_disp_vs_pelvis(pelvis_pos)
        print('stable foot vs pelvis:  dx={:.4f}, dz={:.4f}'.format(dx, dz))
        q_stable = darwin_leg_ik(dx, 0., dz)
        
        # Swing foot motion
        swing_foot_pos = self.get_swing_foot_pos(t)
        dx, dz = self.get_swing_foot_disp_vs_pelvis(swing_foot_pos, pelvis_pos)
        print('swing foot vs pelvis:   dx={:.4f}, dz={:.4f}, z_foot={:.4f}'.format(dx, dz, swing_foot_pos[1]))
        q_swing = darwin_leg_ik(dx, 0., dz)
        
        # Return a map of joint positions
        stable_side = 'l' if self.swing_side == 'r' else 'r'
        stable_joints = ['_'.join([stable_side, n, 'joint']) for n in joint_names]
        swing_joints = ['_'.join([self.swing_side, n, 'joint']) for n in joint_names]
        
        return {**dict(zip(stable_joints, q_stable)),
                **dict(zip(swing_joints, q_swing))}


# #### Initial step SSP

# In[213]:


traj = FootstepTrajectory(.05, .005, 0., 0., .088, 'r', 1.)


# In[214]:


t0 = [traj(t) for t in [.0, .25, .5, .75, 1.]]


# ##### Stable foot vs pelvis

# In `x`:
# - at `t0`, the stable foot is at the same `x` position as the pelvis => `dx = 0.0`
# - halfway (at `tf/2`), the pelvis has moved of 1/4 step forward, so the stable foot is behind => `dx = -0.0125`
# - at the end of the first step (at `tf`), the pelvis has moved of 1/2 step, so the stable foot is behind by half of the step size => `dx = -0.025`
# 
# Note: the step size for this first step is `L = 0.05` (= half of the regular step size)
# 
# In `z`:
# - at `t0`, we start at 0.88 (`dz = 0.03`)
# - at `tf`, we end a bit lower (`dz = 0.0336`), as the feet are appart from each other

# ##### Swing foot vs pelvis

# In `x`:
# - at `t0`, the swing foot is at the same `x` position as the pelvis => `dx = 0.0`
# - at `tf`, the swing foot has moved of one step, so the swing step is ahead of the pelvis by half of the step size => `dx = 0.025`
# 
# In `z`:
# - at `t0`, the swing foot starts at `0.0`, so `dz = 0.03` (same as stable foot)
# - halfway (at `tf/2`), the swing foot is at its highest, `0.005` above the ground, so `dz = 0.026` that is `0.005` less than the `dz` of the stable foot
# - at `tf`, the swing foot height is back to `0.0`, so `dz = 0.0336` (same as stable foot)

# #### Second step SSP

# In[215]:


traj = FootstepTrajectory(0.1, .005, .05, 0., .088, 'l', 1.)


# In[216]:


t1 = [traj(t) for t in [.0, .25, .5, .75, 1.]]


# ##### Stable foot vs pelvis

# Now, we move the other leg, of a full step size of `L = 0.1`.
# 
# In `x`:
# - at `t0`, the stable foot is ahead of the pelvis of 1/4 of the step size (that is, 1/2 of the initial step) => `dx = 0.025`<br>
# (note the sign is reversed wrt the previous SSP, because the stable foot is now what used to be the swing foot before, so it it *ahead* of the pelvis)
# - halfway (at `tf/2`), the pelvis has moved 1/4 step forward, and is just above the stable foot => `dx = 0.0`
# - at the end of the step (at `tf`), the pelvis has moved by 1/2 step since `t0` (i.e. is 1/4 step *ahead* of the stable foot), so the stable foot is behind by 1/4 of the step size => `dx = -0.025`
# 
# In `z`:
# - at `t0`, we start at the same height as where the previous step ended (`dz = 0.0336`)
# - halfway, we are back at `z = 0.088`, that is `dz = 0.03` as the pelvis is just above the stable foot
# - at `tf`, the pelvis goes down again, to end at the same height as the beginning of the step (`dz = 0.0336`)

# ##### Swing foot vs pelvis

# In `x`:
# - at `t0`, the swing foot is behind the pelvis of 1/4 of the step size => `dx = -0.025`
# - halfway, the swing foot moved 1/2 step forward and is at the same location as the pelvis => `dx = 0.0`
# - at `tf`, the swing foot moved by a full step length since `t0`, and is ahead of the pelvis by 1/4 step size => `dx = 0.025`
# 
# In `z`:
# - at `t0`, the swing foot starts at `0.0`, that is `dz` is the same as for the stable foot (`dz = 0.0336`)
# - halfway, the swing foot is at its highest point, so `dz = 0.025`, that is `0.005` less than the stable foot
# - at `tf`, the swing foot height is back to the `0.0`, so `dz` is the same as for the stable foot again (`dz = 0.0336`)

# #### Third step SSP

# In[217]:


traj = FootstepTrajectory(0.1, .005, .1, .05, .088, 'r', 1.)


# In[218]:


t2 = [traj(t) for t in [.0, .25, .5, .75, 1.]]


# We can confirm that the result is exactly the same as the second step, however, the joints should alternate.

# In[219]:


print(t1[2])
print(t2[2])


# #### Last step SSP

# In[222]:


traj = FootstepTrajectory(0.05, .005, .15, .1, .088, 'l', 1.)


# In[223]:


t3 = [traj(t) for t in [.0, .25, .5, .75, 1.]]


# Now, let's try to play this on the simulator.

# In[11]:


from pypot.primitive import LoopPrimitive


class WalkStraight(LoopPrimitive):
    def __init__(self, robot, distance, step_duration, frequency=50):
        LoopPrimitive.__init__(self, robot, frequency)
        
        self.distance_left = distance
        self.step_duration = step_duration

    def setup(self):
        self.step_length = 0.1
        self.step_height = 0.005
        self.pelvis_height = 0.088
        self.swing_side = 'r'
        self.x_stable = 0.
        self.x_swing_start = 0.
        self.current_step = 0
        self.current_step_length = self.get_current_step_length()
        self.current_trajectory = self.create_footstep_trajectory()

    def update(self):
        t = self.elapsed_time
        step = t // self.step_duration
        t = t % step
        if step > self.current_step:
            self.current_step = step
            self.start_next_step()
        q = self.current_trajectory(t)
        print(t, q)
        self.goto_position(q)
    
    def get_current_step_length(self):
        next_step_max_length = self.step_length if self.current_step > 0 else 0.5 * self.step_length
        if self.distance_left > next_step_max_length:
            current_step_length = next_step_max_length
        else:
            current_step_length = self.distance_left
        return current_step_length
    
    def create_footstep_trajectory(self):
        return FootstepTrajectory(self.current_step_length, self.step_height,
                                  self.x_stable, self.x_swing_start,
                                  self.pelvis_height, self.swing_side,
                                  self.step_duration)
    
    def start_next_step(self):
        self.swing_side = 'l' if self.swing_side == 'r' else 'r'
        next_swing_start = self.x_stable
        self.x_stable = self.x_swing_start + self.current_step_length
        self.x_swing_start = next_swing_start
        self.distance_left = max(0, self.distance_left - self.current_step_length)
        if self.distance_left < 1e-6:
            # We have arrived.  Join feet.
            self.current_step_length = self.x_stable - self.x_swing_start
            if self.current_step_length < 1e-6:
                print('Done!')
                self.stop()
                return
        else:
            self.current_step_length = self.get_current_step_length()
        self.current_trajectory = self.create_footstep_trajectory()
    
    def goto_position(self, q):
        for name, p in q.items():
            m = getattr(self.robot, name)
            m.goal_position = p


# In[12]:


from pypot.creatures import DarwinMini

darwin = DarwinMini(simulator='vrep')


# In[13]:


w = WalkStraight(darwin, 0.5, 1.0)


# In[14]:


w.start()


# In[9]:


darwin.neck_joint.goal_position = 0


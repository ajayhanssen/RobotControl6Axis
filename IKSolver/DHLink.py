import numpy as np
from ikpy.chain import Chain
from ikpy.link import DHLink

scale = 1000
a1 = 110.5  / scale
a2 = 23.42  / scale
a3 = 180.0  / scale
a4 = 43.5   / scale
a5 = 176.35 / scale
a6 = 62.8   / scale
a7 = 45.25  / scale

my_chain = Chain(name='robot_arm', links=[
    DHLink(name="base",          d=a1, a=a2),
    DHLink(name="joint_1",       d=0,  a=a3),
    DHLink(name="joint_2",       d=0,  a=-a4),
    DHLink(name="joint_3",       d=-a5,a=0),
    DHLink(name="joint_4",       d=0,  a=0),
    DHLink(name="joint_5",       d=-a6,a=-a7),
])

target_position = [0.3, 0, 0.2]
target_frame = np.eye(4)
target_frame[:3, 3] = target_position

joint_angles = my_chain.inverse_kinematics(target_position)
print(joint_angles)

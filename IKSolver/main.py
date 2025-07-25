from ikpy.chain import Chain
from ikpy.link import URDFLink
import numpy as np
import matplotlib.pyplot as plt

class Joint():
    def __init__(self, thetaOffs, alpha, a, d):
        self.thetaOffs = thetaOffs
        self.alpha = alpha
        self.a = a
        self.d = d

def defineKinematicChain(joints):

    chain = Chain(name="boboter", links=[
        URDFLink(
            name="base_link",
            origin_translation=[0,0,0],
            origin_orientation=[0,0,0],
            rotation=[0,0,0]
        )
    ])

    for i, joint in enumerate(joints):
        link = URDFLink(
            name=f"joint{i+1}",
            origin_translation=[joint.a, 0, joint.d],
            origin_orientation=[joint.alpha, 0, 0],
            rotation=[0,0,1]
        )
        chain.links.append(link)
    
    return chain

def main():
    scale = 1000
    a1 = 110.5  / scale
    a2 = 23.42  / scale
    a3 = 180.0  / scale
    a4 = 43.5   / scale
    a5 = 176.35 / scale
    a6 = 62.8   / scale
    a7 = 45.25  / scale

    joints = [
        Joint(0,       -np.pi/2,   a2,     a1),
        Joint(-np.pi/2, np.pi,     a3,     0 ),
        Joint(np.pi,    np.pi/2,  -a4,     0 ),
        Joint(0,       -np.pi/2,   0,     -a5),
        Joint(0,        np.pi/2,   0,      0),
        Joint(np.pi,    np.pi,    -a7,    -a6)
    ]  

    chain = defineKinematicChain(joints)
    thetaOffs = [0].append([j.thetaOffs for j in joints])

    target = [0.300, 0, 0.200]
    #angles = chain.inverse_kinematics(target_position=target)

    """ print("Inverse kinematics result (radians):")
    for i, a in enumerate(angles):
        print(f"Joint {i}: {a:.4f} rad") """
    
    # create matplotlib figure
    plt.figure(figsize=(10, 10))
    chain.plot(thetaOffs, ax=plt.gca())


if __name__ == "__main__":
    main()

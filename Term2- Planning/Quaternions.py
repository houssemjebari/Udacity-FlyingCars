import numpy as np

def euler_to_quaternion(angles):
    roll = angles[0]
    pitch = angles[1]
    yaw = angles[2]

    sinroll = np.sin(roll/2)
    cosroll = np.cos(roll/2)
    sinpitch = np.sin(pitch/2)
    cospitch = np.cos(pitch/2)
    sinyaw = np.sin(yaw/2)
    cosyaw = np.cos(yaw/2)
    
    a = cosroll * cospitch * cosyaw + sinroll * sinpitch * sinyaw
    b = sinroll * cospitch * cosyaw - cosroll * sinpitch * sinyaw
    c = cosroll * sinpitch * cosyaw + sinroll * cospitch * sinyaw
    d = cosroll * cospitch * sinyaw - sinroll * sinpitch * cosyaw
    
    q = [a,b,c,d]
    
    return q


def quaternion_to_euler(quaternion):
    a = quaternion[0]
    b = quaternion[1]
    c = quaternion[2]
    d = quaternion[3]
    roll = np.arctan2(2*(a*b + c*d),1 - 2*(b**2+c**2))
    pitch = np.arcsin2(2*(a*c-d*b))
    yaw = np.arctan2(2*(a*d+b*c),1-2*(c**2+d**2))
    angles = [roll,pitch,yaw]
    return angles
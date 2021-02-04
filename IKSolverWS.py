from math import sin, cos, tan, atan, atan2, acos, asin, sqrt, radians, pi
"""
spot:   z  y  x
4l:     x  z  y
opqp:   x  y  z
"""


def legIK(x,y,z, l1, femur_offset, l2, l3):
    """
    x/y/z=Position of the Foot in Leg-Space

    F=Length of shoulder-point to target-point on x/y only
    G=length we need to reach to the point on x/y
    H=3-Dimensional length we need to reach
    """
    F=sqrt(x**2+y**2-femur_offset**2)
    theta1=-atan2(-y,x)-atan2(F,-femur_offset)

    G=F-l1
    H=sqrt(G**2+z**2)
    D=(H**2-l2**2-l3**2)/(2*l2*l3)
    theta3=acos(D) 

    theta2=atan2(z,G)-atan2(l3*sin(theta3), l2+l3*cos(theta3))

    return [theta1, theta2, theta3]


def legIK_optimized(x,y,z, l1, femur_offset, l2, l3):
    """
    x/y/z=Position of the Foot in Leg-Space

    F=Length of shoulder-point to target-point on x/y only
    G=length we need to reach to the point on x/y
    H=3-Dimensional length we need to reach
    """
    F=sqrt(x**2+y**2-femur_offset**2)
    theta1=atan2(y,x)+atan2(F,femur_offset)
    theta1 = theta1 - pi/2

    G=F-l1
    H=sqrt(G**2+z**2)
    D=(l2**2+l3**2-H**2)/(2*l2*l3)
    theta3=acos(D) 

    theta2=atan2(l3*sin(theta3), l2-l3*cos(theta3)) - atan2(z,G)

    return [theta1, theta2, theta3]


def solve_R(x, y, z, l1 , l2 , l3): 
    D = (y**2+(-z)**2-l1**2+(-x)**2-l2**2-l3**2)/(2*l3*l2)
    if D > 1 or D < -1:
        if D > 1: 
            D = 0.99
            return D
        elif D < -1:
            D = -0.99

    theta3 = atan2(-sqrt(1-D**2), D)
    theta1 = -atan2(z,y)-atan2(sqrt(y**2+(-z)**2-l1**2),-l1)
    theta2 = atan2(-x,sqrt(y**2+(-z)**2-l1**2))-atan2(l3*sin(theta3),l2+l3*cos(theta3))
    
    return [-theta1, theta2, theta3]


class InverseKinematics:
    def __init__(self, forearm, shoulder, hip_offset):
        self.wrist = forearm
        self.shoulder = shoulder
        self.hip_offset = hip_offset  # z, y
        self.joint_angles = []

    def local_translation_engine(self, legs_xyz):
        '''
        Translation engine for
        '''
        try:
            joint_angles = []
            for i, (x, y, z) in enumerate(legs_xyz):
                h1 = sqrt(self.hip_offset[0]**2 + self.hip_offset[1]**2)
                h2 = sqrt(z**2 + y**2)

                alpha_0 = atan(y / z)
                alpha_1 = atan(self.hip_offset[1] / self.hip_offset[0])
                alpha_2 = atan(self.hip_offset[0] / self.hip_offset[1])
                alpha_3 = asin(h1 * sin(alpha_2 + radians(90)) / h2)
                alpha_4 = radians(180) - (alpha_3 + alpha_2 + radians(90))
                
                alpha_5 = alpha_1 - alpha_4
                theta_h = alpha_0 - alpha_5
                
                r0 = h1 * sin(alpha_4) / sin(alpha_3)
                h = sqrt(r0**2 + x**2)
                phi = asin(x / h)
                theta_s = acos((h**2 + self.shoulder**2 - self.wrist**2) / (2 * h * self.shoulder)) - phi
                theta_w = acos((self.wrist**2 + self.shoulder ** 2 - h**2) / (2 * self.wrist * self.shoulder))

                if i < 2:
                    joint_angles.append((theta_h, theta_s, theta_w))
                else:
                    joint_angles.append((-theta_h, theta_s, theta_w))
            self.joint_angles = joint_angles
        except:
            print("Out of bounds.")
        return self.joint_angles


if __name__ == '__main__':
    htf_vecs = [(10, 30, 90)]  # x(forward) y(offside) z(height)
    opqp_leg = InverseKinematics(forearm=80, shoulder=80, hip_offset=(20, 20))
    opqp_joint_angles = opqp_leg.local_translation_engine(htf_vecs)
    print([round(angle, 4) for angle in opqp_joint_angles[0]])

    htf_x, htf_y, htf_z = htf_vecs[0][2], htf_vecs[0][1], htf_vecs[0][0]
    spot_joint_angles = legIK(x=htf_x, y=htf_y, z=htf_z, l1=20, femur_offset=20, l2=80, l3=80)
    spot_joint_angles[0] = spot_joint_angles[0] + pi/2  # 基准轴为竖直方向
    spot_joint_angles[2] = pi - spot_joint_angles[2]
    spot_joint_angles[1] = -spot_joint_angles[1]
    print([round(angle, 4) for angle in spot_joint_angles])

    htf_x, htf_y, htf_z = htf_vecs[0][2], htf_vecs[0][1], htf_vecs[0][0]
    spot_joint_angles = legIK_optimized(x=htf_x, y=htf_y, z=htf_z, l1=20, femur_offset=20, l2=80, l3=80)
    print([round(angle, 4) for angle in spot_joint_angles])
    
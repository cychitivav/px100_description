import roboticstoolbox as rtb  # Robotics and numerical libraries
from spatialmath import *
from spatialmath.base import *
import numpy as np
import rospy  # ROS libraries
from dynamixel_workbench_msgs.srv import DynamixelCommand
from sensor_msgs.msg import JointState


class PX(rtb.DHRobot):
    def __init__(self, L1=0.08945, L2=0.1, L3=0.1, L4=0.119, Lm=0.035):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L4 = L4
        self.Lm = Lm

        # Create robot
        super().__init__(
            [rtb.DHLink(d=L1,   a=0,        alpha=-np.pi/2, offset=0),
             rtb.DHLink(d=0,    a=np.sqrt(L2**2+self.Lm**2),
                        alpha=0,        offset=-np.arctan2(self.L2, self.Lm)),
             rtb.DHLink(d=0,    a=self.L3,  alpha=0,
                        offset=np.arctan2(self.L2, self.Lm)),
             rtb.DHLink(d=0,    a=self.L4,  alpha=0,        offset=0)],
            name="Filoberta",
            tool=SE3(trotx(-np.pi/2)@troty(np.pi/2))
        )

        self.joints = [0, 0, 0, 0]

        # ROS node to publish joint states if run_dynamixel is False
        self.sub = rospy.Subscriber(
            '/joint_states', JointState, self.updateJoints)


    # Function to send commands to the dynamixel with the dynamixel_command service
    def jointCommand(self, id_num, addr_name, value):
        rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
        try:
            dynamixel_command = rospy.ServiceProxy(
                '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            result = dynamixel_command('', id_num, addr_name, value)
            return result.comm_result
        except rospy.ServiceException as e:
            rospy.logwarn(str(e))

    def rad2bin(self, angle):  # Function to convert radians to binary (0-4095)
        return round(angle/(2*np.pi)*4095+2048)

    def deg2bin(self, angle):  # Function to convert radians to binary (0-4095)
        return round(angle/360*4095+2048)

    def updateJoints(self, msg):
        names = msg.name
        unorderedJoints = msg.position

        # Order joints in the same order as the motors
        self.joints[0] = unorderedJoints[names.index('waist')]
        self.joints[1] = unorderedJoints[names.index('shoulder')]
        self.joints[2] = unorderedJoints[names.index('elbow')]
        self.joints[3] = unorderedJoints[names.index('wrist')]

    def ikine(self, T, **kwargs):
        """ IKINE Returns the position of the joints to reach the desired pose T of 
        the TCP

            q = ikine(T) are the joint coordinates (1x4) corresponding to the  
            pincher x100 robot end-effector pose T (4x4) which is a homogeneous 
            transform.

            q = ikine(...,OPTION,Value) S

            OPTIONS:
                'elbow', E: Change the return configuration between elbow up or elbow 
                            down. Can be specified as 'up' or 'down'."""

        up = True

        if len(kwargs) == 0:
            pass
        elif len(kwargs) > 0:
            if 'elbow' in kwargs:
                if kwargs['elbow'] == 'up':
                    up = True
                elif kwargs['elbow'] == 'down':
                    up = False
                else:
                    rospy.logerror('Invalid elbow option. Use "up" or "down"')
            else:
                rospy.logerror(kwargs.keys()[0] + 'option unknown')
        else:
            rospy.logerror('Invalid number of arguments')

        q = np.zeros([1, 4])[0]

        # q1 (Waist)
        q[0] = np.arctan2(T[1, 3], T[0, 3])

        # Wrist decoupling
        a = T[0:3, 2]
        w = T[0:3, 3]-self.L4*a

        # 2R mechanism
        r = np.sqrt(w[0]**2+w[1]**2)
        h = w[2]-self.L1

        c = np.sqrt(r**2+h**2)

        beta = np.arctan2(self.Lm, self.L2)
        psi = np.pi/2-beta
        Lr = np.sqrt(self.Lm**2+self.L2**2)

        phi = np.arccos((c**2-self.L3**2-Lr**2)/(-2*Lr*self.L3))

        gamma = np.arctan2(h, r)
        alpha = np.arccos((self.L3**2-Lr**2-c**2)/(-2*Lr*c))

        # q2 (Shoulder)
        if up:
            q[1] = np.pi/2-beta-alpha-gamma
        else:
            q[1] = np.pi/2-(gamma-alpha+beta)

        # q3 (Elbow)
        if up:
            q[2] = np.pi - psi-phi
        else:
            q[2] = -np.pi+(phi-psi)

        # q4 (Wrist)
        angA = np.arctan2(np.sqrt(T[1, 2]**2+T[0, 2]**2), T[2, 2])
        q[3] = angA-q[1]-np.pi/2-q[2]

        return q


if __name__ == "__main__":
    # Initialize the node and instantiate the class PXrobot
    rospy.init_node('TeleopKey', anonymous=False)

    PXrobot()

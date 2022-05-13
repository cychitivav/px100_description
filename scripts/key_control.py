import roboticstoolbox as rtb
import rospy  # ROS libraries
from pynput.keyboard import Key, Listener, KeyCode  # Libraries for keyboard input
import numpy as np
from dynamixel_workbench_msgs.srv import DynamixelCommand
from sensor_msgs.msg import JointState
import sys


class myKeyboard():
    def __init__(self, dynamixel):
        self.dynamixel = true if dynamixel == 'true' else False
        self.currentID = 1
        self.homePosition = [0, 0, 0, 0, 0]
        self.goalPosition = [45, -30, 30, -70, 60]  # deg
        self.currentPosition = [0, 0, 0, 0, 0]
        self.jointNames = ['waist', 'shoulder', 'elbow', 'wrist', 'gripper']

        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        welcome = """\nControls:
        * w: next joint
        * s: previous joint

        * a: home position
        * d: goal position"""
        rospy.logwarn('You are control the waist')

        rospy.loginfo(welcome)  # Show welcome message
        listener = Listener(on_press=self.onPress)
        listener.start()

        while not rospy.is_shutdown():
            pass
        listener.stop()

    def onPress(self, key):
        print("\033[A")  # Print especial character to erase key pressed
        if not rospy.is_shutdown():
            if key == KeyCode.from_char('w'):
                if self.dynamixel:
                    self.jointCommand(self.currentID, 'LED', 0)

                if self.currentID == 5:
                    self.currentID = 1
                else:
                    self.currentID += 1
                rospy.logwarn('You are control the ' +
                              self.jointNames[self.currentID-1])

                if self.dynamixel:
                    self.jointCommand(self.currentID, 'LED', 1)

            if key == KeyCode.from_char('s'):
                if self.dynamixel:
                    self.jointCommand(self.currentID, 'LED', 0)

                if self.currentID == 1:
                    self.currentID = 5
                else:
                    self.currentID -= 1
                rospy.logwarn('You are control the ' +
                              self.jointNames[self.currentID-1])
                if self.dynamixel:
                    self.jointCommand(self.currentID, 'LED', 1)

            if key == KeyCode.from_char('a'):
                angle = self.homePosition[self.currentID-1]
                if self.dynamixel:
                    self.jointCommand(self.currentID, 'Goal_Position', self.deg2bin(angle))
                else:
                    self.jointStatePublisher(self.currentID-1, angle)

            if key == KeyCode.from_char('d'):
                angle = self.goalPosition[self.currentID-1]
                if self.dynamixel:
                    self.jointCommand(self.currentID, 'Goal_Position', self.deg2bin(angle))
                else:
                    self.jointStatePublisher(self.currentID-1, angle)

    def jointCommand(self, id_num, addr_name, value):
        rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
        try:
            dynamixel_command = rospy.ServiceProxy(
                '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            result = dynamixel_command('', id_num, addr_name, value)
            return result.comm_result
        except rospy.ServiceException as e:
            rospy.logwarn(str(e))

    def deg2bin(self, angle):
        return round(angle/360*4095+2048)

    def jointStatePublisher(self, id, angle):
        self.currentPosition[id] = angle

        msg = JointState()
        msg.header.stamp = rospy.Time().now()
        msg.name = self.jointNames
        msg.position = np.deg2rad(self.currentPosition)

        self.pub.publish(msg)


if __name__ == "__main__":
    # Initialize the node and instantiate the class myKeyboard
    rospy.init_node('TeleopKey', anonymous=False)

    if len([sys.argv[1]]) != 1:
        rospy.logerror('You must enter a boolean value for dynamixel\n' +
                       'Usage: rosrun key_control key_control.py <run_dynamixel>')
        rospy.signal_shutdown('')
    else:
        myKeyboard(sys.argv[1])

#!/usr/bin/env python

#import roslib
#roslib.load_manifest('roboassistant')
import rospy
import Queue
import sys
from geometry_msgs.msg import Twist
from go_to_specific_point_on_map import GoToPose

PICK_UP = 0
DROP_OFF = 2

FRONT_DESK_POS = {'x': -0.272, 'y' : 1.06, 'z': 0.404}
FRONT_DESK_ID = 69

# append values as dictionary in the format
# position = {'x': -0.585, 'y' : 1.29}
# user_request.append((DICT_PICK_UP, PICK_ID, DICT_DROP_OFF, DROP_ID)))
tasks = Queue.Queue()
tasks.put(({'x': -0.585, 'y' : 5.15, 'z': -0.001}, 70, FRONT_DESK_POS, FRONT_DESK_ID))
tasks.put(({'x': -1.81, 'y': -2.44, 'z': 0.2}, 1, FRONT_DESK_POS, FRONT_DESK_ID))

class AI():

    def __init__(self, isDocked=True):
        #this is not required to be changed
        self.quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        self.isDocked = isDocked
        self.navigator = GoToPose()

        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)


    def start(self):
        while True:
            if rospy.is_shutdown():
                sys.exit()
            if not tasks.empty():
                task = tasks.get()

                isFail = False
                for i in [PICK_UP, DROP_OFF]:
                    if isFail:
                        break

                    # Turtlebot docked in charging station
                    if self.isDocked:
                        self.moveBackwards()
                        self.isDocked = False

                    # Execute SLAM navigation to task point
                    rospy.loginfo("Go to (%s, %s) pose", task[i]['x'], task[i]['y'])
                    success = self.navigator.goto(task[i], self.quaternion)

                    if success:
                        rospy.loginfo("Reached the desired pose")
                    else:
                        # put task back to the Queue
                        if i == PICK_UP:
                            tasks.put(task)
                        else:
                            # pickup and dropoff will be the same
                            task = (task[2], task[3], task[2], task[3])
                            tasks.put(task)
                        isFail = True
                        rospy.loginfo("The base failed to reach the desired pose")
                        continue

                    if i == PICK_UP:
                        rospy.loginfo("Picking up!")
                    else:
                        rospy.loginfo("Dropping off!")
                    rospy.sleep(5)

                # Sleep to give the last log messages time to be sent
                rospy.sleep(1)

            rospy.sleep(.1)

    def moveBackwards(self):
        cmd = Twist()
        cmd.linear.x = -0.3
        cmd.linear.y = 0.
        cmd.linear.z = 0.
        cmd.angular.z = 0.
        rospy.loginfo('Turtlebot docked! Moving backwards.')
        for i in range(10):
            self.cmd_vel.publish(cmd)
            rospy.sleep(.1)


if __name__ == '__main__':
    rospy.init_node('turtlebot_ai_node', anonymous=False)
    ai = AI(isDocked=False)
    rospy.sleep(5)

    ai.start()
    rospy.spin()

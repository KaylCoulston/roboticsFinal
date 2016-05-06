#!/usr/bin/env python

import roslib; roslib.load_manifest('roboassistant')
import rospy
import Queue
import sys
import sqlite3
import math
from roboassistant.msg import Action
from geometry_msgs.msg import Twist
from go_to_specific_point_on_map import GoToPose
from move_to_tag_client import MoveCloseToTag
from tf import transformations

LOCATION = 0
ANGLE = 1
PICK_UP = 0
DROP_OFF = 3

FRONT_DESK = 69
DESK_1 = 1
DESK_2 = 12
DESK_3 = 7
DESK_4 = 200

POSITIONS = {FRONT_DESK: ({'x': -0.750, 'y' : -0.102}, 0.),
             DESK_1:     ({'x': -2.030, 'y' : -2.040}, 180.),
             DESK_2:     ({'x': -1.590, 'y' :  4.290}, 180.),
             DESK_3:     ({'x': -1.670, 'y' :  5.720}, 180.),
             DESK_4:     ({'x': -0.567, 'y' :  6.510}, 0.)}

# append values as dictionary in the format
tasks = Queue.Queue()
tasks.put((POSITIONS[DESK_3][LOCATION], DESK_3,
           POSITIONS[FRONT_DESK][LOCATION], FRONT_DESK))
tasks.put((POSITIONS[DESK_1][LOCATION], DESK_1,
           POSITIONS[FRONT_DESK][LOCATION], FRONT_DESK))

class AI():

    def __init__(self, isDocked=True):
        rospy.on_shutdown(self.willShutdown)

        self.conn = sqlite3.connect('src/roboassistant/scripts/tasks.db')
        self.c = self.conn.cursor()

        self.isDocked = isDocked
        self.navigator = GoToPose()
        self.move_to_tag = MoveCloseToTag()
        self.arm_status = ord('E')

        self.move_arm = rospy.Publisher('/start', Action, queue_size=1)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        rospy.Subscriber('/state', Action, self.move_arm_status_cb)

    def move_arm_status_cb(self, data):
        self.arm_status = data.status

    def start(self):
        while True:
            if rospy.is_shutdown():
                sys.exit()

            self.c.execute('SELECT Count(*) FROM tasks')
            if self.c.fetchone()[0] > 0:
                print "Tasks Queue:"
                for row in self.c.execute('SELECT * FROM tasks ORDER BY date_time'):
                    print row[0], str(row[1]), "from", row[2], "to", row[3]

                #task = tasks.get()
                self.c.execute('SELECT * FROM tasks ORDER BY date_time')
                task = self.c.fetchone()
                task_id = task[0]
                task_pickup_id = task[2]
                task_dropoff_id = task[3]
                task = ((POSITIONS[task[2]][LOCATION], task[2], POSITIONS[task[2]][ANGLE],
                         POSITIONS[task[3]][LOCATION], task[3], POSITIONS[task[3]][ANGLE]))

                isFail = False
                for i in [PICK_UP, DROP_OFF]:
                    # Turtlebot docked in charging station
                    if self.isDocked:
                        self.moveBackwards()
                        self.isDocked = False

                    # Execute SLAM navigation to task point
                    rospy.loginfo("Go to (%s, %s) pose", task[i]['x'], task[i]['y'])
                    turtlebot_orientation_in_degrees = task[i+2]
                    quat = transformations.quaternion_from_euler(0, 0, math.radians(turtlebot_orientation_in_degrees))
                    quaternion = {quat[0], quat[1], quat[2], quat[3]}
                    success = self.navigator.goto(task[i], quaternion)

                    if success:
                        rospy.loginfo("Reached the desired pose")
                    else:
                        # put task back to the Queue
                        # TODO: NEED TO FIX LOGIC WHEN IT FAILS
                        isFail = True
                        rospy.loginfo("WARNING: The base failed to reach the desired pose")

                    if i == PICK_UP:
                        rospy.loginfo("Picking up!")
                        move_success = self.move_to_tag.move_to(task_pickup_id)
                        a = Action()
                        a.status = ord('S')
                        a.value1 = 0
                        a.value2 = 0
                        self.move_arm.publish(a)
                        #while self.arm_status != ord('F'):
                        rospy.sleep(2)
                        self.moveBackwards()
                        self.arm_status = ord('E')
                    else:
                        rospy.loginfo("Dropping off!")
                        move_success = self.move_to_tag.move_to(task_dropoff_id)
                        self.moveBackwards()
                        # TODO: ROBOT YELLS
                    rospy.sleep(5)

                # Sleep to give the last log messages time to be sent
                rospy.sleep(1)

                if not isFail:
                    self.c.execute('DELETE FROM tasks WHERE id = %d' % (task_id))
                    self.conn.commit()

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

    def willShutdown(self):
        self.conn.close()
        rospy.loginfo("Shutting down!")

if __name__ == '__main__':
    rospy.init_node('turtlebot_ai_node', anonymous=False)
    ai = AI(isDocked=False)
    rospy.sleep(5)

    ai.start()
    rospy.spin()

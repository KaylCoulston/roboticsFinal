#!/usr/bin/env python

import roslib; roslib.load_manifest('roboassistant')
import rospy
import Queue
import sys
import sqlite3
from geometry_msgs.msg import Twist
from go_to_specific_point_on_map import GoToPose

PICK_UP = 0
DROP_OFF = 2

FRONT_DESK = 69
DESK_1 = 1
DESK_2 = 12
DESK_3 = 7
DESK_4 = 200

POSITIONS = {FRONT_DESK: {'x': -0.272, 'y' : 1.06},
             DESK_1:     {'x': -2.020, 'y' : 0.0984},
             DESK_2:     {'x': -1.590, 'y' : 6.06},
             DESK_3:     {'x': -1.590, 'y' : 7.41},
             DESK_4:     {'x': -0.823, 'y' : 9.46}}

# append values as dictionary in the format
tasks = Queue.Queue()
tasks.put((POSITIONS[DESK_3], DESK_3,
           POSITIONS[FRONT_DESK], FRONT_DESK))
tasks.put((POSITIONS[DESK_1], DESK_1,
           POSITIONS[FRONT_DESK], FRONT_DESK))

class AI():

    def __init__(self, isDocked=True):
        rospy.on_shutdown(self.willShutdown)

        self.conn = sqlite3.connect('tasks.db')
        self.c = self.conn.cursor()

        #this is not required to be changed
        self.quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        self.isDocked = isDocked
        self.navigator = GoToPose()

        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)

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
                task = ((POSITIONS[task[2]], task[2],
                         POSITIONS[task[3]], task[3]))

                isFail = False
                for i in [PICK_UP, DROP_OFF]:
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
                        # TODO: NEED TO FIX LOGIC WHEN IT FAILS
                        isFail = True
                        rospy.loginfo("WARNING: The base failed to reach the desired pose")

                    if i == PICK_UP:
                        rospy.loginfo("Picking up!")
                    else:
                        rospy.loginfo("Dropping off!")
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

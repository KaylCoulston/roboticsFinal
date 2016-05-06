#! /usr/bin/env python

import roslib; roslib.load_manifest('roboassistant')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the FindHuman action, including the
# goal message and the result message.
from roboassistant.msg import FindHumanAction, FindHumanGoal


class FindHuman():
    def __init__(self):
        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        # Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.
        self.client = actionlib.SimpleActionClient('find_human', FindHumanAction)
        rospy.loginfo("Wait for the action server to come up")

        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server(rospy.Duration(5))

    def find(self):
        # Send a goal
        self.goal_sent = True
        # Creates a goal to send to the action server.
        goal = FindHumanGoal(goal_id=1)

        print "Testing"
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        print "testing 2"

        # Waits for the server to finish performing the action.
        self.client.wait_for_result(rospy.Duration(60))
        print "testing 3"
        # Prints out the result of executing the action
        self.goal_sent = False
        return self.client.get_result()  # A FibonacciResult

    def shutdown(self):
        if self.goal_sent:
            self.client.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('find_human_client')
        navigator = FindHuman()
        result = navigator.find()
        print "Result:", "Failed!" if result.completed else "Success!"
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

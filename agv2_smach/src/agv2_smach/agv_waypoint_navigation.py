#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Starting the AGV1 Navigation...")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow AGV1 up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stopping the AGV1")
        rospy.sleep(1)


def dynamic_waypoint(navigator, waypoints, coordinat):
    for i in range(len(waypoints)):
        current_waypoint = coordinat[waypoints[i]]
        position = current_waypoint["position"]
        quaternion = current_waypoint["quaternion"]
        tolerance = 0.05

        rospy.loginfo("Go to desired pose: (%s, %s)", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

    return success, position

if __name__ == '__main__':
    try:
        rospy.init_node('agv1_nav', anonymous=False)
        navigator = GoToPose()

        
        #tolerance = 0.05
        coordinate = dict(rospy.get_param("~Waypoints"))

        print("###"*10)
        print("\n"*5)
        print(coordinate)
        print("\n"*5)
        print("###"*10)
        #waypoints = ["P1", "P2", "P5", "P7", "P9", "P11", "P12", "P13", "P14", "P15", "P16", "P18", "P19", "P1"]
        #waypoints = ["P5", "P12", "P14", "P16", "P19", "P1"]
        #waypoints_test_1 = ["P2", "P5", "P7", "P9", "P11", "P12", "P13", "P14", "P15", "P16", "P18", "P19", "P1"]
        #waypoints_test_2 = ["P2", "P5", "P7", "P8", "P17", "P18", "P19", "P1"]
        waypoints_test_1 = ["P5", "P12", "P14", "P1"]
        
        waypoints_test_2 = ["P5", "P7", "P8", "P17", "P18", "P1"]

        waypoints = waypoints_test_1

        
        success, position = dynamic_waypoint(navigator, waypoints, coordinate)

        if success:
            rospy.loginfo("AGV1 Reached The Desired Pose: (%s, %s)", position['x'], position['y'])
        else:
            rospy.loginfo("AGV1 failed to reach the desired pose!")
        
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Quit")


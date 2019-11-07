#!/usr/bin/env python

from agv2_smach.srv import *
import rospy


def task_plan_func(req):
    req_task = req.request_task
    rospy.init_node('agv_task_plan_service')

    agv_task = list(rospy.get_param('~Tasks/' + req_task))
    print(agv_task)
    return TaskServiceResponse(str(agv_task))


def task_plan():
    rospy.init_node('agv_task_plan_service')
    s = rospy.Service('agv_task_service', TaskService, task_plan_func)
    
    rospy.spin()

if __name__ == "__main__":
    task_plan()

#!/usr/bin/env python 

import roslib
import rospy
import smach
import smach_ros

from smach import State
from smach import StateMachine

import dynamic_reconfigure.client
import agv_waypoint_navigation as agv_nav
from agv2_smach.srv import *

from actionlib import *
from actionlib_msgs.msg import *


class general_selection_state(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['Navigation', 'Docking', 'Parking', 'Idle'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

    def execute(self, userdata):
        print("\n\n")
        print("General Select State")
        print("\n\n")
        
        temp_task_list = userdata.task_input

        if len(temp_task_list) == 0:
            return 'Idle'

        else:
            temp_task = temp_task_list[0]
            userdata.task_output = temp_task_list
            userdata.current_task_output = temp_task

            return temp_task['State']


class idle_standby_state(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

    def execute(self, userdata):

        print(userdata.task_input)

        if len(list(userdata.task_input)) == 0:

            task_name = "Task_1"
            agv_task = self.agv_task_plan_client(task_name)
            userdata.task_output = eval(agv_task)

            userdata.current_task_output = "Wait Task"
            print("\n\n")
            print("Wait Task")
            print("\n\n")
            
            time.sleep(3)


        print("\n\n")
        print("Idle Standby")
        print("\n\n")

        return 'succeeded'

    def agv_task_plan_client(self, request_task):
        rospy.wait_for_service('agv_task_service')

        try:
            # create a handle to the task_plan service
            agv_task_plan = rospy.ServiceProxy('agv_task_service', TaskService)

            # formal style
            response = agv_task_plan.call(TaskServiceRequest(request_task))

            return response.response_task

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


class wy_states(smach.State):    
    def __init__ (self, coordinate_list):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

        self.coordinate = coordinate_list
        
    def execute(self, userdata):
        print(userdata.task_input)

        temp_task_list = list(userdata.task_input)
        temp_task = temp_task_list.pop(0)
        userdata.task_output = temp_task_list

        userdata.current_task_output = temp_task

        waypoint_list = [temp_task["Waypoint"]]
        

        print("\n\nWaypoint - " + str(waypoint_list) + " - is starting to work.\n\n")

        print("Parametre Degisecek")
        client = dynamic_reconfigure.client.Client("agv1_move_base/DWAPlannerROS")
        client.update_configuration({'xy_goal_tolerance': 0.15, 'yaw_goal_tolerance': 0.15})

        client_2 = dynamic_reconfigure.client.Client("agv1_move_base/local_costmap/inflation_layer")
        client_2.update_configuration({'inflation_radius': 0.2})

        print("Parametre Degisti")

        nav = agv_nav.GoToPose()
        success, position = agv_nav.dynamic_waypoint(nav, waypoint_list, self.coordinate)

        time.sleep(3)

        success = True

        if success:
            return 'succeeded'

        else:
            return 'aborted'


class docking_states(smach.State):    
    def __init__ (self, coordinate_list):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

        self.coordinate = coordinate_list
        self.tolerance = 0.05
        
    def execute(self, userdata):
        print(userdata.task_input)

        temp_task_list = list(userdata.task_input)
        temp_task = temp_task_list.pop(0)
        userdata.task_output = temp_task_list

        userdata.current_task_output = temp_task
        

        waypoint = [temp_task["Waypoint"]]
        
        print("\n\nDocking Waypoint - " + str(waypoint) + " - is starting to work.\n\n")
        
        print("Parametre True")
        client = dynamic_reconfigure.client.Client("agv1_move_base/DWAPlannerROS")
        client.update_configuration({'xy_goal_tolerance': self.tolerance, 'yaw_goal_tolerance': self.tolerance})

        client_2 = dynamic_reconfigure.client.Client("agv1_move_base/local_costmap/inflation_layer")
        client_2.update_configuration({'inflation_radius': 0.0})
        print("Parametre Degisti")

        nav = agv_nav.GoToPose()
        success, position = agv_nav.dynamic_waypoint(nav, waypoint, self.coordinate)

        
        time.sleep(3)

        success = True

        if success:
            return 'succeeded'

        else:
            return 'aborted'


class parking_states(smach.State):    
    def __init__ (self, coordinate_list):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])
        #self.waypoint = waypoint
        self.coordinate = coordinate_list
        
    def execute(self, userdata):
        print(userdata.task_input)

        temp_task_list = list(userdata.task_input)
        temp_task = temp_task_list.pop(0)
        userdata.task_output = temp_task_list

        userdata.current_task_output = temp_task

        waypoint = [temp_task["Waypoint"]]
        
        print("\n\nPark - " + str(waypoint) + " - is starting to work.\n\n")
        
        print("Parametre Degisecek")
        client = dynamic_reconfigure.client.Client("agv1_move_base/DWAPlannerROS")
        client.update_configuration({'xy_goal_tolerance': 0.05, 'yaw_goal_tolerance': 0.05})

        client_2 = dynamic_reconfigure.client.Client("agv1_move_base/local_costmap/inflation_layer")
        client_2.update_configuration({'inflation_radius': 0.05})
        print("Parametre Degisti")

        nav = agv_nav.GoToPose()
        success, position = agv_nav.dynamic_waypoint(nav, waypoint, self.coordinate)
        
        time.sleep(3)

        success = True

        if success:
            return 'succeeded'

        else:
            return 'aborted'


class crash_repair_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

    def execute(self, userdata):
        print("\n\n")
        print("Crash Repair")
        print("\n\n")

        time.sleep(3)
        return 'succeeded'


def main():
    rospy.init_node('agv_task_smach')
    
    coordinate = dict(rospy.get_param("~Waypoints"))


    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['done', 'failed'])

    sm_top.userdata.task = list()

    with sm_top:
        
        smach.StateMachine.add('General_Selection', general_selection_state(),
                                transitions={'Navigation':'Navigation','Docking':'Docking', 'Parking':'Parking', 'Idle':'Idle'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Navigation Container
        sm_navigation = smach.StateMachine(outcomes=['Nav_State_1', 'Nav_State_2'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with sm_navigation:

            smach.StateMachine.add('Navigation_Waypoint', wy_states(coordinate),
                                transitions={'succeeded':'Nav_State_1','aborted':'Nav_State_2'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})            
                                
        smach.StateMachine.add('Navigation', sm_navigation,
                                transitions={'Nav_State_1':'General_Selection','Nav_State_2':'Crash'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})
        

        # Create Docking Container
        sm_docking = smach.StateMachine(outcomes=['Docking_State_1', 'Docking_State_2'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with sm_docking:

            smach.StateMachine.add('Docking_Waypoint', docking_states(coordinate),
                                transitions={'succeeded':'Docking_State_1','aborted':'Docking_State_2'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})            

        smach.StateMachine.add('Docking', sm_docking,
                                transitions={'Docking_State_1':'General_Selection','Docking_State_2':'Crash'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Parking Container
        sm_parking = smach.StateMachine(outcomes=['Park_State_1', 'Park_State_2'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with sm_parking:

            smach.StateMachine.add('Parking_Waypoint', parking_states(coordinate),
                                transitions={'succeeded':'Park_State_1','aborted':'Park_State_2'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        smach.StateMachine.add('Parking', sm_parking,
                                transitions={'Park_State_1':'General_Selection','Park_State_2':'Crash'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Idle Container
        sm_idle = smach.StateMachine(outcomes=['Idle_State_1', 'Idle_State_2'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with sm_idle:

            smach.StateMachine.add('Idle_Standby', idle_standby_state(),
                                transitions={'succeeded':'Idle_State_1', 'aborted':'Idle_State_2'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        smach.StateMachine.add('Idle', sm_idle,
                                transitions={'Idle_State_1':'General_Selection','Idle_State_2':'Crash'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        
        # Create Crash Container
        sm_crash = smach.StateMachine(outcomes=['Crash_State_1', 'Crash_State_2'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with sm_crash:

            smach.StateMachine.add('Crash_Repair', crash_repair_state(),
                                transitions={'succeeded':'Crash_State_1', 'aborted':'Crash_State_2'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        smach.StateMachine.add('Crash', sm_crash,
                                transitions={'Crash_State_1':'General_Selection', 'Crash_State_2':'failed'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_TASK_SMACH')
    sis.start()
    sm_top.execute()
    rospy.spin()
    sis.stop()
        

if __name__ == '__main__':
    main()

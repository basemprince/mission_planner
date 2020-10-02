#!/usr/bin/env python

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

## Custom import
import numpy as np
from moveit_msgs.msg import PickupAction, PickupGoal
from moveit_msgs.msg import Grasp
from robotics_project.msg import PickUpPoseAction, PickUpPoseGoal


class StateMachine(object):
    def __init__(self):
        
        self.node_name = "### Student SM"

        ### Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')

        # Custom params
        self.cube_pose = [float(x) for x in rospy.get_param(rospy.get_name() + "/cube_pose").split(', ')]
        self.pickup_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.marker_pose_topic = rospy.get_param('/robotics_intro/manipulation_client/marker_pose_topic')

        ### Subscribe to topics

        # Custom topic subs


        ### Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        rospy.wait_for_service(self.pickup_srv_nm, timeout=30)
        rospy.wait_for_service(self.place_srv_nm, timeout=30)

        ### Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Custom topic pubs
        self.cube_pub = rospy.Publisher('/marker_pose_topic', PoseStamped, queue_size=10)


        ### Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        # Custom action setups
        self.pickup_pose_ac = SimpleActionClient('/pickup_marker_pose', PickUpPoseAction)
        if not self.pickup_pose_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr('%s: Could not connect to pickup pose action server', self.node_name)
            exit()
        rospy.loginfo('%s: Connected to pickup pose action server', self.node_name)


        # Init state machine
        self.state = 1
        rospy.sleep(3)
        self.check_states()


    def check_states(self):

        while not rospy.is_shutdown() and self.state > 0:


            # State:  Tuck arm 
            if self.state == 1:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

                if success_tucking:
                    rospy.loginfo("%s: Arm tuck: ", self.play_motion_ac.get_result())
                    self.state = 2
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = -1

                rospy.sleep(1)


            # State:  Lower robot head service
            if self.state == 2:
            	try:
                    rospy.loginfo("%s: Lowering robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("down")
                    
                    if move_head_req.success == True:
                        self.state = 3
                        rospy.loginfo("%s: Move head down succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Move head down failed!", self.node_name)
                        self.state = -1

                    rospy.sleep(1)
                
                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e


            # State: Pickup Cube
            if self.state == 3:
                rospy.loginfo('%s: Pickup Cube...', self.node_name)

                ## define pose msg
                pose_msg = PoseStamped()

                pose_msg.header.stamp = rospy.Time()
                pose_msg.header.frame_id = 'base_footprint'

                pose_msg.pose.position.x = self.cube_pose[0] + 0.02
                pose_msg.pose.position.y = self.cube_pose[1]
                pose_msg.pose.position.z = self.cube_pose[2] - 0.05
                pose_msg.pose.orientation.x = self.cube_pose[3]
                pose_msg.pose.orientation.y = self.cube_pose[4]
                pose_msg.pose.orientation.z = self.cube_pose[5]
                pose_msg.pose.orientation.w = self.cube_pose[6]

                #pickuppose_goal = PickUpPoseGoal()
                #pickuppose_goal.object_pose = pose_msg

                self.cube_pub.publish(pose_msg)

                pickup_srv = rospy.ServiceProxy(self.pickup_srv_nm, SetBool)
                pickup_req = pickup_srv()

                if pickup_req.success == True:
                    self.state = 4
                    rospy.loginfo("%s: Pickup SRV succeded!", self.node_name)
                else:
                    rospy.loginfo("%s: Pickup SRV failed!", self.node_name)
                    self.state = -1

                #self.pickup_pose_ac.send_goal(pickuppose_goal)
                #success = self.pickup_pose_ac.wait_for_result(rospy.Duration(100.0))

                #if success:
                #    rospy.loginfo('%s: State 0: Pickup Cube was successfull: %s', self.node_name, self.pickup_pose_ac.get_result())
                #else:
                #    rospy.loginfo('%s: State 0: Pickup Cube was NOT successfull', self.node_name)

                # Next state
                #self.state = -2


            # State: Turn the robot
            if self.state == 4:
                rospy.loginfo("%s: Turning", self.node_name)

                move_msg = Twist()
                move_msg.angular.z = -1

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                while not rospy.is_shutdown() and cnt < 30:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 5
                rospy.sleep(1)


            # State: Move to 2nd table
            if self.state == 5:
                rospy.loginfo("%s: Moving towards table", self.node_name)
                move_msg = Twist()
                move_msg.linear.x = 0.5

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                while not rospy.is_shutdown() and cnt < 17:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 6
                rospy.sleep(1)


            # State: Drop the cube
            if self.state == 6:
                rospy.loginfo("%s: Dropping the cube...", self.node_name)

                try:
                    drop_cube = rospy.ServiceProxy(self.place_srv_nm, SetBool)

                    drop_cube_request = drop_cube(True)

                    if drop_cube_request.success == True:
                        rospy.loginfo("%s: Placed cube", self.node_name)
                        self.state = -2
                    else:
                        rospy.loginfo("%s: failed to place the cube", self.node_name)
                        self.state = -1

                    rospy.sleep(1)

                except rospy.ServiceException, e:
                    print "Service call to place server failed: %s"%e




            # State 6: Home
            if self.state == 7:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))
                
                if success_tucking:
                    rospy.loginfo("%s: Arm Tucked.", self.node_name)
                    self.state = -2
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = -1
                rospy.sleep(1)

            # Error handling
            if self.state == -1:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return
'''

import py_trees as pt, py_trees_ros as ptr

class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):

        rospy.loginfo("Initialising behaviour tree")

        # go to door until at door
        b0 = pt.composites.Selector(
            name="Go to door fallback", 
            children=[Counter(30, "At door?"), Go("Go to door!", 1, 0)]
        )

        # tuck the arm
        b1 = TuckArm()

        # go to table
        b2 = pt.composites.Selector(
            name="Go to table fallback",
            children=[Counter(5, "At table?"), Go("Go to table!", 0, -1)]
        )

        # move to chair
        b3 = pt.composites.Selector(
            name="Go to chair fallback",
            children=[Counter(13, "At chair?"), Go("Go to chair!", 1, 0)]
        )

        # lower head
        b4 = LowerHead()

        # become the tree
        tree = pt.composites.Sequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
        super(BehaviourTree, self).__init__(tree)

        # execute the behaviour tree
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): self.tick_tock(1)


class Counter(pt.behaviour.Behaviour):

    def __init__(self, n, name):

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(Counter, self).__init__(name)

    def update(self):

        # count until n
        while self.i <= self.n:

            # increment count
            self.i += 1

            # return failure :(
            return pt.common.Status.FAILURE

        # succeed after counter done :)
        return pt.common.Status.SUCCESS


class Go(pt.behaviour.Behaviour):

    def __init__(self, name, linear, angular):

        # action space
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(Go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING


class TuckArm(pt.behaviour.Behaviour):

    def __init__(self):

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(TuckArm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING
        


class LowerHead(pt.behaviour.Behaviour):

    def __init__(self):

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # execution checker
        self.tried = False
        self.tucked = False

        # become a behaviour
        super(LowerHead, self).__init__("Lower head!")

    def update(self):

        # try to tuck head if haven't already
        if not self.tried:

            # command
            self.move_head_req = self.move_head_srv("down")
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # react to outcome
        else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE

'''
    

if __name__ == "__main__":

    rospy.init_node('main_state_machine')
    try:
        #BehaviourTree()
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


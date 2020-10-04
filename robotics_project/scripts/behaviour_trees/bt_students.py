#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
#from behaviours_student import *
from reactive_sequence import RSequence

from std_msgs.msg import *
from std_srvs.srv import *
from geometry_msgs.msg import PoseStamped, Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf, math

import functools


# remember current cycle (for part A)
start_over_handler = 1

cube_pose = None
def cube_pose_cb(data):
	global cube_pose
	cube_pose = data
	#print('received cube_pose from aruco')

def cube_pose_publish():
	cube_pose = [float(x) for x in rospy.get_param(rospy.get_name() + "/cube_pose").split(', ')]

	cube_pub = rospy.Publisher('/marker_pose_topic', PoseStamped, queue_size=8)

	pose_msg = PoseStamped()
	pose_msg.header.stamp = rospy.Time()
	pose_msg.header.frame_id = 'base_footprint'

	pose_msg.pose.position.x = cube_pose[0]
	pose_msg.pose.position.y = cube_pose[1]
	pose_msg.pose.position.z = cube_pose[2]
	pose_msg.pose.orientation.x = cube_pose[3]
	pose_msg.pose.orientation.y = cube_pose[4]
	pose_msg.pose.orientation.z = cube_pose[5]
	pose_msg.pose.orientation.w = cube_pose[6]

	cube_pub.publish(pose_msg)

nav_vel = Twist()
def nav_vel_cb(data):
	global nav_vel
	nav_vel = data

def get_summed_twist(twist_msg):
	tsum = 0
	tsum += abs(twist_msg.linear.x)
	tsum += abs(twist_msg.linear.y)
	tsum += abs(twist_msg.linear.z)
	tsum += abs(twist_msg.angular.x)
	tsum += abs(twist_msg.angular.y)
	tsum += abs(twist_msg.angular.z)
	return tsum


#########################################


class counter(pt.behaviour.Behaviour):

	"""
	Returns running for n ticks and success thereafter.
	"""

	def __init__(self, n, name):

		rospy.loginfo("Initialising counter behaviour.")

		# counter
		self.i = 0
		self.n = n
		self.done = False
		self.start_over_count = 0

		# become a behaviour
		super(counter, self).__init__(name)

	def update(self):

		if self.start_over_count < start_over_handler:
			self.i = 0
			self.done = False
			self.start_over_count += 1

		# increment i
		self.i += 1

		# succeed after count is done
		if self.i > self.n:
			self.done = True
		
		if self.done:
			return pt.common.Status.SUCCESS
		else:
			return pt.common.Status.FAILURE


class go(pt.behaviour.Behaviour):

	"""
	Returns running and commands a velocity indefinitely.
	"""

	def __init__(self, name, linear, angular):

		rospy.loginfo("Initialising go behaviour.")

		# action space
		#self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_top = "/key_vel"
		#rospy.loginfo(self.cmd_vel_top)
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

		# command
		self.move_msg = Twist()
		self.move_msg.linear.x = linear
		self.move_msg.angular.z = angular

		# become a behaviour
		super(go, self).__init__(name)

	def update(self):

		# send the message
		rate = rospy.Rate(10)
		self.cmd_vel_pub.publish(self.move_msg)
		rate.sleep()

		# tell the tree that you're running
		return pt.common.Status.RUNNING


class tuckarm(pt.behaviour.Behaviour):

	"""
	Sends a goal to the tuck arm action server.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self, motion_name="home"):

		rospy.loginfo("Initialising tuck arm behaviour.")

		# Set up action client
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

		# personal goal setting
		self.goal = PlayMotionGoal()
		self.goal.motion_name = motion_name
		self.goal.skip_planning = True

		# execution checker
		self.sent_goal = False
		self.finished = False
		self.start_over_count = 0

		# become a behaviour
		super(tuckarm, self).__init__("Tuck arm "+motion_name+"!")

	def update(self):

		if self.start_over_count < start_over_handler:
			self.sent_goal = False
			self.finished = False
			self.start_over_count += 1

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

		# if failed
		elif not self.play_motion_ac.get_result():
			return pt.common.Status.FAILURE

		# if I'm still trying :|
		else:
			return pt.common.Status.RUNNING


class movehead(pt.behaviour.Behaviour):

	"""
	Lowers or raisesthe head of the robot.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self, direction):

		rospy.loginfo("Initialising move head behaviour.")

		# server
		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

		# head movement direction; "down" or "up"
		self.direction = direction

		# execution checker
		self.tried = False
		self.done = False
		self.start_over_count = 0

		# become a behaviour
		super(movehead, self).__init__("Head "+direction+"!")

	def update(self):

		if self.start_over_count < start_over_handler:
			self.tried = False
			self.done = False
			self.start_over_count += 1

		# success if done
		if self.done:
			return pt.common.Status.SUCCESS

		# try if not tried
		elif not self.tried:

			# command
			self.move_head_req = self.move_head_srv(self.direction)
			self.tried = True

			# tell the tree you're running
			return pt.common.Status.RUNNING

		# if succesful
		elif self.move_head_req.success:
			self.done = True
			return pt.common.Status.SUCCESS

		# if failed
		elif not self.move_head_req.success:
			return pt.common.Status.FAILURE

		# if still trying
		else:
			return pt.common.Status.RUNNING


#########################################


class detect_cube(pt.behaviour.Behaviour):

	"""
	Sends a goal to the tuck arm action server.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	# get global variable (target of callback)
	global cube_pose

	def __init__(self, param=""):

		rospy.loginfo("Initialising detect cube behaviour.")

		# get param
		self.param = param

		# set subscriber
		self.marker_pose_sub = rospy.Subscriber('/marker_pose_topic', PoseStamped, cube_pose_cb)

		# reset cube pose
		global cube_pose
		cube_pose = None

		# execution checker
		self.first_run = True
		self.finished = False
		self.start_over_count = 0

		# become a behaviour
		super(detect_cube, self).__init__("Detect Cube "+param+"!")


	def update(self):
		global cube_pose, start_over_handler

		if self.first_run or self.start_over_count < start_over_handler:
			cube_pose = None

			self.first_run = False
			self.finished = False

			self.start_over_count += 1

			return pt.common.Status.RUNNING

		if self.param == "once_invert":
			#rospy.sleep(rospy.Duration(1.0))
			if cube_pose == None:
				print('############### Once: Cube NOT Detected!')
				start_over_handler += 1 ## NEW CYCLE
				return pt.common.Status.SUCCESS
			else:
				print('############### Once: Cube Detected!')
				return pt.common.Status.FAILURE

		if self.param == "once":
			#rospy.sleep(rospy.Duration(1.0))
			if cube_pose == None:
				print('############### Once: Cube NOT Detected!')
				return pt.common.Status.FAILURE
			else:
				print('############### Once: Cube Detected!')
				return pt.common.Status.SUCCESS


		elif self.param == "wait_for_detection":
			# already detected cube
			if self.finished:
				#print('############### Finished.')
				return pt.common.Status.SUCCESS

			if cube_pose == None:
				#print('############### Cube None...')
				return pt.common.Status.FAILURE
			
			else:
				#print('############### Cube Detected!')
				self.finished = True
				return pt.common.Status.SUCCESS


class pickplace_cube(pt.behaviour.Behaviour):

	"""
	Returns running and commands a velocity indefinitely.
	"""

	global cube_pose

	def __init__(self, pickplace):

		rospy.loginfo("Initialising pick/place cube behaviour.")

		# setup publisher
		# override /marker_pose_topic when placeing
		#self.cube_pub = rospy.Publisher('/marker_pose_topic', PoseStamped, queue_size=10)

		# setup services
		self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
		rospy.wait_for_service(self.pick_srv_nm, timeout=30)
		rospy.wait_for_service(self.place_srv_nm, timeout=30)
		self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
		self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)

		# pick or place
		self.pickplace = pickplace

		# get default cube position in base_footprint
		#self.cube_pose = [float(x) for x in rospy.get_param(rospy.get_name() + "/cube_pose").split(', ')]

		# execution checker
		self.tried = False
		self.done = False
		self.start_over_count = 0

		# become a behaviour
		super(pickplace_cube, self).__init__("Cube "+pickplace+"ing!")

	def update(self):

		if self.start_over_count < start_over_handler:
			self.done = False
			self.tried = False
			self.start_over_count += 1

		# success if done
		if self.done:
			return pt.common.Status.SUCCESS

		# try if not tried
		elif not self.tried:

			# command
			if self.pickplace == "pick":
				self.pickplace_req = self.pick_srv()
				self.tried = True
			elif self.pickplace == "place":
				cube_pose_publish()
				self.pickplace_req = self.place_srv()
				self.tried = True
			else:
				self.tried = False

			# tell the tree you're running
			return pt.common.Status.RUNNING

		# if succesful
		elif self.pickplace_req.success:
			self.done = True
			return pt.common.Status.SUCCESS

		# if failed
		elif not self.pickplace_req.success:
			return pt.common.Status.FAILURE

		# if still trying
		else:
			if self.pickplace == "place":
				cube_pose_publish()

			return pt.common.Status.RUNNING


class global_localization(pt.behaviour.Behaviour):

	def __init__(self):

		rospy.loginfo("Initialising global localization request behaviour.")


		# setup service
		self.srv_nm = '/global_localization'
		rospy.wait_for_service(self.srv_nm, timeout=30)
		self.global_localization_srv = rospy.ServiceProxy(self.srv_nm, Empty)

		# execution checker
		self.tried = False
		self.done = False
		self.start_over_count = 0

		# become a behaviour
		super(global_localization, self).__init__("global localization!")

	def update(self):

		if self.start_over_count < start_over_handler:
			self.done = False
			self.tried = False
			self.start_over_count += 1

		if not self.tried:
			self.tried = True
			self.global_localization_req = self.global_localization_srv()
			return pt.common.Status.RUNNING

		# if succesful
		else:
			self.done = True
			return pt.common.Status.SUCCESS


class goto(pt.behaviour.Behaviour):

	def __init__(self, tag):

		rospy.loginfo("Initialising goto behaviour.")

		# parameter
		self.tag = tag

		## set messages
		self.table1_msg = PoseStamped()
		#self.table1_msg.header.stamp
		self.table1_msg.header.frame_id = "map"
		self.table1_msg.pose.position.x = -1.0
		self.table1_msg.pose.position.y = -6.0
		self.table1_msg.pose.position.z = 0.0
		self.table1_msg.pose.orientation.x = 0.0
		self.table1_msg.pose.orientation.y = 0.0
		self.table1_msg.pose.orientation.z = -0.7071068
		self.table1_msg.pose.orientation.w = 0.7071068
		self.table2_msg = PoseStamped()
		#self.table2_msg.header.stamp
		self.table2_msg.header.frame_id = "map"
		self.table2_msg.pose.position.x = 0.0
		self.table2_msg.pose.position.y = 0.0
		self.table2_msg.pose.position.z = 0.0
		self.table2_msg.pose.orientation.x = 0.0
		self.table2_msg.pose.orientation.y = 0.0
		self.table2_msg.pose.orientation.z = 0.0
		self.table2_msg.pose.orientation.w = 0.0


		# setup publisher
		self.pub_nm = '/move_base_simple/goal'
		self.goal_pub = rospy.Publisher(self.pub_nm, PoseStamped, queue_size=8)

		# execution checker
		self.tried = False
		self.done = False
		self.start_over_count = 0

		# become a behaviour
		super(goto, self).__init__("goto "+tag+"!")

	def update(self):

		if self.start_over_count < start_over_handler:
			self.done = False
			self.tried = False
			self.start_over_count += 1

		if not self.tried:
			if self.tag == "table1":
				self.goal_pub.publish(self.table1_msg)
			elif self.tag == "table2":
				self.goal_pub.publish(self.table2_msg)

			self.tried = True
			return pt.common.Status.RUNNING

		# if succesful
		else:
			self.done = True
			return pt.common.Status.SUCCESS


class goto_action(pt.behaviour.Behaviour):

	def __init__(self, tag):

		rospy.loginfo("Initialising goto action behaviour.")

		# parameter
		self.tag = tag

		## set messages
		self.table1_msg = PoseStamped()
		#self.table1_msg.header.stamp
		self.table1_msg.header.frame_id = 'map'
		self.table1_msg.pose.position.x = -1.0
		self.table1_msg.pose.position.y = -6.1
		self.table1_msg.pose.position.z = 0.0
		q = tf.transformations.quaternion_from_euler(0.0, 0.0, -90.0 * (math.pi/180))
		self.table1_msg.pose.orientation.x = q[0]
		self.table1_msg.pose.orientation.y = q[1]
		self.table1_msg.pose.orientation.z = q[2]
		self.table1_msg.pose.orientation.w = q[3]
		self.table2_msg = PoseStamped()
		#self.table2_msg.header.stamp
		self.table2_msg.header.frame_id = 'map'
		self.table2_msg.pose.position.x = 2.6
		self.table2_msg.pose.position.y = -1.8
		self.table2_msg.pose.position.z = 0.0
		q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0 * (math.pi/180))
		self.table2_msg.pose.orientation.x = q[0]
		self.table2_msg.pose.orientation.y = q[1]
		self.table2_msg.pose.orientation.z = q[2]
		self.table2_msg.pose.orientation.w = q[3]


		# setup action
		self.goto_ac = SimpleActionClient('/move_base', MoveBaseAction)
		#self.goto_ac.wait_for_server()

		# execution checker
		self.tried = False
		self.done = False
		self.start_over_count = 0

		# become a behaviour
		super(goto_action, self).__init__("goto "+tag+"!")

	def update(self):

		if self.start_over_count < start_over_handler:
			self.done = False
			self.tried = False
			self.start_over_count += 1

		if not self.tried:
			if self.tag == "table1":
				goal = MoveBaseGoal()
				goal.target_pose = self.table1_msg
				goal.target_pose.header.stamp = rospy.Time()
				self.goto_ac.send_goal(goal)

			elif self.tag == "table2":
				goal = MoveBaseGoal()
				goal.target_pose = self.table2_msg
				goal.target_pose.header.stamp = rospy.Time()
				self.goto_ac.send_goal(goal)

			self.tried = True
			return pt.common.Status.RUNNING

		state = self.goto_ac.get_state()
		if state == 3:
			self.done = True

		# if succesful
		if self.done:
			return pt.common.Status.SUCCESS
		else:
			return pt.common.Status.RUNNING


class standstill(pt.behaviour.Behaviour):

	def __init__(self):

		rospy.loginfo("Initialising stand still behaviour.")

		# setup subscriber
		self.navigation_velocity_sub = rospy.Subscriber('/nav_vel', Twist, nav_vel_cb)

		# execution checker
		self.tried = False
		self.done = False
		self.start_over_count = 0

		# become a behaviour
		super(standstill, self).__init__("stand still!")

	def update(self):
		global nav_vel

		if self.start_over_count < start_over_handler:
			self.done = False
			self.tried = False
			self.start_over_count += 1

		if not self.done:
			self.tried = True
			
			if get_summed_twist(nav_vel) > 0:
				return pt.common.Status.FAILURE
			else:
				self.done = True
				return pt.common.Status.SUCCESS

		# if succesful
		else:
			self.done = True
			return pt.common.Status.SUCCESS



### TREE

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")
		

		# tuck the arm
		b0 = tuckarm()


		# raise head
		b01 = movehead("up")


		# request the global position (spawn particles)
		b02 = global_localization()


		# turn in a circle
		b03 = pt.composites.Selector(
			name="Turn 360 fallback",
			children=[counter(62, "Turned?"), go("Turn!", 0, -1)]
		)


		# wait a bit for localization
		b04 = pt.composites.Selector(
			name="Wait fallback",
			children=[counter(20, "Waited?"), go("Wait!", 0, 0)]
		)


		# go to cube
		b05 = goto_action("table1")



		# wait a bit for localization
		b06 = pt.composites.Selector(
			name="Wait fallback",
			children=[counter(20, "Waited?"), go("Wait!", 0, 0)]
		)


		# lower head
		b07 = movehead("down")


		# wait until cube detected
		b08 = detect_cube("wait_for_detection")


		# pick cube
		b09 = pickplace_cube("pick")


		# raise head
		b10 = movehead("up")


		# go to target
		b11 = goto_action("table2")


		# lower head
		b07 = movehead("down")


		# place cube
		b12 = pickplace_cube("place")


		# is cube there? ...



		'''
		# lower head
		b1 = movehead("down")


		# detect cube
		b2 = detect_cube("wait_for_detection")


		# pick up cube when detected
		b3 = pickplace_cube("pick")



		# turn 180
		b4 = pt.composites.Selector(
			name="Turn 180 fallback",
			children=[counter(31, "Turned?"), go("Turn!", 0, -1)]
		)
		# wait a bit
		b5 = pt.composites.Selector(
			name="Wait fallback",
			children=[counter(18, "Waited?"), go("Wait!", 0, 0)]
		)
		# move strait to other table
		b6 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(15, "At table?"), go("Go to table!", 0.5, 0)]
		)
		# go to other table
		b456 = pt.composites.Sequence(
			name="Go to other table sequence",
			children=[b4, b5, b6]
		)



		# place the cube
		b7 = pickplace_cube("place")


		# tuck the arm
		b8 = tuckarm()



		# turn 180
		b42 = pt.composites.Selector(
			name="Turn 180 fallback",
			children=[counter(31, "Turned?"), go("Turn!", 0, -1)]
		)
		# wait a bit
		b52 = pt.composites.Selector(
			name="Wait fallback",
			children=[counter(18, "Waited?"), go("Wait!", 0, 0)]
		)
		# move strait to other table
		b62 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(15, "At table?"), go("Go to table!", 0.5, 0)]
		)
		# go to other table
		b4562 = pt.composites.Sequence(
			name="Go to other table sequence",
			children=[b42, b52, b62]
		)



		# check if cube is placed
		b9 = pt.composites.Selector(
			name="Go to table fallback",
			children=[detect_cube("once"), b4562]
		)


		# be happy
		b10 = pt.composites.Selector(
			name="Happy fallback",
			children=[counter(20, "Happy?"), go("Happy!", 0, 1)]
		)
		'''

		'''

		# go to door until at door
		b0 = pt.composites.Selector(
			name="Go to door fallback", 
			children=[counter(30, "At door?"), go("Go to door!", 1, 0)]
		)

		# tuck the arm
		b1 = tuckarm()

		# go to table
		b2 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(5, "At table?"), go("Go to table!", 0, -1)]
		)

		# move to chair
		b3 = pt.composites.Selector(
			name="Go to chair fallback",
			children=[counter(13, "At chair?"), go("Go to chair!", 1, 0)]
		)

		# lower head
		b4 = movehead("down")

		'''

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b01, b02, b03, b04, b05, b06, b07, b08, b09, b10, b11, b12])
		super(BehaviourTree, self).__init__(tree)


		# https://py-trees.readthedocs.io/en/release-0.6.x/visualisation.html
		def post_tick_handler(snapshot_visitor, behaviour_tree):
			print(pt.display.ascii_tree(behaviour_tree.root,
				snapshot_information=snapshot_visitor))
		snapshot_visitor = pt.visitors.SnapshotVisitor()
		self.add_post_tick_handler(
			functools.partial(post_tick_handler,
							snapshot_visitor))
		self.visitors.append(snapshot_visitor)


		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	


if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()

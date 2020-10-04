#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

from std_msgs.msg import *
from std_srvs.srv import *
from geometry_msgs.msg import PoseStamped
import functools

cube_pose = None
start_over = None

def cube_pose_cb(data):
	global cube_pose
	cube_pose = data

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
		cube_pose = None

		# execution checker
		self.detected = False
		self.finished = False
		self.first = True
		# become a behaviour
		super(detect_cube, self).__init__("Detect Cube "+param+"!")


	def update(self):
		global cube_pose
		global start_over

		if self.first:
			cube_pose = None
			self.first = False
			return pt.common.Status.RUNNING

		elif self.param == "once":
			rospy.sleep(rospy.Duration(1.0))
			if cube_pose == None:
				print('############### Once: Cube NOT Detected!')
				return pt.common.Status.SUCCESS
			else:
				print('############### Once: Cube Detected!')
				start_over = True
				#pt.BehaviourTree.destroy()
				#ptr.trees.BehaviourTree.destroy(self)
				#BehaviourTree()
				return pt.common.Status.FAILURE


		else:
			# already detected cube
			if self.finished and not start_over:
				#print('############### Finished.')
				return pt.common.Status.SUCCESS

			if cube_pose == None:
				start_over = False
				#print('############### Cube None...')
				return pt.common.Status.RUNNING
			else:
				#print('############### Cube Detected!')
				start_over = False
				self.finished = True
				return pt.common.Status.SUCCESS


class pickplace_cube(pt.behaviour.Behaviour):

	"""
	Picks or places a cube when called based on the behaviour string.
	"""

	global cube_pose

	def __init__(self, pickplace):

		rospy.loginfo("Initialising pick/place cube behaviour.")

		# setup publisher
		# override /marker_pose_topic when placeing
		#self.cube_pub = rospy.Publisher('/marker_pose_topic', PoseStamped, queue_size=10)

		# setup services
		self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		rospy.wait_for_service(self.pick_srv_nm, timeout=30)
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
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

		# become a behaviour
		super(pickplace_cube, self).__init__("Cube "+pickplace+"ing!")

	def update(self):

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


### TREE

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")
		

		# tuck the arm
		b0 = tuckarm()


		# lower head
		b1 = movehead("down")


		# detect cube
		b2 = detect_cube()


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
			children=[counter(16, "At table?"), go("Go to table!", 0.5, 0)]
		)

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
			children=[counter(16, "At table?"), go("Go to table!", 0.5, 0)]
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


		# go to other table
		b4562 = pt.composites.Sequence(
			name="Go to other table sequence",
			children=[b42, b52, b62]
		)

		# check if cube is placed
		b9 = pt.composites.Sequence(
			name="Go to table fallback",
			children=[detect_cube("once"), b4562]
		)


		# go back to first table when cube is not detected
		#b10 = 0


		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b456, b7, b8, b9])
		super(BehaviourTree, self).__init__(tree)


		# to visualize the tree
		def post_tick_handler(snapshot_visitor, behaviour_tree):
			print(pt.display.ascii_tree(behaviour_tree.root,snapshot_information=snapshot_visitor))

		
		snapshot_visitor = pt.visitors.SnapshotVisitor()
		self.add_post_tick_handler(functools.partial(post_tick_handler,snapshot_visitor))
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



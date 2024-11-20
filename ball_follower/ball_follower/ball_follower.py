#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from math import *
from dataclasses import dataclass
from typing import List
import csv

@dataclass
class CartesianState:
    timestamp: float
    position: np.ndarray
    velocity: np.ndarray

def invTransform(Transform):
	T = np.matrix(Transform)
	R = T[0:3,0:3]
	t = T[0:3,3]

	inverseT = np.hstack((R.transpose(),-R.transpose().dot(t)))
	inverseT = np.vstack((inverseT,[0,0,0,1]))
	return np.asarray(inverseT)

def transformDHParameter(a,d,alpha,theta):
	T = np.array([
			[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha) ,a*cos(theta)],
			[sin(theta),cos(theta)*cos(alpha) ,-cos(theta)*sin(alpha),a*sin(theta)],
			[0       ,sin(alpha)          ,cos(alpha)          ,d         ],
			[0       ,0                  ,0                  ,1          ]
		])
	return T

def transformRobotParameter(theta):
	d = [0.089159,0,0,0.10915,0.09465,0.0823]
	a = [0,-0.425,-0.39225,0,0,0]
	alpha = [pi/2,0,0,pi/2,-pi/2,0]
	T = np.eye(4)
	for i in range(6):
		T = T.dot(transformDHParameter(a[i],d[i],alpha[i],theta[i]))
	return T

class InverseKinematicsUR5:
	def __init__(self):
		self.d = [0.089159,0,0,0.10915,0.09465,0.0823]
		self.a = [0,-0.425,-0.39225,0,0,0]
		self.alpha = [pi/2,0,0,pi/2,-pi/2,0]
		self.ee_offset = np.eye(4)
		self.limit_max = 2 * pi
		self.limit_min = -2 * pi
		self.joint_weights = np.array([1,1,1,1,1,1])
		self.gd = np.identity(4)
		self.stop_flag = False
		self.theta1 = np.zeros(2)
		self.flags1 = None
		self.theta5 = np.zeros((2,2))
		self.flags5 = None
		self.theta6 = np.zeros((2,2))
		self.theta2 = np.zeros((2,2,2))
		self.theta3 = np.zeros((2,2,2))
		self.flags3 = None
		self.theta4 = np.zeros((2,2,2))

	def normalize(self,value):
		normalized = value
		while normalized > self.limit_max:
			normalized -= 2 * pi
		while normalized < self.limit_min:
			normalized += 2* pi
		return normalized

	def getFlags(self,nominator,denominator):
		if denominator == 0:
			return False
		return abs(nominator/denominator) < 1.01

	def getTheta1(self):
		self.flags1 = np.ones(2)

		p05 = self.gd.dot(np.array([0,0,-self.d[5],1]))-np.array([0,0,0,1])
		psi = atan2(p05[1],p05[0])

		L = sqrt(p05[0]**2+p05[1]**2)
		if abs(self.d[3]) > L:
			self.flags1[:] = self.getFlags(self.d[3],L)
			L = abs(self.d[3])
		phi = acos(self.d[3]/L)

		self.theta1[0] = self.normalize(psi+phi+pi/2)
		self.theta1[1] = self.normalize(psi-phi+pi/2)
		self.stop_flag = not np.any(self.flags1)
	
	def getTheta5(self):
		self.flags5 = np.ones((2,2))

		p06 = self.gd[0:3,3]
		for i in range(2):
			p16z = p06[0]*sin(self.theta1[i])-p06[1]*cos(self.theta1[i]);
			L = self.d[5]

			if abs(p16z - self.d[3]) > L:
				self.flags5[i,:] = self.getFlags(p16z - self.d[3],self.d[5])
				L = abs(p16z-self.d[3]);
			theta5i = acos((p16z-self.d[3])/L)
			self.theta5[i,0] = theta5i
			self.theta5[i,1] = -theta5i
		self.stop_flag = not np.any(self.flags5)

	def getTheta6(self):
		for i in range(2):
			T1 = transformDHParameter(self.a[0],self.d[0],self.alpha[0],self.theta1[i])
			T61 = invTransform(invTransform(T1).dot(self.gd))
			for j in range(2):
				if sin(self.theta5[i,j]) == 0:
					self.theta6[i,j] = 0
				else:
					self.theta6[i,j] = atan2(-T61[1,2]/sin(self.theta5[i,j]),
											  T61[0,2]/sin(self.theta5[i,j]))

	def getTheta23(self):
		self.flags3 = np.ones ((2,2,2))
		for i in range(2):
			T1 = transformDHParameter(self.a[0],self.d[0],self.alpha[0],self.theta1[i])
			T16 = invTransform(T1).dot(self.gd)
			
			for j in range(2):
				T45 = transformDHParameter(self.a[4],self.d[4],self.alpha[4],self.theta5[i,j])
				T56 = transformDHParameter(self.a[5],self.d[5],self.alpha[5],self.theta6[i,j])
				T14 = T16.dot(invTransform(T45.dot(T56)))

				P13 = T14.dot(np.array([0,-self.d[3],0,1]))-np.array([0,0,0,1])
				L = P13.dot(P13.transpose()) - self.a[1]**2 - self.a[2]**2

				if abs(L / (2*self.a[1]*self.a[2]) ) > 1:
					self.flags3[i,j,:] = self.getFlags(L,2*self.a[1]*self.a[2])
					L = np.sign(L) * 2*self.a[1]*self.a[2]
				self.theta3[i,j,0] = acos(L / (2*self.a[1]*self.a[2]) )
				self.theta2[i,j,0] = -atan2(P13[1],-P13[0]) + asin( self.a[2]*sin(self.theta3[i,j,0])/np.linalg.norm(P13) )
				self.theta3[i,j,1] = -self.theta3[i,j,0]
				self.theta2[i,j,1] = -atan2(P13[1],-P13[0]) + asin( self.a[2]*sin(self.theta3[i,j,1])/np.linalg.norm(P13) )
		self.stop_flag = not np.any(self.flags3)
	
	def getTheta4(self):
		for i in range(2):
			T1 = transformDHParameter(self.a[0],self.d[0],self.alpha[0],self.theta1[i])
			T16 = invTransform(T1).dot(self.gd)
			
			for j in range(2):
				T45 = transformDHParameter(self.a[4],self.d[4],self.alpha[4],self.theta5[i,j])
				T56 = transformDHParameter(self.a[5],self.d[5],self.alpha[5],self.theta6[i,j])
				T14 = T16.dot(invTransform(T45.dot(T56)))

				for k in range(2):
					T13 = transformDHParameter(self.a[1],self.d[1],self.alpha[1],self.theta2[i,j,k]).dot(
						  transformDHParameter(self.a[2],self.d[2],self.alpha[2],self.theta3[i,j,k]) )
					T34 = invTransform(T13).dot(T14)
					self.theta4[i,j,k] = atan2(T34[1,0],T34[0,0])

	def countValidSolution(self):
		number_of_solution = 0
		for i in range(2):
			for j in range(2):
				for k in range(2):
					if self.flags1[i] and self.flags3[i,j,k] and self.flags5[i,j]:
						number_of_solution += 1
		return number_of_solution

	def getSolution(self):
		for i in range(4):
			if i == 0:
				self.getTheta1()
			elif i == 1:
				self.getTheta5()
			elif i == 2:
				self.getTheta6()
				self.getTheta23()
			elif i == 3:
				self.getTheta4()
			if self.stop_flag:
				return

	def solveIK(self,forward_kinematics):
		self.gd = forward_kinematics.dot(self.ee_offset)
		self.getSolution()
		number_of_solution = self.countValidSolution()
		if self.stop_flag or number_of_solution < 1:
			return None

		Q = np.zeros((number_of_solution,6))
		index = 0
		for i in range(2):
			for j in range(2):
				for k in range(2):
					if not (self.flags1[i] and self.flags3[i,j,k] and self.flags5[i,j]):
						continue
					Q[index,0] = self.normalize(self.theta1[i])
					Q[index,1] = self.normalize(self.theta2[i,j,k])
					Q[index,2] = self.normalize(self.theta3[i,j,k])
					Q[index,3] = self.normalize(self.theta4[i,j,k])
					Q[index,4] = self.normalize(self.theta5[i,j])
					Q[index,5] = self.normalize(self.theta6[i,j])
					index += 1
		return Q

	def findClosestIK(self,forward_kinematics,current_joint_configuration, logger):
		current_joint = np.array(current_joint_configuration)
		Q = self.solveIK(forward_kinematics)
		logger.info(f"\n{Q.round(4)}\n") 
		if Q is not None:
			delta_Q = np.absolute(Q - current_joint) * self.joint_weights
			delta_Q_weights = np.sum(delta_Q, axis=1)
			closest_ik_index = np.argmin(delta_Q_weights, axis = 0)
			logger.info(f"\n Choosing {Q[closest_ik_index,:].round(4)}\n")
			return Q[closest_ik_index,:]
		else:
			return None

class RobotArmNode(Node):
	def __init__(self, file_path):
		super().__init__('robot_arm')
		self.joint_limits = [(-2 * pi, 2 * pi) for _ in range(6)]
		self.publisher = self.create_publisher(Float64MultiArray, '/ur/arm_controller/commands', 10)
		self.joint_state_sub = self.create_subscription(JointState, '/ur/joint_states', self.joint_state_callback, 10)
		self.current_joint_positions = [0.0] * 6
		self.ur5 = InverseKinematicsUR5()
		self.current_point_index = 0
		self.has_joint_states = False
		self.trajectory = []
		self.get_logger().info("Initializing RobotArmNode")
		self.ik_solution = np.zeros(6)
		if self.read_trajectory_file(file_path):
			self.get_logger().info("Trajectory file loaded")
			self.timer = self.create_timer(0.1, self.timer_callback)
			self.start_time = self.get_clock().now()

	def joint_state_callback(self, msg):
		self.get_logger().debug("Received joint state message")
		joint_indices = {
			'shoulder_pan_joint': 0, 'shoulder_lift_joint': 1, 'elbow_joint': 2,
			'wrist_1_joint': 3, 'wrist_2_joint': 4, 'wrist_3_joint': 5
		}
		for name, position in zip(msg.name, msg.position):
			if name in joint_indices:
				self.current_joint_positions[joint_indices[name]] = position
		self.has_joint_states = True

	def read_trajectory_file(self, file_path):
		try:
			self.get_logger().info(f"Reading trajectory file: {file_path}")
			with open(file_path, 'r') as file:
				reader = csv.reader(file)
				next(reader)
				for row in reader:
					if row:
						values = [float(x) for x in row]
						if len(values) == 7:
							self.trajectory.append(CartesianState(
								timestamp=values[0],
								position=np.array(values[1:4]),
								velocity=np.array(values[4:7])
							))
			self.get_logger().info(f"Trajectory file read successfully, {len(self.trajectory)} points loaded")
			return bool(self.trajectory)
		except Exception as e:
			self.get_logger().error(f"Error reading trajectory file: {e}")
			return False

	def validate_joint_positions(self, joint_positions):
		return True
		if joint_positions is None or len(joint_positions) != 6:
			return False
		return all(
			not (isnan(pos) or isinf(pos)) and min_limit <= pos <= max_limit
			for pos, (min_limit, max_limit) in zip(joint_positions, self.joint_limits)
		)

	def timer_callback(self):
		if not self.has_joint_states:
			return
		if self.current_point_index >= len(self.trajectory):
			self.get_logger().info("Trajectory completed")
			self.timer.cancel()
			return
		point = self.trajectory[self.current_point_index]
		elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
		if elapsed >= point.timestamp:
			if np.any(np.isnan(point.position)) or np.any(np.isnan(point.velocity)):
				self.get_logger().warn(f"Invalid Cartesian data at index {self.current_point_index}")
				self.current_point_index += 1
				return
			
			transformation_matrix = transformRobotParameter(np.concatenate([point.position, np.zeros(3)]))
			self.ik_solution = self.ur5.findClosestIK(np.array(transformation_matrix),self.current_joint_positions,self.get_logger())
			if self.ik_solution is not None:
				if self.validate_joint_positions(self.ik_solution):
					msg = Float64MultiArray()
					msg.data = self.ik_solution.tolist()
					self.publisher.publish(msg)
				else:
					self.get_logger().warn(f"Invalid joint solution at index {self.current_point_index}")
			self.current_point_index += 1

def main():
	rclpy.init()
	node = RobotArmNode('src/parabolic_trajectory.csv')
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
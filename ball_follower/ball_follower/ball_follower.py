#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
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

def tm(yeehaw):
    x, y, z, r, p, yaw = yeehaw
    # Rotation matrix for roll (r), pitch (p), and yaw (yaw)
    R = np.array([
        [np.cos(yaw) * np.cos(p), np.cos(yaw) * np.sin(p) * np.sin(r) - np.sin(yaw) * np.cos(r), np.cos(yaw) * np.sin(p) * np.cos(r) + np.sin(yaw) * np.sin(r)],
        [np.sin(yaw) * np.cos(p), np.sin(yaw) * np.sin(p) * np.sin(r) + np.cos(yaw) * np.cos(r), np.sin(yaw) * np.sin(p) * np.cos(r) - np.cos(yaw) * np.sin(r)],
        [-np.sin(p), np.cos(p) * np.sin(r), np.cos(p) * np.cos(r)]
    ])
    # Transformation matrix: [Rotation | Translation]
    T = np.eye(4)
    T[:3, :3] = R  # Set the rotation part
    T[:3, 3] = [x, y, z]  # Set the translation part
    return T
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
        self.debug = False

        self.workspace_bounds = {
            "x": (-0.5, 0.5),
            "y": (-0.5, 0.5),
            "z": (-5.0, 7.0),
        }

    def enableDebugMode(self, debug = True):
        # This function will enable/disable debug mode
        self.debug = debug

    def setJointLimits(self, limit_min, limit_max):
        # This function is used to set the joint limit for all joint
        self.limit_max = limit_max
        self.limit_min = limit_min

    def setJointWeights(self, weights):
        # This function will assign weights list for each joint
        self.joint_weight = np.array(weights)

    def setEERotationOffset(self,r_offset_3x3):
        # This function will assign rotation offset to the ee. r_offset_3x3 should be a numpy array
        self.ee_offset[0:3,0:3] = r_offset_3x3

    def setEERotationOffsetROS(self):
        # This function will assign proper tool orientation offset for ROS ur5's urdf.
        r_offset_3x3 = np.array( [[ 0, 0, 1],[-1, 0, 0],[ 0,-1, 0]] )
        self.setEERotationOffset(r_offset_3x3)

    def normalize(self,value):
        # This function will normalize the joint values according to the joint limit parameters
        normalized = value
        while normalized > self.limit_max:
            normalized -= 2 * pi
        while normalized < self.limit_min:
            normalized += 2* pi
        return normalized

    def getFlags(self,nominator,denominator):
        # This function is used to check whether the joint value will be valid or not
        if denominator == 0:
            return False
        return abs(nominator/denominator) < 1.01

    def getTheta1(self):
        # This function will solve joint 1
        self.flags1 = np.ones(2)

        p05 = self.gd.dot(np.array([0,0,-self.d[5],1]))-np.array([0,0,0,1])
        psi = atan2(p05[1],p05[0])

        L = sqrt(p05[0]**2+p05[1]**2)

        # gives tolerance if acos just a little bit bigger than 1 to return
        # real result, otherwise the solution will be flagged as invalid
        if abs(self.d[3]) > L:
            if self.debug:
                print('L1 = ', L, ' denominator = ', self.d[3])
            self.flags1[:] = self.getFlags(self.d[3],L) # false if the ratio > 1.001
            L = abs(self.d[3])
        phi = acos(self.d[3]/L)

        self.theta1[0] = self.normalize(psi+phi+pi/2)
        self.theta1[1] = self.normalize(psi-phi+pi/2)

        # stop the program early if no solution is possible
        self.stop_flag = not np.any(self.flags1)
        if self.debug:
            print('t1: ', self.theta1)
            print('flags1: ',self.flags1)
    
    def getTheta5(self):
        # This function will solve joint 5
        self.flags5 = np.ones((2,2))

        p06 = self.gd[0:3,3]
        for i in range(2):
            p16z = p06[0]*sin(self.theta1[i])-p06[1]*cos(self.theta1[i])
            L = self.d[5]

            if abs(p16z - self.d[3]) > L:
                if self.debug:
                    print('L5 = ', L, ' denominator = ', abs(p16z - self.d[3]))
                self.flags5[i,:] = self.getFlags(p16z - self.d[3],self.d[5])
                L = abs(p16z-self.d[3])
            theta5i = acos((p16z-self.d[3])/L)
            self.theta5[i,0] = theta5i
            self.theta5[i,1] = -theta5i

        # stop the program early if no solution is possible
        self.stop_flag = not np.any(self.flags5)
        if self.debug:
            print('t5: ', self.theta5)
            print('flags5: ',self.flags5)

    def getTheta6(self):
        # This function will solve joint 6
        for i in range(2):
            T1 = transformDHParameter(self.a[0],self.d[0],self.alpha[0],self.theta1[i])
            T61 = invTransform(invTransform(T1).dot(self.gd))
            for j in range(2):
                if sin(self.theta5[i,j]) == 0:
                    if self.debug:
                        print("Singular case. selected theta 6 = 0")
                    self.theta6[i,j] = 0
                else:
                    self.theta6[i,j] = atan2(-T61[1,2]/sin(self.theta5[i,j]),
                                              T61[0,2]/sin(self.theta5[i,j]))
        # # print 't6: ', self.theta6

    def getTheta23(self):
        # This function will solve joint 2 and 3
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
                    if self.debug:
                        print('L3 = ', L, ' denominator = ', (2*self.a[1]*self.a[2]))
                    self.flags3[i,j,:] = self.getFlags(L,2*self.a[1]*self.a[2])
                    L = np.sign(L) * 2*self.a[1]*self.a[2]
                self.theta3[i,j,0] = acos(L / (2*self.a[1]*self.a[2]) )
                self.theta2[i,j,0] = -atan2(P13[1],-P13[0]) + asin( self.a[2]*sin(self.theta3[i,j,0])/np.linalg.norm(P13) )
                self.theta3[i,j,1] = -self.theta3[i,j,0]
                self.theta2[i,j,1] = -atan2(P13[1],-P13[0]) + asin( self.a[2]*sin(self.theta3[i,j,1])/np.linalg.norm(P13) )
        if self.debug:
            print('t2: ', self.theta2)
            print('t3: ', self.theta3)
            print('flags3: ',self.flags3)

        # stop the program early if no solution is possible
        self.stop_flag = not np.any(self.flags3)
    
    def getTheta4(self):
        # This function will solve joint 4 value
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
        if self.debug:
            print('t4: ', self.theta4)

    def countValidSolution(self):
        # This function will count the number of available valid solutions
        number_of_solution = 0
        for i in range(2):
            for j in range(2):
                for k in range(2):
                    if self.flags1[i] and self.flags3[i,j,k] and self.flags5[i,j]:
                        number_of_solution += 1
        return number_of_solution

    def getSolution(self):
        # This function will call all function to get all of the joint solutions
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

            # This will stop the solving the IK when there is no valid solution from previous joint calculation
            if self.stop_flag:
                return

    def solveIK(self,forward_kinematics):
        self.gd = forward_kinematics.dot(self.ee_offset)
        if self.debug:
            print('Input to IK:\n', self.gd)
        self.getSolution()
        number_of_solution = self.countValidSolution()
        if self.stop_flag or number_of_solution < 1:
            if self.debug:
                print('No solution')
            return None

        Q = np.zeros((number_of_solution,6))
        index = 0
        for i in range(2):
            for j in range(2):
                for k in range(2):
                    if not (self.flags1[i] and self.flags3[i,j,k] and self.flags5[i,j]):
                        # skip invalid solution
                        continue
                    Q[index,0] = self.normalize(self.theta1[i])
                    Q[index,1] = self.normalize(self.theta2[i,j,k])
                    Q[index,2] = self.normalize(self.theta3[i,j,k])
                    Q[index,3] = self.normalize(self.theta4[i,j,k])
                    Q[index,4] = self.normalize(self.theta5[i,j])
                    Q[index,5] = self.normalize(self.theta6[i,j])
                    index += 1

        if self.debug:
            print('Number of solution: ', number_of_solution)
            print(Q)

        return Q

    def findClosestIK(self,forward_kinematics,current_joint_configuration):
        current_joint = np.array(current_joint_configuration)
        Q = self.solveIK(forward_kinematics)
        if Q is not None:
            delta_Q = np.absolute(Q - current_joint) * self.joint_weights
            delta_Q_weights = np.sum(delta_Q, axis=1)
            closest_ik_index = np.argmin(delta_Q_weights, axis = 0)

            if self.debug:
                print('delta_Q weights for each solutions:', delta_Q_weights)
                print('Closest IK solution: ', Q[closest_ik_index,:])
            return Q[closest_ik_index,:]
        else:
            return None
        
    def compute_jacobian_analytical(self, joint_angles):
        self.a2 = self.a[1]
        self.a3 = self.a[2]
        self.d4 = self.d[3]
        self.d5 = self.d[4]
        theta = joint_angles
        
        # Extract individual angles
        theta1 = theta[0]
        theta2 = theta[1]
        theta3 = theta[2]
        theta4 = theta[3]
        theta5 = theta[4]
        
        # Compute sine and cosine terms
        s1 = np.sin(theta1)
        c1 = np.cos(theta1)
        s2 = np.sin(theta2)
        c2 = np.cos(theta2)
        s5 = np.sin(theta5)
        c5 = np.cos(theta5)
        
        # Compute combined angles
        theta23 = theta2 + theta3
        theta234 = theta23 + theta4
        
        s23 = np.sin(theta23)
        c23 = np.cos(theta23)
        s234 = np.sin(theta234)
        c234 = np.cos(theta234)
        
        # Initialize Jacobian matrix
        J = np.zeros((6, 6))
        
        # Fill in the Jacobian matrix elements
        # Position components (first 3 rows)
        J[0, 0] = self.d4*c1 + s1*(self.a2*s2 + self.a3*s23 + self.d5*s234)
        J[0, 1] = -c1*(self.a2*c2 + self.a3*c23 + self.d5*c234)
        J[0, 2] = -c1*(self.a3*c23 + self.d5*c234)
        J[0, 3] = -self.d5*c1*c234
        J[0, 4] = 0
        J[0, 5] = 0
        
        J[1, 0] = self.d4*s1 - c1*(self.a2*s2 + self.a3*s23 + self.d5*s234)
        J[1, 1] = -s1*(self.a2*c2 + self.a3*c23 + self.d5*c234)
        J[1, 2] = -s1*(self.a3*c23 + self.d5*c234)
        J[1, 3] = -self.d5*s1*c234
        J[1, 4] = 0
        J[1, 5] = 0
        
        J[2, 0] = 0
        J[2, 1] = -(self.a2*s2 + self.a3*s23 + self.d5*s234)
        J[2, 2] = -(self.a3*s23 + self.d5*s234)
        J[2, 3] = -self.d5*s234
        J[2, 4] = 0
        J[2, 5] = 0
        
        # Orientation components (last 3 rows)
        J[3, 0] = 0
        J[3, 1] = s1
        J[3, 2] = s1
        J[3, 3] = s1
        J[3, 4] = -s234*c1
        J[3, 5] = -s1*c5 + c1*c234*s5
        
        J[4, 0] = 0
        J[4, 1] = -c1
        J[4, 2] = -c1
        J[4, 3] = -c1
        J[4, 4] = -s234*s1
        J[4, 5] = -c1*c5 + s1*c234*s5
        
        J[5, 0] = 0
        J[5, 1] = 0
        J[5, 2] = 0
        J[5, 3] = 0
        J[5, 4] = c234
        J[5, 5] = s234*s5
        
        return J
    def compute_pseudo_inverse(self, J):
    # Ensure J is a numpy array
        J = np.array(J)
        JTJ = np.dot(J.T, J)
        JTJ_inv = np.linalg.pinv(JTJ)
        J_pseudo_inv = np.dot(JTJ_inv, J.T)
        return J_pseudo_inv
    def is_within_workspace(self,pose):
        x, y, z = pose
        return ( 
            self.workspace_bounds["x"][0] <= x <= self.workspace_bounds["x"][1] and
            self.workspace_bounds["y"][0] <= y <= self.workspace_bounds["y"][1] and
            self.workspace_bounds["z"][0] <= z <= self.workspace_bounds["z"][1]
        )

class RobotArmNode(Node):
    def __init__(self, file_path):
        super().__init__('robot_arm')
        self.joint_limits = [(-2 * pi, 2 * pi) for _ in range(6)]
        self.publisher = self.create_publisher(Float64MultiArray, '/ur/arm_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/ur/joint_states', self.joint_state_callback, 10)
        self.trajectory_pub = self.create_publisher(Marker, '/end_effector_trajectory', 10)  # New Publisher
        self.current_point_pub = self.create_publisher(Marker, '/current_point', 10)  # New publisher for current point
        self.current_joint_positions = [0.0] * 6
        self.current_joint_positions = [0.0] * 6
        self.current_joint_velocities = [0.0] * 6
        self.ur5 = InverseKinematicsUR5()
        self.current_point_index = 0
        self.has_joint_states = False
        self.trajectory = []
        self.get_logger().info("Initializing RobotArmNode")
        self.ik_solution = np.zeros(6)
        self.counter = 0
        
        if self.read_trajectory_file("/ros2_ws/src/parabolic_trajectory.csv"):
            self.get_logger().info("Trajectory file loaded")
            self.timer = self.create_timer(0.01, self.timer_callback)
            self.start_time = self.get_clock().now()
            self.publish_trajectory()

    def publish_trajectory(self):
        marker = Marker()
        marker.header.frame_id = "world"  # Use the correct frame ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Line strip for trajectory visualization
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Line width
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 0.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0  # Blue

        # Convert trajectory points to geometry_msgs/Point and add them
        for state in self.trajectory:
            point = Point()
            point.x = state.position[0]
            point.y = state.position[1]
            point.z = state.position[2]+2
            marker.points.append(point)

        # Publish the marker
        self.trajectory_pub.publish(marker)
        self.get_logger().info("End-effector trajectory published for visualization")

    def publish_current_point(self):
        # Publish current point marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "current_point"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Make the current point marker larger and red
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Set the position to current trajectory point
        if self.current_point_index < len(self.trajectory):
            current_state = self.trajectory[self.current_point_index]
            marker.pose.position.x = current_state.position[0]
            marker.pose.position.y = current_state.position[1]
            marker.pose.position.z = current_state.position[2]+2

        self.current_point_pub.publish(marker)

    def destroy_node(self):
        # Add your shutdown behavior here
        msg = Float64MultiArray()
        msg.data = np.zeros(6).tolist()
        self.publisher.publish(msg)
        self.get_logger().info('Node is shutting down, performing cleanup...')
        # Cleanup code goes here, like stopping timers, releasing resources, etc.

    def joint_state_callback(self, msg):
        self.get_logger().debug("Received joint state message")
        joint_indices = {
            'shoulder_pan_joint': 0, 'shoulder_lift_joint': 1, 'elbow_joint': 2,
            'wrist_1_joint': 3, 'wrist_2_joint': 4, 'wrist_3_joint': 5
        }
        for name, position in zip(msg.name, msg.position):
            if name in joint_indices:
                self.current_joint_positions[joint_indices[name]] = position
        for name, velocity in zip(msg.name, msg.velocity):
            if name in joint_indices:
                self.current_joint_velocities[joint_indices[name]] = velocity
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
        # return True
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
            msg = Float64MultiArray()
            msg.data = np.zeros(6).tolist()
            self.publisher.publish(msg)
            self.get_logger().info(f"\nCurrent Point Index:{self.current_point_index}")
            return
        self.counter += 1
        if(self.counter>100):
            # self.current_point_index += 1
            self.counter = 000000000000000000000000        
        point = self.trajectory[self.current_point_index]

        self.publish_current_point()
        if not self.ur5.is_within_workspace(point.position):
            print(f"Pose {point.position} is outside the workspace. Computation skipped.")
            self.current_point_index += 1
            return None
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= point.timestamp:
            if np.any(np.isnan(point.position)) or np.any(np.isnan(point.velocity)):
                self.get_logger().warn(f"Invalid Cartesian data at index {self.current_point_index}")
                self.current_point_index += 1
                self.get_logger().info(f"\nCurrents Point Index:{self.current_point_index}")
                return
            temp = np.concatenate([point.position, [0,0,0]])
            transformation_matrix = tm(temp)
            self.ik_solution = self.ur5.findClosestIK(np.array(transformation_matrix),self.current_joint_positions)
        if self.ik_solution is not None:
            if self.validate_joint_positions(self.ik_solution):
                J = self.ur5.compute_jacobian_analytical(self.ik_solution)
                Jv = J[:3, :]
                Jv_pseudo_inverse = self.ur5.compute_pseudo_inverse(Jv)
                velocity_array = np.array(-(point.position - transformRobotParameter(self.current_joint_positions)[0:3,3]))
                joint_velocity = (np.dot(Jv_pseudo_inverse, velocity_array))

                self.get_logger().info(f"\npos:{point.position}")
                self.get_logger().info(f"\ntrans:{transformRobotParameter(self.current_joint_positions)[0:3,3]}")
                msg = Float64MultiArray()
                msg.data = self.ik_solution.tolist()
                self.publisher.publish(msg)

                self.get_logger().info(f"Distance to goal: {np.linalg.norm((point.position - transformRobotParameter(self.current_joint_positions)[0:3,3]))}")
            self.get_logger().info(f"\nCurrent Point Index:{self.current_point_index}")
        else:
            self.get_logger().info(f"Invalid joint solution at index {self.current_point_index}")
            # self.current_point_index += 1
        if(np.linalg.norm((point.position - transformRobotParameter(self.current_joint_positions)[0:3,3]))<0.1):
            self.current_point_index += 1
            self.get_logger().info(f"\nCurrent Point Index:{self.current_point_index}")

def main():
    rclpy.init()
    node = RobotArmNode('src/parabolic_trajectory.csv')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Calls the custom destroy_node method
        rclpy.shutdown()
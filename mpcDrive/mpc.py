import rclpy
from rclpy.node import Node
import numpy as np
import cvxpy as cp
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class MPC(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        self.dt = 0.02

        self.create_subscription(Odometry, 'odom', self.odom_cb, 1)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(self.dt , self.timer_callback)

        self.g_states = np.zeros((10, 1))
        self.g_controls = np.zeros((5, 1))
        self.S = np.array([[1, 0], [0, 1]])
        
        self.N = 5
        self.A = np.array([[1, self.dt], [0, 1]])
        self.B = np.array([[0.5 * self.dt**2], [self.dt]])

        self.x0 = np.array([0, 0])
        self.ref = np.array([10, 0])
        self.R_val = 0.5
        self.L_val = 1

        self.U0 = 0.0
    
    def timer_callback(self):
        v_max = 5
        u_max = 1.0
        self.compute_control(self.x0, self.ref, v_max, u_max)
        self.publish_control()
    
    def compute_control(self, x0, ref, v_max, u_max):
        """
        Solves the MPC optimization problem for the current state.

        Args:
            x0 (np.array): The current state [position, velocity].
            ref (np.array): The reference state [target_position, target_velocity].
            v_max (float): Maximum velocity constraint.
            u_max (float): Maximum acceleration (control) constraint.

        Returns:
            np.array: The optimal control input sequence for the horizon.
        """
        X = cp.Variable((2, self.N + 1))
        U = cp.Variable((1, self.N))

        cost = 0
        R = np.eye(2) * self.R_val
        L = np.eye(1) * self.L_val

        for k in range(self.N):
            cost += cp.quad_form(X[:, k] - ref, R)
            cost += cp.quad_form(U[:, k], L)

        cost += cp.quad_form(X[:, self.N] - ref, R)

        constraints = [X[:, 0] == x0]
        for k in range(self.N):
            constraints += [X[:, k+1] == self.A @ X[:, k] + self.B @ U[:, k]]
            constraints += [cp.abs(X[1, k]) <= v_max] 
            constraints += [cp.abs(U[0, k]) <= u_max]
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        if problem.status not in ["optimal", "optimal_inaccurate"]:
            print(f"Warning: Solver failed with status {problem.status}")
            return np.zeros((1, self.N))
        self.U0 = U.value[0,0]
    
    def publish_control(self):
        msg = Twist()
        msg.linear.x = self.x0[1] + float(self.U0)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def odom_cb(self, msg):
        self.filter_msg(msg)
        print('\n')
        print('Received odometry message' , '\n')
        print('Position: [x: {}, y: {}, z: {}]'.format(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        print('linear velocity: [x: {}, y: {}, z: {}]'.format(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z))
        self.x0 = np.array([msg.pose.pose.position.x, msg.twist.twist.linear.x])

    def filter_msg(self, msg):
        msg.pose.pose.position.x = round(msg.pose.pose.position.x, 2)
        msg.pose.pose.position.y = round(msg.pose.pose.position.y, 2)
        msg.pose.pose.position.z = round(msg.pose.pose.position.z, 2)
        msg.pose.pose.orientation.x = round(msg.pose.pose.orientation.x, 2)
        msg.pose.pose.orientation.y = round(msg.pose.pose.orientation.y, 2)
        msg.pose.pose.orientation.z = round(msg.pose.pose.orientation.z, 2)
        msg.pose.pose.orientation.w = round(msg.pose.pose.orientation.w, 2)
        msg.twist.twist.linear.x = round(msg.twist.twist.linear.x, 2)
        msg.twist.twist.linear.y = round(msg.twist.twist.linear.y, 2)
        msg.twist.twist.linear.z = round(msg.twist.twist.linear.z, 2)
        msg.twist.twist.angular.x = round(msg.twist.twist.angular.x, 2)
        msg.twist.twist.angular.y = round(msg.twist.twist.angular.y, 2)
        msg.twist.twist.angular.z = round(msg.twist.twist.angular.z, 2)
        return msg

def main():
    
    print("""
        ===============================
        MPC CONTROLLER STARTED
        ===============================
        """)
    rclpy.init()
    mpc_cont = MPC()
    rclpy.spin(mpc_cont)
    mpc_cont.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
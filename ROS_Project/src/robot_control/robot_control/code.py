import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class Controller(Node):
    def __init__(self):
        super().__init__("Node")
        self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel",10) 
        self.odom_subs = self.create_subscription(Odometry,"/odom",self.odom_callback,10)
        self.timer = self.create_timer(0.1,self.move_to_goal)
        
        #Target Point's Coordinates
        self.goal_x = 1.0
        self.goal_y = 1.0

        #Starting point's Coordinates and Orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def odom_callback(self,odom_data :Odometry):
        #Coordinates are changing according to odometry datas.
        self.x = odom_data.pose.pose.position.x
        self.y = odom_data.pose.pose.position.y

        #Orientation is changing according to odometry datas.
        q = odom_data.pose.pose.orientation
        siny_cosp = 2*(q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2*(q.y**2 + q.z**2)
        self.theta = math.atan2(siny_cosp,cosy_cosp)

    def move_to_goal(self):
        #Creating Twist data to send publisher which control velocity and angle of turtlebot3
        msg = Twist()

        distance = math.sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)

        if distance > 0.05:
            angle_to_goal = math.atan2(self.goal_y-self.y,self.goal_x-self.x)
            angle_error = angle_to_goal-self.theta

            angle_error = math.atan2(math.sin(angle_error),math.cos(angle_error))

            msg.linear.x = 0.15
            msg.angular.z = 0.5* angle_error

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info("It is reached to the target point.")

        self.cmd_vel_pub.publish(msg)
            

def main(args = None):
    rclpy.init(args = args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math  # สำหรับใช้ Pi

class YawListener(Node):
    def __init__(self):
        super().__init__('yaw_listener')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # หรือ topic อื่นที่คุณต้องการรับข้อมูล
            self.callback,
            10
        )

    def callback(self, data):
        # รับข้อมูล quaternion จาก /odom
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )

        # แปลง quaternion เป็น Euler angles
        euler = euler_from_quaternion(quaternion)

        # ค่า yaw อยู่ที่ index ที่ 2 (เป็น radian)
        yaw_radian = euler[2]

        # แปลง yaw จาก radian เป็น degree
        yaw_degree = yaw_radian * 180.0 / math.pi  # ใช้ค่าของ Pi จาก math

        # ปรับค่ามุม yaw ให้อยู่ในช่วง 0-360 องศา
        if yaw_degree < 0:
            yaw_degree += 360.0  # ถ้าค่ามุมเป็นลบ จะเพิ่ม 360 เพื่อให้เป็นบวก
        
                # ตรวจสอบให้แน่ใจว่าค่ามุมอยู่ในช่วง 0-360 องศา
        if yaw_degree >= 360:
            yaw_degree -= 360  # ถ้ามากกว่า 360 ให้นำออก 360 เพื่อให้ได้ค่าภายในช่วง

        yaw_degree -= 360
        yaw_degree = -yaw_degree

        # แสดงผล yaw ใน degree
        self.get_logger().info(f"Yaw Angle: {yaw_degree:.2f}°")

def main(args=None):
    rclpy.init(args=args)

    yaw_listener = YawListener()

    rclpy.spin(yaw_listener)

    yaw_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

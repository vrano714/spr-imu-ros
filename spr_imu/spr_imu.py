import serial
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Vector3

class SprImu(Node):
    def __init__(self):
        super().__init__("spr_imu")

        self.declare_parameter("acc_range", 2)
        self.acc_range = self.get_parameter("acc_range").get_parameter_value().integer_value
        self.declare_parameter("gyro_range", 125)
        self.gyro_range = self.get_parameter("gyro_range").get_parameter_value().integer_value
        self.declare_parameter("fifo_depth", 4)
        self.fifo_depth = self.get_parameter("fifo_depth").get_parameter_value().integer_value
        self.declare_parameter("sample_rate", 60)
        self.sample_rate = self.get_parameter("sample_rate").get_parameter_value().integer_value

        self.declare_parameter("frame_id", "imu")
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.declare_parameter("imu_topic", "/imu/raw")
        self.pub = self.create_publisher(
            Imu,
            self.get_parameter("imu_topic").get_parameter_value().string_value,
            10
        )
        self.declare_parameter("temperature_topic", "/imu/temperature")
        self.pub2 = self.create_publisher(
            Temperature,
            self.get_parameter("temperature_topic").get_parameter_value().string_value,
            10
        )

        self.declare_parameter("serial_device", "/dev/ttyUSB0")
        self.serial = serial.Serial(
            self.get_parameter("serial_device").get_parameter_value().string_value,
            115200
        )
        # hitting q will quit when already running, otherwise do nothing
        self.serial.write("q\r\n".encode())
        time.sleep(1)
        self.serial.flushInput()
        self.serial.flushOutput()
        self.get_logger().info("ready")

        self.run_imu()

    def run_imu(self):
        cmd = f"pwbimu_logger -s {self.sample_rate} -a {self.acc_range} -g {self.gyro_range} -f {self.fifo_depth}\r\n"
        self.get_logger().info(cmd)
        self.serial.write(cmd.encode())
        #time.sleep(1)
        while True:
            d = self.serial.readline().strip().decode()
            # parse timestamp, temp, gyro, acc in %08x
            sp = d.split(",")
            if len(sp) != 8:
                continue
            imutime = int(sp[0], 16) # imu timestamp -> just hex of unsigned int
            self.get_logger().info(str(imutime))
            # temp [deg celcius], gyro [rad/s], acc[m/s2]
            decoded = [
                imutime,
                *[
                    struct.unpack(">f", bytes.fromhex(x))[0] for x in sp[1:]
                ]
            ]

            p = Imu()
            #print(self.get_clock().now)
            p.header.stamp = self.get_clock().now().to_msg()
            p.header.frame_id = self.frame_id
            gv = Vector3()
            gv.x = decoded[2]
            gv.y = decoded[3]
            gv.z = decoded[4]
            p.angular_velocity = gv
            av = Vector3()
            av.x = decoded[5]
            av.y = decoded[6]
            av.z = decoded[7]
            p.linear_acceleration = av

            t = Temperature()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.frame_id
            t.temperature = decoded[1]

            self.pub.publish(p)
            self.pub2.publish(t)

    # TODO hook shutdown signal and run this
    def shutdown(self):
        self.serial.write("q\r\n".encode())
        self.serial.close()

def main():
    rclpy.init()
    node = SprImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

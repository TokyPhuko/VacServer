import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
import threading
import websocket
import json
from geometry_msgs.msg import Twist
import math

class ESP32ClientNode(Node):
    def __init__(self):
        super().__init__("ESP32ClientNode")

        self.declare_parameter("ip", "192.168.1.247")
        self.declare_parameter("min_lidar_range_m", 0.01)
        self.declare_parameter("max_lidar_range_m", 15.0)

        self.ip = self.get_parameter("ip").value
        self.min_lidar_range_m = self.get_parameter("min_lidar_range_m").value
        self.max_lidar_range_m = self.get_parameter("max_lidar_range_m").value
        self.ws_url = f"ws://{self.ip}:81"
        self._ws_thread = None
        self._ws_connected = False

        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', 1)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 1)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        self._connect_ws()

    def _cmd_vel_callback(self, msg):
        if not self._ws_connected:
            return

        vx = msg.linear.x
        vz = msg.angular.z
        half_base = 0.11
        gain = 270

        left = (vx - vz * half_base) * gain
        right = (vx + vz * half_base) * gain

        left = max(-100, min(100, left))
        right = max(-100, min(100, right))

        cmd = {"l": round(left), "r": round(right)}
        try:
            self._ws.send(json.dumps(cmd))
            self.get_logger().info(f"cmd_vel: vx={vx:.2f} vz={vz:.2f} => l={round(left)} r={round(right)}")
        except Exception as e:
            self.get_logger().error(f"cmd_vel send error: {e}", throttle_duration_sec=2.0)

    def _connect_ws(self):
        self.get_logger().info(f"Подключение к WebSocket: {self.ws_url}")
        self._ws = websocket.WebSocketApp(
            self.ws_url,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
            on_open=self._on_open,
        )
        self._ws_thread = threading.Thread(target=self._ws.run_forever, daemon=True)
        self._ws_thread.start()

    def _on_message(self, ws, message):
        try:
            data = json.loads(message)
            now = self.get_clock().now().to_msg()

            yaw = data.get('currentDeg', 0.0);
            self.get_logger().info(f"yaw={yaw}, x={data.get('x', '?')}, y={data.get('y', '?')}, has_currentDeg={'currentDeg' in data}, keys={list(data.keys())[:10]}")

            raw_points = data.get('points', [])
            NUM_BINS = 360
            ranges = [0.0] * NUM_BINS

            for p in raw_points:
                if p.get('qua', 0.0) == 0 or p.get('len', 0.0) == 0:
                    continue

                deg = p.get('deg', 0.0)
                dist_m = p.get('len', 0.0) / 1000.0

                if dist_m > self.max_lidar_range_m or dist_m < self.min_lidar_range_m:
                    continue

                bin_idx = (180 - int(round(deg))) % NUM_BINS
                if ranges[bin_idx] == 0.0 or dist_m < ranges[bin_idx]:
                    ranges[bin_idx] = dist_m

            scan_msg = LaserScan()
            scan_msg.header.stamp = now
            scan_msg.header.frame_id = 'laser_link'
            scan_msg.angle_min = -math.pi
            scan_msg.angle_increment = 2.0 * math.pi / NUM_BINS
            scan_msg.angle_max = math.pi
            scan_msg.range_min = self.min_lidar_range_m
            scan_msg.range_max = self.max_lidar_range_m
            scan_msg.ranges = ranges
            self.lidar_publisher.publish(scan_msg)

            imu_data = data.get('imu', None)
            if imu_data:
                imu_msg = Imu()
                imu_msg.header.stamp = now
                imu_msg.header.frame_id = 'imu_link'
                imu_msg.angular_velocity.x = imu_data.get('gx', 0.0)
                imu_msg.angular_velocity.y = imu_data.get('gy', 0.0)
                imu_msg.angular_velocity.z = imu_data.get('gz', 0.0)
                imu_msg.linear_acceleration.x = imu_data.get('ax', 0.0)
                imu_msg.linear_acceleration.y = imu_data.get('ay', 0.0)
                imu_msg.linear_acceleration.z = imu_data.get('az', 0.0)
                imu_msg.orientation_covariance[0] = -1.0 # скип ориентации
                self.imu_publisher.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки сообщения: {e}")

    def _on_error(self, ws, error):
        self.get_logger().error(f"WebSocket ошибка: {error}")

    def _on_close(self, ws, close_status_code, close_msg):
            self._ws_connected = False
            self.get_logger().warn(f"WebSocket закрыт (code={close_status_code}), переподключение через 2с...")
            import time
            time.sleep(2)
            self._connect_ws() 

    def _on_open(self, ws):
        self._ws_connected = True
        self.get_logger().info("WebSocket подключён")

def main(args=None):
    rclpy.init()
    node = ESP32ClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
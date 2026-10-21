"""CSV recorder (sim time): imu yaw, odom pose, cmd_vel_raw, cmd_vel, scan front-cone min."""
import math, sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
def yaw_of(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
class R(Node):
    def __init__(s, out):
        super().__init__('entry_test_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
        s.f = {k: open(f'{out}/{k}.csv', 'w') for k in ('imu', 'odom', 'cmd_raw', 'cmd', 'scan')}
        s.f['imu'].write('t,yaw\n'); s.f['odom'].write('t,x,y,yaw\n'); s.f['cmd_raw'].write('t,lin,ang\n')
        s.f['cmd'].write('t,lin,ang\n'); s.f['scan'].write('t,cone30_min,rear30_min\n')
        s.last_imu = -1
        s.create_subscription(Imu, '/imu', s.imu, qos_profile_sensor_data)
        s.create_subscription(Odometry, '/odom', s.odom, qos_profile_sensor_data)
        s.create_subscription(TwistStamped, '/cmd_vel_raw', lambda m: s.cmd('cmd_raw', m), 10)
        s.create_subscription(TwistStamped, '/cmd_vel', lambda m: s.cmd('cmd', m), 10)
        s.create_subscription(LaserScan, '/scan', s.scan, qos_profile_sensor_data)
    def now(s): return s.get_clock().now().nanoseconds * 1e-9
    def imu(s, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - s.last_imu >= 0.02:
            s.last_imu = t; s.f['imu'].write('%.4f,%.6f\n' % (t, yaw_of(m.orientation))); s.f['imu'].flush()
    def odom(s, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; p = m.pose.pose
        s.f['odom'].write('%.4f,%.4f,%.4f,%.6f\n' % (t, p.position.x, p.position.y, yaw_of(p.orientation))); s.f['odom'].flush()
    def cmd(s, k, m):
        s.f[k].write('%.4f,%.4f,%.4f\n' % (s.now(), m.twist.linear.x, m.twist.angular.z)); s.f[k].flush()
    def scan(s, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        def cone(c):
            v = [r for i, r in enumerate(m.ranges) if math.isfinite(r) and m.range_min <= r <= m.range_max and
                 abs(((math.degrees(m.angle_min + i * m.angle_increment) - c + 180) % 360) - 180) <= 30.0]
            return min(v) if v else float('inf')
        s.f['scan'].write('%.4f,%.4f,%.4f\n' % (t, cone(0.0), cone(180.0))); s.f['scan'].flush()
rclpy.init(); n = R(sys.argv[1])
try:
    rclpy.spin(n)
except Exception:
    pass

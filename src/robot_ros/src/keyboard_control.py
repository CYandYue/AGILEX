#!/usr/bin/env python3
"""
键盘控制底盘节点
  W - 以 LINEAR_SPEED m/s 前进 FORWARD_DURATION 秒
  S - 以 LINEAR_SPEED m/s 后退 FORWARD_DURATION 秒
  A - 原地左转 ROTATE_DURATION 秒
  D - 原地右转 ROTATE_DURATION 秒
  Space - 立即停止
  Q - 退出
"""

import sys
import tty
import termios
import threading
import rospy
from geometry_msgs.msg import Twist

# ============================================================
# 参数配置（根据需要修改）
# ============================================================
LINEAR_SPEED      = 0.3   # 前进/后退线速度 (m/s)
ANGULAR_SPEED     = 0.5   # 原地旋转角速度 (rad/s)
FORWARD_DURATION  = 2.0   # W/S 持续时间 (秒)
ROTATE_DURATION   = 2.0   # A/D 持续时间 (秒)
CMD_VEL_TOPIC     = "cmd_vel"
PUBLISH_RATE      = 20    # Hz
# ============================================================

KEY_MAP = {
    'w': ( LINEAR_SPEED,  0.0),
    's': (-LINEAR_SPEED,  0.0),
    'a': ( 0.0,  ANGULAR_SPEED),
    'd': ( 0.0, -ANGULAR_SPEED),
}


def get_key():
    """读取单个按键（非阻塞终端模式）"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    if ch == '\x03':   # Ctrl+C
        raise KeyboardInterrupt
    return ch.lower()


def make_twist(linear, angular):
    msg = Twist()
    msg.linear.x  = linear
    msg.angular.z = angular
    return msg


def run_motion(pub, linear, angular, duration, rate):
    """发布运动指令持续 duration 秒，然后停止"""
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        pub.publish(make_twist(linear, angular))
        rate.sleep()
    pub.publish(make_twist(0.0, 0.0))


def print_help():
    print("\n===== 键盘控制底盘 =====")
    print(f"  W  前进 {FORWARD_DURATION}s @ {LINEAR_SPEED} m/s")
    print(f"  S  后退 {FORWARD_DURATION}s @ {LINEAR_SPEED} m/s")
    print(f"  A  左转 {ROTATE_DURATION}s @ {ANGULAR_SPEED} rad/s")
    print(f"  D  右转 {ROTATE_DURATION}s @ {ANGULAR_SPEED} rad/s")
    print("  Space  急停")
    print("  Q  退出")
    print("========================\n")


def main():
    rospy.init_node("keyboard_control", anonymous=True)
    pub  = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
    rate = rospy.Rate(PUBLISH_RATE)

    print_help()
    print(">>> 等待按键输入（按键后生效，Ctrl+C 退出）")
    sys.stdout.flush()

    motion_thread = None

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'q':
            pub.publish(make_twist(0.0, 0.0))
            rospy.loginfo("退出键盘控制")
            break

        if key == ' ':
            # 急停：如果有运动线程，等它被 rospy.is_shutdown 或超时终止
            pub.publish(make_twist(0.0, 0.0))
            rospy.loginfo("急停")
            continue

        if key in KEY_MAP:
            linear, angular = KEY_MAP[key]
            duration = FORWARD_DURATION if linear != 0.0 else ROTATE_DURATION

            # 上一个动作还在跑时，先发停止再启动新动作
            if motion_thread and motion_thread.is_alive():
                pub.publish(make_twist(0.0, 0.0))
                rospy.sleep(0.05)

            label = {'w':'前进','s':'后退','a':'左转','d':'右转'}[key]
            rospy.loginfo(f"{label} {duration}s")

            motion_thread = threading.Thread(
                target=run_motion,
                args=(pub, linear, angular, duration, rate),
                daemon=True,
            )
            motion_thread.start()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

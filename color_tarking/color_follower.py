#!/usr/bin/env python3
import os
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum, auto

# ==========================================
# [GLOBAL STANDARDS & CONSTANTS] - v5.16
# ==========================================
BLUE_LOWER   = np.array([100, 150,  50])
BLUE_UPPER   = np.array([140, 255, 255])
# 노란색 필터 강화 (진짜 샛노란색만 추출)
YELLOW_LOWER = np.array([ 25, 180, 180]) 
YELLOW_UPPER = np.array([ 35, 255, 255])
GREEN_LOWER  = np.array([ 40, 100,  50])
GREEN_UPPER  = np.array([ 80, 255, 255])

# Perception Settings
SCAN_ROI_Y    = 0.70  # 하단 30% 영역 (이 위로는 노란색 무시)
TRIGGER_ROI_Y = 0.95  # 맨 바닥 5% 영역 (회전축 정렬용)
MIN_AREA_YELLOW = 5000 # 유효 면적
JUNCTION_DEBOUNCE = 3 
CONTROL_HZ    = 20    

# Control Constants
LIN_VEL      = 0.12   
TURN_VEL     = 0.8    
KP           = 0.005
COOLDOWN_M   = 0.30   

class State(Enum):
    IDLE            = auto()
    FOLLOWING       = auto()
    JUNCTION_ENTRY  = auto() # 노란색 밟는 중
    TURNING         = auto()
    POST_TURN_CLEAR = auto()
    SEARCHING       = auto()
    DOCKING         = auto()
    ARRIVED         = auto()

class ColorFollower(Node):
    def __init__(self):
        super().__init__('color_follower')

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image,    '/camera/image_raw',    self.image_callback, qos)
        self.create_subscription(Odometry, '/odom',                self.odom_callback,  qos)
        self.create_subscription(String,   '/color_follower/goal', self.goal_callback,  10)

        self.pub_cmd    = self.create_publisher(Twist,  '/cmd_vel',                   10)
        self.pub_debug  = self.create_publisher(Image,  '/color_follower/debug_view', 10)
        self.pub_status = self.create_publisher(String, '/color_follower/status',     10)

        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)

        self.bridge = CvBridge()
        self.state  = State.IDLE
        self.odom_initialized = False
        
        self.percept = {
            'blue_cx': None, 
            'yellow_visible': False,    # 하단 30% 영역 감지
            'yellow_at_axle': False,    # 맨 바닥 5% 영역 감지
            'green_cx': None, 'green_area': 0
        }
        
        self.odom_x, self.odom_y, self.current_yaw = 0.0, 0.0, 0.0
        self.last_junc_pos = (-99.0, -99.0)
        self.state_start_pos = (0.0, 0.0)
        self.state_start_time = self.get_clock().now()
        self.target_yaw = 0.0
        
        self.yellow_seen_frames = 0 
        self.passed_trigger_zone = False 
        
        self.active_goal = None
        self.turn_queue = []
        self.current_turn = 'S'

        self.declare_parameter('routes_file', '/mnt/c/turtle/src/color_tracking_navigation/goal_routes.yaml')
        self._load_routes()

        self.get_logger().info('=== [v5.16] Pure Visual Edge Alignment Online ===')

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = cv_img.shape[:2]
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            
            # 1. Blue Line (Lower 60% ROI)
            blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
            blue_mask[0:int(h*0.4), :] = 0
            M_blue = cv2.moments(blue_mask)
            self.percept['blue_cx'] = int(M_blue['m10']/M_blue['m00']) if M_blue['m00'] > 2000 else None
            
            # 2. Yellow Junction (Two-stage Edge Analysis)
            yellow_mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
            
            # [Detection] 하단 30% 영역만 감시 (멀리 있는 타일 완전 무시)
            scan_roi_y = int(h * SCAN_ROI_Y)
            scan_roi = yellow_mask[scan_roi_y:h, :]
            self.percept['yellow_visible'] = (cv2.moments(scan_roi)['m00'] > MIN_AREA_YELLOW)
            
            # [Trigger] 맨 바닥 5% 영역 (로봇 회전축 정렬 확인용)
            trigger_roi_y = int(h * TRIGGER_ROI_Y)
            trigger_roi = yellow_mask[trigger_roi_y:h, :]
            self.percept['yellow_at_axle'] = (cv2.moments(trigger_roi)['m00'] > 500)

            # 3. Green Marker
            green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
            Mg = cv2.moments(green_mask)
            self.percept['green_area'] = int(Mg['m00'])
            self.percept['green_cx'] = int(Mg['m10']/Mg['m00']) if Mg['m00'] > 5000 else None

            self._publish_debug(cv_img, h, w, scan_roi_y, trigger_roi_y)
        except Exception as e:
            self.get_logger().error(f'Perception Error: {e}')

    def control_loop(self):
        if not self.odom_initialized or self.state == State.IDLE: return

        twist = Twist()
        dist_from_state = math.hypot(self.odom_x - self.state_start_pos[0], self.odom_y - self.state_start_pos[1])
        dt = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

        if self.state == State.FOLLOWING:
            dist_from_last_junc = math.hypot(self.odom_x - self.last_junc_pos[0], self.odom_y - self.last_junc_pos[1])
            
            # 하단 30% 영역에 노란색이 나타나면 교차로 모드 진입
            if self.percept['yellow_visible'] and dist_from_last_junc > COOLDOWN_M:
                self.yellow_seen_frames += 1
            else: self.yellow_seen_frames = 0

            if self.yellow_seen_frames >= JUNCTION_DEBOUNCE:
                self.current_turn = self.turn_queue.pop(0) if self.turn_queue else 'S'
                self.get_logger().info(f'!!! JUNCTION REACHED: {self.current_turn} !!!')
                self.last_junc_pos = (self.odom_x, self.odom_y)
                self.enter_state(State.JUNCTION_ENTRY)
            
            elif not self.turn_queue and self.percept['green_area'] > 1000000 and dt > 3.0:
                self.enter_state(State.DOCKING)
            else:
                if not self._apply_p_control(twist): twist.linear.x = 0.05 

        elif self.state == State.JUNCTION_ENTRY:
            # 5cm/s로 기어가며 노란색 타일이 '발밑을 통과'하길 기다림
            twist.linear.x = 0.05
            self._apply_p_control(twist)
            
            # 1. 먼저 타일이 맨 바닥 5% 영역(Trigger ROI)을 밟아야 함
            if self.percept['yellow_at_axle']:
                self.passed_trigger_zone = True 
            
            # 2. 밟았던 타일이 화면 하단 밖으로 '완전히 사라지는 찰나'에 회전 트리거
            # 이것이 로봇의 바퀴축이 타일 중앙에 왔다는 시각적 증거
            if self.passed_trigger_zone and not self.percept['yellow_at_axle']:
                self.get_logger().warn('Visual Alignment Complete. Start Turn.')
                self.enter_state(State.TURNING)
            
            # 안전장치 (시각 신호 놓쳤을 때 대비)
            if dist_from_state > 0.20:
                self.get_logger().error('Visual Trigger Missed. Emergency Turn.')
                self.enter_state(State.TURNING)

        elif self.state == State.TURNING:
            if self.current_turn == 'S':
                self._apply_p_control(twist)
                if dist_from_state > 0.25: self.enter_state(State.POST_TURN_CLEAR)
            else:
                yaw_err = self._wrap_angle(self.target_yaw - self.current_yaw)
                if abs(yaw_err) < math.radians(3.0):
                    self.enter_state(State.POST_TURN_CLEAR)
                else:
                    twist.angular.z = max(-TURN_VEL, min(TURN_VEL, 2.5 * yaw_err))
                # 정밀 라인 캐치
                if abs(yaw_err) < math.radians(15.0) and self.percept['blue_cx'] is not None:
                    if abs(self.percept['blue_cx'] - 320) < 20:
                        self.enter_state(State.FOLLOWING)
            if dt > 8.0: self.enter_state(State.POST_TURN_CLEAR)

        elif self.state == State.POST_TURN_CLEAR:
            twist.linear.x = 0.10
            if not self._apply_p_control(twist):
                sign = 1.0 if self.current_turn == 'L' else -1.0
                if self.current_turn != 'S': twist.angular.z = sign * 0.1
            if dist_from_state > 0.15: self.enter_state(State.SEARCHING)

        elif self.state == State.SEARCHING:
            twist.linear.x = 0.05
            if self.percept['blue_cx'] is not None:
                if abs(self.percept['blue_cx'] - 320) < 80:
                    self.enter_state(State.FOLLOWING)
            else:
                sign = 1.0 if self.current_turn == 'L' else -1.0
                if self.current_turn != 'S': twist.angular.z = sign * 0.15

        elif self.state == State.DOCKING:
            twist.linear.x = 0.06
            if self.percept['green_cx'] is not None:
                err = self.percept['green_cx'] - 320
                twist.angular.z = -float(err) * KP
            if self.percept['green_area'] > 8000000:
                self.enter_state(State.ARRIVED)

        elif self.state == State.ARRIVED:
            twist.linear.x, twist.angular.z = 0.0, 0.0

        self.pub_cmd.publish(twist)

    def _apply_p_control(self, twist, v=None):
        if self.percept['blue_cx'] is not None:
            err = self.percept['blue_cx'] - 320
            twist.linear.x = v if v is not None else LIN_VEL
            twist.angular.z = -float(err) * KP
            return True
        return False

    def enter_state(self, new_state):
        if self.state == new_state: return
        self.get_logger().info(f'[{self.state.name} -> {new_state.name}]')
        
        self.yellow_seen_frames = 0 
        self.passed_trigger_zone = False 
        
        self.state = new_state
        self.state_start_pos = (self.odom_x, self.odom_y)
        self.state_start_time = self.get_clock().now()
        if new_state == State.TURNING:
            sign = {'L': 1.0, 'R': -1.0}.get(self.current_turn, 0.0)
            self.target_yaw = self._wrap_angle(self.current_yaw + sign * math.pi/2.0)
        self.pub_status.publish(String(data=new_state.name))

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
        if not self.odom_initialized:
            self.get_logger().info('Odometry Ready.')
            self.odom_initialized = True

    def goal_callback(self, msg):
        goal = msg.data.strip()
        if goal not in self.routes: return
        self.active_goal = goal
        self.turn_queue = [t.strip().upper() for t in self.routes[goal].split(',') if t.strip()]
        self.get_logger().info(f'Mission Start: {goal} | Path: {self.turn_queue}')
        self.last_junc_pos = (-99.0, -99.0)
        self.enter_state(State.FOLLOWING)

    def _wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def _load_routes(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        path = self.get_parameter('routes_file').value
        if os.path.exists(path):
            with open(path) as f: self.routes = yaml.safe_load(f).get('routes', {})
        else: self.routes = {}

    def _publish_debug(self, img, h, w, scan_y, trigger_y):
        debug = img.copy()
        cv2.line(debug, (0, scan_y), (w, scan_y), (255, 255, 0), 1) # Scan line
        cv2.line(debug, (0, trigger_y), (w, trigger_y), (0, 255, 255), 2) # Axle line
        label = f'STATE: {self.state.name} CMD: {self.current_turn}'
        cv2.putText(debug, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = ColorFollower()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()

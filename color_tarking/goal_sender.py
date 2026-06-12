import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class GoalSender(Node):
    def __init__(self, routes_path):
        super().__init__('goal_sender')
        self.pub = self.create_publisher(String, '/color_follower/goal', 10)
        self.create_subscription(String, '/color_follower/status', self.status_callback, 10)
        self.current_status = 'IDLE'
        self.routes_path = routes_path
        self.known_goals = self.load_known_goals()

    def load_known_goals(self):
        try:
            if os.path.exists(self.routes_path):
                with open(self.routes_path) as f:
                    data = yaml.safe_load(f)
                    goals = list(data.get('routes', {}).keys())
                    return goals
            return []
        except Exception:
            return []

    def status_callback(self, msg):
        self.current_status = msg.data

    def send(self, goal: str):
        msg = String()
        msg.data = goal
        self.pub.publish(msg)

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_path = os.path.join(script_dir, 'goal_routes.yaml')
    
    routes_file = sys.argv[1] if len(sys.argv) > 1 else default_path
    
    rclpy.init()
    node = GoalSender(routes_file)

    # Spin in a separate thread for status updates
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    print('\n=== Color Tracking Goal Sender (4 Destinations) ===')
    if node.known_goals:
        print(f'📍 가능한 목적지: {", ".join(node.known_goals)}')
    else:
        print('⚠️  주의: goal_routes.yaml을 찾을 수 없습니다.')
    print('종료: q\n')

    try:
        while rclpy.ok():
            goal = input(f'[{node.current_status}] 목적지 입력 > ').strip()
            if not goal: continue
            if goal.lower() == 'q': break
            
            if node.known_goals and goal not in node.known_goals:
                print(f'❌ 알 수 없는 목적지: {goal}')
                continue
            
            node.send(goal)
            print(f'✅ 전송됨: {goal}')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

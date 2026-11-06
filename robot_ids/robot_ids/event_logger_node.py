"""Event logger: a readable summary of what the monitor detected.

``/ids/events`` repeats every active detector once per second, which is
hard to read in ``ros2 topic echo``. This node prints only the first event
from each detector (with the time since the alarm was last cleared) and,
optionally, writes every event to a CSV file for later analysis.

  ros2 run robot_ids event_logger
  ros2 run robot_ids event_logger --ros-args -p csv_path:=~/ids_events.csv
"""

import csv
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

CSV_FIELDS = ['t', 'detector', 'topic', 'message', 'score']


class EventLoggerNode(Node):

    def __init__(self):
        super().__init__('ids_event_logger')
        self.declare_parameter('csv_path', '')
        self.seen = set()
        self.csv_file = None
        self.writer = None
        path = self.get_parameter('csv_path').value
        if path:
            path = os.path.expanduser(path)
            self.csv_file = open(path, 'a', newline='')
            self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
            if self.csv_file.tell() == 0:
                self.writer.writeheader()
            self.get_logger().info(f'Writing events to {path}')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/ids/attack_detected', self.on_alarm, latched)
        self.create_subscription(String, '/ids/events', self.on_event, 50)

    def on_alarm(self, msg):
        if not msg.data:
            self.seen.clear()
            self.get_logger().info('--- alarm clear, waiting for detections ---')
        else:
            self.get_logger().warn('=== ALARM RAISED ===')

    def on_event(self, msg):
        event = json.loads(msg.data)
        if self.writer is not None:
            self.writer.writerow({k: event.get(k) for k in CSV_FIELDS})
            self.csv_file.flush()
        key = (event['detector'], event['topic'])
        if key in self.seen:
            return
        self.seen.add(key)
        self.get_logger().warn(
            f"[t={event['t']:.1f}] {event['detector']} on {event['topic']}: {event['message']}")

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EventLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Independent exact-snapshot and post-acceptance validation of accepted guides."""
import json
import math
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node

LETHAL, INSCRIBED, UNKNOWN = 254, 253, 255


def empty_stats():
    return dict(total_accepted_guides=0, total_original_path_points=0,
                total_interpolated_samples=0, lethal_samples=0, inscribed_samples=0,
                unknown_samples=0, traversable_samples=0, out_of_map_samples=0,
                guides_with_lethal=0, guides_with_out_of_map=0, guides_with_inscribed=0)


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class Validator(Node):
    def __init__(self):
        super().__init__('m1_local_fast2d_guide_validator')
        self.c = self.create_client(GetCostmap, '/local_costmap/get_costmap')
        self.exact, self.post, self.post_skews = empty_stats(), empty_stats(), []
        self.paths, self.maps, self.pending, self.guide_ids, self.errors = {}, {}, {}, {}, []
        self.create_subscription(NavPath, '/local_fast2d/accepted_path', self.path_cb, 10)
        self.create_subscription(Costmap, '/local_fast2d/accepted_costmap', self.map_cb, 10)
        self.create_subscription(DiagnosticArray, '/local_fast2d/diagnostics', self.diag_cb, 20)

    def path_cb(self, path):
        key = stamp_ns(path.header)
        if not key or key in self.paths:
            self.errors.append(f'invalid/duplicate accepted_path stamp {key}'); return
        self.paths[key] = path; self.try_pair(key)
        # This deliberately remains only a later, dynamic-exposure check.
        if self.c.wait_for_service(timeout_sec=0.2):
            future = self.c.call_async(GetCostmap.Request())
            future.add_done_callback(lambda f, p=path: self.post_check(p, f.result()))

    def map_cb(self, costmap):
        key = stamp_ns(costmap.header)
        if not key or key in self.maps:
            self.errors.append(f'invalid/duplicate accepted_costmap stamp {key}'); return
        self.maps[key] = costmap; self.try_pair(key)

    def diag_cb(self, array):
        for status in array.status:
            if status.name != 'local_fast2d_accepted_snapshot':
                continue
            values = {v.key: v.value for v in status.values}
            try:
                key, guide_id = int(values['snapshot_stamp_ns']), int(values['accepted_guide_id'])
            except (KeyError, ValueError):
                self.errors.append('malformed accepted-snapshot diagnostic'); continue
            if key != stamp_ns(array.header) or key in self.guide_ids:
                self.errors.append(f'invalid/duplicate accepted diagnostic stamp {key}'); continue
            self.guide_ids[key] = guide_id; self.try_pair(key)

    def try_pair(self, key):
        if key not in self.paths or key not in self.maps:
            return
        # Header stamp and unique planning-result id are both required.
        if key not in self.guide_ids:
            self.pending[key] = True; return
        path, costmap = self.paths.pop(key), self.maps.pop(key)
        self.pending.pop(key, None)
        self.validate(path, costmap, self.exact)

    def validate(self, path, costmap, stats):
        info, pts, res = costmap.metadata, path.poses, costmap.metadata.resolution
        expected = info.size_x * info.size_y
        if not pts or res <= 0.0 or len(costmap.data) != expected:
            self.errors.append('invalid costmap payload: metadata=%dx%d data=%d' %
                               (info.size_x, info.size_y, len(costmap.data)))
            return
        stats['total_accepted_guides'] += 1; stats['total_original_path_points'] += len(pts)
        bad = [False, False, False]
        segments = zip(pts, pts[1:]) if len(pts) > 1 else [(pts[0], pts[0])]
        for a, b in segments:
            dx, dy = b.pose.position.x-a.pose.position.x, b.pose.position.y-a.pose.position.y
            count = max(1, math.ceil(math.hypot(dx, dy) / (res * .5)))
            for i in range(count + 1):
                x, y = a.pose.position.x + dx*i/count, a.pose.position.y + dy*i/count
                stats['total_interpolated_samples'] += 1
                mx, my = math.floor((x-info.origin.position.x)/res), math.floor((y-info.origin.position.y)/res)
                if mx < 0 or my < 0 or mx >= info.size_x or my >= info.size_y:
                    stats['out_of_map_samples'] += 1; bad[1] = True; continue
                value = costmap.data[my*info.size_x+mx]
                if value == UNKNOWN: stats['unknown_samples'] += 1
                elif value == LETHAL: stats['lethal_samples'] += 1; bad[0] = True
                elif value == INSCRIBED: stats['inscribed_samples'] += 1; bad[2] = True
                else: stats['traversable_samples'] += 1
        stats['guides_with_lethal'] += bad[0]; stats['guides_with_out_of_map'] += bad[1]
        stats['guides_with_inscribed'] += bad[2]

    def post_check(self, path, response):
        if not response:
            return
        costmap = response.map; a, b = stamp_ns(path.header), stamp_ns(costmap.header)
        self.post_skews.append(abs(a-b)/1e9 if a and b else None)
        self.validate(path, costmap, self.post)

    def save(self, out):
        for key in self.pending: self.errors.append(f'path/costmap missing diagnostic stamp {key}')
        for key in self.paths: self.errors.append(f'unmatched accepted_path stamp {key}')
        for key in self.maps: self.errors.append(f'unmatched accepted_costmap stamp {key}')
        skews = [s for s in self.post_skews if s is not None]
        result = {
            'exact_planning_snapshot': self.exact,
            'post_acceptance_latest_map_check': self.post,
            'post_acceptance_path_costmap_skew_seconds': {
                'measured_guides': len(skews), 'unavailable_zero_stamp_guides': len(self.post_skews)-len(skews),
                'min': min(skews) if skews else None, 'max': max(skews) if skews else None,
                'mean': sum(skews)/len(skews) if skews else None},
            'exact_snapshot_pairing': {'paired_guides': self.exact['total_accepted_guides'],
                'guide_ids': sorted(self.guide_ids.values()), 'errors': self.errors}}
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        Path(out).write_text(json.dumps(result, indent=2))


def main():
    import argparse
    parser = argparse.ArgumentParser(); parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=30); args = parser.parse_args()
    rclpy.init(); node = Validator()
    try:
        end = node.get_clock().now().nanoseconds + int(args.seconds*1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        # The unattended runner ends this passive observer with SIGINT once
        # NavigateToPose has completed.  Persist the pairs already received.
        pass
    finally:
        node.save(args.output); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

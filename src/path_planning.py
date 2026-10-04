from __future__ import annotations

import math
from typing import List, Tuple

from src.models import CarPose, Cone, Path2D


class PathPlanning:

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        cx, cy, yaw = self.car_pose.x, self.car_pose.y, self.car_pose.yaw
        u_dir = (math.cos(yaw), math.sin(yaw))
        v_left = (-math.sin(yaw), math.cos(yaw))
        v_right = (math.sin(yaw), -math.cos(yaw))

        yellow = [c for c in self.cones if c.color == 0]
        blue = [c for c in self.cones if c.color == 1]

        waypoints: List[Tuple[float, float, float, float]] = []

        if yellow and blue:
            smaller = yellow if len(yellow) <= len(blue) else blue
            larger = blue if len(yellow) <= len(blue) else yellow
            used_larger = set()
            matched_pairs = []

            for c_sm in smaller:
                avail = [c for c in larger if id(c) not in used_larger]
                best_c = min(avail, key=lambda c: math.hypot(c.x - c_sm.x, c.y - c_sm.y))
                used_larger.add(id(best_c))
                y_c = c_sm if c_sm.color == 0 else best_c
                b_c = best_c if c_sm.color == 0 else c_sm
                matched_pairs.append((y_c, b_c))

            widths = []
            for y_c, b_c in matched_pairs:
                mx = (y_c.x + b_c.x) / 2.0
                my = (y_c.y + b_c.y) / 2.0
                w = math.hypot(b_c.x - y_c.x, b_c.y - y_c.y)
                widths.append(w)
                tx = (b_c.y - y_c.y) / w
                ty = -(b_c.x - y_c.x) / w
                waypoints.append((mx, my, tx, ty))

            half_w = (sum(widths) / len(widths) / 2.0) if widths else 1.0
            t_ref = (waypoints[0][2], waypoints[0][3])

            for c_lg in larger:
                if not any(id(c_lg) in (id(y), id(b)) for y, b in matched_pairs):
                    same_color = yellow if c_lg.color == 0 else blue
                    other_same = [c for c in same_color if id(c) != id(c_lg)]
                    if other_same:
                        c_near = min(other_same, key=lambda c: math.hypot(c.x - c_lg.x, c.y - c_lg.y))
                        dx = c_lg.x - c_near.x
                        dy = c_lg.y - c_near.y
                        if dx * t_ref[0] + dy * t_ref[1] < 0:
                            dx, dy = -dx, -dy
                        d_len = math.hypot(dx, dy)
                        tx, ty = dx / d_len, dy / d_len
                    else:
                        tx, ty = t_ref[0], t_ref[1]

                    if c_lg.color == 0:
                        nx, ny = -ty, tx
                    else:
                        nx, ny = ty, -tx
                    waypoints.append((c_lg.x + half_w * nx, c_lg.y + half_w * ny, tx, ty))

        elif yellow:
            half_w = 1.0
            s_yellow = sorted(yellow, key=lambda c: math.hypot(c.x - cx, c.y - cy))
            if len(s_yellow) == 1:
                waypoints.append((s_yellow[0].x + half_w * v_left[0], s_yellow[0].y + half_w * v_left[1], u_dir[0], u_dir[1]))
            else:
                for i, c in enumerate(s_yellow):
                    if i < len(s_yellow) - 1:
                        dx = s_yellow[i + 1].x - c.x
                        dy = s_yellow[i + 1].y - c.y
                    else:
                        dx = c.x - s_yellow[i - 1].x
                        dy = c.y - s_yellow[i - 1].y
                    d_len = math.hypot(dx, dy)
                    tx, ty = dx / d_len, dy / d_len
                    nx, ny = -ty, tx
                    waypoints.append((c.x + half_w * nx, c.y + half_w * ny, tx, ty))

        elif blue:
            half_w = 1.0
            s_blue = sorted(blue, key=lambda c: math.hypot(c.x - cx, c.y - cy))
            if len(s_blue) == 1:
                waypoints.append((s_blue[0].x + half_w * v_right[0], s_blue[0].y + half_w * v_right[1], u_dir[0], u_dir[1]))
            else:
                for i, c in enumerate(s_blue):
                    if i < len(s_blue) - 1:
                        dx = s_blue[i + 1].x - c.x
                        dy = s_blue[i + 1].y - c.y
                    else:
                        dx = c.x - s_blue[i - 1].x
                        dy = c.y - s_blue[i - 1].y
                    d_len = math.hypot(dx, dy)
                    tx, ty = dx / d_len, dy / d_len
                    nx, ny = ty, -tx
                    waypoints.append((c.x + half_w * nx, c.y + half_w * ny, tx, ty))

        ref_dir = (waypoints[0][2], waypoints[0][3]) if waypoints else u_dir
        waypoints = sorted(waypoints, key=lambda w: (w[0] - cx) * ref_dir[0] + (w[1] - cy) * ref_dir[1])

        control_nodes = [(cx, cy, u_dir[0], u_dir[1])] + waypoints
        last_pt = control_nodes[-1]
        last_tx, last_ty = last_pt[2], last_pt[3]

        for ext_d in (2.0, 5.0, 8.0, 11.0):
            control_nodes.append((last_pt[0] + ext_d * last_tx, last_pt[1] + ext_d * last_ty, last_tx, last_ty))

        dense_path: List[Tuple[float, float]] = []
        for i in range(len(control_nodes) - 1):
            p0 = control_nodes[i]
            p1 = control_nodes[i + 1]
            seg_d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if seg_d < 1e-4:
                continue
            scale = min(seg_d, 2.5)
            num_sub = max(15, int(seg_d / 0.04))
            for step_idx in range(num_sub):
                u = step_idx / num_sub
                h00 = 2 * u**3 - 3 * u**2 + 1
                h10 = u**3 - 2 * u**2 + u
                h01 = -2 * u**3 + 3 * u**2
                h11 = u**3 - u**2
                x = h00 * p0[0] + h10 * scale * p0[2] + h01 * p1[0] + h11 * scale * p1[2]
                y = h00 * p0[1] + h10 * scale * p0[3] + h01 * p1[1] + h11 * scale * p1[3]
                dense_path.append((x, y))

        target_length = 7.0
        step_size = 0.25
        num_steps = int(target_length / step_size)
        sampled_path: Path2D = [(cx, cy)]

        current_dist = 0.0
        target_dist = step_size

        for i in range(1, len(dense_path)):
            prev = dense_path[i - 1]
            curr = dense_path[i]
            d = math.hypot(curr[0] - prev[0], curr[1] - prev[1])
            while current_dist + d >= target_dist and len(sampled_path) <= num_steps:
                ratio = (target_dist - current_dist) / d if d > 1e-6 else 0.0
                interp_x = prev[0] + ratio * (curr[0] - prev[0])
                interp_y = prev[1] + ratio * (curr[1] - prev[1])
                sampled_path.append((interp_x, interp_y))
                target_dist += step_size
            current_dist += d
            if len(sampled_path) > num_steps:
                break

        return sampled_path

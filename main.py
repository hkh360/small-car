import json
import math
import time
from my_udp import UDPClient


class Control:
    def __init__(self):

        self.vehicle_name = '1'
        self.udp_port = 9000
        self.udp_send_port = 9001
        self.server_ip = '192.168.1.100'

        net = "KmRzLF1R34PSDzlXgphCnddQ0HCI,192.168.112.1,6720,6721"
        if net != "":
            net = net.split(",")
            self.vehicle_name = net[0]
            self.server_ip = net[1]
            self.udp_port = int(net[2])
            self.udp_send_port = int(net[3])

        print(self.vehicle_name)
        print(self.udp_port)
        print(self.udp_send_port)
        print(self.server_ip)
        self.udp_client = UDPClient(self.server_ip, self.udp_port, self.udp_send_port, self.vehicle_name)

        self.m_v = 0
        self.m_x = 0
        self.m_y = 0
        self.m_yaw = 0
        self.vehpos_initial_index = 0
        self.num_preview = 5
        self.targetPos_Info = [0.0, 0.0]
        self.Y_points = []
        self.X_points = []
        self.control_rate = 10  # hz
        self.wheel_base = 2.7
        self.cruise_speed = 10.0
        self.lane_change_active = False
        self.lane_change_start_time = None
        self.lane_change_direction = 0  # 1=左变道, -1=右变道
        self.lane_change_target_vehicle = None
        self.lane_change_state = "checking"  # checking, changing, returning
        self.original_lane_offset = 0
        self.tuning = {
            "road_width": 3.0,
            "vehicle_half_width": 0.9,
            "min_crawl_speed": 1.0,
            "follow_time_gap": 2.0,
            "safe_distance": 15.0,       # 增大安全距离
            "warning_distance": 25.0,    # 增大预警距离
            "emergency_stop_distance": 5.0,  # 增大紧急刹车距离
            "meeting_distance": 18.0,    # 增大对向车检测距离
            "blockage_trigger_distance": 10.0,
            "blockage_timeout": 4.0,
            "blockage_speed_threshold": 0.6,
            "escape_reverse_speed": -1.5,
            "escape_forward_speed": 1.5,
            "escape_turn_rate": 0.2,
            "escape_reverse_duration": 1.5,
            "escape_borrow_duration": 2.0,
            "escape_straighten_duration": 1.2,
            "escape_side_clearance_margin": 0.4,
            "rear_safety_distance": 4.0,
            "curve_speed_factor": 0.5,  # 急弯速度因子
            "medium_curve_speed_factor": 0.7,  # 中等弯道速度因子
            "gentle_curve_speed_factor": 0.85,  # 缓弯速度因子
            "curve_detection_threshold": 0.06,  # 弯道检测阈值（弧度/帧）
            "sharp_curve_threshold": 0.12,  # 急弯阈值
            "medium_curve_threshold": 0.08,  # 中等弯道阈值
            "curve_preview_threshold": 0.08,
            "curve_exit_threshold": 0.04,
            "curve_inner_error_threshold": 0.25,
            "straight_preview_distance": 18.0,
            "curve_preview_distance": 4.5,
            "curve_min_preview_distance": 3.0,
            "curve_outer_offset": 0.65,
            "curve_max_outer_offset": 1.05,
            "curve_inner_error_gain": 0.8,
            "straight_steering_deadband": 0.035,
            "straight_yaw_deadband": 0.045,
            "straight_max_yaw_rate": 0.20,
            "curve_max_yaw_rate": 0.55,
            "yaw_filter_alpha": 0.82,
            "curve_exit_recovery_duration": 1.0,
            "curve_exit_recovery_speed": 5.0,
            "curve_exit_fast_recovery_speed": 12.5,
            "curve_exit_recovery_preview": 8.0,
            "curve_exit_straight_search": 35.0,
            "curve_exit_line_heading_span": 18.0,
            "curve_exit_heading_start": 8.0,
            "curve_exit_heading_end": 20.0,
            "curve_exit_heading_kp": 0.75,
            "curve_exit_parallel_tolerance": 0.012,
            "curve_exit_release_heading": 0.012,
            "curve_exit_release_lateral": 0.45,
            "curve_exit_fast_heading": 0.025,
            "curve_exit_fast_lateral": 0.85,
            "curve_exit_extend_lateral": 0.75,
            "curve_exit_extend_duration": 0.15,
            "curve_exit_lateral_kp": 0.12,
            "curve_exit_softening": 3.0,
            "curve_exit_max_yaw_rate": 0.34,
            "curve_exit_yaw_rate_delta": 0.05,
            "straight_yaw_rate_delta": 0.025,
            "curve_yaw_rate_delta": 0.08,
            "max_steering_angle": 0.45,
            "straight_speed": 15.0,  # 直道目标速度
            "gentle_curve_speed": 5.0,  # 缓弯速度
            "medium_curve_speed": 3.8,  # 中等弯道速度
            "sharp_curve_speed": 2.8,  # 急弯速度
            "speed_smoothing": 0.85,  # 速度平滑系数
            "lane_change_distance": 12.0,  # 变道检测距离
            "lane_change_min_gap": 8.0,  # 变道最小安全距离
            "lane_change_duration": 2.0,  # 变道持续时间（秒）
            "lane_change_speed_min": 4.0,  # 变道最低速度
            "lane_change_speed_max": 9.0,  # 变道最高速度
            "lane_change_clearance": 1.5,  # 侧向安全净空（米）
            "lane_merge_distance": 10.0,
            "intersection_center_x": 50.0,
            "intersection_center_y": 50.0,
            "intersection_radius": 10.0,
            "intersection_approach_radius": 18.0,
            "intersection_time_gap": 2.5,
            "yield_timeout": 3.0,
            "deadlock_timeout": 3.0,
            "max_control_speed": 16.0,
            "manual_relocate_distance": 6.0,
            "route_relocalize_distance": 8.0,
            "relocalize_recovery_duration": 1.2,
            "relocalize_recovery_speed": 4.0,
        }
        self.deadlock_start_time = None
        self.blockage_start_time = None
        self.escape_state = None
        self.escape_state_start_time = None
        self.escape_direction = 1
        self.escape_target_name = None
        self.intersection_yield_start_time = None
        self.last_interaction_mode = "free"
        self.in_effective_curve = False
        self.just_exited_curve = False
        self.curve_exit_hold_until = 0.0
        self.curve_exit_line = None
        self.curve_exit_heading_error = 0.0
        self.curve_exit_lateral_error = 0.0
        self.filtered_steering_angle = 0.0
        self.last_path_w = 0.0
        self.last_m_x = None
        self.last_m_y = None
        self.relocalize_recovery_until = 0.0

    def control_node(self):
        start_time = time.time()
        self.load_route('exp_routes/rightInside.json')

        while True:
            vehicle_data = self.udp_client.get_vehicle_state()
            self.m_x = vehicle_data.x
            self.m_y = vehicle_data.y
            self.m_yaw = vehicle_data.yaw / 180 * math.pi
            self.m_v = max(getattr(vehicle_data, 'speed', 0.0), 0.0)
            self.detect_manual_relocation()
            self.update_vehpos_index()
            self.search_target_pos()

            # 计算原始控制指令
            v, w = self.calc_pure_pursuit(self.m_x, self.m_y, self.m_yaw, self.targetPos_Info)

            v, w = self.apply_interaction_control(self.m_x, self.m_y, self.m_yaw, v, w)
            v, w = self.apply_relocalize_recovery_limit(v, w)
            self.udp_client.send_control_command(v, w)

            elapsed_time = time.time() - start_time
            sleep_time = max((1.0 / self.control_rate) - elapsed_time, 0.0)
            time.sleep(sleep_time)
            start_time = time.time()

    def load_route(self, file_path):
        with open(file_path, 'r', encoding='utf-8') as file:
            json_track = json.load(file)

        if isinstance(json_track, list):
            self.X_points = [point["x"] for point in json_track]
            self.Y_points = [point["y"] for point in json_track]
        elif isinstance(json_track, dict) and "X" in json_track and "Y" in json_track:
            self.X_points = json_track["X"]
            self.Y_points = json_track["Y"]
        else:
            raise ValueError(
                "Unsupported route format. Expected [{'x': ..., 'y': ...}, ...] "
                "or {'X': [...], 'Y': [...]}."
            )

        if len(self.X_points) != len(self.Y_points) or len(self.X_points) == 0:
            raise ValueError("Route file must contain the same non-zero number of X and Y points.")

        self.X_points = [float(x) for x in self.X_points]
        self.Y_points = [float(y) for y in self.Y_points]

    def get_param(self, name):
        return self.tuning[name]

    def calc_pure_pursuit(self, m_x, m_y, m_yaw, target_pos):
        dx = target_pos[0] - m_x
        dy = target_pos[1] - m_y
        Ld = math.sqrt(dx ** 2 + dy ** 2)
        alpha = math.atan2(dy, dx) - m_yaw
        alpha = self.normalize_angle(alpha)

        if Ld > 0.001:
            steering_angle = math.atan2(2 * self.wheel_base * math.sin(alpha), Ld)
        else:
            steering_angle = 0

        steering_angle = self.limit_steering_angle(steering_angle)

        steering_angle_deg = abs(steering_angle) * 180 / math.pi
        v = self.get_param("straight_speed")

        if getattr(self, "in_effective_curve", False):
            route_curve = getattr(self, "upcoming_curve_rate", 0.0)
            if route_curve > self.get_param("sharp_curve_threshold") or steering_angle_deg > 12:
                v = min(v, self.get_param("sharp_curve_speed"))
            elif route_curve > self.get_param("medium_curve_threshold") or steering_angle_deg > 8:
                v = min(v, self.get_param("medium_curve_speed"))
            else:
                v = min(v, self.get_param("gentle_curve_speed"))
        elif self.is_curve_exit_recovering():
            self.update_curve_exit_errors()
            v = min(v, self.curve_exit_speed_limit())

        w = v * math.tan(steering_angle) / self.wheel_base
        w = self.stabilize_yaw_rate(w)
        return v, w

    def is_curve_exit_recovering(self):
        return time.time() < self.curve_exit_hold_until

    def curve_exit_speed_limit(self):
        heading = abs(self.curve_exit_heading_error)
        lateral = abs(self.curve_exit_lateral_error)

        if heading < self.get_param("curve_exit_release_heading") and lateral < self.get_param("curve_exit_release_lateral"):
            self.curve_exit_hold_until = 0.0
            return self.get_param("straight_speed")

        if heading < self.get_param("curve_exit_fast_heading") and lateral < self.get_param("curve_exit_fast_lateral"):
            return self.get_param("curve_exit_fast_recovery_speed")

        return self.get_param("curve_exit_recovery_speed")

    def limit_steering_angle(self, steering_angle):
        if not getattr(self, "in_effective_curve", False):
            if abs(steering_angle) < self.get_param("straight_steering_deadband"):
                return 0.0
        max_steering = self.get_param("max_steering_angle")
        return max(-max_steering, min(max_steering, steering_angle))

    def stabilize_yaw_rate(self, w):
        in_curve = getattr(self, "in_effective_curve", False)
        if not in_curve and self.is_curve_exit_recovering():
            self.just_exited_curve = False
            return self.curve_exit_recovery_yaw_rate()

        if in_curve:
            max_w = self.get_param("curve_max_yaw_rate")
            max_delta = self.get_param("curve_yaw_rate_delta")
        else:
            if abs(w) < self.get_param("straight_yaw_deadband"):
                w = 0.0
            max_w = self.get_param("straight_max_yaw_rate")
            max_delta = self.get_param("straight_yaw_rate_delta")

        w = max(-max_w, min(max_w, w))
        alpha = self.get_param("yaw_filter_alpha")
        w = alpha * self.last_path_w + (1.0 - alpha) * w
        w = max(self.last_path_w - max_delta, min(self.last_path_w + max_delta, w))

        if not in_curve and abs(w) < self.get_param("straight_yaw_deadband"):
            w = 0.0

        self.last_path_w = w
        return w

    def curve_exit_recovery_yaw_rate(self):
        route_heading, lateral_error = self.update_curve_exit_errors()
        heading_error = self.curve_exit_heading_error

        release_ready = (
            abs(heading_error) < self.get_param("curve_exit_release_heading")
            and abs(lateral_error) < self.get_param("curve_exit_release_lateral")
        )
        if release_ready:
            self.curve_exit_hold_until = 0.0
        elif (
            abs(heading_error) > self.get_param("curve_exit_parallel_tolerance")
            or abs(lateral_error) > self.get_param("curve_exit_extend_lateral")
        ):
            self.curve_exit_hold_until = max(
                self.curve_exit_hold_until,
                time.time() + self.get_param("curve_exit_extend_duration"),
            )

        heading_term = self.get_param("curve_exit_heading_kp") * heading_error
        lateral_term = math.atan2(
            self.get_param("curve_exit_lateral_kp") * lateral_error,
            max(self.m_v, 0.0) + self.get_param("curve_exit_softening"),
        )
        w = heading_term - lateral_term

        max_w = self.get_param("curve_exit_max_yaw_rate")
        max_delta = self.get_param("curve_exit_yaw_rate_delta")
        w = max(-max_w, min(max_w, w))
        w = max(self.last_path_w - max_delta, min(self.last_path_w + max_delta, w))
        self.last_path_w = w
        return w

    def update_curve_exit_errors(self):
        if self.curve_exit_line is None:
            self.prepare_curve_exit_line()

        if self.curve_exit_line is not None:
            line_x, line_y, route_heading = self.curve_exit_line
            lateral_error = self.lateral_error_to_line(line_x, line_y, route_heading, self.m_x, self.m_y)
        else:
            route_heading = self.route_heading_over_distance(
                self.vehpos_initial_index,
                self.get_param("curve_exit_heading_start"),
                self.get_param("curve_exit_heading_end"),
            )
            lateral_error = self.route_lateral_error_at(self.vehpos_initial_index, self.m_x, self.m_y)

        heading_error = self.normalize_angle(route_heading - self.m_yaw)
        self.curve_exit_heading_error = heading_error
        self.curve_exit_lateral_error = lateral_error
        return route_heading, lateral_error

    @staticmethod
    def normalize_angle(angle):
        while angle < -math.pi:
            angle += 2 * math.pi
        while angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def detect_manual_relocation(self):
        if self.last_m_x is None or self.last_m_y is None:
            self.last_m_x = self.m_x
            self.last_m_y = self.m_y
            return

        moved_distance = math.sqrt((self.m_x - self.last_m_x) ** 2 + (self.m_y - self.last_m_y) ** 2)
        self.last_m_x = self.m_x
        self.last_m_y = self.m_y

        if moved_distance > self.get_param("manual_relocate_distance"):
            print(f"Manual relocation detected: jump={moved_distance:.2f}, resetting controller state")
            self.relocalize_to_route()

    def reset_controller_state_after_relocation(self):
        self.in_effective_curve = False
        self.just_exited_curve = False
        self.curve_exit_hold_until = 0.0
        self.curve_exit_line = None
        self.curve_exit_heading_error = 0.0
        self.curve_exit_lateral_error = 0.0
        self.filtered_steering_angle = 0.0
        self.last_path_w = 0.0
        self.lane_change_state = "checking"
        self.lane_change_start_time = None
        self.lane_change_direction = 0
        self.lane_change_target_vehicle = None
        self.blockage_start_time = None
        self.deadlock_start_time = None
        self.intersection_yield_start_time = None
        self.reset_escape_state()
        if hasattr(self, 'overtaking_state'):
            delattr(self, 'overtaking_state')
        if hasattr(self, 'overtaking_start_x'):
            delattr(self, 'overtaking_start_x')

    def relocalize_to_route(self):
        self.search_vehicle_initial_index()
        self.reset_controller_state_after_relocation()
        self.relocalize_recovery_until = time.time() + self.get_param("relocalize_recovery_duration")
        self.detect_upcoming_curve()
        target_index = self.find_lookahead_index_by_distance(self.vehpos_initial_index, 6.0)
        self.targetPos_Info[0] = self.X_points[target_index]
        self.targetPos_Info[1] = self.Y_points[target_index]

    def apply_relocalize_recovery_limit(self, v, w):
        if time.time() >= self.relocalize_recovery_until:
            return v, w

        limited_v = min(v, self.get_param("relocalize_recovery_speed"))
        limited_w = max(-0.25, min(0.25, w))
        return limited_v, limited_w

    def check_lane_change_possible(self, m_x, m_y, m_yaw, direction):
        """检查是否可以变道"""
        look_distance = 15.0
        road_half_width = self.get_param("road_width") / 2.0
        required_clearance = self.get_param("lane_change_clearance")
        min_gap = self.get_param("lane_change_min_gap")

        for other in self.udp_client.get_neighbor_vehicle_state():
            dx = other.x - m_x
            dy = other.y - m_y
            longitudinal, lateral = self.project_to_vehicle_frame(dx, dy, m_yaw)

            if longitudinal < -min_gap or longitudinal > look_distance:
                continue

            target_side = lateral * direction
            in_target_lane = 0.4 < target_side < road_half_width + self.get_param("vehicle_half_width")
            if in_target_lane and abs(longitudinal) < min_gap:
                return False

        side_clearance = self.compute_side_clearance(m_x, m_y, m_yaw, direction, look_distance)
        if side_clearance < required_clearance:
            return False

        return True

    def find_vehicle_for_overtake(self, m_x, m_y, m_yaw, v):
        """寻找可以超车的目标车辆"""
        detection_distance = self.get_param("lane_change_distance")
        road_half_width = self.get_param("road_width") / 2.0

        # 获取所有邻居车辆
        neighbor_data = self.udp_client.get_neighbor_vehicle_state()

        for other in neighbor_data:
            dx = other.x - m_x
            dy = other.y - m_y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance > detection_distance:
                continue

            # 投影到车辆坐标系
            longitudinal, lateral = self.project_to_vehicle_frame(dx, dy, m_yaw)

            # 只考虑前方车辆
            if longitudinal <= 0:
                continue

            # 只考虑同向车辆（航向角差小于30度）
            other_yaw = getattr(other, 'yaw', 0)
            heading_diff = abs(self.normalize_angle(other_yaw - m_yaw))
            if heading_diff > math.radians(30):
                continue

            # 只考虑在本车道或相邻车道的车辆
            if abs(lateral) > road_half_width + 1.0:
                continue

            other_speed = max(getattr(other, 'speed', 0), 0)

            # 前车速度比我慢，或者几乎静止
            if other_speed < v * 0.7 or other_speed < 3.0:
                print(f"Found vehicle to overtake: distance={distance:.2f}, "
                      f"my_speed={v:.2f}, other_speed={other_speed:.2f}")
                return other

        return None

    def lane_change_overtake(self, m_x, m_y, m_yaw, v, w):
        """变道超车主函数"""
        current_time = time.time()

        # 状态机：检查是否需要变道
        if self.lane_change_state == "checking":
            # 只在一定速度以上才考虑变道
            if v < self.get_param("lane_change_speed_min"):
                return v, w

            # 寻找可超车的目标
            target = self.find_vehicle_for_overtake(m_x, m_y, m_yaw, v)

            if target is None:
                return v, w

            # 检查左右哪边可以变道
            left_possible = self.check_lane_change_possible(m_x, m_y, m_yaw, 1)
            right_possible = self.check_lane_change_possible(m_x, m_y, m_yaw, -1)

            if not left_possible and not right_possible:
                print("Lane change: no space available, following")
                # 没有变道空间，减速跟随
                return max(v * 0.6, self.get_param("min_crawl_speed")), w

            # 优先选择左侧变道（通常左侧是快车道）
            if left_possible:
                self.lane_change_direction = 1
                print("Lane change: starting left lane change")
            else:
                self.lane_change_direction = -1
                print("Lane change: starting right lane change")

            self.lane_change_state = "changing"
            self.lane_change_start_time = current_time
            self.lane_change_target_vehicle = target
            self.original_lane_offset = 0

            return v, w

        # 状态：正在变道
        elif self.lane_change_state == "changing":
            elapsed = current_time - self.lane_change_start_time
            duration = self.get_param("lane_change_duration")

            if elapsed >= duration:
                # 变道完成，进入返回状态
                self.lane_change_state = "returning"
                self.lane_change_start_time = current_time
                print("Lane change completed, preparing to return")
                return v, w

            # 计算变道进度（0到1）
            progress = elapsed / duration

            # 使用平滑曲线计算偏移量
            # 正弦曲线：从0到1再到0的平滑过渡
            lane_offset = math.sin(math.pi * progress) * 1.2  # 最大偏移1.2米

            # 根据方向调整偏移符号
            if self.lane_change_direction == -1:
                lane_offset = -lane_offset

            # 计算变道时的控制指令
            v_lane, w_lane = self.calculate_lane_change_control(
                m_x, m_y, m_yaw, v, w, lane_offset
            )

            # 变道时保持适当速度
            v_lane = min(v_lane, self.get_param("lane_change_speed_max"))

            print(f"Lane changing: progress={progress:.2f}, offset={lane_offset:.2f}")
            return v_lane, w_lane

        # 状态：超车后返回原车道
        elif self.lane_change_state == "returning":
            elapsed = current_time - self.lane_change_start_time
            duration = self.get_param("lane_change_duration") * 0.8  # 返回稍快

            if elapsed >= duration:
                # 完全返回，重置状态
                self.lane_change_state = "checking"
                self.lane_change_target_vehicle = None
                print("Lane change: returned to original lane")
                return v, w

            # 返回原车道（偏移量逐渐归零）
            progress = 1.0 - (elapsed / duration)
            lane_offset = math.sin(math.pi * progress) * 1.2

            if self.lane_change_direction == -1:
                lane_offset = -lane_offset

            v_lane, w_lane = self.calculate_lane_change_control(
                m_x, m_y, m_yaw, v, w, lane_offset
            )

            print(f"Lane returning: progress={progress:.2f}, offset={lane_offset:.2f}")
            return v_lane, w_lane

        return v, w

    def calculate_lane_change_control(self, m_x, m_y, m_yaw, v, w, lateral_offset):
        """计算变道时的控制指令"""
        # 找到当前路径上的目标点
        if hasattr(self, 'targetPos_Info') and len(self.targetPos_Info) >= 2:
            # 计算路径方向
            current_idx = self.vehpos_initial_index
            if current_idx + 1 < len(self.X_points):
                path_dx = self.X_points[current_idx + 1] - self.X_points[current_idx]
                path_dy = self.Y_points[current_idx + 1] - self.Y_points[current_idx]
                path_yaw = math.atan2(path_dy, path_dx)

                # 计算垂直于路径的方向向量
                perp_x = -math.sin(path_yaw) * lateral_offset
                perp_y = math.cos(path_yaw) * lateral_offset

                # 调整目标点位置
                adjusted_target_x = self.targetPos_Info[0] + perp_x
                adjusted_target_y = self.targetPos_Info[1] + perp_y

                # 重新计算纯跟踪控制
                dx = adjusted_target_x - m_x
                dy = adjusted_target_y - m_y
                Ld = math.sqrt(dx ** 2 + dy ** 2)
                alpha = math.atan2(dy, dx) - m_yaw
                alpha = self.normalize_angle(alpha)

                if Ld > 0.001:
                    steering_angle = math.atan2(2 * self.wheel_base * math.sin(alpha), Ld)
                else:
                    steering_angle = 0

                # 限制最大转向角
                max_steering = 0.5
                steering_angle = max(-max_steering, min(max_steering, steering_angle))

                # 变道时使用中等速度
                w_new = v * math.tan(steering_angle) / self.wheel_base

                return v, w_new

        return v, w

    def project_to_vehicle_frame(self, dx, dy, yaw):
        longitudinal = dx * math.cos(yaw) + dy * math.sin(yaw)
        lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return longitudinal, lateral

    def find_closest_vehicle_in_corridor(self, m_x, m_y, m_yaw, max_distance, half_width, max_heading_diff):
        closest_vehicle = None
        closest_longitudinal = float('inf')
        closest_lateral = 0.0

        for other in self.udp_client.get_neighbor_vehicle_state():
            dx = other.x - m_x
            dy = other.y - m_y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            if distance > max_distance:
                continue

            longitudinal, lateral = self.project_to_vehicle_frame(dx, dy, m_yaw)
            if longitudinal <= 0 or abs(lateral) > half_width:
                continue

            heading_diff = abs(self.normalize_angle(getattr(other, 'yaw', 0.0) - m_yaw))
            if heading_diff > max_heading_diff:
                continue

            if longitudinal < closest_longitudinal:
                closest_vehicle = other
                closest_longitudinal = longitudinal
                closest_lateral = lateral

        return closest_vehicle, closest_longitudinal, closest_lateral

    def compute_side_clearance(self, m_x, m_y, m_yaw, direction, look_distance):
        half_road_width = self.get_param("road_width") / 2.0
        occupied = 0.0

        for other in self.udp_client.get_neighbor_vehicle_state():
            dx = other.x - m_x
            dy = other.y - m_y
            longitudinal, lateral = self.project_to_vehicle_frame(dx, dy, m_yaw)
            if abs(longitudinal) > look_distance:
                continue

            if direction > 0 and lateral <= 0:
                continue
            if direction < 0 and lateral >= 0:
                continue

            occupied = max(occupied, abs(lateral) + self.get_param("vehicle_half_width"))

        return max(half_road_width - occupied, 0.0)

    def has_rear_space_for_escape(self, m_x, m_y, m_yaw):
        for other in self.udp_client.get_neighbor_vehicle_state():
            dx = other.x - m_x
            dy = other.y - m_y
            longitudinal, lateral = self.project_to_vehicle_frame(dx, dy, m_yaw)
            if longitudinal >= 0:
                continue

            if abs(lateral) < self.get_param("vehicle_half_width") * 1.8 and abs(longitudinal) < self.get_param(
                "rear_safety_distance"
            ):
                return False

        return True

    def choose_escape_direction(self, m_x, m_y, m_yaw):
        left_clearance = self.compute_side_clearance(m_x, m_y, m_yaw, direction=1, look_distance=8.0)
        right_clearance = self.compute_side_clearance(m_x, m_y, m_yaw, direction=-1, look_distance=8.0)
        if left_clearance >= right_clearance:
            return 1, left_clearance
        return -1, right_clearance

    def reset_escape_state(self):
        self.blockage_start_time = None
        self.escape_state = None
        self.escape_state_start_time = None
        self.escape_target_name = None

    def persistent_blockage_recovery(self, m_x, m_y, m_yaw, v, w):
        now = time.time()
        blockage_distance = self.get_param("blockage_trigger_distance")
        road_half_width = self.get_param("road_width") / 2.0
        blockage_vehicle, front_gap, _ = self.find_closest_vehicle_in_corridor(
            m_x,
            m_y,
            m_yaw,
            max_distance=blockage_distance,
            half_width=road_half_width,
            max_heading_diff=math.radians(100.0),
        )

        if self.escape_state is not None:
            if blockage_vehicle is None:
                self.reset_escape_state()
                return v, w

            elapsed = now - self.escape_state_start_time
            turn_rate = self.get_param("escape_turn_rate")

            if self.escape_state == "reverse":
                if elapsed < self.get_param("escape_reverse_duration"):
                    return self.get_param("escape_reverse_speed"), -self.escape_direction * turn_rate
                self.escape_state = "borrow"
                self.escape_state_start_time = now
                elapsed = 0.0

            if self.escape_state == "borrow":
                if elapsed < self.get_param("escape_borrow_duration"):
                    return self.get_param("escape_forward_speed"), self.escape_direction * turn_rate
                self.escape_state = "straighten"
                self.escape_state_start_time = now
                elapsed = 0.0

            if self.escape_state == "straighten":
                if elapsed < self.get_param("escape_straighten_duration"):
                    return self.get_param("escape_forward_speed"), -self.escape_direction * turn_rate * 0.8
                self.reset_escape_state()
                return v, w

        blocked = (
            blockage_vehicle is not None
            and front_gap < blockage_distance
            and abs(v) <= self.get_param("blockage_speed_threshold")
        )

        if not blocked:
            self.blockage_start_time = None
            return v, w

        front_speed = max(getattr(blockage_vehicle, "speed", 0.0), 0.0)
        if front_speed > self.get_param("min_crawl_speed") * 1.2:
            self.blockage_start_time = None
            return v, w

        if self.blockage_start_time is None:
            self.blockage_start_time = now
            self.escape_target_name = getattr(blockage_vehicle, "name", None)
            return v, w

        if now - self.blockage_start_time < self.get_param("blockage_timeout"):
            return v, w

        self.escape_direction, side_clearance = self.choose_escape_direction(m_x, m_y, m_yaw)
        if side_clearance < self.get_param("escape_side_clearance_margin"):
            return v, w

        if not self.has_rear_space_for_escape(m_x, m_y, m_yaw):
            return v, w

        self.escape_state = "reverse"
        self.escape_state_start_time = now
        print(
            f"Persistent blockage detected, starting recovery: direction={self.escape_direction}, "
            f"clearance={side_clearance:.2f}"
        )
        return self.get_param("escape_reverse_speed"), -self.escape_direction * self.get_param("escape_turn_rate")

    def search_vehicle_initial_index(self):
        min_distance = float('inf')
        nearest_index = 0

        for i in range(len(self.X_points)):
            this_point_x = self.X_points[i]
            this_point_y = self.Y_points[i]

            distance = math.sqrt((self.m_x - this_point_x) ** 2 + (self.m_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        self.vehpos_initial_index = nearest_index


    def find_nearest_point_index(self, target_x, target_y):
        min_distance = float('inf')
        nearest_index = -1

        for i in range(len(self.X_points)):
            this_point_x = self.X_points[i]
            this_point_y = self.Y_points[i]

            distance = math.sqrt((target_x - this_point_x) ** 2 + (target_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        return nearest_index

    def update_vehpos_index(self):
        """改进版：同时检测前方弯道"""
        min_distance = float('inf')
        nearest_index = 0
        for i in range(40):
            find_index = (self.vehpos_initial_index + i) % len(self.X_points)
            this_point_x = self.X_points[find_index]
            this_point_y = self.Y_points[find_index]

            distance = math.sqrt((self.m_x - this_point_x) ** 2 + (self.m_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = find_index

        if min_distance > self.get_param("route_relocalize_distance"):
            print(f"Route relocalization: local route distance={min_distance:.2f}")
            self.search_vehicle_initial_index()
            self.reset_controller_state_after_relocation()
            self.relocalize_recovery_until = time.time() + self.get_param("relocalize_recovery_duration")
        else:
            self.vehpos_initial_index = nearest_index

        # 记录前方路线曲率，实际降速由当前转向需求决定。
        self.detect_upcoming_curve()

    def detect_upcoming_curve(self):
        """Record route curvature only; speed is decided by current steering demand."""
        self.upcoming_curve_rate = self.compute_route_curve_rate(self.vehpos_initial_index, 12)
        self.upcoming_curve_sign = self.compute_route_curve_sign(self.vehpos_initial_index, 12)

    def compute_route_curve_rate(self, start_index, lookahead_indices):
        if not hasattr(self, 'X_points') or len(self.X_points) < 3:
            return 0.0

        end_index = min(start_index + lookahead_indices, len(self.X_points) - 1)
        angles = []
        for i in range(start_index, end_index):
            dx = self.X_points[i + 1] - self.X_points[i]
            dy = self.Y_points[i + 1] - self.Y_points[i]
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                continue
            angles.append(math.atan2(dy, dx))

        if len(angles) < 2:
            return 0.0

        total_change = 0.0
        for i in range(len(angles) - 1):
            total_change += abs(self.normalize_angle(angles[i + 1] - angles[i]))

        return total_change / max(len(angles) - 1, 1)

    def compute_route_curve_sign(self, start_index, lookahead_indices):
        if not hasattr(self, 'X_points') or len(self.X_points) < 3:
            return 0

        end_index = min(start_index + lookahead_indices, len(self.X_points) - 1)
        angles = []
        for i in range(start_index, end_index):
            dx = self.X_points[i + 1] - self.X_points[i]
            dy = self.Y_points[i + 1] - self.Y_points[i]
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                continue
            angles.append(math.atan2(dy, dx))

        signed_change = 0.0
        for i in range(len(angles) - 1):
            signed_change += self.normalize_angle(angles[i + 1] - angles[i])

        if signed_change > 1e-4:
            return 1
        if signed_change < -1e-4:
            return -1
        return 0

    def route_heading_at(self, index):
        if not hasattr(self, 'X_points') or len(self.X_points) < 2:
            return self.m_yaw

        index = max(0, min(index, len(self.X_points) - 2))
        dx = self.X_points[index + 1] - self.X_points[index]
        dy = self.Y_points[index + 1] - self.Y_points[index]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return self.m_yaw
        return math.atan2(dy, dx)

    def route_heading_over_distance(self, start_index, start_distance, end_distance):
        if not hasattr(self, 'X_points') or len(self.X_points) < 2:
            return self.m_yaw

        start_idx = self.find_lookahead_index_by_distance(start_index, start_distance)
        end_idx = self.find_lookahead_index_by_distance(start_index, end_distance)
        dx = self.X_points[end_idx] - self.X_points[start_idx]
        dy = self.Y_points[end_idx] - self.Y_points[start_idx]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return self.route_heading_at(end_idx)
        return math.atan2(dy, dx)

    def find_lookahead_index_by_distance(self, start_index, lookahead_distance):
        if not hasattr(self, 'X_points') or len(self.X_points) == 0:
            return 0

        total_distance = 0.0
        index = start_index
        point_count = len(self.X_points)

        for _ in range(point_count - 1):
            next_index = (index + 1) % point_count
            dx = self.X_points[next_index] - self.X_points[index]
            dy = self.Y_points[next_index] - self.Y_points[index]
            total_distance += math.sqrt(dx ** 2 + dy ** 2)
            index = next_index
            if total_distance >= lookahead_distance:
                return index

        return index

    def route_lateral_error_at(self, index, x, y):
        if not hasattr(self, 'X_points') or len(self.X_points) == 0:
            return 0.0

        index = max(0, min(index, len(self.X_points) - 1))
        dx = x - self.X_points[index]
        dy = y - self.Y_points[index]
        route_heading = self.route_heading_at(index)
        return -dx * math.sin(route_heading) + dy * math.cos(route_heading)

    def lateral_error_to_line(self, line_x, line_y, line_heading, x, y):
        dx = x - line_x
        dy = y - line_y
        return -dx * math.sin(line_heading) + dy * math.cos(line_heading)

    def prepare_curve_exit_line(self):
        max_search = int(self.get_param("curve_exit_straight_search"))
        ref_idx = self.vehpos_initial_index

        for distance in range(2, max_search + 1):
            idx = self.find_lookahead_index_by_distance(self.vehpos_initial_index, float(distance))
            if self.compute_route_curve_rate(idx, 8) < self.get_param("curve_exit_threshold"):
                ref_idx = idx
                break

        end_idx = self.find_lookahead_index_by_distance(
            ref_idx,
            self.get_param("curve_exit_line_heading_span"),
        )
        dx = self.X_points[end_idx] - self.X_points[ref_idx]
        dy = self.Y_points[end_idx] - self.Y_points[ref_idx]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            heading = self.route_heading_at(ref_idx)
        else:
            heading = math.atan2(dy, dx)

        self.curve_exit_line = (self.X_points[ref_idx], self.Y_points[ref_idx], heading)

    def target_on_curve_exit_line(self, preview_distance):
        if self.curve_exit_line is None:
            self.prepare_curve_exit_line()

        if self.curve_exit_line is None:
            return None

        line_x, line_y, heading = self.curve_exit_line
        along = (self.m_x - line_x) * math.cos(heading) + (self.m_y - line_y) * math.sin(heading)
        target_along = along + preview_distance
        return (
            line_x + target_along * math.cos(heading),
            line_y + target_along * math.sin(heading),
        )

    def apply_curve_outer_offset(self, target_index, curve_sign, inner_error):
        target_x = self.X_points[target_index]
        target_y = self.Y_points[target_index]
        if curve_sign == 0:
            return target_x, target_y

        route_heading = self.route_heading_at(target_index)
        curve_rate = getattr(self, "upcoming_curve_rate", 0.0)
        curve_offset = self.get_param("curve_outer_offset") + curve_rate * 2.0
        inner_offset = max(0.0, inner_error) * self.get_param("curve_inner_error_gain")
        offset = min(self.get_param("curve_max_outer_offset"), curve_offset + inner_offset)
        normal_x = -math.sin(route_heading)
        normal_y = math.cos(route_heading)

        # Positive curve_sign means a left turn; offset to the outside of the curve.
        return target_x - curve_sign * normal_x * offset, target_y - curve_sign * normal_y * offset

    def search_target_pos(self):
        """Select a stable lookahead target without reacting to vehicle yaw jitter."""
        base_preview = self.num_preview
        route_curve = getattr(self, "upcoming_curve_rate", 0.0)
        route_curve_sign = getattr(self, "upcoming_curve_sign", 0)
        lateral_error = self.route_lateral_error_at(self.vehpos_initial_index, self.m_x, self.m_y)
        inner_error = lateral_error * route_curve_sign

        was_in_curve = self.in_effective_curve
        if self.in_effective_curve:
            effective_curve = route_curve > self.get_param("curve_exit_threshold")
        else:
            effective_curve = route_curve > self.get_param("curve_preview_threshold")

        if route_curve_sign != 0 and route_curve > self.get_param("curve_exit_threshold"):
            effective_curve = effective_curve or inner_error > self.get_param("curve_inner_error_threshold")

        self.in_effective_curve = effective_curve
        self.just_exited_curve = was_in_curve and not effective_curve
        if self.just_exited_curve:
            self.curve_exit_hold_until = time.time() + self.get_param("curve_exit_recovery_duration")
            self.last_path_w = 0.0
            self.prepare_curve_exit_line()
        self.curve_inner_error = max(0.0, inner_error)

        if effective_curve:
            curve_factor = max(0.45, 1.0 - route_curve * 3.0 - max(0.0, inner_error) * 0.4)
            dynamic_preview = min(base_preview * curve_factor, self.get_param("curve_preview_distance"))
        elif self.is_curve_exit_recovering():
            dynamic_preview = self.get_param("curve_exit_recovery_preview")
        else:
            if self.m_v > 13.0:
                dynamic_preview = 16.0
            elif self.m_v > 10.0:
                dynamic_preview = 14.0
            else:
                dynamic_preview = max(base_preview, 10.0)
            self.cruise_speed = min(self.get_param("straight_speed"), self.cruise_speed + 0.15)

        # 速度因子：慢速时前视距离适当减小
        if self.m_v < 3:
            speed_factor = 0.6
        elif self.m_v < 5:
            speed_factor = 0.8
        elif self.m_v < 7:
            speed_factor = 0.9
        else:
            speed_factor = 1.0

        final_preview = dynamic_preview * speed_factor

        if effective_curve:
            final_preview = max(
                self.get_param("curve_min_preview_distance"),
                min(self.get_param("curve_preview_distance"), final_preview),
            )
        elif self.is_curve_exit_recovering():
            final_preview = self.get_param("curve_exit_recovery_preview")
        else:
            final_preview = max(7.0, min(self.get_param("straight_preview_distance"), final_preview))

        line_target = None
        if self.is_curve_exit_recovering():
            line_target = self.target_on_curve_exit_line(final_preview)

        if line_target is not None:
            target_x, target_y = line_target
        else:
            if not self.is_curve_exit_recovering():
                self.curve_exit_line = None
            target_pos_index = self.find_lookahead_index_by_distance(self.vehpos_initial_index, final_preview)
            target_x, target_y = self.apply_curve_outer_offset(
                target_pos_index,
                route_curve_sign if effective_curve else 0,
                inner_error,
            )
        self.targetPos_Info[0] = target_x
        self.targetPos_Info[1] = target_y

    def apply_interaction_control(self, m_x, m_y, m_yaw, v, w):
        """Multi-vehicle conflict manager with a fixed safety priority."""
        mode = "free"

        if self.escape_state is not None:
            v, w = self.persistent_blockage_recovery(m_x, m_y, m_yaw, v, w)
            self.last_interaction_mode = "blockage_escape"
            return self.clamp_control(v, w)

        front_vehicle, front_gap, front_lateral = self.find_closest_vehicle_in_corridor(
            m_x,
            m_y,
            m_yaw,
            max_distance=self.get_param("warning_distance"),
            half_width=self.get_param("road_width") / 2.0,
            max_heading_diff=math.radians(75.0),
        )

        if front_vehicle is not None and abs(front_lateral) < self.get_param("vehicle_half_width") * 1.4:
            front_speed = max(getattr(front_vehicle, "speed", 0.0), 0.0)
            emergency_distance = self.get_param("emergency_stop_distance")
            dynamic_gap = self.dynamic_follow_gap(front_speed)

            if front_gap <= emergency_distance:
                print(f"Emergency stop: front={getattr(front_vehicle, 'name', '')}, gap={front_gap:.2f}")
                self.last_interaction_mode = "emergency_stop"
                return 0.0, 0.0

            if front_gap < dynamic_gap:
                v = min(v, self.follow_speed(front_speed, front_gap, dynamic_gap))
                mode = "following"

        v, w = self.meeting_deadlock_resolution(m_x, m_y, m_yaw, v, w)
        if v < 0:
            self.last_interaction_mode = "meeting_deadlock"
            return self.clamp_control(v, w)
        if v <= 0.05:
            self.last_interaction_mode = "meeting_yield"
            return 0.0, 0.0

        before_intersection_v = v
        v, w = self.intersection_conflict_resolution(m_x, m_y, m_yaw, v, w)
        if v < before_intersection_v * 0.6:
            mode = "intersection_yield"
        if v <= 0.05:
            self.last_interaction_mode = mode
            return 0.0, 0.0

        before_lane_v = v
        v, w = self.lane_competition_avoidance(m_x, m_y, m_yaw, v, w)
        if v < before_lane_v * 0.7:
            mode = "lane_yield"

        if mode not in ("intersection_yield", "lane_yield") and v > self.get_param("lane_change_speed_min"):
            v, w = self.lane_change_overtake(m_x, m_y, m_yaw, v, w)

        v, w = self.following_conflict_avoidance(m_x, m_y, m_yaw, v, w)

        if v > 3.0 and self.lane_change_state == "checking":
            v, w = self.slow_vehicle_overtaking(m_x, m_y, m_yaw, v, w)

        v, w = self.persistent_blockage_recovery(m_x, m_y, m_yaw, v, w)
        self.last_interaction_mode = mode
        return self.clamp_control(v, w)

    def clamp_control(self, v, w):
        max_speed = self.get_param("max_control_speed")
        v = max(-2.5, min(max_speed, v))
        w = max(-1.0, min(1.0, w))
        return v, w

    def dynamic_follow_gap(self, front_speed):
        relative_speed = max(self.m_v - front_speed, 0.0)
        braking_gap = (relative_speed ** 2) / 4.0
        time_gap = self.m_v * self.get_param("follow_time_gap")
        return max(self.get_param("safe_distance"), time_gap + braking_gap + 2.0)

    def follow_speed(self, front_speed, gap, target_gap):
        gap_ratio = max(0.0, min(1.0, gap / max(target_gap, 0.1)))
        desired = front_speed * 0.9 + self.get_param("min_crawl_speed") * (1.0 - gap_ratio)
        return max(0.0, min(self.m_v, desired * gap_ratio))

    def vehicle_priority(self, name):
        text = str(name)
        if text.isdigit():
            return int(text)
        return sum(ord(ch) for ch in text)

    def is_right_side_vehicle(self, other, m_x, m_y, m_yaw):
        dx = other.x - m_x
        dy = other.y - m_y
        _, lateral = self.project_to_vehicle_frame(dx, dy, m_yaw)
        return lateral < 0

    def obstacle_avoidance(self, m_x, m_y, m_yaw, v, w):
        front_vehicle, front_gap, lateral_offset = self.find_closest_vehicle_in_corridor(
            m_x,
            m_y,
            m_yaw,
            max_distance=15.0,
            half_width=self.get_param("road_width") / 2.0,
            max_heading_diff=math.radians(75.0),
        )

        if front_vehicle is None:
            return v, w

        front_speed = max(getattr(front_vehicle, 'speed', 0.0), 0.0)
        same_lane_margin = self.get_param("vehicle_half_width") * 1.4
        dynamic_follow_distance = max(4.0, self.m_v * self.get_param("follow_time_gap") + 2.0)

        print(
            f"front vehicle detected: gap={front_gap:.2f}, lateral={lateral_offset:.2f}, "
            f"front_speed={front_speed:.2f}"
        )

        if abs(lateral_offset) > same_lane_margin:
            return v, w

        if front_gap < self.get_param("emergency_stop_distance"):
            print("Front vehicle too close, emergency stop")
            return 0, 0

        if front_gap < dynamic_follow_distance:
            follow_speed = min(v, max(front_speed - 0.3, self.get_param("min_crawl_speed")))
            speed_scale = max(0.35, min(1.0, front_gap / dynamic_follow_distance))
            adjusted_v = follow_speed * speed_scale
            print(f"Front vehicle in narrow corridor, following at {adjusted_v:.2f}")
            return adjusted_v, w

        if front_gap < dynamic_follow_distance * 1.5 and front_speed < v:
            adjusted_v = min(v, max(front_speed + 0.5, self.get_param("min_crawl_speed") * 1.5))
            print(f"Front vehicle ahead, proactively slowing to {adjusted_v:.2f}")
            return adjusted_v, w
        return v, w

    def lane_competition_avoidance(self, m_x, m_y, m_yaw, v, w):
        """Resolve lane competition by yielding to higher-priority nearby vehicles."""
        competition_distance = self.get_param("lane_merge_distance")
        neighbor_vehicle_data = self.udp_client.get_neighbor_vehicle_state()
        my_priority = self.vehicle_priority(self.vehicle_name)

        for other in neighbor_vehicle_data:
            dx = other.x - m_x
            dy = other.y - m_y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance > competition_distance:
                continue

            longitudinal, lateral = self.project_to_vehicle_frame(dx, dy, m_yaw)
            other_yaw = getattr(other, "yaw", 0.0)
            heading_diff = abs(self.normalize_angle(other_yaw - m_yaw))
            same_lane = abs(lateral) < self.get_param("vehicle_half_width") * 2.0
            merging_side_by_side = abs(longitudinal) < 4.0 and abs(lateral) < self.get_param("road_width")
            converging = heading_diff < math.radians(35.0) and (same_lane or merging_side_by_side)

            if converging:
                other_priority = self.vehicle_priority(getattr(other, "name", ""))
                if my_priority > other_priority or (my_priority == other_priority and longitudinal > 0):
                    print(f"Lane competition: yielding to {getattr(other, 'name', '')}")
                    return max(v * 0.35, self.get_param("min_crawl_speed")), w

        return v, w

    def following_conflict_avoidance(self, m_x, m_y, m_yaw, v, w):
        """跟车避障冲突解决：保持安全距离和预测性减速"""
        safe_distance = self.get_param("safe_distance")
        warning_distance = self.get_param("warning_distance")
        reaction_time = 1.5  # 反应时间

        neighbor_data = self.udp_client.get_neighbor_vehicle_state()

        # 寻找前方最近的车辆
        closest_vehicle = None
        min_distance = float('inf')

        for other in neighbor_data:
            dx = other.x - m_x
            dy = other.y - m_y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            # 检测前方车辆（相对角度小于30度）
            relative_angle = self.normalize_angle(math.atan2(dy, dx) - m_yaw)
            if abs(relative_angle) < math.radians(30) and distance < min_distance:
                min_distance = distance
                closest_vehicle = other

        if closest_vehicle:
            # 计算前车速度（假设可以从other获取）
            other_speed = getattr(closest_vehicle, 'speed', 5)
            relative_speed = self.m_v - other_speed

            # 计算安全距离：d = v_rel * t_reaction + v_rel^2/(2*a_max)
            dynamic_safe_distance = max(safe_distance, max(relative_speed, 0) * reaction_time)

            if min_distance < dynamic_safe_distance:
                # 需要减速
                if min_distance < self.get_param("emergency_stop_distance"):
                    print(f"Following conflict: emergency braking! distance={min_distance:.2f}")
                    return 0, w  # 紧急停车
                else:
                    # 平滑减速
                    distance_buffer = max(min_distance - safe_distance, 0.5)
                    deceleration = (max(relative_speed, 0) ** 2) / (2 * distance_buffer)
                    new_v = max(self.get_param("min_crawl_speed"), min(v, self.m_v - deceleration * 0.1))
                    print(f"Following conflict:减速 to {new_v:.2f}, distance={min_distance:.2f}")
                    return new_v, w
            elif min_distance < warning_distance and relative_speed > 0:
                # 预警：轻踩刹车提示
                print(f"Following conflict: warning, distance={min_distance:.2f}")
                return max(self.get_param("min_crawl_speed"), v * 0.85), w

        return v, w

    def intersection_conflict_resolution(self, m_x, m_y, m_yaw, v, w):
        """Solve unsignalized intersection conflicts with ETA and priority rules."""
        intersection_center = (
            self.get_param("intersection_center_x"),
            self.get_param("intersection_center_y"),
        )
        intersection_radius = self.get_param("intersection_radius")
        approach_radius = self.get_param("intersection_approach_radius")

        dx_int = intersection_center[0] - m_x
        dy_int = intersection_center[1] - m_y
        distance_to_intersection = math.sqrt(dx_int ** 2 + dy_int ** 2)

        if distance_to_intersection > approach_radius:
            self.intersection_yield_start_time = None
            return v, w

        my_eta = distance_to_intersection / max(v, self.get_param("min_crawl_speed"))
        my_priority = self.vehicle_priority(self.vehicle_name)
        now = time.time()

        for other in self.udp_client.get_neighbor_vehicle_state():
            dx_other = other.x - intersection_center[0]
            dy_other = other.y - intersection_center[1]
            other_distance_to_intersection = math.sqrt(dx_other ** 2 + dy_other ** 2)
            if other_distance_to_intersection > approach_radius:
                continue

            heading_diff = abs(self.normalize_angle(getattr(other, "yaw", 0.0) - m_yaw))
            paths_cross = heading_diff > math.radians(45.0)
            if not paths_cross and other_distance_to_intersection > intersection_radius:
                continue

            other_speed = max(getattr(other, "speed", 0.0), self.get_param("min_crawl_speed"))
            other_eta = other_distance_to_intersection / other_speed
            eta_close = abs(my_eta - other_eta) < self.get_param("intersection_time_gap")
            other_inside = other_distance_to_intersection < intersection_radius
            other_has_priority = (
                other_inside
                or self.is_right_side_vehicle(other, m_x, m_y, m_yaw)
                or self.vehicle_priority(getattr(other, "name", "")) < my_priority
            )

            if eta_close and other_has_priority:
                if self.intersection_yield_start_time is None:
                    self.intersection_yield_start_time = now

                if now - self.intersection_yield_start_time > self.get_param("yield_timeout"):
                    print("Intersection: yield timeout, creeping forward")
                    return min(v, self.get_param("min_crawl_speed")), w

                print(f"Intersection: yielding to {getattr(other, 'name', '')}")
                return 0.0, 0.0

        self.intersection_yield_start_time = None
        if distance_to_intersection < intersection_radius and v > self.get_param("min_crawl_speed"):
            return min(v * 1.1, self.get_param("max_control_speed")), w

        return v, w

    def meeting_deadlock_resolution(self, m_x, m_y, m_yaw, v, w):
        """Resolve narrow-road opposite meeting and deadlock."""
        meeting_distance = self.get_param("meeting_distance")
        road_width = self.get_param("road_width")

        neighbor_data = self.udp_client.get_neighbor_vehicle_state()
        meeting_vehicle = None
        min_distance = float('inf')

        for other in neighbor_data:
            dx = other.x - m_x
            dy = other.y - m_y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            other_yaw = getattr(other, 'yaw', 0)
            yaw_diff = abs(self.normalize_angle(other_yaw - m_yaw))

            longitudinal, lateral = self.project_to_vehicle_frame(dx, dy, m_yaw)

            if (
                distance < meeting_distance
                and longitudinal > 0
                and abs(lateral) < (road_width / 2.0 + 0.3)
                and abs(yaw_diff - math.pi) < 0.6
            ):
                meeting_vehicle = other
                min_distance = distance
                break

        if meeting_vehicle:
            current_time = time.time()
            if not hasattr(self, 'deadlock_start_time'):
                self.deadlock_start_time = None

            my_priority = self.vehicle_priority(self.vehicle_name)
            other_priority = self.vehicle_priority(getattr(meeting_vehicle, "name", ""))
            i_should_yield = my_priority > other_priority
            both_stopped = abs(v) < 0.5 and getattr(meeting_vehicle, 'speed', 0) < 0.5

            if both_stopped:
                if self.deadlock_start_time is None:
                    self.deadlock_start_time = current_time
                elif current_time - self.deadlock_start_time > self.get_param("deadlock_timeout"):
                    if i_should_yield and self.has_rear_space_for_escape(m_x, m_y, m_yaw):
                        print(f"Meeting deadlock: backing up for {getattr(meeting_vehicle, 'name', '')}")
                        return -2.0, 0.0
                    print("Meeting deadlock: keeping priority and creeping")
                    return min(max(v, self.get_param("min_crawl_speed")), 1.5), w
            else:
                self.deadlock_start_time = None

            if i_should_yield:
                print(f"Meeting vehicle: yielding to {getattr(meeting_vehicle, 'name', '')}")
                return 0.0 if min_distance < 6.0 else min(v, 1.5), w

            if min_distance < 6.0:
                print("Meeting vehicle: priority pass at crawl speed")
                return min(max(v, self.get_param("min_crawl_speed")), 2.0), w

        return v, w

    def slow_vehicle_overtaking(self, m_x, m_y, m_yaw, v, w):
        """低速车占道处理：安全超车策略"""
        overtake_distance = 20.0
        slow_threshold = 4.0  # 判定为低速的速度阈值

        neighbor_data = self.udp_client.get_neighbor_vehicle_state()
        slow_vehicle = None
        distance_to_slow = float('inf')

        for other in neighbor_data:
            dx = other.x - m_x
            dy = other.y - m_y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            # 检测前方车辆
            relative_angle = math.atan2(dy, dx) - m_yaw
            other_speed = getattr(other, 'speed', 10)

            if abs(relative_angle) < math.radians(30) and other_speed < slow_threshold and distance < overtake_distance:
                slow_vehicle = other
                distance_to_slow = distance
                break

        if slow_vehicle and self.m_v > 6:
            # 超车状态机
            if not hasattr(self, 'overtaking_state'):
                self.overtaking_state = 'checking'  # checking, preparing, overtaking, returning
                self.overtaking_start_x = m_x

            if self.overtaking_state == 'checking':
                # 检查是否满足超车条件
                if distance_to_slow < 15 and distance_to_slow > 8 and self.m_v > other_speed * 1.2:
                    self.overtaking_state = 'preparing'
                    print("Starting overtaking maneuver")

            elif self.overtaking_state == 'preparing':
                # 准备超车：稍微左移并加速
                w_adjustment = 0.15  # 向左转向
                v_adjustment = min(self.m_v * 1.3, 12)  # 加速
                print("Overtaking: preparing and accelerating")

                # 检测是否完成准备
                if distance_to_slow < 10:
                    self.overtaking_state = 'overtaking'
                return v_adjustment, w_adjustment

            elif self.overtaking_state == 'overtaking':
                # 超车中：保持加速
                v_adjustment = min(self.m_v * 1.2, 12)
                print(f"Overtaking in progress, distance to slow: {distance_to_slow:.2f}")

                # 检测是否完成超车
                if distance_to_slow > 20 or (m_x - self.overtaking_start_x) > 30:
                    self.overtaking_state = 'returning'
                return v_adjustment, w

            elif self.overtaking_state == 'returning':
                # 返回原车道
                w_adjustment = -0.1
                print("Returning to lane after overtaking")

                # 重置状态
                if abs(w_adjustment) < 0.05:
                    self.overtaking_state = 'checking'
                    delattr(self, 'overtaking_start_x')
                return v, w_adjustment

        else:
            # 重置超车状态
            if hasattr(self, 'overtaking_state'):
                delattr(self, 'overtaking_state')
                if hasattr(self, 'overtaking_start_x'):
                    delattr(self, 'overtaking_start_x')

        return v, w

if __name__ == '__main__':
    control = Control()
    control.udp_client.start()
    control.control_node()

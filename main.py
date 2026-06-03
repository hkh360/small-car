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

        net = "TYe74XzzzAIe3B7ybmZBCHCCEpLe,172.25.73.110,4088,4089"
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
        self.num_preview = 6
        self.targetPos_Info = [0.0, 0.0]
        self.Y_points = []
        self.X_points = []
        self.control_rate = 10  # hz
        self.wheel_base = 2.7

    def control_node(self):
        start_time = time.time()
        self.load_route('exp_routes/Big.json')

        # 初始化状态变量
        self.deadlock_start_time = None

        while True:
            vehicle_data = self.udp_client.get_vehicle_state()
            self.m_x = vehicle_data.x
            self.m_y = vehicle_data.y
            self.m_yaw = vehicle_data.yaw / 180 * math.pi
            self.m_v = 10
            self.update_vehpos_index()
            self.search_target_pos()

            # 计算原始控制指令
            v, w = self.calc_pure_pursuit(self.m_x, self.m_y, self.m_yaw, self.targetPos_Info)

            # 按优先级依次应用各种避障策略
            # 1. 基础避障（最高优先级）
            v, w = self.obstacle_avoidance(self.m_x, self.m_y, self.m_yaw, v, w)

            # 2. 紧急情况处理
            v, w = self.following_conflict_avoidance(self.m_x, self.m_y, self.m_yaw, v, w)

            # 3. 死锁解除
            v, w = self.meeting_deadlock_resolution(self.m_x, self.m_y, self.m_yaw, v, w)

            # 4. 路口冲突
            v, w = self.intersection_conflict_resolution(self.m_x, self.m_y, self.m_yaw, v, w)

            # 5. 车道竞争
            v, w = self.lane_competition_avoidance(self.m_x, self.m_y, self.m_yaw, v, w)

            # 6. 低速车超车（最低优先级）
            if v > 3:  # 只有在正常行驶时才考虑超车
                v, w = self.slow_vehicle_overtaking(self.m_x, self.m_y, self.m_yaw, v, w)

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

    def calc_pure_pursuit(self, m_x, m_y, m_yaw, target_pos):
        ###################################
        ##输出控制：速度（v）和转向角（steering_angle）
        ##请在此处补全纯跟踪算法的核心计算公式
        ##所需参数：
        ## m_x, m_y          --车辆位置
        ## m_yaw             --车辆航向角
        ## target_pos        --目标点
        ## self.wheel_base   --轴距

        # 计算车辆到目标点的距离（前视距离）
        dx = target_pos[0] - m_x
        dy = target_pos[1] - m_y
        Ld = math.sqrt(dx ** 2 + dy ** 2)  # 前视距离

        # 计算目标点相对于车辆航向角的角度
        alpha = math.atan2(dy, dx) - m_yaw

        # 纯跟踪算法公式：转向角 = atan2(2 * 轴距 * sin(alpha), 前视距离)
        if Ld > 0.001:  # 避免除零
            steering_angle = math.atan2(2 * self.wheel_base * math.sin(alpha), Ld)
        else:
            steering_angle = 0

        # 速度控制（可根据曲率或前视距离调整，这里使用固定速度）
        v = self.m_v

        ###################################
        w = v * math.tan(steering_angle) / self.wheel_base
        return v, w

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
        if min_distance > 25:
            self.search_vehicle_initial_index()
        else:
            self.vehpos_initial_index = nearest_index

    def search_target_pos(self):
        # 根据速度动态调整前视距离
        # 速度越快，前视距离越大；弯道时减小前视距离
        base_preview = self.num_preview

        # 计算当前曲率（通过前后帧航向角变化）
        if hasattr(self, 'last_yaw'):
            yaw_rate = abs(self.m_yaw - self.last_yaw)
            if yaw_rate > 0.03:  # 正在转弯
                # 弯道时减小前视距离，让车辆更早转向
                curve_factor = max(0.3, 1.0 - yaw_rate * 5)
                dynamic_preview = base_preview * curve_factor
            else:
                dynamic_preview = base_preview
        else:
            dynamic_preview = base_preview

        self.last_yaw = self.m_yaw

        # 根据速度调整
        speed_factor = min(1.5, max(0.5, self.m_v / 8.0))
        final_preview = dynamic_preview * speed_factor
        final_preview = max(6, min(18, final_preview))  # 限制范围

        target_x = self.m_x + final_preview * math.cos(self.m_yaw)
        target_y = self.m_y + final_preview * math.sin(self.m_yaw)
        target_pos_index = self.find_nearest_point_index(target_x, target_y)
        self.targetPos_Info[0] = self.X_points[target_pos_index]
        self.targetPos_Info[1] = self.Y_points[target_pos_index]

    def obstacle_avoidance(self, m_x, m_y, m_yaw, v, w):
        front_distance = 10.0
        front_half_angle = math.radians(45.0)
        neighbor_vehicle_data = self.udp_client.get_neighbor_vehicle_state()

        for other_vehicle in neighbor_vehicle_data:
            dx = other_vehicle.x - m_x
            dy = other_vehicle.y - m_y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            if distance > front_distance:
                continue

            target_yaw = math.atan2(dy, dx)
            relative_angle = target_yaw - m_yaw
            if relative_angle < -math.pi:
                relative_angle += 2 * math.pi
            elif relative_angle > math.pi:
                relative_angle -= 2 * math.pi

            print("obstacle distance:", distance, "relative angle:", relative_angle)
            if abs(relative_angle) <= front_half_angle:
                print("front obstacle detected, stop")
                return 0, 0
        return v, w

    def lane_competition_avoidance(self, m_x, m_y, m_yaw, v, w):
        """车道竞争解决策略：基于优先级和速度调整"""
        competition_distance = 8.0
        neighbor_vehicle_data = self.udp_client.get_neighbor_vehicle_state()

        # 定义优先级：车辆名称数字越小优先级越高
        my_priority = int(self.vehicle_name) if self.vehicle_name.isdigit() else 0

        for other in neighbor_vehicle_data:
            dx = other.x - m_x
            dy = other.y - m_y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance > competition_distance:
                continue

            # 检测是否在同一车道（横向距离小于1.5米）
            lateral_distance = abs(dy * math.cos(m_yaw) - dx * math.sin(m_yaw))

            if lateral_distance < 1.5 and distance < competition_distance:
                other_priority = int(other.name) if hasattr(other, 'name') and str(other.name).isdigit() else 10

                if my_priority > other_priority:
                    # 低优先级让行
                    print("Lane competition: yielding to higher priority vehicle")
                    return max(v * 0.3, 2), w  # 减速让行
                else:
                    # 高优先级维持速度
                    print("Lane competition: maintaining priority")
                    pass

        return v, w

    def following_conflict_avoidance(self, m_x, m_y, m_yaw, v, w):
        """跟车避障冲突解决：保持安全距离和预测性减速"""
        safe_distance = 8.0  # 安全距离
        warning_distance = 15.0  # 预警距离
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
            relative_angle = math.atan2(dy, dx) - m_yaw
            if abs(relative_angle) < math.radians(30) and distance < min_distance:
                min_distance = distance
                closest_vehicle = other

        if closest_vehicle:
            # 计算前车速度（假设可以从other获取）
            other_speed = getattr(closest_vehicle, 'speed', 5)
            relative_speed = self.m_v - other_speed

            # 计算安全距离：d = v_rel * t_reaction + v_rel^2/(2*a_max)
            dynamic_safe_distance = max(safe_distance, relative_speed * reaction_time)

            if min_distance < dynamic_safe_distance:
                # 需要减速
                if min_distance < safe_distance:
                    print(f"Following conflict: emergency braking! distance={min_distance:.2f}")
                    return 0, w  # 紧急停车
                else:
                    # 平滑减速
                    deceleration = (relative_speed ** 2) / (2 * (min_distance - safe_distance))
                    new_v = max(0, self.m_v - deceleration * 0.1)
                    print(f"Following conflict:减速 to {new_v:.2f}, distance={min_distance:.2f}")
                    return new_v, w
            elif min_distance < warning_distance and relative_speed > 0:
                # 预警：轻踩刹车提示
                print(f"Following conflict: warning, distance={min_distance:.2f}")
                return v * 0.85, w

        return v, w

    def intersection_conflict_resolution(self, m_x, m_y, m_yaw, v, w):
        """路口冲突解决：基于时间和优先级的通行权管理"""
        intersection_center = (50, 50)  # 示例路口中心坐标，实际应根据地图设置
        intersection_radius = 10.0

        # 计算到路口的距离
        dx_int = intersection_center[0] - m_x
        dy_int = intersection_center[1] - m_y
        distance_to_intersection = math.sqrt(dx_int ** 2 + dy_int ** 2)

        if distance_to_intersection > intersection_radius:
            return v, w

        # 收集路口内的其他车辆
        vehicles_in_intersection = []
        neighbor_data = self.udp_client.get_neighbor_vehicle_state()

        for other in neighbor_data:
            dx_other = other.x - intersection_center[0]
            dy_other = other.y - intersection_center[1]
            if math.sqrt(dx_other ** 2 + dy_other ** 2) < intersection_radius:
                vehicles_in_intersection.append(other)

        # 路口通行规则：让行右侧车辆
        my_time_to_intersection = distance_to_intersection / max(v, 0.1)

        for other in vehicles_in_intersection:
            # 计算其他车辆的行驶方向
            other_yaw = getattr(other, 'yaw', 0)
            angle_diff = abs(m_yaw - other_yaw)

            # 判断是否路径交叉（方向差在60-120度之间）
            if 60 < math.degrees(angle_diff) < 120:
                # 简化规则：让行右侧车辆（根据相对位置判断）
                cross_product = dx_int * math.sin(other_yaw) - dy_int * math.cos(other_yaw)

                if cross_product < 0:  # 右侧有车
                    print("Intersection: yielding to vehicle on right")
                    return max(v * 0.2, 2), w
                elif my_time_to_intersection < 2.0:
                    # 快速通过路口
                    print("Intersection: passing through quickly")
                    return min(v * 1.2, 12), w

        return v, w

    def meeting_deadlock_resolution(self, m_x, m_y, m_yaw, v, w):
        """对向会车死锁解决：基于策略的让行机制"""
        meeting_distance = 12.0
        road_width = 3.0  # 道路宽度

        neighbor_data = self.udp_client.get_neighbor_vehicle_state()
        meeting_vehicle = None
        min_distance = float('inf')

        for other in neighbor_data:
            dx = other.x - m_x
            dy = other.y - m_y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            # 检测对向车辆（航向角差在150-210度之间）
            other_yaw = getattr(other, 'yaw', 0)
            yaw_diff = abs((other_yaw - m_yaw) % (2 * math.pi))

            if distance < meeting_distance and math.pi - 0.5 < yaw_diff < math.pi + 0.5:
                meeting_vehicle = other
                min_distance = distance
                break

        if meeting_vehicle:
            # 判断停车时间
            current_time = time.time()
            if not hasattr(self, 'deadlock_start_time'):
                self.deadlock_start_time = None

            # 检查是否陷入死锁（双方都静止超过3秒）
            if v < 0.5 and getattr(meeting_vehicle, 'speed', 0) < 0.5:
                if self.deadlock_start_time is None:
                    self.deadlock_start_time = current_time
                elif current_time - self.deadlock_start_time > 3.0:
                    # 死锁解除：根据车辆编号决定谁后退
                    my_num = int(self.vehicle_name) if self.vehicle_name.isdigit() else 0
                    other_num = int(getattr(meeting_vehicle, 'name', '0')) if str(
                        getattr(meeting_vehicle, 'name', '0')).isdigit() else 1

                    if my_num < other_num:
                        print("Deadlock resolved: I will move forward")
                        return v, w  # 前进
                    else:
                        print("Deadlock resolved: I will back up")
                        return -2, 0  # 后退
            else:
                # 未死锁，正常让行
                self.deadlock_start_time = None
                if min_distance < 5.0:
                    # 寻找避让点
                    print("Meeting vehicle detected, slowing down to pass")
                    return max(v * 0.3, 2), w

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

"""
Robot Navigator for RoboMaster EP
ควบคุมการเคลื่อนที่ของหุ่นยนต์ RoboMaster EP ตามเส้นทาง BFS
โดยประมวลผลข้อมูลจาก:
1. ToF Sensor: วัดระยะตรวจจับสิ่งกีดขวางด้านหน้าก่อนเคลื่อนที่เข้าสู่แต่ละช่อง (หน่วย mm)
2. IMU Sensor: ติดตามและควบคุมมุมหัน (Yaw) และความเร่ง (Acc/Gyro) ให้เดินตรง 60 cm ต่อช่อง
3. Gimbal Lock: ล็อก Gimbal ให้อยู่กึ่งกลางตรงกับหน้ารถ (CHASSIS_LEAD) เพื่อให้ ToF ส่องตรง
4. CSV Logging: บันทึกข้อมูลค่าเซนเซอร์จริงในแต่ละรอบ *เฉพาะเมื่อรันกับหุ่นจริงเท่านั้น*
"""

import time
import math
from typing import List, Tuple, Optional, Callable
from csv_logger import NavigationLogger


class RobotNavigator:
    def __init__(
        self,
        conn_type: str = "sta",
        sim_mode: bool = False,
        cell_size_m: float = 0.60,
        obstacle_threshold_mm: float = 420.0,
        logger: Optional[NavigationLogger] = None
    ):
        """
        :param conn_type: 'sta' (เชื่อมต่อผ่าน Router/WiFi) หรือ 'ap' (เชื่อมต่อกับ WiFi ของหุ่นโดยตรง)
        :param sim_mode: True เพื่อรันโหมดจำลอง (ไม่ต้องต่อหุ่นจริง)
        :param cell_size_m: ขนาดแต่ละช่อง = 0.60 ม. (60 ซม.) ตาม RQM.md
        :param obstacle_threshold_mm: ระยะ ToF ขั้นต่ำที่ถือว่ามีสิ่งกีดขวางขวางหน้า (mm)
        :param logger: อ็อบเจกต์ NavigationLogger สำหรับบันทึกข้อมูลแต่ละรอบลง CSV
        """
        self.conn_type = conn_type
        self.sim_mode = sim_mode
        self.cell_size_m = cell_size_m
        self.obstacle_threshold_mm = obstacle_threshold_mm
        self.logger = logger

        # เงื่อนไข: บันทึก CSV เฉพาะเมื่อรันกับหุ่นจริงเท่านั้น (ยกเว้นมีการเปิด allow_sim_logging ชัดเจน)
        if self.logger:
            if self.sim_mode and not getattr(self.logger, 'allow_sim_logging', False):
                self.logger.enabled = False
            else:
                self.logger.enabled = True

        # ขนาดหุ่นตาม RQM.md (33 cm x 25 cm)
        self.robot_length_m = 0.33
        self.robot_width_m = 0.25

        # สถานะตำแหน่งและทิศทาง
        # ทิศทางอ้างอิง: 0 deg = ขวา (+x / East), 90 deg = ขึ้น (+y / North),
        # 180 deg = ซ้าย (-x / West), -90 deg = ลง (-y / South)
        self.current_heading_deg = 0.0
        self.current_pos = (1, 4)

        # ข้อมูลเซนเซอร์ปัจจุบัน
        self.latest_tof_distances = [9999, 9999, 9999, 9999]  # mm
        self.latest_attitude = (0.0, 0.0, 0.0)  # yaw, pitch, roll (องศา)
        self.latest_imu = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # ax, ay, az, gx, gy, gz
        self.latest_gimbal_angle = (0.0, 0.0, 0.0, 0.0)  # pitch, yaw, pitch_ground, yaw_ground

        self.ep_robot = None
        self.ep_chassis = None
        self.ep_sensor = None
        self.ep_gimbal = None

    def connect(self) -> bool:
        """เชื่อมต่อกับหุ่นยนต์ RoboMaster EP"""
        if self.sim_mode:
            print("[INFO] เริ่มต้นการทำงานในโหมดจำลอง (Simulation Mode)")
            if self.logger and not getattr(self.logger, 'allow_sim_logging', False):
                self.logger.enabled = False
                print("[CSV INFO] โหมดจำลอง - ไม่บันทึก CSV (บันทึกเฉพาะเมื่อรันกับหุ่นจริงเท่านั้น)")
            elif self.logger and getattr(self.logger, 'allow_sim_logging', False):
                print(f"[CSV INFO] โหมดจำลอง (Sim Logging Enabled) - บันทึกผลลงใน {self.logger.filename}")
            self.lock_and_center_gimbal()
            return True

        try:
            from robomaster import robot
            print(f"[INFO] กำลังเชื่อมต่อ RoboMaster EP (โหมด: {self.conn_type})...")
            self.ep_robot = robot.Robot()
            self.ep_robot.initialize(conn_type=self.conn_type)

            self.ep_chassis = self.ep_robot.chassis
            self.ep_sensor = self.ep_robot.sensor
            self.ep_gimbal = self.ep_robot.gimbal

            # สมัครรับข้อมูลเซนเซอร์ IMU (Attitude), ToF และ Gimbal
            self.ep_chassis.sub_attitude(freq=10, callback=self._on_attitude_data)
            self.ep_chassis.sub_imu(freq=10, callback=self._on_imu_data)
            self.ep_sensor.sub_distance(freq=10, callback=self._on_tof_data)
            self.ep_gimbal.sub_angle(freq=10, callback=self._on_gimbal_angle)

            # รอข้อมูลเซนเซอร์ชุดแรก
            time.sleep(1.0)
            print("[INFO] เชื่อมต่อหุ่นยนต์และเปิดใช้งาน ToF + IMU สำเร็จ")
            print(f"[INFO] ค่าเริ่มต้น IMU Yaw: {self.latest_attitude[0]:.2f} deg | ToF ด้านหน้า: {self.latest_tof_distances[0]} mm")

            # ล็อก Gimbal ให้อยู่กึ่งกลางตามแนวรถ (CHASSIS_LEAD)
            self.lock_and_center_gimbal()

            # เปิดใช้งานการบันทึก CSV เนื่องจากเชื่อมต่อกับหุ่นจริงสำเร็จ
            if self.logger:
                self.logger.enabled = True
                self.logger._init_csv()
                print(f"[CSV LOG] ตรวจพบหุ่นยนต์จริง - เริ่มบันทึกค่าเซนเซอร์ลงใน: {self.logger.filename}")

            return True

        except Exception as e:
            print(f"[WARNING] ไม่สามารถเชื่อมต่อหุ่นยนต์จริงได้ ({e})")
            print("[INFO] สลับไปใช้โหมดจำลอง (Simulation Mode) โดยอัตโนมัติ")
            self.sim_mode = True
            if self.logger:
                self.logger.enabled = False
                print("[CSV INFO] โหมดจำลอง - ไม่บันทึก CSV (บันทึกเฉพาะเมื่อรันกับหุ่นจริงเท่านั้น)")
            self.lock_and_center_gimbal()
            return True

    def disconnect(self):
        """ยกเลิกการเชื่อมต่อและปิดระบบ"""
        if self.ep_robot:
            try:
                if self.ep_chassis:
                    self.ep_chassis.unsub_attitude()
                    self.ep_chassis.unsub_imu()
                if self.ep_sensor:
                    self.ep_sensor.unsub_distance()
                if self.ep_gimbal:
                    self.ep_gimbal.unsub_angle()
                from robomaster import robot
                self.ep_robot.set_robot_mode(mode=robot.FREE)
                self.ep_robot.close()
                print("[INFO] ปิดการเชื่อมต่อ RoboMaster สำเร็จ")
            except Exception as e:
                print(f"[ERROR] เกิดข้อผิดพลาดขณะปิดการเชื่อมต่อ: {e}")

    def lock_and_center_gimbal(self):
        """
        ล็อก Gimbal ให้อยู่กึ่งกลางตรงกับแนวตัวรถ (Pitch=0, Yaw=0) ในโหมด CHASSIS_LEAD
        เพื่อให้เซนเซอร์ ToF ที่ติดตั้งบน Gimbal ชี้ตรงไปข้างหน้าตลอดเวลา
        """
        print("  [GIMBAL LOCK] ดำเนินการล็อก Gimbal (Pitch=0 deg, Yaw=0 deg, Mode: CHASSIS_LEAD)...")
        if self.sim_mode or not self.ep_gimbal:
            time.sleep(0.2)
            print("  [GIMBAL LOCK] Gimbal ถูกล็อกกึ่งกลางตรงกับแนวรถเรียบร้อย (ToF ชี้ตรงไปข้างหน้า)")
            return

        try:
            from robomaster import robot
            recenter_action = self.ep_gimbal.recenter(pitch_speed=100, yaw_speed=100)
            recenter_action.wait_for_completed()
            self.ep_robot.set_robot_mode(mode=robot.CHASSIS_LEAD)
            print("  [GIMBAL LOCK] Gimbal ถูกล็อกกึ่งกลางในโหมด CHASSIS_LEAD เรียบร้อย (ToF ชี้ตรงไปข้างหน้า)")
        except Exception as e:
            print(f"  [GIMBAL WARNING] ไม่สามารถตั้งค่า Gimbal lock ได้: {e}")

    # Callback เซนเซอร์
    def _on_attitude_data(self, attitude_info):
        self.latest_attitude = attitude_info

    def _on_imu_data(self, imu_info):
        self.latest_imu = imu_info

    def _on_tof_data(self, sub_info):
        self.latest_tof_distances = [d if d > 0 else 9999 for d in sub_info]

    def _on_gimbal_angle(self, angle_info):
        self.latest_gimbal_angle = angle_info

    def get_front_tof_distance(self) -> float:
        """อ่านระยะ ToF ตัวหน้าสุด (เซนเซอร์ตัวที่ 1) ในหน่วย mm"""
        if self.sim_mode:
            return self._sim_get_front_tof()
        return float(self.latest_tof_distances[0])

    def _sim_get_front_tof(self) -> float:
        """จำลองค่า ToF ในโหมด Simulation"""
        rad = math.radians(self.current_heading_deg)
        dx = int(round(math.cos(rad)))
        dy = int(round(math.sin(rad)))
        target_pos = (self.current_pos[0] + dx, self.current_pos[1] + dy)

        obstacles = {(4, 4), (2, 3), (3, 2)}
        is_obstacle = (
            target_pos in obstacles or
            target_pos[0] < 1 or target_pos[0] > 4 or
            target_pos[1] < 1 or target_pos[1] > 4
        )
        if is_obstacle:
            return 380.0
        return 1200.0

    def turn_to_heading(self, target_heading_deg: float):
        """หมุนหุ่นยนต์ไปยังทิศทางเป้าหมาย พร้อมตรวจสอบด้วย IMU"""
        delta_turn = target_heading_deg - self.current_heading_deg
        while delta_turn > 180:
            delta_turn -= 360
        while delta_turn <= -180:
            delta_turn += 360

        if abs(delta_turn) < 1.0:
            return

        print(f"  [IMU ROTATE] กำลังหมุนตัว {delta_turn:+.1f} deg (จาก {self.current_heading_deg:.1f} deg สู่ {target_heading_deg:.1f} deg)...")

        if not self.sim_mode and self.ep_chassis:
            initial_yaw = self.latest_attitude[0]
            action = self.ep_chassis.move(x=0, y=0, z=delta_turn, z_speed=45)
            action.wait_for_completed()
            time.sleep(0.3)

            final_yaw = self.latest_attitude[0]
            measured_turn = final_yaw - initial_yaw
            while measured_turn > 180:
                measured_turn -= 360
            while measured_turn <= -180:
                measured_turn += 360
            print(f"  [IMU VERIFIED] มุมที่หมุนจริงจาก IMU: {measured_turn:+.2f} deg (เป้าหมาย: {delta_turn:+.1f} deg)")
        else:
            time.sleep(0.4)

        self.current_heading_deg = target_heading_deg
        if self.sim_mode:
            self.latest_attitude = (target_heading_deg, 0.0, 0.0)

        # ตรวจสอบการล็อกของ Gimbal ให้อยู่ตรงกลาง (0 deg) เสมอ
        if not self.sim_mode and self.ep_gimbal:
            gimbal_yaw_rel = self.latest_gimbal_angle[1]
            if abs(gimbal_yaw_rel) > 2.0:
                self.ep_gimbal.recenter(pitch_speed=100, yaw_speed=100).wait_for_completed()

    def move_forward_one_cell(self) -> bool:
        """
        ตรวจเช็ค ToF sensor ด้านหน้า และเดินหน้า 1 ช่อง (60 cm)
        :return: True หากเคลื่อนที่สำเร็จ, False หากพบสิ่งกีดขวาง
        """
        front_dist = self.get_front_tof_distance()
        print(f"  [ToF SENSOR] ระยะด้านหน้า = {front_dist:.1f} mm (เกณฑ์ความปลอดภัย: > {self.obstacle_threshold_mm} mm)")

        if front_dist < self.obstacle_threshold_mm:
            print(f"  [ToF WARNING] ตรวจพบสิ่งกีดขวางด้านหน้า! (ระยะ {front_dist:.1f} mm) หยุดเดินเพื่อความปลอดภัย!")
            return False

        print(f"  [CHASSIS MOVE] กำลังเดินหน้า {self.cell_size_m * 100:.0f} cm...")

        if not self.sim_mode and self.ep_chassis:
            yaw_before = self.latest_attitude[0]
            action = self.ep_chassis.move(x=self.cell_size_m, y=0, z=0, xy_speed=0.5)
            action.wait_for_completed()
            yaw_after = self.latest_attitude[0]
            drift = yaw_after - yaw_before
            print(f"  [IMU DRIFT] การเบี่ยงเบนของ Yaw ระหว่างเดินหน้า: {drift:+.2f} deg")
        else:
            time.sleep(0.5)

        print(f"  [SUCCESS] เดินหน้าครบ {self.cell_size_m * 100:.0f} cm เรียบร้อย")
        return True

    def navigate_path(
        self,
        path: List[Tuple[int, int]],
        cell_costs: Optional[dict] = None,
        on_step_callback: Optional[Callable[[int, Tuple[int, int]], None]] = None
    ) -> bool:
        """
        นำทางหุ่นยนต์ตามรายการพิกัด path จาก BFS พร้อมบันทึกข้อมูลลง CSV เฉพาะเมื่อรันกับหุ่นจริง
        :param path: รายการพิกัด [(x1,y1), (x2,y2), ...]
        :param cell_costs: dict ของค่าใช้จ่ายแต่ละช่อง {(x,y): cost}
        :param on_step_callback: ฟังก์ชัน callback เมื่อเดินถึงแต่ละช่อง (step_idx, pos)
        """
        if not path or len(path) < 1:
            print("[ERROR] เส้นทางไม่ถูกต้อง")
            return False

        self.current_pos = path[0]
        print(f"\n[START NAVIGATION] เริ่มนำทางจากจุด {self.current_pos} สู่เป้าหมาย {path[-1]} (ทั้งหมด {len(path)-1} ก้าว)")

        # ล็อก Gimbal ให้อยู่กึ่งกลางตรงกับหน้ารถ
        self.lock_and_center_gimbal()

        if on_step_callback:
            on_step_callback(0, self.current_pos)

        cumulative_cost = 0
        total_dist_m = 0.0

        # บันทึกข้อมูลเริ่มต้น (รอบที่ 0 / Start) เมื่อเชื่อมต่อหุ่นจริง
        if self.logger and self.logger.enabled:
            start_cost = cell_costs.get(self.current_pos, 0) if cell_costs else 0
            self.logger.log_step(
                step_round=0,
                pos_x=self.current_pos[0],
                pos_y=self.current_pos[1],
                action="START",
                target_heading_deg=self.current_heading_deg,
                imu_yaw_deg=self.latest_attitude[0],
                imu_drift_deg=0.0,
                tof_distance_mm=self.get_front_tof_distance(),
                gimbal_pitch_deg=self.latest_gimbal_angle[0],
                gimbal_yaw_deg=self.latest_gimbal_angle[1],
                cell_cost=start_cost,
                cumulative_cost=cumulative_cost,
                distance_m=total_dist_m,
                status="SUCCESS",
                imu_raw=self.latest_imu
            )

        for step_idx in range(1, len(path)):
            next_pos = path[step_idx]
            print(f"\n--- รอบที่ {step_idx}: จาก {self.current_pos} -> {next_pos} ---")

            # คำนวณทิศทางที่ต้องมุ่งหน้าไป
            dx = next_pos[0] - self.current_pos[0]
            dy = next_pos[1] - self.current_pos[1]

            if dx == 1 and dy == 0:
                target_heading = 0.0     # East (ขวา)
            elif dx == -1 and dy == 0:
                target_heading = 180.0   # West (ซ้าย)
            elif dx == 0 and dy == 1:
                target_heading = 90.0    # North (ขึ้น)
            elif dx == 0 and dy == -1:
                target_heading = -90.0   # South (ลง)
            else:
                print(f"[ERROR] ไม่สามารถเคลื่อนที่ทะแยงหรือเกิน 1 ช่องได้: {self.current_pos} -> {next_pos}")
                return False

            # 1. หมุนตัวไปในทิศที่จะเดินโดยอิง IMU
            self.turn_to_heading(target_heading)

            # 2. ตรวจสอบเซนเซอร์ ToF ด้านหน้า และเดินหน้า 60 cm
            tof_dist = self.get_front_tof_distance()
            success = self.move_forward_one_cell()

            step_cost = cell_costs.get(next_pos, 0) if cell_costs else 0
            if success:
                cumulative_cost += step_cost
                total_dist_m += self.cell_size_m
                self.current_pos = next_pos
                status = "GOAL_REACHED" if step_idx == len(path) - 1 else "SUCCESS"
            else:
                status = "OBSTACLE_BLOCKED"

            # 3. บันทึกข้อมูลของรอบนี้ลงใน CSV เมื่อเชื่อมต่อหุ่นจริง
            if self.logger and self.logger.enabled:
                yaw_drift = self.latest_attitude[0] - target_heading
                while yaw_drift > 180:
                    yaw_drift -= 360
                while yaw_drift <= -180:
                    yaw_drift += 360

                self.logger.log_step(
                    step_round=step_idx,
                    pos_x=self.current_pos[0],
                    pos_y=self.current_pos[1],
                    action="REACH_GOAL" if step_idx == len(path) - 1 else "MOVE_TO_CELL",
                    target_heading_deg=target_heading,
                    imu_yaw_deg=self.latest_attitude[0],
                    imu_drift_deg=yaw_drift,
                    tof_distance_mm=tof_dist,
                    gimbal_pitch_deg=self.latest_gimbal_angle[0],
                    gimbal_yaw_deg=self.latest_gimbal_angle[1],
                    cell_cost=step_cost,
                    cumulative_cost=cumulative_cost,
                    distance_m=total_dist_m,
                    status=status,
                    imu_raw=self.latest_imu
                )

            if not success:
                print(f"[ABORT] ยกเลิกการเดิน ณ ตำแหน่ง {self.current_pos} เนื่องจากมีสิ่งกีดขวาง")
                return False

            if on_step_callback:
                on_step_callback(step_idx, self.current_pos)

        print(f"\n[GOAL REACHED] หุ่นยนต์เดินทางถึงเป้าหมาย {self.current_pos} สำเร็จสมบูรณ์!")
        return True

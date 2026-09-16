"""
CSV Logger for RoboMaster EP Navigation
บันทึกข้อมูลการเคลื่อนที่และค่าเซนเซอร์ในแต่ละรอบ (Round / Step) ลงในไฟล์ CSV
*เงื่อนไข: บันทึกเฉพาะเมื่อรันกับหุ่นยนต์จริงเท่านั้น (Real Robot Only)
*รองรับการบันทึกทับซ้อน/สะสมหลายรอบ (Append Mode ไม่ลบข้อมูลเดิม) เช่น การทดสอบ 3 รอบ
"""

import os
import csv
import datetime
from typing import Optional, Dict, Any, List


class NavigationLogger:
    def __init__(
        self,
        filename: str = "navigation_log.csv",
        enabled: bool = True,
        append: bool = True,
        run_id: Optional[int] = None
    ):
        """
        :param filename: ชื่อไฟล์ CSV ที่ต้องการบันทึก
        :param enabled: เปิดใช้งานการบันทึกหรือไม่ (หาก False จะไม่สร้างหรือเขียนไฟล์)
        :param append: บันทึกต่อท้ายไฟล์เดิมโดยไม่ลบ (True เพื่อเก็บข้อมูลซ้อนหลายรอบได้)
        :param run_id: หมายเลขรอบที่ต้องการระบุเจาะจง (หากเป็น None จะตรวจจับและนับต่ออัตโนมัติ)
        """
        self.filename = filename
        self.enabled = enabled
        self.append = append
        self.run_id = run_id
        self.current_run = 1
        self.headers = [
            "run",
            "timestamp",
            "step_round",
            "pos_x",
            "pos_y",
            "action",
            "target_heading_deg",
            "imu_yaw_deg",
            "imu_drift_deg",
            "tof_distance_mm",
            "gimbal_pitch_deg",
            "gimbal_yaw_deg",
            "cell_cost",
            "cumulative_cost",
            "distance_m",
            "status",
            "acc_x",
            "acc_y",
            "acc_z",
            "gyro_x",
            "gyro_y",
            "gyro_z"
        ]
        self.records: List[Dict[str, Any]] = []

        if self.enabled:
            self._init_csv()

    def _init_csv(self):
        """
        จัดการไฟล์ CSV:
        - หากไฟล์ยังไม่มี หรือถูกสั่งไม่ให้ append: สร้างไฟล์ใหม่พร้อมเขียน Header
        - หากไฟล์มีอยู่แล้วและ append=True: ตรวจสอบ Header และหาหมายเลขรอบล่าสุดเพื่อรันรอบถัดไป
        """
        dirname = os.path.dirname(os.path.abspath(self.filename))
        if dirname:
            os.makedirs(dirname, exist_ok=True)

        file_exists = os.path.exists(self.filename) and os.path.getsize(self.filename) > 0

        if not self.append or not file_exists:
            # สร้างไฟล์ใหม่ / เขียนทับเฉพาะเมื่อสั่ง append=False ชัดเจน
            with open(self.filename, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(self.headers)
            self.current_run = 1 if self.run_id is None else self.run_id
            return

        # กรณี append=True และมีไฟล์เดิมอยู่แล้ว: อ่านข้อมูลเดิมและตรวจหาหมายเลขรอบ
        existing_rows = []
        has_run_column = False
        max_run = 0

        try:
            with open(self.filename, mode='r', newline='', encoding='utf-8') as f:
                reader = csv.reader(f)
                header_row = next(reader, None)
                if header_row:
                    cleaned_header = [h.strip() for h in header_row]
                    has_run_column = ("run" in cleaned_header)
                    for row in reader:
                        if not row or not any(row):
                            continue
                        existing_rows.append(row)
                        if has_run_column:
                            run_idx = cleaned_header.index("run")
                            try:
                                r_val = int(row[run_idx].strip())
                                if r_val > max_run:
                                    max_run = r_val
                            except (ValueError, IndexError):
                                pass

            # หากเดิมยังไม่มีคอลัมน์ run ให้ migrate ข้อมูลเดิมเป็น run=1
            if not has_run_column and existing_rows:
                migrated_rows = []
                for row in existing_rows:
                    migrated_rows.append([1] + row)
                with open(self.filename, mode='w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(self.headers)
                    writer.writerows(migrated_rows)
                max_run = 1

            if max_run == 0 and existing_rows:
                max_run = 1

        except Exception as e:
            print(f"[CSV LOGGER WARNING] ตรวจสอบไฟล์เดิมเกิดข้อผิดพลาด ({e}) จะเริ่มเขียนต่อท้าย")
            max_run = 0

        if self.run_id is not None:
            self.current_run = self.run_id
        else:
            self.current_run = max_run + 1

        print(f"[CSV LOGGER] โหมดสะสมข้อมูล (Append): มีข้อมูลเดิมแล้ว {max_run} รอบ -> เริ่มบันทึกรอบที่ {self.current_run}")

    def log_step(
        self,
        step_round: int,
        pos_x: int,
        pos_y: int,
        action: str,
        target_heading_deg: float,
        imu_yaw_deg: float,
        imu_drift_deg: float,
        tof_distance_mm: float,
        gimbal_pitch_deg: float,
        gimbal_yaw_deg: float,
        cell_cost: int,
        cumulative_cost: int,
        distance_m: float,
        status: str = "SUCCESS",
        imu_raw: Optional[tuple] = None,
        run_id: Optional[int] = None
    ):
        """บันทึกข้อมูล 1 แถวลงใน CSV เมื่อเปิดใช้งาน (รันกับหุ่นจริง)"""
        if not self.enabled:
            return

        run_val = run_id if run_id is not None else self.current_run
        now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        acc_x, acc_y, acc_z = 0.0, 0.0, 0.0
        gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0
        if imu_raw and len(imu_raw) >= 6:
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_raw[:6]

        row = [
            run_val,
            now_str,
            step_round,
            pos_x,
            pos_y,
            action,
            f"{target_heading_deg:.2f}",
            f"{imu_yaw_deg:.2f}",
            f"{imu_drift_deg:.2f}",
            f"{tof_distance_mm:.1f}",
            f"{gimbal_pitch_deg:.2f}",
            f"{gimbal_yaw_deg:.2f}",
            cell_cost,
            cumulative_cost,
            f"{distance_m:.2f}",
            status,
            f"{acc_x:.3f}",
            f"{acc_y:.3f}",
            f"{acc_z:.3f}",
            f"{gyro_x:.3f}",
            f"{gyro_y:.3f}",
            f"{gyro_z:.3f}"
        ]

        with open(self.filename, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(row)

        record_dict = dict(zip(self.headers, row))
        self.records.append(record_dict)

    def print_summary(self):
        """พิมพ์สรุปข้อมูลในไฟล์ CSV ออกทางหน้าจอเฉพาะรอบปัจจุบัน"""
        if not self.enabled or not os.path.exists(self.filename) or not self.records:
            return

        print("\n" + "=" * 75)
        print(f"📊 สรุปประวัติเซนเซอร์จริงในไฟล์ CSV (รอบที่ {self.current_run}): {self.filename}")
        print("=" * 75)
        print(f"{'รอบ':<5} {'Step':<6} {'พิกัด':<8} {'ทิศทาง':<10} {'ToF (mm)':<10} {'Cost':<6} {'Cost รวม':<10} {'สถานะ'}")
        print("-" * 75)
        for r in self.records:
            pos_str = f"({r['pos_x']},{r['pos_y']})"
            heading_str = f"{float(r['target_heading_deg']):.0f} deg"
            print(
                f"#{r['run']:<4} "
                f"#{r['step_round']:<5} "
                f"{pos_str:<8} "
                f"{heading_str:<10} "
                f"{float(r['tof_distance_mm']):<10.1f} "
                f"{r['cell_cost']:<6} "
                f"{r['cumulative_cost']:<10} "
                f"{r['status']}"
            )
        print("=" * 75 + "\n")

    def get_total_runs(self) -> int:
        """นับจำนวนรอบทั้งหมดที่มีอยู่ในไฟล์ CSV"""
        if not os.path.exists(self.filename):
            return 0
        runs_seen = set()
        with open(self.filename, mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                r = row.get("run")
                if r:
                    runs_seen.add(r)
        return len(runs_seen)

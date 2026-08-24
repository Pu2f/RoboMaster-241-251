"""
Classwork 5 - Stable Wall-Follower with Heading Correction
============================================================
- Right wall-follower ใช้ IR/Sharp (ระยะ) + TOF (หน้า) ร่วมกัน
- PID เต็มรูปแบบ (P+I+D) คุม 2 แกนพร้อมกัน:
    1) ระยะห่างจากกำแพงขวา -> strafe (y)
    2) heading (yaw จาก IMU) -> rotate (z)
- หยุดอัตโนมัติเมื่อ TOF หน้า <= 10 cm
- จับเวลาการเคลื่อนที่ + บันทึกค่า start/stop สำหรับกรอกตารางในรายงาน
"""

import time
# pyrefly: ignore [missing-import]
from robomaster import robot

# =====================================================================
# CONFIG — ต้อง calibrate ค่าพวกนี้จากการทดสอบจริงในสนามก่อนใช้งานจริง
# =====================================================================
BASE_SPEED       = 0.20     # m/s ความเร็วเดินหน้า
MAX_STRAFE       = 0.12     # m/s strafe สูงสุด
MAX_ROTATE       = 25.0     # deg/s rotate สูงสุด (กันหมุนแรงเกินตอนแก้ heading)

STOP_DISTANCE_MM = 100      # หยุดอัตโนมัติเมื่อ TOF หน้า <= 10 cm ตามโจทย์
DIST_TOLERANCE_CM = 12.0    # เกณฑ์ยอมรับ ±12 cm (ใช้รายงานผล ไม่ใช่ตัวหยุดโปรแกรม)
YAW_TOLERANCE_DEG = 12.0    # เกณฑ์ยอมรับ ±12°

CONTROL_DT       = 0.05     # ความถี่ loop (วินาที)

# --- PID gains: ระยะห่างจากกำแพง (strafe) ---
KP_DIST, KI_DIST, KD_DIST = 0.02, 0.001, 0.01   

# --- PID gains: heading correction (rotate) ---
KP_YAW, KI_YAW, KD_YAW = 1.2, 0.02, 0.3


# =====================================================================
# PID controller (generic, ใช้ซ้ำได้ทั้งสองแกน)
# =====================================================================
class PID:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error):
        now = time.time()
        dt = (now - self.prev_time) if self.prev_time is not None else CONTROL_DT
        dt = max(dt, 1e-3)   # กัน dt=0

        self.integral += error * dt
        # anti-windup: จำกัด integral ไม่ให้สะสมจนเกินขอบเขต output
        max_i = self.output_limit / max(self.ki, 1e-6) if self.ki > 0 else 0
        if self.ki > 0:
            self.integral = max(-max_i, min(max_i, self.integral))

        derivative = (error - self.prev_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(-self.output_limit, min(self.output_limit, output))

        self.prev_error = error
        self.prev_time = now
        return output


# =====================================================================
# SHARP CALIBRATION — ใช้สมการตัวอย่าง ต้องแทนค่าจริงจากการวัดของคุณ
# =====================================================================
def sharp_to_cm(adc_value):
    if adc_value is None:
        return None
    voltage = adc_value * (3.3 / 1023)
    if voltage <= 0.1:
        return 999.0
    distance_cm = 27.86 / (voltage - 0.42)   # TODO: แทนค่าจริงจาก calibration
    return max(distance_cm, 8.0)


# =====================================================================
# เชื่อมต่อหุ่นและตั้งค่า sensor
# =====================================================================
ep_robot = robot.Robot()
print("Connecting via AP Mode...")
ep_robot.initialize(conn_type="ap")

ep_chassis = ep_robot.chassis
sensor = ep_robot.sensor              # built-in TOF (หน้า)
sensor_adaptor = ep_robot.sensor_adaptor   # Sharp/IR ภายนอก
sensor_adaptor.start()

tof_front_mm = 9999

def on_tof_data(sub_info):
    global tof_front_mm
    if isinstance(sub_info, (list, tuple)) and len(sub_info) > 0:
        tof_front_mm = sub_info[0]
    elif isinstance(sub_info, (int, float)):
        tof_front_mm = sub_info

sensor.sub_distance(freq=10, callback=on_tof_data)

current_yaw = 0.0

def on_attitude(attitude_info):
    global current_yaw
    yaw, pitch, roll = attitude_info
    current_yaw = yaw

ep_chassis.sub_attitude(freq=20, callback=on_attitude)


def safe_get_adc(hub_id, port):
    try:
        return sensor_adaptor.get_adc(id=hub_id, port=port)
    except Exception as e:
        print(f"[WARN] ADC hub={hub_id} port={port} error: {e}")
        return None


def read_right_wall_cm():
    """ระยะห่างจากกำแพงขวา — แก้ hub id/port ให้ตรงกับ Sharp/IR ที่เสียบจริง"""
    return sharp_to_cm(safe_get_adc(hub_id=1, port=2))


def read_ir_slant():
    """อ่านค่า IR ตรวจจับทางเอียง (ค่า 0 = เจอสิ่งกีดขวาง/ทางเอียง) สมมติใช้พอร์ต 1"""
    try:
        io_val = sensor_adaptor.get_io(id=1, port=1)
        return io_val == 0
    except Exception:
        return False


def stop_robot():
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)


def wrap_angle(angle_deg):
    """ทำให้มุมอยู่ในช่วง [-180, 180] กัน error ตอน yaw ข้ามจาก 179 -> -179"""
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg


# =====================================================================
# วิ่ง 1 รอบการทดลอง — คืนค่า dict สำหรับกรอกตารางในรายงาน
# =====================================================================
def run_trial(trial_no, max_duration=60.0):
    dist_pid = PID(KP_DIST, KI_DIST, KD_DIST, MAX_STRAFE)
    yaw_pid = PID(KP_YAW, KI_YAW, KD_YAW, MAX_ROTATE)

    # --- Calibrate ค่าเริ่มต้น (เฉลี่ยหลายครั้งกันสัญญาณรบกวน) ---
    print(f"\n=== Trial {trial_no}: Calibrating start position... ===")
    samples_dist, samples_yaw = [], []
    for _ in range(10):
        d = read_right_wall_cm()
        if d is not None and d < 999:
            samples_dist.append(d)
        samples_yaw.append(current_yaw)
        time.sleep(0.05)

    if not samples_dist:
        print("[ERROR] อ่านค่ากำแพงขวาไม่ได้เลย เช็ค hub id/port ก่อนเริ่ม")
        return None

    target_dist_cm = sum(samples_dist) / len(samples_dist)
    target_yaw_deg = sum(samples_yaw) / len(samples_yaw)
    start_dist_cm = target_dist_cm
    start_yaw_deg = target_yaw_deg

    print(f"[START] wall_dist={start_dist_cm:.1f} cm | yaw={start_yaw_deg:.1f} deg")
    print(f"[GO] Base speed={BASE_SPEED} m/s | Stop when front <= {STOP_DISTANCE_MM} mm")

    start_time = time.time()
    stop_dist_cm = None
    stop_yaw_deg = None

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > max_duration:
                print("\n[TIMEOUT] เกินเวลาสูงสุดที่กำหนด หยุดฉุกเฉิน")
                break

            # --- อ่านค่า sensor ---
            wall_dist = read_right_wall_cm()
            yaw_now = current_yaw
            is_slant = read_ir_slant()

            if is_slant:
                # ถ้าเจอทางเอียง ให้หันหน้าออก (เลี้ยวซ้าย)
                strafe_y = 0.0
                z_correct = MAX_ROTATE  # เลี้ยวซ้ายด้วยความเร็วหมุนบวก
                target_yaw_deg = yaw_now  # อัปเดตเป้าหมายองศาใหม่ตามปัจจุบันเรื่อยๆ ไม่ให้สะบัดกลับตอนพ้นทางเอียง
                print(f"\rt={elapsed:5.1f}s  [SLANT DETECTED! Turning Left...]                                     ", end="")
            else:
                # --- PID ระยะห่างจากกำแพง (strafe) ---
                strafe_y = 0.0
                if wall_dist is not None and wall_dist < 999:
                    dist_error = wall_dist - target_dist_cm   # + = ไกลผนังขวาไป (ต้องแถไปทางขวา)
                    strafe_y = dist_pid.compute(dist_error)

                # --- PID heading (rotate) ---
                yaw_error = wrap_angle(target_yaw_deg - yaw_now)  # เอาค่าเป้าหมายตั้ง ลบด้วยค่าปัจจุบัน
                z_correct = yaw_pid.compute(yaw_error)

                # --- แสดงผล real-time ---
                dist_ok = "OK" if wall_dist is not None and abs(wall_dist - target_dist_cm) <= DIST_TOLERANCE_CM else "!!"
                yaw_ok = "OK" if abs(yaw_error) <= YAW_TOLERANCE_DEG else "!!"
                wd_str = f"{wall_dist:.1f}" if wall_dist is not None else "N/A"
                print(
                    f"\rt={elapsed:5.1f}s  ToF:{tof_front_mm:4}mm  "
                    f"WallDist:{wd_str}cm[{dist_ok}]  YawErr:{yaw_error:5.1f}deg[{yaw_ok}]  "
                    f"strafe:{strafe_y:+.3f}  z:{z_correct:+.2f}",
                    end=""
                )

            ep_chassis.drive_speed(x=BASE_SPEED, y=strafe_y, z=z_correct, timeout=CONTROL_DT)

            # --- เงื่อนไขหยุดอัตโนมัติ: TOF หน้า <= 10 cm ---
            if tof_front_mm <= STOP_DISTANCE_MM:
                stop_robot()
                stop_dist_cm = wall_dist
                stop_yaw_deg = yaw_now
                elapsed_time = time.time() - start_time
                print(f"\n[STOP] Front wall reached ({tof_front_mm} mm <= {STOP_DISTANCE_MM} mm)")
                break

            time.sleep(CONTROL_DT)

    except KeyboardInterrupt:
        stop_robot()
        print("\n[INTERRUPTED] หยุดโดยผู้ใช้")
        return None

    stop_robot()

    result = {
        "trial": trial_no,
        "start_dist_cm": round(start_dist_cm, 1),
        "stop_dist_cm": round(stop_dist_cm, 1) if stop_dist_cm else None,
        "dist_error": round((stop_dist_cm - start_dist_cm), 1) if stop_dist_cm else None,
        "start_yaw_deg": round(start_yaw_deg, 1),
        "stop_yaw_deg": round(stop_yaw_deg, 1) if stop_yaw_deg is not None else None,
        "yaw_error": round(wrap_angle(stop_yaw_deg - start_yaw_deg), 1) if stop_yaw_deg is not None else None,
        "stop_distance_cm": round(tof_front_mm / 10.0, 1),
        "elapsed_time_sec": round(elapsed_time, 2),
    }
    return result


def print_summary_table(results):
    print("\n" + "=" * 100)
    print("สรุปผลการทดลอง (คัดลอกไปกรอกในตารางรายงานได้เลย)")
    print("=" * 100)
    header = f"{'รอบ':<5}{'DistStart':<12}{'DistStop':<12}{'DistErr':<10}{'YawStart':<11}{'YawStop':<11}{'YawErr':<9}{'StopDist':<11}{'Time(s)':<9}"
    print(header)
    print("-" * 100)
    for r in results:
        if r is None:
            continue
        print(
            f"{r['trial']:<5}{r['start_dist_cm']:<12}{r['stop_dist_cm']:<12}{r['dist_error']:<10}"
            f"{r['start_yaw_deg']:<11}{r['stop_yaw_deg']:<11}{r['yaw_error']:<9}"
            f"{r['stop_distance_cm']:<11}{r['elapsed_time_sec']:<9}"
        )


# =====================================================================
# MAIN — รันหลายรอบตามตารางในรายงาน (ค่า default = 3 รอบ)
# =====================================================================
if __name__ == "__main__":
    NUM_TRIALS = 3
    all_results = []

    try:
        for i in range(1, NUM_TRIALS + 1):
            input(f"\n>>> วางหุ่นที่จุด START แล้วกด Enter เพื่อเริ่ม Trial {i}...")
            result = run_trial(i)
            all_results.append(result)
            time.sleep(1)

        print_summary_table(all_results)

    finally:
        try:
            sensor.unsub_distance()
        except Exception:
            pass
        try:
            ep_chassis.unsub_attitude()
        except Exception:
            pass
        try:
            sensor_adaptor.stop()
        except Exception:
            pass
        ep_robot.close()
        print("\nConnection closed.")
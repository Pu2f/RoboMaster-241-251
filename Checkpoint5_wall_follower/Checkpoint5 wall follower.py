"""
Checkpoint 4: Robot Controlling (IR+ToF)
============================================================
เส้นทาง (ตามไดอะแกรมหน้า 56):
  1) Start: เช็คสิ่งกีดขวางหน้าหุ่น ห่าง 10 cm (IR)
  2) เดินหน้า ผ่านทางลาด 2 ช่วง (10+10 cm) - ใช้ IMU pitch ตรวจว่ากำลังขึ้น/ลงลาด
  3) ถึงพื้นราบ -> เดินขนานกำแพงข้าง รักษาระยะ 10 cm (Sharp/IR analog, ใช้สมการ
     calibration จาก Checkpoint 3)
  4) เจอสิ่งกีดขวางหน้าสุดทาง -> หยุดแม่นยำที่ระยะ 5 cm (ToF)

ใช้ค่า calibration (slope, intercept) จาก Checkpoint 3 ของตัวเอง แทนที่ตัวอย่างในสไลด์
"""

import time
# pyrefly: ignore [missing-import]
from robomaster import robot

# =====================================================================
# CONFIG — ต้องแก้ค่าจริงจากการทดลองของคุณเองก่อนใช้งาน
# =====================================================================

# --- สมการ calibration จาก Checkpoint 3: ADC = SLOPE * distance_cm + INTERCEPT ---
# ตัวอย่างในสไลด์: ADC = 20.362*distance + 12.136 (R^2=0.9984)
# *** ต้องแทนค่า slope/intercept จากกราฟ ADC-Distance ของ Sharp ตัวคุณเอง ***
CAL_SLOPE = 20.362
CAL_INTERCEPT = 12.136

def adc_to_cm(adc_value):
    """แปลงค่า ADC กลับเป็นระยะทาง (cm) จากสมการ calibration เชิงเส้น"""
    if adc_value is None:
        return None
    distance_cm = (adc_value - CAL_INTERCEPT) / CAL_SLOPE
    return max(distance_cm, 0.0)


# --- ความเร็ว ---
BASE_SPEED      = 0.15     # m/s ความเร็วเดินหน้าปกติ (พื้นราบ)
RAMP_SPEED      = 0.10     # m/s ความเร็วช้าลงตอนขึ้น/ลงทางลาด (กันหุ่นเสียการทรงตัว)
MAX_STRAFE      = 0.10     # m/s strafe สูงสุดตอน wall-following
APPROACH_SPEED  = 0.06     # m/s ความเร็วช้าตอนเข้าใกล้จุดหยุดสุดท้าย (แม่นยำกว่า)

# --- ระยะเป้าหมาย ---
START_CHECK_CM      = 10.0   # ระยะสิ่งกีดขวางหน้าหุ่นตอน Start (แค่เช็ค ไม่ใช่จุดหยุด)
WALL_TARGET_CM       = 12.0  # ระยะเป้าหมายขนานกำแพงข้าง (ตอน wall-following)
WALL_TOLERANCE_CM    = 2.0   # ค่าเผื่อ error ยอมรับได้ตอนคุม strafe
FINAL_STOP_CM        = 7.0   # ระยะหยุดสุดท้าย (ToF) - จุดสำคัญที่สุดของด่านนี้
APPROACH_SLOWDOWN_CM = 20.0  # เริ่มชะลอความเร็วเมื่อ ToF เหลือระยะนี้ ก่อนถึงจุดหยุดจริง

# --- IMU / Pitch (ตรวจจับทางลาด) ---
PITCH_RAMP_THRESHOLD = 5.0   # องศา, pitch เกินนี้ถือว่ากำลังอยู่บนทางลาด
RAMP_TIMEOUT_SEC      = 15.0 # เวลาสูงสุดที่ยอมให้อยู่บนทางลาด (safety, กันค้าง)

# --- PID เดินขนานกำแพงและหมุน (ใช้ค่าจาก Classwork 5 เป็นฐาน) ---
KP_WALL, KI_WALL, KD_WALL = 0.02, 0.001, 0.008
KP_YAW, KI_YAW, KD_YAW = 1.2, 0.02, 0.3
MAX_ROTATE = 25.0

CONTROL_DT = 0.05


# =====================================================================
# PID controller
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
        dt = max(dt, 1e-3)

        self.integral += error * dt
        if self.ki > 0:
            max_i = self.output_limit / self.ki
            self.integral = max(-max_i, min(max_i, self.integral))

        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(-self.output_limit, min(self.output_limit, output))

        self.prev_error = error
        self.prev_time = now
        return output


# =====================================================================
# เชื่อมต่อหุ่นและตั้งค่า sensor
# =====================================================================
ep_robot = robot.Robot()
print("Connecting via AP Mode...")
ep_robot.initialize(conn_type="ap")

ep_chassis = ep_robot.chassis
sensor = ep_robot.sensor              # built-in ToF (หน้า)
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

current_pitch = 0.0
current_yaw = 0.0

def on_attitude(attitude_info):
    global current_pitch, current_yaw
    yaw, pitch, roll = attitude_info
    current_yaw = yaw
    current_pitch = pitch

ep_chassis.sub_attitude(freq=20, callback=on_attitude)


def safe_get_adc(hub_id, port):
    try:
        return sensor_adaptor.get_adc(id=hub_id, port=port)
    except Exception as e:
        print(f"[WARN] ADC hub={hub_id} port={port} error: {e}")
        return None

def safe_get_io(hub_id, port):
    try:
        return sensor_adaptor.get_io(id=hub_id, port=port)
    except Exception as e:
        print(f"[WARN] IO hub={hub_id} port={port} error: {e}")
        return None


def read_side_wall_cm():
    """ระยะห่างจากกำแพงข้าง (Sharp) — แก้ hub id/port ให้ตรงกับที่เสียบจริง"""
    return adc_to_cm(safe_get_adc(hub_id=1, port=1))

def read_front_ir_cm():
    """ระยะห่างจากสิ่งกีดขวางหน้า (Sharp/IR แยกจาก ToF) — ใช้เช็คตอน Start"""
    return adc_to_cm(safe_get_adc(hub_id=2, port=1))


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
# STEP 1: Start — เช็คว่าสิ่งกีดขวางหน้าหุ่นอยู่ที่ระยะ 10 cm ตามที่กำหนด
# =====================================================================
def step1_start_check():
    print("\n=== STEP 1: START CHECK ===")
    samples = []
    for _ in range(10):
        d = read_front_ir_cm()
        if d is not None:
            samples.append(d)
        time.sleep(0.05)

    if not samples:
        print("[WARN] อ่านค่า IR หน้าไม่ได้ ข้ามการเช็คนี้ไป")
        return

    avg_dist = sum(samples) / len(samples)
    print(f"ระยะสิ่งกีดขวางหน้าหุ่นที่วัดได้: {avg_dist:.1f} cm (คาดหวัง ~{START_CHECK_CM} cm)")
    if abs(avg_dist - START_CHECK_CM) > 3.0:
        print(f"[WARN] ระยะเริ่มต้นห่างจากที่คาดไว้เกิน 3cm ตรวจสอบตำแหน่งเริ่มต้นของหุ่นอีกครั้ง")
    else:
        print("[OK] ตำแหน่งเริ่มต้นถูกต้อง")


# =====================================================================
# STEP 2: เดินหน้าผ่านทางลาด — ใช้ IMU pitch ตรวจจับว่ากำลังอยู่บนทางลาดหรือไม่
# แล้วชะลอความเร็วเพื่อความเสถียร จนกลับมาที่พื้นราบ (pitch ~ 0)
# =====================================================================
def step2_cross_ramp():
    print("\n=== STEP 2: CROSSING RAMP ===")
    start_time = time.time()
    on_ramp_ever = False

    while True:
        elapsed = time.time() - start_time
        if elapsed > RAMP_TIMEOUT_SEC:
            print("\n[TIMEOUT] ข้ามทางลาดนานเกินกำหนด หยุดฉุกเฉิน")
            break

        # เช็ค IR ว่าเจอสิ่งกีดขวาง/กำแพงเอียงหรือไม่ (เหมือนใน Step 3)
        ir_val = safe_get_io(hub_id=1, port=3)
        if ir_val == 1:
            ep_chassis.drive_speed(x=0.0, y=0.0, z=MAX_ROTATE, timeout=CONTROL_DT)
            print("\r[IR=1] Turning away to avoid crash! (Step 2)             ", end="")
        else:
            is_on_ramp = abs(current_pitch) > PITCH_RAMP_THRESHOLD
            if is_on_ramp:
                on_ramp_ever = True
                speed = RAMP_SPEED
            else:
                speed = BASE_SPEED

            ep_chassis.drive_speed(x=speed, y=0, z=0, timeout=CONTROL_DT)

            print(
                f"\rpitch:{current_pitch:5.1f}deg  "
                f"{'[ON RAMP]' if is_on_ramp else '[FLAT]   '}  "
                f"speed:{speed:.2f}m/s  t={elapsed:.1f}s    ",
                end=""
            )

        # เงื่อนไขจบ step: เคยขึ้นทางลาดมาแล้ว (on_ramp_ever) และตอนนี้กลับมาราบแล้ว
        # ให้เดินต่ออีกนิดหน่อยเพื่อให้พ้นฐานทางลาดจริง ๆ ก่อนเข้าสู่ wall-following
        if on_ramp_ever and not is_on_ramp and elapsed > 1.0:
            print("\n[OK] ผ่านทางลาดแล้ว กลับสู่พื้นราบ")
            stop_robot()
            time.sleep(0.3)
            break

        time.sleep(CONTROL_DT)


# =====================================================================
# STEP 3: เดินขนานกำแพงข้าง รักษาระยะ 12 cm ด้วย PID (Sharp)
# เดินไปเรื่อย ๆ จนกว่า ToF หน้าจะเริ่มตรวจพบสิ่งกีดขวาง (เข้าสู่ STEP 4)
# =====================================================================
def step3_wall_parallel():
    print("\n=== STEP 3: WALL-PARALLEL WALKING (target=12cm) ===")
    wall_pid = PID(KP_WALL, KI_WALL, KD_WALL, MAX_STRAFE)
    yaw_pid = PID(KP_YAW, KI_YAW, KD_YAW, MAX_ROTATE)
    target_yaw = current_yaw  # ล็อคเป้าหมายการหันหน้าไว้ที่ตอนเริ่มเข้าสู่ Step 3

    while True:
        # เช็ค IR (ค่า 1 = เจอสิ่งกีดขวาง ให้หันหน้าออก)
        ir_val = safe_get_io(hub_id=1, port=3)
        
        if ir_val == 1:
            # หันหน้าออกเพื่อไม่ให้ชน (หมุนซ้าย)
            ep_chassis.drive_speed(x=0.0, y=0.0, z=MAX_ROTATE, timeout=CONTROL_DT)
            target_yaw = current_yaw  # อัปเดตเป้าหมายใหม่เรื่อยๆ เพื่อไม่ให้สะบัดกลับตอนหลุด
            print("\r[IR=1] Turning away to avoid crash!                      ", end="")
        else:
            wall_dist = read_side_wall_cm()
            strafe_y = 0.0
            if wall_dist is not None:
                error = wall_dist - WALL_TARGET_CM
                strafe_y = wall_pid.compute(error)

            # คุมหน้าให้ตรงเสมอ (Heading PID)
            yaw_error = wrap_angle(target_yaw - current_yaw)
            z_correct = yaw_pid.compute(yaw_error)

            ep_chassis.drive_speed(x=BASE_SPEED, y=strafe_y, z=z_correct, timeout=CONTROL_DT)

            wd_str = f"{wall_dist:.1f}" if wall_dist is not None else "N/A"
            status = "OK" if (wall_dist is not None and abs(wall_dist - WALL_TARGET_CM) <= WALL_TOLERANCE_CM) else "!!"
            print(
                f"\rWallDist:{wd_str}cm[{status}]  strafe:{strafe_y:+.3f}  z:{z_correct:+.2f}  "
                f"ToF_front:{tof_front_mm}mm      ",
                end=""
            )

        # เริ่มเจอสิ่งกีดขวางหน้าสุดทาง -> ออกจาก wall-following ไปเข้า step หยุดแม่นยำ
        if tof_front_mm <= APPROACH_SLOWDOWN_CM * 10:  # cm -> mm
            print(f"\n[OK] เริ่มตรวจพบสิ่งกีดขวางหน้าสุดทาง (ToF={tof_front_mm}mm) เข้าสู่โหมดหยุดแม่นยำ")
            break

        time.sleep(CONTROL_DT)


# =====================================================================
# STEP 4: หยุดแม่นยำที่ระยะ 7 cm จากสิ่งกีดขวางหน้าสุดทาง (ToF)
# ชะลอความเร็วลงเรื่อย ๆ เมื่อใกล้ถึงเป้าหมาย เพื่อไม่ให้เลยจุดหยุด
# =====================================================================
def step4_precise_stop():
    print("\n=== STEP 4: PRECISE STOP AT 7cm ===")
    target_mm = FINAL_STOP_CM * 10

    while True:
        current_mm = tof_front_mm
        remaining_mm = current_mm - target_mm

        if remaining_mm <= 0:
            stop_robot()
            print(f"\n[STOP] ถึงจุดหยุดแล้ว! ToF={current_mm}mm (target={target_mm}mm)")
            break

        # ยิ่งใกล้เป้าหมาย ยิ่งลดความเร็วลง (proportional to remaining distance)
        speed_ratio = min(1.0, remaining_mm / (APPROACH_SLOWDOWN_CM * 10))
        speed = max(0.02, APPROACH_SPEED * speed_ratio)

        ep_chassis.drive_speed(x=speed, y=0, z=0, timeout=CONTROL_DT)
        print(f"\rToF:{current_mm}mm  remaining:{remaining_mm}mm  speed:{speed:.3f}m/s", end="")

        time.sleep(CONTROL_DT)

    # ยืนยันระยะสุดท้ายหลังหยุดสนิท
    time.sleep(0.3)
    final_error_cm = (tof_front_mm - target_mm) / 10.0
    print(f"[RESULT] ระยะสุดท้าย: {tof_front_mm/10:.1f} cm | Error จากเป้าหมาย 7cm: {final_error_cm:+.1f} cm")


# =====================================================================
# MAIN — รันครบทุก step ตามลำดับในไดอะแกรม
# =====================================================================
if __name__ == "__main__":
    try:
        step1_start_check()
        input("\n>>> กด Enter เพื่อเริ่มเดินหน้า...")

        step2_cross_ramp()
        step3_wall_parallel()
        step4_precise_stop()

        print("\n\n=== CHECKPOINT 4 COMPLETE ===")

    except KeyboardInterrupt:
        stop_robot()
        print("\n[INTERRUPTED] หยุดโดยผู้ใช้")

    finally:
        stop_robot()
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
        print("Connection closed.")
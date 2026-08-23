import time
from robomaster import robot

# =====================================================================
# CONFIG — ตั้งค่าความเร็วและระบบ PID Control
# =====================================================================
BASE_SPEED      = 0.25     # ความเร็วเดินหน้าปกติ (m/s)
MAX_STRAFE      = 0.15     # ความเร็ว strafe ซ้าย-ขวา สูงสุด (m/s)
TURN_SPEED      = 45       # ความเร็วหมุนตอนเลี้ยว (deg/s)

# --- PID Gains สำหรับการเดินกึ่งกลาง (Corridor-centering) ---
KP_CENTER       = 0.015    # Proportional: ดึงกลับเมื่อห่างจากเป้าหมาย
KI_CENTER       = 0.001    # Integral: แก้ความคลาดเคลื่อนสะสม 
KD_CENTER       = 0.005    # Derivative: ลดการแกว่ง (ส่ายซ้าย-ขวา)

CORRIDOR_HALF_CM = 15.0    # ระยะครึ่งหนึ่งของทางเดิน (ซม.)

# --- ปรับแก้ใหม่ เพื่อป้องกันปัญหาหุ่นค้างและตัดสินใจรวน ---
FRONT_STOP_MM   = 220      # ToF หน้า: หยุดห่างกำแพง 25 ซม. เพื่อให้มีพื้นที่ตีวงเลี้ยว
WALL_LOST_CM    = 45.0     # Sharp: ต้องไกลกว่า 45 ซม. ถึงจะถือว่าเป็นทางแยกเปิดโล่ง
IR_BLOCKED_THRESH = 0      # ค่า IO ที่แปลว่า "IR เจอกำแพง" (ถ้าฮาร์ดแวร์จริงคือ 1 ให้เปลี่ยน)
CONTROL_DT      = 0.05     # ความถี่ loop คุมการเคลื่อนที่ (วินาที)

# ตัวแปรสำหรับ PID
integral_error = 0.0
last_error = 0.0

# =====================================================================
# SHARP CALIBRATION
# =====================================================================
def sharp_to_cm(adc_value):
    if adc_value is None:
        return None
    voltage = adc_value * (3.3 / 1023)
    # ป้องกัน Division by zero และค่าติดลบ เมื่อห่างกำแพงมากเกินไป
    if voltage <= 0.45:
        return 999.0
    # ปรับจูนสมการตามเซนเซอร์ Sharp ที่ใช้จริง
    distance_cm = 27.86 / (voltage - 0.42)
    return max(distance_cm, 8.0)

# =====================================================================
# การเชื่อมต่อหุ่นยนต์และเซนเซอร์
# =====================================================================
ep_robot = robot.Robot()
print("Connecting via AP Mode...")
ep_robot.initialize(conn_type="ap")

ep_chassis = ep_robot.chassis
sensor = ep_robot.sensor
sensor_adaptor = ep_robot.sensor_adaptor
sensor_adaptor.start()

tof_front = 9999

def on_tof_data(sub_info):
    global tof_front
    if isinstance(sub_info, (list, tuple)) and len(sub_info) > 0:
        tof_front = sub_info[0]
    elif isinstance(sub_info, (int, float)):
        tof_front = sub_info

sensor.sub_distance(freq=10, callback=on_tof_data)

# IMU
current_yaw = 0.0
def on_attitude(attitude_info):
    global current_yaw
    current_yaw = attitude_info[0]

ep_chassis.sub_attitude(freq=20, callback=on_attitude)

def safe_get_adc(hub_id, port):
    try:
        return sensor_adaptor.get_adc(id=hub_id, port=port)
    except Exception:
        return None

def safe_get_io(hub_id, port):
    try:
        return sensor_adaptor.get_io(id=hub_id, port=port)
    except Exception:
        return None

def read_all_sensors():
    return {
        "tof_front_mm": tof_front,
        # Port 2 (id=2): Sharp ซ้าย
        "sharp_left_cm": sharp_to_cm(safe_get_adc(hub_id=2, port=1)),
        # Port 1 (id=1): Sharp ขวา
        "sharp_right_cm": sharp_to_cm(safe_get_adc(hub_id=1, port=1)),
        # Port 4 (id=4): IR ซ้าย (สมมติว่าใช้ช่อง port=1 ของ Adaptor)
        "ir_45l": safe_get_io(hub_id=4, port=1),
        # Port 3 (id=3): IR ขวา (สมมติว่าใช้ช่อง port=1 ของ Adaptor)
        "ir_45r": safe_get_io(hub_id=3, port=1),
    }

path_log = []

# =====================================================================
# ฟังก์ชันการเคลื่อนที่ & PID Control
# =====================================================================
def stop_robot():
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)

def turn(angle_deg):
    """
    มุมบวก (+) = ทวนเข็มนาฬิกา (เลี้ยวซ้าย)
    มุมลบ (-) = ตามเข็มนาฬิกา (เลี้ยวขวา)
    ใส่ระบบ Timeout ป้องกันโปรแกรมค้างหากหน้ารถขูดกำแพง
    """
    stop_robot()
    time.sleep(0.1)
    try:
        action = ep_chassis.move(x=0, y=0, z=angle_deg, z_speed=TURN_SPEED)
        # ให้เวลาหมุนเต็มที่ 4 วินาที ถ้าหมุนไม่ไปให้หลุดลูปทันที จะได้ไม่ค้าง
        action.wait_for_completed(timeout=4)
    except Exception as e:
        print(f"\n[WARN] Turn might be blocked -> skipped. ({e})")
        
    path_log.append(("TURN", angle_deg))
    time.sleep(0.1)

def compute_pid(error):
    """คำนวณ PID Control สำหรับการ Strafe ซ้าย-ขวา"""
    global integral_error, last_error
    
    # คำนวณ Integral พร้อม Anti-windup
    integral_error += error * CONTROL_DT
    integral_error = max(-50, min(50, integral_error)) 
    
    # คำนวณ Derivative
    derivative = (error - last_error) / CONTROL_DT
    last_error = error
    
    output = (KP_CENTER * error) + (KI_CENTER * integral_error) + (KD_CENTER * derivative)
    return max(-MAX_STRAFE, min(MAX_STRAFE, output))

def wall_follow_step():
    global integral_error, last_error
    s = read_all_sensors()
    dl = s["sharp_left_cm"]
    dr = s["sharp_right_cm"]

    # 1. เช็คระบบป้องกันการชนด้วย IR เซนเซอร์ก่อน (Emergency Override)
    ir_l_blocked = (s["ir_45l"] == IR_BLOCKED_THRESH)
    ir_r_blocked = (s["ir_45r"] == IR_BLOCKED_THRESH)

    strafe_y = 0.0

    if ir_l_blocked:
        # หุ่นกำลังจะชนกำแพงซ้าย -> บังคับแถหนีไปทางขวาเต็มที่ (ค่าบวก)
        strafe_y = MAX_STRAFE
        integral_error = 0.0
        last_error = 0.0
        
    elif ir_r_blocked:
        # หุ่นกำลังจะชนกำแพงขวา -> บังคับแถหนีไปทางซ้ายเต็มที่ (ค่าลบ)
        strafe_y = -MAX_STRAFE
        integral_error = 0.0
        last_error = 0.0
        
    else:
        # 2. ถ้า IR ไม่เตือน (ปลอดภัย) ให้ใช้ Sharp + PID คุมให้อยู่กึ่งกลาง
        left_ok  = dl is not None and dl < WALL_LOST_CM
        right_ok = dr is not None and dr < WALL_LOST_CM
    
        error = 0.0
        if left_ok and right_ok:
            error = dr - dl  
        elif left_ok and not right_ok:
            error = CORRIDOR_HALF_CM - dl
        elif right_ok and not left_ok:
            error = dr - CORRIDOR_HALF_CM
        else:
            # ไม่เจอกำแพงทั้งสองฝั่ง (ลานกว้าง)
            integral_error = 0.0
            last_error = 0.0
            error = 0.0
    
        # ถ้าระยะคลาดเคลื่อนไม่เกิน 14 cm (Acceptable Error) ให้ถือว่าอยู่ตรงกลางแล้ว
        if abs(error) <= 14.0:
            error = 0.0
            integral_error = 0.0  # รีเซ็ตค่าสะสมเพื่อไม่ให้สวิงตอนหลุดระยะ
            
        if error != 0.0:
            strafe_y = compute_pid(error)

    # ส่งคำสั่งขับเคลื่อน
    ep_chassis.drive_speed(x=BASE_SPEED, y=strafe_y, z=0, timeout=CONTROL_DT)
    return s

def decide_turn(s):
    """
    ตัดสินใจเลี้ยวโดยเปรียบเทียบค่าระยะจาก Sharp
    จะเลี้ยวไปทางที่ Sharp อ่านค่าได้ไกลกว่า (มีพื้นที่ว่างมากกว่า)
    -90 = เลี้ยวขวา
     90 = เลี้ยวซ้าย
    """
    dl = s["sharp_left_cm"] if s["sharp_left_cm"] is not None else 0.0
    dr = s["sharp_right_cm"] if s["sharp_right_cm"] is not None else 0.0

    if dr > dl:
        return -90  # เลี้ยวขวา (ฝั่งขวาโล่งกว่า)
    else:
        return 90   # เลี้ยวซ้าย (ฝั่งซ้ายโล่งกว่า หรือเท่ากัน)

# =====================================================================
# ลอจิกการวางของ (Target Placement)
# =====================================================================
def hold_object_at_start():
    """สั่งให้หุ่นหนีบสิ่งของไว้ในมือตั้งแต่เปิดเครื่อง"""
    print("[INIT] Gripping object...")
    ep_gripper = ep_robot.gripper
    ep_gripper.close(power=50)
    time.sleep(1)

def is_target_reached(sensors):
    """
    ตรวจสอบว่าหุ่นถึงจุดวางของหรือยัง
    แก้ไขเงื่อนไขนี้ตามรูปแบบสนามของคุณ เช่น ถ้าระยะ ToF < 100mm และไม่มีกำแพงซ้ายขวา
    """
    return False 

def place_object():
    """ฟังก์ชันทำงานเมื่อถึงเป้าหมาย"""
    print("\n[TARGET REACHED] Placing the object...")
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    
    # ขยับแขนไปยังตำแหน่งที่ต้องการวาง
    ep_arm.moveto(x=180, y=-40).wait_for_completed()
    time.sleep(0.5)
    
    # ปล่อยของ
    ep_gripper.open(power=50)
    time.sleep(1)
    
    # หุบแขนกลับ
    ep_gripper.close(power=30)
    ep_arm.moveto(x=100, y=50).wait_for_completed()

# =====================================================================
# State Machine หลัก
# =====================================================================
def maze_solve_loop(max_steps=2000):
    print("=" * 70)
    print("  START MAZE SOLVING (Right-hand rule + PID centering + Anti-Crash)")
    print("=" * 70)

    # 1. หนีบของตั้งแต่เริ่ม
    hold_object_at_start()

    step_count = 0
    try:
        while step_count < max_steps:
            s = wall_follow_step()
            
            # ปริ้นค่าดูสถานะแบบ Real-time
            sl_str = f"{s['sharp_left_cm']:.1f}" if s["sharp_left_cm"] is not None else "N/A"
            sr_str = f"{s['sharp_right_cm']:.1f}" if s["sharp_right_cm"] is not None else "N/A"
            print(
                f"\rToF:{s['tof_front_mm']:4}mm  "
                f"SharpL:{sl_str}cm  SharpR:{sr_str}cm  "
                f"IR_L:{s['ir_45l']}  IR_R:{s['ir_45r']}   ",
                end=""
            )
            
            # เช็คว่าถึงจุดหมายหรือยัง
            if is_target_reached(s):
                stop_robot()
                place_object()
                print("\n[SUCCESS] Mission Completed!")
                break

            # เจอกำแพงหน้า -> ตัดสินใจเลี้ยว
            if s["tof_front_mm"] < FRONT_STOP_MM:
                stop_robot()
                time.sleep(0.1)
                
                print(f"\n[STOP] Front blocked at {s['tof_front_mm']}mm -> deciding turn...")
                angle = decide_turn(s)
                
                if angle != 0:
                    print(f"[TURN] {angle} degrees")
                    turn(angle)
                    
                    # รีเซ็ต PID เมื่อเลี้ยวเสร็จ 
                    global integral_error, last_error
                    integral_error = 0.0
                    last_error = 0.0
                else:
                    print("[FORWARD] Path clear, continuing")

            time.sleep(CONTROL_DT)
            step_count += 1

    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Stopping by user...")
    finally:
        stop_robot()
        print("\nFinal path log:")
        for action in path_log:
            print("  ", action)

# =====================================================================
# MAIN
# =====================================================================
if __name__ == "__main__":
    try:
        maze_solve_loop()
    finally:
        try: sensor.unsub_distance()
        except: pass
        try: ep_chassis.unsub_attitude()
        except: pass
        try: sensor_adaptor.stop()
        except: pass
        
        ep_robot.close()
        print("Connection closed.")
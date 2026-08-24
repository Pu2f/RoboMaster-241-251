import math
import time
# pyrefly: ignore [missing-import]
from robomaster import robot

# =====================================================================
# CONFIG — ตั้งค่าความเร็วและระบบ
# =====================================================================
BASE_SPEED      = 0.18     # ความเร็วเดินหน้า (m/s)
TURN_SPEED      = 45       # ความเร็วหมุนตอนเลี้ยว (deg/s)

FRONT_WALL_THRES_MM = 280  # ToF ด้านหน้า: น้อยกว่า 28 ซม. ถือว่ามีกำแพง
SIDE_WALL_THRES_CM  = 42.0 # Sharp ด้านข้าง: น้อยกว่า 42 ซม. ถือว่ามีกำแพง
TARGET_FRONT_MM     = 200  # ระยะเทียบกำแพงหน้าเป้าหมาย (20 ซม. จากกำแพง)
ACTION_PAUSE_SEC    = 0.5  # หยุดพัก 0.5 วินาทีก่อนเริ่มทำแอคชันใหม่

# PID Control สำหรับประคองหุ่น
KP_CENTER       = 0.015    # ดึงกลับเมื่อห่างจากกำแพง
KP_YAW          = 0.02     # ดึงองศากลับเมื่อหัวเริ่มเบี้ยว

# =====================================================================
# PURE FLOOD FILL CONFIG
# =====================================================================
MAZE_WIDTH = 4           # จำนวนช่องแกน X (ช่อง 0, 1, 2, 3)
MAZE_HEIGHT = 4          # จำนวนช่องแกน Y (ช่อง 0, 1, 2, 3)
CELL_SIZE_M = 0.60       # ขนาด 1 ช่องตาราง = 0.60 เมตร (60 ซม.)
GOAL_X = 3               # พิกัดเป้าหมาย X
GOAL_Y = 2               # พิกัดเป้าหมาย Y

# walls[x][y] เก็บกำแพง 4 ทิศ: 0=North (+Y), 1=East (+X), 2=South (-Y), 3=West (-X)
walls = [[[0, 0, 0, 0] for _ in range(MAZE_HEIGHT)] for _ in range(MAZE_WIDTH)]
distances = [[999 for _ in range(MAZE_HEIGHT)] for _ in range(MAZE_WIDTH)]

# ใส่กำแพงขอบสนามรอบนอก
for x in range(MAZE_WIDTH):
    walls[x][0][2] = 1                # ขอบล่าง (South)
    walls[x][MAZE_HEIGHT - 1][0] = 1  # ขอบบน (North)
for y in range(MAZE_HEIGHT):
    walls[0][y][3] = 1                # ขอบซ้าย (West)
    walls[MAZE_WIDTH - 1][y][1] = 1   # ขอบขวา (East)

# =====================================================================
# SHARP SENSOR CALIBRATION
# =====================================================================
def sharp_to_cm(adc_value):
    if adc_value is None:
        return None
    voltage = adc_value * (3.3 / 1023)
    if voltage <= 0.45:
        return 999.0
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

# 1. ToF Front Distance
tof_front = 9999
def on_tof_data(sub_info):
    global tof_front
    if isinstance(sub_info, (list, tuple)) and len(sub_info) > 0:
        tof_front = sub_info[0]
    elif isinstance(sub_info, (int, float)):
        tof_front = sub_info

sensor.sub_distance(freq=10, callback=on_tof_data)

# 2. IMU Attitude (Yaw)
current_yaw = 0.0
initial_yaw = 0.0
def on_attitude(attitude_info):
    global current_yaw
    current_yaw = attitude_info[0]

ep_chassis.sub_attitude(freq=20, callback=on_attitude)

# 3. Wheel Odometry (Position Tracking)
pos_x = 0.0
pos_y = 0.0
def on_position(pos_info):
    global pos_x, pos_y
    pos_x, pos_y = pos_info[0], pos_info[1]

ep_chassis.sub_position(freq=20, callback=on_position)

def get_relative_yaw():
    """คำนวณองศาแบบสัมพัทธ์เทียบกับทิศทางเริ่มต้นตอนเปิดเครื่อง (Feature B)"""
    diff = current_yaw - initial_yaw
    if diff > 180: diff -= 360
    elif diff < -180: diff += 360
    return diff

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
        "sharp_left_cm": sharp_to_cm(safe_get_adc(hub_id=2, port=1)),
        "sharp_right_cm": sharp_to_cm(safe_get_adc(hub_id=1, port=1)),
        "ir_45l": safe_get_io(hub_id=4, port=1),
        "ir_45r": safe_get_io(hub_id=3, port=1),
    }

# =====================================================================
# ฟังก์ชันการเคลื่อนที่ระดับพื้นฐาน (Discrete Movement & Auto-Calibration)
# =====================================================================
def stop_robot():
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=0.1)

def align_front_wall(target_mm=TARGET_FRONT_MM):
    """
    Feature A: จัดระยะห่างจากกำแพงหน้าให้อยู่กึ่งกลางช่องพอดีเป๊ะ (เช่น 20 ซม.) ก่อนเลี้ยว
    """
    s = read_all_sensors()
    if s["tof_front_mm"] > 350 or s["tof_front_mm"] < 40:
        return # ไม่ได้อยู่ใกล้กำแพงหน้า ไม่ต้องปรับ
        
    print(f"[ALIGN] Calibrating distance to front wall -> Target: {target_mm}mm")
    for _ in range(8):
        s = read_all_sensors()
        err_mm = s["tof_front_mm"] - target_mm
        if abs(err_mm) < 15: # คลาดเคลื่อนไม่เกิน 1.5 ซม. ถือว่าเป๊ะแล้ว
            break
            
        adj_speed = 0.10 if err_mm > 0 else -0.10
        ep_chassis.drive_speed(x=adj_speed, y=0, z=0, timeout=0.1)
        time.sleep(0.08)
        
    stop_robot()
    time.sleep(0.05)

def turn(angle_deg):
    stop_robot()
    time.sleep(0.1)
    try:
        action = ep_chassis.move(x=0, y=0, z=angle_deg, z_speed=TURN_SPEED)
        action.wait_for_completed(timeout=4)
    except Exception as e:
        print(f"\n[WARN] Turn blocked. ({e})")
    time.sleep(0.1)

def turn_to_heading(current_heading, target_heading):
    """หมุนหุ่นไปยังทิศที่ต้องการ พร้อมชดเชยองศา (Yaw Correction) ด้วย IMU"""
    diff = (target_heading - current_heading) % 4
    if diff == 0:
        return current_heading
    
    stop_robot()
    time.sleep(0.5) # พัก 0.5 วินาทีก่อนเริ่มหมุน
    
    # Feature A: ถ้าด้านหน้ามีกำแพง ให้จัดระยะกึ่งกลางช่องก่อนเริ่มเลี้ยว
    s = read_all_sensors()
    if s["tof_front_mm"] < FRONT_WALL_THRES_MM:
        align_front_wall(TARGET_FRONT_MM)

    # ป้องกันท้ายฟาดกำแพงตอนกลับตัว 180 องศา
    s = read_all_sensors()
    if diff == 2 and s["tof_front_mm"] < 190:
        print("[SAFETY] Backing up slightly before 180-deg turn...")
        try:
            ep_chassis.move(x=-0.08, y=0, z=0, xy_speed=0.15).wait_for_completed(timeout=2)
            time.sleep(0.5)
        except Exception:
            pass

    angle = 0
    if diff == 1: angle = -90   # เลี้ยวขวา
    elif diff == 2: angle = 180 # กลับหลังหัน
    elif diff == 3: angle = 90  # เลี้ยวซ้าย
        
    if angle != 0:
        print(f"[TURN] Turning {angle} deg -> Target Heading {target_heading}")
        turn(angle)
        
        target_rel_yaw = target_heading * -90.0
        if target_rel_yaw <= -180: target_rel_yaw += 360
        
        # Fine-tune ด้วย IMU ให้ตรงเป๊ะ
        for _ in range(15):
            yaw_error = target_rel_yaw - get_relative_yaw()
            if yaw_error > 180: yaw_error -= 360
            elif yaw_error < -180: yaw_error += 360
            
            if abs(yaw_error) < 3.0:
                break
                
            corr_z = 15 if yaw_error > 0 else -15
            ep_chassis.drive_speed(x=0, y=0, z=corr_z, timeout=0.1)
            time.sleep(0.1)
            
        stop_robot()
        time.sleep(0.5) # พัก 0.5 วินาทีหลังหมุนเสร็จ
        
    return target_heading

def move_forward_one_cell(target_heading):
    """
    Feature C: เดินหน้า 1 ช่องตารางโดยวัดระยะจริงจากล้อ (Wheel Odometry) ครบ 0.60m เป๊ะๆ
    พร้อมระบบ PID Centering และระบบเบรกฉุกเฉิน
    """
    stop_robot()
    time.sleep(0.5) # พัก 0.5 วินาทีก่อนเริ่มเดินหน้า
    
    # บันทึกพิกัดล้อเริ่มต้น
    start_x, start_y = pos_x, pos_y
    target_rel_yaw = target_heading * -90.0
    if target_rel_yaw <= -180: target_rel_yaw += 360
    
    print(f"[MOVE] Forward 1 cell (Odometry target: {CELL_SIZE_M}m)")
    
    max_timeout = (CELL_SIZE_M / BASE_SPEED) * 1.5 # เผื่อเวลา timeout ป้องกันค้าง
    start_time = time.time()
    
    while time.time() - start_time < max_timeout:
        # คำนวณระยะทางที่ล้อวิ่งไปจริง (Euclidean Distance จาก Odometry)
        dist_traveled = math.hypot(pos_x - start_x, pos_y - start_y)
        if dist_traveled >= CELL_SIZE_M:
            print(f"[ODOM] Target reached: {dist_traveled:.3f}m")
            break
            
        s = read_all_sensors()
        dl = s["sharp_left_cm"]
        dr = s["sharp_right_cm"]
        
        # ระบบเบรกฉุกเฉิน: ถ้าจวนตัวจะชนกำแพงด้านหน้า ให้หยุดเดินทันที
        if s["tof_front_mm"] < 130:
            print(f"\n[BRAKE] Front wall reached ({s['tof_front_mm']}mm) -> Stopped.")
            break
            
        strafe_y = 0.0
        turn_z = 0.0
        
        # 1. Wall Centering PID (ใช้เมื่อมีกำแพงทั้ง 2 ด้าน)
        left_ok = dl is not None and 8.0 <= dl < SIDE_WALL_THRES_CM
        right_ok = dr is not None and 8.0 <= dr < SIDE_WALL_THRES_CM
        
        error = 0.0
        if left_ok and right_ok:
            error = dr - dl
            if abs(error) > 18.0: # กรอง Noise
                error = 0.0
                
        if abs(error) > 2.0:
            strafe_y = KP_CENTER * error
            strafe_y = max(-0.12, min(0.12, strafe_y))
            
        # 2. Emergency Side Scraping Protection (ใช้ค่าจาก IR ซ้าย-ขวา)
        # ถ้าตัวรถเบี้ยวจนชิดกำแพงระยะประชิด (IR=1) ให้สั่งแถหลบฉุกเฉินทันที
        if s["ir_45l"] == 1 and s["ir_45r"] != 1:
            strafe_y = 0.14  # แถหนีไปทางขวา
        elif s["ir_45r"] == 1 and s["ir_45l"] != 1:
            strafe_y = -0.14 # แถหนีไปทางซ้าย
            
        # 3. Yaw Correction (รักษามุมให้ขนานกับแกนสนาม)
        yaw_error = target_rel_yaw - get_relative_yaw()
        if yaw_error > 180: yaw_error -= 360
        elif yaw_error < -180: yaw_error += 360
        
        if abs(yaw_error) > 1.5:
            turn_z = KP_YAW * yaw_error * 45.0
            turn_z = max(-18, min(18, turn_z))
            
        ep_chassis.drive_speed(x=BASE_SPEED, y=strafe_y, z=turn_z, timeout=0.1)
        time.sleep(0.04)
        
    stop_robot()
    time.sleep(0.1)

# =====================================================================
# ลอจิก PURE FLOOD FILL ALGORITHM
# =====================================================================
def get_sensors_as_walls():
    s = read_all_sensors()
    front_wall = 1 if s["tof_front_mm"] < FRONT_WALL_THRES_MM else 0
    left_wall = 1 if (s["sharp_left_cm"] is not None and s["sharp_left_cm"] < SIDE_WALL_THRES_CM) else 0
    right_wall = 1 if (s["sharp_right_cm"] is not None and s["sharp_right_cm"] < SIDE_WALL_THRES_CM) else 0
    return front_wall, left_wall, right_wall

def update_walls_in_map(x, y, heading, front, left, right):
    global walls
    walls[x][y][heading] = front
    walls[x][y][(heading + 1) % 4] = right
    walls[x][y][(heading + 3) % 4] = left
    
    dx = [0, 1, 0, -1]
    dy = [1, 0, -1, 0]
    for i in range(4):
        if walls[x][y][i] == 1:
            nx, ny = x + dx[i], y + dy[i]
            if 0 <= nx < MAZE_WIDTH and 0 <= ny < MAZE_HEIGHT:
                walls[nx][ny][(i + 2) % 4] = 1

def calculate_flood_fill(target_x, target_y):
    global distances
    for x in range(MAZE_WIDTH):
        for y in range(MAZE_HEIGHT):
            distances[x][y] = 999
            
    distances[target_x][target_y] = 0
    queue = [(target_x, target_y)]
    
    dx = [0, 1, 0, -1]
    dy = [1, 0, -1, 0]
    
    while queue:
        cx, cy = queue.pop(0)
        curr_dist = distances[cx][cy]
        for i in range(4):
            if walls[cx][cy][i] == 0:
                nx, ny = cx + dx[i], cy + dy[i]
                if 0 <= nx < MAZE_WIDTH and 0 <= ny < MAZE_HEIGHT:
                    if distances[nx][ny] > curr_dist + 1:
                        distances[nx][ny] = curr_dist + 1
                        queue.append((nx, ny))

def get_best_next_move(x, y, heading):
    best_dist = 999
    best_heading = heading
    
    dx = [0, 1, 0, -1]
    dy = [1, 0, -1, 0]
    heading_order = [heading, (heading + 1) % 4, (heading + 3) % 4, (heading + 2) % 4]
    
    for check_heading in heading_order:
        if walls[x][y][check_heading] == 0:
            nx, ny = x + dx[check_heading], y + dy[check_heading]
            if 0 <= nx < MAZE_WIDTH and 0 <= ny < MAZE_HEIGHT:
                dist = distances[nx][ny]
                if dist < best_dist:
                    best_dist = dist
                    best_heading = check_heading
                    
    if best_dist == 999:
        for check_heading in heading_order:
            if walls[x][y][check_heading] == 0:
                best_heading = check_heading
                break
                
    return best_heading

# =====================================================================
# ลอจิกการวางของ (Target Placement)
# =====================================================================
def hold_object_at_start():
    print("[INIT] Gripping object & Zeroing IMU...")
    ep_gripper = ep_robot.gripper
    ep_gripper.close(power=50)
    
    # Feature B: เซ็ตศูนย์ IMU ตอนเริ่มต้น
    global initial_yaw
    time.sleep(0.5)
    initial_yaw = current_yaw
    print(f"[IMU] Calibrated Initial Yaw = {initial_yaw:.2f}°")

def place_object():
    print("\n[TARGET REACHED] Placing the object...")
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    
    ep_arm.moveto(x=180, y=-40).wait_for_completed()
    time.sleep(0.5)
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.close(power=30)
    ep_arm.moveto(x=100, y=50).wait_for_completed()

# =====================================================================
# State Machine หลัก (Pure Flood Fill Loop)
# =====================================================================
def flood_fill_loop(max_steps=200):
    print("=" * 70)
    print("  START MAZE SOLVING (Odometry + Auto-Calibrated Flood Fill)")
    print("=" * 70)

    hold_object_at_start()

    current_x, current_y = 0, 0
    current_heading = 0 # 0=North, 1=East, 2=South, 3=West
    
    step_count = 0
    try:
        while step_count < max_steps:
            print(f"\n--- Step {step_count} | Pos: ({current_x}, {current_y}) | Heading: {current_heading} ---")
            
            # 1. เช็คว่าถึง Goal หรือยัง
            if current_x == GOAL_X and current_y == GOAL_Y:
                stop_robot()
                place_object()
                print("\n[SUCCESS] Goal reached successfully!")
                break
                
            # 2. อ่านค่าเซนเซอร์
            s = read_all_sensors()
            sl_str = f"{s['sharp_left_cm']:.1f}" if s["sharp_left_cm"] is not None else "N/A"
            sr_str = f"{s['sharp_right_cm']:.1f}" if s["sharp_right_cm"] is not None else "N/A"
            print(f"Raw Sensors -> ToF:{s['tof_front_mm']}mm, SharpL:{sl_str}cm, SharpR:{sr_str}cm, IR_L:{s['ir_45l']}, IR_R:{s['ir_45r']}")
            
            front_wall, left_wall, right_wall = get_sensors_as_walls()
            print(f"Walls Detected -> Front:{front_wall}, Left:{left_wall}, Right:{right_wall}")
            
            # 3. อัปเดตกำแพง & คำนวณ Flood Fill
            update_walls_in_map(current_x, current_y, current_heading, front_wall, left_wall, right_wall)
            calculate_flood_fill(GOAL_X, GOAL_Y)
            
            # 4. เลือกทิศทางที่ดีที่สุด
            next_heading = get_best_next_move(current_x, current_y, current_heading)
            print(f"Decision -> Current Dist: {distances[current_x][current_y]} | Next Heading: {next_heading}")
            
            # 5. หมุนตัว (มี Feature A: จัดระยะเทียบกำแพงหน้าอัตโนมัติก่อนเลี้ยว)
            current_heading = turn_to_heading(current_heading, next_heading)
            
            # 6. เดินหน้า 1 ช่อง (มี Feature C: วัดระยะจากล้อ Odometry 0.60m)
            move_forward_one_cell(current_heading)
            
            # 7. อัปเดตพิกัดช่องปัจจุบัน
            if current_heading == 0: current_y += 1
            elif current_heading == 1: current_x += 1
            elif current_heading == 2: current_y -= 1
            elif current_heading == 3: current_x -= 1
            
            # ป้องกันพิกัดหลุดขอบตาราง
            current_x = max(0, min(MAZE_WIDTH - 1, current_x))
            current_y = max(0, min(MAZE_HEIGHT - 1, current_y))
            
            step_count += 1
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Stopping by user...")
    finally:
        stop_robot()
        print("\nFinal Distance Map:")
        for y in reversed(range(MAZE_HEIGHT)):
            row = ""
            for x in range(MAZE_WIDTH):
                d = distances[x][y]
                row += f"{d:3} " if d != 999 else "  X "
            print(row)

# =====================================================================
# MAIN
# =====================================================================
if __name__ == "__main__":
    try:
        flood_fill_loop()
    finally:
        try: sensor.unsub_distance()
        except: pass
        try: ep_chassis.unsub_attitude()
        except: pass
        try: ep_chassis.unsub_position()
        except: pass
        try: sensor_adaptor.stop()
        except: pass
        
        ep_robot.close()
        print("Connection closed.")
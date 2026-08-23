# -*-coding:utf-8-*-
# Assignment 1: เดินเขาวงกตด้วยกฎเกาะผนังขวา (right-hand wall-following)
# แบบ "cell-by-cell" บน grid ขนาด 60x60cm ต่อช่อง แล้ว "วาง" วัตถุที่ถืออยู่ในตัวจับ (gripper)
# เมื่อผู้ใช้กด Ctrl+C ที่ตำแหน่งเป้าหมาย
#
# ทำไมเปลี่ยนจาก continuous P-control (เวอร์ชันก่อนหน้า) มาเป็น cell-by-cell:
#   เวอร์ชันก่อนหน้าตัดสินใจเลี้ยว/เดินหน้าใหม่ทุก 100ms จากค่าเซนเซอร์ขณะเคลื่อนที่ ทำให้เวลาเจอ
#   ทางตัน/มุมแคบที่ R และ L ใกล้เคียงกัน โปรแกรมสลับทิศหมุน (turn_dir) ไปมาทุกรอบ หุ่นจึงหมุนแกว่ง
#   อยู่กับที่ไม่ไปไหน (ดู log ตัวอย่างที่ pos ค้างที่จุดเดิมนับสิบรอบ)
#   เวอร์ชันนี้ใช้ประโยชน์จากการที่เขาวงกตเป็น grid ขนาดคงที่ (60cm/ช่อง): หยุดนิ่งที่กึ่งกลางช่อง
#   ก่อนตัดสินใจ (อ่านเซนเซอร์แบบเฉลี่ยหลายครั้งตอนหยุดนิ่ง ลด noise) แล้วเลี้ยว 90/180 องศาแบบ
#   "เด็ดขาด" ด้วย chassis.move() (blocking, ปิด loop ควบคุมในตัวหุ่นเอง รู้แน่ว่าหมุน/เดินครบตามสั่ง)
#   จากนั้นค่อยเดินหน้า 1 ช่อง (60cm) แบบ closed-loop โดยวัดระยะจริงจาก odometry (sub_position)
#   พร้อม centering ต่อเนื่องระหว่างเดิน เพื่อไม่ให้ชนผนังข้างระหว่างทาง
#
# เซนเซอร์ที่ใช้ (รวม 5 ตัว ตามข้อกำหนด, ไม่ใช้กล้อง):
#   ระบบที่ 1 - sensor_adaptor board (ADC / IO), มี 4 บอร์ด บอร์ดละ 2 พอร์ต:
#     1) SHARP GP2Y0A21 ขวา  -> board_id=1, port=1 (ADC) หันขวา ใช้เช็คผนัง/ช่องเปิดขวา + centering
#     2) SHARP GP2Y0A21 ซ้าย -> board_id=2, port=1 (ADC) หันซ้าย ใช้เช็คผนัง/ช่องเปิดซ้าย + centering
#     3) IR ดิจิทัล มุมหน้า-ขวา -> board_id=3, port=1 (IO) กันชนมุมขวา (safety, ระหว่างเดิน 1 ช่อง)
#     4) IR ดิจิทัล มุมหน้า-ซ้าย -> board_id=4, port=1 (IO) กันชนมุมซ้าย (safety, ระหว่างเดิน 1 ช่อง)
#   ระบบที่ 2 - infrared depth / TOF sensor (คนละพอร์ตกับ sensor_adaptor, ผ่าน ep_robot.sensor):
#     5) infrared depth sensor กลางหน้า -> port_id=1 -> distance[0] ใช้เช็คผนังหน้าตอนตัดสินใจ +
#        ใช้ชะลอ/หยุดฉุกเฉินระหว่างเดิน 1 ช่อง
#
# ลำดับความสำคัญของการตัดสินใจ ณ กึ่งกลางแต่ละช่อง (สูง -> ต่ำ, ตามกฎ right-hand):
#   1. IR มุมหน้าเจอสิ่งกีดขวางประชิด -> ถอยหลังเล็กน้อยแล้วอ่านค่าใหม่ (safety, ไม่หมุนแกว่งอยู่กับที่)
#   2. มีช่องเปิดด้านขวา (ไม่มีผนัง) -> เลี้ยวขวา 90 องศา
#   3. ผนังหน้าไกลพอ (ไม่มีผนังกั้น) -> เดินหน้าตรงต่อ (ไม่เลี้ยว)
#   4. มีช่องเปิดด้านซ้าย -> เลี้ยวซ้าย 90 องศา
#   5. ตันทั้ง 3 ทาง -> ทางตัน, กลับหลังหัน 180 องศา
#   จากนั้นเดินหน้า 1 ช่อง (60cm) แบบ closed-loop + centering ก่อนตัดสินใจรอบถัดไป
#
# หมายเหตุก่อนรันจริง:
#   - ต้องปรับค่าคงที่ในหัวข้อ "ค่าคงที่ที่ต้องปรับจูนหน้างาน" ให้ตรงกับตำแหน่งพอร์ตที่ต่อสายจริง,
#     calibrate สูตร ADC->cm ของ SHARP, ตรวจสอบหน่วยของ ToF (มักเป็น mm), และวัด CELL_SIZE_M จริง
#   - ต้องตรวจสอบ YAW_SIGN บนหุ่นจริงก่อนใช้งาน (ดูคำอธิบายที่ค่าคงที่ YAW_SIGN ด้านล่าง) เพราะ
#     ทิศทาง z ที่เป็นบวก/ลบ = เลี้ยวซ้าย/ขวา อาจต่างกันไปตามการตั้งค่า/เวอร์ชัน SDK

import csv
import math
import time
from robomaster import robot


# ------------------------- ค่าคงที่ที่ต้องปรับจูนหน้างาน -------------------------

# ---- ตำแหน่งพอร์ตบน sensor_adaptor (board_id: 1-6, port: 1-2 ต่อบอร์ด) ----
SHARP_RIGHT_BOARD, SHARP_RIGHT_PORT = 1, 1   # SHARP หันขวา
SHARP_LEFT_BOARD, SHARP_LEFT_PORT = 2, 1     # SHARP หันซ้าย
IR_FRONT_RIGHT_BOARD, IR_FRONT_RIGHT_PORT = 3, 1   # IR มุมหน้า-ขวา
IR_FRONT_LEFT_BOARD, IR_FRONT_LEFT_PORT = 4, 1      # IR มุมหน้า-ซ้าย

# ---- ตำแหน่งพอร์ตของ infrared depth (ToF) sensor กลางหน้า ----
TOF_PORT_ID = 1          # port_id ของ infrared depth sensor (1-4), เสียบช่องที่ 1 = tof1
TOF_UNIT_IS_MM = True    # SDK นี้มักคืนค่าเป็น mm -> ตรวจสอบจริงแล้วปรับถ้าคืนเป็น cm อยู่แล้ว

# ---- แปลงค่า ADC (SHARP) เป็นระยะทาง (cm) ----
# สูตร 27.86 * v^-1.15 เป็นค่าประมาณจาก datasheet ของ GP2Y0A21 (ช่วงวัด 10-80cm)
# ให้ calibrate จริงโดยวัดระยะจริงหลายจุด (เช่น 10,20,...,80 cm) จดค่า ADC คู่กันแล้ว fit ใหม่
ADC_MAX = 1023
ADC_VREF = 3.3
SHARP_MIN_CM = 10.0
SHARP_MAX_CM = 80.0

# โมดูล IR ราคาถูกส่วนใหญ่ output LOW(0) เมื่อเจอสิ่งกีดขวาง -> ตั้ง True ถ้าตัวที่ใช้เป็นแบบนี้
IR_ACTIVE_LOW = True

# ---- ขนาด grid ของเขาวงกต ----
CELL_SIZE_M = 0.60           # ขนาดช่อง 60x60cm ตามที่กำหนด
CELL_DISTANCE_TOLERANCE_M = 0.02   # ยอมรับคลาดเคลื่อนตอนหยุดครบ 1 ช่อง (odometry ไม่มีทาง exact)
CELL_SLOWDOWN_MARGIN_M = 0.15      # เริ่มชะลอความเร็วก่อนถึงเป้าหมาย 1 ช่อง

# ---- เกณฑ์ตัดสินใจว่ามี/ไม่มีผนัง ----
OPEN_WALL_CM = 35.0          # ถ้าระยะข้าง > ค่านี้ ถือว่า "ไม่มีผนัง" ด้านนั้น (มีทางเลี้ยว/เปิดโล่ง)
SIDE_HUG_TARGET_CM = 15.0    # setpoint ตอนเกาะผนังข้างเดียว (fallback เมื่อมีผนังแค่ด้านขวา)

FRONT_TOF_STOP_CM = 25.0     # ผนังหน้าใกล้กว่านี้ -> ถือว่ามีผนังหน้ากั้น (ตัดสินใจ/หยุดฉุกเฉิน)
FRONT_TOF_SLOW_CM = 45.0     # เริ่มชะลอความเร็วตั้งแต่ระยะนี้

SENSOR_SAMPLES = 5           # จำนวนครั้งที่อ่านเซนเซอร์เฉลี่ยตอนหยุดนิ่งกึ่งกลางช่อง (ลด noise)
SENSOR_SAMPLE_DELAY_S = 0.02

# ---- ความเร็วในการเดิน/เลี้ยว ----
FORWARD_SPEED = 0.3          # m/s ความเร็วเดินหน้าสูงสุดระหว่าง 1 ช่อง
MIN_FORWARD_SPEED = 0.12     # m/s ความเร็วต่ำสุดตอนชะลอ (ใกล้ผนังหน้า/ใกล้ครบระยะ)
MOVE_XY_SPEED = 0.7          # m/s ความเร็วอ้างอิงให้ chassis.move() ตอนถอยหลังฉุกเฉิน
TURN_Z_SPEED = 60            # deg/s ความเร็วหมุนตอนเลี้ยว 90/180 องศาแบบ blocking

BACKUP_DISTANCE_M = 0.08     # ระยะถอยหลังตอน IR มุมหน้าเจอสิ่งกีดขวางประชิด (safety)

CENTER_KP = 1.5              # gain สำหรับ centering (มีผนัง 2 ข้าง)
SIDE_KP = 2.0                # gain สำหรับเกาะผนังข้างเดียว (fallback)
MAX_YAW_CORRECTION = 30      # deg/s มุมแก้ไขสูงสุดตอน proportional steering ระหว่างเดิน 1 ช่อง

# ทิศทางบวก/ลบของ z ที่ป้อนให้ chassis (drive_speed/move) ว่าตรงกับ "เลี้ยวซ้าย" ทางกายภาพหรือไม่
# ตาม official example (examples/02_chassis/01_move.py): z=+90 มีคอมเมนต์ว่า "เลี้ยวซ้าย 90 องศา"
# จึงตั้งค่าเริ่มต้น YAW_SIGN=+1 หมายถึง "ป้อน z บวก = หมุนซ้าย(ทวนเข็ม), z ลบ = หมุนขวา"
# **ต้องทดสอบยืนยันบนหุ่นจริงก่อนใช้งานจริง**: ลองสั่ง ep_chassis.move(x=0,y=0,z=90,z_speed=45)
# .wait_for_completed() เพียงคำสั่งเดียว ถ้าหุ่นหมุนไปทางขวาแทน ให้เปลี่ยนค่านี้เป็น -1
# (โค้ดทั้งไฟล์อ้างอิงทิศทางผ่านค่านี้ค่าเดียว แก้จุดเดียวก็สลับทิศทั้งระบบ)
YAW_SIGN = 1

CONTROL_HZ = 10               # ความถี่ loop ควบคุมระหว่างเดิน 1 ช่อง
MAX_RUN_SECONDS = 8 * 60      # เวลาตัดสูงสุด (safety กันหุ่นวิ่งค้าง)
MAX_CELLS = 300               # จำนวนช่องสูงสุดที่ยอมให้เดิน (safety กัน loop ตัดสินใจผิดพลาดวนไม่จบ)

PATH_LOG_FILE = "maze_path_log.csv"


# ------------------------------- ฟังก์ชันแปลงค่าเซนเซอร์ -------------------------------

def sharp_adc_to_cm(adc_value):
    """แปลงค่าดิบ ADC จาก SHARP GP2Y0A21 เป็นระยะทางโดยประมาณ (cm)"""
    voltage = (adc_value / ADC_MAX) * ADC_VREF
    if voltage <= 0.05:
        return SHARP_MAX_CM
    distance_cm = 27.86 * (voltage ** -1.15)
    return max(SHARP_MIN_CM, min(SHARP_MAX_CM, distance_cm))


def ir_obstacle_detected(io_value):
    """คืนค่า True เมื่อ IR ตรวจพบสิ่งกีดขวาง (ผนัง/มุม) ในระยะใกล้"""
    if IR_ACTIVE_LOW:
        return io_value == 0
    return io_value == 1


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


# ------------------------------------ ตัวแปรสถานะ ------------------------------------

state = {"x": 0.0, "y": 0.0, "yaw": 0.0, "front_tof_cm": FRONT_TOF_SLOW_CM}
path_log = []

ep_robot = None
ep_chassis = None
ep_sensor_adaptor = None
ep_sensor = None


def position_handler(position_info):
    """callback รับตำแหน่ง (odometry x,y) ของหุ่นยนต์ ใช้วัดระยะเดินจริงต่อช่อง + บันทึกเส้นทาง"""
    x, y, _z = position_info
    state["x"], state["y"] = x, y
    path_log.append((time.time(), x, y, state["yaw"]))


def attitude_handler(attitude_info):
    """callback รับมุม yaw จริงของตัวหุ่น (ไม่ใช่ z จาก sub_position ซึ่งเป็นความสูง ไม่ใช่มุมหัน)"""
    yaw, _pitch, _roll = attitude_info
    state["yaw"] = yaw


def tof_handler(sub_info):
    """callback รับระยะจาก infrared depth (ToF) sensor กลางหน้า, distance[0] = tof1"""
    distance = sub_info
    raw = distance[TOF_PORT_ID - 1]
    state["front_tof_cm"] = (raw / 10.0) if TOF_UNIT_IS_MM else raw


# ------------------------------------ ฟังก์ชันช่วย ------------------------------------

def stop_chassis():
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=1)


def read_sensors_once():
    right_adc = ep_sensor_adaptor.get_adc(id=SHARP_RIGHT_BOARD, port=SHARP_RIGHT_PORT)
    left_adc = ep_sensor_adaptor.get_adc(id=SHARP_LEFT_BOARD, port=SHARP_LEFT_PORT)
    ir_front_right = ep_sensor_adaptor.get_io(id=IR_FRONT_RIGHT_BOARD, port=IR_FRONT_RIGHT_PORT)
    ir_front_left = ep_sensor_adaptor.get_io(id=IR_FRONT_LEFT_BOARD, port=IR_FRONT_LEFT_PORT)
    return (
        sharp_adc_to_cm(right_adc),
        sharp_adc_to_cm(left_adc),
        ir_obstacle_detected(ir_front_right),
        ir_obstacle_detected(ir_front_left),
    )


def read_sensors_averaged(samples=SENSOR_SAMPLES):
    """อ่านเซนเซอร์หลายครั้งตอนหยุดนิ่งกึ่งกลางช่อง เพื่อลด noise ก่อนตัดสินใจเลี้ยว
    (ค่าระยะ SHARP เฉลี่ยกัน แต่ IR มุม/ToF หน้า ใช้ "เจอแม้แค่ครั้งเดียวก็นับว่าเจอ" เพราะเป็น safety)"""
    right_sum = 0.0
    left_sum = 0.0
    corner_r = False
    corner_l = False
    for _ in range(samples):
        right_cm, left_cm, ir_r, ir_l = read_sensors_once()
        right_sum += right_cm
        left_sum += left_cm
        corner_r = corner_r or ir_r
        corner_l = corner_l or ir_l
        time.sleep(SENSOR_SAMPLE_DELAY_S)
    front_tof_cm = state["front_tof_cm"]
    return right_sum / samples, left_sum / samples, front_tof_cm, corner_r, corner_l


def turn_relative(deg):
    """หมุนสัมพัทธ์กับทิศปัจจุบัน deg องศา แบบ blocking (deg บวก = ซ้าย ตาม YAW_SIGN=+1 เริ่มต้น)"""
    ep_chassis.move(x=0, y=0, z=YAW_SIGN * deg, z_speed=TURN_Z_SPEED).wait_for_completed()
    stop_chassis()


def turn_right_90():
    turn_relative(-90)


def turn_left_90():
    turn_relative(90)


def turn_180():
    turn_relative(180)


def back_up(distance_m):
    ep_chassis.move(x=-distance_m, y=0, z=0, xy_speed=MOVE_XY_SPEED).wait_for_completed()
    stop_chassis()


def drive_one_cell():
    """เดินหน้า 1 ช่อง (CELL_SIZE_M) แบบ closed-loop วัดระยะจริงจาก odometry พร้อม centering
    ต่อเนื่องระหว่างทาง และหยุดฉุกเฉินถ้าเจอผนังหน้า/มุมกีดขวางก่อนครบระยะ
    คืนค่า "done" ถ้าเดินครบช่อง, "blocked" ถ้าหยุดก่อนครบเพราะเจอสิ่งกีดขวาง"""
    start_x, start_y = state["x"], state["y"]
    period = 1.0 / CONTROL_HZ

    while True:
        loop_start = time.time()

        right_adc = ep_sensor_adaptor.get_adc(id=SHARP_RIGHT_BOARD, port=SHARP_RIGHT_PORT)
        left_adc = ep_sensor_adaptor.get_adc(id=SHARP_LEFT_BOARD, port=SHARP_LEFT_PORT)
        ir_front_right = ep_sensor_adaptor.get_io(id=IR_FRONT_RIGHT_BOARD, port=IR_FRONT_RIGHT_PORT)
        ir_front_left = ep_sensor_adaptor.get_io(id=IR_FRONT_LEFT_BOARD, port=IR_FRONT_LEFT_PORT)

        right_cm = sharp_adc_to_cm(right_adc)
        left_cm = sharp_adc_to_cm(left_adc)
        corner_blocked = ir_obstacle_detected(ir_front_right) or ir_obstacle_detected(ir_front_left)
        front_tof_cm = state["front_tof_cm"]

        traveled_m = math.hypot(state["x"] - start_x, state["y"] - start_y)
        remaining_m = CELL_SIZE_M - traveled_m

        if corner_blocked:
            stop_chassis()
            print("  [เดิน 1 ช่อง] เจอ IR มุมหน้าประชิดระหว่างทาง หยุดกลางช่อง (traveled={:.2f}m)".format(traveled_m))
            return "blocked"

        if front_tof_cm < FRONT_TOF_STOP_CM:
            stop_chassis()
            print("  [เดิน 1 ช่อง] ผนังหน้าใกล้เกินไป (front_tof={:.1f}cm) หยุดกลางช่อง (traveled={:.2f}m)".format(
                front_tof_cm, traveled_m))
            return "blocked"

        if traveled_m >= CELL_SIZE_M - CELL_DISTANCE_TOLERANCE_M:
            stop_chassis()
            return "done"

        # centering: มีผนังทั้ง 2 ข้าง -> รักษากึ่งกลาง, มีแค่ขวา -> เกาะผนังขวา (fallback)
        if left_cm > OPEN_WALL_CM and right_cm <= OPEN_WALL_CM:
            error = SIDE_HUG_TARGET_CM - right_cm   # error>0 = ชิดผนังขวาเกินไป ต้องหันซ้ายหนี
            yaw = clamp(YAW_SIGN * SIDE_KP * error, -MAX_YAW_CORRECTION, MAX_YAW_CORRECTION)
        else:
            error = right_cm - left_cm              # error>0 = ห่างขวามากกว่าซ้าย ต้องหันขวาเข้าหา
            yaw = clamp(-YAW_SIGN * CENTER_KP * error, -MAX_YAW_CORRECTION, MAX_YAW_CORRECTION)

        # ชะลอความเร็วตามระยะผนังหน้า และตามระยะที่เหลือก่อนครบ 1 ช่อง (กันเลยระยะตอนหยุด)
        front_ratio = clamp(
            (front_tof_cm - FRONT_TOF_STOP_CM) / (FRONT_TOF_SLOW_CM - FRONT_TOF_STOP_CM), 0.0, 1.0)
        distance_ratio = clamp(remaining_m / CELL_SLOWDOWN_MARGIN_M, 0.0, 1.0)
        speed_ratio = min(front_ratio, distance_ratio)
        forward = MIN_FORWARD_SPEED + speed_ratio * (FORWARD_SPEED - MIN_FORWARD_SPEED)

        ep_chassis.drive_speed(x=forward, y=0, z=yaw, timeout=1)

        print("  [เดิน 1 ช่อง] R={:.1f}cm L={:.1f}cm front_tof={:.1f}cm traveled={:.2f}m pos=({:.2f},{:.2f})".format(
            right_cm, left_cm, front_tof_cm, traveled_m, state["x"], state["y"]))

        elapsed = time.time() - loop_start
        time.sleep(max(0.0, period - elapsed))


def decide_and_turn(right_cm, left_cm, front_tof_cm):
    """ตัดสินใจตามกฎ right-hand ที่กึ่งกลางช่อง (right > straight > left > dead-end) แล้วเลี้ยวถ้าจำเป็น"""
    if right_cm > OPEN_WALL_CM:
        print("  -> เลี้ยวขวา 90 องศา (มีช่องเปิดด้านขวา)")
        turn_right_90()
    elif front_tof_cm >= FRONT_TOF_STOP_CM:
        print("  -> เดินหน้าตรง (ไม่มีผนังกั้นหน้า)")
    elif left_cm > OPEN_WALL_CM:
        print("  -> เลี้ยวซ้าย 90 องศา (ผนังหน้า+ขวาไม่มีทาง แต่ซ้ายเปิด)")
        turn_left_90()
    else:
        print("  -> ทางตัน กลับหลังหัน 180 องศา")
        turn_180()


# ---------------------------------------- main ----------------------------------------

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    ep_sensor = ep_robot.sensor
    ep_gripper = ep_robot.gripper

    ep_chassis.sub_position(freq=10, callback=position_handler)
    ep_chassis.sub_attitude(freq=10, callback=attitude_handler)
    ep_sensor.sub_distance(freq=10, callback=tof_handler)

    print("เริ่มเดินเขาวงกตด้วยกฎเกาะผนังขวา แบบ cell-by-cell (grid {:.0f}x{:.0f}cm)".format(
        CELL_SIZE_M * 100, CELL_SIZE_M * 100))
    print("กด Ctrl+C เมื่อหุ่นยนต์ถึงตำแหน่งเป้าหมายแล้ว เพื่อหยุดและวางวัตถุ")

    start_time = time.time()
    cells_moved = 0

    try:
        while True:
            right_cm, left_cm, front_tof_cm, corner_r, corner_l = read_sensors_averaged()

            print("[ช่องที่ {0}] R={1:.1f}cm L={2:.1f}cm front_tof={3:.1f}cm cornerR={4} cornerL={5} pos=({6:.2f},{7:.2f}) yaw={8:.1f}".format(
                cells_moved, right_cm, left_cm, front_tof_cm, corner_r, corner_l,
                state["x"], state["y"], state["yaw"]))

            # ---- priority 1: safety cutoff เมื่อ IR มุมหน้าเจอสิ่งกีดขวางประชิดตอนหยุดนิ่ง ----
            if corner_r or corner_l:
                print("  -> IR มุมหน้าเจอสิ่งกีดขวางประชิด ถอยหลัง {:.2f}m แล้วอ่านค่าใหม่".format(BACKUP_DISTANCE_M))
                back_up(BACKUP_DISTANCE_M)
                continue

            decide_and_turn(right_cm, left_cm, front_tof_cm)
            result = drive_one_cell()
            if result == "done":
                cells_moved += 1

            if time.time() - start_time > MAX_RUN_SECONDS:
                print("เกินเวลาสูงสุดที่กำหนด หยุดหุ่นยนต์เพื่อความปลอดภัย")
                break

            if cells_moved >= MAX_CELLS:
                print("เดินครบจำนวนช่องสูงสุดที่กำหนด ({0} ช่อง) หยุดหุ่นยนต์เพื่อความปลอดภัย".format(MAX_CELLS))
                break

    except KeyboardInterrupt:
        print("หยุดโดยผู้ใช้งาน (ถึงตำแหน่งเป้าหมายแล้ว)")

    finally:
        stop_chassis()

    # -------------------- ถึงตำแหน่งเป้าหมายแล้ว: วางวัตถุ --------------------
    # หมายเหตุ: ต้อง close gripper คีบวัตถุไว้ตั้งแต่ก่อนเริ่มรันสคริปต์นี้
    print("วางวัตถุที่ตำแหน่งเป้าหมาย...")
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()

    # -------------------- บันทึกเส้นทางเดินลงไฟล์ CSV สำหรับทำรายงาน --------------------
    with open(PATH_LOG_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "x_m", "y_m", "yaw_deg"])
        writer.writerows(path_log)
    print("บันทึกเส้นทางเดินไว้ที่ {0} ({1} จุด, เดินไปทั้งหมด {2} ช่อง)".format(
        PATH_LOG_FILE, len(path_log), cells_moved))

    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_sensor.unsub_distance()
    ep_robot.close()

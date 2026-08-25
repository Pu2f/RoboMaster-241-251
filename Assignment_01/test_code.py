"""Assignment 01 - Maze Solver ด้วย Flood Fill (Micromouse) สำหรับ RoboMaster EP

โหมดการทำงาน
------------
สำรวจเขาวงกตด้วย Flood Fill จากช่องเริ่มต้นจนถึงช่องเป้าหมายแล้วจบ (search run
อย่างเดียว ไม่มี return run / speed run) ระหว่างทางหุ่นคีบวัตถุไว้ และวางลงเมื่อ
ถึงช่องเป้าหมาย

ฮาร์ดแวร์ที่ต้องต่อ
------------------
- ToF (Distance Sensor) ด้านหน้า        -> sensor.sub_distance()
- Sharp IR analog ซ้าย                  -> sensor adaptor hub 2 port 1 (ADC)
- Sharp IR analog ขวา                   -> sensor adaptor hub 1 port 1 (ADC)
- IR digital 45 องศา ซ้าย               -> sensor adaptor hub 4 port 1 (IO)
- IR digital 45 องศา ขวา                -> sensor adaptor hub 3 port 1 (IO)
- Robotic arm + Gripper                 -> สำหรับคีบและวางวัตถุ

วิธีใช้
------
    python test_code.py --calib    วัดค่าเซนเซอร์จริง (ต้องทำก่อนใช้งานครั้งแรก)
    python test_code.py --sim      ทดสอบตรรกะ Flood Fill โดยไม่ต้องต่อหุ่น
    python test_code.py            วิ่งจริงในสนาม

ข้อควรรู้เรื่อง SDK (ตรวจสอบจาก source ใน src/robomaster/ แล้ว)
--------------------------------------------------------------
1. ``chassis.move(z=)`` กับ ``chassis.drive_speed(z=)`` ใช้เครื่องหมายตรงข้ามกัน
   ตัวอย่างทางการ examples/02_chassis/01_move.py ระบุ ``move(z=+90)`` = เลี้ยวซ้าย
   ส่วน examples/02_chassis/03_speed.py ระบุ ``drive_speed(z=+30)`` = เลี้ยวขวา
   ไฟล์นี้จึงใช้ ``drive_speed`` สำหรับการหมุน "เท่านั้น" ไม่แตะ ``move(z=)`` เลย
2. ``sensor_adaptor.get_adc()`` / ``get_io()`` เป็น blocking round-trip ที่มี
   timeout 3 วินาที (client.py ``send_sync_msg``) เรียกใน control loop ไม่ได้
   จึงใช้ ``sensor_adaptor.sub_adapter()`` แบบ push แทน และเหลือ get_adc ไว้เป็น
   fallback กรณี sub_adapter ไม่ส่งข้อมูลมาเท่านั้น
3. ค่าที่ ``sub_adapter`` คืนมาเป็น ADC ดิบสเกลเดียวกับ ``get_adc`` (uint16 ทั้งคู่
   ดู protocol.py:1954 เทียบกับ sensor.py:75) สลับไปมาได้โดยค่าไม่เพี้ยน
4. ``sensor_adaptor.start()`` เป็น no-op (module.py ``Module.start`` = pass)
5. ``robot.initialize()`` เรียก ``set_robot_mode(FREE)`` ให้อยู่แล้ว
"""

import argparse
import math
import statistics
import sys
import threading
import time
from collections import deque

try:
    # pyrefly: ignore [missing-import]
    from robomaster import robot
except ImportError as exc:  # โหมด --sim ทดสอบตรรกะได้โดยไม่ต้องมี SDK ติดตั้ง
    robot = None
    ROBOT_IMPORT_ERROR = exc
else:
    ROBOT_IMPORT_ERROR = None


# =====================================================================
# CONFIG - แก้ค่าทั้งหมดที่นี่ที่เดียว
# =====================================================================

# ---------- ขนาดสนามและเป้าหมาย ----------
MAZE_W = 4                      # จำนวนช่องแกน X (ทิศตะวันออกเป็นบวก)
MAZE_H = 4                      # จำนวนช่องแกน Y (ทิศเหนือเป็นบวก)
CELL_SIZE_M = 0.60              # ความกว้าง 1 ช่อง หน่วยเมตร
START_CELL = (0, 0)
START_HEADING = 0               # 0=North 1=East 2=South 3=West
GOAL_CELLS = [(2, 3)]           # รองรับหลายช่อง เช่นโซนกลาง 2x2 ของ micromouse

# ---------- การต่อสายเซนเซอร์ ----------
TOF_INDEX = 0                   # sub_distance คืน list 4 ตัว ใช้ตัวไหนเป็นด้านหน้า
SHARP_LEFT = (2, 1)             # (hub_id, port) อ่านด้วย ADC
SHARP_RIGHT = (1, 1)            # (hub_id, port) อ่านด้วย ADC
IR_LEFT_45 = (4, 1)             # (hub_id, port) อ่านด้วย IO (digital 0/1)
IR_RIGHT_45 = (3, 1)            # (hub_id, port) อ่านด้วย IO (digital 0/1)

# ---------- ค่าที่ได้จาก `--calib` เท่านั้น ห้ามเดา ----------
# ตราบใดที่ยังเป็น None โปรแกรมจะปฏิเสธที่จะวิ่งในสนามจริง
# เหตุผล: threshold ที่ผิดทำให้หุ่น "วิ่งดูปกติทุกอย่างแต่สร้างแผนที่ผิด"
# ซึ่งแยกไม่ออกจากบั๊กของ odometry หรือของ flood fill ตอนอยู่หน้าสนาม
SHARP_LEFT_WALL_ADC = (374, 352)      # (enter, exit) ทำ hysteresis กันค่ากระพริบ
SHARP_RIGHT_WALL_ADC = (374, 351)     # (enter, exit)
SHARP_LEFT_REF = 350           # ค่า ADC ซ้าย ตอนหุ่นอยู่กลางช่องพอดี
SHARP_RIGHT_REF = 350          # ค่า ADC ขวา ตอนหุ่นอยู่กลางช่องพอดี
IR_TRIGGERED_VALUE = 0       # ค่า IO ตอนมีสิ่งกีดขวาง (0 หรือ 1)
FRONT_STOP_MM = 116            # ToF ที่อ่านได้ตอนหุ่นอยู่กลางช่องและหันชนกำแพง

FRONT_WALL_MM_OVERRIDE = None   # ปกติปล่อย None ให้คำนวณจากเรขาคณิตของช่อง

# ---------- เกณฑ์ตรวจกำแพง ----------
TOF_MAX_VALID_MM = 4000         # เกินนี้ถือว่าอ่านไม่ได้ / ไม่มีอะไรอยู่ในระยะ
WALL_VOTE_SAMPLES = 5           # อ่านกี่ครั้งตอนจอดนิ่ง ก่อนโหวตลงแผนที่
WALL_VOTE_INTERVAL = 0.06

# ---------- IR 45 องศา ----------
# ใช้เป็นตัวกันชนจุดบอดทแยงหน้า (Sharp ยิงตรงข้าง ToF ยิงตรงหน้า มุมทแยงไม่มีใครดู)
# ไม่ใช้เป็นตัวตัดสินใจว่ามีกำแพงหรือไม่ และไม่ใช้จบการเคลื่อนที่
USE_IR_AS_GUARD = True          # ติด -> ชะลอความเร็ว + เบี่ยงหนีเล็กน้อย
USE_IR_AS_SLANT = False         # ติดข้างเดียว -> เดาว่าเข้าช่องเอียง แล้วแก้ yaw
IR_SLANT_BIAS_DEG = 4.0         # องศาที่เอียงเป้าหมายหนีเมื่อเปิด USE_IR_AS_SLANT
GUARD_STRAFE = 0.05             # m/s เบี่ยงหนีเมื่อ IR guard ติดข้างเดียว

# ---------- ความเร็ว ----------
BASE_SPEED = 0.18               # m/s เดินหน้าปกติ
SLOW_SPEED = 0.09               # m/s ตอนเข้าใกล้กำแพงหน้า
GUARD_SPEED = 0.07              # m/s ตอน IR guard ติด
BACKUP_SPEED = 0.12             # m/s ตอนถอยกลับเข้าช่องเดิม
ALIGN_SPEED = 0.08              # m/s ตอนจัดระยะกับกำแพงหน้า
TURN_MAX_DPS = 55.0             # deg/s ความเร็วหมุนสูงสุด
TURN_MIN_DPS = 14.0             # deg/s กำลังขับขั้นต่ำเพื่อชนะแรงเสียดทานพื้น

# ---------- ตัวคุม (controller) ----------
# หมายเหตุ: drive_speed ปัดค่าก่อนส่งเสมอ (util.py val2proto) โดย x/y เหลือทศนิยม
# 2 ตำแหน่ง และ z ถูกปัดเป็นจำนวนเต็ม ดังนั้นคำสั่งที่เล็กกว่า 0.01 m/s หรือ
# 1 deg/s จะกลายเป็นศูนย์ การจูนเกนให้ผลลัพธ์ต่ำกว่าขั้นนี้จึงไม่มีผลอะไรเลย
KP_TURN = 1.5                   # deg/s ต่อ 1 องศา error ตอนหมุนอยู่กับที่
TURN_TOLERANCE_DEG = 1.5
TURN_TIMEOUT_S = 5.0
KP_YAW_HOLD = 0.9               # deg/s ต่อ 1 องศา error ตอนเดินหน้า
MAX_YAW_CORRECT_DPS = 16.0
YAW_HOLD_DEADBAND_DEG = 1.2
KP_CENTER = 0.0006              # m/s ต่อ 1 หน่วย ADC error (ไม่แปลงเป็น cm)
CENTER_DEADBAND_ADC = 15        # error ต่ำกว่านี้ถือว่ากลางช่องแล้ว
MAX_STRAFE = 0.08               # m/s strafe สูงสุด
ALIGN_TOLERANCE_MM = 18

# ---------- การนับช่อง ----------
CELL_COMPLETE_RATIO = 0.85      # เดินได้ >= 85% ของช่อง = ถือว่าเข้าช่องใหม่แล้ว
TOF_STOP_MIN_RATIO = 0.50       # ถ้าจบเพราะ ToF ต้องเดินมาแล้วอย่างน้อย 50%
MOVE_TIMEOUT_RATIO = 2.0        # timeout = (CELL_SIZE_M / BASE_SPEED) * ค่านี้

# ---------- payload (คีบ / วางวัตถุ) ----------
DO_PAYLOAD = True               # ปิดชั่วคราวได้ตอนดีบักเฉพาะการเดิน
ARM_CARRY_XY = (100, 50)        # ตำแหน่งแขนตอนวิ่ง (mm) ต้องไม่บัง ToF
ARM_PLACE_XY = (180, -40)       # ตำแหน่งแขนตอนวางของ (mm)
GRIPPER_POWER = 50

# ---------- ระบบ ----------
CONTROL_DT = 0.04               # คาบของ control loop
SETTLE_S = 0.25                 # เวลารอให้หุ่นนิ่งก่อนอ่านค่าลงแผนที่
MAX_STEPS = 200
DDS_FREQ = 20                   # Hz รองรับเฉพาะ 1, 5, 10, 20, 50
ADAPTER_WAIT_S = 2.0            # รอ sub_adapter นานแค่ไหนก่อนสลับไป fallback
SENSOR_STALE_S = 0.5            # ไม่มีข้อมูลใหม่เกินนี้ = ถือว่าค่าใช้ไม่ได้
DRIVE_WATCHDOG_S = 0.5          # หุ่นหยุดเองถ้าเราไม่ส่งคำสั่งใหม่ภายในเวลานี้
YAW_SIGN_TEST_DPS = 40.0        # ความเร็วหมุนตอนหาเครื่องหมายของ yaw
YAW_SIGN_TEST_S = 0.45          # นานแค่ไหน (~18 องศา)
CONN_TYPE = "ap"                # ap / sta / rndis


# =====================================================================
# ค่าคงที่และฟังก์ชันช่วยทั่วไป
# =====================================================================
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DIR_NAMES = ("N", "E", "S", "W")
DIR_ARROWS = ("^", ">", "v", "<")
DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)
INF = 9999


def wrap_deg(angle):
    """บีบมุมให้อยู่ในช่วง [-180, 180) องศา"""
    return (angle + 180.0) % 360.0 - 180.0


def clamp(value, low, high):
    """จำกัดค่าให้อยู่ระหว่าง low ถึง high"""
    return max(low, min(high, value))


def front_wall_threshold_mm():
    """เกณฑ์ ToF ที่ใช้ตัดสินว่ามีกำแพงอยู่ด้านหน้า หน่วย mm

    เรขาคณิต: เมื่อหุ่นจอดกลางช่อง กำแพงที่ขอบช่องนี้อ่านได้ ``FRONT_STOP_MM``
    ส่วนกำแพงที่ขอบช่องถัดไปอ่านได้ ``FRONT_STOP_MM + CELL_SIZE`` เส้นแบ่งจึงวางไว้
    ตรงกลางระหว่างสองค่านั้นพอดี เพราะเป็นจุดที่ทนต่อการจอดคลาดเคลื่อนได้มากที่สุด
    """
    if FRONT_WALL_MM_OVERRIDE is not None:
        return FRONT_WALL_MM_OVERRIDE
    return FRONT_STOP_MM + int(CELL_SIZE_M * 1000.0 / 2.0)


def sharp_polarity():
    """+1 ถ้า ADC สูง = อยู่ใกล้, -1 ถ้า ADC ต่ำ = อยู่ใกล้

    อนุมานจากลำดับของ (enter, exit) ที่ ``--calib`` คำนวณมาให้ จึงไม่ต้องมี
    ค่าคอนฟิกแยกอีกตัว และไม่ต้องสมมติว่าเซนเซอร์ตอบสนองไปทางไหน
    """
    enter, exit_ = SHARP_LEFT_WALL_ADC
    return 1 if enter > exit_ else -1


def wall_from_adc(adc, thresholds):
    """แปลงค่า ADC ดิบเป็นสถานะกำแพง

    Args:
        adc (int): ค่า ADC ดิบ
        thresholds (tuple): (enter, exit) จาก ``--calib``

    Returns:
        bool or None: True=มีกำแพง, False=ไม่มี, None=ก้ำกึ่งตัดสินไม่ได้
    """
    if adc is None:
        return None
    enter, exit_ = thresholds
    if enter > exit_:
        if adc >= enter:
            return True
        if adc <= exit_:
            return False
    else:
        if adc <= enter:
            return True
        if adc >= exit_:
            return False
    return None


def ir_triggered(io_value):
    """True ถ้า IR 45 องศาตัวนั้นกำลังเจอสิ่งกีดขวาง"""
    if io_value is None or IR_TRIGGERED_VALUE is None:
        return False
    return int(io_value) == int(IR_TRIGGERED_VALUE)


def require_calibration():
    """ตรวจว่าค่าจาก --calib ครบแล้ว ถ้าไม่ครบให้หยุดพร้อมบอกว่าขาดตัวไหน"""
    missing = [name for name in (
        "SHARP_LEFT_WALL_ADC", "SHARP_RIGHT_WALL_ADC",
        "SHARP_LEFT_REF", "SHARP_RIGHT_REF",
        "IR_TRIGGERED_VALUE", "FRONT_STOP_MM",
    ) if globals()[name] is None]
    if not missing:
        return
    print("\n[STOP] ยังไม่ได้คาลิเบรตเซนเซอร์ ค่าที่ยังขาด:")
    for name in missing:
        print("         - {0}".format(name))
    print("\n  รัน `python test_code.py --calib` ก่อน (ใช้เวลาไม่ถึง 2 นาที)")
    print("  แล้วคัดลอกบล็อกค่าที่มันพิมพ์ออกมา ไปวางทับใน CONFIG ด้านบนของไฟล์นี้")
    print("\n  เหตุผลที่ไม่ให้รันด้วยค่าเดา: threshold ที่ผิดจะทำให้หุ่นวิ่งดูปกติ")
    print("  ทุกอย่างแต่สร้างแผนที่ผิด ซึ่งหน้าสนามจะแยกไม่ออกเลยว่าปัญหาอยู่ที่")
    print("  threshold, odometry หรือ flood fill\n")
    sys.exit(1)


# =====================================================================
# ชั้นอ่านเซนเซอร์
# =====================================================================
class SensorSnapshot(object):
    """ค่าเซนเซอร์ทั้งชุด ณ เวลาเดียวกัน แช่แข็งไว้แล้ว

    ทั้งรอบการตัดสินใจจะอ้างอิงจากออบเจกต์ตัวเดียวนี้ ไม่ใช่ไปอ่านตัวแปร global
    ทีละตัวซึ่งอาจถูก callback เขียนทับกลางคันจนได้ค่าคนละช่วงเวลามาปนกัน
    """

    __slots__ = ("t", "tof_mm", "adc_left", "adc_right", "ir_left", "ir_right",
                 "yaw", "pos_x", "pos_y", "fresh", "stale_reason")

    def __init__(self, t, tof_mm, adc_left, adc_right, ir_left, ir_right,
                 yaw, pos_x, pos_y, fresh, stale_reason):
        #: float: เวลาที่ถ่ายภาพนิ่งชุดนี้
        self.t = t
        #: int or None: ระยะด้านหน้า mm, None = ไม่มีอะไรอยู่ในระยะที่วัดได้
        self.tof_mm = tof_mm
        #: int or None: ค่า ADC ดิบของ Sharp ซ้าย
        self.adc_left = adc_left
        #: int or None: ค่า ADC ดิบของ Sharp ขวา
        self.adc_right = adc_right
        #: int or None: ค่า IO ดิบของ IR 45 องศาซ้าย
        self.ir_left = ir_left
        #: int or None: ค่า IO ดิบของ IR 45 องศาขวา
        self.ir_right = ir_right
        #: float: yaw ดิบจาก IMU หน่วยองศา ช่วง [-180, 180]
        self.yaw = yaw
        #: float: ตำแหน่งล้อแกน x หน่วยเมตร
        self.pos_x = pos_x
        #: float: ตำแหน่งล้อแกน y หน่วยเมตร
        self.pos_y = pos_y
        #: bool: False เมื่อมีสตรีมสำคัญตัวใดตัวหนึ่งขาดการอัปเดต
        self.fresh = fresh
        #: str: ชื่อสตรีมที่ขาดการอัปเดต ว่างเปล่าถ้าปกติ
        self.stale_reason = stale_reason

    def front_wall(self):
        """bool: มีกำแพงด้านหน้าหรือไม่

        ``tof_mm`` เป็น None แปลว่าไม่มีอะไรอยู่ในระยะที่เซนเซอร์วัดได้ ซึ่งใน
        เขาวงกตคือ "ไม่มีกำแพง" ส่วนกรณีที่สตรีมขาดจริง ๆ จะถูกจับด้วย ``fresh``
        ที่ระดับบนแทน ไม่ปนกันตรงนี้
        """
        if self.tof_mm is None:
            return False
        return self.tof_mm < front_wall_threshold_mm()


class SensorHub(object):
    """เจ้าของ subscription ทั้งหมด และเป็นทางเดียวที่โค้ดส่วนอื่นอ่านเซนเซอร์

    callback ของ DDS ทำงานคนละเธรดกับ control loop ทุกฟิลด์จึงถูกเขียนและอ่าน
    ภายใต้ล็อกเดียวกัน

    Args:
        ep_robot: ออบเจกต์ robot.Robot ที่ initialize แล้ว
    """

    def __init__(self, ep_robot):
        self._chassis = ep_robot.chassis
        self._sensor = ep_robot.sensor
        self._adaptor = ep_robot.sensor_adaptor

        self._lock = threading.Lock()
        self._tof = [0, 0, 0, 0]
        self._tof_t = 0.0
        self._io = [0] * 12
        self._ad = [0] * 12
        self._adapter_t = 0.0
        self._yaw = 0.0
        self._att_t = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._pos_t = 0.0

        #: bool: True = ใช้ sub_adapter (push), False = fallback ไป get_adc (blocking)
        self.use_adapter = True
        self._poll_cache = (None, None)
        self._poll_t = 0.0
        self._started = False

    # ---------- callback (ทำงานบนเธรดของ DDS) ----------
    def _on_tof(self, info):
        now = time.time()
        with self._lock:
            self._tof = list(info)
            self._tof_t = now

    def _on_adapter(self, info):
        io_values, ad_values = info
        now = time.time()
        with self._lock:
            self._io = list(io_values)
            self._ad = list(ad_values)
            self._adapter_t = now

    def _on_attitude(self, info):
        now = time.time()
        with self._lock:
            self._yaw = info[0]
            self._att_t = now

    def _on_position(self, info):
        now = time.time()
        with self._lock:
            self._pos_x, self._pos_y = info[0], info[1]
            self._pos_t = now

    # ---------- วงจรชีวิต ----------
    def _check_wiring(self):
        """ตรวจว่า hub id ที่คอนฟิกไว้อยู่ในช่วงที่ sub_adapter รองรับ

        AdapterSubject.decode (sensor.py:71) ถอดรหัสมาแค่ 6 บอร์ด (index 0-11)
        ส่วน get_adc รับ id ได้ถึง 8 ถ้าคอนฟิกเกิน 6 จึงต้องบังคับใช้ fallback
        ไม่งั้นจะได้ IndexError กลางทางตอนวิ่ง
        """
        ports = (("SHARP_LEFT", SHARP_LEFT), ("SHARP_RIGHT", SHARP_RIGHT),
                 ("IR_LEFT_45", IR_LEFT_45), ("IR_RIGHT_45", IR_RIGHT_45))
        for name, (hub_id, port) in ports:
            if not 1 <= hub_id <= 8 or port not in (1, 2):
                raise ValueError(
                    "{0} = (hub {1}, port {2}) ไม่ถูกต้อง hub ต้องอยู่ 1-8 "
                    "และ port ต้องเป็น 1 หรือ 2".format(name, hub_id, port))
        too_high = [name for name, (hub_id, _) in ports if hub_id > 6]
        if too_high:
            self.use_adapter = False
            print("[WARN] {0} ใช้ hub เกิน 6 ซึ่ง sub_adapter ถอดรหัสไม่ถึง "
                  "จึงต้องใช้ get_adc แบบ blocking ทั้งหมด (ช้ากว่ามาก)"
                  .format(", ".join(too_high)))

    def start(self):
        """เปิด subscription ทั้งหมด แล้วรอจนมีข้อมูลชุดแรกเข้ามา"""
        # ตั้งธงก่อนเริ่ม subscribe เพื่อให้ stop() ยังเก็บกวาดตัวที่สมัครไปแล้วได้
        # ถ้าเกิด exception กลางคัน
        self._started = True
        self._check_wiring()
        self._sensor.sub_distance(freq=DDS_FREQ, callback=self._on_tof)
        self._adaptor.sub_adapter(freq=DDS_FREQ, callback=self._on_adapter)
        self._chassis.sub_attitude(freq=DDS_FREQ, callback=self._on_attitude)
        self._chassis.sub_position(freq=DDS_FREQ, callback=self._on_position)

        deadline = time.time() + ADAPTER_WAIT_S
        while time.time() < deadline:
            with self._lock:
                got_all = self._tof_t > 0 and self._att_t > 0 and self._pos_t > 0
                got_adapter = self._adapter_t > 0
            if got_all and (got_adapter or not self.use_adapter):
                print("[SENSOR] subscription พร้อมใช้งาน (ToF, attitude, position"
                      "{0})".format(", adapter" if self.use_adapter else ""))
                return
            time.sleep(0.05)

        with self._lock:
            got_adapter = self._adapter_t > 0 or not self.use_adapter
            missing = []
            if self._tof_t == 0:
                missing.append("ToF")
            if self._att_t == 0:
                missing.append("attitude")
            if self._pos_t == 0:
                missing.append("position")
        if missing:
            raise RuntimeError(
                "ไม่ได้รับข้อมูลจาก subscription: {0} - ตรวจการเชื่อมต่อหุ่น"
                .format(", ".join(missing)))
        if not got_adapter:
            self.use_adapter = False
            print("[WARN] sub_adapter ไม่ส่งข้อมูลใน {0} วินาที "
                  "สลับไปใช้ get_adc/get_io แบบ blocking แทน"
                  .format(ADAPTER_WAIT_S))
            print("[WARN] โหมดนี้ control loop จะช้าลงมาก ควรตรวจว่าเสียบ hub "
                  "ถูกพอร์ตหรือไม่")

    def stop(self):
        """ปิด subscription ทั้งหมด (เรียกซ้ำได้ ไม่ throw)"""
        if not self._started:
            return
        for name, fn in (("distance", self._sensor.unsub_distance),
                         ("adapter", self._adaptor.unsub_adapter),
                         ("attitude", self._chassis.unsub_attitude),
                         ("position", self._chassis.unsub_position)):
            try:
                fn()
            except Exception as exc:                    # noqa: BLE001
                print("[WARN] unsub {0} ล้มเหลว: {1}".format(name, exc))
        self._started = False

    # ---------- การอ่าน ----------
    @staticmethod
    def _adapter_index(hub_id, port):
        """แปลง (hub_id, port) เป็น index ของ list ที่ sub_adapter คืนมา

        อ้างอิง AdapterSubject.decode ใน sensor.py:71 ที่ไล่ ``for i in range(0, 6)``
        แล้วเก็บ port 1 ที่ ``i*2`` และ port 2 ที่ ``i*2+1``
        """
        return (hub_id - 1) * 2 + (port - 1)

    def _poll_adaptor(self, now):
        """fallback: อ่านผ่าน get_adc/get_io ซึ่ง block ได้นานถึง 3 วินาทีต่อครั้ง"""
        if self._poll_cache[0] is not None and now - self._poll_t < 0.05:
            return self._poll_cache
        adc = {}
        io = {}
        for key, (hub, port) in (("l", SHARP_LEFT), ("r", SHARP_RIGHT)):
            try:
                adc[key] = self._adaptor.get_adc(id=hub, port=port)
            except Exception:                           # noqa: BLE001
                adc[key] = None
        for key, (hub, port) in (("l", IR_LEFT_45), ("r", IR_RIGHT_45)):
            try:
                io[key] = self._adaptor.get_io(id=hub, port=port)
            except Exception:                           # noqa: BLE001
                io[key] = None
        self._poll_cache = (adc, io)
        self._poll_t = now
        return self._poll_cache

    def snapshot(self):
        """SensorSnapshot: อ่านเซนเซอร์ทุกตัวเป็นชุดเดียว"""
        now = time.time()
        with self._lock:
            tof_raw = self._tof[TOF_INDEX] if TOF_INDEX < len(self._tof) else 0
            tof_age = now - self._tof_t if self._tof_t else 1e9
            att_age = now - self._att_t if self._att_t else 1e9
            pos_age = now - self._pos_t if self._pos_t else 1e9
            yaw = self._yaw
            pos_x, pos_y = self._pos_x, self._pos_y
            if self.use_adapter:
                ad = list(self._ad)
                io = list(self._io)
                adapter_age = now - self._adapter_t if self._adapter_t else 1e9
            else:
                ad = io = None
                adapter_age = 0.0

        if ad is not None:
            adc_left = ad[self._adapter_index(*SHARP_LEFT)]
            adc_right = ad[self._adapter_index(*SHARP_RIGHT)]
            ir_left = io[self._adapter_index(*IR_LEFT_45)]
            ir_right = io[self._adapter_index(*IR_RIGHT_45)]
        else:
            adc, io_map = self._poll_adaptor(now)
            adc_left, adc_right = adc["l"], adc["r"]
            ir_left, ir_right = io_map["l"], io_map["r"]

        # ToF: ค่า 0 หรือค่าเกินพิสัย แปลว่าไม่มีเป้าหมายอยู่ในระยะ ไม่ใช่สตรีมพัง
        # กรณีสตรีมพังจริงจะถูกจับด้วย tof_age ด้านล่างแทน
        tof_mm = tof_raw if 0 < tof_raw <= TOF_MAX_VALID_MM else None

        stale = ""
        if tof_age > SENSOR_STALE_S:
            stale = "tof"
        elif att_age > SENSOR_STALE_S:
            stale = "attitude"
        elif pos_age > SENSOR_STALE_S:
            stale = "position"
        elif adapter_age > SENSOR_STALE_S:
            stale = "adapter"

        return SensorSnapshot(
            t=now, tof_mm=tof_mm, adc_left=adc_left, adc_right=adc_right,
            ir_left=ir_left, ir_right=ir_right, yaw=yaw,
            pos_x=pos_x, pos_y=pos_y, fresh=(stale == ""), stale_reason=stale)

    def read_walls_settled(self, samples=WALL_VOTE_SAMPLES):
        """อ่านกำแพงหน้า/ซ้าย/ขวา ตอนหุ่นจอดนิ่งแล้ว โดยโหวตเสียงข้างมาก

        ค่าที่ก้ำกึ่งจนตัดสินไม่ได้จะนับเป็น "ไม่มีกำแพง" โดยตั้งใจ เพราะสองความ
        ผิดพลาดนี้ไม่เท่ากัน:

        * เดาว่าไม่มีกำแพงทั้งที่มี - แก้ตัวเองได้ พอหมุนไปทางนั้นแล้ว pre-move
          check จะเห็นกำแพงด้วย ToF แล้วมาร์กลงแผนที่ให้เอง
        * เดาว่ามีกำแพงทั้งที่ไม่มี - ปิดทางเดินนั้นถาวรและไม่มีอะไรมาแก้ให้

        Returns:
            tuple: (front, left, right) เป็น bool ทั้งสามตัว
        """
        votes = {"front": [0, 0], "left": [0, 0], "right": [0, 0]}
        for _ in range(max(1, samples)):
            snap = self.snapshot()
            for key, value in (("front", snap.front_wall()),
                               ("left", wall_from_adc(snap.adc_left,
                                                      SHARP_LEFT_WALL_ADC)),
                               ("right", wall_from_adc(snap.adc_right,
                                                       SHARP_RIGHT_WALL_ADC))):
                if value is True:
                    votes[key][0] += 1
                elif value is False:
                    votes[key][1] += 1
            time.sleep(WALL_VOTE_INTERVAL)
        return (votes["front"][0] > votes["front"][1],
                votes["left"][0] > votes["left"][1],
                votes["right"][0] > votes["right"][1])


# =====================================================================
# แผนที่และ Flood Fill
# =====================================================================
class Maze(object):
    """แผนที่กำแพงและตัวคำนวณ Flood Fill

    กำแพงเก็บเป็น bitmask ต่อช่อง โดยบิตที่ i หมายถึงกำแพงทางทิศ i
    (0=N 1=E 2=S 3=W) และเก็บ ``known`` แยกไว้อีกชุดเพื่อบอกว่าด้านนั้น
    "เคยเห็นมาแล้วจริง" หรือ "ยังไม่เคยไปดู"

    Flood Fill จะถือว่าด้านที่ยังไม่เคยเห็นเป็นทางเปิดไว้ก่อน ซึ่งเป็นหลักการของ
    micromouse คือมองโลกในแง่ดีแล้วเดินไปแก้เอาข้างหน้า ทำให้ไม่ต้องเสียเวลา
    สำรวจช่องที่พิสูจน์แล้วว่าไม่มีทางอยู่บนเส้นทางที่ดีที่สุด

    Args:
        width (int): จำนวนช่องแกน x
        height (int): จำนวนช่องแกน y
        goals (list): รายการช่องเป้าหมาย [(x, y), ...]
    """

    def __init__(self, width, height, goals):
        self.width = width
        self.height = height
        self.goals = [tuple(g) for g in goals]
        self.walls = [[0] * height for _ in range(width)]
        self.known = [[0] * height for _ in range(width)]
        self._add_borders()

    def _add_borders(self):
        """ใส่กำแพงขอบสนามรอบนอก ซึ่งรู้แน่นอนอยู่แล้วโดยไม่ต้องไปวัด"""
        for x in range(self.width):
            self.set_wall(x, 0, SOUTH, True)
            self.set_wall(x, self.height - 1, NORTH, True)
        for y in range(self.height):
            self.set_wall(0, y, WEST, True)
            self.set_wall(self.width - 1, y, EAST, True)

    def in_bounds(self, x, y):
        """bool: พิกัดนี้อยู่ในสนามหรือไม่"""
        return 0 <= x < self.width and 0 <= y < self.height

    def has_wall(self, x, y, direction):
        """bool: ช่อง (x, y) มีกำแพงทางทิศ direction หรือไม่"""
        return bool(self.walls[x][y] & (1 << direction))

    def is_known(self, x, y, direction):
        """bool: เคยตรวจด้านนี้ของช่องนี้แล้วหรือยัง"""
        return bool(self.known[x][y] & (1 << direction))

    def set_wall(self, x, y, direction, present):
        """บันทึกผลการตรวจกำแพงหนึ่งด้าน พร้อมอัปเดตช่องข้างเคียงให้สอดคล้องกัน

        เมื่อ present เป็น True จะติดบิตกำแพง แต่เมื่อเป็น False จะไม่ลบบิตที่เคย
        ติดไว้แล้ว เพราะกำแพงจริงไม่เดินหนีไปไหน การอ่านพลาดครั้งเดียวจึงไม่ควร
        ลบสิ่งที่เคยยืนยันแล้วออกจากแผนที่ แผนที่จะเพิ่มกำแพงได้อย่างเดียว
        ซึ่งทำให้ผลของ Flood Fill นิ่งและไม่แกว่งไปมาระหว่างรอบ
        """
        if not self.in_bounds(x, y):
            return
        if present:
            self.walls[x][y] |= (1 << direction)
        self.known[x][y] |= (1 << direction)

        nx, ny = x + DX[direction], y + DY[direction]
        if self.in_bounds(nx, ny):
            opposite = (direction + 2) % 4
            if present:
                self.walls[nx][ny] |= (1 << opposite)
            self.known[nx][ny] |= (1 << opposite)

    def observe(self, x, y, heading, front, left, right):
        """บันทึกผลการตรวจกำแพงจากท่ายืนปัจจุบันลงแผนที่

        เซนเซอร์ให้ผลเป็น "หน้า/ซ้าย/ขวา" เทียบกับตัวหุ่น ต้องแปลงเป็นทิศสัมบูรณ์
        ของสนามก่อนตามทิศที่หุ่นหันอยู่
        """
        self.set_wall(x, y, heading, front)
        self.set_wall(x, y, (heading + 1) % 4, right)
        self.set_wall(x, y, (heading + 3) % 4, left)

    def flood(self):
        """คำนวณระยะจากทุกช่องไปยังเป้าหมายที่ใกล้ที่สุด ด้วย BFS ย้อนจากเป้าหมาย

        Returns:
            list: ตาราง distance[x][y] ค่า INF แปลว่าไปไม่ถึงด้วยความรู้ปัจจุบัน
        """
        dist = [[INF] * self.height for _ in range(self.width)]
        queue = deque()
        for gx, gy in self.goals:
            if self.in_bounds(gx, gy):
                dist[gx][gy] = 0
                queue.append((gx, gy))

        while queue:
            cx, cy = queue.popleft()
            next_dist = dist[cx][cy] + 1
            for direction in range(4):
                if self.has_wall(cx, cy, direction):
                    continue
                nx, ny = cx + DX[direction], cy + DY[direction]
                if self.in_bounds(nx, ny) and dist[nx][ny] > next_dist:
                    dist[nx][ny] = next_dist
                    queue.append((nx, ny))
        return dist

    def choose_next_heading(self, x, y, heading, dist):
        """เลือกทิศถัดไปที่ควรเดิน

        ไล่ดูตามลำดับ ตรงไป -> ขวา -> ซ้าย -> หลัง แล้วเทียบ distance ด้วย ``<``
        อย่างเคร่งครัด ผลคือเมื่อหลายทิศมี distance เท่ากันจะได้ทิศที่มาก่อนใน
        ลำดับนี้ ซึ่งทำให้หุ่นชอบเดินตรงมากกว่าหมุน ประหยัดเวลาและลด yaw drift

        Returns:
            int or None: ทิศที่เลือก หรือ None เมื่อไม่มีทางออกเลย
        """
        order = (heading, (heading + 1) % 4, (heading + 3) % 4, (heading + 2) % 4)
        best_dir = None
        best_dist = INF
        for direction in order:
            if self.has_wall(x, y, direction):
                continue
            nx, ny = x + DX[direction], y + DY[direction]
            if not self.in_bounds(nx, ny):
                continue
            if dist[nx][ny] < best_dist:
                best_dist = dist[nx][ny]
                best_dir = direction
        return best_dir

    def edge_stats(self):
        """tuple: (จำนวนด้านที่ตรวจแล้ว, จำนวนด้านทั้งหมด)

        นับด้านละหนึ่งครั้ง โดยไล่เฉพาะทิศเหนือกับตะวันออกของทุกช่อง แล้วเติม
        ขอบใต้ของแถวล่างสุดและขอบตะวันตกของคอลัมน์ซ้ายสุด
        """
        total = 0
        seen = 0
        for x in range(self.width):
            for y in range(self.height):
                for direction in (NORTH, EAST):
                    total += 1
                    seen += 1 if self.is_known(x, y, direction) else 0
        for x in range(self.width):
            total += 1
            seen += 1 if self.is_known(x, 0, SOUTH) else 0
        for y in range(self.height):
            total += 1
            seen += 1 if self.is_known(0, y, WEST) else 0
        return seen, total

    def _edge_glyph(self, x, y, direction, horizontal):
        """str: สัญลักษณ์ของขอบหนึ่งด้าน แยกกำแพง / โล่งที่ยืนยันแล้ว / ยังไม่เคยดู"""
        if self.has_wall(x, y, direction):
            return "---" if horizontal else "|"
        if self.is_known(x, y, direction):
            return "   " if horizontal else " "
        return " . " if horizontal else ":"

    def render(self, dist=None, robot=None, legend=False):
        """วาดแผนที่เป็น ASCII

        ขอบที่ยังไม่เคยตรวจจะแสดงเป็นจุด เพื่อให้แยกออกจากขอบที่ยืนยันแล้วว่าโล่ง
        Flood Fill มองสองอย่างนี้เหมือนกันคือเดินผ่านได้ แต่ตอนอ่านแผนที่เรา
        ต้องแยกให้ออกว่าอะไรคือข้อมูลจริง อะไรคือการมองโลกในแง่ดีไว้ก่อน

        Args:
            dist (list): ตาราง distance จาก :meth:`flood` ใส่ None ได้
            robot (tuple): (x, y, heading) ตำแหน่งหุ่น ใส่ None ได้
            legend (bool): ต่อท้ายด้วยคำอธิบายสัญลักษณ์และความคืบหน้าการสำรวจ

        Returns:
            str: แผนที่หลายบรรทัด พร้อม print ได้เลย
        """
        lines = []
        for y in range(self.height - 1, -1, -1):
            top = "+"
            for x in range(self.width):
                top += self._edge_glyph(x, y, NORTH, True) + "+"
            lines.append(top)

            mid = self._edge_glyph(0, y, WEST, False)
            for x in range(self.width):
                if robot is not None and (robot[0], robot[1]) == (x, y):
                    cell = " {0} ".format(DIR_ARROWS[robot[2]])
                elif (x, y) in self.goals:
                    cell = " G "
                elif dist is not None:
                    cell = "   " if dist[x][y] >= INF else "{0:3d}".format(dist[x][y])
                else:
                    cell = "   "
                mid += cell + self._edge_glyph(x, y, EAST, False)
            lines.append(mid)

        bottom = "+"
        for x in range(self.width):
            bottom += self._edge_glyph(x, 0, SOUTH, True) + "+"
        lines.append(bottom)

        if legend:
            seen, total = self.edge_stats()
            lines.append("--- = กำแพง | ว่าง = ตรวจแล้วโล่ง | . = ยังไม่เคยตรวจ"
                         "   (สำรวจแล้ว {0}/{1} ด้าน)".format(seen, total))
        return "\n".join(lines)


# =====================================================================
# การเคลื่อนที่
# =====================================================================
class Driver(object):
    """ชั้นควบคุมการเคลื่อนที่ระดับ "หนึ่งช่อง" และ "หนึ่งการเลี้ยว"

    การหมุนทุกครั้งใช้ ``chassis.drive_speed`` แบบปิดลูปกับ IMU เท่านั้น
    ไม่ใช้ ``chassis.move(z=)`` เลย เพราะสองตัวนี้ใช้เครื่องหมายตรงข้ามกัน
    (ดูหัวข้อ "ข้อควรรู้เรื่อง SDK" ด้านบนของไฟล์)

    Args:
        chassis: ออบเจกต์ chassis ของหุ่น
        hub (SensorHub): ตัวอ่านเซนเซอร์
    """

    def __init__(self, chassis, hub):
        self.chassis = chassis
        self.hub = hub
        #: int: +1 ถ้า yaw เพิ่มขึ้นเมื่อหมุนตามเข็ม, -1 ถ้าลดลง หาได้จาก calibrate_yaw_sign
        self.yaw_sign = 1
        #: float: ค่า yaw ดิบที่ถือว่าเป็นทิศเหนือของสนาม
        self.yaw_zero = 0.0

    # ---------- พื้นฐาน ----------
    def stop(self):
        """หยุดล้อทันที"""
        self.chassis.drive_speed(x=0, y=0, z=0)

    def _drive(self, x=0.0, y=0.0, z=0.0):
        """ส่งคำสั่งขับพร้อม watchdog กันหุ่นวิ่งต่อถ้า control loop ตาย"""
        self.chassis.drive_speed(x=x, y=y, z=z, timeout=DRIVE_WATCHDOG_S)

    def heading_yaw(self, heading):
        """float: ค่า yaw ดิบที่หุ่นควรอ่านได้เมื่อหันไปทาง heading"""
        return wrap_deg(self.yaw_zero + self.yaw_sign * heading * 90.0)

    # ---------- การตั้งศูนย์ ----------
    def calibrate_yaw_sign(self):
        """หาว่า yaw จาก IMU เพิ่มหรือลดเมื่อหมุนตามเข็ม แล้วหมุนกลับที่เดิม

        เรื่องนี้พิสูจน์จาก source ของ SDK ไม่ได้ เพราะเป็นข้อตกลงของฮาร์ดแวร์
        ถ้าเดาผิดหุ่นจะเลี้ยวกลับทางทุกครั้งโดยไม่มีอะไรเตือน จึงวัดเอาจริง
        ด้วยการหมุนทดสอบราว 18 องศาแล้วดูว่า yaw เคลื่อนไปทางไหน
        """
        start_yaw = self.hub.snapshot().yaw
        self._drive(z=YAW_SIGN_TEST_DPS)
        time.sleep(YAW_SIGN_TEST_S)
        self.stop()
        time.sleep(0.5)

        delta = wrap_deg(self.hub.snapshot().yaw - start_yaw)
        if abs(delta) < 3.0:
            raise RuntimeError(
                "หมุนทดสอบแล้ว yaw แทบไม่ขยับ ({0:.2f} องศา) - ล้ออาจติดขัด "
                "หรือ sub_attitude ไม่ทำงาน".format(delta))

        self.yaw_sign = 1 if delta > 0 else -1
        print("[IMU] หมุนตามเข็ม {0:.1f} องศาแล้ว yaw เปลี่ยน {1:+.2f} "
              "-> YAW_SIGN = {2:+d}"
              .format(YAW_SIGN_TEST_DPS * YAW_SIGN_TEST_S, delta, self.yaw_sign))

        self._rotate_to_abs(start_yaw)
        time.sleep(0.2)

    def set_north_reference(self, start_heading):
        """ตั้งว่า ณ ตอนนี้หุ่นหันไปทาง start_heading แล้วคำนวณ yaw ของทิศเหนือไว้"""
        current = self.hub.snapshot().yaw
        self.yaw_zero = wrap_deg(current - self.yaw_sign * start_heading * 90.0)
        print("[IMU] ตั้งศูนย์แล้ว: yaw ปัจจุบัน {0:.2f} = ทิศ {1}, "
              "ทิศเหนือ = yaw {2:.2f}"
              .format(current, DIR_NAMES[start_heading], self.yaw_zero))

    # ---------- การหมุน ----------
    def _rotate_to_abs(self, target_yaw, tolerance=TURN_TOLERANCE_DEG):
        """หมุนแบบปิดลูปจนกว่า yaw ดิบจะเท่ากับ target_yaw

        Returns:
            bool: True ถ้าเข้าเป้าภายในเวลาที่กำหนด
        """
        deadline = time.time() + TURN_TIMEOUT_S
        while time.time() < deadline:
            error = wrap_deg(target_yaw - self.hub.snapshot().yaw)
            if abs(error) <= tolerance:
                self.stop()
                time.sleep(0.15)
                return True
            speed = clamp(KP_TURN * error, -TURN_MAX_DPS, TURN_MAX_DPS)
            if abs(speed) < TURN_MIN_DPS:
                speed = TURN_MIN_DPS if error > 0 else -TURN_MIN_DPS
            self._drive(z=self.yaw_sign * speed)
            time.sleep(0.03)

        self.stop()
        time.sleep(0.15)
        final = wrap_deg(target_yaw - self.hub.snapshot().yaw)
        print("[WARN] หมุนไม่เข้าเป้าใน {0} วินาที เหลือ error {1:.2f} องศา"
              .format(TURN_TIMEOUT_S, final))
        return False

    def turn_to(self, current_heading, target_heading):
        """หมุนจากทิศหนึ่งไปอีกทิศหนึ่ง

        การกลับหลังหัน 180 องศาจะถูกแยกเป็นสองครั้ง ครั้งละ 90 องศา เพราะที่
        error พอดี 180 องศานั้นทิศทางที่ใกล้ที่สุดมีสองทางเท่ากัน ตัวคุมจะลังเล
        และอาจสั่นไปมา การบังคับผ่านจุดกึ่งกลางทำให้ทิศทางชัดเจนเสมอ

        Returns:
            int: ทิศที่หุ่นหันอยู่จริงหลังหมุนเสร็จ
        """
        delta = (target_heading - current_heading) % 4
        if delta == 0:
            return current_heading

        self.stop()
        time.sleep(0.2)

        # ถ้ามีกำแพงอยู่ข้างหน้า จัดระยะให้เข้ากลางช่องก่อน แล้วค่อยหมุน
        # เพื่อไม่ให้จุดหมุนของหุ่นเบียดกำแพง และให้เริ่มเดินช่องถัดไปจากกลางช่องพอดี
        snap = self.hub.snapshot()
        if snap.front_wall():
            self.align_front()

        if delta == 2:
            intermediate = (current_heading + 1) % 4
            print("[TURN] {0} -> {1} (แยกเป็น 2 ครั้งผ่าน {2})"
                  .format(DIR_NAMES[current_heading], DIR_NAMES[target_heading],
                          DIR_NAMES[intermediate]))
            self._rotate_to_abs(self.heading_yaw(intermediate))
            self._rotate_to_abs(self.heading_yaw(target_heading))
        else:
            print("[TURN] {0} -> {1}".format(DIR_NAMES[current_heading],
                                             DIR_NAMES[target_heading]))
            self._rotate_to_abs(self.heading_yaw(target_heading))

        return target_heading

    def align_front(self):
        """ขยับหน้า/หลังจนระยะถึงกำแพงหน้าเท่ากับ FRONT_STOP_MM

        ใช้ตอนก่อนหมุน เพื่อรีเซ็ตความคลาดเคลื่อนของ odometry ตามแนวเดินทิ้ง
        กำแพงหน้าเป็นจุดอ้างอิงสัมบูรณ์ที่แม่นกว่าการนับระยะจากล้อสะสม
        """
        snap = self.hub.snapshot()
        if snap.tof_mm is None or snap.tof_mm > FRONT_STOP_MM + 150:
            return
        if snap.tof_mm < 60:
            print("[ALIGN] ใกล้กำแพงเกินไป ({0}mm) ไม่จัดระยะ".format(snap.tof_mm))
            return

        for _ in range(25):
            snap = self.hub.snapshot()
            if snap.tof_mm is None:
                break
            error = snap.tof_mm - FRONT_STOP_MM
            if abs(error) < ALIGN_TOLERANCE_MM:
                break
            self._drive(x=ALIGN_SPEED if error > 0 else -ALIGN_SPEED)
            time.sleep(0.06)
        self.stop()
        time.sleep(0.1)

    # ---------- การเดินหน้า ----------
    def _centering_strafe(self, snap):
        """m/s: ความเร็ว strafe ที่ต้องใช้เพื่อประคองหุ่นให้อยู่กลางช่อง

        คุมบนค่า ADC ดิบโดยตรง ไม่แปลงเป็นเซนติเมตร เพราะสิ่งที่ต้องการคือ
        "จุดที่ซ้ายกับขวาสมดุลกัน" ซึ่งอยู่ที่เดิมเสมอไม่ว่าเส้นโค้งของเซนเซอร์
        จะเป็นรูปอะไร การแปลงเป็นระยะทางก่อนจึงไม่ได้เพิ่มความแม่นยำ มีแต่จะ
        เพิ่มโอกาสผิดจากสมการ calibration ที่อาจไม่ตรงกับเซนเซอร์ตัวจริง
        """
        polarity = sharp_polarity()
        left = wall_from_adc(snap.adc_left, SHARP_LEFT_WALL_ADC)
        right = wall_from_adc(snap.adc_right, SHARP_RIGHT_WALL_ADC)

        if left and right:
            # มีกำแพงสองข้าง: คุมให้ผลต่างซ้าย-ขวาเท่ากับตอนอยู่กลางช่องพอดี
            balance = SHARP_LEFT_REF - SHARP_RIGHT_REF
            error = (snap.adc_left - snap.adc_right) - balance
        elif left:
            # มีกำแพงข้างเดียว: คุมให้ระยะถึงกำแพงนั้นเท่ากับค่าอ้างอิง
            error = snap.adc_left - SHARP_LEFT_REF
        elif right:
            error = SHARP_RIGHT_REF - snap.adc_right
        else:
            return 0.0

        if abs(error) < CENTER_DEADBAND_ADC:
            return 0.0
        return clamp(polarity * KP_CENTER * error, -MAX_STRAFE, MAX_STRAFE)

    def advance_one_cell(self, heading):
        """เดินหน้าหนึ่งช่องตาราง

        จบการเดินได้ 2 แบบที่ถือว่าสำเร็จ คือเดินครบระยะตาม odometry หรือหยุด
        เพราะ ToF เจอกำแพงที่ระยะกลางช่องพอดี ส่วนกรณีอื่นถือว่าไม่สำเร็จ

        Returns:
            tuple: (ok, traveled_m, reason)
        """
        self.stop()
        time.sleep(SETTLE_S)

        start = self.hub.snapshot()
        start_x, start_y = start.pos_x, start.pos_y
        base_target_yaw = self.heading_yaw(heading)
        timeout = (CELL_SIZE_M / BASE_SPEED) * MOVE_TIMEOUT_RATIO
        deadline = time.time() + timeout
        reason = "timeout"
        brake_hits = 0

        while time.time() < deadline:
            snap = self.hub.snapshot()
            if not snap.fresh:
                reason = "sensor_stale:" + snap.stale_reason
                break

            traveled = math.hypot(snap.pos_x - start_x, snap.pos_y - start_y)
            if traveled >= CELL_SIZE_M:
                reason = "odometry"
                break

            # เบรกเมื่อ ToF บอกว่าถึงระยะกลางช่องที่ติดกำแพงแล้ว ต้องเห็นติดกัน
            # สองรอบเพื่อกันค่าแวบเดียว และต้องเดินมาแล้วพอสมควรเพื่อไม่ให้ค่า
            # ตอนเพิ่งออกตัวจากกำแพงเดิมมาทำให้หยุดทันที
            if (snap.tof_mm is not None and snap.tof_mm <= FRONT_STOP_MM
                    and traveled > 0.15):
                brake_hits += 1
                if brake_hits >= 2:
                    reason = "tof_stop"
                    break
            else:
                brake_hits = 0

            speed = BASE_SPEED
            if snap.tof_mm is not None and snap.tof_mm < FRONT_STOP_MM + 150:
                speed = SLOW_SPEED

            left_ir = ir_triggered(snap.ir_left)
            right_ir = ir_triggered(snap.ir_right)

            # IR 45 องศาอุดจุดบอดมุมทแยงหน้า ที่ Sharp (ยิงตรงข้าง) และ ToF
            # (ยิงตรงหน้า) มองไม่เห็น แต่ไม่ให้มันจบการเคลื่อนที่เอง เพราะระยะ
            # ที่มันติดขึ้นกับการหมุน pot ของโมดูล ซึ่งเราคุมไม่ได้จากโค้ด
            guard_dir = 0
            if USE_IR_AS_GUARD and (left_ir or right_ir):
                speed = min(speed, GUARD_SPEED)
                if left_ir and not right_ir:
                    guard_dir = 1
                elif right_ir and not left_ir:
                    guard_dir = -1

            if guard_dir:
                strafe = guard_dir * GUARD_STRAFE
            else:
                strafe = self._centering_strafe(snap)

            # โหมด slant ปรับ "เป้าหมายของมุม" ไม่ใช่ไปบวกความเร็วหมุนตรง ๆ
            # เพื่อไม่ให้ไปสู้กับตัวคุม yaw ที่กำลังดึงกลับเป้าหมายเดิมอยู่
            target_yaw = base_target_yaw
            if USE_IR_AS_SLANT and guard_dir:
                target_yaw = wrap_deg(
                    base_target_yaw + self.yaw_sign * guard_dir * IR_SLANT_BIAS_DEG)

            yaw_error = wrap_deg(target_yaw - snap.yaw)
            turn = 0.0
            if abs(yaw_error) > YAW_HOLD_DEADBAND_DEG:
                turn = clamp(KP_YAW_HOLD * yaw_error,
                             -MAX_YAW_CORRECT_DPS, MAX_YAW_CORRECT_DPS)

            self._drive(x=speed, y=strafe, z=self.yaw_sign * turn)
            time.sleep(CONTROL_DT)

        self.stop()
        time.sleep(0.15)

        snap = self.hub.snapshot()
        traveled = math.hypot(snap.pos_x - start_x, snap.pos_y - start_y)
        ok = (traveled >= CELL_SIZE_M * CELL_COMPLETE_RATIO
              or (reason == "tof_stop"
                  and traveled >= CELL_SIZE_M * TOF_STOP_MIN_RATIO))
        print("[MOVE] {0} เดินได้ {1:.3f} m (เป้า {2:.2f}) เหตุที่จบ: {3} -> {4}"
              .format(DIR_NAMES[heading], traveled, CELL_SIZE_M, reason,
                      "สำเร็จ" if ok else "ไม่สำเร็จ"))
        return ok, traveled, reason

    def backup(self, distance_m, heading):
        """ถอยกลับตามระยะที่กำหนด เพื่อกลับไปยืนกลางช่องเดิมหลังเดินไม่ผ่าน

        ถ้าไม่ถอย หุ่นจะค้างอยู่กลางทางในตำแหน่งที่ระบบไม่รู้ว่าอยู่ตรงไหน
        แล้วการเดินครั้งถัดไปจะวัดระยะจากจุดที่ผิดตั้งแต่ต้น
        """
        if distance_m < 0.03:
            return
        print("[BACK] ถอยกลับ {0:.3f} m เข้าช่องเดิม".format(distance_m))
        start = self.hub.snapshot()
        start_x, start_y = start.pos_x, start.pos_y
        target_yaw = self.heading_yaw(heading)
        deadline = time.time() + (distance_m / BACKUP_SPEED) * 2.0 + 1.0

        while time.time() < deadline:
            snap = self.hub.snapshot()
            if not snap.fresh:
                break
            if math.hypot(snap.pos_x - start_x, snap.pos_y - start_y) >= distance_m:
                break
            yaw_error = wrap_deg(target_yaw - snap.yaw)
            turn = 0.0
            if abs(yaw_error) > YAW_HOLD_DEADBAND_DEG:
                turn = clamp(KP_YAW_HOLD * yaw_error,
                             -MAX_YAW_CORRECT_DPS, MAX_YAW_CORRECT_DPS)
            self._drive(x=-BACKUP_SPEED, z=self.yaw_sign * turn)
            time.sleep(CONTROL_DT)

        self.stop()
        time.sleep(0.15)


# =====================================================================
# การคีบและวางวัตถุ
# =====================================================================
class Payload(object):
    """ควบคุมแขนกลและกริปเปอร์สำหรับคีบวัตถุไปวางที่เป้าหมาย

    ``gripper.open()`` / ``close()`` ไม่คืน action ให้ ``wait_for_completed``
    (ดู gripper.py:65) จึงต้องหน่วงเวลาเอาเอง ต่างจากแขนกลที่คืน action มา

    Args:
        arm: ออบเจกต์ robotic_arm ของหุ่น
        gripper: ออบเจกต์ gripper ของหุ่น
    """

    def __init__(self, arm, gripper):
        self.arm = arm
        self.gripper = gripper

    def pick_up(self):
        """หุบกริปเปอร์คีบวัตถุ แล้วเก็บแขนเข้าท่าวิ่ง"""
        print("[ARM] หุบกริปเปอร์คีบวัตถุ")
        self.gripper.close(power=GRIPPER_POWER)
        time.sleep(1.5)
        print("[ARM] เก็บแขนเข้าท่าวิ่งที่ {0}".format(ARM_CARRY_XY))
        self.arm.moveto(x=ARM_CARRY_XY[0], y=ARM_CARRY_XY[1]).wait_for_completed(timeout=6)
        time.sleep(0.3)

    def place(self):
        """ยื่นแขนออกไปวางวัตถุ แล้วเก็บแขนกลับ"""
        print("[ARM] ยื่นแขนไปที่ {0} เพื่อวางวัตถุ".format(ARM_PLACE_XY))
        self.arm.moveto(x=ARM_PLACE_XY[0], y=ARM_PLACE_XY[1]).wait_for_completed(timeout=6)
        time.sleep(0.5)
        self.gripper.open(power=GRIPPER_POWER)
        time.sleep(1.5)
        print("[ARM] เก็บแขนกลับท่าวิ่ง")
        self.arm.moveto(x=ARM_CARRY_XY[0], y=ARM_CARRY_XY[1]).wait_for_completed(timeout=6)
        # หุบกริปเปอร์เบา ๆ ไว้ กันนิ้วกางไปเกี่ยวกำแพงตอนถอยออก
        self.gripper.close(power=30)
        time.sleep(1.0)


# =====================================================================
# State machine หลัก - search run ด้วย Flood Fill
# =====================================================================
def run_search(hub, driver, payload):
    """เดินสำรวจด้วย Flood Fill จากช่องเริ่มต้นจนถึงช่องเป้าหมาย

    Returns:
        bool: True เมื่อถึงเป้าหมายสำเร็จ
    """
    maze = Maze(MAZE_W, MAZE_H, GOAL_CELLS)
    x, y = START_CELL
    heading = START_HEADING

    print("=" * 62)
    print("  MAZE SEARCH RUN - Flood Fill")
    print("  สนาม {0}x{1} ช่องละ {2:.2f} m | เริ่มที่ {3} หัน {4} | เป้าหมาย {5}"
          .format(MAZE_W, MAZE_H, CELL_SIZE_M, START_CELL,
                  DIR_NAMES[START_HEADING], GOAL_CELLS))
    print("=" * 62)

    if payload is not None:
        payload.pick_up()

    # นับความล้มเหลวซ้ำที่ (ช่อง, ทิศ) เดิม ใช้ตัดวงจรกรณีเดินไม่ผ่านแต่ ToF
    # ก็ไม่เห็นกำแพง (ล้อลื่น ติดขอบ ฯลฯ) ซึ่งถ้าไม่ตัดจะเลือกทิศเดิมซ้ำไปเรื่อย ๆ
    fail_key = None
    fail_count = 0

    for step in range(MAX_STEPS):
        if (x, y) in maze.goals:
            print("\n[GOAL] ถึงช่องเป้าหมาย {0} แล้ว ใช้ไป {1} ก้าว"
                  .format((x, y), step))
            driver.stop()
            if payload is not None:
                payload.place()
            print(maze.render(robot=(x, y, heading), legend=True))
            return True

        print("\n--- ก้าวที่ {0} | ช่อง ({1}, {2}) | หัน {3} ---"
              .format(step, x, y, DIR_NAMES[heading]))

        driver.stop()
        time.sleep(SETTLE_S)
        snap = hub.snapshot()
        if not snap.fresh:
            print("[ERROR] เซนเซอร์ {0} ขาดการอัปเดต หยุดเพื่อความปลอดภัย"
                  .format(snap.stale_reason))
            return False

        front, left, right = hub.read_walls_settled()
        print("ค่าดิบ -> ToF:{0} SharpL:{1} SharpR:{2} IR_L:{3} IR_R:{4}"
              .format(snap.tof_mm, snap.adc_left, snap.adc_right,
                      snap.ir_left, snap.ir_right))
        print("กำแพง -> หน้า:{0:d} ซ้าย:{1:d} ขวา:{2:d}"
              .format(front, left, right))

        maze.observe(x, y, heading, front, left, right)
        dist = maze.flood()
        print(maze.render(dist=dist, robot=(x, y, heading), legend=True))

        if dist[x][y] >= INF:
            print("\n[FAIL] จากความรู้ปัจจุบัน ไปเป้าหมายไม่ได้แล้ว "
                  "(ทุกทางที่รู้จักถูกกำแพงปิดหมด)")
            return False

        next_heading = maze.choose_next_heading(x, y, heading, dist)
        if next_heading is None:
            print("\n[FAIL] ช่องนี้ถูกล้อมทุกด้าน ออกไปไหนไม่ได้")
            return False
        print("ตัดสินใจ -> distance ที่นี่ = {0}, เดินไปทาง {1}"
              .format(dist[x][y], DIR_NAMES[next_heading]))

        heading = driver.turn_to(heading, next_heading)

        # ตรวจ ToF อีกครั้งหลังหมุนเสร็จ ก่อนออกตัวจริง เป็นด่านสุดท้ายที่กัน
        # ไม่ให้พุ่งชนกำแพง และเป็นตัวแก้ให้อัตโนมัติเมื่อ Sharp อ่านพลาดว่าโล่ง
        time.sleep(0.15)
        snap = hub.snapshot()
        if snap.front_wall():
            print("[SAFETY] หันมาแล้วเจอกำแพงที่ {0}mm - ยกเลิกการเดิน "
                  "แล้วมาร์กลงแผนที่".format(snap.tof_mm))
            maze.set_wall(x, y, heading, True)
            continue

        ok, traveled, _ = driver.advance_one_cell(heading)
        if ok:
            x, y = x + DX[heading], y + DY[heading]
            fail_key = None
            fail_count = 0
        else:
            key = (x, y, heading)
            fail_count = fail_count + 1 if key == fail_key else 1
            fail_key = key

            snap = hub.snapshot()
            if snap.front_wall():
                print("[RECOVER] ยืนยันด้วย ToF ว่ามีกำแพงจริง มาร์กลงแผนที่")
                maze.set_wall(x, y, heading, True)
            elif fail_count >= 2:
                print("[RECOVER] เดินไม่ผ่านทางเดิมเป็นครั้งที่ {0} ทั้งที่ ToF "
                      "ว่าโล่ง - ปิดทางนี้ไว้ก่อนเพื่อไม่ให้ติดวนอยู่ที่เดิม"
                      .format(fail_count))
                maze.set_wall(x, y, heading, True)
            else:
                print("[RECOVER] ToF บอกว่าข้างหน้าโล่ง แต่เดินไม่ไป "
                      "น่าจะล้อลื่นหรือติดขัด - ลองใหม่อีกครั้งก่อนตัดสิน")
            driver.backup(traveled, heading)

    print("\n[FAIL] ครบ {0} ก้าวแล้วยังไม่ถึงเป้าหมาย".format(MAX_STEPS))
    return False


# =====================================================================
# โหมด --calib : วัดค่าเซนเซอร์จริงจากหุ่นตัวนี้ในสนามนี้
# =====================================================================
def _collect(hub, samples=40, interval=0.05, label=""):
    """เก็บตัวอย่างค่าเซนเซอร์ตามจำนวนที่กำหนด แล้วคืนเป็น dict ของ list"""
    data = {"adc_l": [], "adc_r": [], "ir_l": [], "ir_r": [], "tof": []}
    for _ in range(samples):
        snap = hub.snapshot()
        if snap.adc_left is not None:
            data["adc_l"].append(snap.adc_left)
        if snap.adc_right is not None:
            data["adc_r"].append(snap.adc_right)
        if snap.ir_left is not None:
            data["ir_l"].append(int(snap.ir_left))
        if snap.ir_right is not None:
            data["ir_r"].append(int(snap.ir_right))
        if snap.tof_mm is not None:
            data["tof"].append(snap.tof_mm)
        time.sleep(interval)
    if label:
        print("      {0}: L={1} R={2}".format(label, _fmt(data["adc_l"]),
                                              _fmt(data["adc_r"])))
    return data


def _stats(values):
    """tuple: (mean, sd) หรือ (None, None) ถ้าไม่มีข้อมูล"""
    if not values:
        return None, None
    if len(values) == 1:
        return float(values[0]), 0.0
    return statistics.mean(values), statistics.pstdev(values)


def _fmt(values):
    """str: ข้อความสรุป mean/sd แบบสั้น"""
    mean, sd = _stats(values)
    if mean is None:
        return "ไม่มีข้อมูล"
    return "mean={0:6.1f} sd={1:5.1f}".format(mean, sd)


def _mode_value(values):
    """int or None: ค่าที่พบบ่อยที่สุดในลิสต์"""
    if not values:
        return None
    return max(set(values), key=values.count)


def _threshold(wall_mean, wall_sd, open_mean, open_sd):
    """คำนวณคู่ threshold แบบมี hysteresis จากสองกลุ่มตัวอย่าง

    จุดแบ่งถูกถ่วงน้ำหนักด้วยส่วนเบี่ยงเบนของแต่ละกลุ่ม กลุ่มที่ค่ากระจายมากกว่า
    จะได้พื้นที่กว้างกว่า แล้วเปิดช่องว่างรอบจุดแบ่งไว้เป็น hysteresis เพื่อไม่ให้
    ค่าที่แกว่งอยู่แถวเส้นทำให้สถานะกำแพงกระพริบไปมา

    Returns:
        tuple: (enter, exit, separation) โดย separation คือระยะห่างของสองกลุ่ม
        วัดเป็นจำนวนเท่าของผลรวมส่วนเบี่ยงเบน
    """
    gap = wall_mean - open_mean
    sd_sum = wall_sd + open_sd
    if sd_sum > 1e-9:
        split = (wall_mean * open_sd + open_mean * wall_sd) / sd_sum
        separation = abs(gap) / sd_sum
    else:
        split = (wall_mean + open_mean) / 2.0
        separation = float("inf")

    band = max(3.0 * max(wall_sd, open_sd), 0.10 * abs(gap))
    band = min(band, 0.6 * abs(gap))
    if gap > 0:
        enter, exit_ = split + band / 2.0, split - band / 2.0
    else:
        enter, exit_ = split - band / 2.0, split + band / 2.0
    return int(round(enter)), int(round(exit_)), separation


def run_calibration(hub):
    """ไล่วัดค่าเซนเซอร์ทีละขั้น แล้วพิมพ์บล็อก CONFIG ที่คัดลอกไปวางได้เลย"""
    print("\n" + "=" * 62)
    print("  โหมดคาลิเบรตเซนเซอร์ - หุ่นจะไม่ขยับตลอดกระบวนการนี้")
    print("=" * 62)
    if not hub.use_adapter:
        print("[WARN] กำลังใช้ fallback get_adc ซึ่งช้า การเก็บตัวอย่างจะนานขึ้น")

    print("\n[1/6] วางหุ่น 'กลางช่อง' ที่มีกำแพงทั้งซ้ายและขวา")
    print("      ตำแหน่งต้องอยู่กลางจริง ๆ เพราะค่านี้จะกลายเป็นจุดอ้างอิงที่")
    print("      ตัวคุมใช้ประคองหุ่นให้อยู่กลางช่องตอนวิ่ง")
    input("      พร้อมแล้วกด Enter...")
    walls = _collect(hub, label="กำแพงสองข้าง")

    ir_l_walls = _mode_value(walls["ir_l"])
    ir_r_walls = _mode_value(walls["ir_r"])
    print("      IR 45 องศาตอนนี้อ่านได้ L={0} R={1}".format(ir_l_walls, ir_r_walls))

    print("\n[2/6] ย้ายหุ่นไปช่องที่ 'ไม่มีกำแพงทั้งสองข้าง'")
    input("      พร้อมแล้วกด Enter...")
    opens = _collect(hub, label="ไม่มีกำแพง")

    print("\n[3/6] ขยับหุ่นให้ชิดกำแพง 'ด้านซ้าย' มาก ๆ (ราว 5 ซม.)")
    print("      ขั้นนี้ตรวจกับดักของ Sharp ที่เมื่อใกล้เกินพิสัย ค่าจะย้อนกลับลง")
    print("      ทำให้กำแพงที่ใกล้มากอ่านได้เท่ากับกำแพงที่ไกล")
    input("      พร้อมแล้วกด Enter...")
    very_close = _collect(hub, label="ชิดกำแพงซ้าย")

    print("\n[4/6] เอามือบังหน้าเซนเซอร์ IR 45 องศา 'ทั้งสองตัว' ไว้")
    input("      พร้อมแล้วกด Enter...")
    ir_blocked = _collect(hub, samples=20)
    print("      IR ตอนถูกบัง: L={0} R={1}".format(_mode_value(ir_blocked["ir_l"]),
                                                   _mode_value(ir_blocked["ir_r"])))

    print("\n[5/6] เอามือออกให้ IR ทั้งสองตัวโล่ง")
    input("      พร้อมแล้วกด Enter...")
    ir_clear = _collect(hub, samples=20)
    print("      IR ตอนโล่ง:   L={0} R={1}".format(_mode_value(ir_clear["ir_l"]),
                                                   _mode_value(ir_clear["ir_r"])))

    print("\n[6/6] วางหุ่น 'กลางช่อง' โดยหันหน้าชนกำแพง")
    print("      ค่านี้จะกลายเป็นระยะที่หุ่นใช้หยุดตอนเข้าช่องที่มีกำแพงข้างหน้า")
    input("      พร้อมแล้วกด Enter...")
    facing = _collect(hub, samples=30)
    tof_mean, tof_sd = _stats(facing["tof"])
    print("      ToF: {0}".format(_fmt(facing["tof"]) if facing["tof"] else "ไม่มีข้อมูล"))

    # ---------- สรุปผล ----------
    print("\n" + "=" * 62)
    print("  ผลการคาลิเบรต")
    print("=" * 62)

    problems = []
    results = {}
    for side, key, ref_name in (("ซ้าย", "adc_l", "SHARP_LEFT"),
                                ("ขวา", "adc_r", "SHARP_RIGHT")):
        wall_mean, wall_sd = _stats(walls[key])
        open_mean, open_sd = _stats(opens[key])
        if wall_mean is None or open_mean is None:
            problems.append("Sharp{0}: ไม่ได้รับค่า ADC เลย ตรวจการต่อสาย".format(side))
            continue
        enter, exit_, separation = _threshold(wall_mean, wall_sd, open_mean, open_sd)
        results[ref_name] = (enter, exit_, int(round(wall_mean)))
        mark = "ผ่าน" if separation >= 3.0 else "ไม่ผ่าน"
        print("  Sharp{0}: มีกำแพง {1:.1f}  ไม่มีกำแพง {2:.1f}  "
              "ห่างกัน {3:.1f} SD [{4}]"
              .format(side, wall_mean, open_mean, separation, mark))
        if separation < 3.0:
            problems.append(
                "Sharp{0}: สองกลุ่มห่างกันแค่ {1:.1f} SD ซึ่งน้อยเกินจะแยกได้แน่นอน "
                "ควรขยับตำแหน่งหรือมุมติดตั้งเซนเซอร์ก่อนแล้วคาลิเบรตใหม่"
                .format(side, separation))

    close_mean, _ = _stats(very_close["adc_l"])
    wall_l_mean, _ = _stats(walls["adc_l"])
    if close_mean is not None and wall_l_mean is not None and "SHARP_LEFT" in results:
        rising = results["SHARP_LEFT"][0] > results["SHARP_LEFT"][1]
        monotonic = close_mean > wall_l_mean if rising else close_mean < wall_l_mean
        print("  ชิดกำแพงมาก: {0:.1f} เทียบกับกำแพงปกติ {1:.1f} -> {2}"
              .format(close_mean, wall_l_mean,
                      "ผ่าน ค่ายังไปทางเดิม" if monotonic else "ไม่ผ่าน ค่าย้อนกลับ"))
        if not monotonic:
            problems.append(
                "Sharp ซ้าย: ที่ระยะใกล้มากค่าย้อนกลับ ถ้าหุ่นเบี่ยงไปชิดกำแพง "
                "เมื่อไหร่จะอ่านผิดว่าไม่มีกำแพง ควรถอยเซนเซอร์ให้ห่างกำแพงขึ้น")

    ir_value = None
    for side, key in (("ซ้าย", "ir_l"), ("ขวา", "ir_r")):
        blocked = _mode_value(ir_blocked[key])
        clear = _mode_value(ir_clear[key])
        if blocked is None or clear is None:
            problems.append("IR{0}: ไม่ได้รับค่า IO ตรวจการต่อสาย".format(side))
        elif blocked == clear:
            problems.append(
                "IR{0}: บังกับไม่บังได้ค่าเท่ากัน ({1}) เซนเซอร์อาจไม่ทำงาน "
                "หรือ pot ตั้งไว้สั้น/ยาวเกินไป".format(side, blocked))
        else:
            print("  IR{0}: บัง={1} โล่ง={2} -> ค่าที่แปลว่าเจอสิ่งกีดขวางคือ {1}"
                  .format(side, blocked, clear))
            if ir_value is None:
                ir_value = blocked
            elif ir_value != blocked:
                problems.append(
                    "IR ซ้ายกับขวาใช้ตรรกะกลับด้านกัน โค้ดรองรับได้ค่าเดียว "
                    "ต้องตั้ง pot หรือสลับสายให้เหมือนกันก่อน")

    if ir_value is not None and ir_l_walls == ir_value and ir_r_walls == ir_value:
        problems.append(
            "IR 45 องศาติดตั้งแต่ตอนอยู่กลางช่องที่มีกำแพงปกติ แปลว่าตั้งระยะไว้ไกล "
            "เกินไป จะติดตลอดเวลาในทางเดินทุกช่องจนไม่มีประโยชน์ "
            "ควรหมุน pot ให้ระยะสั้นลงจนไม่ติดตอนอยู่กลางช่อง")

    if tof_mean is None:
        problems.append("ToF: ไม่ได้รับค่า ตรวจว่าเสียบโมดูลและ TOF_INDEX ถูกตัวหรือไม่")
    else:
        print("  ToF ตอนจอดกลางช่องชนกำแพง: {0:.0f} mm (sd {1:.1f})"
              .format(tof_mean, tof_sd))

    print("\n" + "-" * 62)
    if problems:
        print("  พบปัญหาที่ควรแก้ก่อนลงสนามจริง:")
        for item in problems:
            print("    * {0}".format(item))
        print("-" * 62)

    print("\nคัดลอกบล็อกนี้ไปวางทับใน CONFIG ด้านบนของไฟล์:\n")
    def _show(name, value):
        print("{0:<22}= {1}".format(name, value))

    _show("SHARP_LEFT_WALL_ADC",
          "({0}, {1})".format(*results["SHARP_LEFT"][:2])
          if "SHARP_LEFT" in results else "None   # วัดไม่ได้")
    _show("SHARP_RIGHT_WALL_ADC",
          "({0}, {1})".format(*results["SHARP_RIGHT"][:2])
          if "SHARP_RIGHT" in results else "None   # วัดไม่ได้")
    _show("SHARP_LEFT_REF",
          results["SHARP_LEFT"][2] if "SHARP_LEFT" in results else "None")
    _show("SHARP_RIGHT_REF",
          results["SHARP_RIGHT"][2] if "SHARP_RIGHT" in results else "None")
    _show("IR_TRIGGERED_VALUE", ir_value if ir_value is not None else "None")
    _show("FRONT_STOP_MM",
          int(round(tof_mean)) if tof_mean is not None else "None")
    if tof_mean is not None:
        print("\n# เกณฑ์ตรวจกำแพงหน้าจะถูกคำนวณเป็น {0} mm โดยอัตโนมัติ"
              .format(int(round(tof_mean)) + int(CELL_SIZE_M * 1000 / 2)))
    print()


# =====================================================================
# โหมด --sim : ทดสอบตรรกะ Flood Fill โดยไม่ต้องต่อหุ่น
# =====================================================================
#: list: คู่ช่องที่มีกำแพงกั้นระหว่างกันในเขาวงกตจำลอง
#: ออกแบบไว้ให้เส้นทางที่สั้นที่สุดมองไม่เห็นตั้งแต่ต้น หุ่นต้องเดินเข้าทางตัน
#: ที่ (3,1) ก่อนแล้วถอยกลับมาหาทางใหม่ จึงได้ทดสอบทั้งการวางแผนและการย้อนกลับ
SIM_BLOCKED_EDGES = [
    ((0, 0), (1, 0)),
    ((0, 1), (0, 2)),
    ((1, 1), (1, 2)),
    ((2, 0), (2, 1)),
    ((1, 2), (2, 2)),
    ((3, 1), (3, 2)),
]


def _edge_direction(cell_a, cell_b):
    """int: ทิศจาก cell_a ไป cell_b"""
    for direction in range(4):
        if (cell_a[0] + DX[direction], cell_a[1] + DY[direction]) == tuple(cell_b):
            return direction
    raise ValueError("{0} กับ {1} ไม่ได้ติดกัน".format(cell_a, cell_b))


def run_sim():
    """เดินตรรกะเดียวกับของจริงบนเขาวงกตจำลอง

    ใช้ ``Maze`` และ ``choose_next_heading`` ตัวเดียวกับที่หุ่นใช้จริง ต่างกันแค่
    แทนที่จะอ่านเซนเซอร์ ก็ไปถามเขาวงกตความจริงตรง ๆ จึงยืนยันได้ว่าตรรกะการ
    วางแผนถูกต้อง ก่อนเอาไปเจอกับความไม่แน่นอนของเซนเซอร์และล้อในสนามจริง

    Returns:
        bool: True เมื่อเดินถึงเป้าหมาย
    """
    truth = Maze(MAZE_W, MAZE_H, GOAL_CELLS)
    for cell_a, cell_b in SIM_BLOCKED_EDGES:
        truth.set_wall(cell_a[0], cell_a[1], _edge_direction(cell_a, cell_b), True)

    print("=" * 62)
    print("  โหมดจำลอง - ไม่ต้องต่อหุ่น")
    print("=" * 62)
    print("\nเขาวงกตความจริง (หุ่นยังไม่รู้):")
    print(truth.render())

    known = Maze(MAZE_W, MAZE_H, GOAL_CELLS)
    x, y = START_CELL
    heading = START_HEADING
    path = [(x, y)]

    for step in range(MAX_STEPS):
        if (x, y) in known.goals:
            print("\n[GOAL] ถึงเป้าหมายใน {0} ก้าว".format(step))
            print("เส้นทางที่เดินจริง: {0}".format(
                " -> ".join(str(cell) for cell in path)))
            print("\nแผนที่ที่หุ่นสร้างได้:")
            print(known.render(dist=known.flood(), robot=(x, y, heading),
                               legend=True))
            return True

        front = truth.has_wall(x, y, heading)
        right = truth.has_wall(x, y, (heading + 1) % 4)
        left = truth.has_wall(x, y, (heading + 3) % 4)
        known.observe(x, y, heading, front, left, right)

        dist = known.flood()
        print("\n--- ก้าวที่ {0} | ช่อง ({1}, {2}) | หัน {3} | distance {4} ---"
              .format(step, x, y, DIR_NAMES[heading], dist[x][y]))
        print(known.render(dist=dist, robot=(x, y, heading), legend=True))

        if dist[x][y] >= INF:
            print("\n[FAIL] ไปเป้าหมายไม่ได้แล้ว")
            return False

        next_heading = known.choose_next_heading(x, y, heading, dist)
        if next_heading is None:
            print("\n[FAIL] ถูกล้อมทุกด้าน")
            return False

        # ตรวจความถูกต้องของตรรกะ: ทิศที่เลือกต้องไม่มีกำแพงอยู่จริง
        if truth.has_wall(x, y, next_heading):
            print("\n[BUG] เลือกเดินไปทาง {0} ทั้งที่มีกำแพงจริงอยู่"
                  .format(DIR_NAMES[next_heading]))
            return False

        heading = next_heading
        x, y = x + DX[heading], y + DY[heading]
        path.append((x, y))

    print("\n[FAIL] ครบ {0} ก้าวแล้วยังไม่ถึงเป้าหมาย".format(MAX_STEPS))
    return False


# =====================================================================
# MAIN
# =====================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Maze solver ด้วย Flood Fill สำหรับ RoboMaster EP")
    parser.add_argument("--calib", action="store_true",
                        help="วัดค่าเซนเซอร์จริง หุ่นจะไม่ขยับ")
    parser.add_argument("--sim", action="store_true",
                        help="ทดสอบตรรกะ Flood Fill โดยไม่ต้องต่อหุ่น")
    parser.add_argument("--conn", default=CONN_TYPE,
                        choices=["ap", "sta", "rndis"],
                        help="วิธีเชื่อมต่อหุ่น (ค่าเริ่มต้น {0})".format(CONN_TYPE))
    parser.add_argument("--no-payload", action="store_true",
                        help="ข้ามการคีบและวางวัตถุ ใช้ตอนดีบักเฉพาะการเดิน")
    args = parser.parse_args()

    if args.sim:
        return 0 if run_sim() else 1

    if robot is None:
        print("[ERROR] import robomaster ไม่สำเร็จ: {0}".format(ROBOT_IMPORT_ERROR))
        print("        ติดตั้ง SDK ก่อน หรือใช้ --sim เพื่อทดสอบเฉพาะตรรกะ")
        return 1

    if not args.calib:
        require_calibration()

    ep_robot = robot.Robot()
    print("กำลังเชื่อมต่อหุ่นแบบ {0} ...".format(args.conn))
    ep_robot.initialize(conn_type=args.conn)

    hub = SensorHub(ep_robot)
    success = False
    try:
        hub.start()
        if args.calib:
            run_calibration(hub)
            success = True
        else:
            # initialize() ตั้ง FREE ให้อยู่แล้ว (robot.py reset) แต่สั่งซ้ำให้ชัดเจน
            # ว่าโค้ดนี้ต้องการให้แชสซีขยับอิสระจากกิมบอล
            ep_robot.set_robot_mode(robot.FREE)
            time.sleep(0.5)

            driver = Driver(ep_robot.chassis, hub)
            driver.calibrate_yaw_sign()
            driver.set_north_reference(START_HEADING)

            payload = None
            if DO_PAYLOAD and not args.no_payload:
                payload = Payload(ep_robot.robotic_arm, ep_robot.gripper)
            else:
                print("[INFO] ข้ามการคีบและวางวัตถุ")

            success = run_search(hub, driver, payload)
    except KeyboardInterrupt:
        print("\n[STOP] ผู้ใช้สั่งหยุด")
    finally:
        try:
            ep_robot.chassis.drive_speed(x=0, y=0, z=0)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] สั่งหยุดล้อไม่สำเร็จ: {0}".format(exc))
        hub.stop()
        try:
            ep_robot.close()
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] ปิดการเชื่อมต่อไม่สำเร็จ: {0}".format(exc))

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())

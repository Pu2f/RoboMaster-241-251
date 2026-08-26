"""Assignment 01 - Maze Solver ด้วย Flood Fill (Micromouse) สำหรับ RoboMaster EP

โหมดการทำงาน
------------
สำรวจเขาวงกตด้วย Flood Fill จากช่องเริ่มต้นไปช่องเป้าหมาย ระหว่างทางหุ่นคีบ
วัตถุไว้ วางลงเมื่อถึงช่องเป้าหมาย แล้วเดินกลับไปที่ ``RETURN_CELL`` (ค่าเริ่มต้น
คือช่องเริ่มต้น) เป็นอันจบงาน

ขากลับใช้ตรรกะชุดเดียวกับขาไปทุกอย่าง คือ Flood Fill บนแผนที่ก้อนเดิมที่สะสมมา
ตลอดขาไป แค่เปลี่ยนช่องเป้าหมายเท่านั้น จึงยังเป็น search run ที่อ่านเซนเซอร์และ
เติมแผนที่ต่อไปเรื่อย ๆ ไม่ใช่ speed run ที่วิ่งตามเส้นทางที่จำไว้ตอนขาไป ผลคือ
ขากลับอาจไม่ซ้ำทางเดิม เพราะ Flood Fill ถือว่าด้านที่ยังไม่เคยเห็นเป็นทางเปิด
ไว้ก่อน จึงเลือกทางที่สั้นกว่าถ้ามันดูเป็นไปได้ แล้วค่อยแก้เอาหน้างานถ้าตัน

การได้วัตถุมาคีบคุมด้วยสองค่าที่ไม่เกี่ยวกัน ไม่ใช่ "โหมด" ที่ผูกกันเป็นชุด
- ``PICK_CELL``   หยิบที่ช่องไหน (ค่าเริ่มต้น ``START_CELL`` คือหยิบตรงจุดเริ่ม)
- ``ARM_PICK_XY`` หยิบยังไง ``None`` = คนวางใส่มือ, เป็นพิกัด = ยื่นแขนหยิบเอง

ทั้งสองค่าเดินผ่านโค้ดเส้นทางเดียวกันหมด คือ Flood Fill จะพาหุ่นไป ``PICK_CELL``
ก่อน (ถ้าเป็นช่องเริ่มต้นก็ไม่ต้องขยับ) หยิบของ แล้วค่อยตั้งเป้าใหม่ไป
``GOAL_CELLS`` จึงไม่มีเส้นทางไหนที่ไม่เคยถูกรันเลยในคอนฟิกปกติ

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
    python test_code.py --armtest  จูนท่าแขนกับวัตถุจริง หุ่นไม่เดินไปไหน
    python test_code.py --sim      ทดสอบตรรกะ Flood Fill โดยไม่ต้องต่อหุ่น
    python test_code.py            วิ่งจริงในสนาม
    python tests/run_tests.py      รันเทสต์ทั้งหมดด้วยหุ่นปลอม

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
6. ``robotic_arm.moveto()`` เป็นพิกัด "สัมบูรณ์" (robotic_arm.py:123 ส่ง mode=1)
   ไม่ใช่ระยะเลื่อนจากท่าปัจจุบัน ต้องเรียก ``recenter()`` พาแขนกลับจุดอ้างอิง
   ก่อนใช้งานครั้งแรก ไม่งั้นตำแหน่งที่สั่งจะเพี้ยนตามท่าที่แขนค้างอยู่ตอนบูต
7. ช่วงการเคลื่อนที่ของแขนคือ x 0-220 mm และ y 0-150 mm (คู่มือในโปรเจกต์นี้เอง
   docs/source/extension_module/robotic_arm_and_gripper.rst:16) และกริปเปอร์
   กางได้ราว 100 mm ค่าติดลบหรือเกินช่วงนี้ไม่มีใครดักให้ - SDK ไม่ตรวจช่วงเลย
   (util.py:150 ตั้ง start/end เป็น None) จะถูกส่งลงเฟิร์มแวร์ตรง ๆ แล้วแขนก็ไป
   ค้างที่ลิมิตกลไกเอง สังเกตได้จาก sub_position ที่ถอดค่าเป็น unsigned ('<II'
   ที่ robotic_arm.py:52) คือตำแหน่งติดลบไม่มีทางถูกรายงานกลับมาได้เลย
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
MAZE_W = 5                      # จำนวนช่องแกน X (ทิศตะวันออกเป็นบวก)
MAZE_H = 6                      # จำนวนช่องแกน Y (ทิศเหนือเป็นบวก)
CELL_SIZE_M = 0.60              # ความกว้าง 1 ช่อง หน่วยเมตร
START_CELL = (0, 0)
START_HEADING = 0                # 0=North 1=East 2=South 3=West
# ช่องที่หุ่น "ไปยืน" ตอนจบ ไม่ใช่ช่องที่ของไปอยู่ สองอย่างนี้เป็นคนละช่องกัน
# เมื่อจุดวางไม่ได้อยู่กลางช่อง เพราะแขนยื่นตรงไปข้างหน้าอย่างเดียว หุ่นจึงต้อง
# ยืนถอยออกมาหนึ่งช่องแล้วเอื้อมเข้าไปวาง (ดู AIM_SEQUENCE)
GOAL_CELLS = [(4, 4)]           # รองรับหลายช่อง เช่นโซนกลาง 2x2 ของ micromouse

# ช่องที่ให้เดินกลับไปหลังวางของเสร็จ None = วางแล้วจบตรงนั้นเลยไม่ต้องเดินกลับ
# (สั่งชั่วคราวด้วย --no-return ได้โดยไม่ต้องแก้ไฟล์)
#
# ขากลับไม่ใช่โค้ดคนละชุด เป็นการตั้ง maze.goals ใหม่แล้วปล่อยให้ลูปเดิมทำงานต่อ
# บนแผนที่ก้อนเดิม สิ่งที่ต่างจากขาไปมีอย่างเดียวคือหุ่นไม่ได้จอดอยู่กลางช่อง
# เป๊ะ ๆ ตอนเริ่ม เพราะ AIM_SEQUENCE เพิ่งเลื่อนมันไปเล็งจุดวาง ระยะที่เยื้องนี้
# ติดตัวไปทุกช่องเท่า ๆ กัน (advance_one_cell นับ odometry เป็นช่วง ไม่ใช่พิกัด
# สัมบูรณ์) จึงไม่สะสมขึ้นเรื่อย ๆ และหายไปเองในช่องแรกที่ ToF เบรกให้ที่กำแพง
RETURN_CELL = START_CELL

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
SHARP_LEFT_WALL_ADC = (455, 339)      # (enter, exit) ทำ hysteresis กันค่ากระพริบ
SHARP_RIGHT_WALL_ADC = (404, 394)    # (enter, exit)
SHARP_LEFT_REF = 397     # ค่า ADC ซ้าย ตอนหุ่นอยู่กลางช่องพอดี
SHARP_RIGHT_REF = 412          # ค่า ADC ขวา ตอนหุ่นอยู่กลางช่องพอดี
IR_TRIGGERED_VALUE = 0       # ค่า IO ตอนมีสิ่งกีดขวาง (0 หรือ 1)
FRONT_STOP_MM = 95           # ToF ที่อ่านได้ตอนหุ่นอยู่กลางช่องและหันชนกำแพง

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
BASE_SPEED = 0.25               # m/s เดินหน้าปกติ
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
# ท่าแขนทุกค่าในไฟล์นี้ต้องอยู่ในช่วง x 0-220 mm, y 0-150 mm (ดูหมายเหตุ SDK
# ข้อ 7 หัวไฟล์) y = 0 คือต่ำสุดติดพื้น ไม่มีค่าติดลบ
ARM_CARRY_XY = (0, 150)        # ตำแหน่งแขนตอนวิ่ง (mm) ต้องไม่บัง ToF
ARM_PLACE_XY = (220, 0)         # ตำแหน่งแขนตอนวางของ (mm) ยื่นไกลกว่าท่าวิ่ง
                                # ให้ของพ้นตัวหุ่นก่อนปล่อย และ y = 0 คือติดพื้น
# ท่าวางตอนจบงานแบบไม่ถึงเป้าหมาย (เซนเซอร์หลุด / ไปต่อไม่ได้ / Ctrl-C)
# ตั้งให้ยื่นสั้นกว่า ARM_PLACE_XY โดยตั้งใจ เพราะการจบแบบนี้เกิดตอนหุ่นจอดอยู่
# ที่ไหนก็ไม่รู้ - เคส "ไปเป้าหมายไม่ได้แล้ว" กับ "ถูกล้อมทุกด้าน" คือหุ่นจอด
# หันชนกำแพงพอดี ถ้ายื่นสุดแขน 220 mm ของกับนิ้วจะไปกระแทกกำแพงแทนที่จะวางลง
# พื้น ที่ 110 mm ของยังพ้นล้อหน้าแต่ไม่ชนกำแพงที่ห่าง FRONT_STOP_MM
ARM_DROP_XY = (110, 0)

# ระยะห่างจากกำแพง "ด้านหน้า" ที่ต้องการก่อนยื่นแขนวางของที่ช่องเป้าหมาย (mm)
# None = ไม่ต้องถอย ยื่นแขนวางจากตรงที่หุ่นจอดเลย
#
# มีไว้เพราะตอนเข้าช่องเป้าหมาย ToF เบรกให้ที่ FRONT_STOP_MM ซึ่งใกล้กำแพงกว่า
# ระยะที่แขนยื่นออกไป (ARM_PLACE_XY) มาก วางจากตรงนั้นของกับนิ้วจะกระแทกกำแพง
# แทนที่จะลงพื้น การถอยคือวิธีให้แขนมีที่ยื่นโดยไม่ต้องหดระยะวางให้สั้นลง
#
# ข้อจำกัดที่ต้องรู้ก่อนตั้งค่า
# 1. บังคับได้เฉพาะกำแพงหน้า เพราะมีแต่ ToF ที่คืนค่าเป็นมิลลิเมตรจริง Sharp
#    ข้างคืนแค่ ADC ดิบที่บอกได้ว่า "มีกำแพงไหม" กับ "ชิดกว่าหรือห่างกว่าจุด
#    กึ่งกลางช่อง" แปลงเป็นระยะไม่ได้ ด้านข้างจึงทำได้แค่ประคองให้อยู่กลางช่อง
#    ระหว่างถอย ซึ่งก็คือระยะห่างด้านข้างที่มากที่สุดเท่าที่ช่องกว้างเท่านี้จะให้ได้
# 2. ในช่องกว้าง CELL_SIZE_M เมตร กำแพงซ้ายขวาห่างกันแค่นั้น ค่าที่เกินครึ่งช่อง
#    จึงเป็นไปไม่ได้อยู่แล้วสำหรับด้านข้าง ต่อให้หุ่นเป็นจุดเดียว
# 3. หุ่นถอยได้ไม่เกินระยะที่เพิ่งเดินเข้าช่องนี้มา (พื้นที่ที่ยืนยันแล้วว่าโล่ง)
#    ถ้าเป้าหมายคือช่องเริ่มต้นที่ไม่ได้เดินมา จะไม่ถอยเลย
GOAL_WALL_CLEARANCE_MM = 400

# ---------- การเล็งเป้าก่อนวางของ ----------
# ใช้เมื่อจุดวางถูกกำหนดเป็น "ระยะจากกำแพง" ไม่ใช่ "กลางช่อง" ซึ่งกลไกเดินทีละ
# ช่องเล็งให้ไม่ได้ ตั้งเป็น [] = ไม่เล็ง ถอยตาม GOAL_WALL_CLEARANCE_MM แล้ววาง
# จากตรงที่จอด (พฤติกรรมก่อนมีฟีเจอร์นี้) ถ้าตั้ง AIM_SEQUENCE ไว้ ค่า
# GOAL_WALL_CLEARANCE_MM จะถูกข้ามไปเอง เพราะการเล็งคุมระยะได้ละเอียดกว่า
#
# แต่ละขั้นคือ (ทิศที่ต้องหันก่อนวัด, ระยะ ToF ที่ต้องการ หน่วย mm)
#   ทิศ None  = ใช้ทิศที่มาถึงเลย ไม่ต้องหัน
#   ระยะ None = หันอย่างเดียว ไม่จัดระยะ ใช้ตำแหน่งตามแกนนั้นที่ได้มาตอนเดินเข้า
#               ช่อง ซึ่ง Sharp ประคองไว้เทียบกำแพงข้างระหว่างเดิน ไม่ใช่ odometry
#               ใช้เมื่อวัดแล้วพบว่าตำแหน่งที่ได้มาเองตรงเป้าอยู่แล้ว การจัดระยะ
#               ซ้ำมีแต่จะดันหุ่นออกจากจุดที่ถูกอยู่แล้ว
#   ทิศของขั้นสุดท้ายคือทิศที่ยื่นแขนวาง เพราะแขนยื่นตรงไปข้างหน้าอย่างเดียว
#   ห้ามใส่ทิศที่ ToF มองไม่เห็นกำแพง (เช่นด้านที่เป็นประตูเข้าห้อง)
#
# วิธีหาตัวเลขสองตัวนี้: อย่าคำนวณ เพราะระหว่าง "ToF อ่านได้เท่าไร" กับ "ของไป
# ตกตรงไหน" มีค่าคงที่ที่ไม่มีในโค้ดหลายตัว (ToF ล้ำหน้าจุดหมุนเท่าไร ฐานแขน
# อยู่ตรงไหน จุดหมุนเลื่อนตอนหมุนตัวเท่าไร) ให้รันหนึ่งรอบด้วยค่าเริ่มต้นนี้
# แล้ววัดด้วยตลับเมตรว่าของไปตกห่างกำแพงเท่าไรจริง จากนั้นชดเชยครั้งเดียวจบ
# เพราะสองแกนแยกกันสนิทและเป็นเชิงเส้น 1:1
#
#   ขั้นที่วัดกำแพง "ตรงข้าม" กับด้านที่นับระยะเป้า:  ค่าใหม่ = เดิม - (เป้า - วัดได้)
#   ขั้นที่วัดกำแพง "เดียวกัน" กับด้านที่นับระยะเป้า:  ค่าใหม่ = เดิม + (เป้า - วัดได้)
#
# เงื่อนไขเดียวคือลำดับต้องเหมือนเดิมทุกครั้ง เพราะค่าคงที่ทั้งหมดถูกกลืนไว้ในลำดับ
#
# ค่าด้านล่างคือของสนามที่ห้องเป้าหมายเป็น 2x2 ช่อง หุ่นเข้าห้องได้ทางเดียวคือ
# ตกลงมาจากทางเหนือเข้าช่อง (4,4) หันใต้ แล้วเป้าอยู่ห่างกำแพงเหนือกับกำแพง
# ตะวันตกของห้องด้านละ 400 mm
AIM_SEQUENCE = [
    (2, 560),       # หันใต้ วัดกำแพงล่างของห้อง (กำแพงเหนือเป็นประตู มองไม่เห็น)
    (3, None),      # หันตะวันตกแล้ววางเลย ตำแหน่งตะวันออก-ตะวันตกที่ได้มาตอนเดิน
                    # เข้าช่องตรงเป้าอยู่แล้ว เพราะ Sharp ประคองเทียบกำแพงข้าง
                    # ถ้าวัดแล้วเยื้อง ให้ใส่ระยะ ToF เทียบกำแพงตะวันตกแทน None
]
AIM_MAX_MOVE_M = 0.35           # ขยับได้ไกลสุดต่อหนึ่งขั้น กันความเสียหายเมื่อ ToF เพี้ยน

GRIPPER_POWER = 50
GRIPPER_TUCK_POWER = 30         # แรงหุบเบา ๆ ตอนมือเปล่า กันนิ้วเกี่ยวกำแพง
PAYLOAD_LOAD_S = 2.0            # เวลากางกริปเปอร์ค้างไว้ให้วางวัตถุก่อนหุบคีบ
GRIPPER_ACT_S = 1.5             # เวลารอให้นิ้วขยับจนสุด (ไม่มี action ให้รอ)
GRIPPER_RELEASE_TRIES = 3       # สั่งกางซ้ำได้กี่ครั้งตอนปล่อยของ
GRIPPER_STATUS_FREQ = 5         # Hz ของ sub_status สถานะเปลี่ยนช้า ไม่ต้องถี่
GRIPPER_STATUS_STALE_S = 1.5    # เกินนี้ถือว่าสถานะค้างเก่า เอามาตัดสินไม่ได้
ARM_TIMEOUT_S = 6               # timeout ของ action แขนกล (วินาที)
ARM_SETTLE_S = 0.3              # รอให้แขนหยุดสั่นหลังจบ action ก่อนสั่งท่าต่อไป
PLACE_SETTLE_S = 0.5            # รอให้แขนนิ่งที่ท่าวางก่อนกางนิ้ว กันวัตถุล้ม
GRIPPER_TUCK_S = 1.0            # รอให้นิ้วหุบเก็บจนสุดตอนมือเปล่า

# ---------- payload: หยิบที่ไหน และหยิบยังไง ----------
# สองค่านี้ไม่เกี่ยวกัน ผสมกันได้อิสระ ปิดการคีบทั้งหมดด้วย --no-payload
#
# PICK_CELL = ช่องที่หุ่นไปยืนหยิบของ
#   START_CELL  หยิบตรงจุดเริ่มโดยไม่ต้องขยับ (ค่าเริ่มต้น)
#   (1, 0) ฯลฯ  Flood Fill พาไปช่องนั้นก่อน แล้วค่อยตั้งเป้าใหม่ไป GOAL_CELLS
#   ถ้าให้หุ่นวิ่งไปหยิบที่ช่องอื่น ต้องวางของไว้ "กลางช่อง" เท่านั้น เพราะ
#   1. ToF จะเห็นแล้วเบรกให้เองที่ FRONT_STOP_MM ซึ่งเป็นระยะหยิบที่ซ้ำเดิมได้
#   2. ถ้าวางชิดขอบช่อง ToF จะเห็นเป็นกำแพงตั้งแต่ยังอยู่ช่องก่อนหน้า (เกณฑ์ =
#      FRONT_STOP_MM + ครึ่งช่อง) แล้ว Maze จะปิดทางเข้าช่องนั้น "ถาวร" เพราะ
#      set_wall เพิ่มกำแพงได้อย่างเดียว ลบออกไม่ได้ (ดู Maze.set_wall)
#
# ARM_PICK_XY = วิธีหยิบ
#   None        กางกริปเปอร์ค้าง PAYLOAD_LOAD_S วินาทีให้คนวางของใส่มือ
#               ทนต่อการวางคลาดเคลื่อนที่สุด ไม่ต้องจูนอะไรเลย (ค่าเริ่มต้น)
#   (x, y)      ยื่นแขนลงไปหยิบเองที่ท่านั้น ต้องจูนกับของจริงก่อนด้วย --armtest
#               และของต้องอยู่ในระยะเอื้อม (x 0-220 mm, y 0-150 mm)
PICK_CELL = START_CELL          # หยิบตรงจุดเริ่ม ไม่ต้องขยับไปไหน
# ARM_PICK_XY = None              # คนวางของใส่มือให้
# PICK_CELL = (1, 0)            # ให้หุ่นวิ่งไปหยิบเองที่ช่องนี้
ARM_PICK_XY = (220, 0)        # ให้ยื่นแขนหยิบเองที่ท่านี้ (จูนด้วย --armtest)

# แขน EP ไม่มีแกนหมุนซ้ายขวา (moveto รับแค่ x, y) การเล็งจึงต้องอาศัยแชสซีหัน
# ให้ตรงอย่างเดียว ปกติไม่ต้องตั้ง เพราะหุ่นจอดหันหน้าใส่ของอยู่แล้วตอน ToF เบรก
# ตั้งเมื่อรู้ว่าของเยื้องไปทางใดทางหนึ่งของช่องเท่านั้น
PICK_HEADING = None             # ทิศที่ต้องหันก่อนหยิบ (0=N 1=E 2=S 3=W)

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


def sharp_polarity(thresholds):
    """+1 ถ้า ADC สูง = อยู่ใกล้, -1 ถ้า ADC ต่ำ = อยู่ใกล้

    อนุมานจากลำดับของ (enter, exit) ที่ ``--calib`` คำนวณมาให้ จึงไม่ต้องมี
    ค่าคอนฟิกแยกอีกตัว และไม่ต้องสมมติว่าเซนเซอร์ตอบสนองไปทางไหน

    รับ threshold ของข้างที่ถามมาโดยตรง ไม่ใช่อ่านจากข้างซ้ายข้างเดียวแล้วเหมาว่า
    ขวาเหมือนกัน เพราะ Sharp สองตัวอาจเป็นคนละรุ่นหรือต่อสลับขั้วกัน ซึ่งจะทำให้
    การประคองกลางช่องดันผิดทางแบบเงียบ ๆ โดยไม่มีอะไรเตือน

    Args:
        thresholds (tuple): (enter, exit) ของข้างนั้น

    Returns:
        int: +1 หรือ -1
    """
    enter, exit_ = thresholds
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
    """เจ้าของ subscription ของเซนเซอร์นำทาง และเป็นทางเดียวที่ส่วนอื่นอ่านค่า

    สถานะกริปเปอร์ไม่ได้อยู่ที่นี่ ``Payload`` ถือ ``gripper.sub_status`` ของ
    ตัวเองไว้ เพราะไม่เกี่ยวกับการนำทางและต้องไม่ถูกสมัครตอน ``--no-payload``

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
        #: bool: True เมื่อสมัคร sub_adapter ไปแล้วจริง ใช้ตัดสินตอน stop
        self._adapter_subscribed = False
        self._poll_cache = (None, None)
        self._poll_t = 0.0
        #: float: เวลาล่าสุดที่ fallback อ่าน Sharp ได้ครบสองข้าง 0 = ยังไม่เคยได้เลย
        self._poll_ok_t = 0.0
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
        if self.use_adapter:
            self._adaptor.sub_adapter(freq=DDS_FREQ, callback=self._on_adapter)
            self._adapter_subscribed = True
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
        targets = [("distance", self._sensor.unsub_distance),
                   ("attitude", self._chassis.unsub_attitude),
                   ("position", self._chassis.unsub_position)]
        # ดูจากธง subscribe ไม่ใช่จาก use_adapter เพราะ use_adapter อาจถูกปิดทีหลัง
        # ตอนที่สมัครไปแล้วแต่ไม่มีข้อมูลส่งมา ซึ่งกรณีนั้นยังต้องปลดออกอยู่ดี
        if self._adapter_subscribed:
            targets.insert(1, ("adapter", self._adaptor.unsub_adapter))
        for name, fn in targets:
            try:
                fn()
            except Exception as exc:                    # noqa: BLE001
                print("[WARN] unsub {0} ล้มเหลว: {1}".format(name, exc))
        self._adapter_subscribed = False
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
        # นับว่าสดเฉพาะรอบที่อ่าน Sharp ได้ครบสองข้าง เพราะการตัดสินกำแพงข้างพึ่งพา
        # สองค่านี้ ถ้า get_adc พังจนคืน None ตลอด snapshot ต้องมองเห็นว่า adapter
        # ค้าง ไม่ใช่ปล่อยให้ fresh เป็น True แล้วหุ่นวิ่งต่อโดยมองไม่เห็นกำแพงข้าง
        if adc["l"] is not None and adc["r"] is not None:
            self._poll_ok_t = now
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
                adapter_age = None      # โหมด fallback คำนวณหลังอ่านค่าเสร็จ

        if ad is not None:
            adc_left = ad[self._adapter_index(*SHARP_LEFT)]
            adc_right = ad[self._adapter_index(*SHARP_RIGHT)]
            ir_left = io[self._adapter_index(*IR_LEFT_45)]
            ir_right = io[self._adapter_index(*IR_RIGHT_45)]
        else:
            adc, io_map = self._poll_adaptor(now)
            adc_left, adc_right = adc["l"], adc["r"]
            ir_left, ir_right = io_map["l"], io_map["r"]
            adapter_age = now - self._poll_ok_t if self._poll_ok_t else 1e9

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

        แต่ละข้างถูกคูณ polarity ของตัวเองก่อน จนกลายเป็นค่า "ชิดกว่าจุดอ้างอิง
        เท่าไร" ที่บวกเสมอเมื่อเข้าใกล้กำแพงข้างนั้น หลังจากนั้นสองข้างอยู่ในหน่วย
        เดียวกันแล้วจึงเอามาลบกันได้ตรง ๆ แม้ Sharp สองตัวจะตอบสนองคนละทาง
        """
        left = wall_from_adc(snap.adc_left, SHARP_LEFT_WALL_ADC)
        right = wall_from_adc(snap.adc_right, SHARP_RIGHT_WALL_ADC)
        near_left = near_right = None
        if left:
            near_left = (sharp_polarity(SHARP_LEFT_WALL_ADC)
                         * (snap.adc_left - SHARP_LEFT_REF))
        if right:
            near_right = (sharp_polarity(SHARP_RIGHT_WALL_ADC)
                          * (snap.adc_right - SHARP_RIGHT_REF))

        if near_left is not None and near_right is not None:
            # มีกำแพงสองข้าง: คุมให้ความชิดซ้ายกับความชิดขวาเท่ากัน
            error = near_left - near_right
        elif near_left is not None:
            # มีกำแพงข้างเดียว: คุมให้ระยะถึงกำแพงนั้นเท่ากับค่าอ้างอิง
            error = near_left
        elif near_right is not None:
            error = -near_right
        else:
            return 0.0

        if abs(error) < CENTER_DEADBAND_ADC:
            return 0.0
        return clamp(KP_CENTER * error, -MAX_STRAFE, MAX_STRAFE)

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

    def _travel(self, speed, heading, budget_m, stop=None, center=True):
        """ลูปเคลื่อนที่ตามแนวที่หันอยู่ ใช้ร่วมกันทุกการเดินที่ไม่ใช่เต็มช่อง

        รวมสิ่งที่ทุกการเดินต้องทำเหมือนกันไว้ที่เดียว คือประคอง yaw ให้ตรงทิศ
        ประคองตัวให้อยู่กลางช่องด้วย Sharp นับระยะที่ขยับจริงจาก odometry และ
        หยุดเมื่อเซนเซอร์ขาดการอัปเดต ส่วนที่ต่างกันของแต่ละงานอยู่ที่ ``stop``
        กับทิศของ ``speed`` เท่านั้น

        Args:
            speed (float): m/s บวก = เดินหน้า ลบ = ถอย
            heading (int): ทิศที่ต้องประคอง yaw ไว้
            budget_m (float): ระยะทางสูงสุดที่ยอมให้ขยับ
            stop: ฟังก์ชันรับ snapshot คืน True เมื่อถึงเงื่อนไขที่ต้องการ
                None = เดินจนครบ budget_m
            center (bool): True = ให้ Sharp ประคองกลางช่องระหว่างทาง

        Returns:
            tuple: (moved_m, reason) โดย reason เป็น "stop" / "budget" /
                "sensor_stale:<ชื่อสตรีม>" / "timeout"
        """
        start = self.hub.snapshot()
        start_x, start_y = start.pos_x, start.pos_y
        target_yaw = self.heading_yaw(heading)
        deadline = (time.time()
                    + (budget_m / abs(speed)) * MOVE_TIMEOUT_RATIO + 1.0)
        reason = "timeout"

        while time.time() < deadline:
            snap = self.hub.snapshot()
            if not snap.fresh:
                reason = "sensor_stale:" + snap.stale_reason
                break
            if stop is not None and stop(snap):
                reason = "stop"
                break
            if math.hypot(snap.pos_x - start_x,
                          snap.pos_y - start_y) >= budget_m:
                reason = "budget"
                break

            yaw_error = wrap_deg(target_yaw - snap.yaw)
            turn = 0.0
            if abs(yaw_error) > YAW_HOLD_DEADBAND_DEG:
                turn = clamp(KP_YAW_HOLD * yaw_error,
                             -MAX_YAW_CORRECT_DPS, MAX_YAW_CORRECT_DPS)
            strafe = self._centering_strafe(snap) if center else 0.0
            self._drive(x=speed, y=strafe, z=self.yaw_sign * turn)
            time.sleep(CONTROL_DT)

        self.stop()
        time.sleep(0.15)
        snap = self.hub.snapshot()
        moved = math.hypot(snap.pos_x - start_x, snap.pos_y - start_y)
        return moved, reason

    def align_to_wall(self, target_mm, heading, budget_m, floor_mm=None):
        """เดินหน้าหรือถอยจนกำแพงที่หันหน้าใส่ห่างเท่ากับ target_mm

        นี่คือความสามารถพื้นฐานที่ทำให้หุ่นจอดที่ตำแหน่งย่อยในช่องได้ ไม่ใช่แค่
        กลางช่อง งานอย่างการวางของให้ตรงเป้าที่กำหนดเป็นระยะจากกำแพง จึงทำได้
        ด้วยการเรียกเมธอดนี้ทีละแกน โดยหันหน้าเข้าหากำแพงของแกนนั้น

        เหตุที่วัดได้แค่กำแพงด้านหน้า: ToF เป็นเซนเซอร์ตัวเดียวในหุ่นที่คืนค่า
        เป็นมิลลิเมตรจริง และมันยิงไปข้างหน้าอย่างเดียว Sharp ข้างคืนแค่ ADC ดิบ
        ที่บอกได้ว่ามีกำแพงไหมกับชิดกว่าหรือห่างกว่ากลางช่อง แปลงเป็นระยะไม่ได้
        จึงได้แค่ประคองไม่ให้เบียดกำแพงข้างระหว่างขยับ

        การขยับตามแนวที่หันอยู่ไม่กระทบตำแหน่งตามแกนตั้งฉาก การจัดทีละแกนจึง
        ไม่รบกวนกัน ตราบใดที่ไม่ใช้ strafe มาขยับเอง

        Args:
            target_mm: ระยะที่ต้องการให้ ToF อ่านได้เมื่อจบ
            heading: ทิศที่หุ่นหันอยู่ ใช้ประคอง yaw
            budget_m: ขยับได้ไกลสุดกี่เมตร กันความเสียหายเมื่อ ToF อ่านเพี้ยน
            floor_mm: ห้ามเข้าใกล้กำแพงกว่านี้ None = ใช้ FRONT_STOP_MM

        Returns:
            tuple: (tof_mm หลังจัด, moved_m, reason)
        """
        floor = FRONT_STOP_MM if floor_mm is None else floor_mm
        snap = self.hub.snapshot()
        if snap.tof_mm is None:
            print("[ALIGN] ToF ไม่เห็นกำแพงในระยะวัด จัดระยะไม่ได้")
            return None, 0.0, "no_wall"

        error_mm = snap.tof_mm - target_mm
        if abs(error_mm) <= ALIGN_TOLERANCE_MM:
            print("[ALIGN] ToF = {0}mm ตรงเป้า {1}mm อยู่แล้ว"
                  .format(snap.tof_mm, target_mm))
            return snap.tof_mm, 0.0, "already_there"
        if budget_m <= 0.0:
            print("[ALIGN] ToF = {0}mm ห่างเป้า {1}mm แต่ขยับไม่ได้ (งบ 0)"
                  .format(snap.tof_mm, target_mm))
            return snap.tof_mm, 0.0, "no_room"

        # ToF มากกว่าเป้า = อยู่ไกลกำแพงเกินไป ต้องเดินหน้าเข้าหา และกลับกัน
        forward = error_mm > 0
        if forward and target_mm < floor:
            print("[ALIGN] เป้า {0}mm ใกล้กว่าระยะต่ำสุด {1}mm ไม่เดินเข้าไป"
                  .format(target_mm, floor))
            return snap.tof_mm, 0.0, "below_floor"

        # ขยับเกินระยะที่ผิดอยู่ไม่มีประโยชน์ เอาค่าที่น้อยกว่าเป็นงบระยะทาง
        budget = min(abs(error_mm) / 1000.0, budget_m)
        print("[ALIGN] ToF = {0}mm เป้า {1}mm -> {2} ไม่เกิน {3:.3f} m "
              "(งบ {4:.3f} m)"
              .format(snap.tof_mm, target_mm, "เดินหน้า" if forward else "ถอย",
                      budget, budget_m))

        if forward:
            def reached(s):
                return s.tof_mm is None or s.tof_mm <= max(target_mm, floor)
        else:
            def reached(s):
                return s.tof_mm is None or s.tof_mm >= target_mm

        speed = ALIGN_SPEED if forward else -ALIGN_SPEED
        moved, reason = self._travel(speed, heading, budget, stop=reached)

        snap = self.hub.snapshot()
        got = ("ไกลเกินระยะวัด" if snap.tof_mm is None
               else "{0}mm".format(snap.tof_mm))
        print("[ALIGN] ขยับ {0:.3f} m แล้ว ToF = {1} (เป้า {2}mm) เหตุที่จบ: {3}"
              .format(moved, got, target_mm, "ถึงเป้า" if reason == "stop"
                      else reason))
        if reason == "budget":
            print("[WARN] ใช้งบระยะทางหมดก่อนถึงเป้า ตำแหน่งยังไม่ตรงที่ตั้งไว้")
        return snap.tof_mm, moved, reason

    def back_off_from_wall(self, clearance_mm, heading, limit_m):
        """ถอยจนกำแพงหน้าห่าง "อย่างน้อย" clearance_mm เพื่อเปิดที่ให้แขนยื่น

        ต่างจาก ``align_to_wall`` ตรงที่นี่เป็นเกณฑ์ขั้นต่ำ ไม่ใช่ค่าเป้า ห่าง
        เกินไม่ใช่ปัญหาจึงไม่ดึงหุ่นกลับเข้าหากำแพง หน้าที่ของมันคือกันแขนชน
        กำแพงตอนวางของ ไม่ใช่การเล็งตำแหน่ง

        Args:
            clearance_mm: ระยะขั้นต่ำจาก ToF ถึงกำแพงหน้า
            heading: ทิศที่หุ่นหันอยู่ ใช้ประคอง yaw ระหว่างถอย
            limit_m: ถอยได้ไกลสุดกี่เมตร 0 = ห้ามถอย

        Returns:
            tuple: (tof_mm หลังถอย, moved_m, reason)
        """
        snap = self.hub.snapshot()
        if snap.tof_mm is None:
            print("[BACKOFF] ToF ไม่เห็นกำแพงในระยะวัด ไม่ต้องถอย")
            return None, 0.0, "no_wall"
        if snap.tof_mm >= clearance_mm:
            print("[BACKOFF] กำแพงหน้าห่าง {0}mm อยู่แล้ว (ต้องการ {1}) ไม่ต้องถอย"
                  .format(snap.tof_mm, clearance_mm))
            return snap.tof_mm, 0.0, "already_clear"
        if limit_m <= 0.0:
            print("[BACKOFF] กำแพงหน้าห่างแค่ {0}mm แต่ถอยไม่ได้ "
                  "(ไม่มีพื้นที่ข้างหลังที่ยืนยันแล้วว่าโล่ง)".format(snap.tof_mm))
            return snap.tof_mm, 0.0, "no_room"

        # ถอยเกินที่ต้องการไม่มีประโยชน์ และถอยเกิน limit_m คือถอยเข้าไปในพื้นที่
        # ที่ยังไม่รู้ว่าโล่งจริงไหม จึงเอาค่าที่น้อยกว่าเป็นงบระยะทาง
        budget_m = min((clearance_mm - snap.tof_mm) / 1000.0, limit_m)
        print("[BACKOFF] กำแพงหน้าห่าง {0}mm ต้องการ {1}mm -> ถอยไม่เกิน "
              "{2:.3f} m (เพดาน {3:.3f} m)"
              .format(snap.tof_mm, clearance_mm, budget_m, limit_m))

        moved, reason = self._travel(
            -ALIGN_SPEED, heading, budget_m,
            stop=lambda s: s.tof_mm is None or s.tof_mm >= clearance_mm)
        if reason == "stop":
            reason = "clear"
        elif reason == "budget":
            reason = "limit"

        snap = self.hub.snapshot()
        got = ("ไกลเกินระยะวัด" if snap.tof_mm is None
               else "{0}mm".format(snap.tof_mm))
        print("[BACKOFF] ถอยไป {0:.3f} m กำแพงหน้าห่าง {1} (ต้องการ {2}mm) "
              "เหตุที่จบ: {3}".format(moved, got, clearance_mm, reason))
        if reason == "limit":
            print("[WARN] ถอยจนสุดพื้นที่ที่ปลอดภัยแล้วแต่ยังไม่ได้ระยะที่ตั้งไว้ "
                  "แขนอาจยังชนกำแพงตอนวาง - ลดค่า GOAL_WALL_CLEARANCE_MM "
                  "หรือหด x ของ ARM_PLACE_XY ลง")
        return snap.tof_mm, moved, reason

    def backup(self, distance_m, heading):
        """ถอยกลับตามระยะที่กำหนด เพื่อกลับไปยืนกลางช่องเดิมหลังเดินไม่ผ่าน

        ถ้าไม่ถอย หุ่นจะค้างอยู่กลางทางในตำแหน่งที่ระบบไม่รู้ว่าอยู่ตรงไหน
        แล้วการเดินครั้งถัดไปจะวัดระยะจากจุดที่ผิดตั้งแต่ต้น

        ไม่ให้ Sharp ประคองกลางช่องระหว่างถอย เพราะกรณีที่เรียกเมธอดนี้คือหุ่น
        เพิ่งเดินไม่ผ่าน อาจติดขัดหรือเบียดอะไรอยู่ การเพิ่มการเลื่อนข้างเข้าไป
        ตอนนั้นมีแต่จะทำให้เดาไม่ออกว่าหุ่นไปจบตรงไหน
        """
        if distance_m < 0.03:
            return
        print("[BACK] ถอยกลับ {0:.3f} m เข้าช่องเดิม".format(distance_m))
        self._travel(-BACKUP_SPEED, heading, distance_m, center=False)


# =====================================================================
# การคีบและวางวัตถุ
# =====================================================================
class Payload(object):
    """ควบคุมแขนกลและกริปเปอร์สำหรับคีบวัตถุไปวางที่เป้าหมาย

    ``gripper.open()`` / ``close()`` ไม่คืน action ให้ ``wait_for_completed``
    (ดู gripper.py:65) จึงต้องหน่วงเวลาเอาเอง ต่างจากแขนกลที่คืน action มา

    ``arm.moveto()`` ใช้พิกัดสัมบูรณ์ ``pick_up()`` จึง ``recenter()`` แขนก่อน
    เสมอ เพื่อให้ ARM_CARRY_XY / ARM_PLACE_XY ตรงกันทุกครั้งที่รัน

    คำสั่งแขนและกริปเปอร์ทุกคำสั่งถูกห่อ try/except ไว้ ถ้าฮาร์ดแวร์ไม่ตอบจะ
    พิมพ์เตือนแล้วไปต่อ ไม่ลาก search run ทั้งรอบล้มไปด้วย

    คลาสนี้ถือ subscription ของตัวเอง (``gripper.sub_status``) ไม่ได้ฝากไว้กับ
    SensorHub เพราะสถานะกริปเปอร์ไม่เกี่ยวกับการนำทาง และต้องไม่ถูกสมัครเลยตอน
    สั่ง ``--no-payload`` ผู้เรียกต้องเรียก ``start()`` ก่อนใช้ และ ``stop()``
    ตอนจบงาน

    Args:
        arm: ออบเจกต์ robotic_arm ของหุ่น
        gripper: ออบเจกต์ gripper ของหุ่น
    """

    def __init__(self, arm, gripper):
        self.arm = arm
        self.gripper = gripper

        self._lock = threading.Lock()
        self._status = ""
        self._status_t = 0.0
        self._subscribed = False
        #: bool: True เมื่อเชื่อว่าคีบวัตถุอยู่ ใช้ตัดสินว่าต้องปล่อยตอนจบไหม
        self.holding = False
        #: bool: True เมื่อสั่งหุบคีบของไปแล้วและยังไม่ได้กางปล่อย
        #:
        #: แยกจาก ``holding`` เพราะ ``holding`` เชื่อ ``_confirm_grip()`` ซึ่ง
        #: ตัดสินผิดฝั่งอันตรายได้ - ของบางหรือนิ่มทำให้นิ้วหุบเกือบสุดจนสถานะ
        #: รายงาน "closed" (= ไม่มีอะไรคาอยู่) ทั้งที่ยังคีบของอยู่จริง ถ้าเชื่อ
        #: ``holding`` อย่างเดียว ตอนจบงานจะไม่ปล่อยอะไรเลย ของค้างในมือและ
        #: มอเตอร์บีบค้างยาวจนกว่าจะปิดเครื่อง ธงนี้จึงบันทึก "ข้อเท็จจริงเชิง
        #: คำสั่ง" ที่ตัดสินผิดไม่ได้ คือสั่งหุบไปแล้วหรือยัง
        self.grip_closed = False

    # ---------- callback (ทำงานบนเธรดของ DDS) ----------
    def _on_status(self, status):
        # dds.py:132 ส่งค่าที่ data_info() คืนมาให้ตรง ๆ ของกริปเปอร์เป็นสตริง
        # เดี่ยว "opened" / "closed" / "normal" (gripper.py:36) ไม่ใช่ tuple
        now = time.time()
        with self._lock:
            self._status = status
            self._status_t = now

    def start(self):
        """สมัครรับสถานะกริปเปอร์ ถ้าสมัครไม่ได้ก็ยังทำงานต่อได้แบบไม่ยืนยัน"""
        try:
            self.gripper.sub_status(freq=GRIPPER_STATUS_FREQ,
                                    callback=self._on_status)
            self._subscribed = True
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] สมัคร gripper.sub_status ไม่สำเร็จ: {0}".format(exc))
            print("[WARN] จะข้ามการยืนยันว่าคีบวัตถุติดจริง")

    def stop(self):
        """ยกเลิก subscription ของกริปเปอร์ (เรียกซ้ำได้ ไม่ throw)"""
        if not self._subscribed:
            return
        try:
            self.gripper.unsub_status()
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] unsub gripper status ล้มเหลว: {0}".format(exc))
        self._subscribed = False

    def status(self):
        """สถานะกริปเปอร์ล่าสุด

        Returns:
            str: "opened" / "closed" / "normal" หรือ "" เมื่อยังไม่มีข้อมูล
                หรือข้อมูลค้างเก่าเกิน GRIPPER_STATUS_STALE_S
        """
        with self._lock:
            if self._status_t == 0.0:
                return ""
            if time.time() - self._status_t > GRIPPER_STATUS_STALE_S:
                return ""
            return self._status

    # ---------- คำสั่งฮาร์ดแวร์ (กลืน exception ไม่ให้ล้มทั้งรอบ) ----------
    def _arm_recenter(self):
        """พาแขนกลับจุดอ้างอิงก่อนใช้พิกัดสัมบูรณ์ (robotic_arm.py:105)"""
        print("[ARM] พาแขนกลับจุดอ้างอิง")
        try:
            self.arm.recenter().wait_for_completed(timeout=ARM_TIMEOUT_S)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] พาแขนกลับจุดอ้างอิงไม่สำเร็จ: {0}".format(exc))
            print("[WARN] ตำแหน่งแขนที่สั่งต่อจากนี้อาจเพี้ยน")
            return False
        time.sleep(ARM_SETTLE_S)
        return True

    def _arm_moveto(self, xy, label):
        """ขยับแขนไปพิกัดสัมบูรณ์ xy

        Args:
            xy: tuple (x, y) หน่วย mm
            label: ชื่อท่าที่เอาไว้พิมพ์ log

        Returns:
            bool: True เมื่อขยับจนจบ action
        """
        print("[ARM] ขยับแขนไป{0} {1}".format(label, xy))
        try:
            action = self.arm.moveto(x=xy[0], y=xy[1])
            action.wait_for_completed(timeout=ARM_TIMEOUT_S)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] ขยับแขนไป{0} ไม่สำเร็จ: {1}".format(label, exc))
            return False
        time.sleep(ARM_SETTLE_S)
        return True

    def _grip(self, fn, label, power, wait_s):
        """สั่งกริปเปอร์แล้วหน่วงเวลารอให้นิ้วขยับจนสุด

        Args:
            fn: ``self.gripper.open`` หรือ ``self.gripper.close``
            label: คำกริยาที่เอาไว้พิมพ์ log
            power: แรงบีบ 1-100
            wait_s: เวลาที่หน่วงรอหลังส่งคำสั่ง

        Returns:
            bool: True เมื่อส่งคำสั่งออกไปได้
        """
        try:
            fn(power=power)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] สั่งกริปเปอร์{0}ไม่สำเร็จ: {1}".format(label, exc))
            return False
        time.sleep(wait_s)
        return True

    # ---------- ลำดับงาน ----------
    def _confirm_grip(self):
        """ตรวจจากสถานะว่าหุบแล้วคีบวัตถุติดจริงไหม

        gripper.py:36 นิยามไว้ว่า closed = นิ้วหุบจนสุด ซึ่งแปลว่าไม่มีอะไรขวาง
        อยู่ระหว่างนิ้ว ส่วน normal = หุบค้างกลางทาง = มีวัตถุคาอยู่

        Returns:
            bool: True เมื่อเชื่อว่าคีบติด
        """
        status = self.status()
        if status == "":
            print("[WARN] ไม่มีสถานะกริปเปอร์ให้ตรวจ ถือว่าคีบติดไว้ก่อน")
            return True
        if status == "normal":
            print("[ARM] ยืนยันคีบวัตถุติด (นิ้วหุบค้างกลางทาง)")
            return True
        if status == "closed":
            print("[WARN] นิ้วหุบจนสุด = ไม่มีวัตถุคาอยู่ระหว่างนิ้ว")
        else:
            print("[WARN] นิ้วยังกางอยู่ คำสั่งหุบไม่มีผล")
        return False

    def _release(self, reason):
        """กางกริปเปอร์ปล่อยวัตถุ แล้วยืนยันจากสถานะว่าไม่ได้คีบค้างอยู่

        Args:
            reason: ข้อความบอกเหตุผลที่ปล่อย เอาไว้พิมพ์ log

        Returns:
            bool: True เมื่อเชื่อว่าปล่อยออกไปแล้ว
        """
        print("[ARM] กางกริปเปอร์{0}".format(reason))
        for attempt in range(1, GRIPPER_RELEASE_TRIES + 1):
            self._grip(self.gripper.open, "กาง", GRIPPER_POWER, GRIPPER_ACT_S)
            status = self.status()
            if status != "closed" and status != "normal":
                # "opened" = ยืนยันว่านิ้วกางสุด ส่วน "" = ไม่มีข้อมูลตรวจ
                # ซึ่งถือว่าปล่อยแล้ว ไม่งั้นจะวนสั่งกางซ้ำไปเรื่อยตอนจบงาน
                self.holding = False
                self.grip_closed = False
                return True
            print("[WARN] กางครั้งที่ {0}/{1} แล้วสถานะยังเป็น {2}"
                  .format(attempt, GRIPPER_RELEASE_TRIES, status))

        print("[WARN] ปล่อยวัตถุไม่ออก อาจเพราะนิ้วกางไม่สุดเนื่องจากติดพื้น")
        print("       ลองยก y ของท่าวางขึ้นสัก 20-30 mm ให้นิ้วกางได้อิสระ")
        return False

    def pick_up(self, reach_xy=None):
        """พาแขนเข้าจุดอ้างอิง กางกริปเปอร์รับวัตถุ แล้วหุบคีบเก็บเข้าท่าวิ่ง

        Args:
            reach_xy: ท่าแขนที่ยื่นลงไปหยิบของก่อนหุบ ปกติส่ง ARM_PICK_XY มา
                ถ้าเป็น None จะไม่ยื่นแขนไปไหน แต่กางนิ้วค้างไว้
                PAYLOAD_LOAD_S วินาทีให้คนวางของใส่มือแทน

        Returns:
            bool: True เมื่อเชื่อว่าคีบวัตถุติดจริง
        """
        # moveto() เป็นพิกัดสัมบูรณ์ ถ้าไม่พาแขนกลับจุดอ้างอิงก่อน ท่า CARRY และ
        # PLACE จะเพี้ยนไปตามตำแหน่งที่แขนค้างอยู่ตอนเปิดเครื่อง
        self._arm_recenter()

        # กางกริปเปอร์ก่อนเสมอ ถ้าตอนบูตนิ้วหุบอยู่แล้ว close() ข้างล่างจะไม่มี
        # ผลอะไรเลย หุ่นจะวิ่งออกไปทั้งที่ไม่ได้คีบอะไรมา และต้องกางก่อนยื่นแขน
        # ลงไปด้วย ให้นิ้วครอบวัตถุลงไป ไม่ใช่เอานิ้วที่หุบอยู่ไปเขี่ยมันล้ม
        if reach_xy is None:
            print("[ARM] กางกริปเปอร์ รอวางวัตถุ {0:.1f} วินาที"
                  .format(PAYLOAD_LOAD_S))
            open_wait = GRIPPER_ACT_S + PAYLOAD_LOAD_S
        else:
            print("[ARM] กางกริปเปอร์ก่อนยื่นแขนลงไปหยิบ")
            open_wait = GRIPPER_ACT_S
        self._grip(self.gripper.open, "กาง", GRIPPER_POWER, open_wait)

        if reach_xy is not None:
            self._arm_moveto(reach_xy, "ท่าหยิบของ")

        print("[ARM] หุบกริปเปอร์คีบวัตถุ")
        # ตั้ง grip_closed จาก "คำสั่งออกไปได้ไหม" ไม่ใช่จากสถานะที่อ่านกลับมา
        # เพราะสถานะคือสิ่งที่เชื่อไม่ได้ตั้งแต่แรก ส่วนคำสั่งที่ throw ไปเลย
        # แปลว่านิ้วไม่ได้บีบอะไรไว้จริง ๆ
        self.grip_closed = self._grip(self.gripper.close, "หุบ",
                                      GRIPPER_POWER, GRIPPER_ACT_S)
        self.holding = self._confirm_grip()

        self._arm_moveto(ARM_CARRY_XY, "ท่าวิ่ง")
        return self.holding

    def _lower_and_release(self, pose, move_label, release_reason):
        """ก้มแขนลงไปที่ pose ปล่อยวัตถุ แล้วเก็บแขนกลับท่าวิ่ง

        Args:
            pose: ท่าแขนตอนปล่อยของ (x, y) หน่วย mm
            move_label: ชื่อท่าที่เอาไว้พิมพ์ log
            release_reason: เหตุผลที่ปล่อย เอาไว้พิมพ์ log

        Returns:
            bool: True เมื่อเชื่อว่าวางวัตถุลงแล้ว
        """
        self._arm_moveto(pose, move_label)
        time.sleep(PLACE_SETTLE_S)  # รอให้แขนนิ่งก่อนปล่อย กันวัตถุล้ม
        released = self._release(release_reason)

        if not released:
            # ห้ามหุบนิ้วตรงนี้เด็ดขาด ถ้าวัตถุยังคาอยู่จริง การหุบคือการคีบมัน
            # กลับขึ้นมาใหม่ ซึ่งเท่ากับไม่ได้วางอะไรลงเลย ปล่อยแขนค้างต่ำและ
            # กางนิ้วไว้แบบนั้น ให้คนหยิบวัตถุออกได้ง่ายที่สุด
            print("[ARM] คาแขนไว้ที่ท่าวางและกางนิ้วค้างไว้ ให้เอาวัตถุออกเอง")
            return False

        self._arm_moveto(ARM_CARRY_XY, "ท่าวิ่ง")
        # หุบกริปเปอร์เบา ๆ ไว้ กันนิ้วกางไปเกี่ยวกำแพงตอนถอยออก
        self._grip(self.gripper.close, "หุบ", GRIPPER_TUCK_POWER,
                   GRIPPER_TUCK_S)
        return True

    def place(self):
        """ยื่นแขนออกไปวางวัตถุที่ช่องเป้าหมาย แล้วเก็บแขนกลับ

        Returns:
            bool: True เมื่อเชื่อว่าวางวัตถุลงแล้ว
        """
        return self._lower_and_release(ARM_PLACE_XY, "จุดวางของ", "วางวัตถุ")

    def put_down_if_holding(self):
        """ก้มวางวัตถุลงพื้นเมื่อจบงานแบบไม่ถึงเป้าหมาย

        ครอบคลุมทุกทางที่จบโดยไม่ได้เรียก place() คือเซนเซอร์ขาดการอัปเดต
        ไปเป้าหมายไม่ได้แล้ว ช่องถูกล้อมทุกด้าน เดินครบ MAX_STEPS และ Ctrl-C

        ก้มลงวางแทนที่จะกางนิ้วปล่อยเฉย ๆ เพราะปล่อยจากท่าวิ่งของจะตกกระแทกพื้น
        แล้วล้มกลิ้ง ส่วนการปล่อยให้หุบคาวัตถุไว้ก็ทำให้มอเตอร์กริปเปอร์บีบค้าง
        ไปจนกว่าจะปิดเครื่อง

        เกณฑ์คือ ``holding`` หรือ ``grip_closed`` ไม่ใช่ ``holding`` อย่างเดียว
        เพราะความผิดพลาดสองฝั่งราคาไม่เท่ากัน ``holding`` ที่ผิดว่ามือเปล่าแล้ว
        ไม่ปล่อย คือของค้างในมือและมอเตอร์บีบค้างจนกว่าจะปิดเครื่อง ส่วนการปล่อย
        เผื่อทั้งที่มือเปล่า แค่เสียเวลาก้มแขนหนึ่งครั้งตอนจบงาน

        Returns:
            bool: True เมื่อวางลงแล้ว หรือไม่ได้สั่งหุบอะไรไว้แต่แรก
        """
        if not self.holding and not self.grip_closed:
            return True
        if self.holding:
            print("[ARM] ยังคีบวัตถุอยู่ ก้มวางลงพื้นก่อนจบงาน")
        else:
            print("[ARM] สถานะบอกว่ามือเปล่า แต่สั่งหุบค้างไว้ตั้งแต่ตอนหยิบ")
            print("      ก้มลงกางนิ้วเผื่อไว้ เผื่อสถานะอ่านผิดแล้วของยังคาอยู่")
        return self._lower_and_release(ARM_DROP_XY, "จุดวางตอนจบงาน",
                                       "วางวัตถุก่อนจบงาน")


def place_on_target(driver, payload, heading, room_behind_m):
    """วางของที่ช่องเป้าหมาย โดยเล็งด้วยกำแพงก่อนถ้าตั้ง AIM_SEQUENCE ไว้

    การเล็งคือการจัดตำแหน่งทีละแกน โดยหันหน้าเข้าหากำแพงของแกนนั้นแล้วใช้ ToF
    ซึ่งเป็นเซนเซอร์ตัวเดียวที่คืนระยะเป็นมิลลิเมตรจริง วิธีนี้ทิ้งความคลาด
    เคลื่อนที่ odometry สะสมมาตลอดทางไปทั้งหมด เพราะกำแพงเป็นจุดอ้างอิงสัมบูรณ์

    ถ้าไม่ได้ตั้ง AIM_SEQUENCE จะกลับไปใช้ทางเดิมคือถอยห่างกำแพงพอให้แขนยื่นได้
    แล้ววางจากตรงที่จอด ซึ่งวางได้แค่ "ในช่อง" ไม่ใช่ "ตรงจุด"

    Args:
        driver (Driver): ตัวควบคุมการเคลื่อนที่
        payload (Payload): แขนกลและกริปเปอร์
        heading (int): ทิศที่หุ่นหันอยู่ตอนถึงช่องเป้าหมาย
        room_behind_m (float): ระยะที่ถอยกลับได้อย่างปลอดภัย ใช้เฉพาะทางเดิม

    Returns:
        int: ทิศที่หุ่นหันอยู่หลังวางเสร็จ
    """
    if not AIM_SEQUENCE:
        if GOAL_WALL_CLEARANCE_MM is not None:
            driver.back_off_from_wall(GOAL_WALL_CLEARANCE_MM, heading,
                                      room_behind_m)
        payload.place()
        return heading

    print("[AIM] เล็งเป้าก่อนวาง {0} ขั้น".format(len(AIM_SEQUENCE)))
    for index, (face, target_mm) in enumerate(AIM_SEQUENCE, 1):
        if face is not None:
            heading = driver.turn_to(heading, face)
        if target_mm is None:
            # ขั้นที่หันอย่างเดียว ใช้ตำแหน่งตามแกนนี้ที่ได้มาจากตอนเดินเข้าช่อง
            # ซึ่ง Sharp ประคองไว้เทียบกำแพงข้าง ไม่ใช่ odometry
            print("[AIM] ขั้นที่ {0}/{1} หัน{2} แล้วไม่จัดระยะ "
                  "(ใช้ตำแหน่งที่ Sharp ประคองไว้ตอนเข้าช่อง)"
                  .format(index, len(AIM_SEQUENCE), DIR_NAMES[heading]))
            continue
        print("[AIM] ขั้นที่ {0}/{1} หัน{2} จัดระยะให้ ToF = {3}mm"
              .format(index, len(AIM_SEQUENCE), DIR_NAMES[heading], target_mm))
        driver.align_to_wall(target_mm, heading, AIM_MAX_MOVE_M)

    # วางต่อแม้จัดระยะไม่ครบ เพราะวางเยื้องเป้ายังดีกว่าไม่ได้วางเลย
    # align_to_wall พิมพ์เตือนไว้แล้วว่าขั้นไหนไม่เข้าเป้า
    payload.place()
    return heading


def face_way_back(driver, heading, entry_heading):
    """หันกลับไปทางด้านที่เพิ่งเดินเข้าช่องเป้าหมายมา ก่อนออกเดินขากลับ

    การหันครั้งเดียวนี้ได้สองอย่างพร้อมกัน

    1. เอา ToF ออกจากของที่เพิ่งวาง ซึ่งตอนนี้กองอยู่ตรงหน้าหุ่นในระยะแขน
       (``ARM_PLACE_XY``) ใกล้กว่าเกณฑ์กำแพงหน้ามาก ถ้าอ่านเซนเซอร์จากท่านั้น
       เลย ``observe()`` จะบันทึกของเป็นกำแพงลงแผนที่ ซึ่งลบออกไม่ได้ (ดู
       ``Maze.set_wall``) แล้วขากลับจะเสียด้านนั้นไปตลอด
    2. ได้ทิศตั้งต้นที่ยืนยันแล้วว่าโล่งจริง เพราะเป็นด้านที่หุ่นเพิ่งเดินผ่าน
       มาเองเมื่อกี้ ไม่ใช่ด้านที่แผนที่แค่ "ยังไม่เคยเห็น" แล้วเดาว่าเปิด

    ข้อควรรู้: ถ้าตั้งให้ขั้นสุดท้ายของ ``AIM_SEQUENCE`` หันไปทางเดียวกับด้านที่
    เดินเข้าห้องมา ของที่วางจะไปกองขวางทางกลับพอดี กรณีนั้นด่าน ToF ก่อนออกตัว
    จะเห็นแล้วมาร์กเป็นกำแพง ทำให้ขากลับต้องหาทางอื่น หรือจบด้วย [FAIL] ถ้าไม่มี
    ทางอื่นให้ไป

    Args:
        driver (Driver): ตัวควบคุมการเคลื่อนที่
        heading (int): ทิศที่หันอยู่หลังวางของเสร็จ
        entry_heading (int or None): ทิศที่เดินเข้าช่องนี้มา None = ไม่เคยเดิน
            เข้ามา (ช่องเป้าหมายคือช่องที่ยืนอยู่ตั้งแต่แรก) จึงไม่มีด้านไหน
            ที่ยืนยันแล้วให้หันไปหา ปล่อยให้ Flood Fill เลือกเองทั้งหมด

    Returns:
        int: ทิศที่หุ่นหันอยู่หลังจบ
    """
    if entry_heading is None:
        return heading
    return driver.turn_to(heading, (entry_heading + 2) % 4)


# =====================================================================
# State machine หลัก - search run ด้วย Flood Fill
# =====================================================================
def run_search(hub, driver, payload, go_home=True):
    """เดินสำรวจด้วย Flood Fill ตามลำดับ หยิบของ -> วางของ -> เดินกลับ

    ทั้งสามเฟสใช้ลูปเดียวกันหมด ต่างกันแค่ ``maze.goals`` ที่ตั้งใหม่ตอนจบแต่ละ
    เฟส แผนที่ไม่ถูกล้างระหว่างเฟส ขากลับจึงเริ่มจากความรู้ทั้งหมดที่สะสมมา

    Args:
        hub (SensorHub): ตัวอ่านเซนเซอร์
        driver (Driver): ตัวควบคุมการเคลื่อนที่
        payload (Payload or None): แขนกลและกริปเปอร์ None = ไม่คีบไม่วาง
        go_home (bool): False = วางของแล้วจบตรงช่องเป้าหมาย ไม่ต้องเดินกลับ
            (มาจาก --no-return) ค่า True ยังเดินกลับก็ต่อเมื่อตั้ง
            ``RETURN_CELL`` ไว้ด้วย

    Returns:
        bool: True เมื่อจบครบทุกเฟสที่ตั้งไว้
    """
    maze = Maze(MAZE_W, MAZE_H, GOAL_CELLS)
    x, y = START_CELL
    heading = START_HEADING

    return_cell = tuple(RETURN_CELL) if go_home and RETURN_CELL is not None \
        else None

    print("=" * 62)
    print("  MAZE SEARCH RUN - Flood Fill")
    print("  สนาม {0}x{1} ช่องละ {2:.2f} m | เริ่มที่ {3} หัน {4} | เป้าหมาย {5}"
          .format(MAZE_W, MAZE_H, CELL_SIZE_M, START_CELL,
                  DIR_NAMES[START_HEADING], GOAL_CELLS))
    if return_cell is not None:
        print("  วางของเสร็จแล้วเดินกลับไปที่ช่อง {0}".format(return_cell))
    print("=" * 62)

    # ตั้งเป้าเฟสแรกไปที่ช่องหยิบของเสมอ ถ้าเป็นช่องเริ่มต้นก็แค่หยิบอยู่กับที่
    # โดยไม่ขยับ ทำแบบนี้เพื่อให้ทุกคอนฟิกเดินผ่านโค้ดชุดเดียวกันหมด
    pick_pending = payload is not None and PICK_CELL is not None
    if pick_pending:
        maze.goals = [tuple(PICK_CELL)]
        if tuple(PICK_CELL) != tuple(START_CELL):
            print("[PLAN] เฟส 1 ไปหยิบของที่ช่อง {0} แล้วค่อยไปเป้าหมาย {1}"
                  .format(tuple(PICK_CELL), GOAL_CELLS))
    elif payload is not None:
        print("[WARN] PICK_CELL เป็น None จะไม่หยิบอะไรเลยแต่ยังสั่งวางตอนจบ")
        print("       ถ้าตั้งใจจะไม่คีบของ ให้ใช้ --no-payload แทน")

    # นับความล้มเหลวซ้ำที่ (ช่อง, ทิศ) เดิม ใช้ตัดวงจรกรณีเดินไม่ผ่านแต่ ToF
    # ก็ไม่เห็นกำแพง (ล้อลื่น ติดขอบ ฯลฯ) ซึ่งถ้าไม่ตัดจะเลือกทิศเดิมซ้ำไปเรื่อย ๆ
    fail_key = None
    fail_count = 0
    # ระยะที่เพิ่งเดินเข้าช่องปัจจุบันมาตามทิศที่หันอยู่ตอนนี้ 0 = ไม่รู้ว่าถอย
    # กลับไปได้แค่ไหน (ยังไม่เคยเดิน หรือหมุนตัวหลังเข้าช่องไปแล้ว)
    entry_travel = 0.0
    # ทิศที่เดินเข้าช่องปัจจุบันมา ต่างจาก entry_travel ตรงที่การหมุนตัวไม่ทำให้
    # ค่านี้ใช้ไม่ได้ เพราะมันบอก "ด้านไหนของช่องที่เปิด" ซึ่งไม่เปลี่ยนตามท่าหุ่น
    entry_heading = None
    # ยังไม่ได้วางของ ใช้แยกว่ารอบที่ถึง maze.goals คือถึงจุดวาง หรือกลับถึงบ้าน
    place_pending = True
    # นับ "ช่องที่เดินผ่านจริง" แยกจากรอบของลูป เพราะรอบที่หยิบของ รอบที่เดินไม่
    # ผ่าน และรอบที่ยกเลิกเพราะ SAFETY ก็กินรอบไปด้วยทั้งที่หุ่นไม่ได้ย้ายช่อง
    moves = 0

    for step in range(MAX_STEPS):
        if (x, y) in maze.goals and pick_pending:
            # ToF เบรกให้หน้าวัตถุที่ FRONT_STOP_MM แล้ว หยิบได้เลยจากตรงนี้
            # หมายเหตุ: ยังไม่มีการ observe() ที่ช่องนี้ เพราะ goal ถูกเช็คก่อน
            # ดังนั้นวัตถุจึงไม่ถูกบันทึกเป็นกำแพงหน้าลงแผนที่ ซึ่งลบออกไม่ได้
            if ARM_PICK_XY is None:
                print("\n[PICK] รับวัตถุที่ช่อง {0}".format((x, y)))
            else:
                print("\n[PICK] ถึงช่องหยิบของ {0} แล้ว เดินมา {1} ช่อง"
                      .format((x, y), moves))
            driver.stop()
            if PICK_HEADING is not None:
                heading = driver.turn_to(heading, PICK_HEADING)
                entry_travel = 0.0      # หันแล้ว ข้างหลังไม่ใช่ทางที่เพิ่งผ่าน
            if not payload.pick_up(reach_xy=ARM_PICK_XY):
                print("[WARN] ไม่ได้คีบวัตถุมาด้วย จะเดินต่อแต่ไม่มีของไปวาง")
            maze.goals = list(GOAL_CELLS)
            pick_pending = False
            print("[PLAN] มุ่งหน้าไปเป้าหมาย {0}".format(maze.goals))
            continue

        if (x, y) in maze.goals and place_pending:
            print("\n[GOAL] ถึงช่องเป้าหมาย {0} แล้ว เดินมา {1} ช่อง"
                  .format((x, y), moves))
            driver.stop()
            place_pending = False
            if payload is not None:
                # entry_travel = ระยะที่เพิ่งเดินเข้าช่องนี้มา ซึ่งเป็นพื้นที่
                # เดียวข้างหลังที่หุ่นเพิ่งผ่านมาเองแล้วว่าโล่งจริง แผนที่บอก
                # ไม่ได้ เพราะ goal ถูกเช็คก่อน observe() ที่ช่องนี้
                heading = place_on_target(driver, payload, heading,
                                          entry_travel)

            # ปล่อยของไม่ออก = _lower_and_release คาแขนไว้ที่ท่าวางและกางนิ้ว
            # ค้างไว้ให้คนมาหยิบออก เดินทั้งท่านั้นคือลากแขนที่ยื่นสุดไปครูด
            # กำแพง จบตรงนี้ดีกว่าเดินกลับ
            arm_stuck_out = payload is not None and payload.holding
            if arm_stuck_out:
                print("[WARN] ปล่อยวัตถุไม่ออก แขนยังยื่นค้างอยู่ที่ท่าวาง "
                      "จึงไม่เดินกลับ ให้เอาวัตถุออกจากมือก่อน")

            if return_cell is not None and not arm_stuck_out:
                maze.goals = [return_cell]
                heading = face_way_back(driver, heading, entry_heading)
                entry_travel = 0.0      # หันแล้ว ข้างหลังไม่ใช่ทางที่เพิ่งผ่าน
                print("[PLAN] วางของแล้ว เดินกลับไปที่ช่อง {0}"
                      .format(return_cell))
                continue

            print(maze.render(robot=(x, y, heading), legend=True))
            return True

        if (x, y) in maze.goals:
            print("\n[HOME] กลับถึงช่อง {0} แล้ว เดินทั้งหมด {1} ช่อง"
                  .format((x, y), moves))
            driver.stop()
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
            # บอกด้วยว่าตันตอนขาไหน เพราะขากลับใช้ลูปเดียวกันและพิมพ์ที่เดียวกัน
            print("\n[FAIL] จากความรู้ปัจจุบัน ไป{0}ไม่ได้แล้ว "
                  "(ทุกทางที่รู้จักถูกกำแพงปิดหมด)"
                  .format("เป้าหมาย" if place_pending else "ช่องที่จะกลับไป"))
            return False

        next_heading = maze.choose_next_heading(x, y, heading, dist)
        if next_heading is None:
            print("\n[FAIL] ช่องนี้ถูกล้อมทุกด้าน ออกไปไหนไม่ได้")
            return False
        print("ตัดสินใจ -> distance ที่นี่ = {0}, เดินไปทาง {1}"
              .format(dist[x][y], DIR_NAMES[next_heading]))

        heading = driver.turn_to(heading, next_heading)
        # หมุนแล้วข้างหลังไม่ใช่ทางที่เพิ่งผ่านมาอีกต่อไป ล้างเพดานการถอยทิ้ง
        # แล้วให้ advance_one_cell ที่สำเร็จเป็นตัวตั้งค่าใหม่
        entry_travel = 0.0

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
            moves += 1
            entry_travel = traveled
            entry_heading = heading
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

    if "SHARP_LEFT" in results and "SHARP_RIGHT" in results:
        pol = {}
        for ref_name in ("SHARP_LEFT", "SHARP_RIGHT"):
            enter, exit_ = results[ref_name][:2]
            pol[ref_name] = "สูงขึ้น" if enter > exit_ else "ต่ำลง"
        same = pol["SHARP_LEFT"] == pol["SHARP_RIGHT"]
        print("  ทิศทางตอบสนอง: เข้าใกล้กำแพงแล้วซ้ายค่า{0} ขวาค่า{1} -> {2}"
              .format(pol["SHARP_LEFT"], pol["SHARP_RIGHT"],
                      "เหมือนกัน" if same else "กลับด้านกัน"))
        if not same:
            problems.append(
                "Sharp ซ้ายกับขวาตอบสนองกลับด้านกัน ตัวประคองกลางช่องรองรับได้ "
                "(แยก polarity รายข้างแล้ว) แต่ปกติแปลว่าใช้คนละรุ่นหรือต่อสลับขั้ว "
                "ควรยืนยันก่อนว่าตั้งใจให้เป็นแบบนี้")

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
# โหมด --armtest : จูนท่าแขนกับวัตถุจริง โดยไม่ต้องรันทั้งเขาวงกต
# =====================================================================
ARMTEST_TOF_SAMPLES = 20        # จำนวนครั้งที่อ่าน ToF ตอนเช็คว่าของบังหรือไม่


def run_arm_test(hub, payload):
    """ยื่นแขนหยิบของที่วางไว้ตรงหน้า แล้วรายงานว่าใช้ค่าปัจจุบันได้จริงไหม

    มีสองอย่างที่ต้องรู้ก่อนใช้ ARM_PICK_XY จริง ทั้งคู่ต้องวัดกับของจริง
    1. ``ARM_PICK_XY`` เอื้อมถึงของและคีบติดหรือเปล่า
    2. ของที่คีบแล้วยกขึ้นท่าวิ่ง ไปบัง ToF หรือเปล่า ซึ่งถ้าบังหุ่นจะเห็นเป็น
       กำแพงหน้าตลอดทางแล้วเดินไม่ได้เลย ของสูง ๆ อย่างขวดน้ำเจอปัญหานี้ง่ายมาก

    Returns:
        bool: True เมื่อคีบติดและของที่คีบไม่บัง ToF
    """
    print("\n" + "=" * 62)
    print("  ARM TEST - จูนท่าแขนกับวัตถุจริง")
    print("  ARM_PICK_XY  = {0}".format(ARM_PICK_XY))
    print("  ARM_CARRY_XY = {0}".format(ARM_CARRY_XY))
    print("  ARM_PLACE_XY = {0}".format(ARM_PLACE_XY))
    print("=" * 62)
    if ARM_PICK_XY is None:
        print("\n[STOP] ARM_PICK_XY เป็น None = ตั้งไว้ให้คนวางของใส่มือ")
        print("       โหมดนี้มีไว้จูนท่ายื่นแขน ตั้ง ARM_PICK_XY เป็นพิกัดก่อน")
        return False

    print("\nวางวัตถุไว้ตรงหน้าหุ่น ให้อยู่กึ่งกลางลำตัว")
    print("และอยู่ในระยะที่แขนเอื้อมถึง")
    print("(แขนยื่นได้ไกลสุด 220 mm วัดจากฐานแขน)")
    input("พร้อมแล้วกด Enter...")

    gripped = payload.pick_up(reach_xy=ARM_PICK_XY)
    if not gripped:
        print("\n[ARMTEST] คีบไม่ติด ลองปรับ ARM_PICK_XY แล้วรันใหม่")
        print("          x น้อยลง = หดเข้าหาตัว, y น้อยลง = ต่ำลงติดพื้น")
        print("          ช่วงที่สั่งได้คือ x 0-220 mm, y 0-150 mm")

    # ของอยู่ในมือที่ท่าวิ่งแล้ว ตรงนี้คือจุดที่ ToF ต้องยังมองทะลุไปข้างหน้าได้
    print("\n[ARMTEST] อ่าน ToF ตอนคีบของค้างไว้ที่ท่าวิ่ง...")
    readings = []
    for _ in range(ARMTEST_TOF_SAMPLES):
        snap = hub.snapshot()
        if snap.tof_mm is not None:
            readings.append(snap.tof_mm)
        time.sleep(0.1)

    threshold = front_wall_threshold_mm()
    blocked = True
    if not readings:
        print("[ARMTEST] ไม่ได้ค่า ToF เลย ตรวจการต่อเซนเซอร์ก่อน")
    else:
        avg = sum(readings) / float(len(readings))
        print("[ARMTEST] ToF เฉลี่ย {0:.0f} mm "
              "(ต่ำสุด {1} สูงสุด {2} จาก {3} ครั้ง)"
              .format(avg, min(readings), max(readings), len(readings)))
        blocked = avg < threshold
        if blocked:
            print("[ARMTEST] ต่ำกว่าเกณฑ์กำแพงหน้า {0} mm = ของบัง ToF อยู่"
                  .format(threshold))
            print("          หุ่นจะเห็นเป็นกำแพงตลอดทางแล้วเดินไม่ได้")
            print("          แก้ที่ ARM_CARRY_XY: ยก y สูงขึ้น (ไม่เกิน 150)")
            print("          หรือหด x ให้เข้าหาตัวมากขึ้น แล้วรันใหม่")
        else:
            print("[ARMTEST] ไม่บัง ToF (เกณฑ์กำแพงหน้าคือ {0} mm)"
                  .format(threshold))

    input("\nกด Enter เพื่อให้หุ่นวางของลง...")
    placed = payload.place()

    ok = gripped and not blocked and placed
    print("\n" + "=" * 62)
    print("  สรุป: คีบติด {0} | ไม่บัง ToF {1} | วางลงได้ {2}"
          .format("ใช่" if gripped else "ไม่", "ใช่" if not blocked else "ไม่",
                  "ใช่" if placed else "ไม่"))
    print("  {0}".format("ค่าปัจจุบันใช้ได้" if ok else "ยังต้องปรับค่าอีก"))
    print("=" * 62)
    return ok


# =====================================================================
# โหมด --sim : ทดสอบตรรกะ Flood Fill โดยไม่ต้องต่อหุ่น
# =====================================================================
#: list: คู่ช่องที่มีกำแพงกั้นระหว่างกันในเขาวงกตจำลอง
#:
#: หุ่นเดินถึงเป้าหมายใน 5 ก้าวตามเส้นทาง (0,0) -> (0,1) -> (1,1) -> (2,1) ->
#: (2,2) -> (2,3) ซึ่งเป็นเส้นทางที่สั้นที่สุดจริงของเขาวงกตนี้ ไม่มีการย้อนกลับ
#:
#: สิ่งที่ทดสอบได้จริง
#:   * การวางแผนใหม่เมื่อความรู้เปลี่ยน - ก้าวที่ 1 ที่ (0,1) ทางตรงที่ Flood Fill
#:     เดาไว้ว่าโล่งกลับมีกำแพง distance ของทิศตรงจึงพุ่งขึ้น ต้องเลี้ยวขวาแทน
#:   * การยอมหมุนเมื่อคุ้ม - ก้าวที่ 3 ที่ (2,1) ทางตรงไปต่อได้ แต่ทางซ้ายมี
#:     distance ต่ำกว่าอย่างเคร่งครัด ตัวเลือกจึงต้องข้ามความชอบเดินตรงไป
#:   * การอ่านกำแพงครบทั้งสามด้านของ observe() - กำแพง 5 ใน 6 เส้นถูกพบจริง
#:     โดยกระจายกันทั้งด้านหน้า ซ้าย และขวา จบด้วยสำรวจไป 29/40 ด้าน
#:
#: สิ่งที่ยังไม่ถูกทดสอบ
#:   เส้นทางนี้ไม่มีทางตันเลย ตรรกะการถอยออกจากทางตันและ recovery ทั้งหมดใน
#:   run_search จึงไม่ถูกแตะ ส่วนกำแพง (3,1)-(3,2) อยู่นอกเส้นทาง หุ่นไม่เคย
#:   เดินไปเห็น เปลี่ยนหรือลบทิ้งได้โดยผลการรันไม่เปลี่ยน
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

    จำลองครบทุกเฟสเหมือน ``run_search`` คือไปหยิบของ ไปวางของ แล้วเดินกลับ
    ส่วนการคีบและการวางจริงไม่มีอะไรให้จำลอง

    Returns:
        bool: True เมื่อเดินครบทุกเฟส
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

    # จำลองการสลับเป้าหมายทุกเฟสด้วย เพื่อให้ตรวจแผนการเดินได้ก่อนลงสนามจริง
    return_cell = tuple(RETURN_CELL) if RETURN_CELL is not None else None
    place_pending = True
    pick_pending = PICK_CELL is not None
    if pick_pending:
        known.goals = [tuple(PICK_CELL)]
        if tuple(PICK_CELL) != tuple(START_CELL):
            print("\n[PLAN] เฟส 1 ไปหยิบของที่ช่อง {0} แล้วค่อยไปเป้าหมาย {1}"
                  .format(tuple(PICK_CELL), GOAL_CELLS))

    for step in range(MAX_STEPS):
        if (x, y) in known.goals and pick_pending:
            print("\n[PICK] หยิบของที่ช่อง {0} หลังเดินมา {1} ช่อง (จำลอง)"
                  .format((x, y), len(path) - 1))
            known.goals = list(GOAL_CELLS)
            pick_pending = False
            print("[PLAN] มุ่งหน้าไปเป้าหมาย {0}".format(known.goals))
            continue

        if (x, y) in known.goals:
            if place_pending:
                print("\n[GOAL] ถึงเป้าหมายใน {0} ช่อง".format(len(path) - 1))
                place_pending = False
                if return_cell is not None:
                    known.goals = [return_cell]
                    print("[PLAN] วางของแล้ว เดินกลับไปที่ช่อง {0}"
                          .format(return_cell))
                    continue
            else:
                print("\n[HOME] กลับถึงช่อง {0} แล้ว เดินทั้งหมด {1} ช่อง"
                      .format((x, y), len(path) - 1))
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
    parser.add_argument("--no-return", action="store_true",
                        help="วางของแล้วจบตรงช่องเป้าหมาย ไม่ต้องเดินกลับ")
    parser.add_argument("--armtest", action="store_true",
                        help="ยื่นแขนหยิบของตรงหน้า ใช้จูน ARM_PICK_XY "
                             "หุ่นจะไม่เดินไปไหน")
    args = parser.parse_args()

    if args.sim:
        return 0 if run_sim() else 1

    if robot is None:
        print("[ERROR] import robomaster ไม่สำเร็จ: {0}".format(ROBOT_IMPORT_ERROR))
        print("        ติดตั้ง SDK ก่อน หรือใช้ --sim เพื่อทดสอบเฉพาะตรรกะ")
        return 1

    # --armtest ใช้แค่ ToF กับแขน ไม่ได้อ่าน Sharp/IR จึงไม่ต้องบังคับคาลิเบรต
    if not args.calib and not args.armtest:
        require_calibration()

    ep_robot = robot.Robot()
    print("กำลังเชื่อมต่อหุ่นแบบ {0} ...".format(args.conn))
    ep_robot.initialize(conn_type=args.conn)

    hub = SensorHub(ep_robot)
    payload = None
    success = False
    try:
        hub.start()
        if args.calib:
            run_calibration(hub)
            success = True
        elif args.armtest:
            payload = Payload(ep_robot.robotic_arm, ep_robot.gripper)
            payload.start()
            success = run_arm_test(hub, payload)
        else:
            # initialize() ตั้ง FREE ให้อยู่แล้ว (robot.py reset) แต่สั่งซ้ำให้ชัดเจน
            # ว่าโค้ดนี้ต้องการให้แชสซีขยับอิสระจากกิมบอล
            ep_robot.set_robot_mode(robot.FREE)
            time.sleep(0.5)

            driver = Driver(ep_robot.chassis, hub)
            driver.calibrate_yaw_sign()
            driver.set_north_reference(START_HEADING)

            if DO_PAYLOAD and not args.no_payload:
                payload = Payload(ep_robot.robotic_arm, ep_robot.gripper)
                payload.start()
            else:
                print("[INFO] ข้ามการคีบและวางวัตถุ")

            success = run_search(hub, driver, payload,
                                 go_home=not args.no_return)
    except KeyboardInterrupt:
        print("\n[STOP] ผู้ใช้สั่งหยุด")
    finally:
        try:
            ep_robot.chassis.drive_speed(x=0, y=0, z=0)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] สั่งหยุดล้อไม่สำเร็จ: {0}".format(exc))
        if payload is not None:
            # จบแบบไม่ถึงเป้าหมาย (เซนเซอร์หลุด / ไปต่อไม่ได้ / Ctrl-C) place()
            # ไม่ได้ถูกเรียก วัตถุจะค้างอยู่ในกริปเปอร์ ต้องก้มวางลงเองตรงนี้
            # ทำหลังสั่งหยุดล้อแล้ว เพื่อให้หุ่นนิ่งก่อนขยับแขน
            payload.put_down_if_holding()
            payload.stop()
        hub.stop()
        try:
            ep_robot.close()
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] ปิดการเชื่อมต่อไม่สำเร็จ: {0}".format(exc))

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())

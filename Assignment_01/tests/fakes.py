# -*- coding: utf-8 -*-
"""ฮาร์ดแวร์ปลอมสำหรับทดสอบ test_code.py โดยไม่ต้องต่อหุ่น

โหลด ``test_code.py`` เป็นโมดูลผ่าน importlib เพราะมันเป็นสคริปต์ที่รันตรง ๆ
ไม่ใช่แพ็กเกจ ทุกอย่างที่อยู่หลัง ``if __name__ == "__main__"`` จึงไม่ถูกรัน

ตัวปลอมในไฟล์นี้เลียนแบบเฉพาะส่วนที่โค้ดจริงเรียกใช้เท่านั้น ไม่ได้จำลองฟิสิกส์
- Fake* = ตอบตามสคริปต์ที่เทสต์กำหนด ใช้ตรวจว่าสั่งอะไรออกไปบ้าง
- Truth* = ตอบตามเขาวงกตความจริง ใช้รัน run_search ทั้งรอบให้จบจริง
"""
import importlib.util
import io
import os
import sys

#: str: พาธของไฟล์ที่กำลังทดสอบ อ้างจากตำแหน่งไฟล์นี้ ย้ายโฟลเดอร์ทั้งชุดได้
TARGET = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      os.pardir, "test_code.py")


def load():
    """โหลด test_code.py ใหม่หนึ่งชุด แล้วย่นเวลารอทั้งหมดให้เทสต์จบเร็ว

    โหลดใหม่ทุกครั้งเพื่อให้แต่ละเทสต์แก้ค่าคงที่ได้โดยไม่กระทบเทสต์อื่น

    Returns:
        module: โมดูล test_code ที่พร้อมใช้
    """
    spec = importlib.util.spec_from_file_location("test_code_under_test",
                                                  TARGET)
    tc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tc)
    tc.GRIPPER_ACT_S = 0.01
    tc.PAYLOAD_LOAD_S = 0.01
    tc.ARMTEST_TOF_SAMPLES = 3
    tc.SETTLE_S = 0.0
    # ย่นเวลาหน่วงของแขนและ control loop ทิ้ง ไม่กระทบสิ่งที่เทสต์ตรวจ เพราะ
    # หุ่นปลอมตอบทันทีอยู่แล้ว และฟิสิกส์ของ Corridor อินทิเกรตด้วย dt ของตัวเอง
    tc.CONTROL_DT = 0.0
    tc.ARM_SETTLE_S = 0.0
    tc.PLACE_SETTLE_S = 0.0
    tc.GRIPPER_TUCK_S = 0.01
    return tc


def quiet(fn, *args, **kw):
    """เรียกฟังก์ชันโดยเก็บ stdout ไว้ตรวจแทนที่จะพ่นออกจอ

    Returns:
        tuple: (ค่าที่ฟังก์ชันคืนมา, ข้อความที่มันพิมพ์)
    """
    buf, saved = io.StringIO(), sys.stdout
    sys.stdout = buf
    try:
        result = fn(*args, **kw)
    finally:
        sys.stdout = saved
    return result, buf.getvalue()


class Checker(object):
    """ตัวนับผลเทสต์แบบง่าย ไม่ต้องพึ่ง pytest ที่โปรเจกต์นี้ไม่ได้ติดตั้ง"""

    def __init__(self):
        self.failed = []
        self.passed = 0

    def section(self, title):
        print("\n--- {0} ---".format(title))

    def check(self, name, condition):
        """บันทึกผลหนึ่งข้อ

        Returns:
            bool: ค่า condition ที่รับมา เผื่อเอาไปตัดสินต่อ
        """
        print("{0} {1}".format("PASS" if condition else "FAIL", name))
        if condition:
            self.passed += 1
        else:
            self.failed.append(name)
        return condition

    def report(self):
        """พิมพ์สรุปแล้วบอกว่าผ่านหมดไหม

        Returns:
            bool: True เมื่อไม่มีข้อไหนตก
        """
        print("\n" + "=" * 58)
        if self.failed:
            print("ตก {0} ข้อ จาก {1} ข้อ".format(
                len(self.failed), self.passed + len(self.failed)))
            for name in self.failed:
                print("  - {0}".format(name))
        else:
            print("ผ่านทั้งหมด {0} ข้อ".format(self.passed))
        print("=" * 58)
        return not self.failed


# =====================================================================
# ตัวปลอมแบบสคริปต์ - ใช้ตรวจว่าโค้ดสั่งอะไรออกไปบ้าง
# =====================================================================
class FakeAction(object):
    """action ของแขนที่จบทันที"""

    def wait_for_completed(self, timeout=None):
        return True


class FakeArm(object):
    """แขนปลอม เก็บลำดับคำสั่งไว้ใน ``calls``

    Args:
        fail (bool): True = ทุกคำสั่งโยน exception ใช้ทดสอบว่าโค้ดไม่ล้มตาม
    """

    def __init__(self, fail=False):
        self.calls = []
        self.fail = fail

    def recenter(self):
        self.calls.append("recenter")
        if self.fail:
            raise RuntimeError("arm busy")
        return FakeAction()

    def moveto(self, x=0, y=0):
        self.calls.append((x, y))
        if self.fail:
            raise RuntimeError("arm busy")
        return FakeAction()


class FakeGripper(object):
    """กริปเปอร์ปลอม เก็บคำสั่งไว้ใน ``calls`` และ push สถานะกลับตามสคริปต์

    Args:
        after (dict): สถานะที่จะ push หลังคำสั่ง เช่น {"close": "normal"}
        fail (bool): True = open/close โยน exception
        sub_fail (bool): True = sub_status โยน exception
    """

    def __init__(self, after=None, fail=False, sub_fail=False):
        self.calls = []
        self.after = after or {}
        self.fail = fail
        self.sub_fail = sub_fail
        self.subscribed = False
        self._callback = None

    def sub_status(self, freq=5, callback=None):
        if self.sub_fail:
            raise RuntimeError("dds busy")
        self._callback = callback
        self.subscribed = True
        return True

    def unsub_status(self):
        self.subscribed = False
        return True

    def _push(self, key):
        if self._callback is not None and key in self.after:
            self._callback(self.after[key])

    def open(self, power=50):
        self.calls.append(("open", power))
        if self.fail:
            raise RuntimeError("gripper offline")
        self._push("open")

    def close(self, power=50):
        self.calls.append(("close", power))
        if self.fail:
            raise RuntimeError("gripper offline")
        self._push("close")


class FakeHub(object):
    """SensorHub ปลอมที่คืนค่า ToF คงที่ ใช้ทดสอบ --armtest

    Args:
        tof_mm (int or None): ค่าที่ให้ snapshot คืนทุกครั้ง
    """

    def __init__(self, tof_mm=None):
        self.tof_mm = tof_mm

    def snapshot(self):
        snap = _Snapshot()
        snap.tof_mm = self.tof_mm
        return snap


class _Snapshot(object):
    """snapshot เปล่า ๆ ที่เทสต์เติมฟิลด์เอง"""

    fresh = True
    stale_reason = ""
    tof_mm = None
    adc_left = 0
    adc_right = 0
    ir_left = 0
    ir_right = 0
    yaw = 0.0
    pos_x = 0.0
    pos_y = 0.0

    def front_wall(self):
        return False


class Corridor(object):
    """ทางตรงแกนเดียว ทำหน้าที่เป็นทั้งแชสซีและ SensorHub ให้ ``Driver`` ตัวจริง

    ``TruthWorld`` จำลองแค่ระดับ "ช่อง" ซึ่งหยาบเกินจะทดสอบการถอยทีละไม่กี่
    เซนติเมตรได้ คลาสนี้จึงจำลองละเอียดกว่า คือตำแหน่งตามแนวเดินเป็นเมตรและ
    ระยะ ToF ที่ขยับตามกันจริง ๆ เมื่อมีคำสั่ง drive_speed เข้ามา

    Args:
        tof_mm (int or None): ระยะถึงกำแพงหน้าตอนเริ่ม None = ไม่เห็นกำแพง
        adc (tuple): (ซ้าย, ขวา) ค่า Sharp ดิบที่ให้ snapshot คืนทุกครั้ง
        stale (bool): True = ทำเป็นว่าสตรีมเซนเซอร์ขาดการอัปเดต
        dt (float): เวลาที่ถือว่าผ่านไปต่อหนึ่งคำสั่ง ใช้อินทิเกรตตำแหน่ง
    """

    def __init__(self, tof_mm, adc=(0, 0), stale=False, dt=0.04):
        self.start_mm = tof_mm
        self.adc = adc
        self.stale = stale
        self.dt = dt
        self.pos = 0.0
        #: int or None: เกณฑ์ ToF ที่ถือว่ามีกำแพงหน้า make_driver เป็นคนเติมให้
        #: จากคอนฟิกของโมดูลที่กำลังทดสอบ None = front_wall() คืน False เสมอ
        #: (พฤติกรรมเดิมของ _Snapshot สำหรับ Corridor ที่ไม่ได้ผูกกับ Driver)
        self.wall_mm = None
        #: list: ทุก (x, y, z) ที่ถูกสั่งออกไป รวมคำสั่งหยุดด้วย
        self.commands = []

    @property
    def tof_mm(self):
        """int or None: ระยะถึงกำแพงหน้า คำนวณจากตำแหน่งปัจจุบันทุกครั้ง

        คำนวณสดจาก ``pos`` แทนการบวกลบสะสม เพื่อไม่ให้การปัดเศษเป็นจำนวนเต็ม
        ทำให้ระยะที่รายงานค่อย ๆ เพี้ยนออกจากระยะทางที่ขยับจริง
        """
        if self.start_mm is None:
            return None
        return int(round(self.start_mm - self.pos * 1000))

    # ---------- ฝั่งแชสซี ----------
    def drive_speed(self, x=0.0, y=0.0, z=0.0, timeout=None):
        # เดินหน้า = เข้าใกล้กำแพง ระยะถึงกำแพงหน้าจึงลดลงตามที่ขยับ
        self.commands.append((x, y, z))
        self.pos += x * self.dt

    # ---------- ฝั่ง SensorHub ----------
    def snapshot(self):
        snap = _Snapshot()
        snap.tof_mm = self.tof_mm
        snap.pos_x = self.pos
        snap.adc_left, snap.adc_right = self.adc
        snap.fresh = not self.stale
        snap.stale_reason = "tof" if self.stale else ""
        # front_wall() ต้องตัดสินจากระยะจริง ไม่ใช่ False ตายตัวแบบ _Snapshot
        # ไม่งั้นทางที่มีแต่ front_wall() เป็นประตู - เช่น align_front ที่ turn_to
        # เรียกก่อนหมุน - จะไม่ถูกรันในเทสต์ไหนเลย
        wall_mm = self.wall_mm
        tof_mm = snap.tof_mm
        snap.front_wall = lambda: (wall_mm is not None and tof_mm is not None
                                   and tof_mm < wall_mm)
        return snap

    @property
    def moves(self):
        """list: เฉพาะคำสั่งที่สั่งให้ขยับจริง ตัดคำสั่งหยุดออก"""
        return [c for c in self.commands if c != (0.0, 0.0, 0.0)]


def make_driver(tc, corridor):
    """Driver ตัวจริงที่ต่อกับ Corridor ทั้งฝั่งแชสซีและฝั่งเซนเซอร์

    Returns:
        Driver: ตั้งศูนย์ไว้ที่ทิศเหนือแล้ว พร้อมเรียกเมธอดเคลื่อนที่ได้เลย
    """
    corridor.wall_mm = tc.front_wall_threshold_mm()
    driver = tc.Driver(corridor, corridor)
    driver.yaw_sign = 1
    driver.yaw_zero = 0.0
    return driver


def make_payload(tc, after=None, arm_fail=False, **kw):
    """สร้าง Payload พร้อมแขนและกริปเปอร์ปลอม แล้ว start() ให้เลย

    Returns:
        tuple: (payload, arm, gripper)
    """
    arm = FakeArm(fail=arm_fail)
    gripper = FakeGripper(after, **kw)
    payload = tc.Payload(arm, gripper)
    payload.start()
    return payload, arm, gripper


# =====================================================================
# ตัวปลอมแบบเดินบนเขาวงกตจริง - ใช้รัน run_search ให้จบทั้งรอบ
# =====================================================================
class TruthWorld(object):
    """เขาวงกตความจริงพร้อมตำแหน่งหุ่น ใช้ร่วมกันระหว่าง driver กับ hub

    Args:
        tc: โมดูล test_code ที่โหลดมา
    """

    def __init__(self, tc):
        self.tc = tc
        self.maze = tc.Maze(tc.MAZE_W, tc.MAZE_H, tc.GOAL_CELLS)
        for cell_a, cell_b in tc.SIM_BLOCKED_EDGES:
            self.maze.set_wall(cell_a[0], cell_a[1],
                               tc._edge_direction(cell_a, cell_b), True)
        self.xy = tuple(tc.START_CELL)
        self.heading = tc.START_HEADING
        #: list: ทุกช่องที่หุ่นเดินผ่าน เรียงตามลำดับ เริ่มด้วยช่องเริ่มต้น
        #: ใช้ตรวจ "เส้นทางที่เดินจริง" ซึ่งเป็นสิ่งเดียวที่บอกได้ว่าขากลับ
        #: เลี่ยงของที่วางไว้จริงไหม แผนที่ภายในของ run_search มองจากข้างนอกไม่ได้
        self.path = [self.xy]

    def walls_here(self):
        """กำแพงหน้า/ซ้าย/ขวา ที่ตำแหน่งและทิศปัจจุบัน

        Returns:
            tuple: (front, left, right)
        """
        x, y = self.xy
        head = self.heading
        return (self.maze.has_wall(x, y, head),
                self.maze.has_wall(x, y, (head + 3) % 4),
                self.maze.has_wall(x, y, (head + 1) % 4))


class TruthDriver(object):
    """แชสซีปลอมที่เดินบนเขาวงกตความจริง ทะลุกำแพงไม่ได้

    Args:
        world (TruthWorld): โลกที่ใช้ร่วมกับ TruthHub
    """

    def __init__(self, world):
        self.world = world
        self.turns = []
        #: list: อาร์กิวเมนต์ของทุกครั้งที่ run_search สั่งถอยห่างกำแพง
        self.backoffs = []
        #: list: อาร์กิวเมนต์ของทุกครั้งที่ถูกสั่งจัดระยะเทียบกำแพง
        self.aligns = []

    def stop(self):
        pass

    def back_off_from_wall(self, clearance_mm, heading, limit_m):
        """บันทึกว่าถูกสั่งถอย โดยไม่ขยับโลกจำลอง

        TruthWorld รู้แค่ว่าหุ่นอยู่ช่องไหน ไม่มีตำแหน่งย่อยในช่องให้ขยับ
        การถอยจริงถูกทดสอบแยกด้วย Corridor กับ Driver ตัวจริง

        Returns:
            tuple: (tof_mm, moved_m, reason) แบบเดียวกับ Driver ตัวจริง
        """
        self.backoffs.append((clearance_mm, heading, limit_m))
        print("[BACKOFF] (จำลอง) ต้องการ {0}mm เพดาน {1:.3f} m"
              .format(clearance_mm, limit_m))
        return clearance_mm, 0.0, "clear"

    def align_to_wall(self, target_mm, heading, budget_m, floor_mm=None):
        """บันทึกว่าถูกสั่งจัดระยะ โดยไม่ขยับโลกจำลอง

        เหตุผลเดียวกับ back_off_from_wall คือ TruthWorld หยาบระดับช่อง
        การจัดระยะจริงถูกทดสอบแยกด้วย Corridor กับ Driver ตัวจริง

        Returns:
            tuple: (tof_mm, moved_m, reason) แบบเดียวกับ Driver ตัวจริง
        """
        self.aligns.append((target_mm, heading, budget_m))
        print("[ALIGN] (จำลอง) เป้า {0}mm หัน {1}"
              .format(target_mm, self.world.tc.DIR_NAMES[heading]))
        return target_mm, 0.0, "stop"

    def turn_to(self, current_heading, target_heading, align_first=True):
        if current_heading != target_heading:
            self.turns.append((current_heading, target_heading))
        self.world.heading = target_heading
        return target_heading

    def advance_one_cell(self, heading):
        """เดินหนึ่งช่องถ้าไม่มีกำแพงขวาง

        Returns:
            tuple: (ok, traveled, reason) แบบเดียวกับ Driver ตัวจริง
        """
        tc = self.world.tc
        self.world.heading = heading
        x, y = self.world.xy
        if self.world.maze.has_wall(x, y, heading):
            return False, 0.0, "wall"
        self.world.xy = (x + tc.DX[heading], y + tc.DY[heading])
        self.world.path.append(self.world.xy)
        return True, tc.CELL_SIZE_M, "ok"

    def backup(self, distance_m, heading):
        pass


class TruthHub(object):
    """SensorHub ปลอมที่ตอบตามเขาวงกตความจริง ณ ตำแหน่งปัจจุบัน

    Args:
        world (TruthWorld): โลกที่ใช้ร่วมกับ TruthDriver
    """

    def __init__(self, world):
        self.world = world

    def read_walls_settled(self, samples=None):
        return self.world.walls_here()

    def snapshot(self):
        front = self.world.walls_here()[0]
        snap = _Snapshot()
        snap.tof_mm = 70 if front else 900
        snap.front_wall = lambda: front
        return snap


def make_world(tc, pick_cell, arm_pick_xy, gripper_status="normal"):
    """ตั้งคอนฟิกการหยิบของ แล้วสร้างหุ่นปลอมครบชุดสำหรับ run_search

    Args:
        tc: โมดูล test_code
        pick_cell: ค่าที่จะใส่ให้ PICK_CELL
        arm_pick_xy: ค่าที่จะใส่ให้ ARM_PICK_XY
        gripper_status (str): สถานะที่กริปเปอร์จะ push หลังสั่งหุบ

    Returns:
        tuple: (hub, driver, payload, arm, gripper)
    """
    tc.PICK_CELL = pick_cell
    tc.ARM_PICK_XY = arm_pick_xy
    world = TruthWorld(tc)
    arm = FakeArm()
    gripper = FakeGripper({"open": "opened", "close": gripper_status})
    payload = tc.Payload(arm, gripper)
    payload.start()
    return TruthHub(world), TruthDriver(world), payload, arm, gripper

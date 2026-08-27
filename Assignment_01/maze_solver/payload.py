# -*- coding: utf-8 -*-
"""การคีบและวางวัตถุ รวมถึงการเล็งเป้าด้วยกำแพงก่อนปล่อยมือ"""
import threading
import time

from . import config
from .directions import DIR_NAMES
from .geometry import aim_tof_target_mm
from .readings import aim_reading_is_sane


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
            self.gripper.sub_status(freq=config.GRIPPER_STATUS_FREQ,
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
            if time.time() - self._status_t > config.GRIPPER_STATUS_STALE_S:
                return ""
            return self._status

    # ---------- คำสั่งฮาร์ดแวร์ (กลืน exception ไม่ให้ล้มทั้งรอบ) ----------
    def _arm_recenter(self):
        """พาแขนกลับจุดอ้างอิงก่อนใช้พิกัดสัมบูรณ์ (robotic_arm.py:105)"""
        print("[ARM] พาแขนกลับจุดอ้างอิง")
        try:
            self.arm.recenter().wait_for_completed(timeout=config.ARM_TIMEOUT_S)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] พาแขนกลับจุดอ้างอิงไม่สำเร็จ: {0}".format(exc))
            print("[WARN] ตำแหน่งแขนที่สั่งต่อจากนี้อาจเพี้ยน")
            return False
        time.sleep(config.ARM_SETTLE_S)
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
            action.wait_for_completed(timeout=config.ARM_TIMEOUT_S)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] ขยับแขนไป{0} ไม่สำเร็จ: {1}".format(label, exc))
            return False
        time.sleep(config.ARM_SETTLE_S)
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
        for attempt in range(1, config.GRIPPER_RELEASE_TRIES + 1):
            self._grip(self.gripper.open, "กาง", config.GRIPPER_POWER, config.GRIPPER_ACT_S)
            status = self.status()
            if status != "closed" and status != "normal":
                # "opened" = ยืนยันว่านิ้วกางสุด ส่วน "" = ไม่มีข้อมูลตรวจ
                # ซึ่งถือว่าปล่อยแล้ว ไม่งั้นจะวนสั่งกางซ้ำไปเรื่อยตอนจบงาน
                self.holding = False
                self.grip_closed = False
                return True
            print("[WARN] กางครั้งที่ {0}/{1} แล้วสถานะยังเป็น {2}"
                  .format(attempt, config.GRIPPER_RELEASE_TRIES, status))

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
                  .format(config.PAYLOAD_LOAD_S))
            open_wait = config.GRIPPER_ACT_S + config.PAYLOAD_LOAD_S
        else:
            print("[ARM] กางกริปเปอร์ก่อนยื่นแขนลงไปหยิบ")
            open_wait = config.GRIPPER_ACT_S
        self._grip(self.gripper.open, "กาง", config.GRIPPER_POWER, open_wait)

        if reach_xy is not None:
            self._arm_moveto(reach_xy, "ท่าหยิบของ")

        print("[ARM] หุบกริปเปอร์คีบวัตถุ")
        # ตั้ง grip_closed จาก "คำสั่งออกไปได้ไหม" ไม่ใช่จากสถานะที่อ่านกลับมา
        # เพราะสถานะคือสิ่งที่เชื่อไม่ได้ตั้งแต่แรก ส่วนคำสั่งที่ throw ไปเลย
        # แปลว่านิ้วไม่ได้บีบอะไรไว้จริง ๆ
        self.grip_closed = self._grip(self.gripper.close, "หุบ",
                                      config.GRIPPER_POWER, config.GRIPPER_ACT_S)
        self.holding = self._confirm_grip()

        self._arm_moveto(config.ARM_CARRY_XY, "ท่าวิ่ง")
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
        time.sleep(config.PLACE_SETTLE_S)  # รอให้แขนนิ่งก่อนปล่อย กันวัตถุล้ม
        released = self._release(release_reason)

        if not released:
            # ห้ามหุบนิ้วตรงนี้เด็ดขาด ถ้าวัตถุยังคาอยู่จริง การหุบคือการคีบมัน
            # กลับขึ้นมาใหม่ ซึ่งเท่ากับไม่ได้วางอะไรลงเลย ปล่อยแขนค้างต่ำและ
            # กางนิ้วไว้แบบนั้น ให้คนหยิบวัตถุออกได้ง่ายที่สุด
            print("[ARM] คาแขนไว้ที่ท่าวางและกางนิ้วค้างไว้ ให้เอาวัตถุออกเอง")
            return False

        self._arm_moveto(config.ARM_CARRY_XY, "ท่าวิ่ง")
        # หุบกริปเปอร์เบา ๆ ไว้ กันนิ้วกางไปเกี่ยวกำแพงตอนถอยออก
        self._grip(self.gripper.close, "หุบ", config.GRIPPER_TUCK_POWER,
                   config.GRIPPER_TUCK_S)
        return True

    def place(self):
        """ยื่นแขนออกไปวางวัตถุที่ช่องเป้าหมาย แล้วเก็บแขนกลับ

        Returns:
            bool: True เมื่อเชื่อว่าวางวัตถุลงแล้ว
        """
        return self._lower_and_release(config.ARM_PLACE_XY, "จุดวางของ", "วางวัตถุ")

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
        return self._lower_and_release(config.ARM_DROP_XY, "จุดวางตอนจบงาน",
                                       "วางวัตถุก่อนจบงาน")


def place_on_target(driver, payload, heading, room_behind_m,
                    hub=None, maze=None, cell=None):
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
        hub (SensorHub or None): ใช้วัด ToF ตอนจอดนิ่งเพื่อตรวจก่อนขยับ
            None = ข้ามการตรวจ ปล่อยให้ ``align_to_wall`` วัดเอง
        maze (Maze or None): แผนที่ที่สะสมมา ใช้ทำนายระยะเทียบกับที่วัดได้จริง
            None = ไม่มีด่านที่สอง
        cell (tuple or None): ช่องที่หุ่นยืนอยู่ (x, y) ต้องมีคู่กับ ``maze``

    Returns:
        int: ทิศที่หุ่นหันอยู่หลังวางเสร็จ
    """
    if not config.AIM_SEQUENCE:
        if config.GOAL_WALL_CLEARANCE_MM is not None:
            driver.back_off_from_wall(config.GOAL_WALL_CLEARANCE_MM, heading,
                                      room_behind_m)
        payload.place()
        return heading

    total = len(config.AIM_SEQUENCE)
    print("[AIM] เล็งเป้าก่อนวาง {0} ขั้น".format(total))
    for index, step in enumerate(config.AIM_SEQUENCE, 1):
        if step.face is not None:
            # align_first=False: ห้ามให้การหมุนแตะล้อตามแนวเดิน ไม่งั้นตำแหน่งที่
            # ขั้นก่อนหน้าเพิ่งจัดไว้จะถูก align_front ดันกลับไปที่ FRONT_STOP_MM
            # เงียบ ๆ ทุกครั้งที่ ToF เผอิญอ่านได้ต่ำกว่าเกณฑ์กำแพงหน้า
            heading = driver.turn_to(heading, step.face, align_first=False)

        # ขั้นสุดท้ายคือขั้นที่ยื่นแขนวางจริง จึงเป็นขั้นเดียวที่ระยะเอื้อมของแขน
        # เข้าสมการ ขั้นอื่นแขนยื่นตั้งฉากกับแกนที่กำลังจัด
        target_mm = aim_tof_target_mm(step, heading, index == total)
        if target_mm is None:
            # ขั้นที่หันอย่างเดียว ใช้ตำแหน่งตามแกนนี้ที่ได้มาจากตอนเดินเข้าช่อง
            # ซึ่ง Sharp ประคองไว้เทียบกำแพงข้าง ไม่ใช่ odometry
            print("[AIM] ขั้นที่ {0}/{1} หัน{2} แล้วไม่จัดระยะ "
                  "(ใช้ตำแหน่งที่ Sharp ประคองไว้ตอนเข้าช่อง)"
                  .format(index, total, DIR_NAMES[heading]))
            continue

        print("[AIM] ขั้นที่ {0}/{1} หัน{2} จัดระยะให้ ToF = {3:.0f}mm "
              "(ของต้องห่างกำแพง{4} {5:.0f}mm)"
              .format(index, total, DIR_NAMES[heading], target_mm,
                      DIR_NAMES[step.ref], float(step.target_mm)))

        # ตรวจค่าที่วัดได้ก่อนขยับตาม เพราะการเชื่อ ToF ที่มองทะลุประตูไปเจอ
        # กำแพงห้องถัดไป แปลว่าหุ่นจะขยับเต็มงบไปผิดทางอย่างมั่นใจ
        measured = None
        if hub is not None:
            measured = hub.read_tof_settled()
        # measured เป็น None = ToF ไม่เห็นอะไรในระยะวัด ปล่อยให้ align_to_wall
        # จัดการ (มันคืน no_wall โดยไม่ขยับ) ด่านนี้ตัดสินเฉพาะค่าที่วัดได้จริง
        if measured is not None:
            predicted = None
            if maze is not None and cell is not None:
                predicted = maze.predict_tof(cell[0], cell[1], heading)
            sane, why = aim_reading_is_sane(measured, target_mm, predicted)
            if not sane:
                print("[WARN] ขั้นที่ {0}/{1} ไม่ผ่านด่านตรวจ: {2}"
                      .format(index, total, why))
                print("       ข้ามการจัดระยะขั้นนี้ ใช้ตำแหน่งที่ได้มาตอนเข้าช่อง "
                      "(ขยับตามค่าที่เชื่อไม่ได้ อันตรายกว่าวางเยื้อง)")
                continue
            if predicted is not None:
                print("[AIM] ด่านตรวจผ่าน วัดได้ {0:.0f}mm แผนที่ว่า {1:.0f}mm"
                      .format(measured, predicted))
            else:
                print("[AIM] ด่านตรวจผ่าน วัดได้ {0:.0f}mm "
                      "(แผนที่ยังไม่รู้ระยะด้านนี้ ตรวจได้แค่เทียบกับเป้า)"
                      .format(measured))
        # center: เฉพาะขั้นแรกเท่านั้นที่ยอมให้ Sharp ประคองกลางช่องระหว่างขยับ
        # ตอนนั้นแกนตั้งฉากยังไม่ถูกจัด การถูกดึงเข้ากลางช่องจึงเป็นผลดี เพราะแก้
        # การเยื้องที่ yaw error สะสมไว้ระหว่างขยับให้ด้วย พอขั้นที่ 2 เป็นต้นไป
        # แกนตั้งฉากคือแกนที่ขั้นก่อนหน้าเพิ่งจัดเสร็จ ปล่อยให้ Sharp แตะคือล้างทิ้ง
        driver.align_to_wall(target_mm, heading, config.AIM_MAX_MOVE_M,
                             center=(index == 1), start_tof=measured)

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
    เดินเข้าห้องมา ของที่วางจะไปกองขวางทางกลับพอดี กรณีนั้นหุ่นหันหลังให้ของแล้ว
    เจอของขวางทางที่เพิ่งเดินมา ซึ่ง ``Maze.mark_object`` จดไว้ให้แล้วตั้งแต่ตอน
    ปล่อยมือ ขากลับจึงต้องหาทางอื่น หรือจบด้วย [FAIL] ถ้าไม่มีทางอื่นให้ไป

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
    # align_first=False: ของที่เพิ่งวางกองอยู่หน้าหุ่นในระยะแขน ซึ่งใกล้กว่าเกณฑ์
    # กำแพงหน้า ถ้าปล่อยให้ align_front ทำงาน หุ่นจะเดินเข้าหา "กำแพง" ที่จริง ๆ
    # แล้วคือของของตัวเอง แล้วเขี่ยมันล้มก่อนจะได้หมุน
    return driver.turn_to(heading, (entry_heading + 2) % 4, align_first=False)

# -*- coding: utf-8 -*-
"""ชั้นควบคุมการเคลื่อนที่ระดับหนึ่งช่องและหนึ่งการเลี้ยว"""
import math
import time

from . import config
from .directions import DIR_NAMES
from .geometry import clamp, wrap_deg
from .readings import ir_triggered, sharp_polarity, wall_from_adc


class Driver(object):
    """ชั้นควบคุมการเคลื่อนที่ระดับ "หนึ่งช่อง" และ "หนึ่งการเลี้ยว"

    การหมุนทุกครั้งใช้ ``chassis.drive_speed`` แบบปิดลูปกับ IMU เท่านั้น
    ไม่ใช้ ``chassis.move(z=)`` เลย เพราะสองตัวนี้ใช้เครื่องหมายตรงข้ามกัน
    (ดูหัวข้อ "ข้อควรรู้เรื่อง SDK" ใน ``maze_solver/__init__.py``)

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
        self.chassis.drive_speed(x=x, y=y, z=z, timeout=config.DRIVE_WATCHDOG_S)

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
        self._drive(z=config.YAW_SIGN_TEST_DPS)
        time.sleep(config.YAW_SIGN_TEST_S)
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
              .format(config.YAW_SIGN_TEST_DPS * config.YAW_SIGN_TEST_S, delta, self.yaw_sign))

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
    def _rotate_to_abs(self, target_yaw, tolerance=config.TURN_TOLERANCE_DEG):
        """หมุนแบบปิดลูปจนกว่า yaw ดิบจะเท่ากับ target_yaw

        Returns:
            bool: True ถ้าเข้าเป้าภายในเวลาที่กำหนด
        """
        deadline = time.time() + config.TURN_TIMEOUT_S
        while time.time() < deadline:
            error = wrap_deg(target_yaw - self.hub.snapshot().yaw)
            if abs(error) <= tolerance:
                self.stop()
                time.sleep(0.15)
                return True
            speed = clamp(config.KP_TURN * error, -config.TURN_MAX_DPS, config.TURN_MAX_DPS)
            if abs(speed) < config.TURN_MIN_DPS:
                speed = config.TURN_MIN_DPS if error > 0 else -config.TURN_MIN_DPS
            self._drive(z=self.yaw_sign * speed)
            time.sleep(0.03)

        self.stop()
        time.sleep(0.15)
        final = wrap_deg(target_yaw - self.hub.snapshot().yaw)
        print("[WARN] หมุนไม่เข้าเป้าใน {0} วินาที เหลือ error {1:.2f} องศา"
              .format(config.TURN_TIMEOUT_S, final))
        return False

    def turn_to(self, current_heading, target_heading, align_first=True):
        """หมุนจากทิศหนึ่งไปอีกทิศหนึ่ง

        การกลับหลังหัน 180 องศาจะถูกแยกเป็นสองครั้ง ครั้งละ 90 องศา เพราะที่
        error พอดี 180 องศานั้นทิศทางที่ใกล้ที่สุดมีสองทางเท่ากัน ตัวคุมจะลังเล
        และอาจสั่นไปมา การบังคับผ่านจุดกึ่งกลางทำให้ทิศทางชัดเจนเสมอ

        Args:
            current_heading (int): ทิศที่หันอยู่ตอนนี้
            target_heading (int): ทิศที่ต้องการหันไป
            align_first (bool): True = จัดระยะกับกำแพงหน้าก่อนหมุนถ้ามีกำแพง
                False = หมุนอย่างเดียว ห้ามแตะล้อตามแนวเดิน ใช้เมื่อตำแหน่งตาม
                แนวที่หันอยู่ "ถูกตั้งไว้แล้ว" และการจัดระยะซ้ำคือการทำลายมัน
                (ดูที่เรียกใน place_on_target กับ face_way_back)

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
        #
        # ข้ามได้ด้วย align_first=False เพราะแกนที่ align_front ขยับคือแกนที่
        # กำลังจะกลายเป็นแกนด้านข้างหลังหมุนเสร็จ ตอนเดินสำรวจนั่นคือสิ่งที่
        # ต้องการ (เข้ากลางช่องก่อนออกตัว) แต่ตอนเล็งเป้าก่อนวาง แกนนั้นคือค่าที่
        # ขั้นก่อนหน้าเพิ่งจัดไว้ การจัดซ้ำเข้าหา FRONT_STOP_MM คือการล้างทิ้ง
        if align_first:
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
        if snap.tof_mm is None or snap.tof_mm > config.FRONT_STOP_MM + 150:
            return
        if snap.tof_mm < 60:
            print("[ALIGN] ใกล้กำแพงเกินไป ({0}mm) ไม่จัดระยะ".format(snap.tof_mm))
            return

        for _ in range(25):
            snap = self.hub.snapshot()
            if snap.tof_mm is None:
                break
            error = snap.tof_mm - config.FRONT_STOP_MM
            if abs(error) < config.ALIGN_TOLERANCE_MM:
                break
            self._drive(x=config.ALIGN_SPEED if error > 0 else -config.ALIGN_SPEED)
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
        left = wall_from_adc(snap.adc_left, config.SHARP_LEFT_WALL_ADC)
        right = wall_from_adc(snap.adc_right, config.SHARP_RIGHT_WALL_ADC)
        near_left = near_right = None
        if left:
            near_left = (sharp_polarity(config.SHARP_LEFT_WALL_ADC)
                         * (snap.adc_left - config.SHARP_LEFT_REF))
        if right:
            near_right = (sharp_polarity(config.SHARP_RIGHT_WALL_ADC)
                          * (snap.adc_right - config.SHARP_RIGHT_REF))

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

        if abs(error) < config.CENTER_DEADBAND_ADC:
            return 0.0
        return clamp(config.KP_CENTER * error, -config.MAX_STRAFE, config.MAX_STRAFE)

    def advance_one_cell(self, heading):
        """เดินหน้าหนึ่งช่องตาราง

        จบการเดินได้ 2 แบบที่ถือว่าสำเร็จ คือเดินครบระยะตาม odometry หรือหยุด
        เพราะ ToF เจอกำแพงที่ระยะกลางช่องพอดี ส่วนกรณีอื่นถือว่าไม่สำเร็จ

        Returns:
            tuple: (ok, traveled_m, reason)
        """
        self.stop()
        time.sleep(config.SETTLE_S)

        start = self.hub.snapshot()
        start_x, start_y = start.pos_x, start.pos_y
        base_target_yaw = self.heading_yaw(heading)
        timeout = (config.CELL_SIZE_M / config.BASE_SPEED) * config.MOVE_TIMEOUT_RATIO
        deadline = time.time() + timeout
        reason = "timeout"
        brake_hits = 0

        while time.time() < deadline:
            snap = self.hub.snapshot()
            if not snap.fresh:
                reason = "sensor_stale:" + snap.stale_reason
                break

            traveled = math.hypot(snap.pos_x - start_x, snap.pos_y - start_y)
            if traveled >= config.CELL_SIZE_M:
                reason = "odometry"
                break

            # เบรกเมื่อ ToF บอกว่าถึงระยะกลางช่องที่ติดกำแพงแล้ว ต้องเห็นติดกัน
            # สองรอบเพื่อกันค่าแวบเดียว และต้องเดินมาแล้วพอสมควรเพื่อไม่ให้ค่า
            # ตอนเพิ่งออกตัวจากกำแพงเดิมมาทำให้หยุดทันที
            if (snap.tof_mm is not None and snap.tof_mm <= config.FRONT_STOP_MM
                    and traveled > 0.15):
                brake_hits += 1
                if brake_hits >= 2:
                    reason = "tof_stop"
                    break
            else:
                brake_hits = 0

            speed = config.BASE_SPEED
            if snap.tof_mm is not None and snap.tof_mm < config.FRONT_STOP_MM + 150:
                speed = config.SLOW_SPEED

            left_ir = ir_triggered(snap.ir_left)
            right_ir = ir_triggered(snap.ir_right)

            # IR 45 องศาอุดจุดบอดมุมทแยงหน้า ที่ Sharp (ยิงตรงข้าง) และ ToF
            # (ยิงตรงหน้า) มองไม่เห็น แต่ไม่ให้มันจบการเคลื่อนที่เอง เพราะระยะ
            # ที่มันติดขึ้นกับการหมุน pot ของโมดูล ซึ่งเราคุมไม่ได้จากโค้ด
            guard_dir = 0
            if config.USE_IR_AS_GUARD and (left_ir or right_ir):
                speed = min(speed, config.GUARD_SPEED)
                if left_ir and not right_ir:
                    guard_dir = 1
                elif right_ir and not left_ir:
                    guard_dir = -1

            if guard_dir:
                strafe = guard_dir * config.GUARD_STRAFE
            else:
                strafe = self._centering_strafe(snap)

            # โหมด slant ปรับ "เป้าหมายของมุม" ไม่ใช่ไปบวกความเร็วหมุนตรง ๆ
            # เพื่อไม่ให้ไปสู้กับตัวคุม yaw ที่กำลังดึงกลับเป้าหมายเดิมอยู่
            target_yaw = base_target_yaw
            if config.USE_IR_AS_SLANT and guard_dir:
                target_yaw = wrap_deg(
                    base_target_yaw + self.yaw_sign * guard_dir * config.IR_SLANT_BIAS_DEG)

            yaw_error = wrap_deg(target_yaw - snap.yaw)
            turn = 0.0
            if abs(yaw_error) > config.YAW_HOLD_DEADBAND_DEG:
                turn = clamp(config.KP_YAW_HOLD * yaw_error,
                             -config.MAX_YAW_CORRECT_DPS, config.MAX_YAW_CORRECT_DPS)

            self._drive(x=speed, y=strafe, z=self.yaw_sign * turn)
            time.sleep(config.CONTROL_DT)

        self.stop()
        time.sleep(0.15)

        snap = self.hub.snapshot()
        traveled = math.hypot(snap.pos_x - start_x, snap.pos_y - start_y)
        ok = (traveled >= config.CELL_SIZE_M * config.CELL_COMPLETE_RATIO
              or (reason == "tof_stop"
                  and traveled >= config.CELL_SIZE_M * config.TOF_STOP_MIN_RATIO))
        print("[MOVE] {0} เดินได้ {1:.3f} m (เป้า {2:.2f}) เหตุที่จบ: {3} -> {4}"
              .format(DIR_NAMES[heading], traveled, config.CELL_SIZE_M, reason,
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
                    + (budget_m / abs(speed)) * config.MOVE_TIMEOUT_RATIO + 1.0)
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
            if abs(yaw_error) > config.YAW_HOLD_DEADBAND_DEG:
                turn = clamp(config.KP_YAW_HOLD * yaw_error,
                             -config.MAX_YAW_CORRECT_DPS, config.MAX_YAW_CORRECT_DPS)
            strafe = self._centering_strafe(snap) if center else 0.0
            self._drive(x=speed, y=strafe, z=self.yaw_sign * turn)
            time.sleep(config.CONTROL_DT)

        self.stop()
        time.sleep(0.15)
        snap = self.hub.snapshot()
        moved = math.hypot(snap.pos_x - start_x, snap.pos_y - start_y)
        return moved, reason

    def _align_pass(self, target_mm, heading, budget_m, floor_mm, center,
                    start_tof):
        """จัดระยะหนึ่งรอบ ดู :meth:`align_to_wall` สำหรับความหมายของทุกอย่าง

        แยกออกมาเพราะ ``align_to_wall`` เรียกซ้ำได้หลายรอบ ตัวนี้คือเนื้อของหนึ่ง
        รอบ ตั้งแต่วัด ตัดสินทิศ ไปจนขยับเสร็จ

        Args:
            start_tof (float or None): ระยะที่วัดมาแล้วจากรอบก่อน None = วัดใหม่

        Returns:
            tuple: (tof_mm หลังจบรอบ, moved_m, reason)
        """
        floor = config.FRONT_STOP_MM if floor_mm is None else floor_mm
        if start_tof is None:
            start_tof = self.hub.read_tof_settled()
        if start_tof is None:
            print("[ALIGN] ToF ไม่เห็นกำแพงในระยะวัด จัดระยะไม่ได้")
            return None, 0.0, "no_wall"

        error_mm = start_tof - target_mm
        if abs(error_mm) <= config.ALIGN_TOLERANCE_MM:
            print("[ALIGN] ToF = {0:.0f}mm ตรงเป้า {1:.0f}mm อยู่แล้ว"
                  .format(start_tof, target_mm))
            return start_tof, 0.0, "already_there"
        if budget_m <= 0.0:
            print("[ALIGN] ToF = {0:.0f}mm ห่างเป้า {1:.0f}mm แต่ขยับไม่ได้ (งบ 0)"
                  .format(start_tof, target_mm))
            return start_tof, 0.0, "no_room"

        # ToF มากกว่าเป้า = อยู่ไกลกำแพงเกินไป ต้องเดินหน้าเข้าหา และกลับกัน
        forward = error_mm > 0
        if forward and target_mm < floor:
            print("[ALIGN] เป้า {0:.0f}mm ใกล้กว่าระยะต่ำสุด {1}mm ไม่เดินเข้าไป"
                  .format(target_mm, floor))
            return start_tof, 0.0, "below_floor"

        # ขยับเกินระยะที่ผิดอยู่ไม่มีประโยชน์ เอาค่าที่น้อยกว่าเป็นงบระยะทาง
        budget = min(abs(error_mm) / 1000.0, budget_m)
        print("[ALIGN] ToF = {0:.0f}mm เป้า {1:.0f}mm -> {2} ไม่เกิน {3:.3f} m "
              "(งบ {4:.3f} m)"
              .format(start_tof, target_mm, "เดินหน้า" if forward else "ถอย",
                      budget, budget_m))

        if forward:
            def reached(s):
                return s.tof_mm is None or s.tof_mm <= max(target_mm, floor)
        else:
            def reached(s):
                return s.tof_mm is None or s.tof_mm >= target_mm

        speed = config.ALIGN_SPEED if forward else -config.ALIGN_SPEED
        moved, reason = self._travel(speed, heading, budget, stop=reached,
                                     center=center)
        return None, moved, reason

    def align_to_wall(self, target_mm, heading, budget_m, floor_mm=None,
                      center=True, start_tof=None):
        """เดินหน้าหรือถอยจนกำแพงที่หันหน้าใส่ห่างเท่ากับ target_mm

        นี่คือความสามารถพื้นฐานที่ทำให้หุ่นจอดที่ตำแหน่งย่อยในช่องได้ ไม่ใช่แค่
        กลางช่อง งานอย่างการวางของให้ตรงเป้าที่กำหนดเป็นระยะจากกำแพง จึงทำได้
        ด้วยการเรียกเมธอดนี้ทีละแกน โดยหันหน้าเข้าหากำแพงของแกนนั้น

        เหตุที่วัดได้แค่กำแพงด้านหน้า: ToF เป็นเซนเซอร์ตัวเดียวในหุ่นที่คืนค่า
        เป็นมิลลิเมตรจริง และมันยิงไปข้างหน้าอย่างเดียว Sharp ข้างคืนแค่ ADC ดิบ
        ที่บอกได้ว่ามีกำแพงไหมกับชิดกว่าหรือห่างกว่ากลางช่อง แปลงเป็นระยะไม่ได้
        จึงได้แค่ประคองไม่ให้เบียดกำแพงข้างระหว่างขยับ

        การขยับตามแนวที่หันอยู่ไม่กระทบตำแหน่งตามแกนตั้งฉาก การจัดทีละแกนจึง
        ไม่รบกวนกัน แต่มีเงื่อนไขว่าต้องไม่ strafe ระหว่างทาง ซึ่งคุมด้วย ``center``

        ``center=True`` ให้ Sharp ประคองกลางช่องระหว่างขยับ ซึ่งเป็นการ strafe
        ตามแกนตั้งฉาก ใช้ได้เฉพาะตอนที่แกนนั้น "ยังไม่ถูกจัด" - ตอนนั้นการถูกดึง
        เข้ากลางช่องเป็นผลดี เพราะแก้การเยื้องที่ yaw error สะสมไว้ให้ด้วย แต่ถ้า
        แกนตั้งฉากถูกจัดไปแล้วโดยขั้นก่อนหน้า ต้องใช้ ``center=False`` ไม่งั้น
        Sharp จะดันหุ่นกลับเข้ากลางช่องแล้วล้างงานของขั้นนั้นทิ้งเงียบ ๆ
        (Sharp เล็งกลางช่องเสมอ ไม่รู้จักเป้าที่เราตั้งไว้)

        Args:
            target_mm: ระยะที่ต้องการให้ ToF อ่านได้เมื่อจบ
            heading: ทิศที่หุ่นหันอยู่ ใช้ประคอง yaw
            budget_m: ขยับได้ไกลสุดกี่เมตร กันความเสียหายเมื่อ ToF อ่านเพี้ยน
            floor_mm: ห้ามเข้าใกล้กำแพงกว่านี้ None = ใช้ FRONT_STOP_MM
            center: True = ให้ Sharp ประคองกลางช่องระหว่างขยับ ต้องเป็น False
                เมื่อแกนตั้งฉากถูกจัดตำแหน่งไปแล้ว
            start_tof: ระยะที่ผู้เรียกวัดตอนจอดนิ่งมาแล้ว None = วัดเอง
                ส่งมาเพื่อไม่ต้องเสียเวลาวัดซ้ำ เมื่อผู้เรียกเพิ่งวัดไปเอง

        Returns:
            tuple: (tof_mm หลังจัด, moved_m, reason) โดย tof_mm เป็นค่ามัธยฐาน
                ที่วัดตอนจอดนิ่งแล้ว ไม่ใช่ค่าดิบครั้งเดียวตอนกำลังเบรก
        """
        total_moved = 0.0
        remaining = budget_m
        tof = None
        reason = "no_wall"
        measured = start_tof

        for attempt in range(1, config.ALIGN_MAX_PASSES + 1):
            tof, moved, reason = self._align_pass(
                target_mm, heading, remaining, floor_mm, center, measured)
            total_moved += moved
            remaining = max(0.0, remaining - moved)
            if reason != "stop":
                # ไม่ได้ขยับ หรือขยับไม่ได้ ทำซ้ำก็ได้ผลเดิม
                break

            # จบด้วย stop = ToF "ข้ามเกณฑ์" ระหว่างวิ่ง ซึ่งไม่ใช่ตำแหน่งสุดท้าย
            # จริง ๆ เพราะยังมีระยะที่หุ่นไหลต่อหลังสั่งหยุด วัดใหม่ตอนจอดนิ่ง
            measured = self.hub.read_tof_settled()
            tof = measured
            if measured is None:
                print("[ALIGN] ขยับ {0:.3f} m แล้ว ToF อ่านไม่ได้ตอนจอดนิ่ง"
                      .format(total_moved))
                break

            residual = measured - target_mm
            print("[ALIGN] รอบที่ {0}: ขยับรวม {1:.3f} m แล้ว ToF = {2:.0f}mm "
                  "(เป้า {3:.0f}mm เหลือ {4:+.0f}mm)"
                  .format(attempt, total_moved, measured, target_mm, residual))
            if abs(residual) <= config.ALIGN_TOLERANCE_MM:
                break
            if attempt >= config.ALIGN_MAX_PASSES:
                print("[WARN] ครบ {0} รอบแล้วยังเหลือ {1:+.0f}mm "
                      "(เกณฑ์ {2}mm) ใช้ตำแหน่งนี้ไปก่อน"
                      .format(config.ALIGN_MAX_PASSES, residual, config.ALIGN_TOLERANCE_MM))
                break
            if remaining <= 0.0:
                print("[WARN] ยังเหลือ {0:+.0f}mm แต่งบระยะทางหมดแล้ว"
                      .format(residual))
                reason = "budget"
                break

        if reason == "budget":
            print("[WARN] ใช้งบระยะทางหมดก่อนถึงเป้า ตำแหน่งยังไม่ตรงที่ตั้งไว้")
        return tof, total_moved, reason

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
            -config.ALIGN_SPEED, heading, budget_m,
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
        self._travel(-config.BACKUP_SPEED, heading, distance_m, center=False)

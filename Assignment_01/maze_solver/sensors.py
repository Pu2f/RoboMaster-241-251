# -*- coding: utf-8 -*-
"""ชั้นอ่านเซนเซอร์ - เจ้าของ subscription ทั้งหมดของเซนเซอร์นำทาง"""
import statistics
import threading
import time

from . import config
from .geometry import front_wall_threshold_mm
from .readings import wall_from_adc


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
        ports = (("SHARP_LEFT", config.SHARP_LEFT), ("SHARP_RIGHT", config.SHARP_RIGHT),
                 ("IR_LEFT_45", config.IR_LEFT_45), ("IR_RIGHT_45", config.IR_RIGHT_45))
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
        self._sensor.sub_distance(freq=config.DDS_FREQ, callback=self._on_tof)
        if self.use_adapter:
            self._adaptor.sub_adapter(freq=config.DDS_FREQ, callback=self._on_adapter)
            self._adapter_subscribed = True
        self._chassis.sub_attitude(freq=config.DDS_FREQ, callback=self._on_attitude)
        self._chassis.sub_position(freq=config.DDS_FREQ, callback=self._on_position)

        deadline = time.time() + config.ADAPTER_WAIT_S
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
                  .format(config.ADAPTER_WAIT_S))
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
        for key, (hub, port) in (("l", config.SHARP_LEFT), ("r", config.SHARP_RIGHT)):
            try:
                adc[key] = self._adaptor.get_adc(id=hub, port=port)
            except Exception:                           # noqa: BLE001
                adc[key] = None
        for key, (hub, port) in (("l", config.IR_LEFT_45), ("r", config.IR_RIGHT_45)):
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
            tof_raw = self._tof[config.TOF_INDEX] if config.TOF_INDEX < len(self._tof) else 0
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
            adc_left = ad[self._adapter_index(*config.SHARP_LEFT)]
            adc_right = ad[self._adapter_index(*config.SHARP_RIGHT)]
            ir_left = io[self._adapter_index(*config.IR_LEFT_45)]
            ir_right = io[self._adapter_index(*config.IR_RIGHT_45)]
        else:
            adc, io_map = self._poll_adaptor(now)
            adc_left, adc_right = adc["l"], adc["r"]
            ir_left, ir_right = io_map["l"], io_map["r"]
            adapter_age = now - self._poll_ok_t if self._poll_ok_t else 1e9

        # ToF: ค่า 0 หรือค่าเกินพิสัย แปลว่าไม่มีเป้าหมายอยู่ในระยะ ไม่ใช่สตรีมพัง
        # กรณีสตรีมพังจริงจะถูกจับด้วย tof_age ด้านล่างแทน
        tof_mm = tof_raw if 0 < tof_raw <= config.TOF_MAX_VALID_MM else None

        stale = ""
        if tof_age > config.SENSOR_STALE_S:
            stale = "tof"
        elif att_age > config.SENSOR_STALE_S:
            stale = "attitude"
        elif pos_age > config.SENSOR_STALE_S:
            stale = "position"
        elif adapter_age > config.SENSOR_STALE_S:
            stale = "adapter"

        return SensorSnapshot(
            t=now, tof_mm=tof_mm, adc_left=adc_left, adc_right=adc_right,
            ir_left=ir_left, ir_right=ir_right, yaw=yaw,
            pos_x=pos_x, pos_y=pos_y, fresh=(stale == ""), stale_reason=stale)

    def read_tof_settled(self, samples=config.TOF_SETTLE_SAMPLES):
        """float or None: ระยะ ToF หน่วย mm จากค่ามัธยฐานของหลายครั้งตอนจอดนิ่ง

        ต่างจากการอ่าน ``snapshot().tof_mm` ครั้งเดียวตรงที่ทนค่าโดดได้ ใช้ทุกที่
        ที่เอาตัวเลขไปตัดสินใจเรื่องระยะจริง ๆ ไม่ใช่แค่ถามว่ามีกำแพงหรือเปล่า

        คืน None เมื่อครึ่งหนึ่งขึ้นไปอ่านไม่ได้ ซึ่งแปลว่าไม่มีอะไรอยู่ในระยะวัด
        ไม่ใช่ค่าที่เชื่อได้แค่บางส่วน - ยอมบอกว่าไม่รู้ ดีกว่าคืนมัธยฐานของ
        ตัวอย่างหยิบมือเดียวแล้วให้ผู้เรียกเข้าใจผิดว่าวัดได้

        Args:
            samples (int): จำนวนครั้งที่อ่าน

        Returns:
            float or None: ระยะหน่วย mm
        """
        samples = max(1, samples)
        values = []
        for _ in range(samples):
            snap = self.snapshot()
            if snap.tof_mm is not None:
                values.append(snap.tof_mm)
            time.sleep(config.WALL_VOTE_INTERVAL)
        if len(values) * 2 <= samples:
            return None
        return float(statistics.median(values))

    def read_walls_settled(self, samples=config.WALL_VOTE_SAMPLES):
        """อ่านกำแพงหน้า/ซ้าย/ขวา ตอนหุ่นจอดนิ่งแล้ว โดยโหวตเสียงข้างมาก

        ค่าที่ก้ำกึ่งจนตัดสินไม่ได้จะนับเป็น "ไม่มีกำแพง" โดยตั้งใจ เพราะสองความ
        ผิดพลาดนี้ไม่เท่ากัน:

        * เดาว่าไม่มีกำแพงทั้งที่มี - แก้ตัวเองได้ พอหมุนไปทางนั้นแล้ว pre-move
          check จะเห็นกำแพงด้วย ToF แล้วมาร์กลงแผนที่ให้เอง
        * เดาว่ามีกำแพงทั้งที่ไม่มี - ปิดทางเดินนั้นถาวรและไม่มีอะไรมาแก้ให้

        คืนระยะ ToF มาด้วย เพราะเก็บตัวอย่างชุดเดียวกันอยู่แล้ว การแยกไปเรียก
        :meth:`read_tof_settled` อีกรอบคือการจอดรออ่านเซนเซอร์ซ้ำฟรี ๆ ทุกช่อง

        Returns:
            tuple: (front, left, right, tof_mm) สามตัวแรกเป็น bool ตัวสุดท้าย
                เป็นระยะหน่วย mm หรือ None เมื่อส่วนใหญ่อ่านไม่ได้
        """
        samples = max(1, samples)
        votes = {"front": [0, 0], "left": [0, 0], "right": [0, 0]}
        distances = []
        for _ in range(samples):
            snap = self.snapshot()
            if snap.tof_mm is not None:
                distances.append(snap.tof_mm)
            for key, value in (("front", snap.front_wall()),
                               ("left", wall_from_adc(snap.adc_left,
                                                      config.SHARP_LEFT_WALL_ADC)),
                               ("right", wall_from_adc(snap.adc_right,
                                                       config.SHARP_RIGHT_WALL_ADC))):
                if value is True:
                    votes[key][0] += 1
                elif value is False:
                    votes[key][1] += 1
            time.sleep(config.WALL_VOTE_INTERVAL)
        tof_mm = (float(statistics.median(distances))
                  if len(distances) * 2 > samples else None)
        return (votes["front"][0] > votes["front"][1],
                votes["left"][0] > votes["left"][1],
                votes["right"][0] > votes["right"][1],
                tof_mm)

# -*- coding: utf-8 -*-
"""ทดสอบการจัดระยะเทียบกำแพงและการเล็งเป้าก่อนวางของ

ครอบคลุมทั้งตระกูลที่ใช้ลูป ``Driver._travel`` ร่วมกัน
- ``align_to_wall``      จอดให้ห่างกำแพงเท่ากับค่าที่กำหนด (เดินหน้าหรือถอยก็ได้)
- ``back_off_from_wall`` ถอยให้ห่าง "อย่างน้อย" เท่านี้ ไม่ดึงกลับเข้าหากำแพง
- ``backup``             ถอยตามระยะ ไม่ประคองกลางช่อง
- ``place_on_target``    เล็งทีละแกนแล้ววาง ผ่าน run_search

ระดับ Driver ใช้ ``Corridor`` ที่จำลองตำแหน่งเป็นเมตรและ ToF ที่ขยับตามจริง
ส่วนระดับ run_search ใช้ ``TruthDriver`` ที่บันทึกคำสั่งไว้ให้ตรวจ
"""
import sys

from fakes import (Checker, Corridor, load, make_driver, make_payload,
                   make_world, quiet)


# =====================================================================
# back_off_from_wall - ตัวกันแขนชนกำแพง
# =====================================================================
def test_no_move_cases(chk):
    """สามกรณีที่ต้องไม่ขยับล้อเลยแม้แต่คำสั่งเดียว"""
    chk.section("ถอยห่างกำแพง: กรณีที่ไม่ต้องถอย")

    tc = load()
    target = tc.GOAL_WALL_CLEARANCE_MM or 400

    road = Corridor(tof_mm=None)
    got, out = quiet(make_driver(tc, road).back_off_from_wall, target, 0, 0.6)
    chk.check("ToF ไม่เห็นกำแพง: reason = no_wall", got[2] == "no_wall")
    chk.check("ToF ไม่เห็นกำแพง: ไม่ขยับล้อ", road.moves == [])
    chk.check("ToF ไม่เห็นกำแพง: บอกเหตุผล", "ไม่เห็นกำแพงในระยะวัด" in out)

    road = Corridor(tof_mm=target + 100)
    got, out = quiet(make_driver(tc, road).back_off_from_wall, target, 0, 0.6)
    chk.check("ห่างพออยู่แล้ว: reason = already_clear",
              got[2] == "already_clear")
    chk.check("ห่างพออยู่แล้ว: ไม่ขยับล้อ", road.moves == [])
    chk.check("ห่างพออยู่แล้ว: ไม่ดึงกลับเข้าหากำแพง",
              "{0}mm อยู่แล้ว".format(target + 100) in out)

    road = Corridor(tof_mm=tc.FRONT_STOP_MM)
    got, out = quiet(make_driver(tc, road).back_off_from_wall, target, 0, 0.0)
    chk.check("ไม่มีที่ถอย: reason = no_room", got[2] == "no_room")
    chk.check("ไม่มีที่ถอย: ไม่ขยับล้อ", road.moves == [])
    chk.check("ไม่มีที่ถอย: บอกว่าไม่มีพื้นที่ข้างหลัง",
              "ไม่มีพื้นที่ข้างหลัง" in out)


def test_backs_off_until_clear(chk):
    """ถอยจนได้ระยะที่ตั้งไว้ แล้วหยุดทันที ไม่ถอยเกิน"""
    chk.section("ถอยห่างกำแพง: ถอยจนได้ระยะ")

    tc = load()
    target = 400
    start_mm = tc.FRONT_STOP_MM                         # จอดชิดกำแพงตามที่ ToF เบรก
    road = Corridor(tof_mm=start_mm)
    got, out = quiet(make_driver(tc, road).back_off_from_wall, target, 0, 0.6)
    tof_after, moved, reason = got

    chk.check("reason = clear", reason == "clear")
    chk.check("ได้ระยะที่ต้องการจริง ({0} >= {1})".format(tof_after, target),
              tof_after >= target)
    need = (target - start_mm) / 1000.0
    chk.check("ถอยไปประมาณเท่าที่ต้องพอดี ({0:.3f} ~ {1:.3f} m)"
              .format(moved, need), abs(moved - need) < 0.01)
    chk.check("ถอยจริง ไม่ใช่เดินหน้า", all(c[0] < 0 for c in road.moves))
    chk.check("จบด้วยคำสั่งหยุด", road.commands[-1] == (0, 0, 0))
    chk.check("รายงานระยะที่ทำได้", "เหตุที่จบ: clear" in out)


def test_stops_at_limit(chk):
    """เพดานระยะทางต้องชนะค่าที่ตั้งไว้ และต้องเตือนว่าได้ไม่ครบ"""
    chk.section("ถอยห่างกำแพง: ชนเพดานก่อนได้ระยะ")

    tc = load()
    road = Corridor(tof_mm=tc.FRONT_STOP_MM)
    got, out = quiet(make_driver(tc, road).back_off_from_wall, 400, 0, 0.10)
    tof_after, moved, reason = got

    chk.check("reason = limit", reason == "limit")
    chk.check("ไม่ถอยเกินเพดาน ({0:.3f} <= 0.10 m)".format(moved),
              moved <= 0.10 + 0.01)
    chk.check("ยังไม่ถึงระยะที่ตั้งไว้", tof_after < 400)
    chk.check("เตือนว่าแขนอาจยังชนกำแพง", "แขนอาจยังชนกำแพง" in out)
    chk.check("บอกทางแก้ทั้งสองทาง",
              "GOAL_WALL_CLEARANCE_MM" in out and "ARM_PLACE_XY" in out)


def test_sensor_guards(chk):
    """เซนเซอร์ขาดการอัปเดตต้องหยุดทันที ไม่ใช่ขยับต่อแบบตาบอด"""
    chk.section("ลูปเคลื่อนที่: เซนเซอร์ขาดการอัปเดต")

    tc = load()
    road = Corridor(tof_mm=tc.FRONT_STOP_MM, stale=True)
    got, _ = quiet(make_driver(tc, road).back_off_from_wall, 400, 0, 0.6)
    chk.check("ถอย: reason บอกว่าเซนเซอร์ค้าง", got[2].startswith("sensor_stale"))
    chk.check("ถอย: ไม่ขยับล้อเลย", road.moves == [])
    chk.check("ถอย: จบด้วยคำสั่งหยุด", road.commands[-1] == (0, 0, 0))

    road = Corridor(tof_mm=800, stale=True)
    got, _ = quiet(make_driver(tc, road).align_to_wall, 460, 0, 0.35)
    chk.check("จัดระยะ: reason บอกว่าเซนเซอร์ค้าง",
              got[2].startswith("sensor_stale"))
    chk.check("จัดระยะ: ไม่ขยับล้อเลย", road.moves == [])


def test_sharp_keeps_it_centred(chk):
    """Sharp ต้องประคองกลางช่องระหว่างขยับ ไม่ปล่อยให้เบียดกำแพงข้าง

    Sharp บอกระยะเป็นมิลลิเมตรไม่ได้ สิ่งเดียวที่มันทำได้คือดันหุ่นออกจากข้างที่
    ชิดกว่าจุดอ้างอิง เทสต์นี้จึงตรวจ "ทิศของการดัน" ไม่ใช่ระยะ
    """
    chk.section("ลูปเคลื่อนที่: Sharp ประคองกลางช่อง")

    tc = load()
    # ค่า ADC ที่แปลว่ามีกำแพงซ้ายและชิดกว่าจุดกึ่งกลางช่อง
    near_left = max(tc.SHARP_LEFT_WALL_ADC) + 40
    road = Corridor(tof_mm=tc.FRONT_STOP_MM, adc=(near_left, 0))
    quiet(make_driver(tc, road).back_off_from_wall, 400, 0, 0.6)
    strafes = [c[1] for c in road.moves]
    chk.check("มีการ strafe ระหว่างขยับ", any(s != 0.0 for s in strafes))
    chk.check("ดันออกจากกำแพงซ้ายทุกครั้ง", all(s > 0 for s in strafes if s))
    chk.check("ไม่เกินเพดาน strafe",
              all(abs(s) <= tc.MAX_STRAFE + 1e-9 for s in strafes))

    road = Corridor(tof_mm=tc.FRONT_STOP_MM, adc=(0, 0))
    quiet(make_driver(tc, road).back_off_from_wall, 400, 0, 0.6)
    chk.check("ไม่มีกำแพงข้าง: ไม่ strafe มั่ว",
              all(c[1] == 0.0 for c in road.moves))


# =====================================================================
# align_to_wall - ความสามารถใหม่ จอดที่ตำแหน่งย่อยในช่อง
# =====================================================================
def test_align_both_directions(chk):
    """จัดระยะได้ทั้งสองทาง เพราะจุดวางอาจอยู่หน้าหรือหลังที่หุ่นจอด"""
    chk.section("จัดระยะ: เดินหน้าและถอย")

    tc = load()

    road = Corridor(tof_mm=800)                         # ไกลกำแพงเกินไป
    got, _ = quiet(make_driver(tc, road).align_to_wall, 460, 0, 0.35)
    tof_after, moved, reason = got
    chk.check("ไกลเกิน: เดินหน้าเข้าหากำแพง",
              road.moves and all(c[0] > 0 for c in road.moves))
    chk.check("ไกลเกิน: ถึงเป้า ({0} ~ 460)".format(tof_after),
              abs(tof_after - 460) <= tc.ALIGN_TOLERANCE_MM)
    chk.check("ไกลเกิน: ขยับเท่าที่ผิดพอดี ({0:.3f} ~ 0.340)".format(moved),
              abs(moved - 0.340) < 0.01)
    chk.check("ไกลเกิน: reason = stop", reason == "stop")

    road = Corridor(tof_mm=200)                         # ชิดกำแพงเกินไป
    got, _ = quiet(make_driver(tc, road).align_to_wall, 460, 0, 0.35)
    tof_after, moved, reason = got
    chk.check("ชิดเกิน: ถอยออกจากกำแพง",
              road.moves and all(c[0] < 0 for c in road.moves))
    chk.check("ชิดเกิน: ถึงเป้า ({0} ~ 460)".format(tof_after),
              abs(tof_after - 460) <= tc.ALIGN_TOLERANCE_MM)
    chk.check("ชิดเกิน: ขยับเท่าที่ผิดพอดี ({0:.3f} ~ 0.260)".format(moved),
              abs(moved - 0.260) < 0.01)


def test_align_guards(chk):
    """ด่านกันความเสียหายเมื่อค่าเป้าหรือค่าที่อ่านได้ไม่สมเหตุสมผล"""
    chk.section("จัดระยะ: ด่านกันความเสียหาย")

    tc = load()

    road = Corridor(tof_mm=465)
    got, out = quiet(make_driver(tc, road).align_to_wall, 460, 0, 0.35)
    chk.check("อยู่ในระยะคลาดเคลื่อนแล้ว: ไม่ขยับ", road.moves == [])
    chk.check("อยู่ในระยะคลาดเคลื่อนแล้ว: reason = already_there",
              got[2] == "already_there")
    chk.check("อยู่ในระยะคลาดเคลื่อนแล้ว: บอกว่าตรงเป้าแล้ว", "ตรงเป้า" in out)

    road = Corridor(tof_mm=None)
    got, out = quiet(make_driver(tc, road).align_to_wall, 460, 0, 0.35)
    chk.check("ไม่เห็นกำแพง: ไม่ขยับ", road.moves == [])
    chk.check("ไม่เห็นกำแพง: reason = no_wall", got[2] == "no_wall")

    road = Corridor(tof_mm=300)
    got, out = quiet(make_driver(tc, road).align_to_wall,
                     tc.FRONT_STOP_MM - 20, 0, 0.35)
    chk.check("เป้าใกล้กว่าระยะต่ำสุด: ไม่เดินเข้าไปชน", road.moves == [])
    chk.check("เป้าใกล้กว่าระยะต่ำสุด: reason = below_floor",
              got[2] == "below_floor")
    chk.check("เป้าใกล้กว่าระยะต่ำสุด: บอกระยะต่ำสุดที่ยอมให้",
              "{0}mm".format(tc.FRONT_STOP_MM) in out)

    road = Corridor(tof_mm=800)
    got, out = quiet(make_driver(tc, road).align_to_wall, 460, 0, 0.10)
    chk.check("งบระยะทางหมดก่อน: reason = budget", got[2] == "budget")
    chk.check("งบระยะทางหมดก่อน: ไม่เกินงบ ({0:.3f} <= 0.10)".format(got[1]),
              got[1] <= 0.10 + 0.01)
    chk.check("งบระยะทางหมดก่อน: เตือนว่ายังไม่ตรงที่ตั้งไว้",
              "ใช้งบระยะทางหมดก่อนถึงเป้า" in out)

    road = Corridor(tof_mm=800)
    got, out = quiet(make_driver(tc, road).align_to_wall, 460, 0, 0.0)
    chk.check("งบ 0: ไม่ขยับ", road.moves == [])
    chk.check("งบ 0: reason = no_room", got[2] == "no_room")


def test_backup_behaviour_unchanged(chk):
    """backup ต้องยังถอยตามระยะและยังไม่ประคองกลางช่องเหมือนเดิม

    มันถูกเรียกตอนหุ่นเพิ่งเดินไม่ผ่าน ซึ่งอาจติดขัดหรือเบียดอะไรอยู่ การเพิ่ม
    การเลื่อนข้างเข้าไปตอนนั้นจะทำให้เดาไม่ออกว่าหุ่นไปจบตรงไหน
    """
    chk.section("ถอยกลับเข้าช่องเดิม")

    tc = load()
    near_left = max(tc.SHARP_LEFT_WALL_ADC) + 40
    road = Corridor(tof_mm=1000, adc=(near_left, 0))
    quiet(make_driver(tc, road).backup, 0.20, 0)
    chk.check("ถอยได้ตามระยะ ({0:.3f} ~ 0.200)".format(-road.pos),
              abs(-road.pos - 0.20) < 0.01)
    chk.check("ถอยจริง ไม่ใช่เดินหน้า", all(c[0] < 0 for c in road.moves))
    chk.check("ไม่ประคองกลางช่องแม้มีกำแพงข้าง",
              all(c[1] == 0.0 for c in road.moves))

    road = Corridor(tof_mm=1000)
    quiet(make_driver(tc, road).backup, 0.02, 0)
    chk.check("ระยะสั้นกว่า 3 cm: ไม่ขยับเลย", road.commands == [])


# =====================================================================
# place_on_target - เล็งทีละแกนแล้ววาง ผ่าน run_search
# =====================================================================
def test_aim_sequence_wiring(chk):
    """run_search ต้องเดินตาม AIM_SEQUENCE ครบทุกขั้นก่อนวางของ"""
    chk.section("เล็งเป้า: run_search เดินตามลำดับ")

    tc = load()
    measured = [s for s in tc.AIM_SEQUENCE if s[1] is not None]
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("ถึงเป้าหมาย", got is True)
    chk.check("หันครบทุกขั้นที่กำหนดทิศไว้",
              len(driver.turns) >= len([s for s in tc.AIM_SEQUENCE
                                        if s[0] is not None]))
    chk.check("จัดระยะเฉพาะขั้นที่กำหนดระยะไว้ ({0} จาก {1} ขั้น)"
              .format(len(measured), len(tc.AIM_SEQUENCE)),
              len(driver.aligns) == len(measured))
    chk.check("ส่งค่าเป้าตามที่ตั้งไว้",
              [a[0] for a in driver.aligns] == [s[1] for s in measured])
    chk.check("ส่งงบระยะทางตามที่ตั้งไว้",
              all(a[2] == tc.AIM_MAX_MOVE_M for a in driver.aligns))
    chk.check("จัดระยะในทิศที่กำหนดไว้",
              [a[1] for a in driver.aligns] == [s[0] for s in measured])
    chk.check("เล็งก่อนวางของ", out.index("[AIM]") < out.index("จุดวางของ"))
    chk.check("ขั้นสุดท้ายหันทิศที่จะยื่นแขนวาง",
              "ขั้นที่ {0}/{0} หัน{1}".format(
                  len(tc.AIM_SEQUENCE),
                  tc.DIR_NAMES[tc.AIM_SEQUENCE[-1][0]]) in out)
    chk.check("ไม่ใช้การถอยห่างกำแพงคู่กัน", driver.backoffs == [])
    chk.check("ไม่มีของค้างในมือ", payload.holding is False)


def test_fallback_without_aim(chk):
    """AIM_SEQUENCE ว่าง ต้องกลับไปใช้ทางเดิมคือถอยห่างกำแพงแล้ววาง"""
    chk.section("เล็งเป้า: ทางเดิมเมื่อไม่ได้ตั้ง AIM_SEQUENCE")

    tc = load()
    tc.AIM_SEQUENCE = []
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("ถึงเป้าหมาย", got is True)
    chk.check("ไม่เล็ง", driver.aligns == [])
    chk.check("ถอยห่างกำแพงหนึ่งครั้ง", len(driver.backoffs) == 1)
    chk.check("ส่งค่า GOAL_WALL_CLEARANCE_MM ไปให้",
              driver.backoffs[0][0] == tc.GOAL_WALL_CLEARANCE_MM)
    chk.check("เพดาน = ระยะที่เพิ่งเดินเข้าช่องมา ({0:.2f} m)"
              .format(driver.backoffs[0][2]),
              abs(driver.backoffs[0][2] - tc.CELL_SIZE_M) < 1e-9)
    chk.check("ยังวางของ", "จุดวางของ" in out)

    tc = load()
    tc.AIM_SEQUENCE = []
    tc.GOAL_WALL_CLEARANCE_MM = None
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("ปิดทั้งคู่: ยังถึงเป้าหมาย", got is True)
    chk.check("ปิดทั้งคู่: ไม่เล็งและไม่ถอย",
              driver.aligns == [] and driver.backoffs == [])
    chk.check("ปิดทั้งคู่: ยังวางของตามปกติ", "จุดวางของ" in out)


class _StubDriver(object):
    """driver ที่จัดระยะไม่สำเร็จทุกครั้ง ใช้ตรวจว่ายังวางของต่อไหม"""

    def __init__(self, reason="budget"):
        self.reason = reason
        self.turns = []
        self.aligns = []
        #: list: ค่า align_first ของแต่ละครั้งที่ถูกสั่งหัน
        self.align_flags = []

    def turn_to(self, current_heading, target_heading, align_first=True):
        self.turns.append((current_heading, target_heading))
        self.align_flags.append(align_first)
        return target_heading

    def align_to_wall(self, target_mm, heading, budget_m, floor_mm=None):
        self.aligns.append((target_mm, heading, budget_m))
        return target_mm, 0.0, self.reason


def test_turn_only_step(chk):
    """ขั้นที่ระยะเป็น None ต้องหันแล้วผ่านไป ไม่แตะล้อ

    ใช้เมื่อวัดแล้วพบว่าตำแหน่งตามแกนนั้นตรงเป้าอยู่แล้วตั้งแต่ตอนเดินเข้าช่อง
    การจัดระยะซ้ำมีแต่จะดันหุ่นออกจากจุดที่ถูกอยู่แล้ว
    """
    chk.section("เล็งเป้า: ขั้นที่หันอย่างเดียว")

    tc = load()
    tc.AIM_SEQUENCE = [(2, 560), (3, None)]
    driver = _StubDriver()
    payload, arm, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    quiet(payload.pick_up, None)
    arm.calls = []
    heading, out = quiet(tc.place_on_target, driver, payload, 0, 0.6)

    chk.check("หันครบสองขั้น", [t[1] for t in driver.turns] == [2, 3])
    chk.check("จัดระยะแค่ขั้นแรก", [a[0] for a in driver.aligns] == [560])
    chk.check("บอกว่าขั้นที่สองไม่จัดระยะ", "แล้วไม่จัดระยะ" in out)
    chk.check("วางของหลังหันไปทิศสุดท้าย", tc.ARM_PLACE_XY in arm.calls)
    chk.check("คืนทิศสุดท้าย", heading == 3)

    tc = load()
    tc.AIM_SEQUENCE = [(None, None)]
    driver = _StubDriver()
    payload, arm, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    quiet(payload.pick_up, None)
    arm.calls = []
    heading, _ = quiet(tc.place_on_target, driver, payload, 1, 0.6)
    chk.check("ทิศ None + ระยะ None: ไม่หันไม่จัดระยะ",
              driver.turns == [] and driver.aligns == [])
    chk.check("ทิศ None + ระยะ None: ยังวางของ", tc.ARM_PLACE_XY in arm.calls)
    chk.check("ทิศ None + ระยะ None: คืนทิศเดิม", heading == 1)


def test_place_even_when_aim_fails(chk):
    """จัดระยะไม่เข้าเป้าก็ยังต้องวางของ วางเยื้องดีกว่าไม่ได้วางเลย"""
    chk.section("เล็งเป้า: จัดระยะไม่สำเร็จ")

    tc = load()
    driver = _StubDriver()
    payload, arm, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    quiet(payload.pick_up, None)
    arm.calls = []
    heading, out = quiet(tc.place_on_target, driver, payload, 0, 0.6)
    measured = [s for s in tc.AIM_SEQUENCE if s[1] is not None]
    chk.check("ยังเรียกครบทุกขั้นที่ต้องจัดระยะ",
              len(driver.aligns) == len(measured))
    chk.check("ยังวางของ", tc.ARM_PLACE_XY in arm.calls)
    chk.check("ไม่มีของค้างในมือ", payload.holding is False)
    chk.check("คืนทิศสุดท้ายให้ผู้เรียกไปวาดแผนที่ต่อ",
              heading == tc.AIM_SEQUENCE[-1][0])


def test_turn_keeps_aimed_axis(chk):
    """การหมุนระหว่างเล็งเป้าและก่อนเดินกลับ ต้องไม่ขยับล้อตามแนวเดิน

    ``turn_to`` ปกติจะเรียก ``align_front`` ก่อนหมุนเมื่อ ToF เห็นกำแพงหน้า ซึ่ง
    ดันหุ่นกลับไปที่ ``FRONT_STOP_MM`` แกนที่ถูกดันคือแกนที่กำลังจะกลายเป็นแกน
    ด้านข้างหลังหมุนเสร็จ - ตอนเดินสำรวจนั่นคือสิ่งที่ต้องการ แต่ตอนเล็งเป้ามันคือ
    ค่าที่ขั้นก่อนหน้าเพิ่งจัดไว้ และตอนจะเดินกลับ "กำแพง" ที่ ToF เห็นคือของที่
    เพิ่งวางลงไปเอง ทั้งสองที่จึงต้องส่ง align_first=False
    """
    chk.section("เล็งเป้า: การหมุนไม่รบกวนแกนที่จัดไว้แล้ว")

    tc = load()
    tc.TURN_TIMEOUT_S = 0.05        # ปล่อยให้ยอมแพ้เร็ว ๆ เทสต์นี้ไม่สนใจ yaw
    inside = tc.FRONT_STOP_MM + 100                     # อยู่ในระยะที่ align_front ทำงาน

    road = Corridor(tof_mm=inside)
    quiet(make_driver(tc, road).turn_to, 0, 1, False)
    chk.check("align_first=False: ไม่มีคำสั่งเดินหน้า/ถอยเลย",
              [c for c in road.commands if c[0] != 0.0] == [])
    chk.check("align_first=False: ระยะเดิมไม่ขยับ", road.tof_mm == inside)

    road = Corridor(tof_mm=inside)
    quiet(make_driver(tc, road).turn_to, 0, 1)
    chk.check("ค่าเริ่มต้นยังจัดระยะให้เหมือนเดิม (ตอนเดินสำรวจต้องใช้)",
              [c for c in road.commands if c[0] != 0.0] != [])

    # ระดับ place_on_target: ทุกขั้นที่หัน ต้องสั่งห้ามจัดระยะ
    tc = load()
    tc.AIM_SEQUENCE = [(2, 560), (3, None)]
    driver = _StubDriver()
    payload, _, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    quiet(payload.pick_up, None)
    quiet(tc.place_on_target, driver, payload, 0, 0.6)
    chk.check("ทุกขั้นที่หันตอนเล็งเป้าสั่ง align_first=False",
              driver.align_flags == [False] * len(driver.turns))

    # ระดับ face_way_back: ของที่เพิ่งวางอยู่หน้า ToF ห้ามเดินเข้าหา
    driver = _StubDriver()
    tc.face_way_back(driver, 3, 2)
    chk.check("หันกลับก่อนเดินกลับสั่ง align_first=False",
              driver.align_flags == [False])


def test_aim_config_is_usable(chk):
    """ค่าใน AIM_SEQUENCE ต้องเป็นค่าที่ align_to_wall ทำตามได้จริง

    ตรวจเฉพาะสิ่งที่ยืนยันได้โดยไม่ต้องสมมติเรขาคณิตของหุ่น ค่าที่ต่ำกว่าระยะ
    ต่ำสุดจะถูกด่านกันชนกำแพงปัดตกเงียบ ๆ ส่วนค่าที่เกินพิสัย ToF จะไม่มีวัน
    อ่านได้ หุ่นจะเดินจนหมดงบระยะทางแล้วยอมแพ้
    """
    chk.section("เล็งเป้า: ค่าคอนฟิกใช้ได้จริง")

    tc = load()
    for index, step in enumerate(tc.AIM_SEQUENCE, 1):
        face, target_mm = step
        chk.check("ขั้น {0}: ทิศ {1} ใช้ได้".format(index, face),
                  face is None or face in range(4))
        if target_mm is None:
            chk.check("ขั้น {0}: หันอย่างเดียว ไม่ต้องตรวจระยะ".format(index),
                      True)
            continue
        chk.check("ขั้น {0}: เป้า {1}mm ไม่ต่ำกว่าระยะต่ำสุด {2}mm"
                  .format(index, target_mm, tc.FRONT_STOP_MM),
                  target_mm >= tc.FRONT_STOP_MM)
        chk.check("ขั้น {0}: เป้า {1}mm อยู่ในพิสัย ToF ({2}mm)"
                  .format(index, target_mm, tc.TOF_MAX_VALID_MM),
                  target_mm <= tc.TOF_MAX_VALID_MM)
    chk.check("ทิศของขั้นสุดท้ายกำหนดไว้ชัดเจน หรือรับทิศที่มาถึงโดยตั้งใจ",
              len(tc.AIM_SEQUENCE) > 0)
    chk.check("งบระยะทางต่อขั้นมากกว่าศูนย์", tc.AIM_MAX_MOVE_M > 0)

    # หุ่นเข้าช่องเป้าหมายมาโดย ToF เบรกให้ที่ FRONT_STOP_MM ขั้นแรกที่วัดระยะจึง
    # ต้องขยับเท่ากับส่วนต่างนั้น ถ้างบไม่พอ align_to_wall จะจบด้วย "budget" คือ
    # หยุดกลางทางแล้วหันไปวางทั้งที่ยังไม่ถึงเป้า มีแค่ [WARN] บรรทัดเดียวเตือน
    measured = [s for s in tc.AIM_SEQUENCE if s[1] is not None]
    if measured:
        need_m = abs(measured[0][1] - tc.FRONT_STOP_MM) / 1000.0
        chk.check("งบระยะทาง {0:.2f} m พอสำหรับขั้นแรก (ต้องขยับ {1:.3f} m "
                  "จาก {2}mm ไป {3}mm)"
                  .format(tc.AIM_MAX_MOVE_M, need_m, tc.FRONT_STOP_MM,
                          measured[0][1]),
                  tc.AIM_MAX_MOVE_M >= need_m)
    chk.check("งบระยะทางไม่เกินหนึ่งช่อง (ข้างหลังยืนยันว่าโล่งได้แค่นั้น)",
              tc.AIM_MAX_MOVE_M <= tc.CELL_SIZE_M)


def test_no_backoff_without_ground_behind(chk):
    """ช่องเป้าหมายที่ไม่ได้เดินเข้ามา ต้องส่งเพดาน 0 คือห้ามถอย

    ข้างหลังของช่องที่หุ่นไม่ได้เดินผ่านมาเองคือพื้นที่ที่ยังไม่มีใครยืนยันว่าโล่ง
    ถอยเข้าไปคือถอยชนกำแพงโดยไม่มีเซนเซอร์ตัวไหนมองเห็น
    """
    chk.section("เล็งเป้า: เป้าหมายที่ไม่ได้เดินเข้ามา")

    tc = load()
    tc.AIM_SEQUENCE = []
    tc.GOAL_CELLS = [tuple(tc.START_CELL)]
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    got, _ = quiet(tc.run_search, hub, driver, payload)
    chk.check("ถึงเป้าหมายโดยไม่ต้องเดิน", got is True)
    chk.check("ยังสั่งถอย (ให้เมธอดตัดสินเอง)", len(driver.backoffs) == 1)
    chk.check("เพดาน = 0 คือห้ามถอย", driver.backoffs[0][2] == 0.0)

    tc = load()
    tc.AIM_SEQUENCE = []
    tc.GOAL_CELLS = [tuple(tc.START_CELL)]
    tc.PICK_HEADING = 1                                 # หันก่อนหยิบที่ช่องเดิม
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    quiet(tc.run_search, hub, driver, payload)
    chk.check("หันแล้วก็ยังห้ามถอย", driver.backoffs[0][2] == 0.0)


def run(chk=None):
    """รันเทสต์ทั้งไฟล์

    Returns:
        bool: True เมื่อผ่านหมด
    """
    own = chk is None
    chk = chk or Checker()
    print("=" * 58)
    print("  ทดสอบการจัดระยะเทียบกำแพงและการเล็งเป้า")
    print("=" * 58)
    test_no_move_cases(chk)
    test_backs_off_until_clear(chk)
    test_stops_at_limit(chk)
    test_sensor_guards(chk)
    test_sharp_keeps_it_centred(chk)
    test_align_both_directions(chk)
    test_align_guards(chk)
    test_backup_behaviour_unchanged(chk)
    test_aim_sequence_wiring(chk)
    test_turn_only_step(chk)
    test_place_even_when_aim_fails(chk)
    test_turn_keeps_aimed_axis(chk)
    test_aim_config_is_usable(chk)
    test_fallback_without_aim(chk)
    test_no_backoff_without_ground_behind(chk)
    return chk.report() if own else True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)

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
def expected_aims(tc):
    """ค่าที่ ``place_on_target`` ควรส่งให้ ``align_to_wall`` ของทุกขั้นที่จัดระยะ

    คำนวณด้วย ``aim_tof_target_mm`` ตัวเดียวกับที่โค้ดจริงใช้ เทสต์ชุดนี้จึงตรวจ
    "การต่อสาย" ว่าค่าถูกคำนวณและส่งต่อครบถ้วน ส่วนตัวเลขที่คำนวณออกมาถูกหรือไม่
    ถูกตรวจแยกใน ``test_aim_target_math``

    Returns:
        list: [(target_mm, heading, center), ...] เรียงตามลำดับที่ควรถูกเรียก
    """
    total = len(tc.AIM_SEQUENCE)
    out = []
    for index, step in enumerate(tc.AIM_SEQUENCE, 1):
        target = tc.aim_tof_target_mm(step, step.face, index == total)
        if target is not None:
            out.append((target, step.face, index == 1))
    return out


def test_aim_sequence_wiring(chk):
    """run_search ต้องเดินตาม AIM_SEQUENCE ครบทุกขั้นก่อนวางของ"""
    chk.section("เล็งเป้า: run_search เดินตามลำดับ")

    tc = load()
    expected = expected_aims(tc)
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("ถึงเป้าหมาย", got is True)
    chk.check("หันครบทุกขั้นที่กำหนดทิศไว้",
              len(driver.turns) >= len([s for s in tc.AIM_SEQUENCE
                                        if s.face is not None]))
    chk.check("จัดระยะเฉพาะขั้นที่กำหนดระยะไว้ ({0} จาก {1} ขั้น)"
              .format(len(expected), len(tc.AIM_SEQUENCE)),
              len(driver.aligns) == len(expected))
    chk.check("ส่งค่าเป้าที่คำนวณได้ไปให้",
              [a[0] for a in driver.aligns] == [e[0] for e in expected])
    chk.check("ส่งงบระยะทางตามที่ตั้งไว้",
              all(a[2] == tc.AIM_MAX_MOVE_M for a in driver.aligns))
    chk.check("จัดระยะในทิศที่กำหนดไว้",
              [a[1] for a in driver.aligns] == [e[1] for e in expected])
    chk.check("ประคองกลางช่องเฉพาะขั้นแรก",
              [a[3] for a in driver.aligns] == [e[2] for e in expected])
    chk.check("เล็งก่อนวางของ", out.index("[AIM]") < out.index("จุดวางของ"))
    chk.check("ขั้นสุดท้ายหันทิศที่จะยื่นแขนวาง",
              "ขั้นที่ {0}/{0} หัน{1}".format(
                  len(tc.AIM_SEQUENCE),
                  tc.DIR_NAMES[tc.AIM_SEQUENCE[-1].face]) in out)
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

    def align_to_wall(self, target_mm, heading, budget_m, floor_mm=None,
                      center=True, start_tof=None):
        self.aligns.append((target_mm, heading, budget_m, center))
        return target_mm, 0.0, self.reason


def test_turn_only_step(chk):
    """ขั้นที่ระยะเป็น None ต้องหันแล้วผ่านไป ไม่แตะล้อ

    ใช้เมื่อวัดแล้วพบว่าตำแหน่งตามแกนนั้นตรงเป้าอยู่แล้วตั้งแต่ตอนเดินเข้าช่อง
    การจัดระยะซ้ำมีแต่จะดันหุ่นออกจากจุดที่ถูกอยู่แล้ว
    """
    chk.section("เล็งเป้า: ขั้นที่หันอย่างเดียว")

    tc = load()
    tc.AIM_SEQUENCE = [
        tc.AimStep(face=tc.SOUTH, ref=tc.NORTH, target_mm=400, span_mm=1200),
        tc.AimStep(face=tc.WEST, ref=tc.WEST, target_mm=None),
    ]
    first = tc.aim_tof_target_mm(tc.AIM_SEQUENCE[0], tc.SOUTH, False)
    driver = _StubDriver()
    payload, arm, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    quiet(payload.pick_up, None)
    arm.calls = []
    heading, out = quiet(tc.place_on_target, driver, payload, 0, 0.6)

    chk.check("หันครบสองขั้น", [t[1] for t in driver.turns] == [2, 3])
    chk.check("จัดระยะแค่ขั้นแรก", [a[0] for a in driver.aligns] == [first])
    chk.check("บอกว่าขั้นที่สองไม่จัดระยะ", "แล้วไม่จัดระยะ" in out)
    chk.check("วางของหลังหันไปทิศสุดท้าย", tc.ARM_PLACE_XY in arm.calls)
    chk.check("คืนทิศสุดท้าย", heading == 3)

    tc = load()
    tc.AIM_SEQUENCE = [tc.AimStep(face=None, ref=None, target_mm=None)]
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
    chk.check("ยังเรียกครบทุกขั้นที่ต้องจัดระยะ",
              len(driver.aligns) == len(expected_aims(tc)))
    chk.check("ยังวางของ", tc.ARM_PLACE_XY in arm.calls)
    chk.check("ไม่มีของค้างในมือ", payload.holding is False)
    chk.check("คืนทิศสุดท้ายให้ผู้เรียกไปวาดแผนที่ต่อ",
              heading == tc.AIM_SEQUENCE[-1].face)


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
    tc.AIM_SEQUENCE = [
        tc.AimStep(face=tc.SOUTH, ref=tc.NORTH, target_mm=400, span_mm=1200),
        tc.AimStep(face=tc.WEST, ref=tc.WEST, target_mm=None),
    ]
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


def _raises(fn, *args):
    """bool: เรียกแล้วโยน ValueError ออกมาหรือไม่"""
    try:
        fn(*args)
    except ValueError:
        return True
    return False


def test_tof_forward_offset(chk):
    """ระยะที่หัว ToF ล้ำหน้าจุดหมุน ต้องอนุมานจาก FRONT_STOP_MM ได้เอง"""
    chk.section("เล็งเป้า: ระยะที่หัว ToF ล้ำหน้าจุดหมุน")

    tc = load()
    half_cell = tc.CELL_SIZE_M * 1000.0 / 2.0
    chk.check("อนุมานได้ = ครึ่งช่อง - FRONT_STOP_MM ({0:.0f} mm)"
              .format(half_cell - tc.FRONT_STOP_MM),
              abs(tc.tof_forward_offset_mm()
                  - (half_cell - tc.FRONT_STOP_MM)) < 1e-9)

    tc = load()
    tc.TOF_FORWARD_OFFSET_MM = 180
    chk.check("ตั้งทับได้เมื่อวัดเอง", tc.tof_forward_offset_mm() == 180.0)


def test_aim_target_math(chk):
    """aim_tof_target_mm ต้องแปลงโจทย์เป็นค่า ToF ได้ตรงตามเรขาคณิต

    เดินสมการจากกำแพงที่หันใส่เข้ามาหาหุ่น แยกตรวจทีละตัวแปร เพื่อให้เวลาเลขไม่
    ตรงหน้าสนามจะรู้ได้ทันทีว่าตัวไหนผิด
    """
    chk.section("เล็งเป้า: คณิตศาสตร์ของค่าเป้า")

    tc = load()
    d_tof = tc.tof_forward_offset_mm()

    # หันใส่กำแพงเดียวกับที่เป้านับระยะจาก และไม่ใช่ขั้นที่ยื่นแขนวาง
    step = tc.AimStep(face=tc.WEST, ref=tc.WEST, target_mm=400)
    chk.check("กำแพงเดียวกัน ไม่ได้วาง: เป้า - ระยะล้ำ ({0:.0f} mm)"
              .format(400 - d_tof),
              tc.aim_tof_target_mm(step, tc.WEST, False) == 400 - d_tof)

    # ขั้นเดียวกันแต่เป็นขั้นที่ยื่นแขนวาง ต้องบวกระยะเอื้อมเข้าไป
    chk.check("กำแพงเดียวกัน ขั้นที่วาง: บวกระยะเอื้อมแขน ({0:.0f} mm)"
              .format(400 + tc.ARM_REACH_MM - d_tof),
              tc.aim_tof_target_mm(step, tc.WEST, True)
              == 400 + tc.ARM_REACH_MM - d_tof)

    # เป้านับจากกำแพงตรงข้ามกับที่หันใส่ ต้องพลิกด้านด้วย span_mm
    step = tc.AimStep(face=tc.SOUTH, ref=tc.NORTH, target_mm=400, span_mm=1200)
    chk.check("กำแพงตรงข้าม: พลิกด้วย span ({0:.0f} mm)".format(800 - d_tof),
              tc.aim_tof_target_mm(step, tc.SOUTH, False) == 800 - d_tof)

    step = tc.AimStep(face=tc.SOUTH, ref=tc.NORTH, target_mm=400,
                      span_mm=1200, offset_mm=-35)
    chk.check("offset_mm บวกเข้าผลลัพธ์ตรง ๆ",
              tc.aim_tof_target_mm(step, tc.SOUTH, False) == 800 - d_tof - 35)

    # ทิศจริงเป็นตัวตัดสิน ไม่ใช่ step.face เพราะ face เป็น None ได้
    step = tc.AimStep(face=None, ref=tc.EAST, target_mm=250)
    chk.check("face=None: ใช้ทิศจริงที่ส่งเข้ามาแทน",
              tc.aim_tof_target_mm(step, tc.EAST, False) == 250 - d_tof)

    chk.check("target_mm=None: ไม่ต้องจัดระยะ",
              tc.aim_tof_target_mm(
                  tc.AimStep(face=tc.WEST, ref=tc.WEST, target_mm=None),
                  tc.WEST, False) is None)

    chk.check("กำแพงอ้างอิงคนละแกนกับทิศที่หัน: ต้องโวยวาย ไม่ใช่คำนวณมั่ว",
              _raises(tc.aim_tof_target_mm,
                      tc.AimStep(face=tc.NORTH, ref=tc.EAST, target_mm=400),
                      tc.NORTH, False))
    chk.check("กำแพงตรงข้ามแต่ไม่บอก span: ต้องโวยวาย",
              _raises(tc.aim_tof_target_mm,
                      tc.AimStep(face=tc.SOUTH, ref=tc.NORTH, target_mm=400),
                      tc.SOUTH, False))

    tc = load()
    tc.ARM_REACH_MM = None
    chk.check("ขั้นที่วางแต่ไม่รู้ระยะเอื้อมแขน: ต้องโวยวาย",
              _raises(tc.aim_tof_target_mm,
                      tc.AimStep(face=tc.WEST, ref=tc.WEST, target_mm=400),
                      tc.WEST, True))


def test_aim_matches_previous_hardcoded_values(chk):
    """ค่าที่คำนวณได้ต้องเท่ากับตัวเลขที่เคยฮาร์ดโค้ดไว้

    การเปลี่ยนมาคำนวณเป็นการรื้อ "วิธีได้มาซึ่งตัวเลข" ไม่ใช่การย้ายจุดที่หุ่นไป
    จอด ถ้าเทสต์นี้แดงแปลว่าหุ่นจะไปจอดคนละที่กับเดิม ซึ่งยอมรับได้ก็ต่อเมื่อ
    ตั้งใจแก้ ARM_REACH_MM หรือ offset_mm เท่านั้น
    """
    chk.section("เล็งเป้า: ค่าที่คำนวณตรงกับที่ฮาร์ดโค้ดไว้เดิม")

    tc = load()
    got = [t for t, _, _ in expected_aims(tc)]
    chk.check("ขั้นที่ 1 ยังได้ 560mm เท่าเดิม (ได้ {0})".format(got[0]),
              abs(got[0] - 560) < 1e-9)
    chk.check("ขั้นที่ 2 ยังได้ 650mm เท่าเดิม (ได้ {0})".format(got[1]),
              abs(got[1] - 650) < 1e-9)


def test_predict_tof(chk):
    """แผนที่ต้องทำนายระยะ ToF ได้ และต้องยอมบอกว่า "ไม่รู้" เมื่อไม่รู้จริง"""
    chk.section("เล็งเป้า: แผนที่ทำนายระยะ ToF")

    tc = load()
    cell = tc.CELL_SIZE_M * 1000.0
    maze = tc.Maze(5, 6, [(0, 0)])

    # _add_borders ใส่กำแพงขอบสนามให้แล้ว ช่องมุมจึงทำนายได้ทันที
    chk.check("กำแพงติดช่องเลย = FRONT_STOP_MM ({0}mm)".format(tc.FRONT_STOP_MM),
              maze.predict_tof(0, 0, tc.SOUTH) == tc.FRONT_STOP_MM)
    chk.check("ยังไม่เคยตรวจด้านนั้น: ไม่เดา",
              maze.predict_tof(0, 0, tc.NORTH) is None)

    # เปิดทางให้รู้ทีละด้าน แล้วระยะต้องเพิ่มทีละหนึ่งช่อง
    maze.set_wall(0, 0, tc.NORTH, False)
    chk.check("รู้ว่าโล่งหนึ่งช่องแต่ยังไม่รู้ด้านถัดไป: ยังไม่เดา",
              maze.predict_tof(0, 0, tc.NORTH) is None)
    maze.set_wall(0, 1, tc.NORTH, True)
    chk.check("กำแพงห่างหนึ่งช่อง = FRONT_STOP_MM + 1 ช่อง ({0:.0f}mm)"
              .format(tc.FRONT_STOP_MM + cell),
              maze.predict_tof(0, 0, tc.NORTH) == tc.FRONT_STOP_MM + cell)

    maze = tc.Maze(5, 6, [(0, 0)])
    maze.set_wall(0, 0, tc.NORTH, False)
    maze.set_wall(0, 1, tc.NORTH, False)
    maze.set_wall(0, 2, tc.NORTH, True)
    chk.check("กำแพงห่างสองช่อง = FRONT_STOP_MM + 2 ช่อง ({0:.0f}mm)"
              .format(tc.FRONT_STOP_MM + 2 * cell),
              maze.predict_tof(0, 0, tc.NORTH) == tc.FRONT_STOP_MM + 2 * cell)

    # ของที่วางเองบังอยู่ ทำนายระยะไม่ได้ เพราะไม่รู้ขนาดและตำแหน่งของ
    maze = tc.Maze(5, 6, [(0, 0)])
    maze.mark_object(0, 0, tc.SOUTH)
    chk.check("มีของที่วางเองขวางอยู่: ไม่ทำนาย",
              maze.predict_tof(0, 0, tc.SOUTH) is None)

    # ต้องไม่แตะแผนที่เลย เพราะกำแพงที่เขียนลงไปแล้วลบไม่ได้
    maze = tc.Maze(5, 6, [(0, 0)])
    before = ([row[:] for row in maze.walls], [row[:] for row in maze.known])
    maze.predict_tof(2, 2, tc.NORTH)
    maze.predict_tof(0, 0, tc.SOUTH)
    chk.check("อ่านอย่างเดียว ไม่เขียนอะไรลงแผนที่",
              (maze.walls, maze.known) == before)


def test_aim_reading_sanity(chk):
    """ด่านตรวจต้องปัดค่าที่ "ดูสมเหตุสมผลแต่ผิด" ทิ้ง ก่อนหุ่นจะขยับตาม"""
    chk.section("เล็งเป้า: ด่านตรวจค่า ToF ก่อนขยับ")

    tc = load()
    window = tc.tof_sanity_window_mm()
    chk.check("หน้าต่างเริ่มต้น = ครึ่งช่อง ({0:.0f}mm)".format(window),
              window == tc.CELL_SIZE_M * 1000.0 / 2.0)

    ok, _ = tc.aim_reading_is_sane(560, 560)
    chk.check("ตรงเป้าพอดี: ผ่าน", ok is True)
    ok, _ = tc.aim_reading_is_sane(560 + window - 1, 560)
    chk.check("ห่างเป้าเกือบเต็มหน้าต่าง: ยังผ่าน", ok is True)

    # เคสจริงที่อันตราย: ToF มองทะลุประตูไปโดนกำแพงห้องถัดไป
    ok, why = tc.aim_reading_is_sane(560 + window + 1, 560)
    chk.check("หลุดหน้าต่าง: ไม่ผ่าน", ok is False)
    chk.check("บอกด้วยว่าห่างเป้าไปเท่าไร", "ห่างจากเป้า" in why)

    # ด่านที่สอง: ผ่านด่านเป้าแต่ไม่ตรงกับแผนที่ = หุ่นไม่ได้อยู่ช่องที่คิด
    ok, _ = tc.aim_reading_is_sane(560, 560, predicted_mm=565)
    chk.check("แผนที่เห็นตรงกัน: ผ่าน", ok is True)
    ok, why = tc.aim_reading_is_sane(560, 560, predicted_mm=560 + window + 1)
    chk.check("แผนที่ว่าคนละเรื่อง: ไม่ผ่าน", ok is False)
    chk.check("บอกด้วยว่าแผนที่ว่าเท่าไร", "แผนที่ว่าควรได้" in why)

    ok, _ = tc.aim_reading_is_sane(560, 560, predicted_mm=None)
    chk.check("แผนที่ไม่รู้: ข้ามด่านที่สอง ไม่ใช่ตกด่าน", ok is True)


def test_aim_skips_move_on_bad_reading(chk):
    """ค่าที่ไม่ผ่านด่านตรวจ ต้องไม่ถูกเอาไปขยับ แต่ยังต้องวางของ"""
    chk.section("เล็งเป้า: ค่าที่เชื่อไม่ได้ ไม่ขยับตาม")

    class _Hub(object):
        """hub ที่คืนค่าซึ่งดูสมเหตุสมผลแต่ห่างเป้าเกินครึ่งช่อง"""

        def __init__(self, mm):
            self.mm = mm

        def read_tof_settled(self, samples=None):
            return self.mm

    tc = load()
    driver = _StubDriver()
    payload, arm, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    quiet(payload.pick_up, None)
    arm.calls = []
    # ต้องหลุดหน้าต่างของ "ทุก" ขั้น เพราะด่านตรวจทำงานแยกกันทีละขั้น
    far = max(t for t, _, _ in expected_aims(tc)) + tc.tof_sanity_window_mm() + 50
    _, out = quiet(tc.place_on_target, driver, payload, 0, 0.6,
                   hub=_Hub(far))
    chk.check("ไม่สั่งจัดระยะเลยสักขั้น", driver.aligns == [])
    chk.check("เตือนว่าไม่ผ่านด่านตรวจ", "ไม่ผ่านด่านตรวจ" in out)
    chk.check("ยังวางของ (วางเยื้องดีกว่าไม่วาง)", tc.ARM_PLACE_XY in arm.calls)
    chk.check("ไม่มีของค้างในมือ", payload.holding is False)

    # ค่าที่ผ่านด่าน ต้องถูกส่งต่อให้ align_to_wall ใช้เลย ไม่ต้องวัดซ้ำ
    tc = load()
    driver = _StubDriver()
    payload, arm, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    quiet(payload.pick_up, None)
    good = expected_aims(tc)[0][0]
    quiet(tc.place_on_target, driver, payload, 0, 0.6, hub=_Hub(good))
    chk.check("ผ่านด่าน: ยังสั่งจัดระยะตามปกติ", len(driver.aligns) >= 1)


def test_walls_settled_reports_distance(chk):
    """read_walls_settled ต้องคืนระยะมาด้วย จากตัวอย่างชุดเดียวกับที่โหวตกำแพง"""
    chk.section("เดินสำรวจ: อ่านกำแพงแล้วได้ระยะมาด้วย")

    tc = load()
    tc.WALL_VOTE_INTERVAL = 0.0
    road = Corridor(tof_mm=740)
    road.wall_mm = tc.front_wall_threshold_mm()
    hub_result = tc.SensorHub.read_walls_settled(road, samples=5)
    chk.check("คืนสี่ค่า", len(hub_result) == 4)
    chk.check("ค่าที่สี่คือระยะที่วัดได้ ({0})".format(hub_result[3]),
              hub_result[3] == 740.0)

    road = Corridor(tof_mm=None)
    road.wall_mm = tc.front_wall_threshold_mm()
    chk.check("ไม่เห็นกำแพงในระยะวัด: ระยะเป็น None",
              tc.SensorHub.read_walls_settled(road, samples=5)[3] is None)


class _LyingHub(object):
    """hub ที่บอกกำแพงตามความจริง แต่โกหกเรื่องระยะ

    จำลองอาการที่ด่านเทียบแผนที่มีไว้จับ คือหุ่นนับช่องพลาดจนระยะที่วัดได้ไม่
    เข้ากับที่แผนที่ทำนายไว้ ต่างกับ ``TruthHub`` ตรงที่ตัวนั้นคืน None คือ
    "วัดระยะไม่ได้" ซึ่งทำให้ด่านเงียบไปเฉย ๆ

    Args:
        inner (TruthHub): ตัวจริงที่รู้กำแพง
        mm (float): ระยะที่จะรายงานทุกครั้ง เลือกให้ไกลเกินกว่าที่แผนที่ทำนาย
            ได้ในสนามขนาดนี้ เพื่อให้ขัดกับทุกคำทำนายที่เป็นไปได้
    """

    def __init__(self, inner, mm):
        self.inner = inner
        self.mm = mm

    def read_walls_settled(self, samples=None):
        front, left, right, _ = self.inner.read_walls_settled(samples)
        return front, left, right, self.mm

    def read_tof_settled(self, samples=None):
        return self.mm

    def snapshot(self):
        return self.inner.snapshot()


def test_observation_trust(chk):
    """ตัดสินใจว่าจะบันทึกลงแผนที่ไหม จากระยะที่วัดได้เทียบกับที่ทำนายไว้"""
    chk.section("เดินสำรวจ: เชื่อค่าที่อ่านได้ไหม")

    tc = load()
    cell = tc.CELL_SIZE_M * 1000.0
    window = tc.tof_sanity_window_mm()
    maze = tc.Maze(5, 6, [(0, 0)])

    # (0,0) รู้ด้านใต้กับตะวันตกตั้งแต่ต้น เพราะ _add_borders ใส่ขอบสนามให้แล้ว
    ok, _ = tc.observation_is_trusted(maze, 0, 0, tc.SOUTH, tc.FRONT_STOP_MM)
    chk.check("ตรงกับที่ทำนาย: เชื่อ", ok is True)
    ok, _ = tc.observation_is_trusted(maze, 0, 0, tc.SOUTH,
                                      tc.FRONT_STOP_MM + window - 1)
    chk.check("ต่างไม่เกินหน้าต่าง: ยังเชื่อ", ok is True)

    # เคสจริงที่ด่านนี้มีไว้จับ: หุ่นคิดว่าอยู่ (0,0) แต่จริง ๆ อยู่ไกลออกไป
    # จนระยะถึงขอบสนามใต้ไม่มีทางเป็นค่านี้ได้
    ok, why = tc.observation_is_trusted(maze, 0, 0, tc.SOUTH,
                                        tc.FRONT_STOP_MM + cell)
    chk.check("ต่างเกินหน้าต่าง: ไม่เชื่อ", ok is False)
    chk.check("บอกทั้งค่าที่วัดได้และค่าที่ทำนาย",
              "วัดได้" in why and "แผนที่ว่าควรได้" in why)

    # ไม่มีข้อมูลพอ = เชื่อ ไม่งั้นช่องที่เพิ่งมาถึงครั้งแรกจะไม่ถูกบันทึกเลย
    ok, _ = tc.observation_is_trusted(maze, 0, 0, tc.NORTH, 9999)
    chk.check("แผนที่ยังไม่รู้ด้านนั้น: เชื่อไว้ก่อน", ok is True)
    ok, _ = tc.observation_is_trusted(maze, 0, 0, tc.SOUTH, None)
    chk.check("วัดระยะไม่ได้: เชื่อไว้ก่อน", ok is True)


def test_map_check_blocks_bad_observation(chk):
    """run_search ต้องต่อสายด่านนี้ไว้จริง และยังเดินจนจบได้

    กำแพงที่เขียนลงไปแล้วลบไม่ได้ (ดู Maze.set_wall) การเขียนตอนที่ยังไม่แน่ใจ
    ว่าหุ่นอยู่ช่องไหนจริง จึงแย่กว่าการไม่เขียนอะไรเลย
    """
    chk.section("เดินสำรวจ: ต่อสายด่านเทียบแผนที่")

    tc = load()
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    honest, out_honest = quiet(tc.run_search, hub, driver, payload)
    chk.check("หุ่นที่วัดระยะไม่ได้: ยังถึงเป้าหมาย", honest is True)
    chk.check("ไม่มีคำเตือนเรื่องแผนที่", "แต่แผนที่ว่าควรได้" not in out_honest)

    tc = load()
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    liar = _LyingHub(hub, 3000.0)
    got, out_lying = quiet(tc.run_search, liar, driver, payload)
    chk.check("หุ่นที่โกหกระยะ: ถูกจับได้", "แต่แผนที่ว่าควรได้" in out_lying)
    chk.check("บอกด้วยว่าจะไม่บันทึกลงแผนที่",
              "ไม่บันทึกกำแพงรอบนี้ลงแผนที่" in out_lying)
    chk.check("ด่านนี้ไม่ทำให้หุ่นเดินไม่จบ", got is True)


def test_aim_config_is_usable(chk):
    """ค่าใน AIM_SEQUENCE ต้องเป็นค่าที่ align_to_wall ทำตามได้จริง

    ตรวจเฉพาะสิ่งที่ยืนยันได้โดยไม่ต้องสมมติเรขาคณิตของหุ่น ค่าที่ต่ำกว่าระยะ
    ต่ำสุดจะถูกด่านกันชนกำแพงปัดตกเงียบ ๆ ส่วนค่าที่เกินพิสัย ToF จะไม่มีวัน
    อ่านได้ หุ่นจะเดินจนหมดงบระยะทางแล้วยอมแพ้
    """
    chk.section("เล็งเป้า: ค่าคอนฟิกใช้ได้จริง")

    tc = load()
    total = len(tc.AIM_SEQUENCE)
    for index, step in enumerate(tc.AIM_SEQUENCE, 1):
        chk.check("ขั้น {0}: ทิศ {1} ใช้ได้".format(index, step.face),
                  step.face is None or step.face in range(4))
        target_mm = tc.aim_tof_target_mm(step, step.face, index == total)
        if target_mm is None:
            chk.check("ขั้น {0}: หันอย่างเดียว ไม่ต้องตรวจระยะ".format(index),
                      True)
            continue
        chk.check("ขั้น {0}: เป้า {1:.0f}mm ไม่ต่ำกว่าระยะต่ำสุด {2}mm"
                  .format(index, target_mm, tc.FRONT_STOP_MM),
                  target_mm >= tc.FRONT_STOP_MM)
        chk.check("ขั้น {0}: เป้า {1:.0f}mm อยู่ในพิสัย ToF ({2}mm)"
                  .format(index, target_mm, tc.TOF_MAX_VALID_MM),
                  target_mm <= tc.TOF_MAX_VALID_MM)
    chk.check("ทิศของขั้นสุดท้ายกำหนดไว้ชัดเจน หรือรับทิศที่มาถึงโดยตั้งใจ",
              len(tc.AIM_SEQUENCE) > 0)
    chk.check("งบระยะทางต่อขั้นมากกว่าศูนย์", tc.AIM_MAX_MOVE_M > 0)

    # หุ่นเข้าช่องเป้าหมายมาโดย ToF เบรกให้ที่ FRONT_STOP_MM ขั้นแรกที่วัดระยะจึง
    # ต้องขยับเท่ากับส่วนต่างนั้น ถ้างบไม่พอ align_to_wall จะจบด้วย "budget" คือ
    # หยุดกลางทางแล้วหันไปวางทั้งที่ยังไม่ถึงเป้า มีแค่ [WARN] บรรทัดเดียวเตือน
    measured = [t for t, _, _ in expected_aims(tc)]
    if measured:
        need_m = abs(measured[0] - tc.FRONT_STOP_MM) / 1000.0
        chk.check("งบระยะทาง {0:.2f} m พอสำหรับขั้นแรก (ต้องขยับ {1:.3f} m "
                  "จาก {2}mm ไป {3:.0f}mm)"
                  .format(tc.AIM_MAX_MOVE_M, need_m, tc.FRONT_STOP_MM,
                          measured[0]),
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
    test_tof_forward_offset(chk)
    test_aim_target_math(chk)
    test_aim_matches_previous_hardcoded_values(chk)
    test_predict_tof(chk)
    test_aim_reading_sanity(chk)
    test_aim_skips_move_on_bad_reading(chk)
    test_walls_settled_reports_distance(chk)
    test_observation_trust(chk)
    test_map_check_blocks_bad_observation(chk)
    test_aim_config_is_usable(chk)
    test_fallback_without_aim(chk)
    test_no_backoff_without_ground_behind(chk)
    return chk.report() if own else True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)

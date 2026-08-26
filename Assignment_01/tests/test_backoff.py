# -*- coding: utf-8 -*-
"""ทดสอบการถอยห่างกำแพงก่อนวางของที่ช่องเป้าหมาย

แยกเป็นสองระดับ
- ระดับ Driver: ใช้ ``Corridor`` ที่จำลองตำแหน่งเป็นเมตรและ ToF ที่ขยับตามจริง
  จึงตรวจได้ว่าถอยไปเท่าไรและหยุดด้วยเหตุผลอะไร
- ระดับ run_search: ตรวจว่าถูกเรียกตอนไหน ด้วยเพดานเท่าไร และเรียกก่อนวางของ
"""
import sys

from fakes import Checker, Corridor, load, make_driver, make_world, quiet


def test_no_move_cases(chk):
    """สามกรณีที่ต้องไม่ขยับล้อเลยแม้แต่คำสั่งเดียว"""
    chk.section("กรณีที่ไม่ต้องถอย")

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
    chk.check("ห่างพออยู่แล้ว: รายงานระยะที่วัดได้",
              "{0}mm อยู่แล้ว".format(target + 100) in out)

    road = Corridor(tof_mm=tc.FRONT_STOP_MM)
    got, out = quiet(make_driver(tc, road).back_off_from_wall, target, 0, 0.0)
    chk.check("ไม่มีที่ถอย: reason = no_room", got[2] == "no_room")
    chk.check("ไม่มีที่ถอย: ไม่ขยับล้อ", road.moves == [])
    chk.check("ไม่มีที่ถอย: บอกว่าไม่มีพื้นที่ข้างหลัง",
              "ไม่มีพื้นที่ข้างหลัง" in out)


def test_backs_off_until_clear(chk):
    """ถอยจนได้ระยะที่ตั้งไว้ แล้วหยุดทันที ไม่ถอยเกิน"""
    chk.section("ถอยจนได้ระยะที่ตั้งไว้")

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
    chk.section("ถอยชนเพดานก่อนได้ระยะ")

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
    """เซนเซอร์ขาดการอัปเดตต้องหยุดถอยทันที ไม่ใช่ถอยต่อแบบตาบอด"""
    chk.section("เซนเซอร์ขาดการอัปเดตระหว่างถอย")

    tc = load()
    road = Corridor(tof_mm=tc.FRONT_STOP_MM, stale=True)
    got, out = quiet(make_driver(tc, road).back_off_from_wall, 400, 0, 0.6)
    chk.check("reason บอกว่าเซนเซอร์ค้าง", got[2].startswith("sensor_stale"))
    chk.check("ไม่ขยับล้อเลย", road.moves == [])
    chk.check("จบด้วยคำสั่งหยุด", road.commands[-1] == (0, 0, 0))


def test_sharp_keeps_it_centred(chk):
    """Sharp ต้องประคองกลางช่องระหว่างถอย ไม่ปล่อยให้เบียดกำแพงข้าง

    Sharp บอกระยะเป็นมิลลิเมตรไม่ได้ สิ่งเดียวที่มันทำได้ระหว่างถอยคือดันหุ่น
    ออกจากข้างที่ชิดกว่าจุดอ้างอิง เทสต์นี้จึงตรวจ "ทิศของการดัน" ไม่ใช่ระยะ
    """
    chk.section("Sharp ประคองกลางช่องระหว่างถอย")

    tc = load()
    # ค่า ADC ที่แปลว่ามีกำแพงซ้ายและชิดกว่าจุดกึ่งกลางช่อง
    near_left = max(tc.SHARP_LEFT_WALL_ADC) + 40
    road = Corridor(tof_mm=tc.FRONT_STOP_MM, adc=(near_left, 0))
    quiet(make_driver(tc, road).back_off_from_wall, 400, 0, 0.6)
    strafes = [c[1] for c in road.moves]
    chk.check("มีการ strafe ระหว่างถอย", any(s != 0.0 for s in strafes))
    chk.check("ดันออกจากกำแพงซ้ายทุกครั้ง", all(s > 0 for s in strafes if s))
    chk.check("ไม่เกินเพดาน strafe",
              all(abs(s) <= tc.MAX_STRAFE + 1e-9 for s in strafes))

    road = Corridor(tof_mm=tc.FRONT_STOP_MM, adc=(0, 0))
    quiet(make_driver(tc, road).back_off_from_wall, 400, 0, 0.6)
    chk.check("ไม่มีกำแพงข้าง: ไม่ strafe มั่ว",
              all(c[1] == 0.0 for c in road.moves))


def test_run_search_wiring(chk):
    """run_search ต้องสั่งถอยก่อนวางของ ด้วยเพดานที่ปลอดภัย"""
    chk.section("run_search ต่อสายการถอยเข้ากับการวางของ")

    tc = load()
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("ถึงเป้าหมาย", got is True)
    chk.check("สั่งถอยหนึ่งครั้ง", len(driver.backoffs) == 1)
    clearance, _, limit = driver.backoffs[0]
    chk.check("ส่งค่าที่ตั้งไว้ไปให้", clearance == tc.GOAL_WALL_CLEARANCE_MM)
    chk.check("เพดาน = ระยะที่เพิ่งเดินเข้าช่องมา ({0:.2f} m)".format(limit),
              abs(limit - tc.CELL_SIZE_M) < 1e-9)
    chk.check("ถอยก่อนวางของ", out.index("[BACKOFF]") < out.index("จุดวางของ"))

    tc = load()
    tc.GOAL_WALL_CLEARANCE_MM = None
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("None: ยังถึงเป้าหมาย", got is True)
    chk.check("None: ไม่สั่งถอยเลย", driver.backoffs == [])
    chk.check("None: ยังวางของตามปกติ", "จุดวางของ" in out)


def test_no_backoff_without_ground_behind(chk):
    """ช่องเป้าหมายที่ไม่ได้เดินเข้ามา ต้องส่งเพดาน 0 คือห้ามถอย

    ข้างหลังของช่องที่หุ่นไม่ได้เดินผ่านมาเองคือพื้นที่ที่ยังไม่มีใครยืนยันว่าโล่ง
    ถอยเข้าไปคือถอยชนกำแพงโดยไม่มีเซนเซอร์ตัวไหนมองเห็น
    """
    chk.section("เป้าหมายที่ไม่ได้เดินเข้ามา")

    tc = load()
    tc.GOAL_CELLS = [tuple(tc.START_CELL)]
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (220, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("ถึงเป้าหมายโดยไม่ต้องเดิน", got is True)
    chk.check("ยังสั่งถอย (ให้เมธอดตัดสินเอง)", len(driver.backoffs) == 1)
    chk.check("เพดาน = 0 คือห้ามถอย", driver.backoffs[0][2] == 0.0)

    tc = load()
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
    print("  ทดสอบการถอยห่างกำแพงก่อนวางของ")
    print("=" * 58)
    test_no_move_cases(chk)
    test_backs_off_until_clear(chk)
    test_stops_at_limit(chk)
    test_sensor_guards(chk)
    test_sharp_keeps_it_centred(chk)
    test_run_search_wiring(chk)
    test_no_backoff_without_ground_behind(chk)
    return chk.report() if own else True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)

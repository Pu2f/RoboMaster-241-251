# -*- coding: utf-8 -*-
"""ทดสอบ run_search และ run_sim ครบทุกคอนฟิกการหยิบของ

run_search คือโค้ดที่หุ่นจริงรัน ส่วน run_sim เป็นตรรกะคนละสำเนา จึงทดสอบทั้งคู่
โดยรัน run_search บนหุ่นปลอมที่เดินบนเขาวงกตความจริงจนจบรอบจริง ๆ
"""
import inspect
import sys

from fakes import Checker, load, make_world, quiet

#: list: กำแพงที่ขังหุ่นไว้กับช่อง (0, 0) และ (0, 1) แยกขาดจากช่องเป้าหมาย
#:
#: หุ่นหยิบของที่จุดเริ่มได้ปกติ เดินขึ้นเหนือหนึ่งช่อง แล้วพอ observe() ที่
#: (0, 1) เห็นกำแพงครบทั้งสองด้านที่เหลือ flood ก็คืน INF ที่ช่องของตัวเอง
#: ซึ่งคือเงื่อนไขของ "[FAIL] จากความรู้ปัจจุบัน ไปเป้าหมายไม่ได้แล้ว"
SEALED_EDGES = [((0, 0), (1, 0)), ((0, 1), (1, 1)), ((0, 1), (0, 2))]


def test_arm_poses_in_range(chk):
    """ท่าแขนทุกค่าในไฟล์ต้องอยู่ในช่วงที่แขนรับได้จริง

    SDK ไม่ตรวจช่วงให้ (util.py:150 ตั้ง start/end เป็น None) ค่าที่เกินจะถูก
    ส่งลงเฟิร์มแวร์เงียบ ๆ แล้วแขนไปค้างที่ลิมิตกลไก เทสต์นี้จึงเป็นด่านเดียว
    ที่จับได้ตอนจูนค่าผิด ช่วงมาจาก docs/source/.../robotic_arm_and_gripper.rst
    """
    chk.section("ท่าแขนอยู่ในช่วง x 0-220 mm, y 0-150 mm")

    tc = load()
    poses = [("ARM_CARRY_XY", tc.ARM_CARRY_XY),
             ("ARM_PLACE_XY", tc.ARM_PLACE_XY),
             ("ARM_DROP_XY", tc.ARM_DROP_XY),
             ("ARM_PICK_XY", tc.ARM_PICK_XY)]
    for name, pose in poses:
        if pose is None:
            chk.check("{0} = None (ไม่ยื่นแขน) ใช้ได้".format(name), True)
            continue
        x, y = pose
        chk.check("{0} = {1} อยู่ในช่วง".format(name, pose),
                  0 <= x <= 220 and 0 <= y <= 150)


def test_default_config(chk):
    """คอนฟิกพื้นฐานที่สุด: รับของใส่มือที่จุดเริ่มแล้ววิ่งไปวางที่เป้าหมาย"""
    chk.section("รับของใส่มือที่จุดเริ่ม")

    tc = load()
    hub, driver, payload, arm, _ = make_world(tc, tc.START_CELL, None)
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("run_search ถึงเป้าหมาย", got is True)
    chk.check("รับวัตถุที่จุดเริ่ม", "[PICK] รับวัตถุที่ช่อง (0, 0)" in out)
    chk.check("ไม่พิมพ์เฟส 1 ที่ไม่มีอยู่จริง", "เฟส 1" not in out)
    chk.check("ไม่ยื่นแขนไปหยิบ",
              arm.calls[:2] == ["recenter", tc.ARM_CARRY_XY])
    chk.check("วางของตอนถึงเป้าหมาย", tc.ARM_PLACE_XY in arm.calls)
    chk.check("ไม่มีของค้างในมือตอนจบ", payload.holding is False)


def test_reach_at_start(chk):
    """วางของไว้หน้าหุ่นที่จุดเริ่ม แล้วให้ยื่นแขนหยิบเอง"""
    chk.section("ยื่นแขนหยิบที่จุดเริ่ม")

    tc = load()
    hub, driver, payload, arm, _ = make_world(tc, tc.START_CELL, (200, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("ถึงเป้าหมาย", got is True)
    chk.check("ใช้ข้อความหยิบของ", "[PICK] ถึงช่องหยิบของ (0, 0)" in out)
    chk.check("ยื่นแขน recenter -> PICK -> CARRY",
              arm.calls[:3] == ["recenter", (200, 0), tc.ARM_CARRY_XY])
    chk.check("ไม่พิมพ์เฟส 1", "เฟส 1" not in out)


def test_fetch_from_another_cell(chk):
    """ของอยู่ช่องอื่น หุ่นต้องแวะไปหยิบก่อนแล้วค่อยไปเป้าหมาย"""
    chk.section("วิ่งไปหยิบที่ช่องอื่นก่อน")

    for pick_cell in ((2, 1), (0, 3)):
        tc = load()
        hub, driver, payload, _, _ = make_world(tc, pick_cell, (200, 0))
        got, out = quiet(tc.run_search, hub, driver, payload)
        chk.check("{0}: ถึงเป้าหมาย".format(pick_cell), got is True)
        chk.check("{0}: พิมพ์เฟส 1".format(pick_cell),
                  "[PLAN] เฟส 1 ไปหยิบของที่ช่อง {0}".format(pick_cell) in out)
        chk.check("{0}: แวะหยิบจริง".format(pick_cell),
                  "[PICK] ถึงช่องหยิบของ {0}".format(pick_cell) in out)
        chk.check("{0}: หยิบก่อนถึงเป้าหมาย".format(pick_cell),
                  out.index("[PICK]") < out.index("[GOAL]"))


def test_turn_before_pick(chk):
    """ตั้ง PICK_HEADING แล้วต้องหันก่อนยื่นแขน"""
    chk.section("หันก่อนหยิบเมื่อตั้ง PICK_HEADING")

    tc = load()
    tc.PICK_HEADING = 1                                 # หันไปทางตะวันออก
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (200, 0))
    got, _ = quiet(tc.run_search, hub, driver, payload)
    chk.check("ถึงเป้าหมาย", got is True)
    chk.check("มีการหันไปทิศที่กำหนดก่อนหยิบ",
              driver.turns and driver.turns[0][1] == 1)


def test_degraded_configs(chk):
    """คอนฟิกที่ไม่สมบูรณ์ต้องเตือนแล้วเดินต่อ ไม่ใช่เงียบหรือพัง"""
    chk.section("คอนฟิกที่ไม่สมบูรณ์")

    tc = load()
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, None,
                                            gripper_status="closed")
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("คีบไม่ติด: ยังถึงเป้าหมาย", got is True)
    chk.check("คีบไม่ติด: เตือนว่าไม่มีของไปวาง",
              "ไม่ได้คีบวัตถุมาด้วย" in out)

    tc = load()
    hub, driver, _, _, _ = make_world(tc, tc.START_CELL, None)
    got, out = quiet(tc.run_search, hub, driver, None)
    chk.check("--no-payload: ถึงเป้าหมาย", got is True)
    chk.check("--no-payload: ไม่มีการหยิบเลย", "[PICK]" not in out)

    tc = load()
    hub, driver, payload, _, _ = make_world(tc, None, None)
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("PICK_CELL = None: ยังถึงเป้าหมาย", got is True)
    chk.check("PICK_CELL = None: เตือนว่าคอนฟิกผิด",
              "PICK_CELL เป็น None" in out)
    chk.check("PICK_CELL = None: แนะนำ --no-payload", "--no-payload" in out)


def test_abort_still_puts_object_down(chk):
    """จบแบบไปเป้าหมายไม่ได้ ต้องไม่ทิ้งของค้างในกริปเปอร์

    run_search เองแค่ return False - ตัวที่วางของคือ finally ของ main()
    (test_code.py: payload.put_down_if_holding()) เทสต์นี้จึงต่อสองท่อนนั้นเข้า
    ด้วยกันแบบเดียวกับที่ main() ทำ แล้วตรวจว่าของลงพื้นจริง
    """
    chk.section("ทางจบแบบล้มเหลวต้องวางของก่อนหยุด")

    tc = load()
    tc.SIM_BLOCKED_EDGES = SEALED_EDGES
    hub, driver, payload, arm, gripper = make_world(tc, tc.START_CELL,
                                                    (220, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("ไปเป้าหมายไม่ได้ -> คืน False", got is False)
    chk.check("พิมพ์เหตุผลที่ไปต่อไม่ได้",
              "ทุกทางที่รู้จักถูกกำแพงปิดหมด" in out)
    chk.check("หยิบของมาแล้วจริง", "[PICK]" in out)
    chk.check("run_search ไม่ได้วางของให้ (ของยังอยู่ในมือ)",
              payload.holding is True)
    chk.check("run_search ไม่แตะ ARM_DROP_XY เอง",
              tc.ARM_DROP_XY not in arm.calls)

    arm.calls, gripper.calls = [], []
    got, out = quiet(payload.put_down_if_holding)      # = finally ของ main()
    chk.check("finally วางของลงพื้น", got is True)
    chk.check("ก้มลงท่าวางตอนจบ แล้วเก็บแขน",
              arm.calls == [tc.ARM_DROP_XY, tc.ARM_CARRY_XY])
    chk.check("ไม่มีของค้างในมือตอนจบ", payload.holding is False)
    chk.check("ไม่มีธงหุบค้าง", payload.grip_closed is False)

    # เคสที่อันตรายที่สุด: สถานะกริปเปอร์อ่านว่ามือเปล่าทั้งที่อาจยังคีบอยู่
    tc = load()
    tc.SIM_BLOCKED_EDGES = SEALED_EDGES
    hub, driver, payload, arm, _ = make_world(tc, tc.START_CELL, (220, 0),
                                              gripper_status="closed")
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("คีบไม่ติด: ยังคืน False ที่ทางจบเดิม", got is False)
    chk.check("คีบไม่ติด: เตือนตั้งแต่ตอนหยิบ", "ไม่ได้คีบวัตถุมาด้วย" in out)
    arm.calls = []
    quiet(payload.put_down_if_holding)
    chk.check("คีบไม่ติด: finally ยังก้มลงกางนิ้วเผื่อไว้",
              arm.calls == [tc.ARM_DROP_XY, tc.ARM_CARRY_XY])

    # เดินครบ MAX_STEPS ก็เป็นทางจบแบบเดียวกัน ของต้องไม่ค้างเหมือนกัน
    tc = load()
    tc.MAX_STEPS = 2
    hub, driver, payload, arm, _ = make_world(tc, tc.START_CELL, (220, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("ครบ MAX_STEPS -> คืน False", got is False)
    chk.check("ครบ MAX_STEPS: ของยังอยู่ในมือ", payload.holding is True)
    arm.calls = []
    quiet(payload.put_down_if_holding)
    chk.check("ครบ MAX_STEPS: finally วางของให้",
              arm.calls == [tc.ARM_DROP_XY, tc.ARM_CARRY_XY])


def test_return_to_start(chk):
    """วางของเสร็จแล้วต้องเดินกลับไปที่ RETURN_CELL ด้วยแผนที่ก้อนเดิม"""
    chk.section("เดินกลับหลังวางของ")

    tc = load()
    hub, driver, payload, arm, _ = make_world(tc, tc.START_CELL, (200, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("จบครบทุกเฟส", got is True)
    chk.check("วางของที่เป้าหมายก่อน", "[GOAL] ถึงช่องเป้าหมาย (4, 4)" in out)
    chk.check("บอกว่าจะเดินกลับ",
              "[PLAN] วางของแล้ว เดินกลับไปที่ช่อง (0, 0)" in out)
    chk.check("กลับถึงช่องเริ่มต้นจริง",
              "[HOME] กลับถึงช่อง (0, 0)" in out)
    chk.check("หุ่นจอดอยู่ที่ช่องเริ่มต้นตอนจบ",
              hub.world.xy == tuple(tc.START_CELL))
    chk.check("วางของก่อนแล้วค่อยเดินกลับ",
              out.index("[GOAL]") < out.index("[HOME]"))
    chk.check("วางของครั้งเดียว ไม่ได้วางซ้ำตอนกลับถึงบ้าน",
              arm.calls.count(tc.ARM_PLACE_XY) == 1)
    chk.check("ไม่มีของค้างในมือตอนจบ", payload.holding is False)

    # ขากลับต้องนับต่อจากขาไป ไม่ใช่เริ่มนับใหม่
    goal_moves = _moves_reported(out)
    home_moves = None
    for line in out.split("\n"):
        if line.startswith("[HOME]"):
            home_moves = int([w for w in line.split() if w.isdigit()][0])
    chk.check("รายงานจำนวนช่องรวมทั้งสองขา ({0} -> {1})"
              .format(goal_moves, home_moves),
              home_moves is not None and home_moves > goal_moves)

    tc = load()
    tc.RETURN_CELL = (1, 0)                             # กลับไปช่องอื่นก็ได้
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (200, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("RETURN_CELL อื่น: จบครบ", got is True)
    chk.check("RETURN_CELL อื่น: ไปจอดที่ช่องนั้น", hub.world.xy == (1, 0))


def test_no_return_configs(chk):
    """ปิดการเดินกลับได้ทั้งจากคอนฟิกและจากบรรทัดคำสั่ง"""
    chk.section("ปิดการเดินกลับ")

    tc = load()
    tc.RETURN_CELL = None
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (200, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("RETURN_CELL = None: ถึงเป้าหมายแล้วจบ", got is True)
    chk.check("RETURN_CELL = None: ไม่เดินกลับ", "[HOME]" not in out)
    chk.check("RETURN_CELL = None: จอดค้างที่ช่องเป้าหมาย",
              hub.world.xy == tuple(tc.GOAL_CELLS[0]))

    tc = load()
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (200, 0))
    got, out = quiet(tc.run_search, hub, driver, payload, go_home=False)
    chk.check("--no-return: ถึงเป้าหมายแล้วจบ", got is True)
    chk.check("--no-return: ไม่เดินกลับ", "[HOME]" not in out)

    tc = load()
    body = inspect.getsource(tc.main)
    chk.check("main() ต่อสาย --no-return เข้ากับ run_search",
              "go_home=not args.no_return" in body)


def test_no_return_when_arm_stuck_out(chk):
    """ปล่อยของไม่ออกต้องจบตรงนั้น ไม่ลากแขนที่ยื่นค้างเดินกลับ

    _lower_and_release ตั้งใจคาแขนไว้ที่ท่าวางและกางนิ้วค้างไว้ให้คนหยิบของออก
    ง่ายที่สุด ท่านั้นคือแขนยื่นสุดและต่ำติดพื้น เดินทั้งท่านั้นคือลากแขนไปครูด
    กำแพงตลอดทาง
    """
    chk.section("ปล่อยของไม่ออกแล้วไม่เดินกลับ")

    tc = load()
    hub, driver, payload, _, gripper = make_world(tc, tc.START_CELL, (200, 0))
    gripper.after = {"open": "normal", "close": "normal"}   # กางแล้วยังคาอยู่
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("ยังนับว่าถึงเป้าหมาย", got is True)
    chk.check("เตือนว่าแขนยังยื่นค้าง", "แขนยังยื่นค้างอยู่ที่ท่าวาง" in out)
    chk.check("ไม่เดินกลับ", "[HOME]" not in out)
    chk.check("จอดค้างที่ช่องเป้าหมาย",
              hub.world.xy == tuple(tc.GOAL_CELLS[0]))


def test_face_way_back(chk):
    """ก่อนออกเดินขากลับต้องหันหลังให้ของที่เพิ่งวาง

    ทิศที่หันคือด้านที่เพิ่งเดินเข้าช่องมา ซึ่งเป็นด้านเดียวที่ยืนยันแล้วว่าโล่ง
    และเป็นด้านตรงข้ามกับของที่กองอยู่ตรงหน้า ถ้าอ่านเซนเซอร์จากท่าวางเลย ToF
    จะเห็นของเป็นกำแพงแล้วมาร์กลงแผนที่ ซึ่งลบออกไม่ได้
    """
    chk.section("หันหลังให้ของที่วางก่อนเดินกลับ")

    tc = load()
    driver = _StubTurner()
    got = tc.face_way_back(driver, 3, 2)                # เข้ามาทางใต้ วางทางตะวันตก
    chk.check("หันกลับไปทางตรงข้ามกับทางที่เดินเข้ามา", got == 0)
    chk.check("สั่งหันจริงหนึ่งครั้ง", driver.turns == [(3, 0)])

    driver = _StubTurner()
    got = tc.face_way_back(driver, 1, None)
    chk.check("ไม่เคยเดินเข้ามา: ไม่หัน", got == 1 and driver.turns == [])

    # เป้าหมายคือช่องที่ยืนอยู่ตั้งแต่แรก จึงไม่มีด้านไหนที่ยืนยันแล้ว
    tc = load()
    tc.GOAL_CELLS = [tuple(tc.START_CELL)]
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, (200, 0))
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("เป้าหมาย = ช่องเริ่มต้น: จบทันทีโดยไม่ต้องเดิน", got is True)
    chk.check("เป้าหมาย = ช่องเริ่มต้น: ไม่ต้องเดินกลับไปไหน",
              hub.world.xy == tuple(tc.START_CELL))


class _StubTurner(object):
    """driver ที่ทำได้อย่างเดียวคือจดว่าถูกสั่งหันไปทางไหน"""

    def __init__(self):
        self.turns = []
        #: list: ค่า align_first ของแต่ละครั้งที่ถูกสั่งหัน
        self.align_flags = []

    def turn_to(self, current_heading, target_heading, align_first=True):
        self.turns.append((current_heading, target_heading))
        self.align_flags.append(align_first)
        return target_heading


def test_sim_returns_too(chk):
    """run_sim ต้องจำลองขากลับด้วย ไม่งั้นตรวจแผนก่อนลงสนามไม่ครบ"""
    chk.section("run_sim จำลองขากลับ")

    tc = load()
    got, out = quiet(tc.run_sim)
    chk.check("จบครบทุกเฟส", got is True)
    chk.check("มีขากลับ", "[HOME] กลับถึงช่อง (0, 0)" in out)
    chk.check("เส้นทางที่พิมพ์จบที่ช่องเริ่มต้น",
              "-> (0, 0)" in out.split("เส้นทางที่เดินจริง:")[-1])

    tc = load()
    tc.RETURN_CELL = None
    got, out = quiet(tc.run_sim)
    chk.check("RETURN_CELL = None: จบที่เป้าหมาย", got is True)
    chk.check("RETURN_CELL = None: ไม่มีขากลับ", "[HOME]" not in out)


def test_main_cleans_up_payload(chk):
    """main() ต้องเรียก put_down_if_holding ใน finally จริง ๆ

    เทสต์อื่นเรียก put_down_if_holding เองแทน main() เพราะ main() ต้องต่อหุ่น
    จริง เทสต์นี้จึงเป็นด่านที่กันไม่ให้สายไฟเส้นนั้นหลุดไปเงียบ ๆ
    """
    chk.section("main() ต่อสายทางจบเข้ากับการวางของ")

    tc = load()
    body = inspect.getsource(tc.main)
    chk.check("main() มีบล็อก finally", "finally:" in body)
    chk.check("เรียก put_down_if_holding", "put_down_if_holding()" in body)
    chk.check("เรียกใน finally ไม่ใช่ในเส้นทางปกติ",
              "finally:" in body
              and body.index("finally:") < body.index("put_down_if_holding()"))
    chk.check("วางของก่อนปลด subscription กริปเปอร์",
              body.index("put_down_if_holding()")
              < body.index("payload.stop()"))


def _moves_reported(out):
    """ดึงจำนวนช่องที่รายงานตอนถึงเป้าหมายออกจาก log

    Returns:
        int or None: จำนวนช่อง หรือ None ถ้าไม่เจอบรรทัดเป้าหมาย
    """
    for line in out.split("\n"):
        if line.startswith("[GOAL]"):
            for word in line.split():
                if word.isdigit():
                    return int(word)
    return None


def test_move_counter(chk):
    """เลขที่รายงานต้องเป็นจำนวนช่องที่เดินจริง ไม่ใช่จำนวนรอบของลูป

    รอบที่หยิบของไม่ได้ทำให้หุ่นย้ายช่อง ถ้านับรวมไปด้วยเลขจะเฟ้อขึ้นหนึ่ง
    เทียบผลของสองคอนฟิกแทนที่จะยึดตัวเลขตายตัว เพื่อไม่ให้พังตอนแก้ขนาดสนาม
    """
    chk.section("การนับจำนวนช่องที่เดิน")

    tc = load()
    got, with_pick = quiet(tc.run_sim)
    chk.check("sim (มีการหยิบของ) ถึงเป้าหมาย", got is True)
    chk.check("หยิบที่จุดเริ่มนับเป็น 0 ช่อง", "หลังเดินมา 0 ช่อง" in with_pick)

    tc = load()
    tc.PICK_CELL = None                                 # ไม่มีรอบหยิบของเลย
    got, without_pick = quiet(tc.run_sim)
    chk.check("sim (ไม่มีการหยิบ) ถึงเป้าหมาย", got is True)

    a, b = _moves_reported(with_pick), _moves_reported(without_pick)
    chk.check("อ่านจำนวนช่องจาก log ได้", a is not None and b is not None)
    chk.check("รอบที่หยิบของไม่ทำให้ตัวเลขเฟ้อ ({0} = {1})".format(a, b),
              a == b)

    tc = load()
    hub, driver, payload, _, _ = make_world(tc, tc.START_CELL, None)
    got, out = quiet(tc.run_search, hub, driver, payload)
    chk.check("run_search รายงานเท่ากับ sim ({0})".format(a),
              _moves_reported(out) == a)


def test_sim_matches_configs(chk):
    """run_sim ต้องสลับเฟสตามคอนฟิกเดียวกับ run_search"""
    chk.section("run_sim ตามคอนฟิกเดียวกัน")

    tc = load()
    got, out = quiet(tc.run_sim)
    chk.check("ค่าเริ่มต้น: ถึงเป้าหมาย", got is True)
    chk.check("ค่าเริ่มต้น: ไม่พิมพ์เฟส 1", "เฟส 1" not in out)

    tc = load()
    tc.PICK_CELL = (1, 0)
    got, out = quiet(tc.run_sim)
    chk.check("ช่องอื่น: ถึงเป้าหมาย", got is True)
    chk.check("ช่องอื่น: พิมพ์เฟส 1", "[PLAN] เฟส 1" in out)
    chk.check("ช่องอื่น: แวะหยิบที่ (1, 0)",
              "[PICK] หยิบของที่ช่อง (1, 0)" in out)

    tc = load()
    tc.PICK_CELL = None
    got, out = quiet(tc.run_sim)
    chk.check("PICK_CELL = None: ถึงเป้าหมาย", got is True)
    chk.check("PICK_CELL = None: ไม่มีเฟสหยิบ", "[PICK]" not in out)


def run(chk=None):
    """รันเทสต์ทั้งไฟล์

    Returns:
        bool: True เมื่อผ่านหมด
    """
    own = chk is None
    chk = chk or Checker()
    print("=" * 58)
    print("  ทดสอบ run_search และ run_sim")
    print("=" * 58)
    test_arm_poses_in_range(chk)
    test_default_config(chk)
    test_reach_at_start(chk)
    test_fetch_from_another_cell(chk)
    test_turn_before_pick(chk)
    test_degraded_configs(chk)
    test_return_to_start(chk)
    test_no_return_configs(chk)
    test_no_return_when_arm_stuck_out(chk)
    test_face_way_back(chk)
    test_sim_returns_too(chk)
    test_abort_still_puts_object_down(chk)
    test_main_cleans_up_payload(chk)
    test_move_counter(chk)
    test_sim_matches_configs(chk)
    return chk.report() if own else True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)

# -*- coding: utf-8 -*-
"""ทดสอบคลาส Payload และโหมด --armtest ด้วยฮาร์ดแวร์ปลอม

ครอบคลุมสิ่งที่หน้าสนามตรวจยาก: กริปเปอร์รายงานว่าคีบไม่ติด, DDS สมัครไม่ได้,
แขนหรือกริปเปอร์โยน exception กลางคัน และการปล่อยของค้างตอนจบงานแบบล้มเหลว
"""
import sys
import time

from fakes import Checker, FakeHub, load, make_payload, quiet


def test_pick_up_both_ways(chk):
    """ยื่นแขนหยิบเอง กับ รอคนวางใส่มือ ต้องเดินคนละเส้นแต่ได้ผลเหมือนกัน"""
    chk.section("การหยิบสองแบบ")

    tc = load()
    payload, arm, gripper = make_payload(tc, {"open": "opened",
                                              "close": "normal"})
    got, out = quiet(payload.pick_up, None)
    chk.check("คนวางใส่มือ: คืน True", got is True)
    chk.check("คนวางใส่มือ: ไม่ยื่นแขน มีแค่ recenter -> CARRY",
              arm.calls == ["recenter", tc.ARM_CARRY_XY])
    chk.check("คนวางใส่มือ: มีช่วงรอวางของ", "รอวางวัตถุ" in out)
    chk.check("คนวางใส่มือ: กางก่อนหุบ",
              [name for name, _ in gripper.calls] == ["open", "close"])

    payload, arm, gripper = make_payload(tc, {"open": "opened",
                                              "close": "normal"})
    got, out = quiet(payload.pick_up, (200, 0))
    chk.check("ยื่นแขนหยิบ: คืน True", got is True)
    chk.check("ยื่นแขนหยิบ: recenter -> PICK -> CARRY",
              arm.calls == ["recenter", (200, 0), tc.ARM_CARRY_XY])
    chk.check("ยื่นแขนหยิบ: ไม่มีช่วงรอวางของ", "รอวางวัตถุ" not in out)
    chk.check("ยื่นแขนหยิบ: กางนิ้วก่อนยื่นแขนลงไป",
              [name for name, _ in gripper.calls] == ["open", "close"])


def test_confirm_grip(chk):
    """สถานะกริปเปอร์ต้องแปลเป็น "คีบติด/ไม่ติด" ให้ถูก

    gripper.py:36 นิยามว่า closed = นิ้วหุบจนสุด = ไม่มีอะไรคาอยู่
    ส่วน normal = หุบค้างกลางทาง = มีวัตถุคาอยู่
    """
    chk.section("การยืนยันว่าคีบติด")

    tc = load()
    for reach, label in ((None, "คนวางใส่มือ"), ((200, 0), "ยื่นแขนหยิบ")):
        payload, _, _ = make_payload(tc, {"open": "opened", "close": "normal"})
        got, _ = quiet(payload.pick_up, reach)
        chk.check("{0}: normal = คีบติด".format(label), got is True)

        payload, _, _ = make_payload(tc, {"open": "opened", "close": "closed"})
        got, _ = quiet(payload.pick_up, reach)
        chk.check("{0}: closed = นิ้วชนกัน ไม่มีของ".format(label),
                  got is False)

        payload, _, _ = make_payload(tc, {"open": "opened", "close": "opened"})
        got, _ = quiet(payload.pick_up, reach)
        chk.check("{0}: opened = คำสั่งหุบไม่มีผล".format(label), got is False)

        payload, _, _ = make_payload(tc, sub_fail=True)
        got, _ = quiet(payload.pick_up, reach)
        chk.check("{0}: ไม่มี sub_status = ถือว่าติดไว้ก่อน".format(label),
                  got is True)


def test_hardware_errors_do_not_escape(chk):
    """แขนหรือกริปเปอร์พังต้องไม่ลาก search run ทั้งรอบล้มไปด้วย"""
    chk.section("ฮาร์ดแวร์พังกลางคัน")

    tc = load()
    for reach, label in ((None, "คนวางใส่มือ"), ((200, 0), "ยื่นแขนหยิบ")):
        payload, _, _ = make_payload(tc, {"close": "normal"}, arm_fail=True)
        try:
            quiet(payload.pick_up, reach)
            chk.check("{0}: แขนพังแล้วไม่ throw".format(label), True)
        except Exception as exc:                        # noqa: BLE001
            chk.check("{0}: แขนพังแล้วไม่ throw ({1})".format(label, exc),
                      False)

        payload, _, _ = make_payload(tc, fail=True)
        try:
            quiet(payload.pick_up, reach)
            quiet(payload.place)
            chk.check("{0}: กริปเปอร์พังแล้วไม่ throw".format(label), True)
        except Exception as exc:                        # noqa: BLE001
            chk.check("{0}: กริปเปอร์พังแล้วไม่ throw ({1})".format(label, exc),
                      False)


def test_place_and_release(chk):
    """การวางของ และการปล่อยของค้างตอนจบงานแบบไม่ถึงเป้าหมาย"""
    chk.section("การวางและการปล่อยของ")

    tc = load()
    payload, arm, gripper = make_payload(tc, {"open": "opened",
                                              "close": "normal"})
    quiet(payload.pick_up, None)
    arm.calls, gripper.calls = [], []
    got, _ = quiet(payload.place)
    chk.check("วางสำเร็จคืน True", got is True)
    chk.check("holding = False หลังวาง", payload.holding is False)
    chk.check("ไปจุดวางก่อน แล้วกลับท่าวิ่ง",
              arm.calls == [tc.ARM_PLACE_XY, tc.ARM_CARRY_XY])
    chk.check("ปิดท้ายด้วยหุบเบากันนิ้วเกี่ยวกำแพง",
              gripper.calls[-1] == ("close", tc.GRIPPER_TUCK_POWER))
    chk.check("กางสำเร็จตั้งแต่ครั้งแรก ไม่สั่งซ้ำ",
              [name for name, _ in gripper.calls].count("open") == 1)

    payload, arm, gripper = make_payload(tc, {"open": "normal",
                                              "close": "normal"})
    quiet(payload.pick_up, None)
    arm.calls, gripper.calls = [], []
    got, out = quiet(payload.place)
    chk.check("ปล่อยไม่ออกคืน False", got is False)
    chk.check("holding ยัง True เมื่อปล่อยไม่ออก", payload.holding is True)
    opens = [name for name, _ in gripper.calls].count("open")
    chk.check("ลองกางซ้ำครบ {0} ครั้ง".format(tc.GRIPPER_RELEASE_TRIES),
              opens == tc.GRIPPER_RELEASE_TRIES)
    chk.check("ไม่หุบนิ้วกลับ (ไม่คีบของขึ้นมาใหม่)",
              "close" not in [name for name, _ in gripper.calls])
    chk.check("ไม่ยกแขนกลับท่าวิ่ง", tc.ARM_CARRY_XY not in arm.calls)
    chk.check("บอกให้คนเอาวัตถุออกเอง", "ให้เอาวัตถุออกเอง" in out)
    arm.calls, gripper.calls = [], []
    quiet(payload.put_down_if_holding)
    chk.check("ตอนจบงานลองวางซ้ำ",
              ("open", tc.GRIPPER_POWER) in gripper.calls)

    payload, arm, gripper = make_payload(tc, {"open": "opened",
                                              "close": "normal"})
    got, _ = quiet(payload.put_down_if_holding)
    chk.check("ไม่เคยสั่งหุบ: ไม่สั่งอะไร", gripper.calls == [])
    chk.check("ไม่เคยสั่งหุบ: ไม่ขยับแขน", arm.calls == [])
    chk.check("ไม่เคยสั่งหุบ: คืน True", got is True)


def test_release_even_when_confirm_says_empty(chk):
    """เคยสั่งหุบไว้ต้องปล่อยตอนจบเสมอ ถึงสถานะจะบอกว่ามือเปล่า

    _confirm_grip() ผิดฝั่งอันตรายได้: ของบางหรือนิ่มทำให้นิ้วหุบเกือบสุดจน
    สถานะรายงาน "closed" ทั้งที่ยังคีบอยู่จริง ถ้าตัดสินใจจาก holding อย่างเดียว
    ตอนจบงานจะไม่ปล่อยอะไรเลย ของค้างในมือและมอเตอร์บีบค้างจนกว่าจะปิดเครื่อง
    """
    chk.section("ปล่อยเผื่อไว้เมื่อสถานะอ่านว่ามือเปล่า")

    tc = load()
    for status, label in (("closed", "นิ้วหุบจนสุด"), ("opened", "นิ้วยังกาง")):
        payload, arm, gripper = make_payload(tc, {"open": "opened",
                                                  "close": status})
        got, _ = quiet(payload.pick_up, None)
        chk.check("{0}: pick_up คืน False".format(label), got is False)
        chk.check("{0}: holding = False".format(label),
                  payload.holding is False)
        chk.check("{0}: จำได้ว่าสั่งหุบไปแล้ว".format(label),
                  payload.grip_closed is True)

        arm.calls, gripper.calls = [], []
        got, out = quiet(payload.put_down_if_holding)
        chk.check("{0}: ยังก้มลงกางนิ้วเผื่อไว้".format(label),
                  arm.calls == [tc.ARM_DROP_XY, tc.ARM_CARRY_XY])
        chk.check("{0}: กางด้วยแรงเต็ม".format(label),
                  ("open", tc.GRIPPER_POWER) in gripper.calls)
        chk.check("{0}: บอกเหตุผลที่ปล่อยเผื่อ".format(label),
                  "สถานะบอกว่ามือเปล่า" in out)
        chk.check("{0}: คืน True".format(label), got is True)
        chk.check("{0}: เคลียร์ธงหลังปล่อยแล้ว".format(label),
                  payload.grip_closed is False)

        arm.calls, gripper.calls = [], []
        quiet(payload.put_down_if_holding)
        chk.check("{0}: เรียกซ้ำแล้วไม่สั่งอะไรอีก".format(label),
                  arm.calls == [] and gripper.calls == [])

    # คำสั่งหุบ throw ไปเลย = นิ้วไม่ได้บีบอะไรไว้ ธง grip_closed จึงไม่ขึ้น
    # แต่ก็ไม่มีสถานะให้ตรวจเหมือนกัน holding เลยค้างเป็น True ตามค่าเผื่อไว้ก่อน
    # ของ _confirm_grip() ผลรวมคือตอนจบยังลองปล่อยอยู่ดี ซึ่งเป็นฝั่งที่ปลอดภัย
    payload, arm, gripper = make_payload(tc, fail=True)
    quiet(payload.pick_up, None)
    chk.check("สั่งหุบไม่ออก: ไม่ตั้งธงว่าหุบแล้ว", payload.grip_closed is False)
    arm.calls, gripper.calls = [], []
    quiet(payload.put_down_if_holding)
    chk.check("สั่งหุบไม่ออก: ยังลองปล่อยตอนจบ (ไม่มีสถานะให้ตรวจ)",
              arm.calls == [tc.ARM_DROP_XY, tc.ARM_CARRY_XY])

    # ค่าคงตัวที่ทั้งคลาสยึด: หลังพยายามหยิบไปแล้ว ห้ามมีเคสไหนที่ทั้งสองธงเป็น
    # False พร้อมกัน เพราะนั่นคือเคสเดียวที่ put_down_if_holding() จะเงียบ
    for kwargs, label in ((dict(after={"open": "opened", "close": "normal"}),
                           "คีบติด"),
                          (dict(after={"open": "opened", "close": "closed"}),
                           "นิ้วหุบจนสุด"),
                          (dict(after={"open": "opened", "close": "opened"}),
                           "หุบไม่มีผล"),
                          (dict(sub_fail=True), "ไม่มี sub_status"),
                          (dict(fail=True), "กริปเปอร์พัง"),
                          (dict(after={"close": "normal"}, arm_fail=True),
                           "แขนพัง")):
        for reach in (None, (200, 0)):
            payload, _, _ = make_payload(tc, **kwargs)
            quiet(payload.pick_up, reach)
            chk.check("{0} / reach={1}: ยังมีธงให้ปล่อยตอนจบ"
                      .format(label, reach),
                      payload.holding or payload.grip_closed)


def test_drop_pose_is_shorter_than_place(chk):
    """ท่าวางตอนจบต้องยื่นสั้นกว่าท่าวางปกติ

    การจบแบบไม่ถึงเป้าหมายเกิดตอนหุ่นจอดหันชนกำแพงพอดี (ไปเป้าหมายไม่ได้แล้ว /
    ถูกล้อมทุกด้าน) ถ้ายื่นสุดแขนของกับนิ้วจะไปกระแทกกำแพงแทนที่จะวางลงพื้น
    """
    chk.section("ระยะยื่นของท่าวางตอนจบงาน")

    tc = load()
    chk.check("ARM_DROP_XY ยื่นสั้นกว่า ARM_PLACE_XY ({0} < {1} mm)"
              .format(tc.ARM_DROP_XY[0], tc.ARM_PLACE_XY[0]),
              tc.ARM_DROP_XY[0] < tc.ARM_PLACE_XY[0])
    chk.check("ยังยื่นพ้นตัวหุ่น ไม่ใช่ปล่อยคาอก ({0} mm)"
              .format(tc.ARM_DROP_XY[0]), tc.ARM_DROP_XY[0] > 0)
    chk.check("ก้มถึงพื้น ไม่ใช่ปล่อยจากที่สูง (y = {0})"
              .format(tc.ARM_DROP_XY[1]), tc.ARM_DROP_XY[1] == 0)


def test_put_down_on_abort(chk):
    """จบงานแบบไม่ถึงเป้าหมายต้อง "ก้มวาง" ไม่ใช่กางนิ้วปล่อยจากท่าวิ่ง

    ปล่อยจากท่าวิ่งขวดจะตกกระแทกพื้นแล้วล้มกลิ้ง
    """
    chk.section("การวางของตอนจบงานแบบไม่ถึงเป้าหมาย")

    tc = load()
    payload, arm, gripper = make_payload(tc, {"open": "opened",
                                              "close": "normal"})
    quiet(payload.pick_up, None)
    arm.calls, gripper.calls = [], []
    got, out = quiet(payload.put_down_if_holding)
    chk.check("คืน True เมื่อวางลงแล้ว", got is True)
    chk.check("holding = False หลังวาง", payload.holding is False)
    chk.check("ก้มลงท่าวางก่อน แล้วค่อยเก็บแขน",
              arm.calls == [tc.ARM_DROP_XY, tc.ARM_CARRY_XY])
    chk.check("ปล่อยของหลังก้มลงแล้ว ไม่ใช่ก่อน",
              [name for name, _ in gripper.calls] == ["open", "close"])
    chk.check("บอกผู้ใช้ว่ากำลังวางของก่อนจบ", "ก้มวางลงพื้นก่อนจบงาน" in out)

    payload, arm, _ = make_payload(tc, {"close": "normal"}, arm_fail=True)
    quiet(payload.pick_up, None)
    try:
        quiet(payload.put_down_if_holding)
        chk.check("แขนพังตอนวางแล้วไม่ throw ออกจาก finally", True)
    except Exception as exc:                            # noqa: BLE001
        chk.check("แขนพังตอนวางแล้วไม่ throw ({0})".format(exc), False)


def test_status_and_subscription(chk):
    """สถานะที่ค้างเก่าต้องใช้ตัดสินไม่ได้ และ stop() ต้องเรียกซ้ำได้"""
    chk.section("สถานะกริปเปอร์และ subscription")

    tc = load()
    payload, _, gripper = make_payload(tc, {"close": "normal"})
    chk.check("สมัคร sub_status แล้ว", gripper.subscribed is True)
    payload._on_status("normal")
    chk.check("ค่าสด อ่านได้", payload.status() == "normal")
    payload._status_t = time.time() - (tc.GRIPPER_STATUS_STALE_S + 0.5)
    chk.check("ค่าค้างเก่าคืนสตริงว่าง", payload.status() == "")
    payload.stop()
    chk.check("stop() ปลด subscription", gripper.subscribed is False)
    payload.stop()
    chk.check("stop() ซ้ำได้ไม่ throw", True)


def test_arm_test_mode(chk):
    """--armtest ต้องบอกได้ว่าคีบติดไหม และของที่คีบบัง ToF หรือเปล่า"""
    chk.section("โหมด --armtest")

    tc = load()
    tc.input = lambda *a: ""
    threshold = tc.front_wall_threshold_mm()
    chk.check("เกณฑ์กำแพงหน้าคำนวณได้ {0} mm".format(threshold),
              threshold == tc.FRONT_STOP_MM + int(tc.CELL_SIZE_M * 1000 / 2))

    # ตั้งเองทั้งสองเคส ไม่พึ่งค่าที่อยู่ในไฟล์จริง เพราะมันเป็นค่าที่ผู้ใช้จูน
    tc.ARM_PICK_XY = None
    payload, arm, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    got, out = quiet(tc.run_arm_test, FakeHub(threshold + 200), payload)
    chk.check("ARM_PICK_XY เป็น None -> ปฏิเสธ", got is False)
    chk.check("บอกให้ตั้ง ARM_PICK_XY ก่อน",
              "ตั้ง ARM_PICK_XY เป็นพิกัดก่อน" in out)
    chk.check("ไม่แตะแขนเลยตอนปฏิเสธ", arm.calls == [])

    tc.ARM_PICK_XY = (200, 0)
    payload, arm, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    got, out = quiet(tc.run_arm_test, FakeHub(threshold + 200), payload)
    chk.check("ไม่บัง ToF -> ผ่าน", got is True)
    chk.check("ยื่นแขนไปท่าหยิบจริง", (200, 0) in arm.calls)

    payload, _, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    got, out = quiet(tc.run_arm_test, FakeHub(100), payload)
    chk.check("ของสูงบัง ToF -> ไม่ผ่าน", got is False)
    chk.check("เตือนว่าของบัง ToF", "ของบัง ToF อยู่" in out)
    chk.check("บอกวิธีแก้ที่ ARM_CARRY_XY", "ARM_CARRY_XY" in out)

    payload, _, _ = make_payload(tc, {"open": "opened", "close": "closed"})
    got, out = quiet(tc.run_arm_test, FakeHub(threshold + 200), payload)
    chk.check("คีบไม่ติด -> ไม่ผ่าน", got is False)
    chk.check("บอกช่วงที่สั่งแขนได้", "x 0-220 mm, y 0-150 mm" in out)

    payload, _, _ = make_payload(tc, {"open": "opened", "close": "normal"})
    got, out = quiet(tc.run_arm_test, FakeHub(None), payload)
    chk.check("ไม่มีค่า ToF -> ไม่ผ่าน", got is False)
    chk.check("บอกให้ตรวจการต่อเซนเซอร์", "ตรวจการต่อเซนเซอร์" in out)


def run(chk=None):
    """รันเทสต์ทั้งไฟล์

    Returns:
        bool: True เมื่อผ่านหมด
    """
    own = chk is None
    chk = chk or Checker()
    print("=" * 58)
    print("  ทดสอบ Payload และ --armtest")
    print("=" * 58)
    test_pick_up_both_ways(chk)
    test_confirm_grip(chk)
    test_hardware_errors_do_not_escape(chk)
    test_place_and_release(chk)
    test_release_even_when_confirm_says_empty(chk)
    test_drop_pose_is_shorter_than_place(chk)
    test_put_down_on_abort(chk)
    test_status_and_subscription(chk)
    test_arm_test_mode(chk)
    return chk.report() if own else True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)

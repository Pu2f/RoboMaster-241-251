# -*- coding: utf-8 -*-
"""โหมด --calib : วัดค่าเซนเซอร์จริงจากหุ่นตัวนี้ในสนามนี้"""
import statistics
import time

from . import config


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

    print("\nคัดลอกบล็อกนี้ไปวางทับใน maze_solver/config.py:\n")
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
              .format(int(round(tof_mean)) + int(config.CELL_SIZE_M * 1000 / 2)))
    print()

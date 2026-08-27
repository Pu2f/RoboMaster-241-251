# -*- coding: utf-8 -*-
"""โหมด --armtest : จูนท่าแขนกับวัตถุจริง โดยไม่ต้องรันทั้งเขาวงกต"""
import time

from . import config
from .geometry import front_wall_threshold_mm


def run_arm_test(hub, payload):
    """ยื่นแขนหยิบของที่วางไว้ตรงหน้า แล้วรายงานว่าใช้ค่าปัจจุบันได้จริงไหม

    มีสองอย่างที่ต้องรู้ก่อนใช้ ARM_PICK_XY จริง ทั้งคู่ต้องวัดกับของจริง
    1. ``ARM_PICK_XY`` เอื้อมถึงของและคีบติดหรือเปล่า
    2. ของที่คีบแล้วยกขึ้นท่าวิ่ง ไปบัง ToF หรือเปล่า ซึ่งถ้าบังหุ่นจะเห็นเป็น
       กำแพงหน้าตลอดทางแล้วเดินไม่ได้เลย ของสูง ๆ อย่างขวดน้ำเจอปัญหานี้ง่ายมาก

    Returns:
        bool: True เมื่อคีบติดและของที่คีบไม่บัง ToF
    """
    print("\n" + "=" * 62)
    print("  ARM TEST - จูนท่าแขนกับวัตถุจริง")
    print("  ARM_PICK_XY  = {0}".format(config.ARM_PICK_XY))
    print("  ARM_CARRY_XY = {0}".format(config.ARM_CARRY_XY))
    print("  ARM_PLACE_XY = {0}".format(config.ARM_PLACE_XY))
    print("=" * 62)
    if config.ARM_PICK_XY is None:
        print("\n[STOP] ARM_PICK_XY เป็น None = ตั้งไว้ให้คนวางของใส่มือ")
        print("       โหมดนี้มีไว้จูนท่ายื่นแขน ตั้ง ARM_PICK_XY เป็นพิกัดก่อน")
        return False

    print("\nวางวัตถุไว้ตรงหน้าหุ่น ให้อยู่กึ่งกลางลำตัว")
    print("และอยู่ในระยะที่แขนเอื้อมถึง")
    print("(แขนยื่นได้ไกลสุด 220 mm วัดจากฐานแขน)")
    input("พร้อมแล้วกด Enter...")

    gripped = payload.pick_up(reach_xy=config.ARM_PICK_XY)
    if not gripped:
        print("\n[ARMTEST] คีบไม่ติด ลองปรับ ARM_PICK_XY แล้วรันใหม่")
        print("          x น้อยลง = หดเข้าหาตัว, y น้อยลง = ต่ำลงติดพื้น")
        print("          ช่วงที่สั่งได้คือ x 0-220 mm, y 0-150 mm")

    # ของอยู่ในมือที่ท่าวิ่งแล้ว ตรงนี้คือจุดที่ ToF ต้องยังมองทะลุไปข้างหน้าได้
    print("\n[ARMTEST] อ่าน ToF ตอนคีบของค้างไว้ที่ท่าวิ่ง...")
    readings = []
    for _ in range(config.ARMTEST_TOF_SAMPLES):
        snap = hub.snapshot()
        if snap.tof_mm is not None:
            readings.append(snap.tof_mm)
        time.sleep(0.1)

    threshold = front_wall_threshold_mm()
    blocked = True
    if not readings:
        print("[ARMTEST] ไม่ได้ค่า ToF เลย ตรวจการต่อเซนเซอร์ก่อน")
    else:
        avg = sum(readings) / float(len(readings))
        print("[ARMTEST] ToF เฉลี่ย {0:.0f} mm "
              "(ต่ำสุด {1} สูงสุด {2} จาก {3} ครั้ง)"
              .format(avg, min(readings), max(readings), len(readings)))
        blocked = avg < threshold
        if blocked:
            print("[ARMTEST] ต่ำกว่าเกณฑ์กำแพงหน้า {0} mm = ของบัง ToF อยู่"
                  .format(threshold))
            print("          หุ่นจะเห็นเป็นกำแพงตลอดทางแล้วเดินไม่ได้")
            print("          แก้ที่ ARM_CARRY_XY: ยก y สูงขึ้น (ไม่เกิน 150)")
            print("          หรือหด x ให้เข้าหาตัวมากขึ้น แล้วรันใหม่")
        else:
            print("[ARMTEST] ไม่บัง ToF (เกณฑ์กำแพงหน้าคือ {0} mm)"
                  .format(threshold))

    input("\nกด Enter เพื่อให้หุ่นวางของลง...")
    placed = payload.place()

    ok = gripped and not blocked and placed
    print("\n" + "=" * 62)
    print("  สรุป: คีบติด {0} | ไม่บัง ToF {1} | วางลงได้ {2}"
          .format("ใช่" if gripped else "ไม่", "ใช่" if not blocked else "ไม่",
                  "ใช่" if placed else "ไม่"))
    print("  {0}".format("ค่าปัจจุบันใช้ได้" if ok else "ยังต้องปรับค่าอีก"))
    print("=" * 62)
    return ok

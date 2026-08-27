# -*- coding: utf-8 -*-
"""จุดเข้าโปรแกรม - อ่านอาร์กิวเมนต์ ต่อหุ่น แล้วส่งต่อให้โหมดที่เลือก"""
import argparse
import time

try:
    # pyrefly: ignore [missing-import]
    from robomaster import robot
except ImportError as exc:  # โหมด --sim ทดสอบตรรกะได้โดยไม่ต้องมี SDK ติดตั้ง
    robot = None
    ROBOT_IMPORT_ERROR = exc
else:
    ROBOT_IMPORT_ERROR = None

from . import config
from .armtest import run_arm_test
from .calibrate import run_calibration
from .config import require_calibration
from .driver import Driver
from .payload import Payload
from .search import run_search
from .sensors import SensorHub
from .sim import run_sim


def main():
    parser = argparse.ArgumentParser(
        description="Maze solver ด้วย Flood Fill สำหรับ RoboMaster EP")
    parser.add_argument("--calib", action="store_true",
                        help="วัดค่าเซนเซอร์จริง หุ่นจะไม่ขยับ")
    parser.add_argument("--sim", action="store_true",
                        help="ทดสอบตรรกะ Flood Fill โดยไม่ต้องต่อหุ่น")
    parser.add_argument("--conn", default=config.CONN_TYPE,
                        choices=["ap", "sta", "rndis"],
                        help="วิธีเชื่อมต่อหุ่น (ค่าเริ่มต้น {0})".format(config.CONN_TYPE))
    parser.add_argument("--no-payload", action="store_true",
                        help="ข้ามการคีบและวางวัตถุ ใช้ตอนดีบักเฉพาะการเดิน")
    parser.add_argument("--no-return", action="store_true",
                        help="วางของแล้วจบตรงช่องเป้าหมาย ไม่ต้องเดินกลับ")
    parser.add_argument("--armtest", action="store_true",
                        help="ยื่นแขนหยิบของตรงหน้า ใช้จูน ARM_PICK_XY "
                             "หุ่นจะไม่เดินไปไหน")
    args = parser.parse_args()

    if args.sim:
        return 0 if run_sim() else 1

    if robot is None:
        print("[ERROR] import robomaster ไม่สำเร็จ: {0}".format(ROBOT_IMPORT_ERROR))
        print("        ติดตั้ง SDK ก่อน หรือใช้ --sim เพื่อทดสอบเฉพาะตรรกะ")
        return 1

    # --armtest ใช้แค่ ToF กับแขน ไม่ได้อ่าน Sharp/IR จึงไม่ต้องบังคับคาลิเบรต
    if not args.calib and not args.armtest:
        require_calibration()

    ep_robot = robot.Robot()
    print("กำลังเชื่อมต่อหุ่นแบบ {0} ...".format(args.conn))
    ep_robot.initialize(conn_type=args.conn)

    hub = SensorHub(ep_robot)
    payload = None
    success = False
    try:
        hub.start()
        if args.calib:
            run_calibration(hub)
            success = True
        elif args.armtest:
            payload = Payload(ep_robot.robotic_arm, ep_robot.gripper)
            payload.start()
            success = run_arm_test(hub, payload)
        else:
            # initialize() ตั้ง FREE ให้อยู่แล้ว (robot.py reset) แต่สั่งซ้ำให้ชัดเจน
            # ว่าโค้ดนี้ต้องการให้แชสซีขยับอิสระจากกิมบอล
            ep_robot.set_robot_mode(robot.FREE)
            time.sleep(0.5)

            driver = Driver(ep_robot.chassis, hub)
            driver.calibrate_yaw_sign()
            driver.set_north_reference(config.START_HEADING)

            if config.DO_PAYLOAD and not args.no_payload:
                payload = Payload(ep_robot.robotic_arm, ep_robot.gripper)
                payload.start()
            else:
                print("[INFO] ข้ามการคีบและวางวัตถุ")

            success = run_search(hub, driver, payload,
                                 go_home=not args.no_return)
    except KeyboardInterrupt:
        print("\n[STOP] ผู้ใช้สั่งหยุด")
    finally:
        try:
            ep_robot.chassis.drive_speed(x=0, y=0, z=0)
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] สั่งหยุดล้อไม่สำเร็จ: {0}".format(exc))
        if payload is not None:
            # จบแบบไม่ถึงเป้าหมาย (เซนเซอร์หลุด / ไปต่อไม่ได้ / Ctrl-C) place()
            # ไม่ได้ถูกเรียก วัตถุจะค้างอยู่ในกริปเปอร์ ต้องก้มวางลงเองตรงนี้
            # ทำหลังสั่งหยุดล้อแล้ว เพื่อให้หุ่นนิ่งก่อนขยับแขน
            payload.put_down_if_holding()
            payload.stop()
        hub.stop()
        try:
            ep_robot.close()
        except Exception as exc:                        # noqa: BLE001
            print("[WARN] ปิดการเชื่อมต่อไม่สำเร็จ: {0}".format(exc))

    return 0 if success else 1

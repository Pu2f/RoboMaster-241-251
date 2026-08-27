# -*- coding: utf-8 -*-
"""State machine หลัก - search run ด้วย Flood Fill"""
import time

from . import config
from .directions import DIR_NAMES, DX, DY, INF
from .maze import Maze
from .payload import face_way_back, place_on_target
from .readings import observation_is_trusted


def run_search(hub, driver, payload, go_home=True):
    """เดินสำรวจด้วย Flood Fill ตามลำดับ หยิบของ -> วางของ -> เดินกลับ

    ทั้งสามเฟสใช้ลูปเดียวกันหมด ต่างกันแค่ ``maze.goals`` ที่ตั้งใหม่ตอนจบแต่ละ
    เฟส แผนที่ไม่ถูกล้างระหว่างเฟส ขากลับจึงเริ่มจากความรู้ทั้งหมดที่สะสมมา

    Args:
        hub (SensorHub): ตัวอ่านเซนเซอร์
        driver (Driver): ตัวควบคุมการเคลื่อนที่
        payload (Payload or None): แขนกลและกริปเปอร์ None = ไม่คีบไม่วาง
        go_home (bool): False = วางของแล้วจบตรงช่องเป้าหมาย ไม่ต้องเดินกลับ
            (มาจาก --no-return) ค่า True ยังเดินกลับก็ต่อเมื่อตั้ง
            ``RETURN_CELL`` ไว้ด้วย

    Returns:
        bool: True เมื่อจบครบทุกเฟสที่ตั้งไว้
    """
    maze = Maze(config.MAZE_W, config.MAZE_H, config.GOAL_CELLS)
    x, y = config.START_CELL
    heading = config.START_HEADING

    return_cell = tuple(config.RETURN_CELL) if go_home and config.RETURN_CELL is not None \
        else None

    print("=" * 62)
    print("  MAZE SEARCH RUN - Flood Fill")
    print("  สนาม {0}x{1} ช่องละ {2:.2f} m | เริ่มที่ {3} หัน {4} | เป้าหมาย {5}"
          .format(config.MAZE_W, config.MAZE_H, config.CELL_SIZE_M, config.START_CELL,
                  DIR_NAMES[config.START_HEADING], config.GOAL_CELLS))
    if return_cell is not None:
        print("  วางของเสร็จแล้วเดินกลับไปที่ช่อง {0}".format(return_cell))
    print("=" * 62)

    # ตั้งเป้าเฟสแรกไปที่ช่องหยิบของเสมอ ถ้าเป็นช่องเริ่มต้นก็แค่หยิบอยู่กับที่
    # โดยไม่ขยับ ทำแบบนี้เพื่อให้ทุกคอนฟิกเดินผ่านโค้ดชุดเดียวกันหมด
    pick_pending = payload is not None and config.PICK_CELL is not None
    if pick_pending:
        maze.goals = [tuple(config.PICK_CELL)]
        if tuple(config.PICK_CELL) != tuple(config.START_CELL):
            print("[PLAN] เฟส 1 ไปหยิบของที่ช่อง {0} แล้วค่อยไปเป้าหมาย {1}"
                  .format(tuple(config.PICK_CELL), config.GOAL_CELLS))
    elif payload is not None:
        print("[WARN] PICK_CELL เป็น None จะไม่หยิบอะไรเลยแต่ยังสั่งวางตอนจบ")
        print("       ถ้าตั้งใจจะไม่คีบของ ให้ใช้ --no-payload แทน")

    # นับความล้มเหลวซ้ำที่ (ช่อง, ทิศ) เดิม ใช้ตัดวงจรกรณีเดินไม่ผ่านแต่ ToF
    # ก็ไม่เห็นกำแพง (ล้อลื่น ติดขอบ ฯลฯ) ซึ่งถ้าไม่ตัดจะเลือกทิศเดิมซ้ำไปเรื่อย ๆ
    fail_key = None
    fail_count = 0
    # ระยะที่เพิ่งเดินเข้าช่องปัจจุบันมาตามทิศที่หันอยู่ตอนนี้ 0 = ไม่รู้ว่าถอย
    # กลับไปได้แค่ไหน (ยังไม่เคยเดิน หรือหมุนตัวหลังเข้าช่องไปแล้ว)
    entry_travel = 0.0
    # ทิศที่เดินเข้าช่องปัจจุบันมา ต่างจาก entry_travel ตรงที่การหมุนตัวไม่ทำให้
    # ค่านี้ใช้ไม่ได้ เพราะมันบอก "ด้านไหนของช่องที่เปิด" ซึ่งไม่เปลี่ยนตามท่าหุ่น
    entry_heading = None
    # ยังไม่ได้วางของ ใช้แยกว่ารอบที่ถึง maze.goals คือถึงจุดวาง หรือกลับถึงบ้าน
    place_pending = True
    # นับ "ช่องที่เดินผ่านจริง" แยกจากรอบของลูป เพราะรอบที่หยิบของ รอบที่เดินไม่
    # ผ่าน และรอบที่ยกเลิกเพราะ SAFETY ก็กินรอบไปด้วยทั้งที่หุ่นไม่ได้ย้ายช่อง
    moves = 0

    for step in range(config.MAX_STEPS):
        if (x, y) in maze.goals and pick_pending:
            # ToF เบรกให้หน้าวัตถุที่ FRONT_STOP_MM แล้ว หยิบได้เลยจากตรงนี้
            # หมายเหตุ: ยังไม่มีการ observe() ที่ช่องนี้ เพราะ goal ถูกเช็คก่อน
            # ดังนั้นวัตถุจึงไม่ถูกบันทึกเป็นกำแพงหน้าลงแผนที่ ซึ่งลบออกไม่ได้
            if config.ARM_PICK_XY is None:
                print("\n[PICK] รับวัตถุที่ช่อง {0}".format((x, y)))
            else:
                print("\n[PICK] ถึงช่องหยิบของ {0} แล้ว เดินมา {1} ช่อง"
                      .format((x, y), moves))
            driver.stop()
            if config.PICK_HEADING is not None:
                heading = driver.turn_to(heading, config.PICK_HEADING)
                entry_travel = 0.0      # หันแล้ว ข้างหลังไม่ใช่ทางที่เพิ่งผ่าน
            if not payload.pick_up(reach_xy=config.ARM_PICK_XY):
                print("[WARN] ไม่ได้คีบวัตถุมาด้วย จะเดินต่อแต่ไม่มีของไปวาง")
            maze.goals = list(config.GOAL_CELLS)
            pick_pending = False
            print("[PLAN] มุ่งหน้าไปเป้าหมาย {0}".format(maze.goals))
            continue

        if (x, y) in maze.goals and place_pending:
            print("\n[GOAL] ถึงช่องเป้าหมาย {0} แล้ว เดินมา {1} ช่อง"
                  .format((x, y), moves))
            driver.stop()
            place_pending = False
            # เคยมีของอยู่ในมือจริงไหม ถ้าคีบไม่ติดมาตั้งแต่ต้น การยื่นแขนวางก็
            # ไม่มีอะไรหล่นลงพื้น จึงไม่มีสิ่งกีดขวางให้ต้องกันทาง
            had_object = payload is not None and payload.holding
            if payload is not None:
                # entry_travel = ระยะที่เพิ่งเดินเข้าช่องนี้มา ซึ่งเป็นพื้นที่
                # เดียวข้างหลังที่หุ่นเพิ่งผ่านมาเองแล้วว่าโล่งจริง แผนที่บอก
                # ไม่ได้ เพราะ goal ถูกเช็คก่อน observe() ที่ช่องนี้
                heading = place_on_target(driver, payload, heading,
                                          entry_travel, hub=hub, maze=maze,
                                          cell=(x, y))

            # ปล่อยของไม่ออก = _lower_and_release คาแขนไว้ที่ท่าวางและกางนิ้ว
            # ค้างไว้ให้คนมาหยิบออก เดินทั้งท่านั้นคือลากแขนที่ยื่นสุดไปครูด
            # กำแพง จบตรงนี้ดีกว่าเดินกลับ
            arm_stuck_out = payload is not None and payload.holding
            if arm_stuck_out:
                print("[WARN] ปล่อยวัตถุไม่ออก แขนยังยื่นค้างอยู่ที่ท่าวาง "
                      "จึงไม่เดินกลับ ให้เอาวัตถุออกจากมือก่อน")

            # ของหลุดมือลงพื้นแล้ว = มีสิ่งกีดขวางเพิ่มมาหนึ่งชิ้นในทิศที่เพิ่ง
            # ยื่นแขนไป จดตอนนี้เลยเพราะเป็นจังหวะเดียวที่รู้ตำแหน่งมันแน่นอน
            # ไม่ต้องรอให้ ToF หรือ Sharp เจอ ซึ่งมันไม่เจออยู่แล้วเพราะทั้งคู่
            # ติดอยู่ในระดับกำแพง ส่วนของกองอยู่กับพื้น
            if had_object and not arm_stuck_out:
                maze.mark_object(x, y, heading,
                                 block_cell=config.PLACED_OBJECT_BLOCKS_NEXT_CELL)
                print("[MAP] ของอยู่ทาง {0} ของช่อง {1} กันไม่ให้ขากลับเดินทับ{2}"
                      .format(DIR_NAMES[heading], (x, y),
                              " (กันทั้งช่อง {0})".format(
                                  (x + DX[heading], y + DY[heading]))
                              if config.PLACED_OBJECT_BLOCKS_NEXT_CELL else ""))

            if return_cell is not None and not arm_stuck_out:
                maze.goals = [return_cell]
                heading = face_way_back(driver, heading, entry_heading)
                entry_travel = 0.0      # หันแล้ว ข้างหลังไม่ใช่ทางที่เพิ่งผ่าน
                print("[PLAN] วางของแล้ว เดินกลับไปที่ช่อง {0}"
                      .format(return_cell))
                continue

            print(maze.render(robot=(x, y, heading), legend=True))
            return True

        if (x, y) in maze.goals:
            print("\n[HOME] กลับถึงช่อง {0} แล้ว เดินทั้งหมด {1} ช่อง"
                  .format((x, y), moves))
            driver.stop()
            print(maze.render(robot=(x, y, heading), legend=True))
            return True

        print("\n--- ก้าวที่ {0} | ช่อง ({1}, {2}) | หัน {3} ---"
              .format(step, x, y, DIR_NAMES[heading]))

        driver.stop()
        time.sleep(config.SETTLE_S)
        snap = hub.snapshot()
        if not snap.fresh:
            print("[ERROR] เซนเซอร์ {0} ขาดการอัปเดต หยุดเพื่อความปลอดภัย"
                  .format(snap.stale_reason))
            return False

        front, left, right, tof_mm = hub.read_walls_settled()
        print("ค่าดิบ -> ToF:{0} SharpL:{1} SharpR:{2} IR_L:{3} IR_R:{4}"
              .format(snap.tof_mm, snap.adc_left, snap.adc_right,
                      snap.ir_left, snap.ir_right))
        print("กำแพง -> หน้า:{0:d} ซ้าย:{1:d} ขวา:{2:d}"
              .format(front, left, right))

        # เทียบระยะที่วัดได้กับที่แผนที่ทำนาย ก่อนจะเขียนอะไรลงแผนที่
        #
        # ทำได้ฟรีตรงนี้ เพราะหุ่นจอดนิ่งอยู่แล้วและเพิ่งอ่าน ToF ไปแล้วในชุด
        # เดียวกับที่โหวตกำแพง ไม่ต้องหมุนเพิ่มและไม่ต้องอ่านเซนเซอร์ซ้ำ
        #
        # ด่านนี้จับสิ่งที่ไม่มีอะไรอื่นในโปรแกรมจับได้เลย คือ "หุ่นนับช่องพลาด"
        # ถ้าล้อลื่นหรือ odometry เพี้ยนจนหุ่นคิดว่าอยู่ช่องหนึ่งแต่จริง ๆ อยู่อีก
        # ช่อง ทุกอย่างหลังจากนั้นยังทำงานได้ปกติทุกประการ แค่เขียนแผนที่ผิดที่
        # ไปเรื่อย ๆ ซึ่งลบออกไม่ได้ (ดู Maze.set_wall) กว่าจะรู้ตัวก็ตอนหุ่นวิ่ง
        # ชนกำแพงที่แผนที่บอกว่าเป็นทางโล่ง
        #
        # ทำนายไม่ได้ = ยังไม่เคยเห็นด้านนั้น ซึ่งเป็นเรื่องปกติของช่องที่เพิ่งมา
        # ถึงครั้งแรก ด่านจึงเงียบไปเองแล้วมาทำงานตอนเดินซ้ำที่เดิม (ขากลับ หรือ
        # ตอนถอยออกจากทางตัน) กับตอนหันเข้าหาขอบสนามซึ่งรู้ตั้งแต่ยังไม่ออกเดิน
        trusted, why = observation_is_trusted(maze, x, y, heading, tof_mm)
        if trusted:
            maze.observe(x, y, heading, front, left, right)
        else:
            print("[WARN] {0}".format(why))
            print("       ไม่บันทึกกำแพงรอบนี้ลงแผนที่ - ถ้าหุ่นไม่ได้อยู่ช่อง "
                  "({0}, {1}) จริง สิ่งที่เขียนลงไปจะผิดที่และลบออกไม่ได้"
                  .format(x, y))

        # ขากลับไม่เดาว่าด้านที่ยังไม่เคยเห็นเป็นทางเปิด ใช้เฉพาะที่ตรวจแล้ว
        # ซึ่งครอบคลุมทุกขอบที่เดินผ่านมาแล้วเสมอ ทางกลับจึงมีอยู่แน่ ๆ อย่างน้อย
        # เท่ากับเส้นทางขาไป ถ้าไม่มีจริง ๆ (แผนที่โดนมาร์กกำแพงผิดระหว่างทาง)
        # ค่อยยอมเดาเหมือนขาไป ดีกว่ายืนตายอยู่ตรงนั้น
        known_only = config.RETURN_KNOWN_ONLY and not place_pending
        dist = maze.flood(known_only=known_only)
        if known_only and dist[x][y] >= INF:
            print("[PLAN] ทางกลับที่ยืนยันแล้วตันหมด - ยอมใช้ด้านที่ยังไม่เคย"
                  "ตรวจเหมือนขาไป")
            known_only = False
            dist = maze.flood()
        print(maze.render(dist=dist, robot=(x, y, heading), legend=True))

        if dist[x][y] >= INF:
            # บอกด้วยว่าตันตอนขาไหน เพราะขากลับใช้ลูปเดียวกันและพิมพ์ที่เดียวกัน
            print("\n[FAIL] จากความรู้ปัจจุบัน ไป{0}ไม่ได้แล้ว "
                  "(ทุกทางที่รู้จักถูกกำแพงปิดหมด)"
                  .format("เป้าหมาย" if place_pending else "ช่องที่จะกลับไป"))
            if maze.blocked:
                print("       ช่องที่กันไว้เพราะมีของวางอยู่: {0}"
                      .format(sorted(maze.blocked)))
            return False

        next_heading = maze.choose_next_heading(x, y, heading, dist,
                                                known_only=known_only)
        if next_heading is None:
            print("\n[FAIL] ช่องนี้ถูกล้อมทุกด้าน ออกไปไหนไม่ได้")
            return False
        print("ตัดสินใจ -> distance ที่นี่ = {0}, เดินไปทาง {1}"
              .format(dist[x][y], DIR_NAMES[next_heading]))

        heading = driver.turn_to(heading, next_heading)
        # หมุนแล้วข้างหลังไม่ใช่ทางที่เพิ่งผ่านมาอีกต่อไป ล้างเพดานการถอยทิ้ง
        # แล้วให้ advance_one_cell ที่สำเร็จเป็นตัวตั้งค่าใหม่
        entry_travel = 0.0

        # ตรวจ ToF อีกครั้งหลังหมุนเสร็จ ก่อนออกตัวจริง เป็นด่านสุดท้ายที่กัน
        # ไม่ให้พุ่งชนกำแพง และเป็นตัวแก้ให้อัตโนมัติเมื่อ Sharp อ่านพลาดว่าโล่ง
        time.sleep(0.15)
        snap = hub.snapshot()
        if snap.front_wall():
            print("[SAFETY] หันมาแล้วเจอกำแพงที่ {0}mm - ยกเลิกการเดิน "
                  "แล้วมาร์กลงแผนที่".format(snap.tof_mm))
            maze.set_wall(x, y, heading, True)
            continue

        ok, traveled, _ = driver.advance_one_cell(heading)
        if ok:
            x, y = x + DX[heading], y + DY[heading]
            moves += 1
            entry_travel = traveled
            entry_heading = heading
            fail_key = None
            fail_count = 0
        else:
            key = (x, y, heading)
            fail_count = fail_count + 1 if key == fail_key else 1
            fail_key = key

            snap = hub.snapshot()
            if snap.front_wall():
                print("[RECOVER] ยืนยันด้วย ToF ว่ามีกำแพงจริง มาร์กลงแผนที่")
                maze.set_wall(x, y, heading, True)
            elif fail_count >= 2:
                print("[RECOVER] เดินไม่ผ่านทางเดิมเป็นครั้งที่ {0} ทั้งที่ ToF "
                      "ว่าโล่ง - ปิดทางนี้ไว้ก่อนเพื่อไม่ให้ติดวนอยู่ที่เดิม"
                      .format(fail_count))
                maze.set_wall(x, y, heading, True)
            else:
                print("[RECOVER] ToF บอกว่าข้างหน้าโล่ง แต่เดินไม่ไป "
                      "น่าจะล้อลื่นหรือติดขัด - ลองใหม่อีกครั้งก่อนตัดสิน")
            driver.backup(traveled, heading)

    print("\n[FAIL] ครบ {0} ก้าวแล้วยังไม่ถึงเป้าหมาย".format(config.MAX_STEPS))
    return False

# -*- coding: utf-8 -*-
"""โหมด --sim : ทดสอบตรรกะ Flood Fill โดยไม่ต้องต่อหุ่น"""
from . import config
from .directions import DIR_NAMES, DX, DY, INF
from .geometry import place_heading_for
from .maze import Maze


#: list: คู่ช่องที่มีกำแพงกั้นระหว่างกันในเขาวงกตจำลอง
#:
#: หุ่นเดินถึงเป้าหมายใน 5 ก้าวตามเส้นทาง (0,0) -> (0,1) -> (1,1) -> (2,1) ->
#: (2,2) -> (2,3) ซึ่งเป็นเส้นทางที่สั้นที่สุดจริงของเขาวงกตนี้ ไม่มีการย้อนกลับ
#:
#: สิ่งที่ทดสอบได้จริง
#:   * การวางแผนใหม่เมื่อความรู้เปลี่ยน - ก้าวที่ 1 ที่ (0,1) ทางตรงที่ Flood Fill
#:     เดาไว้ว่าโล่งกลับมีกำแพง distance ของทิศตรงจึงพุ่งขึ้น ต้องเลี้ยวขวาแทน
#:   * การยอมหมุนเมื่อคุ้ม - ก้าวที่ 3 ที่ (2,1) ทางตรงไปต่อได้ แต่ทางซ้ายมี
#:     distance ต่ำกว่าอย่างเคร่งครัด ตัวเลือกจึงต้องข้ามความชอบเดินตรงไป
#:   * การอ่านกำแพงครบทั้งสามด้านของ observe() - กำแพง 5 ใน 6 เส้นถูกพบจริง
#:     โดยกระจายกันทั้งด้านหน้า ซ้าย และขวา จบด้วยสำรวจไป 29/40 ด้าน
#:
#: สิ่งที่ยังไม่ถูกทดสอบ
#:   เส้นทางนี้ไม่มีทางตันเลย ตรรกะการถอยออกจากทางตันและ recovery ทั้งหมดใน
#:   run_search จึงไม่ถูกแตะ ส่วนกำแพง (3,1)-(3,2) อยู่นอกเส้นทาง หุ่นไม่เคย
#:   เดินไปเห็น เปลี่ยนหรือลบทิ้งได้โดยผลการรันไม่เปลี่ยน
SIM_BLOCKED_EDGES = [
    ((0, 0), (1, 0)),
    ((0, 1), (0, 2)),
    ((1, 1), (1, 2)),
    ((2, 0), (2, 1)),
    ((1, 2), (2, 2)),
    ((3, 1), (3, 2)),
]


def _edge_direction(cell_a, cell_b):
    """int: ทิศจาก cell_a ไป cell_b"""
    for direction in range(4):
        if (cell_a[0] + DX[direction], cell_a[1] + DY[direction]) == tuple(cell_b):
            return direction
    raise ValueError("{0} กับ {1} ไม่ได้ติดกัน".format(cell_a, cell_b))


def run_sim():
    """เดินตรรกะเดียวกับของจริงบนเขาวงกตจำลอง

    ใช้ ``Maze`` และ ``choose_next_heading`` ตัวเดียวกับที่หุ่นใช้จริง ต่างกันแค่
    แทนที่จะอ่านเซนเซอร์ ก็ไปถามเขาวงกตความจริงตรง ๆ จึงยืนยันได้ว่าตรรกะการ
    วางแผนถูกต้อง ก่อนเอาไปเจอกับความไม่แน่นอนของเซนเซอร์และล้อในสนามจริง

    จำลองครบทุกเฟสเหมือน ``run_search`` คือไปหยิบของ ไปวางของ แล้วเดินกลับ
    ส่วนการคีบและการวางจริงไม่มีอะไรให้จำลอง

    Returns:
        bool: True เมื่อเดินครบทุกเฟส
    """
    truth = Maze(config.MAZE_W, config.MAZE_H, config.GOAL_CELLS)
    for cell_a, cell_b in SIM_BLOCKED_EDGES:
        truth.set_wall(cell_a[0], cell_a[1], _edge_direction(cell_a, cell_b), True)

    print("=" * 62)
    print("  โหมดจำลอง - ไม่ต้องต่อหุ่น")
    print("=" * 62)
    print("\nเขาวงกตความจริง (หุ่นยังไม่รู้):")
    print(truth.render())

    known = Maze(config.MAZE_W, config.MAZE_H, config.GOAL_CELLS)
    x, y = config.START_CELL
    heading = config.START_HEADING
    path = [(x, y)]

    # จำลองการสลับเป้าหมายทุกเฟสด้วย เพื่อให้ตรวจแผนการเดินได้ก่อนลงสนามจริง
    return_cell = tuple(config.RETURN_CELL) if config.RETURN_CELL is not None else None
    place_pending = True
    pick_pending = config.PICK_CELL is not None
    if pick_pending:
        known.goals = [tuple(config.PICK_CELL)]
        if tuple(config.PICK_CELL) != tuple(config.START_CELL):
            print("\n[PLAN] เฟส 1 ไปหยิบของที่ช่อง {0} แล้วค่อยไปเป้าหมาย {1}"
                  .format(tuple(config.PICK_CELL), config.GOAL_CELLS))

    for step in range(config.MAX_STEPS):
        if (x, y) in known.goals and pick_pending:
            print("\n[PICK] หยิบของที่ช่อง {0} หลังเดินมา {1} ช่อง (จำลอง)"
                  .format((x, y), len(path) - 1))
            known.goals = list(config.GOAL_CELLS)
            pick_pending = False
            print("[PLAN] มุ่งหน้าไปเป้าหมาย {0}".format(known.goals))
            continue

        if (x, y) in known.goals:
            if place_pending:
                print("\n[GOAL] ถึงเป้าหมายใน {0} ช่อง".format(len(path) - 1))
                place_pending = False
                # จำลองการวางของด้วย เพราะของที่วางเป็นสิ่งกีดขวางที่มีผลกับ
                # แผนขากลับโดยตรง ถ้าไม่จำลอง แผนที่ตรวจก่อนลงสนามก็ไม่ใช่แผน
                # เดียวกับที่หุ่นจะเดินจริง
                place_heading = place_heading_for(heading)
                known.mark_object(x, y, place_heading,
                                  block_cell=config.PLACED_OBJECT_BLOCKS_NEXT_CELL)
                print("[MAP] วางของทาง {0} ของช่อง {1}"
                      .format(DIR_NAMES[place_heading], (x, y)))
                if len(path) > 1:
                    # face_way_back หันหลังให้ของ = หันไปทางที่เพิ่งเดินเข้ามา
                    # ถ้ายังไม่เคยเดินเลย ก็ไม่มีด้านไหนที่ยืนยันแล้วให้หันไปหา
                    heading = (heading + 2) % 4
                if return_cell is not None:
                    known.goals = [return_cell]
                    print("[PLAN] วางของแล้ว เดินกลับไปที่ช่อง {0}"
                          .format(return_cell))
                    continue
            else:
                print("\n[HOME] กลับถึงช่อง {0} แล้ว เดินทั้งหมด {1} ช่อง"
                      .format((x, y), len(path) - 1))
            print("เส้นทางที่เดินจริง: {0}".format(
                " -> ".join(str(cell) for cell in path)))
            print("\nแผนที่ที่หุ่นสร้างได้:")
            print(known.render(dist=known.flood(), robot=(x, y, heading),
                               legend=True))
            return True

        front = truth.has_wall(x, y, heading)
        right = truth.has_wall(x, y, (heading + 1) % 4)
        left = truth.has_wall(x, y, (heading + 3) % 4)
        known.observe(x, y, heading, front, left, right)

        known_only = config.RETURN_KNOWN_ONLY and not place_pending
        dist = known.flood(known_only=known_only)
        if known_only and dist[x][y] >= INF:
            print("[PLAN] ทางกลับที่ยืนยันแล้วตันหมด - ยอมใช้ด้านที่ยังไม่เคยตรวจ")
            known_only = False
            dist = known.flood()
        print("\n--- ก้าวที่ {0} | ช่อง ({1}, {2}) | หัน {3} | distance {4} ---"
              .format(step, x, y, DIR_NAMES[heading], dist[x][y]))
        print(known.render(dist=dist, robot=(x, y, heading), legend=True))

        if dist[x][y] >= INF:
            print("\n[FAIL] ไปเป้าหมายไม่ได้แล้ว")
            return False

        next_heading = known.choose_next_heading(x, y, heading, dist,
                                                 known_only=known_only)
        if next_heading is None:
            print("\n[FAIL] ถูกล้อมทุกด้าน")
            return False

        # ตรวจความถูกต้องของตรรกะ: ทิศที่เลือกต้องไม่มีกำแพงอยู่จริง
        if truth.has_wall(x, y, next_heading):
            print("\n[BUG] เลือกเดินไปทาง {0} ทั้งที่มีกำแพงจริงอยู่"
                  .format(DIR_NAMES[next_heading]))
            return False

        # และต้องไม่เดินทับของที่เพิ่งวางไว้เอง
        if (x, y, next_heading) in known.objects:
            print("\n[BUG] เลือกเดินไปทาง {0} ทั้งที่วางของขวางไว้ตรงนั้น"
                  .format(DIR_NAMES[next_heading]))
            return False

        heading = next_heading
        x, y = x + DX[heading], y + DY[heading]
        path.append((x, y))

    print("\n[FAIL] ครบ {0} ก้าวแล้วยังไม่ถึงเป้าหมาย".format(config.MAX_STEPS))
    return False

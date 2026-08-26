# -*- coding: utf-8 -*-
"""ทดสอบ Maze เฉพาะส่วนที่ใช้หลบของที่หุ่นวางไว้เอง และการวางแผนขากลับ

เป็นเทสต์ระดับหน่วย ไม่ต้องมีหุ่นเลยแม้แต่ตัวปลอม เพราะ Maze เป็นข้อมูลกับ
อัลกอริทึมล้วน ๆ ส่วนการต่อสายเข้ากับ run_search อยู่ใน test_run_search.py
"""
import sys

from fakes import Checker, load


def _blank(tc, goals=None):
    """Maze เปล่าที่มีแต่กำแพงขอบสนาม

    Returns:
        Maze: แผนที่ที่ยังไม่เคยตรวจด้านในเลยสักด้าน
    """
    return tc.Maze(tc.MAZE_W, tc.MAZE_H, goals or [(0, 0)])


def _open_path(tc, maze, cells):
    """ทำให้ทุกขอบระหว่างช่องที่ไล่มาเป็น "ตรวจแล้วโล่ง"

    เลียนแบบสิ่งที่ observe() ทิ้งไว้หลังหุ่นเดินผ่านเส้นทางหนึ่ง คือขอบที่ข้าม
    ไปถูกมองเห็นเป็นด้านหน้ามาก่อนเสมอ จึงถูกมาร์กว่ารู้แล้วทั้งสองฝั่ง
    """
    for cell_a, cell_b in zip(cells, cells[1:]):
        maze.set_wall(cell_a[0], cell_a[1],
                      tc._edge_direction(cell_a, cell_b), False)


def test_mark_object_is_not_a_wall(chk):
    """ของที่วางต้องกันทางได้ โดยไม่ไปปนกับกำแพงจริงของสนาม

    สองอย่างนี้ต้องแยกกัน เพราะกำแพงคือของจริงที่ต้องรายงานให้ตรงสนาม ส่วนของ
    ที่วางเป็นสิ่งที่หุ่นเอาไปวางเอง แผนที่ที่พิมพ์ออกมาจึงต้องบอกแยกกันได้
    """
    chk.section("Maze: ของที่วางไม่ใช่กำแพง")

    tc = load()
    maze = _blank(tc)
    maze.mark_object(4, 4, tc.WEST)

    chk.check("จดขอบที่วางของไว้", (4, 4, tc.WEST) in maze.objects)
    chk.check("จดขอบเดียวกันจากฝั่งช่องข้างเคียงด้วย",
              (3, 4, tc.EAST) in maze.objects)
    chk.check("กันช่องที่ของไปตกทั้งช่อง", (3, 4) in maze.blocked)
    chk.check("ไม่ได้ไปเพิ่มกำแพงลงแผนที่", maze.has_wall(4, 4, tc.WEST) is False)
    chk.check("ไม่ได้ทำให้ด้านนั้นกลายเป็น 'ตรวจแล้ว'",
              maze.is_known(4, 4, tc.WEST) is False)
    chk.check("วางแผนเดินข้ามไปไม่ได้",
              maze.passable(4, 4, tc.WEST) is False)
    chk.check("เดินสวนกลับมาก็ไม่ได้", maze.passable(3, 4, tc.EAST) is False)
    chk.check("ทิศอื่นของช่องเดิมยังเดินได้",
              maze.passable(4, 4, tc.SOUTH) is True)

    # เซนเซอร์อ่านทีหลังว่าด้านนั้นโล่ง (ซึ่งจริง - ของเตี้ยกว่ากำแพง Sharp
    # จึงเห็นทะลุไป) ต้องลบของที่จดไว้ทิ้งไม่ได้
    maze.set_wall(4, 4, tc.WEST, False)
    chk.check("Sharp อ่านว่าโล่งทีหลัง ก็ลบของทิ้งไม่ได้",
              maze.passable(4, 4, tc.WEST) is False)

    maze = _blank(tc)
    maze.mark_object(4, 4, tc.WEST, block_cell=False)
    chk.check("block_cell=False: กันแค่ขอบ", maze.passable(4, 4, tc.WEST) is False)
    chk.check("block_cell=False: ไม่กันทั้งช่อง", maze.blocked == set())

    maze = _blank(tc)
    maze.mark_object(0, 0, tc.WEST)                     # ยื่นแขนใส่กำแพงขอบสนาม
    chk.check("วางชนขอบสนาม: ไม่พังและไม่กันช่องนอกสนาม", maze.blocked == set())


def test_flood_walks_around_the_object(chk):
    """Flood Fill ต้องอ้อมของที่วางไว้ ไม่ใช่ลากเส้นทับมัน"""
    chk.section("Maze: flood อ้อมของที่วาง")

    tc = load()
    maze = _blank(tc, goals=[(0, 0)])
    before = maze.flood()[4][4]

    maze.mark_object(4, 4, tc.WEST)
    after = maze.flood()
    chk.check("ช่องที่มีของกองอยู่ ไปไม่ถึงอีกต่อไป", after[3][4] >= tc.INF)
    chk.check("ช่องที่หุ่นยืนยังไปถึงบ้านได้", after[4][4] < tc.INF)
    chk.check("ในสนามโล่ง อ้อมแล้วระยะเท่าเดิม ({0} -> {1})"
              .format(before, after[4][4]), after[4][4] == before)

    heading = tc.NORTH                                  # หันหลังให้ของแล้ว
    chosen = maze.choose_next_heading(4, 4, heading, after)
    chk.check("ไม่เลือกเดินไปทางที่วางของไว้", chosen != tc.WEST)
    chk.check("เลือกทางที่เหลือที่เข้าใกล้บ้านที่สุด", chosen == tc.SOUTH)

    # ปิดทางใต้ด้วย เหลือทางเดียวที่สั้นคือทับของ - ต้องยอมอ้อมไกลขึ้น ไม่ใช่
    # ยอมเดินทับ ระยะที่ยาวขึ้นคือราคาที่จ่ายเพื่อไม่ให้ของโดนเขี่ย
    maze = _blank(tc, goals=[(0, 0)])
    maze.set_wall(4, 4, tc.SOUTH, True)
    short = maze.flood()[4][4]
    maze.mark_object(4, 4, tc.WEST)
    detour = maze.flood()[4][4]
    chk.check("ทางสั้นถูกของทับ: ยังกลับได้ แต่ไกลขึ้น ({0} -> {1})"
              .format(short, detour), tc.INF > detour > short)
    chk.check("ทางสั้นถูกของทับ: เลี่ยงไปทางเหนือแทน",
              maze.choose_next_heading(4, 4, tc.NORTH, maze.flood()) == tc.NORTH)

    # เป้าหมายเองถูกของทับ = ไปไม่ถึงจริง ๆ ต้องคืน INF ไม่ใช่ลากเส้นทะลุของ
    maze = _blank(tc, goals=[(3, 4)])
    maze.mark_object(4, 4, tc.WEST)
    chk.check("เป้าหมายถูกของทับ: ไปไม่ถึง", maze.flood()[4][4] >= tc.INF)


def test_known_only_planning(chk):
    """known_only ต้องเดินเฉพาะด้านที่ตรวจแล้ว ไม่เดาว่าด้านที่ไม่เคยเห็นเปิด"""
    chk.section("Maze: วางแผนเฉพาะด้านที่ตรวจแล้ว")

    tc = load()
    maze = _blank(tc, goals=[(0, 0)])
    optimistic = maze.flood()
    strict = maze.flood(known_only=True)
    chk.check("แบบเดิม: แผนที่เปล่าก็ไปได้ทุกช่อง (เดาว่าเปิด)",
              optimistic[4][4] < tc.INF)
    chk.check("known_only: แผนที่เปล่าไปไม่ได้เลย",
              strict[4][4] >= tc.INF)
    chk.check("known_only: ช่องเป้าหมายเองยังเป็น 0", strict[0][0] == 0)

    # เปิดทางเดินหนึ่งเส้นแบบที่ observe() ทำให้ตอนหุ่นเดินผ่าน
    route = [(0, 0), (0, 1), (1, 1), (2, 1), (2, 2)]
    _open_path(tc, maze, route)
    strict = maze.flood(known_only=True)
    chk.check("known_only: ปลายทางที่เคยเดินผ่านไปถึงได้",
              strict[2][2] == len(route) - 1)
    chk.check("known_only: ช่องที่ไม่เคยไปยังไปไม่ถึง",
              strict[4][4] >= tc.INF)
    chk.check("known_only: ทางลัดที่ยังไม่เคยตรวจไม่ถูกนับ",
              strict[1][0] >= tc.INF and optimistic[1][0] < tc.INF)

    chosen = maze.choose_next_heading(2, 2, tc.NORTH, strict, known_only=True)
    chk.check("known_only: เลือกย้อนทางที่เคยเดินมา", chosen == tc.SOUTH)
    chk.check("แบบเดิม: เลือกทางที่ยังไม่เคยตรวจได้",
              maze.choose_next_heading(2, 2, tc.NORTH, optimistic) == tc.WEST)


def test_render_marks_the_object(chk):
    """แผนที่ที่พิมพ์ออกมาต้องบอกได้ว่าตรงไหนคือของที่วางไว้เอง"""
    chk.section("Maze: แผนที่แสดงของที่วาง")

    tc = load()
    maze = _blank(tc)
    plain = maze.render(legend=True)
    maze.mark_object(4, 4, tc.WEST)
    marked = maze.render(legend=True)

    chk.check("ยังไม่มีของ: ไม่มีคำอธิบายสัญลักษณ์ของ",
              "ของที่วางไว้เอง" not in plain)
    chk.check("มีของแล้ว: มีคำอธิบายสัญลักษณ์", "ของที่วางไว้เอง" in marked)
    chk.check("มีของแล้ว: วาดสัญลักษณ์ลงแผนที่", "o" in marked)
    chk.check("จำนวนบรรทัดเท่าเดิม ไม่ทำให้แผนที่เพี้ยน",
              len(marked.split("\n")) == len(plain.split("\n")) + 1)


def run(chk=None):
    """รันทุกเทสต์ในไฟล์นี้

    Returns:
        bool: True เมื่อผ่านหมด
    """
    own = chk is None
    chk = chk or Checker()
    test_mark_object_is_not_a_wall(chk)
    test_flood_walks_around_the_object(chk)
    test_known_only_planning(chk)
    test_render_marks_the_object(chk)
    return chk.report() if own else True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)

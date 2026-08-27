# -*- coding: utf-8 -*-
"""แผนที่กำแพงและตัวคำนวณ Flood Fill"""
from collections import deque

from . import config
from .directions import DIR_ARROWS, DX, DY, EAST, INF, NORTH, SOUTH, WEST


class Maze(object):
    """แผนที่กำแพงและตัวคำนวณ Flood Fill

    กำแพงเก็บเป็น bitmask ต่อช่อง โดยบิตที่ i หมายถึงกำแพงทางทิศ i
    (0=N 1=E 2=S 3=W) และเก็บ ``known`` แยกไว้อีกชุดเพื่อบอกว่าด้านนั้น
    "เคยเห็นมาแล้วจริง" หรือ "ยังไม่เคยไปดู"

    Flood Fill จะถือว่าด้านที่ยังไม่เคยเห็นเป็นทางเปิดไว้ก่อน ซึ่งเป็นหลักการของ
    micromouse คือมองโลกในแง่ดีแล้วเดินไปแก้เอาข้างหน้า ทำให้ไม่ต้องเสียเวลา
    สำรวจช่องที่พิสูจน์แล้วว่าไม่มีทางอยู่บนเส้นทางที่ดีที่สุด

    Args:
        width (int): จำนวนช่องแกน x
        height (int): จำนวนช่องแกน y
        goals (list): รายการช่องเป้าหมาย [(x, y), ...]
    """

    def __init__(self, width, height, goals):
        self.width = width
        self.height = height
        self.goals = [tuple(g) for g in goals]
        self.walls = [[0] * height for _ in range(width)]
        self.known = [[0] * height for _ in range(width)]
        #: set: {(x, y, direction)} ขอบที่มีของวางขวางอยู่ เก็บแยกจาก walls
        #: เพราะไม่ใช่กำแพงของสนาม เป็นของที่หุ่นเอาไปวางเองแล้วต้องหลบ แยกไว้
        #: แล้วแผนที่ที่ render ออกมายังตรงกับสนามจริง และการที่ Sharp อ่านด้าน
        #: นั้นว่า "โล่ง" ในภายหลังก็ลบของทิ้งไม่ได้
        self.objects = set()
        #: set: {(x, y)} ช่องที่มีของกองอยู่ ห้ามวางแผนเดินเข้าไปทั้งช่อง
        self.blocked = set()
        self._add_borders()

    def _add_borders(self):
        """ใส่กำแพงขอบสนามรอบนอก ซึ่งรู้แน่นอนอยู่แล้วโดยไม่ต้องไปวัด"""
        for x in range(self.width):
            self.set_wall(x, 0, SOUTH, True)
            self.set_wall(x, self.height - 1, NORTH, True)
        for y in range(self.height):
            self.set_wall(0, y, WEST, True)
            self.set_wall(self.width - 1, y, EAST, True)

    def in_bounds(self, x, y):
        """bool: พิกัดนี้อยู่ในสนามหรือไม่"""
        return 0 <= x < self.width and 0 <= y < self.height

    def has_wall(self, x, y, direction):
        """bool: ช่อง (x, y) มีกำแพงทางทิศ direction หรือไม่"""
        return bool(self.walls[x][y] & (1 << direction))

    def is_known(self, x, y, direction):
        """bool: เคยตรวจด้านนี้ของช่องนี้แล้วหรือยัง"""
        return bool(self.known[x][y] & (1 << direction))

    def set_wall(self, x, y, direction, present):
        """บันทึกผลการตรวจกำแพงหนึ่งด้าน พร้อมอัปเดตช่องข้างเคียงให้สอดคล้องกัน

        เมื่อ present เป็น True จะติดบิตกำแพง แต่เมื่อเป็น False จะไม่ลบบิตที่เคย
        ติดไว้แล้ว เพราะกำแพงจริงไม่เดินหนีไปไหน การอ่านพลาดครั้งเดียวจึงไม่ควร
        ลบสิ่งที่เคยยืนยันแล้วออกจากแผนที่ แผนที่จะเพิ่มกำแพงได้อย่างเดียว
        ซึ่งทำให้ผลของ Flood Fill นิ่งและไม่แกว่งไปมาระหว่างรอบ
        """
        if not self.in_bounds(x, y):
            return
        if present:
            self.walls[x][y] |= (1 << direction)
        self.known[x][y] |= (1 << direction)

        nx, ny = x + DX[direction], y + DY[direction]
        if self.in_bounds(nx, ny):
            opposite = (direction + 2) % 4
            if present:
                self.walls[nx][ny] |= (1 << opposite)
            self.known[nx][ny] |= (1 << opposite)

    def mark_object(self, x, y, direction, block_cell=True):
        """จดว่ามีของวางอยู่ทางทิศ direction ของช่อง (x, y) แล้วกันไม่ให้เดินทับ

        เรียกตอนที่หุ่นเพิ่งปล่อยของลงพื้นเอง ซึ่งเป็นจังหวะเดียวที่รู้ตำแหน่งของ
        แน่นอนโดยไม่ต้องพึ่งเซนเซอร์ - แขนยื่นไปทางไหน ของก็อยู่ทางนั้น

        Args:
            x (int): ช่องที่หุ่นยืนตอนวาง
            y (int): ช่องที่หุ่นยืนตอนวาง
            direction (int): ทิศที่ยื่นแขนวาง
            block_cell (bool): True = กันช่องถัดไปทางนั้นทั้งช่องด้วย ไม่ใช่แค่
                ขอบ ใช้เมื่อระยะที่แขนยื่นทำให้ของไปตกคร่อมเส้นแบ่งช่อง
        """
        if not self.in_bounds(x, y):
            return
        self.objects.add((x, y, direction))
        nx, ny = x + DX[direction], y + DY[direction]
        if self.in_bounds(nx, ny):
            # จดขอบเดียวกันจากฝั่งช่องข้างเคียงด้วย ของชิ้นเดียวขวางทั้งสองทาง
            self.objects.add((nx, ny, (direction + 2) % 4))
            if block_cell:
                self.blocked.add((nx, ny))

    def passable(self, x, y, direction, known_only=False):
        """bool: วางแผนเดินออกจากช่อง (x, y) ไปทางทิศนี้ได้ไหม

        รวมทุกเหตุผลที่ห้ามเดินไว้ที่เดียว ทั้ง :meth:`flood` และ
        :meth:`choose_next_heading` ต้องถามผ่านตัวนี้เท่านั้น ไม่งั้นสองอันจะ
        ตอบไม่ตรงกันแล้วหุ่นจะเลือกทิศที่ Flood Fill ไม่เคยคิดว่าจะเดินไป

        Args:
            known_only (bool): True = ด้านที่ยังไม่เคยตรวจถือว่าเดินไม่ได้
                (ใช้ตอนขากลับ ที่ไม่มีอะไรให้สำรวจแล้ว) False = ถือว่าเปิดไว้ก่อน
                ตามหลัก micromouse (ใช้ตอนขาไป)
        """
        if self.has_wall(x, y, direction):
            return False
        if (x, y, direction) in self.objects:
            return False
        if known_only and not self.is_known(x, y, direction):
            return False
        return True

    def observe(self, x, y, heading, front, left, right):
        """บันทึกผลการตรวจกำแพงจากท่ายืนปัจจุบันลงแผนที่

        เซนเซอร์ให้ผลเป็น "หน้า/ซ้าย/ขวา" เทียบกับตัวหุ่น ต้องแปลงเป็นทิศสัมบูรณ์
        ของสนามก่อนตามทิศที่หุ่นหันอยู่
        """
        self.set_wall(x, y, heading, front)
        self.set_wall(x, y, (heading + 1) % 4, right)
        self.set_wall(x, y, (heading + 3) % 4, left)

    def predict_tof(self, x, y, direction):
        """float or None: ระยะที่ ToF ควรอ่านได้ ถ้ายืนกลางช่อง (x, y) หันไปทิศนี้

        ไล่ออกไปทีละช่องตามทิศที่ให้มา จนเจอกำแพงแรกที่ "เคยเห็นมาแล้วจริง" แล้ว
        แปลงจำนวนช่องเป็นมิลลิเมตร ช่องแรกอ่านได้ ``FRONT_STOP_MM`` ตามนิยาม
        ทุกช่องที่ไกลออกไปบวกเพิ่มช่องละ ``CELL_SIZE_M``

        มีไว้เทียบกับค่าที่วัดได้จริง เพื่อจับกรณีที่หุ่นไม่ได้อยู่ช่องที่คิดว่าอยู่
        หรือ ToF กำลังมองทะลุประตูไปเจออะไรที่ไม่ใช่กำแพงที่ตั้งใจวัด

        คืน None เมื่อยังไม่รู้จริง ๆ ซึ่งต่างจาก "รู้ว่าไม่มีกำแพง" - Flood Fill
        มองด้านที่ยังไม่เคยเห็นเป็นทางเปิดไว้ก่อนเพื่อกล้าเดิน แต่การทำนายระยะทำ
        แบบนั้นไม่ได้ เพราะจะได้ตัวเลขที่ดูสมเหตุสมผลแต่ไม่มีอะไรรองรับ แล้วผู้
        เรียกก็เอาไปเทียบราวกับเป็นความจริง

        เป็นเมธอดอ่านอย่างเดียว ไม่แตะแผนที่ ทุกอย่างที่เขียนลง ``walls`` ลบไม่ได้
        (ดู :meth:`set_wall`) การทำนายจึงต้องไม่มีผลข้างเคียงเด็ดขาด

        Args:
            x (int): พิกัดช่องที่ยืนอยู่
            y (int): พิกัดช่องที่ยืนอยู่
            direction (int): ทิศที่หันไป 0=N 1=E 2=S 3=W

        Returns:
            float or None: ระยะหน่วย mm None = แผนที่ยังไม่รู้พอจะทำนาย
        """
        cells = 0
        cx, cy = x, y
        while self.in_bounds(cx, cy):
            if (cx, cy, direction) in self.objects:
                # มีของที่หุ่นวางเองขวางอยู่ ToF เห็นของก่อนกำแพง แต่เราไม่รู้ว่า
                # ของสูงเท่าไรหรือวางเยื้องแค่ไหน ทำนายระยะจึงไม่ได้
                return None
            if not self.is_known(cx, cy, direction):
                return None
            if self.has_wall(cx, cy, direction):
                return config.FRONT_STOP_MM + cells * config.CELL_SIZE_M * 1000.0
            cx, cy = cx + DX[direction], cy + DY[direction]
            cells += 1
        return None

    def flood(self, known_only=False):
        """คำนวณระยะจากทุกช่องไปยังเป้าหมายที่ใกล้ที่สุด ด้วย BFS ย้อนจากเป้าหมาย

        Args:
            known_only (bool): ส่งต่อให้ :meth:`passable` True = นับเฉพาะด้านที่
                ตรวจแล้วว่าโล่งจริง ไม่เดาว่าด้านที่ยังไม่เคยเห็นเป็นทางเปิด

        Returns:
            list: ตาราง distance[x][y] ค่า INF แปลว่าไปไม่ถึงด้วยความรู้ปัจจุบัน
        """
        dist = [[INF] * self.height for _ in range(self.width)]
        queue = deque()
        for gx, gy in self.goals:
            if self.in_bounds(gx, gy) and (gx, gy) not in self.blocked:
                dist[gx][gy] = 0
                queue.append((gx, gy))

        while queue:
            cx, cy = queue.popleft()
            next_dist = dist[cx][cy] + 1
            for direction in range(4):
                if not self.passable(cx, cy, direction, known_only):
                    continue
                nx, ny = cx + DX[direction], cy + DY[direction]
                if not self.in_bounds(nx, ny) or (nx, ny) in self.blocked:
                    continue
                if dist[nx][ny] > next_dist:
                    dist[nx][ny] = next_dist
                    queue.append((nx, ny))
        return dist

    def choose_next_heading(self, x, y, heading, dist, known_only=False):
        """เลือกทิศถัดไปที่ควรเดิน

        ไล่ดูตามลำดับ ตรงไป -> ขวา -> ซ้าย -> หลัง แล้วเทียบ distance ด้วย ``<``
        อย่างเคร่งครัด ผลคือเมื่อหลายทิศมี distance เท่ากันจะได้ทิศที่มาก่อนใน
        ลำดับนี้ ซึ่งทำให้หุ่นชอบเดินตรงมากกว่าหมุน ประหยัดเวลาและลด yaw drift

        Args:
            known_only (bool): ต้องส่งค่าเดียวกับที่ใช้ตอนเรียก :meth:`flood`
                ให้ได้ ``dist`` มา ไม่งั้นจะเลือกทิศที่ตาราง distance คิดมาจาก
                กติกาคนละชุด

        Returns:
            int or None: ทิศที่เลือก หรือ None เมื่อไม่มีทางออกเลย
        """
        order = (heading, (heading + 1) % 4, (heading + 3) % 4, (heading + 2) % 4)
        best_dir = None
        best_dist = INF
        for direction in order:
            if not self.passable(x, y, direction, known_only):
                continue
            nx, ny = x + DX[direction], y + DY[direction]
            if not self.in_bounds(nx, ny):
                continue
            if dist[nx][ny] < best_dist:
                best_dist = dist[nx][ny]
                best_dir = direction
        return best_dir

    def edge_stats(self):
        """tuple: (จำนวนด้านที่ตรวจแล้ว, จำนวนด้านทั้งหมด)

        นับด้านละหนึ่งครั้ง โดยไล่เฉพาะทิศเหนือกับตะวันออกของทุกช่อง แล้วเติม
        ขอบใต้ของแถวล่างสุดและขอบตะวันตกของคอลัมน์ซ้ายสุด
        """
        total = 0
        seen = 0
        for x in range(self.width):
            for y in range(self.height):
                for direction in (NORTH, EAST):
                    total += 1
                    seen += 1 if self.is_known(x, y, direction) else 0
        for x in range(self.width):
            total += 1
            seen += 1 if self.is_known(x, 0, SOUTH) else 0
        for y in range(self.height):
            total += 1
            seen += 1 if self.is_known(0, y, WEST) else 0
        return seen, total

    def _edge_glyph(self, x, y, direction, horizontal):
        """str: สัญลักษณ์ของขอบหนึ่งด้าน แยกกำแพง / ของที่วาง / โล่ง / ยังไม่เคยดู"""
        if self.has_wall(x, y, direction):
            return "---" if horizontal else "|"
        if (x, y, direction) in self.objects:
            return "ooo" if horizontal else "o"
        if self.is_known(x, y, direction):
            return "   " if horizontal else " "
        return " . " if horizontal else ":"

    def render(self, dist=None, robot=None, legend=False):
        """วาดแผนที่เป็น ASCII

        ขอบที่ยังไม่เคยตรวจจะแสดงเป็นจุด เพื่อให้แยกออกจากขอบที่ยืนยันแล้วว่าโล่ง
        Flood Fill มองสองอย่างนี้เหมือนกันคือเดินผ่านได้ แต่ตอนอ่านแผนที่เรา
        ต้องแยกให้ออกว่าอะไรคือข้อมูลจริง อะไรคือการมองโลกในแง่ดีไว้ก่อน

        Args:
            dist (list): ตาราง distance จาก :meth:`flood` ใส่ None ได้
            robot (tuple): (x, y, heading) ตำแหน่งหุ่น ใส่ None ได้
            legend (bool): ต่อท้ายด้วยคำอธิบายสัญลักษณ์และความคืบหน้าการสำรวจ

        Returns:
            str: แผนที่หลายบรรทัด พร้อม print ได้เลย
        """
        lines = []
        for y in range(self.height - 1, -1, -1):
            top = "+"
            for x in range(self.width):
                top += self._edge_glyph(x, y, NORTH, True) + "+"
            lines.append(top)

            mid = self._edge_glyph(0, y, WEST, False)
            for x in range(self.width):
                if robot is not None and (robot[0], robot[1]) == (x, y):
                    cell = " {0} ".format(DIR_ARROWS[robot[2]])
                elif (x, y) in self.blocked:
                    cell = " o "
                elif (x, y) in self.goals:
                    cell = " G "
                elif dist is not None:
                    cell = "   " if dist[x][y] >= INF else "{0:3d}".format(dist[x][y])
                else:
                    cell = "   "
                mid += cell + self._edge_glyph(x, y, EAST, False)
            lines.append(mid)

        bottom = "+"
        for x in range(self.width):
            bottom += self._edge_glyph(x, 0, SOUTH, True) + "+"
        lines.append(bottom)

        if legend:
            seen, total = self.edge_stats()
            lines.append("--- = กำแพง | ว่าง = ตรวจแล้วโล่ง | . = ยังไม่เคยตรวจ"
                         "   (สำรวจแล้ว {0}/{1} ด้าน)".format(seen, total))
            if self.objects:
                lines.append("o = ของที่วางไว้เอง ไม่ใช่กำแพง แต่ห้ามเดินทับ")
        return "\n".join(lines)

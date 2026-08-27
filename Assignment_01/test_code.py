# -*- coding: utf-8 -*-
"""Assignment 01 - Maze Solver ด้วย Flood Fill (Micromouse) สำหรับ RoboMaster EP

ไฟล์นี้เป็นแค่จุดเริ่มโปรแกรม เนื้อทั้งหมดอยู่ในแพ็กเกจ ``maze_solver/``
ซึ่งอธิบายวิธีทำงานและโครงสร้างโมดูลไว้ที่ ``maze_solver/__init__.py``

วิธีใช้
------
    python test_code.py --calib    วัดค่าเซนเซอร์จริง (ต้องทำก่อนใช้งานครั้งแรก)
    python test_code.py --armtest  จูนท่าแขนกับวัตถุจริง หุ่นไม่เดินไปไหน
    python test_code.py --sim      ทดสอบตรรกะ Flood Fill โดยไม่ต้องต่อหุ่น
    python test_code.py            วิ่งจริงในสนาม
    python tests/run_tests.py      รันเทสต์ทั้งหมดด้วยหุ่นปลอม

ค่าคงที่ทั้งหมดที่ต้องแก้อยู่ที่ ``maze_solver/config.py`` ที่เดียว
"""
import os
import sys

# รันสคริปต์นี้จากที่ไหนก็ได้ Python วางโฟลเดอร์ของสคริปต์ไว้ต้น sys.path ให้แล้ว
# แต่กรณีที่ถูกเรียกผ่านทางอื่น (เช่น exec) จะไม่มี ใส่เผื่อไว้ให้หา maze_solver เจอ
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maze_solver.cli import main                        # noqa: E402

if __name__ == "__main__":
    sys.exit(main())

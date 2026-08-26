# -*- coding: utf-8 -*-
"""รันเทสต์ทั้งหมดของ test_code.py ด้วยหุ่นปลอม ไม่ต้องต่อฮาร์ดแวร์

    python tests/run_tests.py

ไม่พึ่ง pytest เพราะโปรเจกต์นี้ไม่ได้ติดตั้งไว้ และเทสต์ชุดนี้ควรรันได้ทันที
บนเครื่องไหนก็ตามที่รัน test_code.py ได้ คืน exit code 1 เมื่อมีข้อไหนตก
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_align                                       # noqa: E402
import test_maze                                        # noqa: E402
import test_payload                                     # noqa: E402
import test_run_search                                  # noqa: E402
from fakes import Checker                               # noqa: E402

#: list: โมดูลเทสต์ตามลำดับที่ควรรัน จากหน่วยย่อยไปหาการรันทั้งรอบ
SUITES = [
    ("Payload และ --armtest", test_payload),
    ("Maze: กำแพง ของที่วาง และการวางแผน", test_maze),
    ("การจัดระยะเทียบกำแพงและการเล็งเป้า", test_align),
    ("run_search และ run_sim", test_run_search),
]


def main():
    """รันทุกชุดแล้วสรุปรวมครั้งเดียว

    Returns:
        int: 0 เมื่อผ่านหมด
    """
    chk = Checker()
    for title, module in SUITES:
        print("\n" + "=" * 58)
        print("  {0}".format(title))
        print("=" * 58)
        module.run(chk)
    return 0 if chk.report() else 1


if __name__ == "__main__":
    sys.exit(main())

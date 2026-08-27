# -*- coding: utf-8 -*-
"""ตีความค่าดิบจากเซนเซอร์ และตัดสินว่าค่าที่อ่านได้เชื่อได้แค่ไหน

แยกจาก :mod:`maze_solver.sensors` ตรงที่นี่ไม่ได้คุยกับฮาร์ดแวร์เลย รับแต่
ตัวเลขที่อ่านมาแล้ว จึงเป็นที่รวมของกติกาการตีความไว้ที่เดียว
"""
from . import config
from .geometry import tof_sanity_window_mm


def sharp_polarity(thresholds):
    """+1 ถ้า ADC สูง = อยู่ใกล้, -1 ถ้า ADC ต่ำ = อยู่ใกล้

    อนุมานจากลำดับของ (enter, exit) ที่ ``--calib`` คำนวณมาให้ จึงไม่ต้องมี
    ค่าคอนฟิกแยกอีกตัว และไม่ต้องสมมติว่าเซนเซอร์ตอบสนองไปทางไหน

    รับ threshold ของข้างที่ถามมาโดยตรง ไม่ใช่อ่านจากข้างซ้ายข้างเดียวแล้วเหมาว่า
    ขวาเหมือนกัน เพราะ Sharp สองตัวอาจเป็นคนละรุ่นหรือต่อสลับขั้วกัน ซึ่งจะทำให้
    การประคองกลางช่องดันผิดทางแบบเงียบ ๆ โดยไม่มีอะไรเตือน

    Args:
        thresholds (tuple): (enter, exit) ของข้างนั้น

    Returns:
        int: +1 หรือ -1
    """
    enter, exit_ = thresholds
    return 1 if enter > exit_ else -1


def wall_from_adc(adc, thresholds):
    """แปลงค่า ADC ดิบเป็นสถานะกำแพง

    Args:
        adc (int): ค่า ADC ดิบ
        thresholds (tuple): (enter, exit) จาก ``--calib``

    Returns:
        bool or None: True=มีกำแพง, False=ไม่มี, None=ก้ำกึ่งตัดสินไม่ได้
    """
    if adc is None:
        return None
    enter, exit_ = thresholds
    if enter > exit_:
        if adc >= enter:
            return True
        if adc <= exit_:
            return False
    else:
        if adc <= enter:
            return True
        if adc >= exit_:
            return False
    return None


def ir_triggered(io_value):
    """True ถ้า IR 45 องศาตัวนั้นกำลังเจอสิ่งกีดขวาง"""
    if io_value is None or config.IR_TRIGGERED_VALUE is None:
        return False
    return int(io_value) == int(config.IR_TRIGGERED_VALUE)


def aim_reading_is_sane(measured_mm, target_mm, predicted_mm=None):
    """ตรวจว่าค่า ToF ที่เพิ่งวัดได้ เชื่อถือได้พอจะขยับตามหรือไม่

    ตรวจสองด่านที่มาจากคนละแหล่ง จึงจับคนละเรื่องกัน

    1. เทียบกับ ``target_mm`` ซึ่งมาจากโจทย์ใน ``AIM_SEQUENCE`` ด่านนี้ทำงานเสมอ
       จับกรณีที่ค่าหลุดจากที่ควรเป็นเกินครึ่งช่อง
    2. เทียบกับ ``predicted_mm`` ซึ่งมาจากแผนที่ที่หุ่นสร้างเอง (ดู
       :meth:`Maze.predict_tof`) ด่านนี้ทำงานเฉพาะตอนแผนที่รู้จริง จับกรณีที่
       หุ่นไม่ได้อยู่ช่องที่คิดว่าอยู่ ซึ่งด่านแรกมองไม่เห็น

    ด่านนี้มีไว้ปฏิเสธค่าที่ "มีอยู่แต่เชื่อไม่ได้" เท่านั้น กรณีที่วัดไม่ได้เลย
    เป็นคนละเรื่องและถูกจัดการที่ :meth:`Driver.align_to_wall` อยู่แล้ว
    (คืน ``no_wall`` โดยไม่ขยับ) ผู้เรียกจึงต้องกรอง None ออกก่อนเรียกตัวนี้
    ไม่งั้นจะกลายเป็นตัดสินใจเรื่องเดียวกันสองที่แล้วไม่ตรงกัน

    Args:
        measured_mm (float): ค่าที่วัดได้ตอนจอดนิ่ง
        target_mm (float): ค่าที่ต้องการให้เป็นเมื่อจัดระยะเสร็จ
        predicted_mm (float or None): ค่าที่แผนที่ทำนายไว้ None = แผนที่ไม่รู้

    Returns:
        tuple: (ok, ข้อความอธิบายเมื่อไม่ผ่าน)
    """
    window = tof_sanity_window_mm()
    off_target = measured_mm - target_mm
    if abs(off_target) > window:
        return False, ("วัดได้ {0:.0f}mm ห่างจากเป้า {1:.0f}mm อยู่ {2:+.0f}mm "
                       "ซึ่งเกินหน้าต่างที่ยอมรับ ({3:.0f}mm)"
                       .format(measured_mm, target_mm, off_target, window))

    if predicted_mm is not None:
        off_map = measured_mm - predicted_mm
        if abs(off_map) > window:
            return False, ("วัดได้ {0:.0f}mm แต่แผนที่ว่าควรได้ {1:.0f}mm "
                           "ต่างกัน {2:+.0f}mm (เกิน {3:.0f}mm)"
                           .format(measured_mm, predicted_mm, off_map, window))
    return True, ""


def observation_is_trusted(maze, x, y, heading, tof_mm):
    """ค่าที่เพิ่งอ่านได้ เชื่อพอจะบันทึกลงแผนที่หรือไม่

    เทียบระยะที่วัดได้กับที่ ``maze`` ทำนายไว้สำหรับช่องและทิศนี้ ด่านนี้จับสิ่งที่
    ไม่มีอะไรอื่นในโปรแกรมจับได้ คือ "หุ่นนับช่องพลาด" - ถ้าล้อลื่นหรือ odometry
    เพี้ยนจนหุ่นคิดว่าอยู่คนละช่องกับที่อยู่จริง ทุกอย่างหลังจากนั้นยังทำงานได้
    ปกติทุกประการ แค่เขียนแผนที่ผิดที่ไปเรื่อย ๆ ซึ่งลบออกไม่ได้ (ดู
    :meth:`Maze.set_wall`) กว่าจะรู้ตัวก็ตอนหุ่นวิ่งชนกำแพงที่แผนที่ว่าโล่ง

    ไม่มีข้อมูลพอให้ตัดสิน = เชื่อ ไม่ใช่ไม่เชื่อ เพราะช่องที่เพิ่งมาถึงครั้งแรก
    ย่อมยังไม่มีใครเคยเห็นด้านหน้ามาก่อนเป็นปกติ ถ้าตีเป็นไม่เชื่อ หุ่นจะไม่ได้
    บันทึกอะไรลงแผนที่เลยตลอดขาไป

    Args:
        maze (Maze): แผนที่ที่สะสมมา
        x (int): ช่องที่หุ่นเชื่อว่าตัวเองอยู่
        y (int): ช่องที่หุ่นเชื่อว่าตัวเองอยู่
        heading (int): ทิศที่หันอยู่ตอนอ่าน
        tof_mm (float or None): ระยะที่วัดได้ None = วัดไม่ได้

    Returns:
        tuple: (ok, ข้อความอธิบายเมื่อไม่ผ่าน)
    """
    if tof_mm is None:
        return True, ""
    predicted = maze.predict_tof(x, y, heading)
    if predicted is None:
        return True, ""
    gap = tof_mm - predicted
    if abs(gap) <= tof_sanity_window_mm():
        return True, ""
    return False, ("ToF วัดได้ {0:.0f}mm แต่แผนที่ว่าควรได้ {1:.0f}mm "
                   "(ต่าง {2:+.0f}mm)".format(tof_mm, predicted, gap))

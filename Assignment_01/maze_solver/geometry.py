# -*- coding: utf-8 -*-
"""เรขาคณิตของสนามและตัวหุ่น - แปลงค่าคงที่เป็นระยะที่เอาไปสั่งงานได้

ทุกฟังก์ชันในนี้เป็นการคำนวณล้วน ไม่แตะฮาร์ดแวร์และไม่มีสถานะ จึงเรียกทดสอบ
ได้ตรง ๆ โดยไม่ต้องมีหุ่น
"""
from . import config
from .directions import DIR_NAMES


def wrap_deg(angle):
    """บีบมุมให้อยู่ในช่วง [-180, 180) องศา"""
    return (angle + 180.0) % 360.0 - 180.0


def clamp(value, low, high):
    """จำกัดค่าให้อยู่ระหว่าง low ถึง high"""
    return max(low, min(high, value))


def front_wall_threshold_mm():
    """เกณฑ์ ToF ที่ใช้ตัดสินว่ามีกำแพงอยู่ด้านหน้า หน่วย mm

    เรขาคณิต: เมื่อหุ่นจอดกลางช่อง กำแพงที่ขอบช่องนี้อ่านได้ ``FRONT_STOP_MM``
    ส่วนกำแพงที่ขอบช่องถัดไปอ่านได้ ``FRONT_STOP_MM + CELL_SIZE`` เส้นแบ่งจึงวางไว้
    ตรงกลางระหว่างสองค่านั้นพอดี เพราะเป็นจุดที่ทนต่อการจอดคลาดเคลื่อนได้มากที่สุด
    """
    if config.FRONT_WALL_MM_OVERRIDE is not None:
        return config.FRONT_WALL_MM_OVERRIDE
    return config.FRONT_STOP_MM + int(config.CELL_SIZE_M * 1000.0 / 2.0)


def tof_forward_offset_mm():
    """float: ระยะที่หัว ToF ล้ำหน้าจุดหมุนของหุ่น หน่วย mm

    ไม่ต้องวัดเองถ้าคาลิเบรต ``FRONT_STOP_MM`` มาแล้ว เพราะค่านั้นนิยามไว้ว่าเป็น
    ระยะที่ ToF อ่านได้ตอนหุ่นจอดกลางช่องแล้วหันชนกำแพงที่ขอบช่อง ระยะจากจุดหมุน
    ถึงขอบช่องคือครึ่งช่องพอดี ส่วนที่หายไประหว่างสองค่านั้นจึงเป็นความยาวของหัว
    เซนเซอร์ที่ยื่นล้ำจุดหมุนออกไป

    เป็นสมบัติของตัวหุ่น ไม่ใช่ของสนาม ย้ายสนามแล้วไม่ต้องหาใหม่
    """
    if config.TOF_FORWARD_OFFSET_MM is not None:
        return float(config.TOF_FORWARD_OFFSET_MM)
    return config.CELL_SIZE_M * 1000.0 / 2.0 - config.FRONT_STOP_MM


def aim_tof_target_mm(step, heading, places):
    """float or None: ระยะที่ ToF ควรอ่านได้เมื่อขั้นเล็งนี้จัดตำแหน่งเสร็จ

    แปลง "โจทย์" (ของต้องไปตกห่างกำแพงไหนเท่าไร) เป็น "คำสั่ง" (ToF ต้องอ่านได้
    เท่าไร) โดยไล่จากกำแพงที่หุ่นหันใส่เข้ามาหาตัวหุ่น

        ToF = ระยะจากกำแพงที่หันใส่ถึงจุดที่ของตก
              + ระยะเอื้อมของแขน (เฉพาะขั้นที่ยื่นแขนวางจริง)
              - ระยะที่หัว ToF ล้ำหน้าจุดหมุน
              + ค่าชดเชยจากตลับเมตร

    ``ARM_REACH_MM`` เข้าสมการเฉพาะขั้นที่ยื่นแขนวาง เพราะขั้นอื่นแขนยื่นตั้งฉาก
    กับแกนที่กำลังจัด ตำแหน่งของตามแกนนั้นจึงเท่ากับตำแหน่งหุ่นพอดี

    Args:
        step (AimStep): ขั้นที่ต้องการหาค่าเป้า
        heading (int): ทิศที่หุ่นหันอยู่จริงตอนวัด (หลังหมุนตาม ``step.face``
            แล้ว) ใช้ตัวนี้แทน ``step.face`` เพราะ ``face`` เป็น None ได้
        places (bool): ขั้นนี้เป็นขั้นสุดท้ายที่ยื่นแขนวางของหรือไม่

    Returns:
        float or None: ระยะ ToF เป้าหมาย หน่วย mm None = ขั้นนี้ไม่จัดระยะ

    Raises:
        ValueError: กำแพงอ้างอิงไม่ได้อยู่แกนเดียวกับทิศที่หัน ไม่ได้บอก
            ``span_mm`` ทั้งที่ต้องใช้ หรือขั้นที่ยื่นแขนวางแต่ยังไม่ได้ตั้ง
            ``ARM_REACH_MM``
    """
    if step.target_mm is None:
        return None

    if step.ref == heading:
        # หันใส่กำแพงที่เป้านับระยะจากพอดี ใช้ระยะเป้าได้ตรง ๆ
        object_from_faced_wall = float(step.target_mm)
    elif step.ref == (heading + 2) % 4:
        # เป้านับจากกำแพงตรงข้ามกับที่หันใส่ (เช่นด้านนั้นเป็นประตู ToF มองไม่เห็น)
        # ต้องพลิกด้านด้วยความกว้างห้องตามแกนนี้
        if step.span_mm is None:
            raise ValueError(
                "AimStep: เป้านับจากกำแพง{0} แต่หันใส่กำแพง{1} ต้องบอก span_mm"
                .format(DIR_NAMES[step.ref], DIR_NAMES[heading]))
        object_from_faced_wall = float(step.span_mm) - float(step.target_mm)
    else:
        raise ValueError(
            "AimStep: กำแพงอ้างอิง {0} ไม่ได้อยู่แกนเดียวกับทิศที่หัน {1} "
            "จัดระยะตามแกนนี้ด้วย ToF ไม่ได้"
            .format(DIR_NAMES[step.ref], DIR_NAMES[heading]))

    reach = 0.0
    if places:
        if config.ARM_REACH_MM is None:
            raise ValueError(
                "ขั้นสุดท้ายของ AIM_SEQUENCE เป็นขั้นที่ยื่นแขนวาง ต้องตั้ง "
                "ARM_REACH_MM ก่อนจึงจะคำนวณระยะเป้าได้")
        reach = float(config.ARM_REACH_MM)

    return (object_from_faced_wall + reach - tof_forward_offset_mm()
            + float(step.offset_mm))


def tof_sanity_window_mm():
    """float: ครึ่งความกว้างของหน้าต่างที่ยอมรับค่า ToF หน่วย mm"""
    if config.TOF_SANITY_WINDOW_MM is not None:
        return float(config.TOF_SANITY_WINDOW_MM)
    return config.CELL_SIZE_M * 1000.0 / 2.0


def place_heading_for(entry_heading):
    """ทิศที่แขนจะยื่นวางของ เมื่อเข้าช่องเป้าหมายมาด้วยทิศ ``entry_heading``

    คำนวณจาก ``AIM_SEQUENCE`` ล้วน ๆ โดยไม่ต้องขยับหุ่น มีไว้ให้ ``run_sim``
    (และเทสต์) รู้ล่วงหน้าว่าของจะไปกองทางไหน ตัว ``run_search`` ไม่ใช้ เพราะมัน
    ได้ทิศจริงจากค่าที่ :func:`place_on_target` คืนมาอยู่แล้ว ซึ่งเชื่อถือได้กว่า

    Returns:
        int: ทิศที่หุ่นจะหันอยู่ตอนปล่อยของ = ทิศที่ของไปกองอยู่
    """
    for step in config.AIM_SEQUENCE:
        if step.face is not None:
            entry_heading = step.face
    return entry_heading

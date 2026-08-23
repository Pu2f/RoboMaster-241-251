# -*-coding:utf-8-*-
# อ่านไฟล์ maze_path_log.csv (สร้างจาก 01_wall_follow_maze.py หลังจบการเดิน)
# แล้ววาดเส้นทางการเดินของหุ่นยนต์ในเขาวงกต สำหรับแนบในรายงาน

import csv
import matplotlib.pyplot as plt


LOG_FILE = "maze_path_log.csv"

if __name__ == '__main__':
    xs, ys = [], []
    with open(LOG_FILE, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))

    plt.figure(figsize=(6, 6))
    plt.plot(xs, ys, "-b", linewidth=1.5, label="เส้นทางหุ่นยนต์")
    plt.plot(xs[0], ys[0], "go", markersize=8, label="จุดเริ่มต้น")
    plt.plot(xs[-1], ys[-1], "rs", markersize=8, label="จุดสิ้นสุด")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("เส้นทางการเดินในเขาวงกต")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.savefig("maze_path.png", dpi=150)
    plt.show()

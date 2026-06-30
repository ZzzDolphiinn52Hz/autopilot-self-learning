# =========================
# install pygame pyserial
# pip install pyserial pygame
# =========================

# =========================
# source /f/TuanAnhStudy/autopilot-self-learning/.venv/Scripts/activate
# C:/Python313/python.exe f:/TuanAnhStudy/autopilot-self-learning/ardumyload/test/icm42688_test.py
# C:/Python313/python.exe icm42688_test.py
# =========================

import pygame
import serial
import math
import sys
import time
from collections import deque

# =========================
# CẤU HÌNH SERIAL
# =========================
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

# =========================
# CẤU HÌNH ĐẢO DẤU HIỂN THỊ
# =========================
ROLL_DISPLAY_SIGN = -1.0
PITCH_DISPLAY_SIGN = 1.0
YAW_DISPLAY_SIGN = 1.0

# Góc nhìn camera ảo
VIEW_YAW = 0.0
VIEW_PITCH = 0.0

# Số mẫu hiển thị trên đồ thị
HISTORY_LEN = 300

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"Da ket noi thanh cong voi {SERIAL_PORT}")
except Exception as e:
    print(f"Loi ket noi Serial: {e}")
    sys.exit()

# =========================
# KHỞI TẠO PYGAME
# =========================
pygame.init()

WIDTH, HEIGHT = 1200, 720
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Mô phỏng 3D Drone + IMU Graph - ICM42688P")
clock = pygame.time.Clock()

font = pygame.font.SysFont("Arial", 18)
font_small = pygame.font.SysFont("Arial", 14)
font_axis = pygame.font.SysFont("Arial", 20, bold=True)

# =========================
# MÔ HÌNH DRONE
# Local coordinates:
# X = phải
# Y = lên
# Z = trước
# =========================
body_vertices = [
    [-30, -10, -50], [30, -10, -50], [30, 10, -50], [-30, 10, -50],
    [-30, -10,  50], [30, -10,  50], [30, 10,  50], [-30, 10,  50]
]

body_edges = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7)
]

arms = [
    ([0, 0, 0], [-120, 0,  120]),   # Trước trái
    ([0, 0, 0], [ 120, 0,  120]),   # Trước phải
    ([0, 0, 0], [-120, 0, -120]),   # Sau trái
    ([0, 0, 0], [ 120, 0, -120])    # Sau phải
]

# =========================
# BIẾN GÓC
# =========================
roll, pitch, yaw = 0.0, 0.0, 0.0
last_sample_time = time.time()

# Giá trị IMU hiện tại
ax = ay = az = 0.0
gx = gy = gz = 0.0

# Lưu lịch sử để vẽ đồ thị
history = {
    "ax": deque(maxlen=HISTORY_LEN),
    "ay": deque(maxlen=HISTORY_LEN),
    "az": deque(maxlen=HISTORY_LEN),
    "gx": deque(maxlen=HISTORY_LEN),
    "gy": deque(maxlen=HISTORY_LEN),
    "gz": deque(maxlen=HISTORY_LEN),
}

# =========================
# HÀM XOAY 3D
# =========================
def rotate_x(x, y, z, angle):
    rad = math.radians(angle)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    return x, y * cos_a - z * sin_a, y * sin_a + z * cos_a

def rotate_y(x, y, z, angle):
    rad = math.radians(angle)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    return x * cos_a + z * sin_a, y, -x * sin_a + z * cos_a

def rotate_z(x, y, z, angle):
    rad = math.radians(angle)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a, z

def transform_3d_point(x, y, z, use_attitude=True):
    if use_attitude:
        x, y, z = rotate_z(x, y, z, ROLL_DISPLAY_SIGN * roll)
        x, y, z = rotate_x(x, y, z, PITCH_DISPLAY_SIGN * pitch)
        x, y, z = rotate_y(x, y, z, -YAW_DISPLAY_SIGN * yaw)

    x, y, z = rotate_y(x, y, z, VIEW_YAW)
    x, y, z = rotate_x(x, y, z, VIEW_PITCH)

    # Đặt mô hình ở nửa trái màn hình
    screen_x = int(WIDTH * 0.30 + x)
    screen_y = int(HEIGHT * 0.50 - y)

    return screen_x, screen_y

def draw_text(text, x, y, color=(255, 255, 255)):
    img = font.render(text, True, color)
    screen.blit(img, (x, y))

def draw_small_text(text, x, y, color=(230, 230, 230)):
    img = font_small.render(text, True, color)
    screen.blit(img, (x, y))

def draw_axis_label(text, pos, color):
    img = font_axis.render(text, True, color)
    screen.blit(img, (pos[0] + 6, pos[1] + 6))

def draw_body_axes():
    origin = transform_3d_point(0, 0, 0)

    axes = [
        ("X", [150, 0, 0], (255, 80, 80)),
        ("Y", [0, 120, 0], (80, 255, 80)),
        ("Z", [0, 0, 150], (80, 150, 255)),
    ]

    for name, end, color in axes:
        end_point = transform_3d_point(end[0], end[1], end[2])
        pygame.draw.line(screen, color, origin, end_point, 3)
        pygame.draw.circle(screen, color, end_point, 5)
        draw_axis_label(name, end_point, color)

def parse_imu_line(line):
    """
    Dữ liệu mong đợi:
    ACC (g): ax ay az | GYR (deg/s): gx gy gz
    """
    if "ACC (g):" not in line or "GYR (deg/s):" not in line:
        return None

    parts = line.split('|')
    if len(parts) < 2:
        return None

    acc_part = parts[0].replace("ACC (g):", "").strip()
    gyr_part = parts[1].replace("GYR (deg/s):", "").strip()

    ax_, ay_, az_ = map(float, acc_part.split())
    gx_, gy_, gz_ = map(float, gyr_part.split())

    return ax_, ay_, az_, gx_, gy_, gz_

# =========================
# VẼ ĐỒ THỊ
# =========================
def draw_graph(surface, rect, data, title, unit, fixed_min=None, fixed_max=None, color=(0, 191, 255)):
    x, y, w, h = rect

    # Khung nền
    pygame.draw.rect(surface, (32, 36, 48), rect, border_radius=6)
    pygame.draw.rect(surface, (90, 90, 100), rect, 1, border_radius=6)

    draw_small_text(title, x + 8, y + 5, (255, 255, 255))

    if len(data) < 2:
        return

    values = list(data)

    if fixed_min is None or fixed_max is None:
        v_min = min(values)
        v_max = max(values)

        # Tránh trường hợp max = min
        if abs(v_max - v_min) < 1e-6:
            v_max += 1.0
            v_min -= 1.0

        # Thêm biên cho dễ nhìn
        margin = 0.15 * (v_max - v_min)
        v_min -= margin
        v_max += margin
    else:
        v_min = fixed_min
        v_max = fixed_max

    # Đường zero
    if v_min < 0 < v_max:
        zero_y = int(y + h - ((0 - v_min) / (v_max - v_min)) * h)
        pygame.draw.line(surface, (85, 85, 90), (x, zero_y), (x + w, zero_y), 1)

    # Vẽ polyline
    points = []
    n = len(values)
    for i, v in enumerate(values):
        px = x + int(i * (w - 1) / (n - 1))
        py = y + h - int((v - v_min) * h / (v_max - v_min))
        py = max(y, min(y + h, py))
        points.append((px, py))

    pygame.draw.lines(surface, color, False, points, 2)

    # Hiện giá trị hiện tại và scale
    current_value = values[-1]
    draw_small_text(f"{current_value:.3f} {unit}", x + w - 95, y + 5, color)
    draw_small_text(f"{v_max:.2f}", x + 5, y + 22, (170, 170, 170))
    draw_small_text(f"{v_min:.2f}", x + 5, y + h - 18, (170, 170, 170))

def draw_imu_graphs():
    graph_x = 650
    graph_y = 40
    graph_w = 510
    graph_h = 90
    gap = 16

    # Accel thường xem trong khoảng ±2g là đủ
    draw_graph(screen, (graph_x, graph_y + 0 * (graph_h + gap), graph_w, graph_h),
               history["ax"], "Accel X - ax", "g", -1.0, 1.0, (255, 100, 100))
    
    draw_graph(screen, (graph_x, graph_y + 1 * (graph_h + gap), graph_w, graph_h),
               history["ay"], "Accel Y - ay", "g", -1.0, 1.0, (100, 255, 100))

    draw_graph(screen, (graph_x, graph_y + 2 * (graph_h + gap), graph_w, graph_h),
               history["az"], "Accel Z - az", "g", -1.0, 1.0, (100, 150, 255))

    # Gyro để ±250 deg/s cho dễ nhìn. Nếu bạn quay nhanh quá thì tăng lên ±500
    draw_graph(screen, (graph_x, graph_y + 3 * (graph_h + gap), graph_w, graph_h),
               history["gx"], "Gyro X - gx", "deg/s", -10.0, 10.0, (255, 160, 100))

    draw_graph(screen, (graph_x, graph_y + 4 * (graph_h + gap), graph_w, graph_h),
               history["gy"], "Gyro Y - gy", "deg/s", -10.0, 10.0, (180, 255, 100))

    draw_graph(screen, (graph_x, graph_y + 5 * (graph_h + gap), graph_w, graph_h),
               history["gz"], "Gyro Z - gz", "deg/s", -10.0, 10.0, (180, 180, 255))

# =========================
# VÒNG LẶP CHÍNH
# =========================
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                roll, pitch, yaw = 0.0, 0.0, 0.0
                print("Reset roll, pitch, yaw")

    # =========================
    # ĐỌC SERIAL
    # =========================
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            data = parse_imu_line(line)

            if data is not None:
                ax, ay, az, gx, gy, gz = data

                # Lưu dữ liệu vào history để vẽ đồ thị
                history["ax"].append(ax)
                history["ay"].append(ay)
                history["az"].append(az)
                history["gx"].append(gx)
                history["gy"].append(gy)
                history["gz"].append(gz)

                current_sample_time = time.time()
                dt = current_sample_time - last_sample_time
                last_sample_time = current_sample_time

                if dt <= 0:
                    dt = 0.001
                if dt > 0.05:
                    dt = 0.05

                # =========================
                # TÍNH GÓC TỪ ACCEL
                # =========================
                roll_acc = math.atan2(ay, az) * 180.0 / math.pi
                pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi

                # =========================
                # COMPLEMENTARY FILTER
                # =========================
                roll = 0.96 * (roll + gx * dt) + 0.04 * roll_acc
                pitch = 0.96 * (pitch + gy * dt) + 0.04 * pitch_acc

                # Yaw chỉ tích phân gyro Z nên sẽ bị trôi
                yaw += gz * dt

        except Exception:
            pass

    # =========================
    # VẼ ĐỒ HỌA
    # =========================
    screen.fill((20, 24, 33))

    # Chia vùng: trái là mô hình, phải là đồ thị
    pygame.draw.line(screen, (80, 80, 90), (610, 0), (610, HEIGHT), 2)

    # Trục tham chiếu vùng mô hình
    pygame.draw.line(screen, (70, 70, 70), (0, HEIGHT // 2), (610, HEIGHT // 2), 1)
    pygame.draw.line(screen, (70, 70, 70), (int(WIDTH * 0.30), 0), (int(WIDTH * 0.30), HEIGHT), 1)

    # Vẽ thân drone
    projected_vertices = []
    for v in body_vertices:
        projected_vertices.append(transform_3d_point(v[0], v[1], v[2]))

    for edge in body_edges:
        p1 = projected_vertices[edge[0]]
        p2 = projected_vertices[edge[1]]
        pygame.draw.line(screen, (0, 191, 255), p1, p2, 2)

    # Vẽ tay motor
    for i, arm in enumerate(arms):
        p1 = transform_3d_point(arm[0][0], arm[0][1], arm[0][2])
        p2 = transform_3d_point(arm[1][0], arm[1][1], arm[1][2])

        color = (255, 69, 0) if i < 2 else (200, 200, 200)
        pygame.draw.line(screen, color, p1, p2, 4)
        pygame.draw.circle(screen, color, p2, 12, 2)

    # Vẽ hệ trục thân
    draw_body_axes()

    # Text góc
    draw_text(f"Roll:  {roll:.2f} deg", 20, 20)
    draw_text(f"Pitch: {pitch:.2f} deg", 20, 45)
    draw_text(f"Yaw:   {yaw:.2f} deg", 20, 70)

    # Text giá trị IMU hiện tại
    draw_text(f"ax: {ax:.3f} g", 20, 115)
    draw_text(f"ay: {ay:.3f} g", 20, 140)
    draw_text(f"az: {az:.3f} g", 20, 165)
    draw_text(f"gx: {gx:.3f} deg/s", 20, 205)
    draw_text(f"gy: {gy:.3f} deg/s", 20, 230)
    draw_text(f"gz: {gz:.3f} deg/s", 20, 255)

    draw_text("Axis: X red, Y green, Z blue", 20, HEIGHT - 55)
    draw_text("Press R to reset roll/pitch/yaw", 20, HEIGHT - 30)

    # Vẽ 6 đồ thị IMU
    draw_imu_graphs()

    pygame.display.flip()
    clock.tick(60)

ser.close()
pygame.quit()
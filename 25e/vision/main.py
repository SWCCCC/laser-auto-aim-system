# ===================================================================================
#
#                      简易自行瞄准装置 - 竞赛专用代码 (UI顶部版)
#
#   >>> [V2.1 - 移除发挥任务，优化UI布局] <<<
#
#   更新日志:
#   - [UI] 移除了“发挥任务(A1/2)”的按钮和相关功能代码。
#   - [UI] 将剩余3个任务按钮的宽度自动调整，使其均匀填充顶部空间，更易于操作。
#   - [核心] 融合了ROI（感兴趣区域）追踪和状态机（搜索/追踪）逻辑，大幅提升追踪速度和稳定性。
#   - [核心] 引入了角点排序功能，提高了倾斜矩形中心点计算的精度。
#   - [核心] 重构了自动瞄准函数 `run_task_B3_track_aim_generic` 以支持新功能。
#   - [参数] 新增了 `ROI_PADDING` 和 `TRACKING_LOST_THRESHOLD` 两个用于追踪优化的参数。
#
# ===================================================================================

import time, os, gc, sys
import math
import cv_lite
import ulab.numpy as np
from machine import UART, FPIOA, Pin, TOUCH
from media.sensor import *
from media.display import *
from media.media import *

# ===================================================================================
#                           1. 硬件引脚和全局参数配置
# ===================================================================================

# --- [!!] 后期需要手动调试的关键参数 (CRITICAL PARAMETERS TO TUNE) [!!] ---

# --- 激光继电器引脚 ---
IO_LASER_RELAY = 47  # 控制激光的继电器引脚, 高电平触发

# --- Y轴（俯仰）电机抬升参数 ---
Y_LIFT_STEPS = 440   # Y轴电机抬升的步数，对应约30度，需要实际测量
Y_LIFT_SPEED = 5     # Y轴抬升速度

# --- 自动瞄准（基础任务3）参数 ---
AIM_ANGLE_THRESHOLD = 0.8  # 自动瞄准时，判定为对准的角度误差(度)。可适当放宽
AIM_TIMEOUT_S = 3.8        # 自动瞄准的最长时间(秒)，题目要求4秒内完成
X_SEARCH_STEPS = 18        # 未找到目标时，X轴单次搜索转动的步数
X_SEARCH_SPEED = 12        # X轴搜索速度

# --- ROI追踪与状态机参数 ---
ROI_PADDING = 40                 # ROI区域向外扩展的像素数
TRACKING_LOST_THRESHOLD = 20     # 连续丢失目标多少帧后，切换回全局搜索状态

# --- 视觉识别和电机控制的基础参数 ---
WIDTH, HEIGHT = 320,240
step_scale = 0.8
max_steps = 160
# !! [重要] 使用你校准后的FOV值，这会显著提高精度 !!
angle_per_pixel = 54.4 / WIDTH

# --- 矩形检测参数 ---
canny_thresh1, canny_thresh2 = 50, 150
approx_epsilon, area_min_ratio = 0.04, 0.01
max_angle_cos, gaussian_blur_size = 0.3, 3

# ===================================================================================
#                           2. 硬件初始化和底层函数
# ===================================================================================

# --- 硬件初始化 ---
lcd_width, lcd_height = 800, 480
sensor = Sensor(); sensor.reset()
sensor.set_framesize(width=WIDTH, height=HEIGHT)
sensor.set_pixformat(Sensor.RGB565)
Display.init(Display.ST7701, width=lcd_width, height=lcd_height, to_ide=True)
MediaManager.init(); sensor.run()
print(">> 摄像头和屏幕初始化完成")

tp = TOUCH(0); print(">> 触摸屏初始化完成")

fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD); fpioa.set_function(4, FPIOA.UART1_RXD)
fpioa.set_function(5, FPIOA.UART2_TXD); fpioa.set_function(6, FPIOA.UART2_RXD)
uart_x, uart_y = UART(UART.UART1, 38400), UART(UART.UART2, 38400)
print(">> 电机串口初始化完成")

laser_relay = Pin(IO_LASER_RELAY, Pin.OUT); laser_relay.value(0)
print(">> 继电器初始化完成")

# --- 任务模式和UI定义 ---
### 修改: 移除了MODE_A12 ###
MODE_IDLE, MODE_B2, MODE_B3_CW, MODE_B3_CCW = 0, 1, 2, 3
MODE_NAMES = {
    MODE_IDLE: "IDLE",
    MODE_B2: "TASK B2",
    MODE_B3_CW: "TASK B3 (CW)",
    MODE_B3_CCW: "TASK B3 (CCW)",
}
current_mode = MODE_IDLE

### 修改: 重新设计的UI按钮布局，3个按钮更宽 ###
# 3个按钮平分320像素宽度 (106, 107, 107)
UI_BUTTONS = {
    MODE_B2:      (0,   10, 106, 50, "B2 Fix"),
    MODE_B3_CW:   (106, 10, 107, 50, "B3 CW"),
    MODE_B3_CCW:  (213, 10, 107, 50, "B3 CCW"),
}

# --- 底层函数 ---
def checksum(data): return sum(data) & 0xFF
def send_cmd(uart, data): uart.write(bytearray(data + [checksum(data)]))
def move_motor(uart, speed, direction, steps):
    if steps == 0: return
    spd_byte = (speed & 0x7F) | (0x80 if direction == 'CCW' else 0)
    step_bytes = [(steps >> i) & 0xFF for i in (24, 16, 8, 0)]
    send_cmd(uart, [0xE0, 0xFD, spd_byte] + step_bytes)
def clear_protect(uart, addr=1): send_cmd(uart, [0xFA, addr, 0x06, 0x00, 0x3D, 0x00, 0x01])
def init_motor():
    for u in [uart_x, uart_y]:
        clear_protect(u); time.sleep(0.05)
        send_cmd(u, [0xE0, 0xF3, 0x01]); time.sleep(0.05)
    print(">> 电机初始化完成")
def control_laser(state):
    laser_relay.value(1 if state else 0)
    print(f">> 激光 {'ON' if state else 'OFF'}")
def calculate_center(points):
    if not points: return None
    return (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))
def sort_corners(corners):
    center = calculate_center(corners)
    if center is None: return []
    sorted_corners = sorted(corners, key=lambda p: math.atan2(p[1]-center[1], p[0]-center[0]))
    if len(sorted_corners) == 4:
        left_top = min(sorted_corners, key=lambda p: p[0]+p[1])
        index = sorted_corners.index(left_top)
        sorted_corners = sorted_corners[index:] + sorted_corners[:index]
    return sorted_corners
def is_valid_rect(rect_info, img_area):
    w, h = rect_info[2], rect_info[3]
    area = w * h
    if not (img_area * area_min_ratio < area < img_area * 0.8): return False
    aspect_ratio = w / max(h, 1)
    if not (0.5 < aspect_ratio < 2.5): return False
    return True
def draw_ui_on_image(img, mode, status="", roi=None):
    for btn_mode, (x, y, w, h, text) in UI_BUTTONS.items():
        color = (0, 255, 0) if btn_mode == mode else (180, 180, 180)
        img.draw_rectangle(x, y, w, h, color, 2)
        # 调整文字位置以适应更宽的按钮
        text_w = len(text) * 8 # 简单估算文字宽度
        img.draw_string_advanced(x + (w - text_w) // 2, y + 15, 16, text, color=color)
    img.draw_string_advanced(5, 70, 20, f"Mode: {MODE_NAMES.get(mode, 'Unknown')}", color=(255, 255, 0))
    if status:
        img.draw_string_advanced(5, 95, 20, f"Status: {status}", color=(0, 255, 255))
    if roi:
        img.draw_rectangle(roi, color=(255, 165, 0), thickness=1)

# ===================================================================================
#                           3. 任务流程函数
# ===================================================================================
def run_task_B2_fixed_aim():
    global current_mode
    print("\n--- 开始执行 [基础任务2: 固定打靶] ---")
    img = sensor.snapshot(); draw_ui_on_image(img, current_mode, "Lifting Y..."); Display.show_image(img, x_offset, y_offset)
    move_motor(uart_y, Y_LIFT_SPEED, 'CW', Y_LIFT_STEPS); time.sleep(1.0)
    img = sensor.snapshot(); draw_ui_on_image(img, current_mode, "Laser ON"); Display.show_image(img, x_offset, y_offset)
    control_laser(True); time.sleep(2)
    control_laser(False); print("--- [基础任务2] 执行完毕 ---\n"); current_mode = MODE_IDLE

def run_task_B3_track_aim_generic(search_direction):
    global current_mode
    print(f"\n--- 开始执行 [基础任务3: {search_direction} 搜索] ---")
    img = sensor.snapshot(); draw_ui_on_image(img, current_mode, "Lifting Y..."); Display.show_image(img, x_offset, y_offset)
    move_motor(uart_y, Y_LIFT_SPEED, 'CW', Y_LIFT_STEPS); time.sleep(0.2)

    start_time = time.time(); is_aimed = False
    tracking_state = 'SEARCHING'
    last_known_roi = None
    frames_since_detection = 0

    while time.time() - start_time < AIM_TIMEOUT_S:
        clock.tick(); img = sensor.snapshot()

        roi_to_process = (0, 0, WIDTH, HEIGHT)
        if tracking_state == 'TRACKING' and last_known_roi:
            x, y, w, h = last_known_roi
            roi_to_process = (
                max(0, x - ROI_PADDING), max(0, y - ROI_PADDING),
                min(WIDTH, w + 2*ROI_PADDING), min(HEIGHT, h + 2*ROI_PADDING)
            )

        gray_img = img.to_grayscale(roi=roi_to_process, copy=True)
        img_np = gray_img.to_numpy_ref()
        rects_in_roi = cv_lite.grayscale_find_rectangles_with_corners(
            [gray_img.height(), gray_img.width()], img_np,
            canny_thresh1, canny_thresh2, approx_epsilon,
            area_min_ratio, max_angle_cos, gaussian_blur_size)
        del gray_img; del img_np

        valid_rects = []
        for r in rects_in_roi:
            r_full_coords = list(r)
            r_full_coords[0] += roi_to_process[0]
            r_full_coords[1] += roi_to_process[1]
            for i in range(4, 12, 2):
                r_full_coords[i] += roi_to_process[0]
                r_full_coords[i+1] += roi_to_process[1]

            if is_valid_rect(r_full_coords, img_area):
                corners = [(r_full_coords[i], r_full_coords[i+1]) for i in range(4, 12, 2)]
                sorted_pts = sort_corners(corners)
                center = calculate_center(sorted_pts)
                if center:
                    valid_rects.append({'info': r_full_coords, 'center': center, 'corners': sorted_pts})

        status = ""
        current_roi_for_drawing = roi_to_process if tracking_state == 'TRACKING' else None

        if valid_rects:
            if tracking_state == 'SEARCHING':
                print(">> Target Found! Switching to TRACKING mode.")
            tracking_state = 'TRACKING'
            frames_since_detection = 0

            best_rect = min(valid_rects, key=lambda r: abs(r['center'][0] - img_center_x))
            rect_center = best_rect['center']
            last_known_roi = best_rect['info'][0:4]

            img.draw_rectangle(best_rect['info'][0], best_rect['info'][1], best_rect['info'][2], best_rect['info'][3], color=(0, 255, 0), thickness=2)
            for p in best_rect['corners']:
                 img.draw_circle(int(p[0]), int(p[1]), 3, color=(0, 255, 0), thickness=-1)
            img.draw_circle(int(rect_center[0]), int(rect_center[1]), 5, color=(255, 0, 0), thickness=-1)

            angle_error_x = (rect_center[0] - img_center_x) * angle_per_pixel
            if abs(angle_error_x) < AIM_ANGLE_THRESHOLD:
                status = "Locked!"; is_aimed = True
            else:
                error_abs = abs(angle_error_x)
                if error_abs > 10: adjust_speed = 12
                elif 4 < error_abs <= 10: adjust_speed = 10
                else: adjust_speed = 8

                if error_abs > 4: steps_x = min(int(error_abs * step_scale), max_steps)
                else: steps_x = 1

                direction_x = 'CCW' if angle_error_x > 0 else 'CW'
                move_motor(uart_x, adjust_speed, direction_x, steps_x)
                status = f"Adjusting ({tracking_state[0]})"
        else:
            if tracking_state == 'TRACKING':
                frames_since_detection += 1
                if frames_since_detection > TRACKING_LOST_THRESHOLD:
                    print(">> Target Lost! Switching back to SEARCHING mode.")
                    tracking_state = 'SEARCHING'
                    last_known_roi = None
                status = "Lost.. Retrying"
            else:
                move_motor(uart_x, X_SEARCH_SPEED, search_direction, X_SEARCH_STEPS)
                status = "Searching.."

        draw_ui_on_image(img, current_mode, status, current_roi_for_drawing)
        Display.show_image(img, x_offset, y_offset)

        if is_aimed: break
        time.sleep(0.02)

    if is_aimed:
        print(">> 目标已锁定，开火！")
        control_laser(True); time.sleep(2)
    else:
        print("!! 瞄准超时 !! 强制开火...")
        control_laser(True); time.sleep(2)

    control_laser(False)
    print(f"--- [基础任务3: {search_direction} 搜索] 执行完毕 ---\n")
    current_mode = MODE_IDLE

# ===================================================================================
#                           4. 主程序入口
# ===================================================================================
init_motor()
clock = time.clock()
image_shape, img_area = [HEIGHT, WIDTH], WIDTH * HEIGHT
img_center_x, img_center_y = WIDTH // 2 -10 , HEIGHT // 2
x_offset = round((lcd_width - WIDTH) / 2)
y_offset = round((lcd_height - HEIGHT) / 2)

print("\n===================================")
print("      系统准备就绪，等待指令")
print("===================================\n")

while True:
    try:
        if current_mode == MODE_IDLE:
            touch_points = tp.read()
            if touch_points and touch_points[0].event == TOUCH.EVENT_DOWN:
                p = touch_points[0]
                img_x = p.x - x_offset
                img_y = p.y - y_offset
                for mode, (x, y, w, h, text) in UI_BUTTONS.items():
                    if x < img_x < x + w and y < img_y < y + h:
                        if mode != MODE_IDLE:
                            current_mode = mode
                            print(f"## 触摸选择模式 -> {MODE_NAMES.get(current_mode)} ##")
                        break

            img = sensor.snapshot()
            img.draw_cross(img_center_x, img_center_y, color=(255,255,255))
            draw_ui_on_image(img, current_mode, "Touch a task to start.")
            Display.show_image(img, x_offset, y_offset)
            time.sleep(0.06)

        elif current_mode == MODE_B2:
            run_task_B2_fixed_aim()
        elif current_mode == MODE_B3_CW:
            run_task_B3_track_aim_generic('CW')
        elif current_mode == MODE_B3_CCW:
            run_task_B3_track_aim_generic('CCW')
        ### 修改: 移除了对 MODE_A12 的处理 ###

        gc.collect()

    except Exception as e:
        print(f"!! 程序发生严重异常: {e} !!")
        import sys
        sys.print_exception(e)
        control_laser(False)
        break

# --- 程序结束清理 ---
sensor.stop(); Display.deinit(); tp.deinit()
control_laser(False)
print("程序已停止，资源已清理")

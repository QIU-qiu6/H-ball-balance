"""MaixCAM-Pro 模型297541钢珠识别稳定版 + 原STM32 UART协议。"""

import os
import time
import traceback

from maix import app


# -------------------- 图像与模型 --------------------
CAM_W = 640
CAM_H = 480
MODEL_W = 224
MODEL_H = 224
ROI_Y = 176                       # 检测带覆盖 y=176..399

# 模型先给出低阈值候选，再使用轨道、尺寸和连续性过滤。
CONF_TH = 0.15
IOU_TH = 0.45
NORMAL_ACCEPT_CONF = 0.28
FAR_END_ACCEPT_CONF = 0.18
TRACKING_ACCEPT_CONF = 0.12
FAR_END_X = 180

MIN_BOX_SIZE = 8
MAX_BOX_SIZE = 72
MIN_ASPECT = 0.52
MAX_ASPECT = 1.92

# 根据新结构实测点 (53,309)、(336,340)、(606,354) 拟合。
TRACK_LINE_Y0 = 307.0
TRACK_LINE_SLOPE = 0.082
TRACK_Y_TOLERANCE = 48

# 丢失时三个窗口逐帧覆盖整条轨道；锁定后窗口跟随钢珠移动。
REACQUIRE_WINDOWS = (0, 208, 416)
MAX_WINDOW_X = CAM_W - MODEL_W
TRACK_BONUS_DISTANCE = 150
MISS_HOLD_FRAMES = 5
TRACK_RESET_FRAMES = 10
DYNAMIC_RETRY_FRAMES = 2
FILTER_ALPHA = 0.68

# -------------------- UART：与现有STM32协议保持一致 --------------------
UART_DEVICE = "/dev/ttyS1"
UART_BAUD = 115200
APP_VERSION = "297541-dynamic-1.3.1"


def clamp_int(value, low, high):
    value = int(value)
    if value < low:
        return low
    if value > high:
        return high
    return value

print("[BOOT] BALL MODEL %s" % APP_VERSION, flush=True)

try:
    from maix import camera, display, image, nn, uart, pinmap, err

    # UART必须先启动，使模型或摄像头异常时也能看到阶段信息。
    err.check_raise(
        pinmap.set_pin_function("A19", "UART1_TX"),
        "A19 UART1_TX config failed",
    )
    err.check_raise(
        pinmap.set_pin_function("A18", "UART1_RX"),
        "A18 UART1_RX config failed",
    )
    vision_uart = uart.UART(UART_DEVICE, UART_BAUD)
    if not vision_uart.is_open():
        raise RuntimeError("UART1 open failed")

    def uart_send(message):
        sent = vision_uart.write_str(message)
        if sent < 0:
            raise RuntimeError("UART1 write failed: %d" % sent)
        return sent

    # 启动标识也沿用现有摄像头工程，STM32端无需作任何适配。
    uart_send("BALL_APP_V110\r\n")
    uart_send("STAGE:UART_OK\r\n")

    project_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(project_dir, "model_297541.mud")
    print("[MODEL]", model_path, flush=True)
    # 按该模型包自带示例的接口初始化，避免不同 MaixPy 固件不支持 dual_buff 参数。
    detector = nn.YOLOv5(model=model_path)

    if (detector.input_width() != MODEL_W or
            detector.input_height() != MODEL_H):
        raise RuntimeError(
            "unexpected model input: %dx%d" %
            (detector.input_width(), detector.input_height())
        )
    uart_send("STAGE:MODEL_OK\r\n")

    cam = camera.Camera(CAM_W, CAM_H, detector.input_format())
    disp = display.Display()
    uart_send("STAGE:CAMERA_OK\r\n")
    print("[READY] YOLOv5 224x224, UART1 A19/A18", flush=True)

    tracked_x = None
    tracked_y = None
    tracked_w = 0
    tracked_h = 0
    miss_count = TRACK_RESET_FRAMES + 1
    frame_count = 0
    reacquire_index = 0

    def detect_window(raw_image, win_x):
        roi = raw_image.crop(win_x, ROI_Y, MODEL_W, MODEL_H)
        objects = detector.detect(roi, conf_th=CONF_TH, iou_th=IOU_TH)
        window_best = None

        for obj in objects:
            if (obj.w < MIN_BOX_SIZE or obj.h < MIN_BOX_SIZE or
                    obj.w > MAX_BOX_SIZE or obj.h > MAX_BOX_SIZE):
                continue

            aspect = float(obj.w) / float(obj.h)
            if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
                continue

            center_x = win_x + obj.x + obj.w // 2
            center_y = ROI_Y + obj.y + obj.h // 2
            expected_y = TRACK_LINE_Y0 + TRACK_LINE_SLOPE * center_x
            if abs(center_y - expected_y) > TRACK_Y_TOLERANCE:
                continue

            if tracked_x is not None and miss_count <= TRACK_RESET_FRAMES:
                # 已锁定时同时受轨道范围、尺寸和运动距离约束，可以采用低门槛
                # 接住反光变化造成的低置信度帧。
                accept_conf = TRACKING_ACCEPT_CONF
            else:
                accept_conf = (FAR_END_ACCEPT_CONF
                               if center_x < FAR_END_X
                               else NORMAL_ACCEPT_CONF)
            if obj.score < accept_conf:
                continue

            rank = obj.score
            if tracked_x is not None and miss_count <= TRACK_RESET_FRAMES:
                dx = center_x - tracked_x
                dy = center_y - tracked_y
                distance = (dx * dx + dy * dy) ** 0.5
                if distance > TRACK_BONUS_DISTANCE * 1.7:
                    continue
                if distance < TRACK_BONUS_DISTANCE:
                    rank += 0.28 * (
                        1.0 - distance / TRACK_BONUS_DISTANCE
                    )

            candidate = {
                "win_x": win_x,
                "x": win_x + obj.x,
                "y": ROI_Y + obj.y,
                "w": obj.w,
                "h": obj.h,
                "cx": center_x,
                "cy": center_y,
                "score": obj.score,
                "rank": rank,
            }
            if window_best is None or rank > window_best["rank"]:
                window_best = candidate

        return window_best

    while not app.need_exit():
        raw = cam.read()
        best = None

        # 锁定后把钢珠放在检测窗口中央；窗口随钢珠在整条轨道上移动。
        # 短暂漏检时仍在最后位置重试，随后切回分区搜索。
        if tracked_x is not None and miss_count <= DYNAMIC_RETRY_FRAMES:
            dynamic_x = clamp_int(
                tracked_x - MODEL_W // 2, 0, MAX_WINDOW_X
            )
            search_windows = (dynamic_x,)
        else:
            search_windows = (REACQUIRE_WINDOWS[reacquire_index],)
            reacquire_index = (reacquire_index + 1) % len(REACQUIRE_WINDOWS)

        for win_x in search_windows:
            candidate = detect_window(raw, win_x)
            if candidate is not None and (
                    best is None or candidate["rank"] > best["rank"]):
                best = candidate

        for win_x in search_windows:
            raw.draw_rect(win_x, ROI_Y, MODEL_W, MODEL_H,
                          color=image.COLOR_GREEN)

        if best is not None:
            measured_x = best["cx"]
            measured_y = best["cy"]

            if tracked_x is None or miss_count > TRACK_RESET_FRAMES:
                tracked_x = float(measured_x)
                tracked_y = float(measured_y)
            else:
                tracked_x = (FILTER_ALPHA * measured_x +
                             (1.0 - FILTER_ALPHA) * tracked_x)
                tracked_y = (FILTER_ALPHA * measured_y +
                             (1.0 - FILTER_ALPHA) * tracked_y)

            tracked_w = best["w"]
            tracked_h = best["h"]
            miss_count = 0
            center_x = int(tracked_x + 0.5)
            center_y = int(tracked_y + 0.5)

            raw.draw_rect(best["x"], best["y"], best["w"], best["h"],
                          color=image.COLOR_RED)
            raw.draw_string(
                best["x"], max(0, best["y"] - 16),
                "G:%.2f" % best["score"], color=image.COLOR_RED,
            )

            # STM32接收格式保持不变，坐标仍为640x480原图坐标。
            tx_len = uart_send("X:%d,Y:%d,W:%d,H:%d\r\n" % (
                center_x, center_y, tracked_w, tracked_h
            ))
            if frame_count % 5 == 0:
                print("[BALL] X=%d Y=%d W=%d H=%d score=%.3f TX=%d" % (
                    center_x, center_y, tracked_w, tracked_h,
                    best["score"], tx_len
                ), flush=True)
        else:
            miss_count += 1
            if tracked_x is not None and miss_count <= MISS_HOLD_FRAMES:
                center_x = int(tracked_x + 0.5)
                center_y = int(tracked_y + 0.5)
                hold_x = int(center_x - tracked_w // 2)
                hold_y = int(center_y - tracked_h // 2)
                raw.draw_rect(hold_x, hold_y, tracked_w, tracked_h,
                              color=image.COLOR_GREEN)
                raw.draw_string(
                    hold_x, max(0, hold_y - 16),
                    "HOLD:%d" % miss_count, color=image.COLOR_GREEN,
                )
                uart_send("X:%d,Y:%d,W:%d,H:%d\r\n" % (
                    center_x, center_y, tracked_w, tracked_h
                ))
                print("[HOLD] miss=%d X=%d Y=%d" % (
                    miss_count, center_x, center_y
                ), flush=True)
            else:
                uart_send("NO_TARGET\r\n")
                if frame_count % 20 == 0:
                    print("[NO TARGET] miss=%d" % miss_count, flush=True)

            if miss_count > TRACK_RESET_FRAMES:
                tracked_x = None
                tracked_y = None

        if frame_count % 20 == 0:
            uart_send("UART1_ALIVE:%d\r\n" % frame_count)

        frame_count += 1
        disp.show(raw)

except Exception:
    print("\n========== MODEL297541 ERROR ==========", flush=True)
    traceback.print_exc()
    print("=======================================\n", flush=True)
    while not app.need_exit():
        time.sleep(0.2)

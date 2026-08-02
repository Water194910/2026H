from maix import app, camera, display, image, nn, pinmap, touchscreen, uart


MODEL_PATH = "/mycode/26_1/best.mud"
BALL_LABELS = ["ball"]
IOU_TH = 0.45
DETECT_ROI = (126, 55, 33, 265)
# 兼容暗绿色和偏青绿色，几何筛选仍负责排除非目标区域。
GREEN_THRESHOLD = (15, 100, -128, -10, -80, 127)
GREEN_RECT_LENGTH_MM = 250.0
TARGET_LIMIT_MM = 110.0
TARGET_STEP_MM = 5.0
MIN_GREEN_PIXELS = 500
MIN_GREEN_FILL = 0.45
MIN_GREEN_ASPECT = 4.0
MIN_GREEN_WIDTH = 8
MAX_GREEN_WIDTH = 33
MIN_GREEN_HEIGHT = 180
MAX_GREEN_HEIGHT = 265
MAX_GREEN_CENTER_OFFSET = 60
MAX_GREEN_HEIGHT_CHANGE = 0.10
MAX_GREEN_CENTER_JUMP = 10
GREEN_LOCK_FRAMES = 5
GREEN_GEOMETRY_FILTER_SIZE = 5
GREEN_RELOCK_MISSES = 10
MIN_BALL_OVERLAP = 0.25
DISTANCE_FILTER_SIZE = 5
MAX_MISSED_FRAMES = 5

PARAMS = [
    {"name": "CONF", "value": 40, "min": 5, "max": 90},
]
TAB_H = 46
ZERO_BAR_Y = TAB_H
ZERO_BAR_H = 40
ZERO_BUTTONS = ("-5", "+5", "OK", "DEFAULT")
ZERO_STATUS_Y = ZERO_BAR_Y + ZERO_BAR_H + 3
BALL_STATUS_Y = ZERO_STATUS_Y + 20

detector = nn.YOLO11(model=MODEL_PATH, dual_buff=False)
cam = camera.Camera(320, 320)
disp = display.Display()
ts = touchscreen.TouchScreen()

pinmap.set_pin_function("A21", "UART4_TX")
pinmap.set_pin_function("A22", "UART4_RX")
serial = uart.UART("/dev/ttyS4", 115200)

select_param = 0
last_pressed = False
touch_start_x = 0
touch_start_value = 0
touch_adjusting = False
distance_samples = []
missed_ball_frames = 0
green_rect_samples = []
locked_green_rect = None
green_stable_frames = 0
green_missed_frames = 0
target_center_ratio = 0.5
pending_center_ratio = 0.5
manual_center_set = False


def clamp(value, low, high):
    return max(low, min(value, high))


def get_param(name):
    for param in PARAMS:
        if param["name"] == name:
            return param["value"]
    return 0


def get_conf_th():
    return get_param("CONF") / 100.0


def target_mm_from_ratio(ratio):
    return (ratio - 0.5) * GREEN_RECT_LENGTH_MM


def ratio_from_target_mm(target_mm):
    target_mm = clamp(target_mm, -TARGET_LIMIT_MM, TARGET_LIMIT_MM)
    return 0.5 + target_mm / GREEN_RECT_LENGTH_MM


def adjust_pending_target(delta_mm):
    global pending_center_ratio
    target_mm = target_mm_from_ratio(pending_center_ratio) + delta_mm
    pending_center_ratio = ratio_from_target_mm(target_mm)


def commit_pending_target():
    global target_center_ratio, manual_center_set
    target_center_ratio = pending_center_ratio
    manual_center_set = abs(target_mm_from_ratio(target_center_ratio)) > 0.05


def restore_default_target():
    global target_center_ratio, pending_center_ratio, manual_center_set
    target_center_ratio = 0.5
    pending_center_ratio = 0.5
    manual_center_set = False


def label_is_ball(class_id):
    if class_id < 0 or class_id >= len(detector.labels):
        return False
    return detector.labels[class_id] in BALL_LABELS


def get_touch(img):
    x, y, pressed = ts.read()
    if not pressed:
        return 0, 0, False

    try:
        screen_width = disp.width()
        screen_height = disp.height()
    except Exception:
        screen_width = img.width()
        screen_height = img.height()

    if screen_width > 0 and screen_height > 0:
        x = int(x * img.width() / screen_width)
        y = int(y * img.height() / screen_height)

    return x, y, True


def handle_touch(img):
    global select_param, last_pressed
    global touch_start_x, touch_start_value, touch_adjusting
    global pending_center_ratio

    x, y, pressed = get_touch(img)
    if not pressed:
        last_pressed = False
        touch_adjusting = False
        return

    if y < TAB_H:
        if not last_pressed:
            tab_width = max(1, img.width() // len(PARAMS))
            select_param = clamp(x // tab_width, 0, len(PARAMS) - 1)
        last_pressed = True
        touch_adjusting = False
        return

    if ZERO_BAR_Y <= y < ZERO_BAR_Y + ZERO_BAR_H:
        if not last_pressed:
            button_width = max(1, img.width() // len(ZERO_BUTTONS))
            button = clamp(x // button_width, 0, len(ZERO_BUTTONS) - 1)
            if button == 0:
                adjust_pending_target(-TARGET_STEP_MM)
            elif button == 1:
                adjust_pending_target(TARGET_STEP_MM)
            elif button == 2:
                commit_pending_target()
            else:
                restore_default_target()
        last_pressed = True
        touch_adjusting = False
        return

    roi_x, roi_y, roi_w, roi_h = DETECT_ROI
    if roi_x <= x < roi_x + roi_w and roi_y <= y < roi_y + roi_h:
        if locked_green_rect is not None:
            _, ref_y, _, ref_h = locked_green_rect
            ratio = (y - ref_y) / max(1, ref_h)
            pending_center_ratio = ratio_from_target_mm(target_mm_from_ratio(ratio))
        last_pressed = True
        touch_adjusting = False
        return

    param = PARAMS[select_param]
    if not touch_adjusting:
        touch_start_x = x
        touch_start_value = param["value"]
        touch_adjusting = True
    else:
        value_span = param["max"] - param["min"]
        delta = int((x - touch_start_x) * value_span / max(1, img.width()))
        param["value"] = clamp(touch_start_value + delta, param["min"], param["max"])

    last_pressed = True


def draw_ui(img):
    tab_width = max(1, img.width() // len(PARAMS))
    for index, param in enumerate(PARAMS):
        x = index * tab_width
        width = img.width() - x if index == len(PARAMS) - 1 else tab_width
        color = image.COLOR_GREEN if index == select_param else image.COLOR_WHITE
        img.draw_rect(x, 0, width, TAB_H, color=color)
        img.draw_string(x + 3, 4, param["name"], color=color)
        img.draw_string(x + 3, 23, str(param["value"]), color=color)


def draw_zero_ui(img):
    button_width = max(1, img.width() // len(ZERO_BUTTONS))
    for index, label in enumerate(ZERO_BUTTONS):
        x = index * button_width
        width = img.width() - x if index == len(ZERO_BUTTONS) - 1 else button_width
        color = image.COLOR_GREEN if label == "OK" else image.COLOR_WHITE
        img.draw_rect(x, ZERO_BAR_Y, width, ZERO_BAR_H, color=color)
        img.draw_string(x + 5, ZERO_BAR_Y + 10, label, color=color)

    pending_mm = target_mm_from_ratio(pending_center_ratio)
    target_mm = target_mm_from_ratio(target_center_ratio)
    img.draw_string(
        0,
        ZERO_STATUS_Y,
        "SET:%+.1fmm TARGET:%+.1fmm" % (pending_mm, target_mm),
        color=image.COLOR_YELLOW,
    )


def center_in_roi(obj, roi):
    if roi is None:
        return False
    x, y, w, h = roi
    center_x = obj.x + obj.w // 2
    center_y = obj.y + obj.h // 2
    return x <= center_x < x + w and y <= center_y < y + h


def object_overlap_ratio(obj, roi):
    x, y, w, h = roi
    left = max(obj.x, x)
    top = max(obj.y, y)
    right = min(obj.x + obj.w, x + w)
    bottom = min(obj.y + obj.h, y + h)
    overlap = max(0, right - left) * max(0, bottom - top)
    return overlap / max(1, obj.w * obj.h)


def find_green_rect(img):
    blobs = img.find_blobs(
        [GREEN_THRESHOLD],
        roi=DETECT_ROI,
        pixels_threshold=MIN_GREEN_PIXELS,
        area_threshold=MIN_GREEN_PIXELS,
        merge=True,
        margin=30,
    )

    rectangles = []
    for blob in blobs:
        box_area = max(1, blob.w() * blob.h())
        fill = blob.pixels() / box_area
        aspect = blob.h() / max(1, blob.w())
        if fill >= MIN_GREEN_FILL and aspect >= MIN_GREEN_ASPECT:
            rectangles.append(blob)

    if not rectangles:
        return None

    blob = max(rectangles, key=lambda item: (item.h(), item.pixels()))
    return blob.x(), blob.y(), blob.w(), blob.h()


def median_value(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def median_green_rect(rectangles):
    center_x = median_value([x + w / 2.0 for x, _, w, _ in rectangles])
    center_y = median_value([y + h / 2.0 for _, y, _, h in rectangles])
    width = median_value([w for _, _, w, _ in rectangles])
    height = median_value([h for _, _, _, h in rectangles])
    return (
        int(round(center_x - width / 2.0)),
        int(round(center_y - height / 2.0)),
        int(round(width)),
        int(round(height)),
    )


def green_geometry_valid(rect):
    if rect is None:
        return False
    _, y, w, h = rect
    _, roi_y, _, roi_h = DETECT_ROI
    center_y = y + h / 2.0
    roi_center_y = roi_y + roi_h / 2.0
    return (
        MIN_GREEN_WIDTH <= w <= MAX_GREEN_WIDTH
        and MIN_GREEN_HEIGHT <= h <= MAX_GREEN_HEIGHT
        and abs(center_y - roi_center_y) <= MAX_GREEN_CENTER_OFFSET
    )


def green_geometry_close(rect, reference):
    _, y, _, h = rect
    _, ref_y, _, ref_h = reference
    center_y = y + h / 2.0
    ref_center_y = ref_y + ref_h / 2.0
    height_change = abs(h - ref_h) / max(1, ref_h)
    return (
        height_change <= MAX_GREEN_HEIGHT_CHANGE
        and abs(center_y - ref_center_y) <= MAX_GREEN_CENTER_JUMP
    )


def stabilize_green_rect(raw_rect):
    global locked_green_rect, green_stable_frames, green_missed_frames

    if not green_geometry_valid(raw_rect):
        green_missed_frames += 1
        if locked_green_rect is not None and green_missed_frames < GREEN_RELOCK_MISSES:
            return locked_green_rect
        locked_green_rect = None
        green_rect_samples.clear()
        green_stable_frames = 0
        return None

    if locked_green_rect is not None:
        if green_geometry_close(raw_rect, locked_green_rect):
            green_missed_frames = 0
            green_rect_samples.append(raw_rect)
            if len(green_rect_samples) > GREEN_GEOMETRY_FILTER_SIZE:
                green_rect_samples.pop(0)
            locked_green_rect = median_green_rect(green_rect_samples)
            return locked_green_rect

        green_missed_frames += 1
        if green_missed_frames < GREEN_RELOCK_MISSES:
            return locked_green_rect

        locked_green_rect = None
        green_rect_samples.clear()
        green_stable_frames = 0

    if green_rect_samples:
        reference = median_green_rect(green_rect_samples)
        if not green_geometry_close(raw_rect, reference):
            green_rect_samples.clear()
            green_stable_frames = 0

    green_rect_samples.append(raw_rect)
    if len(green_rect_samples) > GREEN_GEOMETRY_FILTER_SIZE:
        green_rect_samples.pop(0)
    green_stable_frames += 1
    green_missed_frames = 0

    if green_stable_frames >= GREEN_LOCK_FRAMES:
        locked_green_rect = median_green_rect(green_rect_samples)
        return locked_green_rect
    return None


def find_best_ball(objects, target_roi):
    if target_roi is None:
        return None
    candidates = [
        obj
        for obj in objects
        if label_is_ball(obj.class_id)
        and center_in_roi(obj, target_roi)
        and object_overlap_ratio(obj, target_roi) >= MIN_BALL_OVERLAP
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda obj: obj.score)


def get_target_center_y(green_rect):
    _, rect_y, _, rect_h = green_rect
    return rect_y + rect_h * target_center_ratio


def get_pending_center_y(green_rect):
    _, rect_y, _, rect_h = green_rect
    return rect_y + rect_h * pending_center_ratio


def ball_position_from_center_mm(ball, green_rect):
    _, rect_y, _, rect_h = green_rect
    rect_center_y = rect_y + rect_h * 0.5
    ball_center_y = ball.y + ball.h / 2.0
    mm_per_pixel = GREEN_RECT_LENGTH_MM / rect_h
    return (ball_center_y - rect_center_y) * mm_per_pixel


def distance_from_target_mm(ball, green_rect):
    return ball_position_from_center_mm(ball, green_rect) - target_mm_from_ratio(
        target_center_ratio
    )


def filter_distance_mm(value):
    distance_samples.append(value)
    if len(distance_samples) > DISTANCE_FILTER_SIZE:
        distance_samples.pop(0)

    ordered = sorted(distance_samples)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


while not app.need_exit():
    img = cam.read()
    handle_touch(img)

    raw_green_rect = find_green_rect(img)
    green_rect = stabilize_green_rect(raw_green_rect)
    objects = detector.detect(img, conf_th=get_conf_th(), iou_th=IOU_TH)
    ball = find_best_ball(objects, green_rect)

    img.draw_rect(*DETECT_ROI, color=image.COLOR_YELLOW, thickness=2)
    if green_rect is not None:
        img.draw_rect(*green_rect, color=image.COLOR_BLUE, thickness=2)
        target_x = green_rect[0] + green_rect[2] // 2
        target_y = int(round(get_target_center_y(green_rect)))
        pending_y = int(round(get_pending_center_y(green_rect)))
        img.draw_cross(target_x, target_y, color=image.COLOR_WHITE)
        if pending_y != target_y:
            img.draw_cross(target_x, pending_y, color=image.COLOR_YELLOW)

    distance_mm = None
    if ball is not None:
        missed_ball_frames = 0
        obj = ball
        center_x = obj.x + obj.w // 2
        center_y = obj.y + obj.h // 2
        raw_position_mm = ball_position_from_center_mm(obj, green_rect)
        position_mm = filter_distance_mm(raw_position_mm)
        target_mm = target_mm_from_ratio(target_center_ratio)
        distance_mm = position_mm - target_mm
        serial.write(
            ("P,%+.2f,%+.2f\n" % (position_mm, target_mm)).encode("ascii")
        )
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_GREEN)
        img.draw_cross(center_x, center_y, color=image.COLOR_RED)
        label_x = max(1, obj.x)
        label_y = max(TAB_H + 1, obj.y - 14)
        label = "P:%+.1f E:%+.1fmm" % (position_mm, distance_mm)
        for offset_x, offset_y in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            img.draw_string(
                label_x + offset_x,
                label_y + offset_y,
                label,
                color=image.COLOR_BLACK,
            )
        img.draw_string(label_x, label_y, label, color=image.COLOR_YELLOW)
    else:
        missed_ball_frames += 1
        if missed_ball_frames >= MAX_MISSED_FRAMES:
            distance_samples.clear()

    status_color = image.COLOR_GREEN if ball is not None else image.COLOR_WHITE
    img.draw_string(
        0,
        BALL_STATUS_Y,
        "BALL:%d GREEN:%s C:%d%s RAW:%d"
        % (
            1 if ball is not None else 0,
            "LOCK" if green_rect is not None else "WAIT",
            int(round(target_center_ratio * 100)),
            "M" if manual_center_set else "A",
            len(objects),
        ),
        color=status_color,
    )

    draw_ui(img)
    draw_zero_ui(img)
    disp.show(img)

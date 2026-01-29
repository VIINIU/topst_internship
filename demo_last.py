"""
자율주행 차량 제어 시스템
- 차선 인식 (Lane Detection)
- 장애물 감지 및 회피 (Obstacle Avoidance)
- 조향/속도 제어 (Steering/Speed Control)
"""

import json
import time
import socket
import threading
import select
import cv2
import numpy as np

from Library.IPC_Library import IPC_SendPacketWithIPCHeader

# =========================================================
# 상수 정의
# =========================================================

# IPC 신호
class VCP_IO:
    """차량 제어 신호 정의"""
    BREAK_LIGHT = 0x101
    TURN_SIGNAL = 0x102
    EMER_SIGNAL = 0x103
    HEAD_LIGHT = 0x104
    FUEL_L = 0x105
    MOTOR_A = 0x106
    WHEEL = 0x107
    
    ACTION_ON = 0x01
    ACTION_OFF = 0x02
    
    SUB_LEFT = 0x01
    SUB_RIGHT = 0x02

# AI 서버 설정
AI_CONFIG = {
    "IP": "192.168.0.100",
    "PORT": 9999,
    "TIMEOUT": 3.0,
    "SELECT_TIMEOUT": 0.2,
}

# AI-G 카메라 설정
AI_CAMERA_CONFIG = {
    "WIDTH": 800,
    "HEIGHT": 480,
    # "Y_THRESHOLD" : 100,
}

# 카메라 설정
CAMERA_CONFIG = {
    "WIDTH": 640,
    "HEIGHT": 480,
    "INDEX": 1,
    "BUFFER_SIZE": 1,
}

# 주행 제어
DRIVING_CONFIG = {
    "MAX_SPEED": 45,             # 현재 속도
    "SPEED_INCREMENT": 2,       # 속도 증가량
    "SPEED_DECREMENT": 2,       # 속도 감소량
    "ACCEL_INTERVAL": 0.5,       # 속도 증감 시간
}

# 조향 제어
STEER_CONFIG = {
    "CENTER": 65,
    "MIN": 0,
    "MAX": 127,
    "STEP": 10,
    "HZ": 10,
    "P_GAIN": 0.5,
}

# 깜빡이
BLINK_INTERVAL = 0.5

# 장애물 감지
OBSTACLE_CONFIG = {
    "TIMEOUT": 0.5,
    "CLOSE_RATIO": 0.12,
    "HARD_LEFT": 10,
    "HARD_RIGHT": 120,
    "SOFT_LEFT": 40,
    "SOFT_RIGHT": 90
}

# =========================================================
# 전역 상태
# =========================================================

_zone_state = {"L": (False, False), "F": (False, False), "R": (False, False)}
_can = {
    "park": False,
    "lane_num": 2,
    "avoid_mode": False,
    "left_blinker": False,
    "right_blinker": False,
    "headlights": False,
    "fuel_l": 0,
    "speed_kmh": 0,
    "is_accelerating": False,
    "is_braking": False,
    "is_resverse": False,
    "is_steering": False,
    "is_steer_reverse": False,
    "steer": STEER_CONFIG["CENTER"],
    "target_steer": STEER_CONFIG["CENTER"],
}

_ipc_cache = {}  # IPC 전송 최적화용
_lock = threading.Lock()
_stop = False

# IPC 파일 열기
sndfile = open("/dev/tcc_ipc_micom", 'wb')


# =========================================================
# 장애물 분석 클래스
# =========================================================

class ObjectAnalytics:
    """AI-G에서 받은 객체 정보를 분석하여 장애물 위치 결정"""
    
    def __init__(self, width=AI_CAMERA_CONFIG["WIDTH"], height=AI_CAMERA_CONFIG["HEIGHT"]):
        self.w, self.h = width, height
        self.total_area = width * height
        self.close_area_ratio = OBSTACLE_CONFIG["CLOSE_RATIO"]
        self.timeout = OBSTACLE_CONFIG["TIMEOUT"]
        
        # 각 구역(L/F/R)별 상태
        self.zones = {
            'L': {'obstacle': False, 'close': False, 'last_update': 0.0},
            'F': {'obstacle': False, 'close': False, 'last_update': 0.0},
            'R': {'obstacle': False, 'close': False, 'last_update': 0.0},
        }
        
        self.frame_count = 0
        self.last_result = ((False, False), (False, False), (False, False))

    def update_status(self, json_data: dict):
        """3프레임마다 한 번씩 연산 (CPU 최적화)"""
        self.frame_count += 1
        # y_threshold = AI_CAMERA_CONFIG["Y_THRESHOLD"]
        
        # 3번째 프레임이 아니면 이전 결과 반환
        if self.frame_count % 3 != 0:
            return self.last_result

        current_time = time.time()

        # 새 데이터 리셋
        for zone in self.zones.values():
            zone['obstacle'] = False
            zone['close'] = False

        # 객체 분석
        if json_data and 'boxes' in json_data:
            for det in json_data.get('boxes', []):
                if not isinstance(det, dict):
                    continue

                xmin, xmax = det.get('xmin', 0), det.get('xmax', 0)
                ymin, ymax = det.get('ymin', 0), det.get('ymax', 0)
                
                obj_w, obj_h = xmax - xmin, ymax - ymin
                obj_area = obj_w * obj_h
                center_x = xmin + ( obj_w / 2.0 )
                
                # if ymin < y_threshold:
                #     continue

                # 구역 판단
                div_1 = self.w / 2.8
                div_2 = self.w * (1.8 / 2.8)
                
                if center_x < div_1:
                    zone = 'L'
                elif center_x > div_2:
                    zone = 'R'
                else:
                    zone = 'F'

                # 상태 업데이트
                self.zones[zone]['obstacle'] = True
                self.zones[zone]['last_update'] = current_time
                
                if (obj_area / self.total_area) > self.close_area_ratio:
                    self.zones[zone]['close'] = True

        # 타임아웃 처리
        for zone in self.zones.items():
            if current_time - zone['last_update'] > self.timeout:
                zone['obstacle'] = False
                zone['close'] = False

        # 결과 저장
        self.last_result = (
            (self.zones['L']['obstacle'], self.zones['L']['close']),
            (self.zones['F']['obstacle'], self.zones['F']['close']),
            (self.zones['R']['obstacle'], self.zones['R']['close']),
        )

        return self.last_result


# =========================================================
# 차선 인식 클래스
# =========================================================

class LaneVisualizer:
    """카메라 영상에서 차선 감지"""
    
    def __init__(self):
        self.CAR_CENTER_X = CAMERA_CONFIG["WIDTH"] // 2
        self.ROI_TOP_RATIO = 0.3
        self.GAMMA = 0.6
        self.Y_EVAL = CAMERA_CONFIG["HEIGHT"] - 10
        self.MIN_Y_MAX = int(CAMERA_CONFIG["HEIGHT"] * 0.3)

        self.lane_width_est = 350.0
        self.W_ALPHA = 0.12
        self.W_MIN, self.W_MAX = 160.0, 700.0

        self.STEER_YS = [350, 400, 450]
        self.STEER_W = [0.4, 0.3, 0.3]

        self.STEER_GAIN = 0.9
        self.DEADZONE = 0.01
        self.ALPHA = 0.35
        self.MAX_DELTA = 0.15
        self.prev_steer = 0.0

        self.DEBUG = True
        self.current_lane = 2
        self.candidate_lane = 2
        self.lane_confirm_cnt = 0
        self.CONFIRM_THRESHOLD = 5

    def calc_angle_term(self, poly_func, y_ref, img_width):
        """차선의 기울기 항 계산"""
        dpoly = np.polyder(poly_func)
        slope = float(dpoly(y_ref))
        angle_norm = slope / (img_width * 0.5)
        return float(np.clip(angle_norm, -1.0, 1.0))

    def process_frame(self, frame):
        """프레임 처리: 차선 감지 및 조향값 계산"""
        h, w = frame.shape[:2]

        # 전처리
        darker = self.adjust_gamma(frame, gamma=self.GAMMA)
        blur = cv2.GaussianBlur(darker, (5, 5), 0)
        yellow_mask, white_mask = self.get_color_masks(blur)

        # ROI 적용
        roi_yellow = self.apply_roi_full(yellow_mask)
        roi_white = self.apply_roi_full(white_mask)

        # 차선 후보 추출
        candidates = []
        candidates += self.find_lane_candidates(roi_yellow, "Yellow")
        candidates += self.find_lane_candidates(roi_white, "White")

        # 좌/우 차선 분리
        left_candidates = [c for c in candidates if c['x_eval'] < self.CAR_CENTER_X]
        right_candidates = [c for c in candidates if c['x_eval'] >= self.CAR_CENTER_X]

        def score(c):
            dist = abs(c['x_eval'] - self.CAR_CENTER_X)
            return (c['y_max'] * 10.0) - dist

        left_lane = max(left_candidates, key=score) if left_candidates else None
        right_lane = max(right_candidates, key=score) if right_candidates else None

        # 차선 번호 판단
        self._update_lane_number(left_lane, right_lane)

        # 이미지 준비
        result_img = frame.copy()
        draw_ys = np.arange(int(h * self.ROI_TOP_RATIO), h)

        if self.DEBUG:
            cv2.line(result_img, (self.CAR_CENTER_X, 0), (self.CAR_CENTER_X, h),
                    (255, 0, 255), 2)

        # 차선 그리기
        final_left_poly = None
        final_right_poly = None
        if left_lane:
            final_left_poly = left_lane['poly']
            self.draw_poly(result_img, final_left_poly, draw_ys, (0, 255, 255), "L")
        if right_lane:
            final_right_poly = right_lane['poly']
            self.draw_poly(result_img, final_right_poly, draw_ys, (255, 255, 0), "R")

        # 조향값 계산
        steer = self._calculate_steering(
            final_left_poly, final_right_poly, draw_ys, w, result_img
        )

        # 차선 상태 표시
        lane_status = self._get_lane_status(final_left_poly, final_right_poly)
        cv2.putText(result_img, f"Lane: {lane_status}", (20, 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        return result_img, steer, self.current_lane

    def _update_lane_number(self, left_lane, right_lane):
        """차선 번호 업데이트 (히스테리시스)"""
        detected_lane = 0
        
        if left_lane and right_lane:
            l_cls, r_cls = left_lane['color'], right_lane['color']
            if l_cls == "Yellow" and r_cls == "White":
                detected_lane = 1
            elif l_cls == "White" and r_cls == "White":
                detected_lane = 2
            elif l_cls == "White" and r_cls == "Yellow":
                detected_lane = 3

        if detected_lane != 0:
            if detected_lane == self.candidate_lane:
                self.lane_confirm_cnt += 1
            else:
                self.candidate_lane = detected_lane
                self.lane_confirm_cnt = 1

            if self.lane_confirm_cnt >= self.CONFIRM_THRESHOLD:
                self.current_lane = self.candidate_lane

    def _calculate_steering(self, left_poly, right_poly, draw_ys, w, img):
        """조향값 계산 (OFFSET + ANGLE)"""
        if left_poly is None and right_poly is None:
            return self.prev_steer

        # 중앙선 결정
        if left_poly is not None and right_poly is not None:
            ys = np.array(self.STEER_YS, dtype=np.float32)
            w_now = float(np.mean(right_poly(ys) - left_poly(ys)))
            if self.W_MIN < w_now < self.W_MAX:
                self.lane_width_est = (1 - self.W_ALPHA) * self.lane_width_est + self.W_ALPHA * w_now
            center_poly = lambda y: (left_poly(y) + right_poly(y)) / 2.0

        elif left_poly is not None:
            center_poly = lambda y: left_poly(y) + (self.lane_width_est / 2.0)
        else:
            center_poly = lambda y: right_poly(y) - (self.lane_width_est / 2.0)

        # 조향 계산
        xs, ws, angle_terms = [], [], []
        ref_poly = left_poly if left_poly is not None else right_poly

        for y, wgt in zip(self.STEER_YS, self.STEER_W):
            yy = int(np.clip(y, draw_ys[0], draw_ys[-1]))
            xx = float(np.clip(center_poly(yy), 0, w - 1))
            xs.append(xx)
            ws.append(wgt)
            angle_terms.append(self.calc_angle_term(ref_poly, y_ref=yy, img_width=w))

        x_target = float(np.average(xs, weights=ws))
        error_px = x_target - self.CAR_CENTER_X
        offset_term = float(np.clip(error_px / (w / 4.0), -1.0, 1.0))
        angle_term = float(np.mean(angle_terms)) if angle_terms else 0.0

        # 가중합
        raw = (0.7 * offset_term + 0.3 * angle_term) * 1.5
        raw = float(np.clip(raw, -1.0, 1.0))

        if abs(raw) < self.DEADZONE:
            raw = 0.0

        # LPF + Slew-rate
        filt = (1 - self.ALPHA) * self.prev_steer + self.ALPHA * raw
        filt = float(np.clip(filt, self.prev_steer - self.MAX_DELTA, 
                            self.prev_steer + self.MAX_DELTA))
        self.prev_steer = filt

        if self.DEBUG:
            cv2.putText(img, f"offset={offset_term:+.2f} angle={angle_term:+.2f} steer={filt:+.2f}",
                       (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(img, f"Lane: {self.current_lane}", (20, 125),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return filt

    def _get_lane_status(self, left_poly, right_poly):
        """현재 차선 상태 문자열"""
        if left_poly is not None and right_poly is not None:
            return "Both"
        elif left_poly is not None:
            return "Left"
        elif right_poly is not None:
            return "Right"
        else:
            return "Unknown"

    def find_lane_candidates(self, mask, color_label):
        """차선 후보 추출"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if float(w) / max(h, 1) > 8.0:
                continue

            try:
                pts = cnt.reshape(-1, 2).astype(np.float32)
                y_max = float(pts[:, 1].max())
                
                if y_max < self.MIN_Y_MAX:
                    continue

                poly_coeffs = np.polyfit(pts[:, 1], pts[:, 0], 2)
                poly_func = np.poly1d(poly_coeffs)
                x_eval = float(poly_func(min(self.Y_EVAL, y_max)))

                candidates.append({
                    'poly': poly_func,
                    'color': color_label,
                    'x_eval': x_eval,
                    'y_eval': y_max,
                    'y_max': y_max
                })
            except:
                pass

        return candidates

    def draw_poly(self, img, poly_func, y_range, color, label):
        """다항식 곡선 그리기"""
        try:
            x_plot = np.clip(poly_func(y_range).astype(int), 0, img.shape[1] - 1)
            pts = np.column_stack((x_plot, y_range))
            cv2.polylines(img, [pts], False, color, 4)
            cv2.putText(img, label, (int(x_plot[0]), int(y_range[0]) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        except:
            pass

    def adjust_gamma(self, image, gamma=1.0):
        """감마 보정"""
        inv = 1.0 / max(gamma, 1e-6)
        table = np.array([((i / 255.0) ** inv) * 255 for i in range(256)], dtype=np.uint8)
        return cv2.LUT(image, table)

    def get_color_masks(self, frame):
        """HSV 기반 색상 마스크 추출"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        yellow_mask = cv2.inRange(hsv, (15, 80, 80), (40, 255, 255))
        white_mask = cv2.inRange(hsv, (0, 0, 210), (180, 50, 255))

        kernel = np.ones((3, 3), np.uint8)
        return (cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel),
                cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel))

    def apply_roi_full(self, mask):
        """전체 구간 ROI 적용"""
        height, width = mask.shape
        roi = np.zeros_like(mask)
        top = int(height * self.ROI_TOP_RATIO)
        
        poly = np.array([[(0, height), (width, height), (width, top), (0, top)]], np.int32)
        cv2.fillPoly(roi, poly, 255)
        return cv2.bitwise_and(mask, roi)


# =========================================================
# IPC 통신
# =========================================================

def send_ipc_signal(io_type, value, subtype=None):
    """IPC 신호 전송 (변경이 있을 때만)"""
    if sndfile is None:
        return False

    cache_key = (io_type, subtype)

    # 캐시된 값과 같으면 스킵
    if _ipc_cache.get(cache_key) == value:
        return True

    try:
        payload = bytes([subtype, value]) if subtype else value.to_bytes(1, 'big', signed=True)
        IPC_SendPacketWithIPCHeader(sndfile, 1, 0, io_type, payload)
        _ipc_cache[cache_key] = value
        return True
    except Exception as e:
        _ipc_cache.pop(cache_key, None)
        return False


# =========================================================
# 조향 제어
# =========================================================

def wheel_controller():
    """조향 제어 (P-Control + 회피 모드 통합)"""
    dt = 1.0 / float(STEER_CONFIG["HZ"])
    Kp = STEER_CONFIG["P_GAIN"]
    step_limit = STEER_CONFIG["STEP"]
    
    # 초기값 설정
    current_steer_f = float(STEER_CONFIG["CENTER"])

    while not _stop:
        with _lock:
            is_avoid = _can.get("avoid_mode", False)
            target_steer = float(_can.get("target_steer", STEER_CONFIG["CENTER"]))
            hw_steer = _can.get("steer", STEER_CONFIG["CENTER"])

        if is_avoid:
            current_steer_f = float(hw_steer)

        diff = target_steer - current_steer_f

        if is_avoid:
            delta = np.clip(diff, -step_limit, step_limit)
        else:
            delta = diff * Kp
            
        current_steer_f += delta
        final_out = int(np.clip(current_steer_f, STEER_CONFIG["MIN"], STEER_CONFIG["MAX"]))

        with _lock:
            _can["steer"] = final_out

        send_ipc_signal(VCP_IO.WHEEL, final_out)
        time.sleep(dt)


# =========================================================
# 속도 제어
# =========================================================

def speed_controller():
    """속도 및 브레이크 제어"""
    while not _stop:
        with _lock:
            is_accel = _can.get("is_accelerating", False)
            is_brake = _can.get("is_braking", False)
            is_avoid = _can.get("avoid_mode", False)
            current_speed = _can.get("speed_kmh", 0)
            steer = _can.get("steer", STEER_CONFIG["CENTER"])

        new_speed = current_speed

        # 안전 로직: 회피 모드가 아니고 조향이 위험할 때 
        # 너무 벗어날 경우를 대비해 속도 감소 모드 실행
        if not is_avoid and (steer < 20 or steer > 110):
            is_brake = True

        # 가속/감속
        if is_accel and not is_brake:
            send_ipc_signal(VCP_IO.BREAK_LIGHT, VCP_IO.ACTION_OFF)
            new_speed = min(current_speed + DRIVING_CONFIG["SPEED_INCREMENT"], DRIVING_CONFIG["MAX_SPEED"])
            with _lock:
                _can["speed_kmh"] = new_speed

        elif is_brake:
            send_ipc_signal(VCP_IO.BREAK_LIGHT, VCP_IO.ACTION_ON)
            new_speed = max(current_speed - DRIVING_CONFIG["SPEED_DECREMENT"], 0)
            with _lock:
                _can["speed_kmh"] = new_speed

        else:
            send_ipc_signal(VCP_IO.BREAK_LIGHT, VCP_IO.ACTION_OFF)

        send_ipc_signal(VCP_IO.MOTOR_A, new_speed)
        time.sleep(DRIVING_CONFIG["ACCEL_INTERVAL"])


# =========================================================
# LED 제어
# =========================================================

def get_blink_state():
    """깜빡이 상태"""
    cycle = time.time() % (BLINK_INTERVAL * 2)
    return cycle < BLINK_INTERVAL


def led_worker():
    """비상등 및 방향 깜빡이 제어"""
    while not _stop:
        with _lock:
            emer = _can.get("avoid_mode", False)
            left = _can.get("left_blinker", False)
            right = _can.get("right_blinker", False)

        on = get_blink_state()
        action = VCP_IO.ACTION_ON if on else VCP_IO.ACTION_OFF

        if emer:
            send_ipc_signal(VCP_IO.TURN_SIGNAL, action, VCP_IO.SUB_LEFT)
            send_ipc_signal(VCP_IO.TURN_SIGNAL, action, VCP_IO.SUB_RIGHT)
        elif left:
            send_ipc_signal(VCP_IO.TURN_SIGNAL, action, VCP_IO.SUB_LEFT)
            send_ipc_signal(VCP_IO.TURN_SIGNAL, VCP_IO.ACTION_OFF, VCP_IO.SUB_RIGHT)
        elif right:
            send_ipc_signal(VCP_IO.TURN_SIGNAL, VCP_IO.ACTION_OFF, VCP_IO.SUB_LEFT)
            send_ipc_signal(VCP_IO.TURN_SIGNAL, action, VCP_IO.SUB_RIGHT)
        else:
            send_ipc_signal(VCP_IO.TURN_SIGNAL, VCP_IO.ACTION_OFF, VCP_IO.SUB_LEFT)
            send_ipc_signal(VCP_IO.TURN_SIGNAL, VCP_IO.ACTION_OFF, VCP_IO.SUB_RIGHT)

        time.sleep(0.1)


# =========================================================
# 장애물 회피 제어
# =========================================================

def emergency_control_logic():
    """장애물 감지 시 회피"""
    print("[Emergency] Avoidance logic started.")

    while not _stop:
        with _lock:
            l_has, l_close = _zone_state["L"]
            f_has, f_close = _zone_state["F"]
            r_has, r_close = _zone_state["R"]
            lane_num = _can.get("lane_num", 2)

        target_angle = STEER_CONFIG["CENTER"]

        # 고립 상황 판단
        if f_has and l_has and r_has or (lane_num == 1 and f_has and r_has) or (lane_num == 3 and f_has and l_has):
            with _lock:
                _can["is_accelerating"] = False
                _can["is_braking"] = True
                _can["avoid_mode"] = True

        # 회피 판단
        elif f_has or l_has or r_has:
            is_emergency = f_close or l_close or r_close

            if lane_num == 1:
                target_angle = (OBSTACLE_CONFIG["HARD_RIGHT"] if is_emergency 
                               else OBSTACLE_CONFIG["SOFT_RIGHT"])
            elif lane_num == 3:
                target_angle = (OBSTACLE_CONFIG["HARD_LEFT"] if is_emergency 
                               else OBSTACLE_CONFIG["SOFT_LEFT"])
            else:
                target_angle = (OBSTACLE_CONFIG["HARD_RIGHT" if l_has else "HARD_LEFT"] if is_emergency
                               else OBSTACLE_CONFIG["SOFT_RIGHT" if l_has else "SOFT_LEFT"])

            with _lock:
                _can["avoid_mode"] = True
                _can["target_steer"] = target_angle
                _can["is_steering"] = True
                _can["left_blinker"] = target_angle < STEER_CONFIG["CENTER"]
                _can["right_blinker"] = target_angle > STEER_CONFIG["CENTER"]

        else:
            with _lock:
                _can["avoid_mode"] = False
                _can["left_blinker"] = False
                _can["right_blinker"] = False

        time.sleep(0.1)


# =========================================================
# 바퀴 자동 정렬
# =========================================================

def auto_align_worker():
    """회피 종료 후 바퀴 자동 정렬"""
    print("[Align] Auto-align worker started.")

    was_avoiding = False
    CENTER = STEER_CONFIG["CENTER"]
    TOLERANCE = 5

    while not _stop:
        with _lock:
            is_avoid = _can.get("avoid_mode", False)

        if was_avoiding and not is_avoid:
            print("[Align] Steering to center...")

            while True:
                with _lock:
                    current_steer = _can.get("steer", CENTER)

                    if abs(current_steer - CENTER) <= TOLERANCE:
                        _can["is_steering"] = False
                        _can["steer"] = CENTER
                        break

                    _can["is_steering"] = True
                    _can["is_steer_reverse"] = current_steer < CENTER

                time.sleep(0.05)

            print("[Align] Alignment complete.")

        was_avoiding = is_avoid
        time.sleep(0.05)


# =========================================================
# AI 데이터 수신
# =========================================================

def ai_data_worker():
    """AI 서버에서 장애물 데이터 수신"""
    analyzer = ObjectAnalytics()
    print(f"[AI] Worker started. Target: {AI_CONFIG['IP']}:{AI_CONFIG['PORT']}")

    while not _stop:
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(AI_CONFIG["TIMEOUT"])
            print("[AI] Connecting...")

            sock.connect((AI_CONFIG["IP"], AI_CONFIG["PORT"]))
            print("[AI] Connected!")

            sock.settimeout(None)
            f = sock.makefile("r", encoding="utf-8", newline="\n")

            while not _stop:
                readable, _, _ = select.select([sock], [], [], AI_CONFIG["SELECT_TIMEOUT"])
                data = {}

                if sock in readable:
                    try:
                        line = f.readline()
                        if not line:
                            print("[AI] Disconnected.")
                            break

                        raw_str = line.strip()
                        if raw_str:
                            data = json.loads(raw_str)
                    except (json.JSONDecodeError, Exception) as e:
                        pass

                res_L, res_F, res_R = analyzer.update_status(data)

                with _lock:
                    _zone_state["L"] = res_L
                    _zone_state["F"] = res_F
                    _zone_state["R"] = res_R

        except Exception as e:
            print(f"[AI] Error: {e}")
            time.sleep(2)
        finally:
            if sock:
                try:
                    sock.close()
                except:
                    pass


# =========================================================
# 차선 인식 워커
# =========================================================

def lane_worker(cam_index: int = None, show: bool = True):
    """카메라에서 차선 감지"""
    if cam_index is None:
        cam_index = CAMERA_CONFIG["INDEX"]

    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_CONFIG["WIDTH"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_CONFIG["HEIGHT"])
    cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_CONFIG["BUFFER_SIZE"])

    visualizer = LaneVisualizer()

    while not _stop:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        vis, steer_norm, lane_num = visualizer.process_frame(frame)

        # 차선 정보 업데이트
        if not _can.get("avoid_mode", False):
            with _lock:
                target = int(np.clip(STEER_CONFIG["CENTER"] + steer_norm * 
                            (STEER_CONFIG["MAX"] - STEER_CONFIG["CENTER"]), 
                            STEER_CONFIG["MIN"], STEER_CONFIG["MAX"]))
                _can["target_steer"] = target
                if lane_num != 0:
                    _can["lane_num"] = lane_num

        if show:
            cv2.imshow("Lane Detection", vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    try:
        cv2.destroyAllWindows()
    except:
        pass


# =========================================================
# 콘솔 명령 입력
# =========================================================

def console_input_worker():
    """터미널 명령 입력 (GO/STOP)"""
    print("\n[CMD] Type 'GO' to start, 'STOP' to brake.\n")

    while not _stop:
        try:
            cmd = input().strip().upper()

            if cmd == "GO":
                print(">>> Accelerating...")
                with _lock:
                    _can["is_braking"] = False
                    _can["avoid_mode"] = False
                    _can["steer"] = STEER_CONFIG["CENTER"]
                    _can["target_steer"] = STEER_CONFIG["CENTER"]
                    _can["is_steering"] = False
                    _can["is_accelerating"] = True
                    _can["is_resverse"] = False
                    if _can["speed_kmh"] == 0:
                        _can["speed_kmh"] = 5

            elif cmd == "STOP":
                print(">>> Braking...")
                with _lock:
                    _can["is_accelerating"] = False
                    _can["is_braking"] = True

        except EOFError:
            break
        except Exception as e:
            print(f"[CMD Error] {e}")


# =========================================================
# Main
# =========================================================

def main():
    """시스템 시작"""
    global _stop

    print("=" * 50)
    print("자율주행 제어 시스템 시작")
    print("=" * 50)

    threads = [
        ("AI_Worker", ai_data_worker),
        ("Speed_Controller", speed_controller),
        ("Lane_Worker", lane_worker),
        ("Avoid_Logic", emergency_control_logic),
        ("Emergency_Worker", led_worker),
        ("Wheel_Controller", wheel_controller),
        ("Auto_Align", auto_align_worker),
        ("Console_Input", console_input_worker),
    ]

    thread_list = []
    for name, func in threads:
        t = threading.Thread(target=func, daemon=True, name=name)
        t.start()
        thread_list.append(t)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n시스템 종료 중...")
    finally:
        _stop = True
        for t in thread_list:
            try:
                t.join(timeout=1.0)
            except:
                pass

        print("종료 완료.")
        if sndfile:
            sndfile.close()


if __name__ == "__main__":
    main()
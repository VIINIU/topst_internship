import json, time, socket, threading, http.client
import serial
import select  
from urllib.parse import urlparse
from typing import Any, Dict
import argparse
import cv2
import numpy as np
from Library.IPC_Library import IPC_SendPacketWithIPCHeader, IPC_ReceivePacketFromIPCHeader
from Library.IPC_Library import TCC_IPC_CMD_CA72_EDUCATION_CAN_DEMO, IPC_IPC_CMD_CA72_EDUCATION_CAN_DEMO_START
from Library.IPC_Library import parse_hex_data, parse_string_data, parse_channels, parse_hex16

# VCP IO 정의
class VCP_IO:
    # IO 타입
    BREAK_LIGHT = 0x101 # 브레이크 led -> 브레이크용 서보모터로
    TURN_SIGNAL = 0x102 #비상등 led (급정지) / 좌, 우회전
    EMER_SIGNAL = 0x103 
    HEAD_LIGHT = 0x104
    FUEL_L = 0x105
    MOTOR_A = 0x106
    WHEEL = 0x107
    # 액션
    ACTION_ON = 0x01
    ACTION_OFF = 0x02
    
    # 서브타입 (턴 시그널용)
    SUB_LEFT = 0x01
    SUB_RIGHT = 0x02

# =========================================================
# 🔽 [유빈] AI-G 수신 및 판단용 전역 변수 선언, 클래스, 함수 선언 🔽 
# =========================================================
AI_G_IP = "192.168.0.100"
AI_PORT = 9999

img_width = 800
img_height = 480

# 구역별 상태 저장 (L, F, R)
_zone_state = {
    "L": (False, False),
    "F": (False, False),
    "R": (False, False)
}


_ipc_cache = {}

# [Class] 구역 상태 및 분석 클래스
class ClearCheck:
    def __init__(self):
        self.has_obstacle = False
        self.is_close = False
        self.last_update_time = 0.0

class ObjectAnalytics:
    def __init__(self, width=800, height=480):
        self.w = width
        self.h = height
        self.total_area = width * height
        self.close_area_ratio = 0.12
        
        # [수정] 장애물 미감지 시 플래그 해제 대기 시간 (0.5초로 변경)
        self.timeout = 0.5  
        
        self.zones = {'L': ClearCheck(), 'F': ClearCheck(), 'R': ClearCheck()}
        
        # [이전 요청 유지] 프레임 카운트 및 이전 결과 저장 변수
        self.frame_count = 0
        self.last_result = (
            (False, False),
            (False, False),
            (False, False)
        )

    def update_status(self, json_data: dict):
        """
        [유지] 5프레임마다 한 번씩만 연산 수행 (나머지는 이전 상태 리턴)
        [수정] 타임아웃 1초 적용
        """
        self.frame_count += 1

        # 5번째 프레임이 아니면 이전에 저장된 결과를 바로 반환 (연산 건너뜀)
        if self.frame_count % 3 != 0:
            return self.last_result

        # === 아래부터는 5프레임마다 실행되는 실제 로직 ===
        current_time = time.time()

        if json_data and 'boxes' in json_data and isinstance(json_data['boxes'], list):
            for det in json_data['boxes']:
                if not isinstance(det, dict): continue

                # 좌표 추출
                xmin = det.get('xmin', 0)
                xmax = det.get('xmax', 0)
                ymin = det.get('ymin', 0)
                ymax = det.get('ymax', 0)
                
                # 크기 및 중심점 계산
                obj_w = xmax - xmin
                obj_h = ymax - ymin
                obj_area = obj_w * obj_h
                center_x = xmin + (obj_w / 2.0) 

                # 가깝냐?
                is_close = (obj_area / self.total_area) > self.close_area_ratio

                # 화면 3분할 기준선
                div_1 = self.w / 2.8      
                div_2 = self.w * (1.8 / 2.8) 

                # 중심점이 위치한 구역 판단
                target_zone = ''
                if center_x < div_1:
                    target_zone = 'L'
                elif center_x > div_2:
                    target_zone = 'R'
                else:
                    target_zone = 'F'

                # 상태 업데이트
                self.zones[target_zone].has_obstacle = True
                if is_close:
                    self.zones[target_zone].is_close = True
                # 장애물이 감지되면 마지막 감지 시간을 현재 시간으로 갱신
                self.zones[target_zone].last_update_time = current_time

        # [타임아웃 처리] 
        # 마지막 감지 시간으로부터 1.0초(self.timeout)가 지날 때까지는 False로 바꾸지 않음
        for key in self.zones:
            time_diff = current_time - self.zones[key].last_update_time
            if time_diff > self.timeout:
                if self.zones[key].has_obstacle: # 꺼지는 순간에만 로그 출력
                    print(f"[AI-Timeout] Zone {key} cleared. (Last update: {time_diff:.1f}s ago)")
                
                self.zones[key].has_obstacle = False
                self.zones[key].is_close = False

        # 결과 저장
        self.last_result = (
            (self.zones['L'].has_obstacle, self.zones['L'].is_close),
            (self.zones['F'].has_obstacle, self.zones['F'].is_close),
            (self.zones['R'].has_obstacle, self.zones['R'].is_close)
        )
        
        # [디버깅 5] 최종 결과 출력 (상태가 True인 경우만 강조해서 보거나 매번 출력)
        # if any(z[0] for z in self.last_result): 
        print(f"[AI-Result] L:{self.last_result[0]} F:{self.last_result[1]} R:{self.last_result[2]}")

        return self.last_result

# =========================================================
# 🔼 [유빈] AI-G 수신 및 판단용 전역 변수 선언, 클래스, 함수 선언 끝 🔼
# =========================================================
# =========================================================
# 주행 관련 전역 변수
# =========================================================

sndfile = open("/dev/tcc_ipc_micom", 'wb')

MAX_SPEED = 35        # 최대 속도
SPEED_INCREMENT = 10     # F 누를 때마다 증가량
SPEED_DECREMENT = 10     # B 누를 때마다 감소량
ACCEL_INTERVAL = 0.5    # 가속 업데이트 주기 (초)

QT_MAX_SPEED = 100        # 최대 속도
QT_SPEED_INCREMENT = 1     # F 누를 때마다 증가량
QT_SPEED_DECREMENT = 1     # B 누를 때마다 감소량
QT_ACCEL_INTERVAL = 5   # 가속 업데이트 주기 (초)

STEER_CENTER = 65
STEER_MIN = 0
STEER_MAX = 127       
STEER_STEP = 10

BLINK_INTERVAL = 0.5  # 깜빡이 간격 (0.5초)

# lane detect, wheel 제어 변수
WHEEL_HZ = 10
CAM_INDEX = 1
FRAME_W = 640
FRAME_H = 480

TASK_HZ = {
    "break":     10,  
    "head":      10,   
    "turn":      10,      
    "speed":     10,   
    "fuel":      60, 
    "wheel":     10,
}
# _latest = {"rpm":0.0,"speed_kmh":0.0,"fuel_l":0.0,"steering":0.0}

_can = {"park": False, 
        "left_blinker": False, 
        "right_blinker": False, 
        "emergency": False,
        "emergency_toggle": False, 
        "headlights": False, 
        "fuel_l": 0,
        "speed_kmh":0, 
        "steer":65, 
        "is_accelerating": False, 
        "is_braking": False,
        "is_resverse": False,
        "is_steering": False,
        "is_steer_reverse": False,
        "avoid_mode": False,
        "target_steer" : 65,
        "lane_num" : 2 #현재 주행 차선 정보(기본값 2차선)
        } #[소연] 장애물 회피 플래그  

_lock   = threading.Lock()
_stop   = False
 
# =========================================================
# 주행 관련 함수
# =========================================================

def send_ipc_signal(io_type, action_or_value, subtype=None):
    """IPC 신호 전송 (값 변경 시에만 전송하여 부하 방지)"""
    global _ipc_cache
    
    if sndfile is None: return False
    
    # 캐시 키 생성 (IO타입 + 서브타입)
    cache_key = (io_type, subtype)
    
    # 이전 값과 같으면 전송 스킵 (부하 감소 핵심)
    if _ipc_cache.get(cache_key) == action_or_value:
        return True

    try:
        if subtype is not None:
            payload = bytes([subtype, action_or_value])
        else:
            payload = (action_or_value).to_bytes(1,'big',signed=True)
        
        IPC_SendPacketWithIPCHeader(sndfile, 1, 0, io_type, payload)
        
        # 전송 성공 시 캐시 업데이트
        _ipc_cache[cache_key] = action_or_value
        return True
    except Exception as e:
        # 에러 발생 시 캐시 초기화 (재시도 허용)
        if cache_key in _ipc_cache:
            del _ipc_cache[cache_key]
        print(f"[IPC ERROR] {e}")
        return False

def get_blink_state():
    """현재 시간 기준으로 깜빡이가 켜져있어야 하는지 판단"""
    current_time = time.time()
    cycle_position = current_time % (BLINK_INTERVAL * 2)  # 1초 주기 (0.5초 ON, 0.5초 OFF)
    return cycle_position < BLINK_INTERVAL

def wheel_action(cur):
    cur = max(-127, min(127, cur))
    wheel = int((cur + 127) * (255 / 254))  # 0~255 매핑
    send_ipc_signal(VCP_IO.WHEEL, wheel)

def speed_controller():

    while not _stop:
        with _lock:
            is_accel = _can.get("is_accelerating", False)
            is_resverse = _can.get("is_resverse", False)
            current_speed = _can.get("speed_kmh", 0)
            is_brake = _can.get("is_braking", False)
            steer = _can.get("steer", STEER_CENTER)
            is_avoid = _can.get("avoid_mode", False) # 회피 모드 여부 확인
        
        # new_speed 초기화
        new_speed = current_speed

        # [안전 로직 수정]
        # 1. 장애물 회피 모드(is_avoid) 중이면? -> AI가 제어 중이므로 브레이크 간섭 금지 (Pass)
        # 2. 일반 모드인데 핸들이 위험하게 꺾여있으면? -> 전복 방지 브레이크 (Brake)
        
        if is_avoid:
            # AI가 회피 중일 때는 핸들을 0, 10, 127로 꺾어도 브레이크 밟지 않음
            pass
        else:
            # 회피 모드가 아닐 때 (수동 or 직진) 안전 범위 체크
            if 50 < steer < 80: 
                pass # 안전
            else:
                # 위험 각도 -> 브레이크
                if not is_brake: 
                    print(f"[WARN] Unsafe Steering Detected! Angle: {steer} -> Braking!")
                is_brake = True

        # 가속/감속 계산
        if is_accel and not is_brake:
            send_ipc_signal(VCP_IO.BREAK_LIGHT, VCP_IO.ACTION_OFF) # 브레이크용 서보모터 원위치(가속)
            step = SPEED_INCREMENT
            if not is_resverse:
                new_speed = min(current_speed + step, MAX_SPEED)
            else:
                new_speed = max(current_speed - step, -MAX_SPEED) 
            
            with _lock:
                _can["speed_kmh"] = new_speed

        elif is_brake:
            send_ipc_signal(VCP_IO.BREAK_LIGHT, VCP_IO.ACTION_ON) # 브레이크용 서보모터 동작(감속)
            if current_speed > 0 and not is_avoid: 
                new_speed = max(current_speed - SPEED_DECREMENT, 0)
            elif current_speed < 0: 
                new_speed = min(current_speed + SPEED_DECREMENT, 0)
            else:
                new_speed = 0

            with _lock:
                _can["speed_kmh"] = new_speed
        
        else: #가속도 감속도 아닌 상황 -> 브레이크 서보모터 원위치
            send_ipc_signal(VCP_IO.BREAK_LIGHT, VCP_IO.ACTION_OFF)

        # 최종 모터 신호 관련
        send_ipc_signal(VCP_IO.MOTOR_A, new_speed)
        time.sleep(ACCEL_INTERVAL)


# =========================
# P-Control 제어 함수 (부드럽고 빠름 - 추천)
# =========================
def wheel_controller():
    dt = 1.0 / float(WHEEL_HZ) # 주기 (0.1초)
    Kp = 0.5                   # P-Control 계수
    current_steer_f = float(STEER_CENTER)

    while not _stop:
        with _lock:
            is_avoid = _can.get("avoid_mode", False)
            target_steer_avoid = float(_can.get("target_steer", STEER_CENTER)) # 회피용 목표
            hw_steer = _can.get("steer", STEER_CENTER)

        final_out_val = hw_steer

        if is_avoid:
            current_steer_f = float(hw_steer) 
            
            if abs(hw_steer - target_steer_avoid) > STEER_STEP:
                if hw_steer < target_steer_avoid:
                    final_out_val = min(STEER_MAX, hw_steer + STEER_STEP)
                else:
                    final_out_val = max(STEER_MIN, hw_steer - STEER_STEP)
            else:
                final_out_val = int(target_steer_avoid)

        else:
            
            target = target_steer_avoid 
            error = target - current_steer_f
            control = error * Kp
            
            current_steer_f += control
            final_out_val = int(current_steer_f)
            final_out_val = max(STEER_MIN, min(STEER_MAX, final_out_val))

        with _lock:
            _can["steer"] = final_out_val

        send_ipc_signal(VCP_IO.WHEEL, final_out_val)
        
        # 주기 대기
        time.sleep(dt)

# =========================================================
# 🔽 [소연] LED 제어 코드 수정🔽 
# =========================================================

def emergency_worker():
    """비상등 및 좌/우 깜빡이 통합 제어"""
    while not _stop:
        with _lock:
            emer = _can.get("emergency", False)
            left = _can.get("left_blinker", False)
            right = _can.get("right_blinker", False)
        
        on = get_blink_state()
        action = VCP_IO.ACTION_ON if on else VCP_IO.ACTION_OFF

        # 1순위: 비상등 (둘 다 깜빡임)
        if emer:
            # 좌우 모두 같은 상태로 깜빡이기
            send_ipc_signal(VCP_IO.TURN_SIGNAL, action, VCP_IO.SUB_LEFT)
            send_ipc_signal(VCP_IO.TURN_SIGNAL, action, VCP_IO.SUB_RIGHT)

        # 2순위: 왼쪽 깜빡이만
        elif left:
            send_ipc_signal(VCP_IO.TURN_SIGNAL, action, VCP_IO.SUB_LEFT)
            send_ipc_signal(VCP_IO.TURN_SIGNAL, VCP_IO.ACTION_OFF, VCP_IO.SUB_RIGHT)

        # 3순위: 오른쪽 깜빡이만
        elif right:
            send_ipc_signal(VCP_IO.TURN_SIGNAL, VCP_IO.ACTION_OFF, VCP_IO.SUB_LEFT)
            send_ipc_signal(VCP_IO.TURN_SIGNAL, action, VCP_IO.SUB_RIGHT)

        # 4순위: 모두 꺼짐
        else:
            # 비상등이 꺼진 상태인데, 혹시 켜져있으면 한 번만 OFF 보내기
            send_ipc_signal(VCP_IO.TURN_SIGNAL, VCP_IO.ACTION_OFF, VCP_IO.SUB_LEFT)
            send_ipc_signal(VCP_IO.TURN_SIGNAL, VCP_IO.ACTION_OFF, VCP_IO.SUB_RIGHT)

        time.sleep(0.1)  # 반응 속도를 위해 0.5에서 0.1로 단축 추천

# =========================================================
# 🔼 [소연] LED 제어 코드 수정 완료 🔼
# =========================================================

# =========================================================
# 🔽 [소연] 장애물 회피 관련 코드 추가🔽 
# =========================================================
def emergency_control_logic():
    """
    장애물 위치에 따라 회피할 '목표 조향각(target_steer)'을 설정
    장애물이 사라지면 바퀴를 정렬한 후 제어권 해제
    """
    print("[Emergency Logic] Started.")

    # 하드웨어 세팅
    HARD_LEFT = 10 
    HARD_RIGHT = 120   
    SOFT_LEFT = 40  
    SOFT_RIGHT = 90  
    CENTER = 65

    while not _stop:
        with _lock:
            l_has, l_close = _zone_state["L"]
            f_has, f_close = _zone_state["F"]
            r_has, r_close = _zone_state["R"]
            lane_num = _can.get("lane_num", 2) 
            current_steer = _can.get("steer", CENTER)

        # 제어 변수 초기화
        target_angle = CENTER
        avoid_active = False
        do_emergency_stop = False
        
        # ----------------------------------------
        # 1. 정지 판단 (차선에 따른 물리적 고립 체크)
        # ----------------------------------------
        if f_has and l_has and r_has:
            do_emergency_stop = True
        elif lane_num == 1 and f_has and r_has: # 1차선 고립 상황
            do_emergency_stop = True
        elif lane_num == 3 and f_has and l_has: # 3차선 고립 상황
            do_emergency_stop = True

        # ----------------------------------------
        # 2. 회피 방향 및 강도 판단 (차선 제약 + 거리 반영)
        # ----------------------------------------
        elif not f_has:
            avoid_active = False
        
        elif f_has or l_has or r_has: # 장애물 감지 시
            avoid_active = True
            
            # [긴급도 체크] 하나라도 근접해 있으면 HARD, 아니면 SOFT
            is_emergency = (f_close or l_close or r_close)

            if lane_num == 1:
                # 1차선: 왼쪽 불가 -> 무조건 우측 회피
                target_angle = HARD_RIGHT if is_emergency else SOFT_RIGHT
                
            elif lane_num == 3:
                # 3차선: 오른쪽 불가 -> 무조건 좌측 회피
                target_angle = HARD_LEFT if is_emergency else SOFT_LEFT
                
            else:
                # 2차선: 기본 왼쪽 회피 (왼쪽에 장애물 l_has가 없을 때만)
                if l_has: 
                    # 왼쪽에 뭐가 있으면 오른쪽으로 회피
                    target_angle = HARD_RIGHT if is_emergency else SOFT_RIGHT
                else:
                    # 왼쪽이 비어있으면 기본 구조대로 왼쪽으로 회피
                    target_angle = HARD_LEFT if is_emergency else SOFT_LEFT
        # ----------------------------------------
        # 상태 업데이트 (제어 명령)
        # ----------------------------------------
        with _lock:
            if do_emergency_stop:
                _can["is_accelerating"] = False
                _can["is_braking"] = True
                _can["avoid_mode"] = True
                _can["emergency"] = True
            
            elif avoid_active:
                # [회피 중]
                _can["avoid_mode"] = True
                _can["target_steer"] = target_angle
                _can["is_steering"] = True 
                _can["emergency"] = False
                
                # 깜빡이
                if target_angle < CENTER: # 왼쪽 
                    _can["left_blinker"] = True
                    _can["right_blinker"] = False
                elif target_angle > CENTER: # 오른쪽
                    _can["left_blinker"] = False
                    _can["right_blinker"] = True
            
            else:
                # [장애물 없음] -> 바퀴 정렬 로직 추가
                _can["avoid_mode"] = False 
                _can["emergency"] = False
                _can["left_blinker"] = False
                _can["right_blinker"] = False
                # target_steer를 여기서 건드리지 않아, 평시 주행 로직과 충돌 방지

        time.sleep(0.1)
# =========================================================
# 🔼 [소연] 장애물 회피 관련 코드 끝 🔼
# =========================================================

# =========================================================
# [Thread] AI 데이터 수신 워커
# =========================================================
def ai_data_worker():
    analyzer = ObjectAnalytics()
    print(f"[AI] Worker Started. Target: {AI_G_IP}:{AI_PORT}")

    while not _stop:
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0) # 연결 시도 타임아웃
            print(f"[AI] Connecting...")
            
            try:
                sock.connect((AI_G_IP, AI_PORT))
                print(f"[AI] Connected! Waiting for data...")
            except socket.error:
                print(f"[AI] Connection failed. Retrying...")
                time.sleep(2)
                continue
            
            # 연결 후에는 타임아웃을 None(Blocking)으로 두거나 select로 제어합니다.
            sock.settimeout(None)
            
            f = sock.makefile("r", encoding="utf-8", newline="\n")

            while not _stop:
                # [핵심 수정] select를 사용하여 0.2초 동안 데이터가 있는지 '검사'만 함
                # readable에 sock이 들어있으면 데이터가 온 것, 없으면 타임아웃(데이터 없음)
                readable, _, _ = select.select([sock], [], [], 0.2)

                data = {}

                if sock in readable:
                    # 1. 데이터가 도착함 -> 읽기 시도
                    try:
                        line = f.readline()
                        if not line: # 빈 문자열이면 서버가 연결을 끊은 것
                            print("[AI] Server Disconnected (EOF).")
                            break
                        
                        raw_str = line.strip()
                        if raw_str:
                            # print(f"[RAW] {raw_str}")
                            try:
                                data = json.loads(raw_str)
                            except json.JSONDecodeError:
                                print(f"[AI] JSON Error: {raw_str}")
                                
                    except Exception as e:
                        print(f"[AI] Read Error: {e}")
                        break
                else:
                    # 2. 0.2초 동안 데이터 안 옴 (Timeout) -> 장애물 소멸 로직 수행
                    # 아무것도 하지 않고 pass하면 아래 update_status({}) 가 실행됨
                    pass

                # 3. 상태 업데이트
                # 데이터가 있으면(data 채워짐) -> 장애물 갱신
                # 데이터가 없으면(data 빈 딕셔너리) -> 내부 타이머가 흘러가며 장애물 해제
                res_L, res_F, res_R = analyzer.update_status(data)
                
                # 4. 전역 변수 공유
                with _lock:
                    _zone_state["L"] = res_L
                    _zone_state["F"] = res_F
                    _zone_state["R"] = res_R

        except Exception as e:
            print(f"[AI] Connection Error: {e}")
            time.sleep(2)
        finally:
            if sock: 
                try: sock.close()
                except: pass

# =========================================================
# 2. [추가] 바퀴 자동 정렬 워커 (Console Worker 역할 보조)
# =========================================================
def auto_align_worker():
    """
    장애물 회피(avoid_mode)가 끝났을 때, 
    직접 IPC를 쏘지 않고 'is_steering'과 'is_steer_reverse' 플래그를 조작하여
    wheel_controller가 바퀴를 중앙(65)으로 정렬하도록 유도함.
    """
    print("[Align Worker] Started monitoring avoidance state.")
    
    was_avoiding = False
    CENTER = 65
    TOLERANCE = 5 # 중앙으로 인정할 오차 범위

    while not _stop:
        with _lock:
            is_avoid = _can.get("avoid_mode", False)
        
        # [감지] 회피 모드: True -> False (상황 종료)
        if was_avoiding and not is_avoid:
            print("[Align Worker] Avoidance finished. Steering to Center...")

            # 바퀴가 중앙 근처에 올 때까지 루프를 돌며 플래그를 잡음
            while True:
                with _lock:
                    current_steer = _can.get("steer", CENTER)
                    
                    # 1. 종료 조건: 이미 중앙(오차범위 내)이면 정렬 중단
                    if abs(current_steer - CENTER) <= TOLERANCE:
                        _can["is_steering"] = False
                        # 확실하게 65로 마무리
                        _can["steer"] = CENTER 
                        break

                    # 2. 방향 결정 및 조향 활성화
                    _can["is_steering"] = True
                    
                    if current_steer < CENTER:
                        # 현재가 65보다 작으면(오른쪽?) -> 왼쪽(Reverse)으로 돌려야 커짐
                        _can["is_steer_reverse"] = True 
                    else:
                        # 현재가 65보다 크면(왼쪽?) -> 오른쪽(Normal)으로 돌려야 작아짐
                        _can["is_steer_reverse"] = False
                
                # wheel_controller가 움직일 시간을 줌 (매우 중요)
                time.sleep(0.05) 
            
            print("[Align Worker] Alignment Complete.")

        was_avoiding = is_avoid
        time.sleep(0.05)

# =========================================================
# Lane detection 업데이트
# =========================================================
LANE_ON_TH  = 0.12
LANE_OFF_TH = 0.08

def update_steer_flags_from_lane(steer_norm: float, lane_num: int, on_th: float = LANE_ON_TH, off_th: float = LANE_OFF_TH) -> None:
    """LaneVisualizer가 내는 steer_norm(-1~+1)을 이용해 업데이트"""
    if steer_norm is None:
        steer_norm = 0.0
    steer_norm = float(np.clip(steer_norm, -1.0, 1.0))
    target = int(np.clip(STEER_CENTER + steer_norm * (STEER_MAX - STEER_CENTER), STEER_MIN, STEER_MAX))

    with _lock:
        is_emergency = _can.get("avoid_mode", False)

    if not is_emergency:
        with _lock:
            _can["target_steer"] = target
            if lane_num != 0 :
                _can["lane_num"] = lane_num

            prev_on = bool(_can.get("is_steering", False))
            mag = abs(steer_norm)

            if prev_on:
                is_on = mag >= off_th
            else:
                is_on = mag >= on_th
        
            _can["is_steering"] = is_on
            _can["is_steer_reverse"] = True if steer_norm > 0 else False

# =========================================================
# 🔧 LaneVisualizer - 전체 구간 차선 인식 적용
# =========================================================
class LaneVisualizer:
    def __init__(self):
        self.CAR_CENTER_X = FRAME_W // 2
        
        # ✅ 수정 1: ROI를 전체 화면으로 확장 (원래: 0.35)
        self.ROI_TOP_RATIO = 0.3  # 0% = 맨 위부터 시
        
        self.GAMMA = 0.6

        self.Y_EVAL = FRAME_H - 10
        
        # ✅ 수정 2: 최소 Y값을 매우 낮춤 (원래: 360)
        self.MIN_Y_MAX = int(FRAME_H * 0.3)  # 144 (더 높은 위치 차선도 감지)

        self.lane_width_est = 350.0
        self.W_ALPHA = 0.12
        self.W_MIN, self.W_MAX = 160.0, 700.0

        self.STEER_YS = [350, 400, 450]
        self.STEER_W  = [0.4, 0.3, 0.3]

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
        dpoly = np.polyder(poly_func)
        slope = float(dpoly(y_ref))
        angle_norm = slope / (img_width * 0.5)
        return float(np.clip(angle_norm, -1.0, 1.0))

    def process_frame(self, frame):
        h, w = frame.shape[:2]

        darker = self.adjust_gamma(frame, gamma=self.GAMMA)
        blur = cv2.GaussianBlur(darker, (5, 5), 0)

        yellow_mask, white_mask = self.get_color_masks(blur)
        
        # ✅ 수정: apply_roi_full 사용 (전체 구간)
        roi_yellow = self.apply_roi_full(yellow_mask)
        roi_white  = self.apply_roi_full(white_mask)

        candidates = []
        candidates += self.find_lane_candidates(roi_yellow, "Yellow")
        candidates += self.find_lane_candidates(roi_white,  "White")

        left_candidates, right_candidates = [], []
        for c in candidates:
            if c['x_eval'] < self.CAR_CENTER_X:
                left_candidates.append(c)
            else:
                right_candidates.append(c)

        def score(c):
            dist = abs(c['x_eval'] - self.CAR_CENTER_X)
            return (c['y_max'] * 10.0) - dist

        left_lane  = max(left_candidates,  key=score) if left_candidates  else None
        right_lane = max(right_candidates, key=score) if right_candidates else None

        detected_lane = 0
        if left_lane and right_lane:
            l_cls = left_lane['color']
            r_cls = right_lane['color']
            
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

        result_img = frame.copy()
        draw_ys = np.arange(int(h * self.ROI_TOP_RATIO), h)

        if self.DEBUG:
            cv2.line(result_img, (self.CAR_CENTER_X, 0), (self.CAR_CENTER_X, h),
                    (255, 0, 255), 2)

        final_left_poly = None
        final_right_poly = None

        if left_lane:
            final_left_poly = left_lane['poly']
            self.draw_poly(result_img, final_left_poly, draw_ys, (0,255,255), "L")

        if right_lane:
            final_right_poly = right_lane['poly']
            self.draw_poly(result_img, final_right_poly, draw_ys, (255,255,0), "R")

        center_poly = None
        lane_status = "Unknown"

        if final_left_poly is not None and final_right_poly is not None:
            ys = np.array(self.STEER_YS, dtype=np.float32)
            w_now = float(np.mean(final_right_poly(ys) - final_left_poly(ys)))
            if self.W_MIN < w_now < self.W_MAX:
                self.lane_width_est = (1 - self.W_ALPHA) * self.lane_width_est + self.W_ALPHA * w_now

            center_poly = lambda y: (final_left_poly(y) + final_right_poly(y)) / 2.0
            lane_status = "Both"

        elif final_left_poly is not None:
            center_poly = lambda y: final_left_poly(y) + (self.lane_width_est / 2.0)
            lane_status = "Left"

        elif final_right_poly is not None:
            center_poly = lambda y: final_right_poly(y) - (self.lane_width_est / 2.0)
            lane_status = "Right"

        steer = self.prev_steer

        if center_poly is not None:
            xs = []
            ws = []
            angle_terms = []

            for y, wgt in zip(self.STEER_YS, self.STEER_W):
                yy = int(np.clip(y, draw_ys[0], draw_ys[-1]))
                xx = float(center_poly(yy))
                xx = float(np.clip(xx, 0, w-1))
                xs.append(xx)
                ws.append(wgt)

                angle_terms.append(
                    self.calc_angle_term(
                        final_left_poly if final_left_poly is not None else final_right_poly,
                        y_ref=yy,
                        img_width=w
                    )
                )

            x_target = float(np.average(xs, weights=ws))
            error_px = x_target - self.CAR_CENTER_X
            offset_term = float(np.clip(error_px / (w / 4.0), -1.0, 1.0))

            angle_term = float(np.mean(angle_terms)) if angle_terms else 0.0

            OFFSET_W = 0.7
            ANGLE_W  = 0.3

            raw = (OFFSET_W * offset_term + ANGLE_W * angle_term)* 1.5
            raw = float(np.clip(raw, -1.0, 1.0))

            if abs(raw) < self.DEADZONE:
                raw = 0.0

            filt = (1 - self.ALPHA) * self.prev_steer + self.ALPHA * raw

            filt = float(np.clip(
                filt,
                self.prev_steer - self.MAX_DELTA,
                self.prev_steer + self.MAX_DELTA
            ))

            self.prev_steer = filt
            steer = filt

            if self.DEBUG:
                cv2.putText(result_img, f"Lane: {self.current_lane}", (20, 125),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                cv2.putText(
                    result_img,
                    f"offset={offset_term:+.2f} angle={angle_term:+.2f} steer={steer:+.2f}",
                    (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0,255,255),
                    2
                )

        cv2.putText(result_img,
                    f"Lane: {lane_status}",
                    (20, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255,255,255),
                    2)
        
        return result_img, steer, self.current_lane

    def find_lane_candidates(self, mask, color_label):
        """✅ 수정: 후보 필터링 완화"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # ✅ 최소 면적 낮춤 (200 → 100)
            if area < 100:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / max(h, 1)
            # ✅ aspect_ratio 제한 완화 (6.0 → 8.0)
            if aspect_ratio > 8.0:
                continue

            try:
                pts = cnt.reshape(-1, 2)
                x_data = pts[:, 0].astype(np.float32)
                y_data = pts[:, 1].astype(np.float32)

                y_max = float(y_data.max())
                if y_max < self.MIN_Y_MAX:
                    continue

                poly_coeffs = np.polyfit(y_data, x_data, 2)
                poly_func = np.poly1d(poly_coeffs)

                y_eval = min(self.Y_EVAL, y_max)
                x_eval = float(poly_func(y_eval))

                candidates.append({
                    'poly': poly_func,
                    'color': color_label,
                    'x_eval': x_eval,
                    'y_eval': y_eval,
                    'y_max': y_max
                })
            except:
                pass

        return candidates

    def draw_poly(self, img, poly_func, y_range, color, label):
        try:
            x_plot = poly_func(y_range).astype(int)
            x_plot = np.clip(x_plot, 0, img.shape[1]-1)
            pts = np.column_stack((x_plot, y_range))
            cv2.polylines(img, [pts], False, color, 4)
            cv2.putText(img, label, (int(x_plot[0]), int(y_range[0]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        except:
            pass

    def adjust_gamma(self, image, gamma=1.0):
        inv = 1.0 / max(gamma, 1e-6)
        table = np.array([((i / 255.0) ** inv) * 255 for i in range(256)], dtype=np.uint8)
        return cv2.LUT(image, table)

    def get_color_masks(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_white = np.array([0, 0, 210])
        upper_white = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((3, 3), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        white_mask  = cv2.morphologyEx(white_mask,  cv2.MORPH_OPEN, kernel)

        return yellow_mask, white_mask

    def apply_roi(self, mask):
        """원래 ROI (아래쪽만)"""
        height, width = mask.shape
        roi = np.zeros_like(mask)
        top = int(height * self.ROI_TOP_RATIO)
        
        margin_top = int(width * 0.3)
        
        poly = np.array([
            [(0, height), (width, height), 
             (width - margin_top, top), (margin_top, top)]
        ], np.int32)
        
        cv2.fillPoly(roi, poly, 255)
        return cv2.bitwise_and(mask, roi)

    def apply_roi_full(self, mask):
        """✅ 수정: 전체 화면을 ROI로 사용"""
        height, width = mask.shape
        roi = np.zeros_like(mask)
        
        top = int(height * self.ROI_TOP_RATIO)  # 0.0이면 0 (맨 위)
        
        # 전체 너비 사용 (직사각형)
        poly = np.array([
            [(0, height), (width, height), 
             (width, top), (0, top)]
        ], np.int32)
        
        cv2.fillPoly(roi, poly, 255)
        return cv2.bitwise_and(mask, roi)


def lane_worker(cam_index: int = 1, show: bool = True):
    """카메라 차선 인식"""
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    visualizer = LaneVisualizer()

    while not _stop:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        vis, steer_norm, lane_num = visualizer.process_frame(frame)

        update_steer_flags_from_lane(steer_norm, lane_num)

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
# 3. [유지] 터미널 명령어 입력 워커 (수정 없음)
# =========================================================
def console_input_worker():
    """터미널에서 GO 입력을 기다리는 스레드 (플래그 제어만 담당)"""
    print("\n[CMD] Type 'GO' to start moving straight!")
    print("[CMD] Type 'STOP' to brake manually.\n")
    
    while not _stop:
        try:
            cmd = input().strip().upper()
            
            if cmd == "GO":
                print(">>> [COMMAND] GO RECEIVED! Accelerating to Cruise Speed...")
                with _lock:
                    # 1. 제어 플래그 설정 (직접 값 주입 X, 플래그만 ON)
                    _can["is_braking"] = False
                    _can["avoid_mode"] = False
                    _can["emergency"] = False
                    
                    # 2. 바퀴 정렬
                    _can["steer"] = 65
                    _can["target_steer"] = 65 # (비상용이지만 초기화 차원)
                    _can["is_steering"] = False
                    
                    # 3. 가속 시작 신호
                    _can["is_accelerating"] = True
                    _can["is_resverse"] = False
                    
                    # 초기 기동을 위해 0이면 살짝 쳐줌 (옵션)
                    if _can["speed_kmh"] == 0:
                        _can["speed_kmh"] = 5

            elif cmd == "STOP":
                print(">>> [COMMAND] STOP RECEIVED!")
                with _lock:
                    _can["is_accelerating"] = False
                    _can["is_braking"] = True
                    # 속도 0 설정은 speed_controller가 다음 루프에서 수행함
            
        except EOFError:
            break
        except Exception as e:
            print(f"[CMD Error] {e}")

def main():
    global _stop
    print("Starting System...")

    # AI 수신 스레드 시작
    t_ai = threading.Thread(target=ai_data_worker, daemon=True, name="AI_Worker")
    t_ai.start()
    
    # 속도 제어 스레드 
    t_speed = threading.Thread(target=speed_controller, daemon=True, name="speed_controller")
    t_speed.start()
    
    # 차선 인식 스레드
    t_lane = threading.Thread(target=lane_worker, daemon=True, name="lane_worker")
    t_lane.start()
    
    # 장애물 회피 제어 스레드
    t_avoid = threading.Thread(target=emergency_control_logic, daemon=True, name="Avoid_Logic")
    t_avoid.start()

    # 비상등 제어 스레드
    t_emer = threading.Thread(target=emergency_worker, daemon=True, name="emergency_controller")
    t_emer.start()

    # 휠 제어 스레드　（통합）
    t_wheel = threading.Thread(target=wheel_controller, daemon=True, name="can")
    t_wheel.start()

    # [NEW] 바퀴 자동 정렬 워커 추가
    t_align = threading.Thread(target=auto_align_worker, daemon=True, name="Auto_Align")
    t_align.start()

    # 콘솔 입력 워커
    t_input = threading.Thread(target=console_input_worker, daemon=True, name="Console_Input")
    t_input.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        _stop = True        
        try:
            t_lane.join(timeout=1.0)
            t_wheel.join(timeout=1.0)
            time.sleep(1)
        except NameError:
            pass
        print("Shutdown complete.")
        if sndfile: sndfile.close() 

if __name__ == "__main__":
    main()


import json, time, socket, threading, http.client
import serial
from urllib.parse import urlparse
from typing import Any, Dict
import argparse
from Library.IPC_Library import IPC_SendPacketWithIPCHeader, IPC_ReceivePacketFromIPCHeader
from Library.IPC_Library import TCC_IPC_CMD_CA72_EDUCATION_CAN_DEMO, IPC_IPC_CMD_CA72_EDUCATION_CAN_DEMO_START
from Library.IPC_Library import parse_hex_data, parse_string_data, parse_channels, parse_hex16

# VCP IO 정의
class VCP_IO:
    # IO 타입
    BREAK_LIGHT = 0x101
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
AI_G_IP = "192.168.0.101"
AI_PORT = 9999

img_width = 800
img_height = 480

# 구역별 상태 저장 (L, F, R)
_zone_state = {
    "L": (False, False),
    "F": (False, False),
    "R": (False, False)
}

# [Class] 구역 상태 및 분석 클래스
class ClearCheck:
    def __init__(self):
        self.has_obstacle = False
        self.is_close = False
        self.last_update_time = 0.0

class ObjectAnalytics:
    def __init__(self, width=img_width, height=img_height):
        self.w = width
        self.h = height
        self.total_area = width * height
        self.close_area_ratio = 0.15
        self.timeout = 0.5
        self.zones = {'L': ClearCheck(), 'F': ClearCheck(), 'R': ClearCheck()}

    def update_status(self, json_data: dict):
        """
        [수정됨] 박스의 '중심점'을 기준으로 L/F/R 중 하나만 판별
        """
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
                center_x = xmin + (obj_w / 2.0) # 박스의 가로 중심

                # 가깝냐?
                is_close = (obj_area / self.total_area) > self.close_area_ratio

                # 화면 3분할 기준선
                div_1 = self.w / 3.0       
                div_2 = self.w * (2.0 / 3.0) 

                # [로직 변경] 중심점이 위치한 구역 하나만 True로 설정
                # 박스가 커도 중심이 왼쪽에 있으면 왼쪽 장애물로만 판단함
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
                self.zones[target_zone].last_update_time = current_time

        # 타임아웃 처리
        for key in self.zones:
            if current_time - self.zones[key].last_update_time > self.timeout:
                self.zones[key].has_obstacle = False
                self.zones[key].is_close = False

        return (
            (self.zones['L'].has_obstacle, self.zones['L'].is_close),
            (self.zones['F'].has_obstacle, self.zones['F'].is_close),
            (self.zones['R'].has_obstacle, self.zones['R'].is_close)
        )

# =========================================================
# 🔼 [유빈] AI-G 수신 및 판단용 전역 변수 선언, 클래스, 함수 선언 끝 🔼
# =========================================================
# =========================================================
# 주행 관련 전역 변수
# =========================================================

sndfile = open("/dev/tcc_ipc_micom", 'wb')

MAX_SPEED = 100        # 최대 속도
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

TASK_HZ = {
    "break":     10,  
    "head":      10,   
    "turn":      10,      
    "speed":     10,   
    "fuel":      60, 
    "wheel":     10,
}
_latest = {"rpm":0.0,"speed_kmh":0.0,"fuel_l":0.0,"steering":0.0}

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
        "target_steer" : 65
        } #[소연] 장애물 회피 플래그  

_lock   = threading.Lock()
_stop   = False
 
# =========================================================
# 주행 관련 함수
# =========================================================

def send_ipc_signal(io_type, action_or_value, subtype=None):
    """IPC 신호 전송 통합 함수"""
    try:
        if subtype is not None:
            payload = bytes([subtype, action_or_value])
        else:
            payload = (action_or_value).to_bytes(1,'big',signed=True)
        
        IPC_SendPacketWithIPCHeader(sndfile, 1, 0, io_type, payload)
        return True
    except Exception as e:
        print(f"[IPC ERROR] {e} - io_type: 0x{io_type:X}, value: {action_or_value}")
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
    """F를 누르고 있으면 속도 증가, B를 누르고 있으면 속도 감소"""
    while not _stop:
        with _lock:
            is_accel = _can.get("is_accelerating", False)
            is_resverse = _can.get("is_resverse", False)
            current_speed = _can.get("speed_kmh", 0)
            is_brake = _can.get("is_braking",False)
            steer = _can.get("steer", STEER_CENTER)
        
        if steer < 80 and steer > 50: # 오버스티어, 전복 방지
            is_brake = True

        if is_accel and not is_brake:
            # 가속 / 뒷방향 가속
            if not is_resverse:
                new_speed = min(current_speed + SPEED_INCREMENT, MAX_SPEED)
                with _lock:
                    _can["speed_kmh"] = new_speed

            else:
                new_speed = min(current_speed - SPEED_INCREMENT, MAX_SPEED)
                with _lock:
                    _can["speed_kmh"] = new_speed

        elif is_brake:
            # 감속
            if current_speed > 0 : # 전진 상황
                new_speed = max(current_speed - SPEED_DECREMENT, 0) # 후진 방지
            
            else : # 후진 상황
                new_speed = min(current_speed + SPEED_DECREMENT, 0)

            with _lock:
                _can["speed_kmh"] = new_speed

        send_ipc_signal(VCP_IO.MOTOR_A, new_speed)
        print(f"[ACCEL] Speed: {new_speed}")
        time.sleep(ACCEL_INTERVAL)


def wheel_controller():
    """
    avoid_mode일 때는 target_steer 값에 도달할 때까지 점진적으로 이동
    평상시에는 기존 로직대로 동작
    """
    while not _stop:
        with _lock:
            is_avoid = _can.get("avoid_mode", False)
            target_val = _can.get("target_steer", STEER_CENTER)
            
            is_steering = _can.get("is_steering", False)
            is_steer_reverse = _can.get("is_steer_reverse", False)
            current_steer = _can.get("steer", STEER_CENTER)

        new_steer = current_steer

        # [Case 1] 장애물 회피 모드 (목표 각도 추종)
        if is_avoid:
            # 목표값과 현재값의 차이가 STEP보다 크면 이동
            if abs(current_steer - target_val) > STEER_STEP:
                if current_steer < target_val:
                    new_steer = min(STEER_MAX, current_steer + STEER_STEP)
                else:
                    new_steer = max(STEER_MIN, current_steer - STEER_STEP)
            else:
                # 거의 도달했으면 목표값으로 고정
                new_steer = target_val
            
            # 값 반영 및 전송
            with _lock:
                _can["steer"] = new_steer
            send_ipc_signal(VCP_IO.WHEEL, new_steer)

        # [Case 2] 수동/일반 조향 모드 (기존 로직 유지)
        elif is_steering:
            if is_steer_reverse: # Left
                new_steer = min(STEER_MAX, current_steer + STEER_STEP)
            else: # Right
                new_steer = max(STEER_MIN, current_steer - STEER_STEP)

            with _lock:
                _can["steer"] = new_steer
            send_ipc_signal(VCP_IO.WHEEL, new_steer)
        
        else:
            # 조향 입력 없을 때 (유지)
            send_ipc_signal(VCP_IO.WHEEL, current_steer)

        time.sleep(0.05)
        
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
            # BLINK_INTERVAL 기반으로 ON/OFF 결정
            on = get_blink_state()
            action = VCP_IO.ACTION_ON if on else VCP_IO.ACTION_OFF
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
    """
    print("[Emergency Logic] Started.")

    # 하드웨어 세팅에 따른 각도 설정 (STEER_CENTER=65)
    # 값이 클수록 왼쪽, 작을수록 오른쪽이라고 가정 (기존 코드 wheel_controller 참조)
    HARD_LEFT = 127   # 급커브 (왼쪽 최대)
    HARD_RIGHT = 0    # 급커브 (오른쪽 최대)
    SOFT_LEFT = 95    # 완만 (중앙 + 30)
    SOFT_RIGHT = 35   # 완만 (중앙 - 30)
    CENTER = 65

    while not _stop:
        with _lock:
            l_has, l_close = _zone_state["L"]
            f_has, f_close = _zone_state["F"]
            r_has, r_close = _zone_state["R"]

        # 제어 변수
        target_angle = CENTER
        avoid_active = False
        do_emergency_stop = False
        
        # ---------------------------------------------------------
        # 판단부 (우선순위: 정지 > 근접회피 > 원거리회피 > 직진)
        # ---------------------------------------------------------
        
        # 1. 전방위 차단 -> 정지
        if f_has and l_has and r_has:
            do_emergency_stop = True
            print("[LOGIC] ALL BLOCKED -> STOP")

        # 2. 전방은 뚫림 -> 직진 (회피 모드 해제)
        elif not f_has:
            avoid_active = False
            # print("[LOGIC] Front Clear -> Go Straight")

        # 3. 근접 장애물 (가까움) -> 급커브 (Long Turn)
        elif f_close or l_close or r_close:
            avoid_active = True
            # 왼쪽이 막혀있으면 오른쪽으로, 아니면 왼쪽으로
            if l_has: 
                target_angle = HARD_RIGHT
                print("[LOGIC] Close Obstacle -> Hard Right")
            else:
                target_angle = HARD_LEFT
                print("[LOGIC] Close Obstacle -> Hard Left")

        # 4. 원거리 장애물 -> 완만하게 회피 (Short Turn)
        elif f_has:
            avoid_active = True
            if l_has:
                target_angle = SOFT_RIGHT
                print("[LOGIC] Far Obstacle -> Soft Right")
            else:
                target_angle = SOFT_LEFT
                print("[LOGIC] Far Obstacle -> Soft Left")

        # ---------------------------------------------------------
        # 상태 업데이트 (제어 명령)
        # ---------------------------------------------------------
        with _lock:
            if do_emergency_stop:
                _can["is_accelerating"] = False
                _can["is_braking"] = True
                _can["avoid_mode"] = True
                _can["emergency"] = True
                # 정지 시 핸들은 중앙 유지 혹은 현상태 유지
            
            elif avoid_active:
                _can["avoid_mode"] = True
                _can["target_steer"] = target_angle # [핵심] 목표 각도 설정
                _can["is_steering"] = True          # wheel_controller 활성화
                _can["emergency"] = False
                
                # 깜빡이 연동
                if target_angle > CENTER: # 왼쪽 회전 중
                    _can["left_blinker"] = True
                    _can["right_blinker"] = False
                elif target_angle < CENTER: # 오른쪽 회전 중
                    _can["left_blinker"] = False
                    _can["right_blinker"] = True
            
            else:
                # 회피 상황 아님 -> 운전자/기본 제어권으로 넘김
                _can["avoid_mode"] = False
                _can["emergency"] = False
                _can["left_blinker"] = False
                _can["right_blinker"] = False
                # target_steer는 굳이 초기화 안 해도 avoid_mode가 False면 무시됨

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
            sock.settimeout(3.0)
            print(f"[AI] Connecting...")
            sock.connect((AI_G_IP, AI_PORT))
            print(f"[AI] Connected! Waiting for data...")
            
            f = sock.makefile("r", encoding="utf-8", newline="\n")
            sock.settimeout(None) 

            while not _stop:
                line = f.readline()
                if not line:
                    print("[AI] Server Disconnected.")
                    break
                
                # 1. [모니터링] 들어온 Raw Data 바로 출력
                raw_str = line.strip()
                if not raw_str: continue
                print(f"[RAW] {raw_str}") # 너무 시끄러우면 주석 처리

                try:
                    # 2. JSON 파싱
                    data = json.loads(raw_str)
                    
                    # 3. 상태 업데이트 (boxes 리스트 처리)
                    res_L, res_F, res_R = analyzer.update_status(data)
                    
                    # 4. 전역 변수 공유
                    with _lock:
                        _zone_state["L"] = res_L
                        _zone_state["F"] = res_F
                        _zone_state["R"] = res_R
                    
                    # 5. 상태 출력 (T/F)
                    print(f"[ZONE] L:{res_L} | F:{res_F} | R:{res_R}")

                except json.JSONDecodeError:
                    print(f"[AI] JSON Error: {raw_str}")
        
        except Exception as e:
            print(f"[AI] Connection Error: {e}")
            time.sleep(2)
        finally:
            if sock: 
                try: sock.close()
                except: pass



def main():
    global _stop
    print("Starting System...")

    # AI 수신 스레드 시작
    t_ai = threading.Thread(target=ai_data_worker, daemon=True, name="AI_Worker")
    t_ai.start()
    
    # # 블루투스 송신 스레드
    # t = threading.Thread(target=bt_car, daemon=True)
    # t.start()
    
    # 🚗 속도 제어 스레드 
    t_speed = threading.Thread(target=speed_controller, daemon=True, name="speed_controller")
    t_speed.start()
    
    # [추가] 장애물 회피 제어 스레드 활성화
    t_avoid = threading.Thread(target=emergency_control_logic, daemon=True, name="Avoid_Logic")
    t_avoid.start()

    # 비상등 제어 스레드
    t_emer = threading.Thread(target=emergency_worker, daemon=True, name="emergency_controller")
    t_emer.start()
      
    # 휠 제어 스레드
    t_wheel = threading.Thread(target=wheel_controller, daemon=True, name="can")
    t_wheel.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        global _stop
        _stop = True

        t.join(timeout=1.0)
        t_ai.join(timeout=1.0)
        t_speed.join(timeout=1.0)
        t_emer.join(timeout =1.0)
        t_wheel.join(timeout=1.0)

        print("Shutdown complete.")
        if sndfile: sndfile.close() 
if __name__ == "__main__":
    main()

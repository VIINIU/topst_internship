
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
    TURN_SIGNAL = 0x102
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
        C코드(NnAppMain.c)에서 보내주는 단일 객체 포맷을 처리
        Format: {"cls":1, "xmin":..., "ymin":..., "xmax":..., "ymax":..., "first":...}
        """
        current_time = time.time()

        # 데이터 유효성 검사 (xmin 키가 있는지 확인)
        if json_data and 'xmin' in json_data:
            xmin = json_data.get('xmin', 0)
            xmax = json_data.get('xmax', 0)
            ymin = json_data.get('ymin', 0)
            ymax = json_data.get('ymax', 0)
            
            # 크기 및 중앙값 계산
            obj_w = xmax - xmin
            obj_h = ymax - ymin
            obj_area = obj_w * obj_h
            center_x = xmin + (obj_w / 2.0)

            # 화면 3분할 기준선
            div_1 = self.w / 3.0
            div_2 = self.w * (2.0 / 3.0)

            # 구역 판단 (L/F/R)
            target_zone = ''
            if center_x < div_1: target_zone = 'L'
            elif center_x > div_2: target_zone = 'R'
            else: target_zone = 'F'

            # 해당 구역 상태 업데이트
            self.zones[target_zone].has_obstacle = True
            
            # 가까움 여부 판단 (전체 화면의 15% 이상)
            if (obj_area / self.total_area) > self.close_area_ratio:
                self.zones[target_zone].is_close = True
            
            # 마지막 업데이트 시간 갱신 (Timeout 방지)
            self.zones[target_zone].last_update_time = current_time

        # 타임아웃 처리 (0.5초 동안 업데이트 없는 구역은 Clear 처리)
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
        "is_steer_reverse": False
        "avoid_mode": False,} #[소연] 장애물 회피 플래그  

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

        else if is_brake:
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
    """L/R 플래그에 따라 steer 값을 0~127 범위에서 ±10씩 변경하고,
       그 값을 그대로 WHEEL IPC로 보내는 스레드."""
    while not _stop:
        with _lock:
            is_steering = _can.get("is_steering", False)
            is_steer_reverse = _can.get("is_steer_reverse", False)
            steer = _can.get("steer", STEER_CENTER)

        if is_steering:
            # 왼쪽 (reverse=True) / 오른쪽 (reverse=False)
            if is_steer_reverse:
                new_steer = min(STEER_MAX, steer + STEER_STEP)
            else:
                new_steer = max(STEER_MIN, steer - STEER_STEP)

            with _lock:
                _can["steer"] = new_steer

            # 현재 steer 값을 그대로 전송 (0~127)
            send_ipc_signal(VCP_IO.WHEEL, new_steer)
        else:
            send_ipc_signal(VCP_IO.WHEEL, steer)
        # 조향 안 할 때는 유지 (자동 복귀 안 함)
        time.sleep(0.05)   # 50ms마다 10씩 → 꾹 누르면 쭉쭉 움직이는 느낌


def emergency_worker():
    """비상등 플래그(_can['emergency'])를 보고
       좌/우 깜빡이를 주기적으로 On/Off 한다."""
    while not _stop:
        with _lock:
            emer = _can.get("emergency", False)
        if emer:
            # BLINK_INTERVAL 기반으로 ON/OFF 결정
            on = get_blink_state()
            action = VCP_IO.ACTION_ON if on else VCP_IO.ACTION_OFF
            # 좌우 모두 같은 상태로 깜빡이기
            send_ipc_signal(VCP_IO.TURN_SIGNAL, action, VCP_IO.SUB_LEFT)
            send_ipc_signal(VCP_IO.TURN_SIGNAL, action, VCP_IO.SUB_RIGHT)

        else:
            # 비상등이 꺼진 상태인데, 혹시 켜져있으면 한 번만 OFF 보내기
            send_ipc_signal(VCP_IO.TURN_SIGNAL, VCP_IO.ACTION_OFF, VCP_IO.SUB_LEFT)
            send_ipc_signal(VCP_IO.TURN_SIGNAL, VCP_IO.ACTION_OFF, VCP_IO.SUB_RIGHT)

        time.sleep(0.5)  # 너무 자주 돌 필요는 없음

# =========================================================
# 🔽 [소연] 장애물 회피 관련 코드 추가🔽 
# =========================================================
def emergency_control_logic():
    """
    전방위 차단(Stop) -> 정면 클리어(Straight) -> 근접 장애물(Long) -> 원거리 장애물(Short)
    """
    print("[Emergency Logic] Priority-Based Algorithm Started.")

    TIME_SHORT = 0.8
    TIME_LONG = 1.5

    while not _stop:
        with _lock:
            l_has, l_close = _zone_state["L"]
            f_has, f_close = _zone_state["F"]
            r_has, r_close = _zone_state["R"]

        # 로컬 제어 변수
        steer_time = 0.0
        steer_dir = False
        do_steering = False
        do_emergency_stop = False

        # ---------------------------------------------------------
        # 판단부(플래그 설정)
        # ---------------------------------------------------------
       
        if not f_has: # 1순위: 정면 클리어 (정상 주행 가능) -> lane detection으로 넘겨줄 것
            do_steering = False
            print("[PRIORITY 2] Front Clear. No Action.")

        
        elif f_has and l_has and r_has: 
            do_emergency_stop = True
            print("[PRIORITY 1] ALL BLOCKED! Emergency Stop.")

        
        elif f_close or l_close or r_close: # 3순위: 장애물이 가깝거나 정면이 근접한 경우 (긴 조향)
            do_steering = True
            steer_time = TIME_LONG
            # 회피 방향 결정: 왼쪽이 비어있으면 왼쪽(True), 아니면 오른쪽(False)
            steer_dir = True if not l_has else False
            print(f"[PRIORITY 3] CLOSE OBSTACLE! Steering {'Left' if steer_dir else 'Right'} (Long)")

        elif f_has: # 4순위: 장애물이 멀리 있는 경우 (짧은 조향)
            do_steering = True
            steer_time = TIME_SHORT
            # 회피 방향 결정
            steer_dir = True if not l_has else False
            print(f"[PRIORITY 4] Obstacle Ahead (Far). Steering {'Left' if steer_dir else 'Right'} (Short)")

        # ---------------------------------------------------------
        # 실제 제어 실행부
        # ---------------------------------------------------------
        if do_emergency_stop:
            with _lock:
                # 비상 정지 시에는 즉시 모든 제어 플래그를 정지 상태로
                _can["is_accelerating"] = False
                _can["is_braking"] = True
                _can["is_steering"] = False  # 조향도 멈춤
                _can["avoid_mode"] = True
            
            # # 정지 명령은 긴급하므로 즉시 전송
            # send_ipc_signal(VCP_IO.MOTOR_A, 0)
            # send_ipc_signal(VCP_IO.BREAK_LIGHT, VCP_IO.ACTION_ON)
            # print("[Logic] EMERGENCY STOP EXECUTED")

        elif do_steering:
            # 1. 조향 시작 상태 알림
            with _lock:
                _can["is_steering"] = True
                _can["is_steer_reverse"] = steer_dir  # True면 왼쪽, False면 오른쪽
                _can["avoid_mode"] = True
            
            # 2. steer_time 동안 'wheel_controller'가 동작하도록 대기
            # 이 시간 동안 wheel_controller 스레드가 50ms마다 steer 값을 ±10씩 바꿉니다.
            time.sleep(steer_time) 
            
            # 3. 조향 종료
            with _lock:
                _can["is_steering"] = False
            print(f"[Logic] Steering Action Finished.")
        
        else:
            with _lock:
                _can["avoid_mode"] = False
     
        # 메인 루프 주기 조절
        time.sleep(0.1)
# =========================================================
# 🔼 [소연] 장애물 회피 관련 코드 끝 🔼
# =========================================================

# =========================================================
# BT_제어_함수
# =========================================================

def handle_bt_command(ch):
    with _lock:
        if ch == "F":        #전진 시작 (가속 플래그 ON)
            _can["is_accelerating"] = True
            _can["is_resverse"] = False
            _can["is_braking"] = False
            print("[BT] Forward START (Accelerating)")

        elif ch == "B":      #후진/감속 시작
            _can["is_accelerating"] = True
            _can["is_resverse"] = True
            _can["is_braking"] = False
            print("[BT] Brake START (Decelerating)")

        elif ch == "L":      # 좌회전
            _can["is_steering"] =True
            _can["is_steer_reverse"] =True

        elif ch == "R":      # 우회전
            _can["is_steering"] = True
            _can["is_steer_reverse"] =False

        elif ch == "u":      # 헤드라이트 OFF
            send_ipc_signal(VCP_IO.HEAD_LIGHT, 0x02)
            print("[BT] headlights OFF")

        elif ch == "X":      # 비상등
            _can["emergency"] = True
            print("[BT] Emergency ON")

        elif ch == "x":
            _can["emergency"] = False
            print("[BT] Emergency OFF")

        elif ch == "w":      # 브레이크등 OFF
            _can["is_braking"] = False
            send_ipc_signal(VCP_IO.BREAK_LIGHT, 0x02)
            print("[BT] break OFF")

        elif ch == "W":      # 브레이크등 ON
            _can["is_braking"] = True
            send_ipc_signal(VCP_IO.BREAK_LIGHT, 0x01)
            print("[BT] break ON")

        elif ch == "U":      # 헤드라이트 ON
            send_ipc_signal(VCP_IO.HEAD_LIGHT, 0x01)
            print(f"[BT] Headlight ON")


        elif ch == "G":  
            _can["is_accelerating"] = True
            _can["is_resverse"] = False
            _can["is_steering"] = True
            _can["is_steer_reverse"] =True
            _can["is_braking"] = False

        elif ch == "H":
            _can["is_accelerating"] = True
            _can["is_resverse"] = False
            _can["is_steering"] = True
            _can["is_steer_reverse"] =False
            _can["is_braking"] = False
            

        # ----- 후진 + 회전 -----
        elif ch == "I":
            _can["is_accelerating"] = True
            _can["is_resverse"] = True
            _can["is_steering"] = True
            _can["is_steer_reverse"] =True
            _can["is_braking"] = False
        elif ch == "J":
            _can["is_accelerating"] = True
            _can["is_resverse"] = True
            _can["is_steering"] = True
            _can["is_steer_reverse"] =False
            _can["is_braking"] = False

        elif ch == "S":
            _can["is_accelerating"] = False
            _can["is_braking"] = True
            _can["speed_kmh"] = 0
            _can["is_steering"] = False
            _can["steer"] = 65
            
        else:
            print(f"[BT] Unknown key: {ch}")
         
def bt_car():
    try:
        ser = serial.Serial(
            port="/dev/ttyAMA1",
            baudrate=9600,
            timeout=0.1
        )
    except Exception as e:
        print(f"uart open fail: {e}")
        return

    print("✓ UART Opened (/dev/ttyAMA1 @ 9600)")
    print("Waiting for data...\n")

    try:
        while True:
            data = ser.read(1)
            if data:
                ch = data.decode(errors="ignore").strip()
                print(f"[RX] '{ch}'")

                #여기서 CAN 상태 업데이트
                handle_bt_command(ch)

            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nexit (Ctrl+C)")
    finally:
        ser.close()
        print("UART Closed")

# =========================================================
# [Class] 구역 상태 및 분석 클래스
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
                print(f"[RAW] {raw_str}")

                try:
                    # 2. JSON 파싱
                    data = json.loads(raw_str)
                    
                    # 3. 상태 업데이트 (Clear 여부 판단)
                    res_L, res_F, res_R = analyzer.update_status(data)
                    
                    # 4. 전역 변수 공유
                    with _lock:
                        _zone_state["L"] = res_L
                        _zone_state["F"] = res_F
                        _zone_state["R"] = res_R
                    
                    # (선택) 분석된 Zone 상태 출력 - 너무 빠르면 주석 처리
                    print(f"[Zone] L:{res_L} F:{res_F} R:{res_R}")

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

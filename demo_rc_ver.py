#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json, time, socket, threading, http.client
import serial
from urllib.parse import urlparse
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

sndfile = open("/dev/tcc_ipc_micom", 'wb')
HOST,PORT= "0.0.0.0", 9999          
SEND_HZ  = 30                     
POLL_HZ  = 60   
CAN_HZ = 60       
IDLE_RPM = 552
BLINK_INTERVAL = 0.5  # 깜빡이 간격 (0.5초)

# 🚗 속도 증가 설정
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
        "is_steer_reverse": False}  

_lock   = threading.Lock()
_stop   = False
 
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

# 🚗 속도 증가/감소 처리 스레드
def speed_controller():
    """F를 누르고 있으면 속도 증가, B를 누르고 있으면 속도 감소"""
    while not _stop:
        with _lock:
            is_accel = _can.get("is_accelerating", False)
            is_resverse = _can.get("is_resverse", False)
            current_speed = _can.get("speed_kmh", 0)
            is_brake = _can.get("is_braking",False)
        
        if is_accel and not is_brake:
            # 가속
            if not is_resverse:
                new_speed = min(current_speed + SPEED_INCREMENT, MAX_SPEED)
                with _lock:
                    _can["speed_kmh"] = new_speed
                send_ipc_signal(VCP_IO.MOTOR_A, new_speed)
                print(f"[ACCEL] Speed: {new_speed}")

            else:
                new_speed = min(current_speed - SPEED_INCREMENT, MAX_SPEED)
                with _lock:
                    _can["speed_kmh"] = new_speed
                send_ipc_signal(VCP_IO.MOTOR_A, new_speed)
                
                print(f"[ACCEL] Speed: {new_speed}")
        else:
            # 감속
            new_speed = 0
            with _lock:
                _can["speed_kmh"] = new_speed
            send_ipc_signal(VCP_IO.MOTOR_A, new_speed)
        
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

def serve():
    global _stop
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"[NET] listening on {HOST}:{PORT}")

    # Qt 표시용 속도 (독립적으로 동작)
    qt_speed = 0.0     # 시각적 speed
    qt_rpm = float(IDLE_RPM)
    qt_max = QT_MAX_SPEED

    while not _stop:
        conn, addr = srv.accept()
        print(f"[NET] client connected: {addr[0]}:{addr[1]}")
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            period = 1.0 / SEND_HZ
            next_t = time.monotonic()
            encoder = json.dumps

            while True:
                # CAN 상태 읽기
                with _lock:
                    is_accel = _can["is_accelerating"]
                    is_reverse = _can["is_resverse"]

                # ■■■ Qt speed 증가 로직 ■■■
                if is_accel:
                    if not is_reverse:   # 전진
                        qt_speed = min(qt_speed + QT_SPEED_INCREMENT, qt_max)
                    else:                # 후진
                        qt_speed = max(qt_speed - QT_SPEED_INCREMENT, -qt_max)
                else:
                    # 자연감속 (좀 더 부드럽게 표현)
                    if qt_speed > 0:
                        qt_speed = max(0, qt_speed - QT_SPEED_DECREMENT)
                    elif qt_speed < 0:
                        qt_speed = min(0, qt_speed + QT_SPEED_DECREMENT)

                # rpm은 Qt용 속도 기반으로 계산
                qt_rpm = float(IDLE_RPM + abs(qt_speed) * 40)

                # ■■■ _latest 업데이트 ■■■
                with _lock:
                    _latest["speed_kmh"] = float(qt_speed)
                    _latest["rpm"] = float(qt_rpm)

                # JSON 전송
                payload = dict(_latest)
                line = encoder(payload, separators=(",",":")) + "\n"
                conn.sendall(line.encode("utf-8"))

                # 타이밍 조절
                next_t += period
                now = time.monotonic()
                if next_t > now:
                    time.sleep(next_t - now)
                else:
                    next_t = now

        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            try:
                conn.close()
            except:
                pass



def main():
    print("Starting ETS2 Telemetry Monitor...")
    
    # 블루투스 송신 스레드
    t = threading.Thread(target=bt_car, daemon=True)
    t.start()
    
    # 🚗 속도 제어 스레드 
    t_speed = threading.Thread(target=speed_controller, daemon=True, name="speed_controller")
    t_speed.start()
    
    # 비상등 제어 스레드
    t_emer = threading.Thread(target=emergency_worker, daemon=True, name="emergency_controller")
    t_emer.start()
      
    # 휠 제어 스레드
    t_wheel = threading.Thread(target=wheel_controller, daemon=True, name="can")
    t_wheel.start()

    # qt 쓰레드
    t_serve = threading.Thread(target=serve, daemon=True, name="qt_controller")
    t_serve.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        global _stop
        _stop = True
        t.join(timeout=1.0)
        t_speed.join(timeout=1.0)
        t_emer.join(timeout =1.0)
        t_wheel.join(timeout=1.0)
        t_serve.join(timeout=1.0)
        print("Shutdown complete.")
        sndfile.close()
 
if __name__ == "__main__":
    main()

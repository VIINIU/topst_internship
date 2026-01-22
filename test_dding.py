import socket
import json

AI_G_IP = "192.168.0.101"
PORT = 9999

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((AI_G_IP, PORT))
    print(f"[D3-G] Connected to AI-G ({AI_G_IP}:{PORT})")

    f = sock.makefile("r", encoding="utf-8", newline="\n")

    try:
        while True:
            line = f.readline()
            if not line:
                print("[D3-G] Connection closed by server")
                break

            line = line.strip()
            if not line:
                continue

            try:
                msg = json.loads(line)
            except json.JSONDecodeError as e:
                print(f"[D3-G] JSON decode error: {e}")
                print(f"[D3-G] Raw: {line[:200]}")
                continue

            # AI-G가 안 보내면 기본값 처리
            det_type = msg.get("type", "DET")
            cls_id = msg.get("cls")
            score = msg.get("score", "N/A")   # AI-G에서 안 보내면 N/A
            xmin = msg.get("xmin")
            ymin = msg.get("ymin")
            xmax = msg.get("xmax")
            ymax = msg.get("ymax")

            print("\n[D3-G] Inference Result (ALL JSON FIELDS)")
            print("----------------------------------------")
            print(f"type: {det_type}")
            print(f"cls: {cls_id}")
            print(f"score: {score}")
            print(f"xmin: {xmin}")
            print(f"ymin: {ymin}")
            print(f"xmax: {xmax}")
            print(f"ymax: {ymax}")

    except KeyboardInterrupt:
        print("\n[D3-G] Stopped")
    finally:
        try:
            f.close()
        except Exception:
            pass
        sock.close()
        print("[D3-G] Socket closed")

if __name__ == "__main__":
    main()

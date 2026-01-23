import socket
import json

AI_G_IP = "192.168.0.101"
PORT = 9999

def fmt_score(v):
    if v is None:
        return "N/A"
    try:
        return f"{float(v):.2f}"
    except Exception:
        return str(v)

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
            cls_id = msg.get("cls", "N/A")
            score = msg.get("score", None)
            xmin = msg.get("xmin", "N/A")
            ymin = msg.get("ymin", "N/A")
            xmax = msg.get("xmax", "N/A")
            ymax = msg.get("ymax", "N/A")
            first = msg.get("first", False)       # 추가

            print("\n[D3-G] Inference Result (ALL JSON FIELDS)")
            print("----------------------------------------")
            print(f"type: {det_type}")
            print(f"cls: {cls_id}")
            print(f"score: {score}")
            print(f"xmin: {xmin}")
            print(f"ymin: {ymin}")
            print(f"xmax: {xmax}")
            print(f"ymax: {ymax}")
            print(f"first: {first}")

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

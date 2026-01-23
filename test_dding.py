import socket
import json

AI_G_IP = "192.168.0.101"
PORT = 9999

def fmt_num(v, nd=2):
    if v is None:
        return "N/A"
    try:
        return f"{float(v):.{nd}f}"
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

            boxes = msg.get("boxes", None)
            if not isinstance(boxes, list):
                print("[D3-G] Unexpected JSON format (missing 'boxes' list)")
                print(f"[D3-G] Raw JSON: {msg}")
                continue

            print("\n[D3-G] Inference Result (BOXES)")
            print("----------------------------------------")
            print(f"boxes: {len(boxes)}")

            for i, b in enumerate(boxes):
                if not isinstance(b, dict):
                    print(f"  - box[{i}] invalid: {b}")
                    continue

                cls_id = b.get("cls", "N/A")
                score = b.get("score", None)
                xmin = b.get("xmin", "N/A")
                ymin = b.get("ymin", "N/A")
                xmax = b.get("xmax", "N/A")
                ymax = b.get("ymax", "N/A")

                print(f"\n  [box {i}]")
                print(f"    cls  : {cls_id}")
                print(f"    score: {fmt_num(score, 2)}")
                print(f"    xmin : {xmin}")
                print(f"    ymin : {ymin}")
                print(f"    xmax : {xmax}")
                print(f"    ymax : {ymax}")

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

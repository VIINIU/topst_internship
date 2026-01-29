import socket
import json

AI_G_IP = "192.168.0.101"
PORT = 9999
# 
def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((AI_G_IP, PORT))
        print(f"[D3-G] Connected to {AI_G_IP}:{PORT}")
        
        f = sock.makefile("r", encoding="utf-8", newline="\n")
        
        while True:
            line = f.readline()
            if not line:
                break
            
            line = line.strip()
            if not line:
                continue

            try:
                msg = json.loads(line)
                
                # [디버깅 핵심] 받은 데이터의 키(Key)들을 전부 출력
                print("\n" + "="*40)
                print("[DEBUG] Received JSON Structure:")
                print(json.dumps(msg, indent=2, ensure_ascii=False))
                print("="*40)

                # 기존 로직 테스트
                if "boxes" in msg:
                    print(f" -> Found 'boxes' key with {len(msg['boxes'])} items.")
                else:
                    print(f" -> [WARNING] 'boxes' key NOT found. Keys are: {list(msg.keys())}")

            except json.JSONDecodeError:
                print(f"[Error] Raw line: {line}")

    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
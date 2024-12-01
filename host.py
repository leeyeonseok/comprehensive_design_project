import socket

# 1. 소켓 생성
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 2. IP와 포트 바인딩
server_socket.bind(('0.0.0.0', 12345))

# 3. 클라이언트 연결 대기
server_socket.listen(5)
print("클라이언트를 기다리는 중...")

# 4. 클라이언트 연결 수락
conn, addr = server_socket.accept()
print(f"클라이언트 연결됨: {addr}")

# 5. 데이터 송수신
while True:
    data = conn.recv(1024).decode()  # 클라이언트로부터 데이터 수신
    if not data:
        break
    print(f"클라이언트로부터 받은 데이터: {data}")
    conn.send("서버가 데이터를 받았습니다!".encode())  # 응답 전송

# 6. 연결 종료
conn.close()
server_socket.close()

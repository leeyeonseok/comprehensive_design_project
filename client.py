import socket

# 1. 소켓 생성
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 2. 서버에 연결 요청
client_socket.connect(('127.0.0.1', 12345))

# 3. 데이터 송수신
client_socket.send("안녕하세요, 서버!".encode())  # 데이터 전송
response = client_socket.recv(1024).decode()  # 응답 수신
print(f"서버 응답: {response}")

# 4. 연결 종료
client_socket.close()

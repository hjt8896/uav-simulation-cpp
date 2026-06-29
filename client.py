import socket
import numpy as np
import cv2
import base64
import requests

TCP_IP = "127.0.0.1"
TCP_PORT = 8080

# 1. 创建 TCP Server
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 防止端口重启冲突
sock.bind((TCP_IP, TCP_PORT))
sock.listen(1)

AUTODL_URL = "http://127.0.0.1:6006/get_target"
prompt = "This is a drone FPV image. Find the blue square outline located below the brown bear. Reply ONLY with the bounding box coordinates of this blue square in the format [x_min, y_min, x_max, y_max]."

print("🌉 Python 桥接层已启动，等待 C++ 建立 TCP 连接...")
# 阻塞等待 C++ 的连接
conn, addr = sock.accept()
print(f"✅ C++ 物理引擎已连接！来源地址: {addr}")

IMAGE_SIZE = 256 * 256 * 3

while True:
    # 2. TCP 是流式传输，必须用循环把 196608 字节“吸”满
    data = b''
    while len(data) < IMAGE_SIZE:
        packet = conn.recv(IMAGE_SIZE - len(data))
        if not packet:
            break
        data += packet

    if len(data) < IMAGE_SIZE:
        print("❌ 连接断开或数据不完整，等待下一次传输...")
        break

    print("📸 收到一帧完整 FPV 图像，正在呼叫云端大脑...")

    # 解析图像并翻转
    img_array = np.frombuffer(data, dtype=np.uint8).reshape((256, 256, 3))
    img_array = cv2.flip(img_array, 0)

    # 将 RGB 转换为 OpenCV 默认的 BGR 格式
    img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

    # ==========================================
    # 📺 核心新增：弹出一个独立窗口，实时直播！
    # ==========================================
    # 因为 256x256 在高分屏上可能有点小，我们把它放大一倍方便人类观察
    img_display = cv2.resize(img_bgr, (512, 512))
    cv2.imshow("AI Brain Monitor (FPV)", img_display)
    cv2.waitKey(1)  # 这行极其重要，它能让 OpenCV 刷新窗口并处理系统事件！

    # 编码为 Base64 发给 AutoDL
    _, buffer = cv2.imencode('.jpg', cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR))
    img_b64_str = base64.b64encode(buffer).decode('utf-8')

    try:
        response = requests.post(AUTODL_URL, json={"image_base64": img_b64_str, "prompt": prompt})
        print("🎯 大脑返回:", response.json()["response"])
    except Exception as e:
        print("网络错误:", e)
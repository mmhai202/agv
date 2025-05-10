import serial
import time
from picamera2 import Picamera2
import cv2
import cv2.aruco as aruco
import numpy as np

# Khởi tạo UART
uart = serial.Serial('/dev/ttyS0', 115200)  # UART0 trên Raspberry Pi
time.sleep(2)  # Đợi một lúc để kết nối UART ổn định

# Khởi tạo camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "BGR888"
picam2.configure("preview")
picam2.start()
time.sleep(1)

# Dictionary & thông số phát hiện ArUco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

print("Đang dò ArUco marker từ camera CSI... Nhấn Ctrl+C để dừng")

try:
    while True:
        frame = picam2.capture_array()

        # Chuyển ảnh sang xám
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Phát hiện marker
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for i in range(len(ids)):
                c = corners[i][0]

                marker_id = ids[i][0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))

                # Góc nghiêng marker (so với trục ngang)
                dx = c[1][0] - c[0][0]
                dy = c[1][1] - c[0][1]
                angle = np.degrees(np.arctan2(dy, dx))

                print(f"raspi_camera,{marker_id},{cx},{cy},{angle:.2f}")

                # Gửi thông tin qua UART tới ESP32
                uart.write(f"raspi_camera,{marker_id},{cx},{cy},{angle:.2f}\n".encode())
                uart.flush()  # Đảm bảo dữ liệu đã được gửi ngay lập tức

        time.sleep(0.5)  # ~10 lần mỗi giây

except KeyboardInterrupt:
    print("\nDừng chương trình")
    uart.close()
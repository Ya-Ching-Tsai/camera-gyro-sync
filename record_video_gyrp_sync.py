import os
os.environ["QT_QPA_PLATFORM"] = "offscreen"  # 無視窗模式避免 Qt 錯誤

from picamera2 import Picamera2
import cv2, time, csv, threading, smbus2, math, numpy as np

# ===== LSM6DS3 陀螺儀設定 =====
I2C_ADDR = 0x6b
I2C_BUS = 1
DPS2RADS = math.pi / 180.0  # 度/秒 → 弧度/秒 轉換

class LSM6DS3:
    def __init__(self, address=I2C_ADDR, bus=I2C_BUS):
        self.bus = smbus2.SMBus(bus)
        self.address = address

        # 啟用加速度計與陀螺儀 (ODR=104Hz, FS=±500dps)
        self.bus.write_byte_data(self.address, 0x10, 0x44)  # CTRL1_XL
        self.bus.write_byte_data(self.address, 0x11, 0x44)  # CTRL2_G

        # 啟用 BDU=1（Block Data Update）與 IF_INC=1（地址自增）
        self.bus.write_byte_data(self.address, 0x12, 0x44)  # CTRL3_C

        # 讀取目前 FS 設定
        reg_val = self.bus.read_byte_data(self.address, 0x11)
        fs_bits = (reg_val >> 2) & 0b11
        self.fs_mode = {0: 245, 1: 500, 2: 1000, 3: 2000}[fs_bits]
        self.scale_dps = {0: 0.00875, 1: 0.0175, 2: 0.035, 3: 0.07}[fs_bits]
        self.scale_rads = self.scale_dps * DPS2RADS
        print(f"✅ LSM6DS3 initialized: ±{self.fs_mode} dps, scale={self.scale_rads:.2e} rad/s/LSB")

        # 自動 bias 校正
        self.bias_x, self.bias_y, self.bias_z = self.calibrate_bias()
        print(f"✅ Gyro bias: X={self.bias_x}, Y={self.bias_y}, Z={self.bias_z}")

    def calibrate_bias(self, samples=200, delay=0.005):
        xs, ys, zs = [], [], []
        for _ in range(samples):
            gx, gy, gz = self._read_raw()
            xs.append(gx); ys.append(gy); zs.append(gz)
            time.sleep(delay)
        return int(np.mean(xs)), int(np.mean(ys)), int(np.mean(zs))

    def _read_raw(self):
        # 一次性讀取 6 bytes（確保資料一致）
        data = self.bus.read_i2c_block_data(self.address, 0x22, 6)

        def to_signed(val):
            return val - 65536 if val & 0x8000 else val

        gx = to_signed(data[1] << 8 | data[0])
        gy = to_signed(data[3] << 8 | data[2])
        gz = to_signed(data[5] << 8 | data[4])
        return gx, gy, gz

    def read_gyro(self):
        gx_raw, gy_raw, gz_raw = self._read_raw()
        gx = (gx_raw - self.bias_x) * self.scale_rads
        gy = (gy_raw - self.bias_y) * self.scale_rads
        gz = (gz_raw - self.bias_z) * self.scale_rads
        return gx, gy, gz


# ===== 陀螺儀擷取執行緒 =====
class GyroThread(threading.Thread):
    def __init__(self, sensor, start_time, csv_path="gyro_data.csv", freq_hz=200):
        super().__init__(daemon=True)
        self.sensor = sensor
        self.start_time = start_time
        self.csv_path = csv_path
        self.freq = freq_hz
        self.stop_flag = False

    def run(self):
        period = 1.0 / self.freq
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_ms", "gx(rad/s)", "gy(rad/s)", "gz(rad/s)"])
            while not self.stop_flag:
                now = time.monotonic()
                t_ms = (now - self.start_time) * 1000.0
                gx, gy, gz = self.sensor.read_gyro()
                writer.writerow([f"{t_ms:.3f}", gx, gy, gz])
                time.sleep(period)


# ===== 錄影主程式 =====
def record_video_and_timestamps(video_path="record_sync.mp4",
                                frame_csv="frame_ts.csv",
                                duration=30,      # 錄影秒數
                                fps=30):

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (1280, 720)}, controls={"FrameRate": fps})
    picam2.configure(config)
    picam2.start()

    start_t = time.monotonic()
    sensor = LSM6DS3()
    gyro_thread = GyroThread(sensor, start_t, "gyro_data.csv", freq_hz=200)
    gyro_thread.start()

    out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (1280, 720))
    with open(frame_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame_idx", "time_ms"])
        frame_idx = 0

        print(f"🎥 開始錄影 ({duration}s)...")
        while (time.monotonic() - start_t) < duration:
            frame = picam2.capture_array()
            t_ms = (time.monotonic() - start_t) * 1000.0
            writer.writerow([frame_idx, f"{t_ms:.3f}"])
            out.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            frame_idx += 1

    picam2.stop()
    out.release()
    gyro_thread.stop_flag = True
    gyro_thread.join()

    print("✅ 錄影與陀螺儀同步完成")

    # ===== 統計結果 =====
    with open(frame_csv) as f1, open("gyro_data.csv") as f2:
        frame_count = sum(1 for _ in f1) - 1
        gyro_count = sum(1 for _ in f2) - 1
    print(f"🎬 影片幀數: {frame_count}")
    print(f"🌀 陀螺儀資料筆數: {gyro_count}")


# ===== 主程式 =====
if __name__ == "__main__":
    record_video_and_timestamps(duration=30)

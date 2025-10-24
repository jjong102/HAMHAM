# imu_plot.py
# pip install bleak matplotlib
import asyncio
from bleak import BleakClient, BleakScanner
import struct, collections, time
import matplotlib.pyplot as plt

DEVICE_NAME = "IMU-ROLLPITCH"
CHAR_UUID   = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # Arduino와 동일해야 함(Notify 특성)

WINDOW_SEC = 10.0
buf_roll  = collections.deque()
buf_pitch = collections.deque()
buf_time  = collections.deque()

# === LPF 추가 ===
LPF_ALPHA = 0.2
f_roll = None
f_pitch = None

def now():
    return time.time()

def trim():
    t0 = now() - WINDOW_SEC
    while buf_time and buf_time[0] < t0:
        buf_time.popleft(); buf_roll.popleft(); buf_pitch.popleft()

def notification_handler(_, data: bytearray):
    # 8 bytes => <ff
    if len(data) >= 8:
        roll, pitch = struct.unpack("<ff", data[:8])

        # === LPF 적용 (지수평활) ===
        global f_roll, f_pitch
        if f_roll is None:
            f_roll, f_pitch = roll, pitch
        else:
            f_roll  = LPF_ALPHA * roll  + (1 - LPF_ALPHA) * f_roll
            f_pitch = LPF_ALPHA * pitch + (1 - LPF_ALPHA) * f_pitch

        t = now()
        buf_time.append(t); buf_roll.append(f_roll); buf_pitch.append(f_pitch)
        trim()

async def find_device():
    print("Scanning BLE devices...")
    devices = await BleakScanner.discover(timeout=5.0)
    for d in devices:
        if d.name == DEVICE_NAME:
            print("Found:", d)
            return d
    raise RuntimeError("Device not found. Check name/advertising.")

async def plot_loop():
    """비동기 플롯 업데이트 루프(이벤트 루프를 막지 않음)."""
    plt.ion()
    fig, ax = plt.subplots()
    line_roll,  = ax.plot([], [], label="roll (deg)")
    line_pitch, = ax.plot([], [], label="pitch (deg)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("deg")
    ax.set_ylim(-60, 60)          # === y축 -60~60 고정 ===
    ax.set_xlim(0, WINDOW_SEC)
    ax.legend()
    fig.canvas.draw()
    fig.canvas.flush_events()

    while True:
        if buf_time:
            t0 = buf_time[0]
            t  = [ti - t0 for ti in buf_time]
            line_roll.set_data(t, list(buf_roll))
            line_pitch.set_data(t, list(buf_pitch))
            # y축은 고정, x축만 최신 구간으로 이동
            if t:
                tmax = max(t[-1], WINDOW_SEC)
                ax.set_xlim(tmax - WINDOW_SEC, tmax)
            fig.canvas.draw()
            fig.canvas.flush_events()
        await asyncio.sleep(0.05)  # 20Hz 업데이트

async def run():
    dev = await find_device()
    async with BleakClient(dev.address) as client:
        print("Connected:", client.is_connected)

        # (선택) 서비스/캐릭터리스틱 확인 로그
        try:
            svcs = await client.get_services()
            if CHAR_UUID.lower() not in [c.uuid.lower() for s in svcs for c in s.characteristics]:
                print("[경고] 지정한 CHAR_UUID를 기기에서 못 찾았습니다. UUID를 다시 확인하세요.")
        except Exception:
            pass

        await client.start_notify(CHAR_UUID, notification_handler)

        # 플롯과 BLE를 함께 돌리기
        plot_task = asyncio.create_task(plot_loop())

        # 연결 유지 동안 대기
        try:
            while client.is_connected:
                await asyncio.sleep(0.2)
        finally:
            await client.stop_notify(CHAR_UUID)
            plot_task.cancel()
            try:
                await plot_task
            except asyncio.CancelledError:
                pass

if __name__ == "__main__":
    asyncio.run(run())

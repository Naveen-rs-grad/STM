import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PORT = "COM3"
BAUD = 115200

times, rpms = [], []
pending = bytearray()
ser = serial.Serial(PORT, BAUD, timeout=0)

fig, ax = plt.subplots()
actual_line, = ax.plot([], [], label="Measured RPM")
target_line = ax.axhline(0, color="red", linestyle="--", label="Target RPM")
ax.set(xlabel="Time (seconds)", ylabel="RPM", title="Fan speed")
ax.grid(True)
ax.legend()


def update(_):
    global pending
    pending.extend(ser.read(ser.in_waiting))
    while b"\n" in pending:
        line, _, pending = pending.partition(b"\n")
        fields = line.decode("ascii", errors="ignore").strip().split(",")
        if len(fields) not in (4, 5):
            continue
        try:
            t_ms, target, rpm = map(float, fields[:3])
        except ValueError:
            continue
        if times and t_ms / 1000 < times[-1]:
            times.clear()
            rpms.clear()
        times.append(t_ms / 1000)
        rpms.append(rpm)
        target_line.set_ydata([target, target])
    if times:
        actual_line.set_data(times, rpms)
        ax.relim()
        ax.autoscale_view()
    return actual_line, target_line


animation = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
try:
    plt.show()
finally:
    ser.close()


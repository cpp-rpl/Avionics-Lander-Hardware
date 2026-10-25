#!/usr/bin/env python3
"""
rpl_log.py — capture and plot RPL flight-computer serial telemetry.

  pip3 install pyserial matplotlib

  python3 rpl_log.py capture /dev/tty.usbmodemXXXX run1.log   # Ctrl+C to stop
  python3 rpl_log.py plot run1.log                            # writes run1.png

Lines starting with "T," are telemetry CSV; lines starting with "#" are
monitor/event messages and are echoed but not plotted.
"""
import sys


def capture(port, out_path, baud=115200):
    import serial
    with serial.Serial(port, baud, timeout=1) as ser, open(out_path, "w") as f:
        print(f"Capturing {port} @ {baud} -> {out_path}  (Ctrl+C to stop)")
        try:
            while True:
                line = ser.readline().decode(errors="replace")
                if line:
                    sys.stdout.write(line)
                    f.write(line)
                    f.flush()
        except KeyboardInterrupt:
            print("\nStopped.")


def plot(log_path):
    import matplotlib.pyplot as plt

    t, alt, vz, az = [], [], [], []
    events, mon = [], []
    with open(log_path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("T,"):
                p = line.split(",")
                if len(p) != 8:
                    continue
                try:
                    t.append(int(p[2]) / 1000.0)
                    alt.append(int(p[3]) / 1000.0)
                    vz.append(int(p[4]) / 1000.0)
                    az.append(int(p[7]))
                except ValueError:
                    continue
            elif line.startswith("# EVENT"):
                events.append(line)
            elif line.startswith("# MON"):
                mon.append(line)

    if not t:
        sys.exit("No telemetry lines found.")

    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    ax[0].plot(t, alt);  ax[0].set_ylabel("altitude (m)")
    ax[1].plot(t, vz);   ax[1].set_ylabel("vz (m/s)")
    ax[2].plot(t, az);   ax[2].set_ylabel("accel z (mg)")
    ax[2].set_xlabel("RTOS tick time (s)")
    fig.suptitle("RPL flight computer — FreeRTOS telemetry (STM32H723ZG)")
    for a in ax:
        a.grid(True, alpha=0.3)
    fig.tight_layout()

    out = log_path.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=150)
    print(f"{len(t)} samples, {len(events)} abort events -> {out}")
    if mon:
        print("Last health line:", mon[-1])


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "capture":
        capture(sys.argv[2], sys.argv[3])
    elif len(sys.argv) >= 3 and sys.argv[1] == "plot":
        plot(sys.argv[2])
    else:
        print(__doc__)

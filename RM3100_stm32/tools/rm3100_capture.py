#!/usr/bin/env python3
"""RM3100 传感器2实时显示与 CSV 采集。"""
import csv
import datetime as dt
import math
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, ttk

import serial
from serial.tools import list_ports


# 在 PyCharm 中运行前，改成电脑实际识别到的串口号，例如 "COM5"。
SERIAL_PORT = "COM5"
BAUD_RATE = 115200
OUTPUT_CSV = ""  # 留空时自动生成带日期时间的文件名，也可填写 "scan.csv"

HEADER = [
    "seq", "trigger1_us", "trigger2_us", "ready_mask",
    "sensor1_dx_raw", "sensor1_dy_raw", "sensor1_dz_raw",
    "sensor2_dx_raw", "sensor2_dy_raw", "sensor2_dz_raw",
    "sensor1_dx_uT", "sensor1_dy_uT", "sensor1_dz_uT",
    "sensor2_dx_uT", "sensor2_dy_uT", "sensor2_dz_uT",
]

# PNI's typical gain at cycle count 200 is 75 LSB per microtesla.
GAIN_LSB_PER_UT = 75.0
DISPLAY_EVERY_N_FRAMES = 10
DISPLAY_INTERVAL_MS = 500


def xor_checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def parse_frame(line: bytes):
    line = line.strip()
    if b"*" not in line:
        raise ValueError("missing checksum separator")
    payload, checksum_text = line.rsplit(b"*", 1)
    if xor_checksum(payload) != int(checksum_text, 16):
        raise ValueError("checksum mismatch")
    fields = payload.decode("ascii").split(",")
    if len(fields) != 11 or fields[0] != "RM":
        raise ValueError("unexpected frame")
    values = [int(value) for value in fields[1:]]
    if not 0 <= values[0] <= 0xFFFFFFFF or not 0 <= values[3] <= 3:
        raise ValueError("invalid sequence or ready mask")
    if any(not -0x800000 <= value <= 0x7FFFFF for value in values[4:]):
        raise ValueError("raw reading out of signed 24-bit range")
    return values


class CaptureWindow:
    """Keep serial reception independent of the slower display refresh."""

    def __init__(self, root):
        self.root = root
        root.title("RM3100 磁场监测")
        root.geometry("960x620")
        root.minsize(900, 580)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.worker = None
        self.closing = False
        self.sample = None
        self.every = DISPLAY_EVERY_N_FRAMES
        self.state = {}
        self.reset_state()
        self.last_draw = 0.0
        self.port = tk.StringVar(value=SERIAL_PORT)
        self.baud = tk.StringVar(value=str(BAUD_RATE))
        self.output = tk.StringVar(value=OUTPUT_CSV)
        self.every_text = tk.StringVar(value=str(DISPLAY_EVERY_N_FRAMES))
        self.interval = tk.StringVar(value=str(DISPLAY_INTERVAL_MS))
        self.axes = {axis: tk.BooleanVar(value=True) for axis in "XYZ"}
        self.status = tk.StringVar(value="未连接")
        self.counts = tk.StringVar()
        self.note = tk.StringVar()
        self.file_label = tk.StringVar()
        self.reading_info = tk.StringVar(value="等待数据")
        self.values = {(2, axis): tk.StringVar(value="--") for axis in "XYZB"}
        self.build_ui()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self.poll)

    def reset_state(self):
        self.sample = None
        self.state = dict(good=0, bad=0, resets=0, gaps=0, duplicates=0,
                          status="未连接", note="", init_debug=[], path="", last_rx=None)

    def build_ui(self):
        style = ttk.Style()
        style.configure("TLabel", font=("Microsoft YaHei UI", 10))
        outer = ttk.Frame(self.root, padding=16)
        outer.pack(fill="both", expand=True)
        connection = ttk.Frame(outer)
        connection.pack(fill="x")
        ttk.Label(connection, text="串口").pack(side="left")
        ports = [p.device for p in list_ports.comports()]
        self.port_box = ttk.Combobox(connection, textvariable=self.port, values=ports, width=12)
        self.port_box.pack(side="left", padx=8)
        ttk.Button(connection, text="刷新端口", command=self.refresh_ports).pack(side="left")
        ttk.Label(connection, text="波特率").pack(side="left", padx=(20, 6))
        self.baud_box = ttk.Entry(connection, textvariable=self.baud, width=10)
        self.baud_box.pack(side="left")
        self.start_button = ttk.Button(connection, text="开始采集", command=self.start)
        self.start_button.pack(side="right")
        self.stop_button = ttk.Button(connection, text="停止", command=self.stop, state="disabled")
        self.stop_button.pack(side="right", padx=8)

        files = ttk.Frame(outer)
        files.pack(fill="x", pady=14)
        ttk.Label(files, text="CSV 路径").pack(side="left")
        self.output_box = ttk.Entry(files, textvariable=self.output)
        self.output_box.pack(side="left", fill="x", expand=True, padx=8)
        self.browse_button = ttk.Button(files, text="选择文件", command=self.browse)
        self.browse_button.pack(side="right")

        options = ttk.Frame(outer)
        options.pack(fill="x", pady=(0, 10))
        ttk.Label(options, text="读数单位：µT").pack(side="left", padx=(0, 20))
        for axis in "XYZ":
            ttk.Checkbutton(options, text=axis, variable=self.axes[axis],
                            command=self.draw).pack(side="left", padx=6)

        timing = ttk.Frame(outer)
        timing.pack(fill="x", pady=(0, 14))
        ttk.Label(timing, text="每 N 帧抽取").pack(side="left")
        ttk.Spinbox(timing, from_=1, to=10000, textvariable=self.every_text,
                    width=7).pack(side="left", padx=8)
        ttk.Label(timing, text="显示间隔 (ms)").pack(side="left", padx=(16, 0))
        ttk.Spinbox(timing, from_=50, to=10000, increment=50,
                    textvariable=self.interval, width=7).pack(side="left", padx=8)

        readings = ttk.Frame(outer, padding=(0, 16))
        readings.pack(fill="both", expand=True)
        ttk.Label(readings, text="传感器2", anchor="center").grid(
            row=0, column=0, padx=(0, 12))
        for col, axis in enumerate("XYZB", start=1):
            readings.columnconfigure(col, weight=1, uniform="readings")
            ttk.Label(readings, text=axis if axis != "B" else "模长 |B|",
                      anchor="center").grid(row=0, column=col, sticky="ew")
            ttk.Label(readings, textvariable=self.values[(2, axis)], anchor="center",
                      font=("Consolas", 23, "bold"), width=11).grid(
                          row=1, column=col, sticky="ew", pady=12)
        ttk.Label(outer, textvariable=self.reading_info).pack(anchor="w", pady=(0, 8))
        for variable in (self.status, self.counts, self.note, self.file_label):
            ttk.Label(outer, textvariable=variable, wraplength=800).pack(anchor="w", pady=3)

    def refresh_ports(self):
        self.port_box.configure(values=[p.device for p in list_ports.comports()])

    def browse(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                           filetypes=[("CSV", "*.csv")])
        if path:
            self.output.set(path)

    def start(self):
        if self.worker and self.worker.is_alive():
            return
        try:
            baud = int(self.baud.get())
            every, _ = self.display_settings()
            port = self.port.get().strip()
            if baud <= 0 or not port:
                raise ValueError
        except ValueError:
            self.status.set("请填写有效串口、波特率、抽取帧数和显示间隔。")
            return
        name = self.output.get().strip() or f"rm3100_{dt.datetime.now():%Y%m%d_%H%M%S_%f}.csv"
        path = Path(name).expanduser()
        if not path.is_absolute():
            path = Path(__file__).resolve().parent / path
        with self.lock:
            self.reset_state()
            self.every = every
            self.state.update(status="正在连接…", path=str(path))
        self.stop_event.clear()
        self.last_draw = 0.0
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        for widget in (self.port_box, self.baud_box, self.output_box, self.browse_button):
            widget.configure(state="disabled")
        self.draw()
        self.worker = threading.Thread(target=self.capture, args=(port, baud, path))
        self.worker.start()

    def display_settings(self):
        every, interval = int(self.every_text.get()), int(self.interval.get())
        if not 1 <= every <= 10000 or not 50 <= interval <= 10000:
            raise ValueError
        return every, interval

    def capture(self, port_name, baud, path):
        previous = None
        sample = None
        last_flush = time.monotonic()
        try:
            # Exclusive creation protects a previous capture from being overwritten.
            with serial.Serial(port_name, baud, timeout=0.2) as port, \
                    path.open("x", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(HEADER)
                file.flush()
                with self.lock:
                    self.state["status"] = f"已连接 {port_name}，{baud} baud"
                pending = bytearray()
                while not self.stop_event.is_set():
                    chunk = port.read_until(b"\n")
                    pending.extend(chunk)
                    if len(pending) > 1024:
                        pending.clear()
                        with self.lock:
                            self.state["bad"] += 1
                        continue
                    if not pending.endswith(b"\n"):
                        continue
                    line = bytes(pending)
                    pending.clear()
                    if line.startswith((b"BOOT,", b"DBG,", b"ERR,")):
                        message = line.decode("ascii", errors="replace").strip()
                        with self.lock:
                            if message.startswith("DBG,"):
                                self.state["init_debug"].append(message)
                                self.state["note"] = " | ".join(self.state["init_debug"])
                            elif message.startswith("ERR,"):
                                previous_debug = " | ".join(self.state["init_debug"])
                                self.state["note"] = f"{previous_debug} | {message}" if previous_debug else message
                            else:
                                self.state["note"] = message
                        continue
                    if not line.startswith(b"RM,"):
                        continue
                    try:
                        frame = parse_frame(line)
                    except (ValueError, UnicodeDecodeError):
                        with self.lock:
                            self.state["bad"] += 1
                        continue
                    converted = [value / GAIN_LSB_PER_UT if frame[3] & (1 << (i // 3))
                                 else "" for i, value in enumerate(frame[4:])]
                    writer.writerow(frame + converted)
                    now = time.monotonic()
                    sample = (frame, now)
                    with self.lock:
                        state = self.state
                        state["good"] += 1
                        state["last_rx"] = now
                        if previous is not None:
                            delta = (frame[0] - previous[0]) & 0xFFFFFFFF
                            if delta == 0:
                                state["note"] = f"重复序号：{frame[0]}"
                            elif frame[0] < previous[0] and delta > 0x7FFFFFFF:
                                state["resets"] += 1
                                state["note"] = f"序号回退：{previous[0]} → {frame[0]}"
                            elif delta != 1:
                                state["gaps"] += 1
                                state["note"] = f"序号跳变：{previous[0]} → {frame[0]}"
                            if frame[3] == previous[3] == 2 and frame[7:10] == previous[7:10]:
                                state["duplicates"] += 1
                        if state["good"] == 1 or state["good"] % self.every == 0:
                            self.sample = sample
                    previous = frame
                    if now - last_flush >= 1.0:
                        file.flush()
                        last_flush = now
            with self.lock:
                self.state["status"] = "已停止，CSV 已保存"
        except (OSError, ValueError) as exc:
            with self.lock:
                self.state["status"] = f"采集失败：{exc}"
        finally:
            with self.lock:
                if sample is not None:
                    self.sample = sample

    def draw(self):
        with self.lock:
            sample = self.sample
        for value in self.values.values():
            value.set("--")
        if sample is None:
            self.reading_info.set("传感器2 · µT · 等待数据")
            return
        frame, received = sample
        ready = bool(frame[3] & 0x02)
        raw = frame[7:10]
        values = [value / GAIN_LSB_PER_UT for value in raw]
        if ready:
            for axis, value in zip("XYZ", values):
                if self.axes[axis].get():
                    self.values[(2, axis)].set(f"{value:.3f}")
            self.values[(2, "B")].set(f"{math.sqrt(sum(v*v for v in values)):.3f}")
        self.reading_info.set(
            f"传感器2{'已就绪' if ready else '未就绪'} · µT · 显示帧 {frame[0]} · "
            f"距收到该帧 {time.monotonic()-received:.1f} 秒")

    def poll(self):
        if self.closing:
            if self.worker and self.worker.is_alive():
                self.root.after(50, self.poll)
            else:
                self.root.destroy()
            return
        try:
            every, interval = self.display_settings()
            with self.lock:
                self.every = every
        except ValueError:
            interval = DISPLAY_INTERVAL_MS
        with self.lock:
            state = self.state.copy()
        now = time.monotonic()
        status = state["status"]
        running = bool(self.worker and self.worker.is_alive())
        if running and state["last_rx"] is not None and now-state["last_rx"] > 2:
            status += f" · 已 {now-state['last_rx']:.1f} 秒未收到有效帧"
        self.status.set(status)
        self.counts.set(f"保存 {state['good']} 帧 | 异常 {state['bad']} | 复位 {state['resets']} | "
                        f"跳号 {state['gaps']} | 连续重复向量 {state['duplicates']}")
        self.note.set(state["note"])
        self.file_label.set(state["path"])
        if now - self.last_draw >= interval / 1000:
            self.draw()
            self.last_draw = now
        if not running:
            self.start_button.configure(state="normal")
            self.stop_button.configure(state="disabled")
            self.port_box.configure(state="normal")
            for widget in (self.baud_box, self.output_box, self.browse_button):
                widget.configure(state="normal")
        self.root.after(50, self.poll)

    def stop(self):
        self.stop_event.set()
        self.stop_button.configure(state="disabled")

    def close(self):
        self.closing = True
        self.stop_event.set()
        self.root.title("正在保存并关闭…")


def run_gui():
    root = tk.Tk()
    CaptureWindow(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(run_gui())


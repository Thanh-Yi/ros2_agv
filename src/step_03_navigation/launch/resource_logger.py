#!/usr/bin/env python3
import psutil
import time
from datetime import datetime

log_file = "system_usage.log"

def log_system_usage():
    with open(log_file, "a") as f:
        while True:
            cpu_percent = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage("/")
            now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            log_line = (
                f"[{now}] CPU: {cpu_percent}% | "
                f"RAM: {memory.percent}% ({memory.used // (1024**2)} MB / {memory.total // (1024**2)} MB) | "
                f"Disk: {disk.percent}% ({disk.used // (1024**3)} GB / {disk.total // (1024**3)} GB)\n"
            )

            print(log_line, end="")
            f.write(log_line)
            f.flush()

if __name__ == "__main__":
    log_system_usage()

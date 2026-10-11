"""Read-only source inspection. Does not build firmware or run models."""
import json
import re
import subprocess
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
repo = Path(sys.argv[1])
base = "archive/Desktop/대학교오오/3학뇬/2학기/임시설/"
paths = [
    base + "project3_video/Z_project3/Core/Src/main.c",
    base + "project3_stm32/Z_project3/Core/Src/main.c",
    "archive/Documents/카카오톡 받은 파일/project3_stm32/Z_project3/Core/Src/main.c",
    base + "emb_pj3_spi2/Core/Src/main.c",
    base + "emb_pj3_camera/emb_pj3_camera.sdk/cam_RTOS/src/freertos_hello_world.c",
    "archive/Desktop/잡/visitsssss/pj3/src/main.c",
]
for path in paths:
    local = repo / path
    if not local.is_file():
        print("NOT CHECKED OUT:", path)
        continue
    raw = local.read_bytes()
    text = raw.decode("utf-8", errors="replace")
    print("\nPATH", path, "bytes", len(raw))
    for line in text.splitlines():
        if re.search(r"FRAME_WIDTH|FRAME_HEIGHT|FRAME_SIZE|CHUNK_SIZE|RxTask|AITask|SaveTask|Camera.Task|gBusy|CMD_INFO|CMD_CHUNK|OWNER_TEMPLATE|tinycnn|^#include|xTaskCreate|^static.*Task", line):
            print(line[:180])


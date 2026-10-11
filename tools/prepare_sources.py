"""Select archived P3 sources and record provenance; never build or train."""
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

sys.stdout.reconfigure(encoding="utf-8")
archive = Path(sys.argv[1]).resolve()
portfolio = Path(__file__).resolve().parents[1]
base = "archive/Desktop/대학교오오/3학뇬/2학기/임시설/"
stm = base + "project3_video/Z_project3/"
zynq = base + "emb_pj3_camera/"
training = base + "3class/제출/"
selections = []

for filename in ("main.c", "stm32f1xx_hal_msp.c", "stm32f1xx_it.c"):
    selections.append((stm + "Core/Src/" + filename,
                       "source/stm32-bringup/Core/Src/" + filename,
                       "camera/UART bring-up; not final DMA/READY protocol"))
for filename in ("main.h", "stm32f1xx_hal_conf.h", "stm32f1xx_it.h"):
    selections.append((stm + "Core/Inc/" + filename,
                       "source/stm32-bringup/Core/Inc/" + filename,
                       "headers of archived bring-up project"))
for filename in ("Z_project3.ioc", "STM32F103RBTX_FLASH.ld"):
    selections.append((stm + filename, "source/stm32-bringup/" + filename,
                       "configuration of archived bring-up project"))
selections.append((stm + "Drivers/STM32F1xx_HAL_Driver/LICENSE.txt",
                   "source/stm32-bringup/LICENSE.txt", "original vendor license notice"))
selections += [
    (base + "emb_pj3_spi2/Core/Src/main.c",
     "source/spi-diagnostic/main.c", "two-byte SPI2 interrupt/error/rearm diagnostic; no camera or final protocol"),
    (zynq + "emb_pj3_camera.sdk/cam_RTOS/src/freertos_hello_world.c",
     "source/zynq-camera-experiment/freertos_hello_world.c", "Camera_Task experiment; not final three-task pipeline"),
    (zynq + "emb_pj3_camera.sdk/cam_RTOS_bsp/ps7_cortexa9_0/include/FreeRTOSConfig.h",
     "source/zynq-camera-experiment/FreeRTOSConfig.h", "original experiment BSP configuration; Idle Hook disabled"),
    (zynq + "emb_pj3_camera.sdk/cam_RTOS_bsp/system.mss",
     "source/zynq-camera-experiment/system.mss", "original experiment BSP manifest"),
    (zynq + "emb_pj3_camera.srcs/sources_1/bd/design_1/design_1.bd",
     "source/hardware/design_1.bd", "archived Vivado 2025.2 block design; two EMIO GPIO signals"),
    (zynq + "emb_pj3_camera.srcs/constrs_1/new/arducam.xdc",
     "source/hardware/arducam.xdc", "archived experiment constraints; not final four-GPIO mapping"),
]
for filename in ("train_cnn_3class.py", "export_tinycnn3_to_header.py", "tinycnn3_weights.h"):
    selections.append((training + filename, "source/model/" + filename,
                       "submitted model source; no checkpoint or facial dataset copied"))

records = []
for origin, destination, meaning in selections:
    source = archive / origin
    target = (portfolio / destination).resolve()
    assert target.is_relative_to(portfolio)
    if not source.is_file():
        records.append({"source": origin, "destination": destination, "status": "not exported", "meaning": meaning})
        continue
    raw = source.read_bytes()
    try:
        contents = raw.decode("utf-8")
        encoding = "utf-8"
    except UnicodeDecodeError:
        contents = raw.decode("cp949")
        encoding = "cp949"
    output = contents.encode("utf-8")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(output)
    records.append({"source": origin, "destination": destination, "status": "included",
                    "meaning": meaning, "source_encoding": encoding,
                    "transformation": "none" if encoding == "utf-8" else "CP949 text converted to UTF-8; code unchanged",
                    "source_sha256": hashlib.sha256(raw).hexdigest(),
                    "output_sha256": hashlib.sha256(output).hexdigest(), "bytes": len(output)})

def inspect(path):
    raw = (archive / path).read_bytes()
    return {"path": path, "sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw)}

compared = []
for path in [stm + "Core/Src/main.c",
             base + "project3_stm32/Z_project3/Core/Src/main.c",
             "archive/Documents/카카오톡 받은 파일/project3_stm32/Z_project3/Core/Src/main.c",
             base + "emb_pj3_spi2/Core/Src/main.c",
             "archive/Desktop/잡/visitsssss/pj3/src/main.c"]:
    if (archive / path).is_file():
        compared.append(inspect(path))

weights = (portfolio / "source/model/tinycnn3_weights.h").read_text(encoding="utf-8")
arrays = {}
for name, declared, values in re.findall(r"static const float (\w+)\[(\d+)\] = \{(.*?)\};", weights, re.S):
    count = len([v for v in values.split(",") if v.strip()])
    assert count == int(declared), (name, declared, count)
    arrays[name] = count
duplicates = {}
model_filenames = ("train_cnn_3class.py", "export_tinycnn3_to_header.py", "tinycnn3_weights.h")
tree_paths = [base + "3class/" + f for f in model_filenames] + [training + f for f in model_filenames]
tree_result = subprocess.run(["git", "-c", "core.quotepath=false", "-C", str(archive),
                              "ls-tree", "-r", "HEAD", "--", *tree_paths],
                             capture_output=True, text=True, encoding="utf-8", check=True)
blob_ids = {line.split("\t", 1)[1]: line.split("\t", 1)[0].split()[2]
            for line in tree_result.stdout.splitlines()}
for filename in model_filenames:
    a, b = archive / training / filename, archive / base / "3class" / filename
    first, second = blob_ids.get(training + filename), blob_ids.get(base + "3class/" + filename)
    if first and second:
        duplicates[filename] = {"submission_git_blob": first, "parent_git_blob": second,
                                "byte_identical": first == second,
                                "method": "Compare Git blob IDs in archived commit; no checkpoint deserialization."}

document = {"archive": "jounget5411-lab/portfolio-source-archive",
            "archive_commit": "c8618b1caef9cdf128ed20f62692d63818d9dd61",
            "selection_policy": "Preserve selected project source/config, separate experiments from report reference; omit facial data, checkpoints, personal evaluations and generated build outputs.",
            "files": records, "compared_not_all_published": compared,
            "model_array_counts": arrays, "model_parameter_count": sum(arrays.values()),
            "duplicate_model_comparison": duplicates,
            "validation": "Read-only comparison and source selection; no training, firmware build or device execution."}
(portfolio / "source/provenance.json").write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({"included": sum(r["status"] == "included" for r in records),
                  "pending": [r["source"] for r in records if r["status"] != "included"],
                  "model_parameters": sum(arrays.values()), "model_duplicates": duplicates}, ensure_ascii=False, indent=2))

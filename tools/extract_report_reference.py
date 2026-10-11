"""Extract appendix C reference text from a supplied local P3 report.

No PDF, personal-evaluation page, face data or images are copied.
The output is reference text, not a complete buildable firmware package.
"""
import hashlib
import json
import re
import sys
from pathlib import Path
from pypdf import PdfReader

sys.stdout.reconfigure(encoding="utf-8")
pdf = Path(sys.argv[1]).resolve()
root = Path(__file__).resolve().parents[1]
reader = PdfReader(pdf)

if "--preview" in sys.argv:
    for n in range(67, 111):
        text = reader.pages[n-1].extract_text(extraction_mode="layout")
        print(n, repr(text.strip()[:110]))
    raise SystemExit(0)

def extract(name, first, last, start_pattern, end_pattern=None):
    chunks = []
    for n in range(first, last+1):
        text = reader.pages[n-1].extract_text(extraction_mode="layout")
        # Page numbers occupy an isolated line in the original report.
        text = re.sub(r"(?m)^\s*\d+\s*$", "", text)
        chunks.append(f"\n/* Source report physical page {n} */\n" + text.strip())
    text = "\n".join(chunks)
    match = re.search(start_pattern, text)
    if not match:
        raise ValueError(f"Start marker absent: {name}")
    text = text[match.start():]
    if end_pattern:
        end = re.search(end_pattern, text)
        if end:
            text = text[:end.start()]
    header = (
        "/* Reference restoration from the original P3 report appendix.\n"
        f" * Physical pages {first}-{last}. Layout/page labels cleaned only.\n"
        " * Not the archived source file; not compiled or hardware-retested.\n"
        " * See docs/code-map.md and docs/reproduction.md for version/dependency gaps.\n"
        " */\n\n"
    )
    target = root / "source" / "report-reference" / name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(header + text.strip() + "\n", encoding="utf-8")
    return {"path": target.relative_to(root).as_posix(), "source": "P3 original report appendix", "physical_pages": [first,last], "sha256": hashlib.sha256(target.read_bytes()).hexdigest(), "status": "PDF appendix reference restoration; not compiled"}

entries = [
    extract("stm32_main.c", 70, 94, r"/\*\s*USER\s+CODE\s+BEGIN\s+Header"),
    extract("zynq_main.c", 95, 109, r"#include\s*<stdio\.h>"),
]
manifest = {"report_sha256": hashlib.sha256(pdf.read_bytes()).hexdigest(), "outputs": entries}
(root / "source" / "report-reference" / "provenance.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+"\n", encoding="utf-8")
print(json.dumps(manifest, ensure_ascii=False, indent=2))


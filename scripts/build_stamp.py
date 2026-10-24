"""Refresh one tiny compilation unit's firmware stamp on each build."""
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo

Import("env")

generated = Path(env.subst("$BUILD_DIR")) / "generated"
env.Append(CPPPATH=[str(generated)])

if not env.IsCleanTarget():
    # Use the deployment's Central time and include milliseconds so closely
    # spaced builds can still be distinguished. Both consumers use one string.
    now = datetime.now(ZoneInfo("America/Chicago"))
    stamp = now.strftime("FW %b %d %Y %H:%M:%S") + f".{now.microsecond // 1000:03d} {now.tzname()}"
    generated.mkdir(parents=True, exist_ok=True)
    (generated / "build_stamp.h").write_text(
        "#pragma once\n#define FW_BUILD_STAMP " + json.dumps(stamp) + "\n",
        encoding="utf-8",
    )

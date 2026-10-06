"""Container health check: probe the API's own health endpoint.

Reads the same configuration as the server so the probe always matches the
deployment: ``APP_PORT`` (container port) and ``API_HEALTH_PATH``.
"""

from __future__ import annotations

import os
import sys
import urllib.request


def main() -> int:
    port = os.environ.get("APP_PORT", "8000")
    path = os.environ.get("API_HEALTH_PATH", "/api/health")
    url = f"http://127.0.0.1:{port}{path}"
    try:
        with urllib.request.urlopen(url, timeout=3) as resp:
            return 0 if resp.status == 200 else 1
    except Exception:
        return 1


if __name__ == "__main__":
    sys.exit(main())

from __future__ import annotations

import json
from pathlib import Path

from .models import AppConfig


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = PROJECT_ROOT / "config.json"


def load_config(path: Path = CONFIG_PATH) -> AppConfig:
    if not path.exists():
        config = AppConfig()
        save_config(config, path)
        return config

    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    return AppConfig.from_dict(payload)


def save_config(config: AppConfig, path: Path = CONFIG_PATH) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(config.to_dict(), handle, indent=2)


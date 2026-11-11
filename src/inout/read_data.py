import json
from pathlib import Path

def load_json(path):
    with Path(path).open("r", encoding="utf-8") as f:
        return json.load(f)
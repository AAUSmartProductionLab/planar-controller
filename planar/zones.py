"""Load named table-zone rectangles and speed factors from JSON or URLs."""

import json
import re
import urllib.request
from dataclasses import dataclass

_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]{0,63}")


class ZoneError(ValueError):
    """Invalid zone JSON, coordinates or speed scaling."""
    pass


@dataclass(frozen=True)
class ZoneSpec:
    """A named zone rectangle in metres with its speed factor."""
    name: str
    box: tuple          # (x0, y0, x1, y1), m
    speed: float = 1.0  # the PMC's override of speed and acceleration inside


def load(source):
    """The zones from a file or an http(s) URL. OSError if it cannot be read, ZoneError if it is no zones."""
    if source.startswith(("http://", "https://")):
        with urllib.request.urlopen(source, timeout=10) as response:
            data = response.read()
    else:
        with open(source, "rb") as f:
            data = f.read()
    try:
        return parse(data.decode("utf-8-sig"))
    except UnicodeDecodeError as e:
        raise ZoneError(f"the zones are not UTF-8 text: {e}") from None


def parse(text):
    """Validate zone JSON and convert millimetre coordinates to metre-based ZoneSpec objects."""
    try:
        data = json.loads(text)
    except json.JSONDecodeError as e:
        raise ZoneError(f"the zones are not JSON: {e}") from None
    if not isinstance(data, dict) or set(data) != {"zones"} or not isinstance(data["zones"], list):
        raise ZoneError('the zones must be a JSON object {"zones": [...]}')
    out, names = [], set()
    for k, z in enumerate(data["zones"]):
        what = f"zones[{k}]"
        if not isinstance(z, dict):
            raise ZoneError(f"{what}: an object with name, from, to and perhaps speed")
        unknown = set(z) - {"name", "from", "to", "speed"}
        if unknown:
            raise ZoneError(f"{what}: unknown {', '.join(sorted(unknown))}")
        name = z.get("name")
        if not isinstance(name, str) or not _NAME.fullmatch(name):
            raise ZoneError(f"{what}: name must be a letter, then letters, digits or _ (it is an OPC UA name)")
        if name in names:
            raise ZoneError(f"{what}: a second zone named {name}")
        names.add(name)
        corners = []
        for key in ("from", "to"):
            p = z.get(key)
            if (not isinstance(p, list) or len(p) != 2
                    or not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in p)):
                raise ZoneError(f"{what} ({name}): {key} must be [x, y] in mm")
            corners.append(p)
        (xa, ya), (xb, yb) = corners
        if xa == xb or ya == yb:
            raise ZoneError(f"{what} ({name}): from and to must be opposite corners of a rectangle")
        speed = z.get("speed", 1.0)
        if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not 0 < speed <= 1:
            raise ZoneError(f"{what} ({name}): speed must be more than 0 and at most 1")
        box = (min(xa, xb) / 1000.0, min(ya, yb) / 1000.0, max(xa, xb) / 1000.0, max(ya, yb) / 1000.0)
        out.append(ZoneSpec(name, box, float(speed)))
    return out

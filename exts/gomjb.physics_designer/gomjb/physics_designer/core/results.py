"""Time-series results of a simulation run: channels, CSV export and simple statistics.

Channel names follow ``<quantity>:<item>[:<component>]`` (e.g. ``force:Actuator_L``,
``link_force:Platform:z``) and carry a unit, so a CSV opens directly in Excel / pandas and can
be compared with results from another solver.
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional


@dataclass
class Recording:
    name: str = "recording"
    time: List[float] = field(default_factory=list)
    channels: Dict[str, List[float]] = field(default_factory=dict)
    units: Dict[str, str] = field(default_factory=dict)

    def add_sample(self, t: float, values: Dict[str, float]) -> None:
        n = len(self.time)
        for key in values:
            if key not in self.channels:
                self.channels[key] = [math.nan] * n  # channel appeared later: pad the past
        for key, series in self.channels.items():
            series.append(float(values.get(key, math.nan)))
        self.time.append(float(t))

    def __len__(self) -> int:
        return len(self.time)

    def names(self, prefix: str = "") -> List[str]:
        return [k for k in self.channels if k.startswith(prefix)]

    def at(self, t: float, channel: str) -> float:
        """Value of ``channel`` at the sample closest to time ``t``."""
        if not self.time:
            raise ValueError("empty recording")
        i = min(range(len(self.time)), key=lambda k: abs(self.time[k] - t))
        return self.channels[channel][i]

    def summary(self, channel: str) -> Dict[str, float]:
        vals = [v for v in self.channels[channel] if not math.isnan(v)]
        if not vals:
            return {"min": math.nan, "max": math.nan, "final": math.nan, "abs_max": math.nan}
        return {"min": min(vals), "max": max(vals), "final": vals[-1], "abs_max": max(abs(v) for v in vals)}

    # -- CSV -----------------------------------------------------------------------------
    def to_csv(self, path: str, channels: Optional[Iterable[str]] = None) -> None:
        keys = list(channels) if channels is not None else list(self.channels)
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time [s]"] + [f"{k} [{self.units.get(k, '')}]" for k in keys])
            for i, t in enumerate(self.time):
                w.writerow([repr(t)] + [repr(self.channels[k][i]) for k in keys])

    @classmethod
    def from_csv(cls, path: str, name: Optional[str] = None) -> "Recording":
        rec = cls(name=name or path)
        with open(path, newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        header = rows[0]
        keys = []
        for col in header[1:]:
            key, _, unit = col.rpartition(" [")
            keys.append(key)
            rec.units[key] = unit.rstrip("]")
        for row in rows[1:]:
            rec.add_sample(float(row[0]), {k: float(v) for k, v in zip(keys, row[1:])})
        return rec

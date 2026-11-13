import math

import pytest

from gomjb.physics_designer.core.results import Recording


def test_samples_csv_roundtrip(tmp_path):
    rec = Recording("lift")
    rec.units.update({"force:Actuator_L": "N", "pos:Actuator_L": "m"})
    for i in range(5):
        rec.add_sample(i * 0.1, {"force:Actuator_L": -100.0 - i, "pos:Actuator_L": -0.01 * i})
    rec.add_sample(0.5, {"force:Actuator_L": -105.0, "pos:Actuator_L": -0.05, "force:Late": 1.0})
    assert len(rec) == 6
    assert math.isnan(rec.channels["force:Late"][0])  # padded when a channel appears later
    assert rec.names("force:") == ["force:Actuator_L", "force:Late"]
    assert rec.at(0.21, "force:Actuator_L") == -102.0
    assert rec.summary("force:Actuator_L") == {"min": -105.0, "max": -100.0, "final": -105.0, "abs_max": 105.0}

    path = tmp_path / "lift.csv"
    rec.to_csv(str(path))
    header = path.read_text(encoding="utf-8").splitlines()[0]
    assert header == "time [s],force:Actuator_L [N],pos:Actuator_L [m],force:Late []"
    back = Recording.from_csv(str(path))
    assert back.time == pytest.approx(rec.time)
    assert back.channels["pos:Actuator_L"] == pytest.approx(rec.channels["pos:Actuator_L"])
    assert back.units["force:Actuator_L"] == "N"


def test_empty_recording():
    rec = Recording()
    with pytest.raises(ValueError):
        rec.at(0.0, "x")

"""The rollouts shown in demo04 and demo07 match the verifier's recorded outcome for that seed.

Re-runs the saved attempt-1 and final solution.py files through sim.py (no cache) for the featured
eval route-trial seed and compares with route_<R>_success / route_<R>_collision in artifacts.npz.
"""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

from media import common as C  # noqa: E402

FEATURED_SEED = 2100052  # logged in build/media_cache/selection.json and demo/demo_clips/manifest.json

if not (C.FEATURED / "1" / "artifacts.npz").exists():
    pytest.skip("results/ for the featured run are not present", allow_module_level=True)


def _logged_trial():
    mf = C.CLIPS / "manifest.json"
    if mf.exists():
        sel = json.loads(mf.read_text())["seed_selection"]
        assert sel["seed"] == FEATURED_SEED
    i = FEATURED_SEED - C.route_seed(0)
    assert C.route_seed(i) == FEATURED_SEED
    return C.ROLLOUT_ROUTE, i


@pytest.mark.parametrize("name", ["attempt1", "final"])
def test_demo_rollout_matches_recorded_outcome(name):
    route, i = _logged_trial()
    runner = C._runner(name)
    try:
        ro = C.rollout_one(runner, name, route, i)
    finally:
        runner.close()
    rec = C.recorded_outcome(name, route, i)
    assert ro["outcome"]["success"] == rec["success"]
    assert ro["outcome"]["collision"] == rec["collision"]
    mf = C.CLIPS / "manifest.json"
    if mf.exists():
        clips = json.loads(mf.read_text())["clips"]
        label = {"COLLISION": (False, True), "ARRIVED": (True, False)}
        if name == "attempt1":
            shown = clips["demo04_rollout_attempt1_overconfident"]["numbers_shown"]["outcome"]
            assert label.get(shown, (False, False)) == (rec["success"], rec["collision"])
        shown = clips["demo07_before_after_same_seed"]["numbers_shown"][name]["outcome"]
        assert label.get(shown, (False, False)) == (rec["success"], rec["collision"])

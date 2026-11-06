import pytest

from simcheck import harness


def test_extract_code_picks_last_valid_block():
    text = ("first try\n```python\ndef make_estimator(s):\n    return 1\n```\n"
            "broken\n```python\ndef make_estimator(s)\n```\n"
            "helper\n```python\nx = 1\n```\n")
    assert harness.extract_code(text).strip() == "def make_estimator(s):\n    return 1"


def test_extract_code_rejects_missing():
    with pytest.raises(ValueError):
        harness.extract_code("no code here")


def test_cost_cap(tmp_path):
    c = harness.CostTracker(1.0, tmp_path / "ledger.jsonl")
    c.add(0.6, "a"); c.check()
    c.add(0.5, "b")
    with pytest.raises(harness.CostCapReached):
        c.check()
    assert harness.CostTracker(1.0, tmp_path / "ledger.jsonl").spent == pytest.approx(1.1)  # survives restart


def test_price_requires_configured_prices():
    with pytest.raises(RuntimeError):
        harness.price({"model": "m", "price_per_mtok": {"input": None, "output": None}},
                      {"input_tokens": 1, "output_tokens": 1})
    assert harness.price({"model": "m", "price_per_mtok": {"input": 2.0, "output": 10.0}},
                         {"input_tokens": 1_000_000, "output_tokens": 100_000}) == pytest.approx(3.0)


def test_revision_prompt_carries_code_and_feedback():
    p = harness.build_prompt("standard", "CODE_X", ["fix calibration"], attempt=2)
    assert "CODE_X" in p and "- fix calibration" in p and "Revision 1 of 2" in p
    assert "truth.json" not in p and '"poses"' not in p  # nothing from hidden/ reaches the prompt


def test_fake_sources_are_self_contained():
    for v in ("reference", "cov_x10", "always_A"):
        src = harness.fake_source(v)
        assert "from simcheck" not in src.split("def choose_route(public_spec):\n    return")[-1]
        compile(src, v, "exec")


# ---- added with the sandbox/harness integration

GOOD = "def make_estimator(s):\n    return None\n\n\ndef choose_route(s):\n    return {}\n"


def test_extract_code_prefers_block_with_both_functions():
    # A later block that only defines make_estimator (e.g. a partial patch) must not win over the full file.
    text = f"full:\n```python\n{GOOD}```\npatch:\n```python\ndef make_estimator(s):\n    return 2\n```\n"
    assert harness.extract_code(text) == GOOD


def test_extract_code_untagged_and_py_fences():
    assert harness.extract_code(f"```\n{GOOD}```") == GOOD
    assert harness.extract_code(f"```py\n{GOOD}```") == GOOD


def test_extract_code_skips_unparseable_full_block():
    text = f"```python\n{GOOD}```\n```python\ndef make_estimator(:\ndef choose_route(s): pass\n```"
    assert harness.extract_code(text) == GOOD


def test_cost_reservation_blocks_concurrent_overshoot(tmp_path):
    c = harness.CostTracker(1.0, tmp_path / "ledger.jsonl")
    h1 = c.reserve(0.6)            # first call in flight
    with pytest.raises(harness.CostCapReached):
        c.reserve(0.6)             # second concurrent call could push spend past the cap
    c.release(h1); c.add(0.3, "a")  # first call actually cost 0.3
    c.release(c.reserve(0.6))      # now 0.3 + 0.6 <= 1.0 fits
    assert c.reserved == pytest.approx(0.0)


def test_worst_case_usd_uses_output_limit():
    cfg = {"model": "m", "settings": {"max_completion_tokens": 10_000}, "price_per_mtok": {"input": 0.0, "output": 50.0}}
    assert harness.worst_case_usd(cfg, "", "") == pytest.approx(0.5)
    cfg["settings"] = {"maxOutputTokens": 20_000}
    assert harness.worst_case_usd(cfg, "", "") == pytest.approx(1.0)


def test_unpriced_model_refused_before_any_call(tmp_path, monkeypatch):
    called = []
    monkeypatch.setattr(harness, "call_model", lambda *a, **k: called.append(1))
    monkeypatch.setitem(harness.MODELS, "unpriced", {"provider": "openai", "model": "x", "settings": {},
                                                    "price_per_mtok": {"input": None, "output": None}})
    monkeypatch.setattr(harness, "ROOT", tmp_path)
    monkeypatch.setattr(harness, "build_prompt", lambda *a, **k: "prompt")
    with pytest.raises(RuntimeError):
        harness.run_job("unpriced", "standard", harness.CostTracker(10, tmp_path / "l.jsonl"), echo=lambda *_: None)
    assert called == []  # previously the paid call happened first and pricing failed afterwards


def test_prompt_states_conventions_agents_are_graded_on():
    # Fails if scenarios/ on disk predates the task.md conventions: delete scenarios/ to regenerate.
    p = harness.build_prompt("standard")
    assert "not in the world frame" in p                    # covariance frame (calibration depends on it)
    assert "sampled from the stated prior" in p             # x_0 is not known exactly
    assert "always in SI units" in p                        # noise std units under unit traps
    assert "Grading:" in p and "7.81" not in p and "0.25" not in p.split("## spec.json")[0]   # no thresholds


def test_syntax_error_is_explained():
    with pytest.raises(ValueError, match=r"does not parse: .* at line 3"):
        harness.extract_code("```python\ndef make_estimator(s):\n    pass\ndef _ x(): pass\n```")


def test_feedback_echo_flags_pasted_measurements_only():
    fb = ["Route: stated success probability 0.70 differs from the measured rate 0.505 "
          "(95% CI [0.436, 0.574]) by more than 0.15."]
    assert harness.feedback_echo("cal = 0.505 / 0.70\ntol = 0.15", "p = 0.70", fb) == ["0.505"]
    assert harness.feedback_echo("p = 0.5", "p = 0.7", fb) == []
    assert harness.feedback_echo("x = 0.505", None, None) == []

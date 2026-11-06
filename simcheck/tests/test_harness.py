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


def test_syntax_error_is_explained():
    with pytest.raises(ValueError, match=r"does not parse: .* at line 3"):
        harness.extract_code("```python\ndef make_estimator(s):\n    pass\ndef _ x(): pass\n```")


def test_feedback_echo_flags_pasted_measurements_only():
    fb = ["Route: stated success probability 0.70 differs from the measured rate 0.505 "
          "(95% CI [0.436, 0.574]) by more than 0.15."]
    assert harness.feedback_echo("cal = 0.505 / 0.70\ntol = 0.15", "p = 0.70", fb) == ["0.505"]
    assert harness.feedback_echo("p = 0.5", "p = 0.7", fb) == []
    assert harness.feedback_echo("x = 0.505", None, None) == []

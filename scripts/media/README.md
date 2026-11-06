# Deck stills and demo clips

Offline: numbers come from `results/summary/summary.json` and the saved run folders; rollouts replay the
saved `solution.py` files through `simcheck/sim.py` (Docker if available, else the subprocess fallback;
the runner is recorded in `demo/demo_clips/manifest.json`). No model API calls.

Setup (fonts and the ffmpeg binary are not in git):

```bash
pip install matplotlib imageio-ffmpeg
mkdir -p assets/fonts
for w in Regular Medium SemiBold Bold; do curl -sLo assets/fonts/IBMPlexSans-$w.ttf https://raw.githubusercontent.com/IBM/plex/master/packages/plex-sans/fonts/complete/ttf/IBMPlexSans-$w.ttf; done
for w in Regular Medium SemiBold; do curl -sLo assets/fonts/IBMPlexMono-$w.ttf https://raw.githubusercontent.com/google/fonts/main/ofl/ibmplexmono/IBMPlexMono-$w.ttf; done
```

Render:

```bash
.venv/bin/python scripts/media/common.py   # replay 2x200 route-A trials, pick + log the featured seed
.venv/bin/python scripts/media/deck.py     # demo/deck_assets/*.png + deck_numbers.json
.venv/bin/python scripts/media/clips.py    # demo/demo_clips/*.mp4 + manifest.json
.venv/bin/python -m pytest simcheck/tests/test_media_rollouts.py
```

Featured run: `results/v3.2/argon-medium/r2/unknown_noise/`, attempt 1 vs attempt 2 (final).
Seed choice: see `seed_selection` in either JSON (rule, refinement, unrefined pick, RNG seed).

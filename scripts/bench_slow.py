"""Worst-case proxy for an agent estimator: batch LM + Marginals over the whole graph every step."""
import sys, time
from pathlib import Path
import gtsam, numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from simcheck import generator, reference, sim  # noqa: E402

class Batch(reference.Estimator):
    def __init__(self, spec):
        super().__init__(spec); self.g = gtsam.NonlinearFactorGraph(); self.v = gtsam.Values()
        self.isam = self  # capture factors instead of incremental update
    def update(self, g=None, v=None):
        if g is None: return
        self.g.push_back(g); self.v.insert(v)
        self.v = gtsam.LevenbergMarquardtOptimizer(self.g, self.v).optimize()
        self.k = max(gtsam.Symbol(k).index() for k in self.v.keys())
    def calculateEstimatePose2(self, key): return self.v.atPose2(key)
    def marginalCovariance(self, key): return gtsam.Marginals(self.g, self.v).marginalCovariance(key)

spec = generator.build_spec(generator.ScenarioConfig())
for r in "AB":
    t0 = time.perf_counter(); n = 5; steps = 0
    for i in range(n):
        ep = sim.run_episode(spec, r, 1_000_000 + i, estimator=Batch(spec)); steps += ep["outcome"]["steps"] + 1
    print(r, f"{(time.perf_counter()-t0)/n:.2f} s/episode, {1e3*(time.perf_counter()-t0)/steps:.1f} ms/step")

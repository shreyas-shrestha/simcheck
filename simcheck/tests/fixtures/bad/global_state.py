_COUNT = [0]
def make_estimator(spec):
    _COUNT[0] += 1
    return E(_COUNT[0])
class E:
    def __init__(self, n): self.n = n
    def step(self, m): return [float(self.n), 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
def choose_route(spec): return {"route": "A", "success_prob": 0.5, "reasoning": ""}

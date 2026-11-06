def make_estimator(spec):
    return E()


class E:
    def step(self, m):
        with open("/tmp/fill", "wb") as f:   # 100 MB into a 64 MB tmpfs must fail
            for _ in range(100):
                f.write(b"\0" * (1 << 20))
        return [0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def choose_route(spec):
    return {}

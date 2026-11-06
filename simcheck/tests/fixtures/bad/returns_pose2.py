import gtsam


def make_estimator(spec):
    return E()


class E:
    def step(self, m):
        return gtsam.Pose2(0, 0, 0), [[1, 0, 0], [0, 1, 0], [0, 0, 1]]   # violates the contract (pose must be a list)


def choose_route(spec):
    return {}

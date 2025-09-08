import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional, List, Tuple


@dataclass
class RRTConfig:
    step_size: float = 0.05
    max_iter: int = 5000
    goal_threshold: float = 0.03
    goal_sample_rate: float = 0.1  # probability to sample goal


class RRTPlanner:
    def __init__(
        self,
        dim: int,
        lower: np.ndarray,
        upper: np.ndarray,
        collision_fn: Optional[Callable[[np.ndarray], bool]] = None,
        config: RRTConfig = RRTConfig(),
    ):
        self.dim = dim
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        self.collision_fn = collision_fn
        self.cfg = config

        self.nodes: List[np.ndarray] = []
        self.parents: List[int] = []

    def _sample(self, goal: np.ndarray) -> np.ndarray:
        if np.random.rand() < self.cfg.goal_sample_rate:
            return goal.copy()
        return self.lower + np.random.rand(self.dim) * (self.upper - self.lower)

    @staticmethod
    def _dist(a: np.ndarray, b: np.ndarray) -> float:
        return float(np.linalg.norm(a - b))

    def _nearest(self, q: np.ndarray) -> int:
        dists = [self._dist(n, q) for n in self.nodes]
        return int(np.argmin(dists))

    def _steer(self, q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
        v = q_to - q_from
        d = np.linalg.norm(v)
        if d <= self.cfg.step_size:
            q_new = q_to
        else:
            q_new = q_from + (v / d) * self.cfg.step_size
        return np.clip(q_new, self.lower, self.upper)

    def _collision_free(self, q: np.ndarray) -> bool:
        if self.collision_fn is None:
            return True
        return self.collision_fn(q)

    def plan(self, start: np.ndarray, goal: np.ndarray) -> Optional[List[np.ndarray]]:
        self.nodes = [np.asarray(start, dtype=float)]
        self.parents = [-1]

        for _ in range(self.cfg.max_iter):
            q_rand = self._sample(goal)
            idx = self._nearest(q_rand)
            q_near = self.nodes[idx]
            q_new = self._steer(q_near, q_rand)
            if not self._collision_free(q_new):
                continue
            self.nodes.append(q_new)
            self.parents.append(idx)
            if self._dist(q_new, goal) < self.cfg.goal_threshold:
                # connect to goal
                self.nodes.append(goal.copy())
                self.parents.append(len(self.nodes) - 2)
                return self._reconstruct(len(self.nodes) - 1)
        return None

    def _reconstruct(self, idx: int) -> List[np.ndarray]:
        path: List[np.ndarray] = []
        while idx != -1:
            path.append(self.nodes[idx])
            idx = self.parents[idx]
        path.reverse()
        return path

    @staticmethod
    def interpolate(path: List[np.ndarray], max_step: float) -> List[np.ndarray]:
        if not path:
            return []
        dense: List[np.ndarray] = [path[0]]
        for i in range(1, len(path)):
            a, b = path[i - 1], path[i]
            dist = np.linalg.norm(b - a)
            n = max(int(np.ceil(dist / max_step)), 1)
            for k in range(1, n + 1):
                t = k / n
                dense.append((1 - t) * a + t * b)
        return dense


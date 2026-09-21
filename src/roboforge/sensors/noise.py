"""Named RNG streams, independent of sensor order and other channels' draws."""

import hashlib
import math

import numpy as np

from roboforge.config import NoiseConfig
from roboforge.core import finite, nonnegative


def generator(seed: int, name: str) -> np.random.Generator:
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    digest = hashlib.sha256(f"roboforge-rng-v1:{seed}:{name}".encode()).digest()
    return np.random.Generator(np.random.PCG64(int.from_bytes(digest, "big")))


class NoiseProcess:
    """y=x+b+walk+N(0,sigma); walk increments N(0,q*sqrt(dt))."""

    def __init__(self, config: NoiseConfig, seed: int, name: str):
        self.config = config
        self.white = generator(seed, name + ":white")
        self.walk = generator(seed, name + ":walk")
        self.dropout = generator(seed, name + ":dropout")
        self.bias = config.bias

    def sample(self, value: float, dt: float) -> float | None:
        value, dt = finite(value, "measurement"), nonnegative(dt, "noise interval")
        self.bias += float(self.walk.normal(0, self.config.bias_walk * math.sqrt(dt)))
        noisy = value + self.bias + float(self.white.normal(0, self.config.stddev))
        return (
            None
            if self.dropout.random() < self.config.dropout
            else finite(noisy, "noisy measurement")
        )

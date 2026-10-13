"""Fixed-size NaN-initialised ring buffer.

Used by the residual generator for rolling-mean and rolling-variance
channels (NIS, MPC time, IMU freeze, encoder freeze). Direct port of
the ``ringPush`` / ``buf.*_idx`` / ``buf.*_n`` idiom in
computeResiduals.m.
"""

from __future__ import annotations

from typing import Optional
import numpy as np


class RingBuffer:
    """Wrap-around fixed-window buffer.

    ``values()`` returns only the populated slice (length ``n``).
    NaNs that somehow got pushed in are kept; consumers are expected
    to filter them out with ``np.isnan`` the way the MATLAB code does.
    """

    __slots__ = ("W", "_buf", "_idx", "_n")

    def __init__(self, W: int) -> None:
        if W < 1:
            raise ValueError("RingBuffer window must be >= 1")
        self.W    = int(W)
        self._buf = np.full(self.W, np.nan, dtype=float)
        self._idx = 0
        self._n   = 0

    # ------------------------------------------------------------------
    def push(self, value: float) -> None:
        self._idx = self._idx % self.W
        self._buf[self._idx] = float(value)
        self._idx += 1
        if self._n < self.W:
            self._n += 1

    # ------------------------------------------------------------------
    @property
    def n(self) -> int:
        return self._n

    def values(self) -> np.ndarray:
        """Populated portion (chronological order is not preserved)."""
        return self._buf[: self._n].copy()

    def mean(self, omit_nan: bool = True) -> Optional[float]:
        if self._n == 0:
            return None
        v = self.values()
        if omit_nan:
            v = v[~np.isnan(v)]
            if v.size == 0:
                return None
        return float(np.mean(v))

    def var(self, omit_nan: bool = True) -> Optional[float]:
        if self._n < 2:
            return None
        v = self.values()
        if omit_nan:
            v = v[~np.isnan(v)]
            if v.size < 2:
                return None
        # MATLAB var() uses N-1 by default — match that.
        return float(np.var(v, ddof=1))

    def reset(self) -> None:
        self._buf[:] = np.nan
        self._idx = 0
        self._n   = 0

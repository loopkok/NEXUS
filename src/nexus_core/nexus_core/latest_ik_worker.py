"""One in-flight solve and one replaceable pending target, with no ROS calls."""
from __future__ import annotations
import threading
import time


class LatestIKWorker:
    def __init__(self, solver):
        self.solver = solver
        self._condition = threading.Condition()
        self._pending = None
        self._result = None
        self._closed = False
        self._thread = threading.Thread(target=self._run, name='nero-ik-latest', daemon=True)
        self._thread.start()

    def submit(self, generation, target, seed=None):
        with self._condition:
            if seed is None and self._pending is not None and self._pending[0] == generation:
                seed = self._pending[2]
            self._pending = (generation, target.copy(), None if seed is None else seed.copy())
            self._condition.notify()

    def take(self):
        with self._condition:
            result, self._result = self._result, None
            return result

    def close(self):
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify()
        self._thread.join(timeout=2.)

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                generation, target, seed = self._pending
                self._pending = None
            started = time.monotonic()
            try:
                if seed is not None:
                    self.solver.sync_state(seed)
                q = self.solver.solve(target)
                report = dict(self.solver.last_report)
            except Exception as exc:
                q, report = None, {'reason': 'solver_exception', 'detail': str(exc)}
            finished = time.monotonic()
            with self._condition:
                self._result = (generation, target, q, report, finished, (finished-started)*1000.)

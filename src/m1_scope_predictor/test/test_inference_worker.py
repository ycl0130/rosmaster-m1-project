import sys
import time
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from m1_scope_predictor.inference_worker import InferenceWorker  # noqa: E402
from m1_scope_predictor.latest_mailbox import LatestMailbox  # noqa: E402


class Backend:
    def infer_sequence(self, input_ogm, horizon_steps, num_samples):
        value = float(input_ogm[0, 0, 0, 0])
        grid = np.full((horizon_steps, 1, 64, 64), value, dtype=np.float32)
        return grid, np.zeros_like(grid), 0.01, 123


class Job:
    def __init__(self, value):
        self.input_ogm = np.full((10, 1, 64, 64), value, dtype=np.float32)


def wait_result(mailbox, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = mailbox.take()
        if result is not None:
            return result
        time.sleep(0.01)
    raise AssertionError("worker did not return a result")


def test_worker_returns_backend_result_without_blocking_caller():
    output = LatestMailbox()
    worker = InferenceWorker(Backend(), output, horizon_steps=5, num_samples=4)
    worker.start()
    worker.submit(Job(0.25))
    result = wait_result(output)
    worker.stop()
    assert result.error is None
    assert result.job.input_ogm[0, 0, 0, 0] == 0.25
    assert result.means[0, 0, 0, 0] == 0.25
    assert result.memory_bytes == 123


def test_worker_converts_backend_exception_to_error_result():
    class BrokenBackend:
        def infer_sequence(self, *args, **kwargs):
            raise RuntimeError("inference failed")

    output = LatestMailbox()
    worker = InferenceWorker(BrokenBackend(), output, 5, 4)
    worker.start()
    worker.submit(Job(0.0))
    result = wait_result(output)
    worker.stop()
    assert isinstance(result.error, RuntimeError)
    assert result.means is None

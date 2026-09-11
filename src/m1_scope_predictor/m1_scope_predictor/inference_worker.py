"""Dedicated latest-only inference worker."""

from dataclasses import dataclass
import threading

from .latest_mailbox import LatestMailbox


@dataclass(frozen=True)
class InferenceResult:
    job: object
    means: object = None
    standard_deviations: object = None
    latency_seconds: float = 0.0
    memory_bytes: int = 0
    preprocess_seconds: float = 0.0
    inference_seconds: float = 0.0
    postprocess_seconds: float = 0.0
    error: Exception = None


class InferenceWorker:
    def __init__(self, backend, output_mailbox, horizon_steps, num_samples):
        self.backend = backend
        self.output = output_mailbox
        self.horizon_steps = int(horizon_steps)
        self.num_samples = int(num_samples)
        self.input = LatestMailbox()
        self._thread = threading.Thread(
            target=self._run, name="scope-inference", daemon=True)

    @property
    def dropped(self):
        return self.input.dropped

    def start(self):
        self._thread.start()

    def submit(self, job):
        return self.input.put(job)

    def stop(self):
        self.input.close()
        if self._thread.is_alive():
            self._thread.join(timeout=5.0)

    def _run(self):
        while True:
            job = self.input.get()
            if job is None:
                return
            try:
                means, standard_deviations, latency, memory = self.backend.infer_sequence(
                    job.input_ogm, self.horizon_steps, self.num_samples)
                result = InferenceResult(
                    job=job, means=means,
                    standard_deviations=standard_deviations,
                    latency_seconds=float(latency), memory_bytes=int(memory),
                    preprocess_seconds=float(getattr(self.backend, "last_preprocess_seconds", 0.0)),
                    inference_seconds=float(getattr(self.backend, "last_inference_seconds", latency)),
                    postprocess_seconds=float(getattr(self.backend, "last_postprocess_seconds", 0.0)))
            except Exception as error:  # keep failures out of the ROS executor
                result = InferenceResult(job=job, error=error)
            self.output.put(result)

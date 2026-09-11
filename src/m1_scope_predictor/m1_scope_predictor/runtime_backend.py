"""PyTorch-independent validation plus lazy-loaded SCOPE inference."""

import importlib
import hashlib
from pathlib import Path
import sys
import time

import numpy as np


MODEL_INPUT_SHAPE = (10, 1, 64, 64)
CHECKPOINT_SHA256 = "0eb7d348530670e0547c24e91df583540930c89668937723a4e7161d6fab7d92"
VENDOR_SHA256 = {
    "model.py": "a7b404a2909bcc8e563e3a70520bf4398957fab98f85d80eab4fdf70e24906f6",
    "convlstm.py": "eb350d8d2297a2a2f7e652500aeabaa5c92ab6b3bbd9d0a77d516f0718b114e2",
}


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_frozen_assets(checkpoint):
    vendor = Path(__file__).parent / "third_party" / "templerail_scope"
    for name, expected in VENDOR_SHA256.items():
        if _sha256(vendor / name) != expected:
            raise ValueError("vendored %s SHA-256 does not match upstream" % name)
    if _sha256(checkpoint) != CHECKPOINT_SHA256:
        raise ValueError("checkpoint SHA-256 does not match frozen SCOPE weights")


def validate_model_input(value):
    """Validate the exact tensor contract consumed by the official model."""
    if not isinstance(value, np.ndarray) or value.shape != MODEL_INPUT_SHAPE:
        raise ValueError("model input must have shape [10,1,64,64]")
    if value.dtype != np.float32:
        raise ValueError("model input must use float32")
    if not np.isfinite(value).all():
        raise ValueError("model input must contain only finite values")
    return value


def aggregate_samples(samples):
    """Return pixel-wise Monte Carlo mean and population standard deviation."""
    samples = np.asarray(samples, dtype=np.float32)
    if samples.ndim != 4 or samples.shape[1:] != (1, 64, 64):
        raise ValueError("samples must have shape [N,1,64,64]")
    return samples.mean(axis=0), samples.std(axis=0)


def aggregate_sequence_samples(samples_by_step):
    """Aggregate independent MC samples without mixing autoregressive time."""
    samples = np.asarray(samples_by_step, dtype=np.float32)
    if samples.ndim != 5 or samples.shape[2:] != (1, 64, 64):
        raise ValueError("sequence samples must have shape [T,N,1,64,64]")
    return samples.mean(axis=1), samples.std(axis=1)


class ScopeRuntimeBackend:
    """Load the frozen model lazily and run autoregressive MC inference."""

    def __init__(self, checkpoint, device="cuda", seed=1337):
        import torch

        self.torch = torch
        verify_frozen_assets(checkpoint)
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but torch.cuda.is_available() is false")
        self.device = torch.device(device)
        self.seed = int(seed)
        vendor = Path(__file__).parent / "third_party" / "templerail_scope"
        sys.path.insert(0, str(vendor))
        try:
            official = importlib.import_module("model")
        finally:
            sys.path.pop(0)
        self.model = official.scope(input_channels=1, latent_dim=512, output_channels=1)
        self.model.to(self.device)
        state = torch.load(str(checkpoint), map_location=self.device, weights_only=False)
        self.model.load_state_dict(state["model"], strict=True)
        self.model.eval()

    def infer_sequence(self, input_ogm, horizon_steps=5, num_samples=4):
        preprocess_started = time.perf_counter()
        input_ogm = validate_model_input(input_ogm)
        if int(horizon_steps) < 1 or int(num_samples) < 1:
            raise ValueError("horizon_steps and num_samples must be positive")
        torch = self.torch
        torch.manual_seed(self.seed)
        if self.device.type == "cuda":
            torch.cuda.manual_seed_all(self.seed)
        inputs = torch.from_numpy(input_ogm).unsqueeze(0).to(self.device)
        inputs = inputs.repeat(int(num_samples), 1, 1, 1, 1)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        self.last_preprocess_seconds = time.perf_counter() - preprocess_started
        started = time.perf_counter()
        predictions = []
        with torch.inference_mode():
            for _ in range(int(horizon_steps)):
                prediction, _ = self.model(inputs)
                predictions.append(prediction)
                inputs = torch.cat((inputs[:, 1:], prediction.unsqueeze(1)), dim=1)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        self.last_inference_seconds = time.perf_counter() - started
        postprocess_started = time.perf_counter()
        samples = torch.stack(predictions, dim=0).detach().cpu().numpy().astype(np.float32)
        mean, standard_deviation = aggregate_sequence_samples(samples)
        memory = (int(torch.cuda.memory_allocated(self.device))
                  if self.device.type == "cuda" else 0)
        self.last_postprocess_seconds = time.perf_counter() - postprocess_started
        latency = (self.last_preprocess_seconds + self.last_inference_seconds +
                   self.last_postprocess_seconds)
        return mean, standard_deviation, latency, memory

    def infer(self, input_ogm, horizon_steps=5, num_samples=4):
        """Legacy endpoint API retained for parity tooling and callers."""
        mean, standard_deviation, latency, memory = self.infer_sequence(
            input_ogm, horizon_steps, num_samples)
        return mean[-1], standard_deviation[-1], latency, memory

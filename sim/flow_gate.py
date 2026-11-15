"""Runtime C gate configuration. Fault labels never enter this interface."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FlowGateConfig:
    enabled: int = 0
    threshold: float = 9.
    max_reject: int | None = None  # Resolve the default from C, not a second copy.

    def __post_init__(self):
        if self.enabled not in (0, 1):
            raise ValueError("Flow gate must be 0 or 1")
        if not math.isfinite(self.threshold) or self.threshold <= 0:
            raise ValueError("Flow gate threshold must be positive and finite")
        if self.max_reject is not None and (not isinstance(self.max_reject, int) or not 1 <= self.max_reject <= 65535):
            raise ValueError("Flow gate max rejections must be an integer in [1,65535]")

    def apply(self, cf, logging=False):
        # Supports recording pre-gate stock references before rebuilding C.
        if not hasattr(cf, "kalmanCoreSetFlowGateConfig"):
            if self.enabled or logging:
                raise RuntimeError("Rebuild cffirmware for flow gate support")
            return {"enabled": 0, "implementation": "pre-gate stock bindings"}
        limit = self.max_reject if self.max_reject is not None else cf.kalmanCoreGetFlowGateDefaultMaxReject()
        if not cf.kalmanCoreSetFlowGateConfig(self.enabled, self.threshold, limit):
            raise ValueError("C rejected flow gate configuration")
        return dict(enabled=cf.kalmanCoreGetFlowGateEnabled(), threshold=cf.kalmanCoreGetFlowGateThreshold(),
                    max_reject=cf.kalmanCoreGetFlowGateMaxReject(), logging=logging,
                    fallback="accept Nth consecutive above-threshold sample, reset streak")


def add_gate_arguments(parser):
    parser.add_argument("--flow-gate", type=int, choices=(0, 1), default=0)
    parser.add_argument("--flow-gate-threshold", type=float, default=9.)
    parser.add_argument("--flow-gate-max-reject", type=int, help="Force accept on the Nth consecutive rejection candidate; default from C")
    parser.add_argument("--flow-gate-log", action="store_true", help="Separate per-axis gate CSV; automatic with --flow-gate 1")


def gate_from_args(args):
    return FlowGateConfig(args.flow_gate, args.flow_gate_threshold, args.flow_gate_max_reject)

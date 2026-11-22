"""Default-off synthetic Flow faults, configured in firmware-aligned counts.

No estimator decisions belong here. Labels are for offline scoring only.
"""

from dataclasses import asdict, dataclass
import math

import numpy as np


@dataclass(frozen=True)
class FlowFaultConfig:
    kind: str = "off"
    seed: int | None = None
    start_s: float = 20.2
    duration_s: float = 1.0
    spike_amplitude: float = 20.0
    spike_probability: float = .1
    scale_factor: float = .5

    def __post_init__(self):
        if self.kind not in ("off", "spike", "zero", "dropout", "scale"):
            raise ValueError("Unknown flow fault")
        if self.seed is not None and (self.seed < 0 or int(self.seed) != self.seed):
            raise ValueError("Flow fault seed must be a nonnegative integer")
        for name in ("start_s", "duration_s", "spike_amplitude", "spike_probability", "scale_factor"):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite")
        if self.start_s < 0 or self.duration_s <= 0:
            raise ValueError("Fault start must be nonnegative and duration positive")
        if not 0 < self.spike_amplitude < 100 or not 0 <= self.spike_probability <= 1:
            raise ValueError("Spike amplitude must be in (0,100), probability in [0,1]")
        if self.scale_factor <= 0:
            raise ValueError("Scale factor must be positive")

    def settings(self, sensor_seed):
        return dict(asdict(self), resolved_seed=sensor_seed if self.seed is None else self.seed,
                    end_s=self.start_s+self.duration_s,
                    interval="[start_s,end_s), sample time", rng_domain=0x464C4F57)


class FlowFault:
    def __init__(self, config, sensor_seed):
        self.config = config
        seed = sensor_seed if config.seed is None else config.seed
        self.rng = np.random.default_rng(np.random.SeedSequence([seed, 0x464C4F57]))

    def apply(self, sample_ms, ideal, clean, noise):
        c = self.config
        window = c.kind != "off" and c.start_s*1000 <= sample_ms < (c.start_s+c.duration_s)*1000
        active = window
        counts, status, clipped = clean.copy(), 0xB0, False
        if window:
            if c.kind == "spike":
                # Consume the same schedule/sign draws regardless of truth or clipping.
                active = bool(self.rng.random() < c.spike_probability)
                signs = self.rng.choice((-1., 1.), size=2)
                if active:
                    candidate = clean + signs*c.spike_amplitude
                    counts = np.clip(candidate, -99., 99.)
                    clipped = bool(np.any(counts != candidate))
            elif c.kind == "zero":
                counts = np.zeros(2)
            elif c.kind == "dropout":
                status = 0x30  # Synthetic hypothesis: 0xB0 with motion bit cleared.
            elif c.kind == "scale":
                # Change only ideal flow. Keep the already drawn noise unchanged.
                counts = c.scale_factor*ideal + noise
        labels = {"flow_sample_id": sample_ms//10, "fault_type": c.kind,
                  "fault_window_active": bool(window), "fault_active": bool(active),
                  "fault_changed_x": bool(counts[0] != clean[0]),
                  "fault_changed_y": bool(counts[1] != clean[1])}
        return counts, status, clipped, labels


def add_flow_arguments(parser, sweep=False):
    parser.add_argument("--trajectory", choices=("hover-move", "flow-shuttle-x", "flow-shuttle-y"), default="hover-move")
    parser.add_argument("--flow-audit", action="store_true", help="Separate sample/update CSVs; original CSV schema is unchanged with faults off")
    parser.add_argument("--flow-fault", choices=("off", "spike", "zero", "dropout", "scale"), default="off")
    parser.add_argument("--flow-fault-seed", type=int, help="Defaults to --seed, in a separate RNG domain")
    parser.add_argument("--flow-fault-phase", choices=("takeoff", "hover", "move"), default="move")
    parser.add_argument("--flow-fault-offset", type=float, default=.2)
    parser.add_argument("--flow-fault-duration", type=float, default=1.)
    parser.add_argument("--flow-spike-amplitude", type=float, default=20.)
    parser.add_argument("--flow-spike-probability", type=float, default=.1)
    parser.add_argument("--flow-scale-factor", type=float, default=.5)
    if sweep:
        parser.add_argument("--flow-spike-amplitudes", type=float, nargs="+", help="Spike sweep, e.g. 5 10 20 50; separate matched runs")


def config_from_args(args, hover_hold_s=10., amplitude=None):
    if not math.isfinite(args.flow_fault_offset) or args.flow_fault_offset < 0:
        raise ValueError("Flow fault offset must be finite and nonnegative")
    # Shuttle has a fixed 10 s hover, independent of the GUI demo's duration.
    hover = hover_hold_s if args.trajectory == "hover-move" else 10.
    start = {"takeoff": 0., "hover": 10., "move": 10.+hover}[args.flow_fault_phase]
    return FlowFaultConfig(args.flow_fault, args.flow_fault_seed, start+args.flow_fault_offset,
                           args.flow_fault_duration,
                           args.flow_spike_amplitude if amplitude is None else amplitude,
                           args.flow_spike_probability, args.flow_scale_factor)

"""Audit logging-only NIS runs against preserved stock CSVs and plot each case.

Example (Isaac Python, no SimulationApp):
  python sim/plot_flow_nis.py --input-dir build/step1_nis_seed7/ekf \
      --reference-dir build/step0_stock_33g_seed7/ekf

Phases come from the flight CSV at measurement delivery time. The final
``complete`` sample is included in move, so all 3000 flow samples are counted.
No samples (including startup or large NIS) are discarded. Statistics are
empirical, with NumPy's linear percentile interpolation.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from plot_ekf import load


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compare_csvs(current, reference):
    """Require byte-identical flight files and exact original measurement fields."""
    result = {"flight_sha256": sha256(current),
              "reference_flight_sha256": sha256(reference)}
    result["flight_byte_identical"] = result["flight_sha256"] == result["reference_flight_sha256"]
    with current.with_suffix('.measurements.csv').open() as new_stream, \
            reference.with_suffix('.measurements.csv').open() as old_stream:
        new_reader, old_reader = csv.DictReader(new_stream), csv.DictReader(old_stream)
        result['added_measurement_columns'] = [c for c in new_reader.fieldnames if c not in old_reader.fieldnames]
        old_fields = old_reader.fieldnames
        new_rows, old_rows = list(new_reader), list(old_reader)
        result['measurement_rows'] = len(new_rows)
        result['reference_measurement_rows'] = len(old_rows)
        result['original_measurement_fields_identical'] = (
            set(old_fields).issubset(new_reader.fieldnames)
            and len(new_rows) == len(old_rows)
            and all(all(a[k] == b[k] for k in old_fields) for a, b in zip(new_rows, old_rows)))
    return result


def statistics(values):
    if not len(values) or not np.isfinite(values).all() or np.any(values < 0):
        raise ValueError('Expected nonempty, finite, nonnegative flow NIS values')
    return {'count': len(values), 'mean': float(np.mean(values)),
            'median': float(np.median(values)),
            'p95': float(np.percentile(values, 95)),
            'p99': float(np.percentile(values, 99)),
            'p99_9': float(np.percentile(values, 99.9)),
            'fraction_above_6_63': float(np.mean(values > 6.63)),
            'fraction_above_9': float(np.mean(values > 9.0))}


def analyze_case(flight_path, output_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    flight = load(flight_path)
    measurements = load(flight_path.with_suffix('.measurements.csv'))
    flow = measurements[measurements['sensor'] == 'flow']
    indices = np.searchsorted(flight['time_s'], flow['time_s'], side='right') - 1
    if np.any(indices < 0) or np.any(flow['time_s'] > flight['time_s'][-1]):
        raise ValueError('Measurements outside the logged flight interval')
    phases = flight['phase'][indices].copy()
    phases[phases == 'complete'] = 'move'
    if not np.isin(phases, ['takeoff', 'hover', 'move']).all():
        raise ValueError('Unexpected demo phase')
    settings = json.loads(flight_path.with_suffix('.json').read_text())
    rows = []
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True)
    for column, axis in enumerate('xy'):
        all_values = flow[f'nis_{axis}']
        statistics(all_values)  # Validate every sample, including startup.
        for phase in ('all', 'takeoff', 'hover', 'move'):
            values = all_values if phase == 'all' else all_values[phases == phase]
            row = {'case': flight_path.stem, 'axis': axis, 'phase': phase, **statistics(values)}
            rows.append(row)
            if phase == 'all':
                continue
            ax = axes[('takeoff', 'hover', 'move').index(phase), column]
            if settings['settings']['noise']:
                bins = np.linspace(0, max(10., float(np.max(all_values))*1.05), 81)
            else:
                bins = np.r_[0., np.geomspace(1e-16, max(10., float(np.max(all_values))*1.05), 81)]
                # Match the first bin's width so exact zeros remain visible.
                ax.set_xscale('symlog', linthresh=1e-16)
                ax.set_xticks([0., 1e-15, 1e-12, 1e-9, 1e-6, 1e-3, 1., 10.])
            bin_counts, _, _ = ax.hist(values, bins=bins, color='#336699', alpha=.85)
            ax.axvline(6.63, color='#b44b36', linestyle='--', label='6.63')
            ax.axvline(9., color='#743487', linestyle=':', label='9.0')
            ax.set_yscale('log')
            ax.set_ylim(.8, max(2., float(np.max(bin_counts))*1.5))
            ax.set(title=f'{phase} / {axis.upper()}: n={len(values)}, mean={row["mean"]:.5g}',
                   xlabel='Flow scalar NIS (dimensionless)', ylabel='Sample count (log scale)')
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
    fig.suptitle(f'{flight_path.stem}: stock EKF + logging, seed {settings["settings"]["seed"]}')
    fig.savefig(output_dir / f'{flight_path.stem}.histogram.png', dpi=160)
    plt.close(fig)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, required=True)
    parser.add_argument('--reference-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    output_dir = args.output_dir or args.input_dir.parent / 'nis'
    output_dir.mkdir(parents=True, exist_ok=True)
    rows, comparison = [], {}
    for mode in ('ekf_shadow', 'ekf_closed_loop'):
        for noise in ('off', 'on'):
            name = f'{mode}_noise_{noise}.csv'
            flight_path = args.input_dir / name
            comparison[flight_path.stem] = compare_csvs(flight_path, args.reference_dir / name)
            rows.extend(analyze_case(flight_path, output_dir))
    (output_dir / 'comparison.json').write_text(json.dumps(comparison, indent=2)+'\n')
    (output_dir / 'statistics.json').write_text(json.dumps(rows, indent=2)+'\n')
    with (output_dir / 'statistics.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({'comparison': comparison, 'all_phase_statistics': [r for r in rows if r['phase']=='all']}, indent=2))
    if not all(r['flight_byte_identical'] and r['original_measurement_fields_identical']
               and r['added_measurement_columns'] == ['nis_x', 'nis_y'] for r in comparison.values()):
        raise SystemExit('Logging-only regression differs from the stock reference; inspect comparison.json')


if __name__ == '__main__':
    main()

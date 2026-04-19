#!/usr/bin/env python3
"""
plot_results.py
===============
Post-processing script that reads evaluation CSV files and generates
comparison plots for different planner configurations.

Usage:
  python3 plot_results.py eval_results/summary.csv
  python3 plot_results.py results_apf/ results_mpc/ results_rl/ --compare
"""

import argparse
import os
import sys

import numpy as np


def load_csv(filepath):
    """Load a CSV file into a dict of lists."""
    data = {}
    with open(filepath, 'r') as f:
        headers = f.readline().strip().split(',')
        for h in headers:
            data[h] = []
        for line in f:
            vals = line.strip().split(',')
            for h, v in zip(headers, vals):
                try:
                    data[h].append(float(v))
                except ValueError:
                    data[h].append(v)
    return data


def print_summary(data, label=''):
    """Print summary statistics."""
    print(f'\n{"="*60}')
    print(f'  Summary: {label}')
    print(f'{"="*60}')

    n = len(data.get('episode', []))
    print(f'  Episodes:           {n}')

    if 'goal_reached' in data:
        reached = sum(1 for v in data['goal_reached'] if v == 1.0 or v == 'True')
        print(f'  Goal success rate:  {reached}/{n} ({100*reached/max(n,1):.0f}%)')

    for metric, unit in [
        ('path_length_m', 'm'),
        ('time_to_goal_s', 's'),
        ('collision_count', ''),
        ('mode_switches', ''),
        ('replan_freq_per_m', '/m'),
        ('path_efficiency', ''),
        ('min_obstacle_dist_m', 'm'),
    ]:
        if metric in data:
            vals = [v for v in data[metric] if isinstance(v, (int, float))]
            if vals:
                mean = np.mean(vals)
                std = np.std(vals)
                print(f'  {metric:25s}: {mean:8.3f} ± {std:.3f} {unit}')

    print(f'{"="*60}\n')


def plot_comparison(datasets, labels):
    """Generate comparison bar plots."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed. Skipping plots.')
        print('Install with: pip install matplotlib')
        return

    metrics = ['path_length_m', 'time_to_goal_s', 'collision_count',
               'mode_switches', 'path_efficiency']
    metric_labels = ['Path Length (m)', 'Time to Goal (s)', 'Collisions',
                     'Mode Switches', 'Path Efficiency']

    fig, axes = plt.subplots(1, len(metrics), figsize=(4 * len(metrics), 5))
    if len(metrics) == 1:
        axes = [axes]

    colors = ['#2196F3', '#FF5722', '#4CAF50', '#9C27B0']

    for ax, metric, label in zip(axes, metrics, metric_labels):
        means = []
        stds = []
        for data in datasets:
            if metric in data:
                vals = [v for v in data[metric] if isinstance(v, (int, float))]
                means.append(np.mean(vals) if vals else 0)
                stds.append(np.std(vals) if vals else 0)
            else:
                means.append(0)
                stds.append(0)

        x = np.arange(len(labels))
        bars = ax.bar(x, means, yerr=stds, color=colors[:len(labels)],
                      capsize=5, alpha=0.85, edgecolor='black', linewidth=0.5)
        ax.set_xlabel('')
        ax.set_ylabel(label)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=15)
        ax.grid(axis='y', alpha=0.3)

    fig.suptitle('Path Planner Comparison', fontsize=14, fontweight='bold')
    fig.tight_layout()

    output_path = 'planner_comparison.png'
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'Plot saved to: {output_path}')
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Evaluate and plot path planning results')
    parser.add_argument('paths', nargs='+',
                        help='CSV files or directories containing summary.csv')
    parser.add_argument('--compare', action='store_true',
                        help='Generate comparison plots')
    parser.add_argument('--labels', nargs='*',
                        help='Labels for each dataset')
    args = parser.parse_args()

    datasets = []
    labels = []

    for i, path in enumerate(args.paths):
        if os.path.isdir(path):
            csv_path = os.path.join(path, 'summary.csv')
            label = os.path.basename(path.rstrip('/'))
        else:
            csv_path = path
            label = os.path.splitext(os.path.basename(path))[0]

        if not os.path.exists(csv_path):
            print(f'WARNING: {csv_path} not found, skipping')
            continue

        data = load_csv(csv_path)
        datasets.append(data)

        if args.labels and i < len(args.labels):
            labels.append(args.labels[i])
        else:
            labels.append(label)

    if not datasets:
        print('No data files found!')
        sys.exit(1)

    for data, label in zip(datasets, labels):
        print_summary(data, label)

    if args.compare and len(datasets) > 1:
        plot_comparison(datasets, labels)


if __name__ == '__main__':
    main()

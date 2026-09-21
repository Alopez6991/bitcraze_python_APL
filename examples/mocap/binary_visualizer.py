"""Visualize numeric data stored in a binary file.

This script supports two modes:
1. Raw binary arrays decoded as a chosen NumPy dtype.
2. Crazyflie SD logs with a ``fixedFrequency`` TOC/header.

Examples:
    python binary_visualizer.py my_log.bin
    python binary_visualizer.py my_log.bin --format flaplog
    python binary_visualizer.py my_log.bin --signals motor.m1 motor.m2 motor.m3 motor.m4
    python binary_visualizer.py my_log.bin --dtype float32 --columns 4
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


DTYPE_MAP = {
    'int8': np.int8,
    'uint8': np.uint8,
    'int16': np.int16,
    'uint16': np.uint16,
    'int32': np.int32,
    'uint32': np.uint32,
    'int64': np.int64,
    'uint64': np.uint64,
    'float32': np.float32,
    'float64': np.float64,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Plot a raw binary file as typed numeric samples.',
    )
    parser.add_argument('file', type=Path, help='Path to the binary file to visualize')
    parser.add_argument(
        '--format',
        choices=['auto', 'raw', 'flaplog'],
        default='auto',
        help='Input format. auto detects Crazyflie SD logs, default: %(default)s',
    )
    parser.add_argument(
        '--dtype',
        choices=sorted(DTYPE_MAP.keys()),
        default='float32',
        help='Numeric type used to decode the file, default: %(default)s',
    )
    parser.add_argument(
        '--endian',
        choices=['little', 'big', 'native'],
        default='little',
        help='Byte order for multi-byte dtypes, default: %(default)s',
    )
    parser.add_argument(
        '--columns',
        type=int,
        default=1,
        help='Number of interleaved channels/columns in the file, default: %(default)s',
    )
    parser.add_argument(
        '--offset',
        type=int,
        default=0,
        help='Byte offset to start reading from, default: %(default)s',
    )
    parser.add_argument(
        '--count',
        type=int,
        default=-1,
        help='Number of scalar values to read after offset, default: all',
    )
    parser.add_argument(
        '--sample-rate',
        type=float,
        default=0.0,
        help='Samples per second for time-axis plotting, default: plot against sample index',
    )
    parser.add_argument(
        '--title',
        default='',
        help='Optional custom plot title',
    )
    parser.add_argument(
        '--save',
        type=Path,
        help='Optional path to save the plot image instead of only showing it',
    )
    parser.add_argument(
        '--column-labels',
        nargs='*',
        default=None,
        help='Optional labels for each plotted column',
    )
    parser.add_argument(
        '--signals',
        nargs='*',
        default=None,
        help='Optional subset of signal names to plot for flaplog input',
    )
    parser.add_argument(
        '--max-plots',
        type=int,
        default=12,
        help='Maximum number of subplots to show at once for flaplog input, default: %(default)s',
    )
    parser.add_argument(
        '--export-csv',
        type=Path,
        help='Optional CSV export path for decoded flaplog input',
    )

    args = parser.parse_args()

    if args.columns < 1:
        parser.error('--columns must be at least 1')
    if args.offset < 0:
        parser.error('--offset cannot be negative')
    if args.count == 0 or args.count < -1:
        parser.error('--count must be positive or -1 for all remaining values')
    if args.sample_rate < 0.0:
        parser.error('--sample-rate cannot be negative')
    if args.column_labels is not None and len(args.column_labels) not in (0, args.columns):
        parser.error('--column-labels must match --columns when provided')
    if args.max_plots < 1:
        parser.error('--max-plots must be at least 1')

    return args


def detect_format(file_path: Path) -> str:
    with file_path.open('rb') as file_handle:
        head = file_handle.read(4096)
    if b'fixedFrequency\x00' in head:
        return 'flaplog'
    return 'raw'


def resolve_dtype(dtype_name: str, endian: str) -> np.dtype:
    dtype = np.dtype(DTYPE_MAP[dtype_name])
    if dtype.itemsize == 1 or endian == 'native':
        return dtype
    if endian == 'little':
        return dtype.newbyteorder('<')
    return dtype.newbyteorder('>')


def parse_flaplog(file_path: Path) -> tuple[np.ndarray, list[str], np.ndarray]:
    data = file_path.read_bytes()

    fixed_freq_pos = data.find(b'fixedFrequency\x00')
    if fixed_freq_pos == -1:
        raise ValueError("Could not find 'fixedFrequency' marker in file")

    pos = fixed_freq_pos + len(b'fixedFrequency\x00')
    num_vars = struct.unpack_from('<H', data, pos)[0]
    pos += 2

    variables = []
    for _ in range(num_vars):
        end = data.index(b'\x00', pos)
        variables.append(data[pos:end].decode('ascii'))
        pos = end + 1

    if data[pos:pos + 2] == b'\xff\xff':
        pos += 2

    names = []
    types = []
    for variable in variables:
        if variable.endswith(')') and '(' in variable:
            type_char = variable[variable.rfind('(') + 1:-1]
            name = variable[:variable.rfind('(')].rstrip(' .')
        else:
            type_char = 'f'
            name = variable
        names.append(name)
        types.append(type_char)

    row_fmt = '<' + ''.join(types)
    row_value_size = struct.calcsize(row_fmt)
    row_size = 4 + row_value_size
    data_section = data[pos:]

    row_offsets = []
    if len(data_section) >= row_size:
        row_offsets.append(0)

    index = 0
    while index < len(data_section) - row_size - 1:
        if data_section[index] == 0xFF and data_section[index + 1] == 0xFF:
            timestamp = struct.unpack_from('<I', data_section, index + 2)[0]
            if 0 < timestamp < 0xFFFFFFFF:
                row_offsets.append(index + 2)
            index += 2
        else:
            index += 1

    timestamps = []
    rows = []
    for row_offset in row_offsets:
        if row_offset + row_size > len(data_section):
            break
        timestamps.append(struct.unpack_from('<I', data_section, row_offset)[0])
        rows.append(struct.unpack_from(row_fmt, data_section, row_offset + 4))

    if not rows:
        raise ValueError('No data rows decoded from flaplog file')

    return np.asarray(timestamps, dtype=np.float64), names, np.asarray(rows, dtype=np.float64)


def load_data(file_path: Path, dtype: np.dtype, offset: int, count: int, columns: int) -> np.ndarray:
    file_size = file_path.stat().st_size
    if offset >= file_size:
        raise ValueError(f'Offset {offset} is beyond file size {file_size}')

    raw = np.fromfile(file_path, dtype=dtype, count=count, offset=offset)
    if raw.size == 0:
        raise ValueError('No samples were read. Check dtype, offset, and count.')

    usable_size = (raw.size // columns) * columns
    if usable_size == 0:
        raise ValueError(
            f'Not enough samples ({raw.size}) to fill one row with {columns} column(s).'
        )
    if usable_size != raw.size:
        print(f'Warning: dropping {raw.size - usable_size} trailing sample(s) to fit {columns} column(s).')
        raw = raw[:usable_size]

    return raw.reshape(-1, columns)


def build_x_axis(num_rows: int, sample_rate: float) -> tuple[np.ndarray, str]:
    if sample_rate > 0.0:
        return np.arange(num_rows) / sample_rate, 'Time [s]'
    return np.arange(num_rows), 'Sample index'


def build_flaplog_time_axis(timestamps_us: np.ndarray) -> tuple[np.ndarray, str]:
    if len(timestamps_us) == 0:
        return np.array([]), 'Time [s]'
    return (timestamps_us - timestamps_us[0]) / 1_000_000.0, 'Time [s]'


def plot_data(data: np.ndarray, x_axis: np.ndarray, x_label: str, args: argparse.Namespace) -> None:
    plt.figure(figsize=(12, 7))
    labels = args.column_labels or [f'col_{index}' for index in range(data.shape[1])]

    for column_index in range(data.shape[1]):
        plt.plot(x_axis, data[:, column_index], label=labels[column_index], linewidth=1.0)

    title = args.title or f'{args.file.name} as {args.dtype} ({data.shape[1]} column(s))'
    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel('Value')
    plt.grid(True, alpha=0.3)
    if data.shape[1] > 1:
        plt.legend()
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150)
        print(f'Saved plot to {args.save}')

    plt.show()


def plot_flaplog(timestamps_us: np.ndarray, names: list[str], data: np.ndarray, args: argparse.Namespace) -> None:
    if args.signals:
        selected_indices = [names.index(signal) for signal in args.signals if signal in names]
        missing = [signal for signal in args.signals if signal not in names]
        if missing:
            print(f'Warning: signals not found and skipped: {missing}')
        if not selected_indices:
            raise ValueError('None of the requested --signals were found in the flaplog file')
    else:
        selected_indices = list(range(min(len(names), args.max_plots)))

    x_axis, x_label = build_flaplog_time_axis(timestamps_us)
    selected_names = [names[index] for index in selected_indices]
    selected_data = data[:, selected_indices]

    num_plots = len(selected_indices)
    fig, axes = plt.subplots(num_plots, 1, figsize=(12, max(3, 2.4 * num_plots)), sharex=True)
    if num_plots == 1:
        axes = [axes]

    for axis, signal_name, signal_values in zip(axes, selected_names, selected_data.T):
        axis.plot(x_axis, signal_values, linewidth=1.0)
        axis.set_ylabel(signal_name)
        axis.grid(True, alpha=0.3)

    axes[-1].set_xlabel(x_label)
    title = args.title or f'{args.file.name} flaplog ({len(names)} signals, showing {num_plots})'
    fig.suptitle(title)
    fig.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150)
        print(f'Saved plot to {args.save}')

    plt.show()


def export_flaplog_csv(csv_path: Path, timestamps_us: np.ndarray, names: list[str], data: np.ndarray) -> None:
    header = 'timestamp_us,' + ','.join(names)
    table = np.column_stack((timestamps_us.astype(np.uint32), data))
    np.savetxt(csv_path, table, delimiter=',', header=header, comments='')
    print(f'Exported CSV to {csv_path}')


def main() -> None:
    args = parse_args()
    if not args.file.is_file():
        raise FileNotFoundError(f'Binary file not found: {args.file}')

    input_format = detect_format(args.file) if args.format == 'auto' else args.format

    if input_format == 'flaplog':
        timestamps_us, names, data = parse_flaplog(args.file)
        print(f'Loaded flaplog with {len(names)} signals and {len(data)} row(s) from {args.file}')
        print('Signals:')
        for name in names:
            print(f'  {name}')
        if args.export_csv:
            export_flaplog_csv(args.export_csv, timestamps_us, names, data)
        plot_flaplog(timestamps_us, names, data, args)
        return

    dtype = resolve_dtype(args.dtype, args.endian)
    data = load_data(args.file, dtype, args.offset, args.count, args.columns)
    x_axis, x_label = build_x_axis(len(data), args.sample_rate)

    print(f'Loaded {data.shape[0]} row(s) x {data.shape[1]} column(s) from {args.file}')
    print(f'dtype={dtype}, min={np.min(data):.6g}, max={np.max(data):.6g}')
    plot_data(data, x_axis, x_label, args)


if __name__ == '__main__':
    main()
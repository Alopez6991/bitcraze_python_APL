import struct
import csv

def parse_sd_log(input_path, output_path):
    with open(input_path, 'rb') as f:
        data = f.read()

    # Parse TOC: find variable names and types
    fixed_freq_pos = data.find(b'fixedFrequency\x00')
    if fixed_freq_pos == -1:
        print("ERROR: Could not find 'fixedFrequency' marker")
        return

    pos = fixed_freq_pos + len(b'fixedFrequency\x00')
    num_vars = struct.unpack_from('<H', data, pos)[0]
    pos += 2
    print(f"Variables found: {num_vars}")

    variables = []
    for _ in range(num_vars):
        end = data.index(b'\x00', pos)
        variables.append(data[pos:end].decode('ascii'))
        pos = end + 1

    if data[pos:pos+2] == b'\xff\xff':
        pos += 2  # skip end-of-TOC marker

    print(f"Data starts at byte offset: {pos}")

    col_names = []
    col_types = []
    for var in variables:
        if var.endswith(')') and '(' in var:
            type_char = var[var.rfind('(') + 1:-1]
            name = var[:var.rfind('(')].rstrip(' .')
        else:
            type_char = 'f'
            name = var
        col_names.append(name)
        col_types.append(type_char)

    var_fmt = '<' + ''.join(col_types)
    var_size = struct.calcsize(var_fmt)
    row_size = 4 + var_size   # uint32 timestamp + variable data
    print(f"Variable format: {var_fmt}  ({var_size} bytes)")

    # The SD card log has: Row1 (row_size bytes) then repeating [FF FF + row_size bytes]
    # Collect all row start positions by scanning for FF FF + plausible timestamp
    data_section = data[pos:]
    rows = []

    # First row (no preceding FF FF separator)
    if len(data_section) >= row_size:
        ts = struct.unpack_from('<I', data_section, 0)[0]
        rows.append(0)

    # Subsequent rows: preceded by FF FF
    i = 0
    while i < len(data_section) - row_size - 1:
        if data_section[i] == 0xFF and data_section[i+1] == 0xFF:
            ts = struct.unpack_from('<I', data_section, i+2)[0]
            if 0 < ts < 0xFFFFFFFF:
                rows.append(i + 2)
            i += 2
        else:
            i += 1

    print(f"Rows found: {len(rows)}")

    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp_us'] + col_names)
        written = 0
        for row_offset in rows:
            if row_offset + row_size > len(data_section):
                break
            ts = struct.unpack_from('<I', data_section, row_offset)[0]
            vals = struct.unpack_from(var_fmt, data_section, row_offset + 4)
            writer.writerow([ts] + list(vals))
            written += 1

    print(f"\nDone. Wrote {written} rows to:\n  {output_path}")

if __name__ == '__main__':
    input_path  = "/media/austin/FlapperSD/flap2log00"
    output_path = "/media/austin/FlapperSD/flap2log00.csv"
    parse_sd_log(input_path, output_path)

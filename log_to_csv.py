import argparse
import csv
import os
import re
import sys

ANSI_ESCAPE_RE = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")
KEY_VALUE_RE = re.compile(r"(\w+)=([+-]?\d+(?:\.\d+)?)")

DEFAULT_FIELDS = ["time", "roll", "pitch"]


def strip_ansi(text):
    return ANSI_ESCAPE_RE.sub("", text)


def parse_log_line(line):
    line = strip_ansi(line)
    values = dict(KEY_VALUE_RE.findall(line))
    if all(field in values for field in DEFAULT_FIELDS):
        return {field: values[field] for field in DEFAULT_FIELDS}
    return None


def convert_log_to_csv(log_path, csv_path, delimiter=","):
    if not os.path.exists(log_path):
        raise FileNotFoundError(f"Log file not found: {log_path}")

    rows = []
    with open(log_path, "r", encoding="utf-8", errors="ignore") as log_file:
        for line in log_file:
            parsed = parse_log_line(line)
            if parsed is not None:
                rows.append(parsed)

    if not rows:
        raise ValueError(f"No matching log entries found in {log_path}")

    with open(csv_path, "w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=DEFAULT_FIELDS, delimiter=delimiter)
        writer.writeheader()
        writer.writerows(rows)

    return len(rows)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert a posture log containing time, roll, and pitch values into CSV."
    )
    parser.add_argument("log_path", help="Path to the input log file")
    parser.add_argument("csv_path", nargs="?", default="output.csv", help="Path to the output CSV file")
    parser.add_argument("--delimiter", "-d", default=",", help="CSV delimiter (default: ',')")
    args = parser.parse_args()

    try:
        count = convert_log_to_csv(args.log_path, args.csv_path, delimiter=args.delimiter)
        print(f"Wrote {count} rows to {args.csv_path}")
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)

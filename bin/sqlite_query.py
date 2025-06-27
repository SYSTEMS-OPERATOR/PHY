#!/usr/bin/env python
from __future__ import annotations

import argparse
import sqlite3
from pathlib import Path


def query(db_path: str, table: str) -> None:
    """Print all rows from a SQLite table."""
    if not Path(db_path).is_file():
        raise FileNotFoundError(f"database {db_path!r} does not exist")
    if not table.isidentifier():
        raise ValueError("table name must be a valid identifier")
    conn = sqlite3.connect(db_path)
    try:
        cur = conn.cursor()
        cur.execute(f"SELECT * FROM {table}")
        for row in cur.fetchall():
            print(row)
    finally:
        conn.close()


def main() -> None:
    ap = argparse.ArgumentParser(description="Query a table in a SQLite database.")
    ap.add_argument("db_path", help="Path to SQLite database file")
    ap.add_argument("table", help="Table name to query")
    args = ap.parse_args()
    query(args.db_path, args.table)


if __name__ == "__main__":
    main()

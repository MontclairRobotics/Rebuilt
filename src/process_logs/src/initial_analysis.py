import polars as pl
import sys

# Map each entrytype to the column that should hold its data.
# Exact matches are checked first; prefix matches (struct:*) are handled
# separately in the analysis functions.
TYPE_TO_COLUMN = {
    "double": "data_double",
    "float": "data_double",
    "int64": "data_int",
    "string": "data_str",
    "json": "data_str",
    "msgpack": "data_str",
    "boolean": "data_bool",
    "boolean[]": "data_str",
    "double[]": "data_str",
    "float[]": "data_str",
    "int64[]": "data_str",
    "string[]": "data_str",
    "structschema": "data_str",
    "raw": "data_str",
}

# Entrytype prefixes that map to data_str (binary data stored as base64)
PREFIX_TO_COLUMN = {
    "struct:": "data_str",
}

DATA_COLUMNS = ["data_double", "data_int", "data_str", "data_bool"]


def _expected_column_expr() -> pl.Expr:
    """Build a when/then expression mapping entrytype -> expected data column name."""
    items = list(TYPE_TO_COLUMN.items())
    expr = pl.when(pl.col("entrytype") == items[0][0]).then(pl.lit(items[0][1]))
    for etype, col in items[1:]:
        expr = expr.when(pl.col("entrytype") == etype).then(pl.lit(col))
    for prefix, col in PREFIX_TO_COLUMN.items():
        expr = expr.when(pl.col("entrytype").str.starts_with(prefix)).then(pl.lit(col))
    return expr.otherwise(pl.lit(None))


def _is_known_type_expr() -> pl.Expr:
    """Expression that is true when entrytype is a known exact or prefix type."""
    known_exact = set(TYPE_TO_COLUMN.keys())
    expr = pl.col("entrytype").is_in(known_exact)
    for prefix in PREFIX_TO_COLUMN:
        expr = expr | pl.col("entrytype").str.starts_with(prefix)
    return expr


def add_expected_col_null(df: pl.DataFrame) -> pl.DataFrame:
    """Add expected_col_null (bool) and expected_col_null_comment (str).

    expected_col_null is true when the type-appropriate data column is null.
    expected_col_null_comment explains known cases (e.g. sanitized non-finite
    double/float); empty string means needs investigation.
    """
    # Build the "is null in expected column" expression per type
    items = list(TYPE_TO_COLUMN.items())
    is_null_expr = pl.when(pl.col("entrytype") == items[0][0]).then(
        pl.col(items[0][1]).is_null()
    )
    for etype, col in items[1:]:
        is_null_expr = is_null_expr.when(pl.col("entrytype") == etype).then(
            pl.col(col).is_null()
        )
    for prefix, col in PREFIX_TO_COLUMN.items():
        is_null_expr = is_null_expr.when(
            pl.col("entrytype").str.starts_with(prefix)
        ).then(pl.col(col).is_null())
    # Rows with unknown entrytype: no expected column, flag as null
    is_null_expr = is_null_expr.otherwise(pl.lit(True))

    # Comment: explain known cases, leave empty for unknowns
    sanitized_float_types = {"double", "float"}
    has_unrepresentable = (
        pl.col("unrepresentable").is_not_null()
        if "unrepresentable" in df.columns
        else pl.lit(False)
    )
    comment_expr = (
        pl.when(pl.col("entrytype").is_in(sanitized_float_types) & has_unrepresentable)
        .then(pl.lit("see unrepresentable column"))
        .when(~_is_known_type_expr())
        .then(pl.lit("unhandled entrytype"))
        .otherwise(pl.lit(""))
    )

    return df.with_columns(
        is_null_expr.alias("expected_col_null"),
        pl.when(is_null_expr)
        .then(comment_expr)
        .otherwise(pl.lit(None))
        .alias("expected_col_null_comment"),
    )


def add_wrong_col_populated(df: pl.DataFrame) -> pl.DataFrame:
    """Add wrong_col_populated (bool) and wrong_col_populated_fields (str).

    wrong_col_populated is true when a data column that should be null for the
    row's entrytype is non-null. wrong_col_populated_fields lists those column
    names, comma-separated.
    """
    # For each data column, build an expression: "this column is expected for this row"
    # A column is expected if any exact type or prefix type maps to it.
    wrong_per_col = {}
    for col in DATA_COLUMNS:
        expected_exact = [et for et, c in TYPE_TO_COLUMN.items() if c == col]
        expected_prefixes = [p for p, c in PREFIX_TO_COLUMN.items() if c == col]
        is_expected = pl.col("entrytype").is_in(expected_exact)
        for prefix in expected_prefixes:
            is_expected = is_expected | pl.col("entrytype").str.starts_with(prefix)
        wrong_per_col[col] = ~is_expected & pl.col(col).is_not_null()

    any_wrong = wrong_per_col[DATA_COLUMNS[0]]
    for col in DATA_COLUMNS[1:]:
        any_wrong = any_wrong | wrong_per_col[col]

    # Build the fields string: comma-separated names of wrongly populated columns
    fields_parts = [
        pl.when(wrong_per_col[col]).then(pl.lit(col)).otherwise(pl.lit(None))
        for col in DATA_COLUMNS
    ]
    fields_expr = pl.concat_str(fields_parts, separator=",", ignore_nulls=True)

    return df.with_columns(
        any_wrong.alias("wrong_col_populated"),
        pl.when(any_wrong)
        .then(fields_expr)
        .otherwise(pl.lit(None))
        .alias("wrong_col_populated_fields"),
    )


def add_duplicate_timestamp(df: pl.DataFrame) -> pl.DataFrame:
    """Add duplicate_timestamp (bool).

    True when this (entry, timestamp) pair appears more than once.
    """
    counts = df.group_by("entry", "timestamp").agg(pl.len().alias("_dup_count"))
    return (
        df.join(counts, on=["entry", "timestamp"], how="left")
        .with_columns((pl.col("_dup_count") > 1).alias("duplicate_timestamp"))
        .drop("_dup_count")
    )


def add_timestamp_decreased(df: pl.DataFrame) -> pl.DataFrame:
    """Add timestamp_decreased (bool) and timestamp_prev (f64).

    Computed over rows ordered by their original position within each entry.
    timestamp_decreased is true when this row's timestamp is less than the
    previous row's timestamp for the same entry. timestamp_prev is that
    previous timestamp (null for first row of each entry).
    """
    # Add a row index to preserve original order, compute over that
    return (
        df.with_row_index("_row_idx")
        .sort("entry", "_row_idx")
        .with_columns(
            pl.col("timestamp").shift(1).over("entry").alias("timestamp_prev"),
        )
        .with_columns(
            (pl.col("timestamp") < pl.col("timestamp_prev")).alias(
                "timestamp_decreased"
            ),
        )
        .sort("_row_idx")
        .drop("_row_idx")
    )


UNREPRESENTABLE_MAP = {"inf": float("inf"), "-inf": float("-inf"), "nan": float("nan")}


def restore_unrepresentable_floats(df: pl.DataFrame) -> pl.DataFrame:
    """Parse the unrepresentable column back into data_double where possible.

    Known values (inf, -inf, nan) are restored into data_double and the
    unrepresentable column is cleared for those rows.
    """
    if "unrepresentable" not in df.columns:
        return df

    parseable = pl.col("unrepresentable").is_in(list(UNREPRESENTABLE_MAP.keys()))

    restored = (
        pl.when(pl.col("unrepresentable") == "inf")
        .then(pl.lit(float("inf")))
        .when(pl.col("unrepresentable") == "-inf")
        .then(pl.lit(float("-inf")))
        .when(pl.col("unrepresentable") == "nan")
        .then(pl.lit(float("nan")))
        .otherwise(pl.lit(None))
    )

    return df.with_columns(
        pl.when(parseable)
        .then(restored)
        .otherwise(pl.col("data_double"))
        .alias("data_double"),
        pl.when(parseable)
        .then(pl.lit(None).cast(pl.Utf8))
        .otherwise(pl.col("unrepresentable"))
        .alias("unrepresentable"),
    )


def decode_struct_pose2d(df: pl.DataFrame) -> pl.DataFrame:
    """Decode struct:Pose2d rows from base64 data_str into a Polars struct column.

    Adds a 'pose2d' column of type Struct{translation.x, translation.y,
    rotation.value} (all f64).  Non-Pose2d rows get null.

    The base64 payload is 24 bytes: three little-endian doubles
    (Translation2d.x, Translation2d.y, Rotation2d.value).

    Uses native Polars expressions (str.decode → bin.slice → bin.reinterpret)
    so the entire decode runs in the Rust engine with no Python-level loop.
    """
    is_pose2d = pl.col("entrytype") == "struct:Pose2d"
    payload = pl.col("data_str").str.decode("base64", strict=False)
    pose2d_dtype = pl.Struct(
        {
            "translation.x": pl.Float64,
            "translation.y": pl.Float64,
            "rotation.value": pl.Float64,
        }
    )

    return df.with_columns(
        pl.when(is_pose2d)
        .then(
            pl.struct(
                payload.bin.slice(0, 8)
                .bin.reinterpret(dtype=pl.Float64)
                .alias("translation.x"),
                payload.bin.slice(8, 8)
                .bin.reinterpret(dtype=pl.Float64)
                .alias("translation.y"),
                payload.bin.slice(16, 8)
                .bin.reinterpret(dtype=pl.Float64)
                .alias("rotation.value"),
            )
        )
        .otherwise(pl.lit(None, dtype=pose2d_dtype))
        .alias("pose2d")
    )


def decode_struct_pose3d(df: pl.DataFrame) -> pl.DataFrame:
    """Decode struct:Pose3d rows from base64 data_str into a Polars struct column.

    Adds a 'pose3d' column with all 7 leaf fields:
      translation.x/y/z (f64) and rotation.q.w/x/y/z (f64).
    Non-Pose3d rows get null.

    The base64 payload is 56 bytes: seven little-endian doubles
    (Translation3d x/y/z, then Quaternion w/x/y/z).
    """
    is_pose3d = pl.col("entrytype") == "struct:Pose3d"
    payload = pl.col("data_str").str.decode("base64", strict=False)
    pose3d_dtype = pl.Struct(
        {
            "translation.x": pl.Float64,
            "translation.y": pl.Float64,
            "translation.z": pl.Float64,
            "rotation.q.w": pl.Float64,
            "rotation.q.x": pl.Float64,
            "rotation.q.y": pl.Float64,
            "rotation.q.z": pl.Float64,
        }
    )

    return df.with_columns(
        pl.when(is_pose3d)
        .then(
            pl.struct(
                payload.bin.slice(0, 8)
                .bin.reinterpret(dtype=pl.Float64)
                .alias("translation.x"),
                payload.bin.slice(8, 8)
                .bin.reinterpret(dtype=pl.Float64)
                .alias("translation.y"),
                payload.bin.slice(16, 8)
                .bin.reinterpret(dtype=pl.Float64)
                .alias("translation.z"),
                payload.bin.slice(24, 8)
                .bin.reinterpret(dtype=pl.Float64)
                .alias("rotation.q.w"),
                payload.bin.slice(32, 8)
                .bin.reinterpret(dtype=pl.Float64)
                .alias("rotation.q.x"),
                payload.bin.slice(40, 8)
                .bin.reinterpret(dtype=pl.Float64)
                .alias("rotation.q.y"),
                payload.bin.slice(48, 8)
                .bin.reinterpret(dtype=pl.Float64)
                .alias("rotation.q.z"),
            )
        )
        .otherwise(pl.lit(None, dtype=pose3d_dtype))
        .alias("pose3d")
    )


def load_json_log(path):
    """Scan all rows for schema inference so rare columns like
    "unrepresentable" are not missed."""

    df = pl.read_ndjson(path, infer_schema_length=None)
    df = restore_unrepresentable_floats(df)
    return df


def diagnose_log(df, fail_fast=True):
    """Print summary of findings per analysis column, and if fail_fast then quit if any issues."""
    df = add_expected_col_null(df)
    df = add_wrong_col_populated(df)
    df = add_duplicate_timestamp(df)
    df = add_timestamp_decreased(df)
    fail = False
    for col, comment_col in [
        ("expected_col_null", "expected_col_null_comment"),
        ("wrong_col_populated", "wrong_col_populated_fields"),
        ("duplicate_timestamp", None),
        ("timestamp_decreased", "timestamp_prev"),
    ]:
        flagged = df.filter(pl.col(col))
        print(f"{col}: {flagged.height} rows flagged")
        if not flagged.is_empty() and comment_col:
            print(
                flagged.group_by(comment_col)
                .agg(pl.len().alias("count"))
                .sort("count", descending=True)
            )
            fail = True
    if fail:
        raise (ValueError("Printed findings above, and fail_fast=True"))


def main():
    path = sys.argv[1]
    df = load_json_log(path)
    print(f"Loaded {df.height} rows, {df.width} columns from {path}\n")

    diagnose_log(df)

    # --- Summaries ---
    print("\n" + "=" * 60)
    print("Timeseries summary")
    print("=" * 60)
    records_per_entry = df.group_by("entry", "entryname", "entrytype").agg(
        pl.len().alias("record_count")
    )
    print(f"Distinct timeseries (by entry): {records_per_entry.height}\n")
    print("Records per timeseries distribution:")
    print(records_per_entry.select("record_count").describe())

    print("\n" + "=" * 60)
    print("Struct schemas")
    print("=" * 60)
    schemas = (
        df.filter(pl.col("entrytype") == "structschema")
        .select("entryname", "data_str")
        .unique()
    )
    print(f"Distinct struct schemas: {schemas.height}\n")
    if schemas.height <= 20:
        with pl.Config(fmt_str_lengths=200, tbl_width_chars=200):
            print(schemas)
    else:
        with pl.Config(fmt_str_lengths=200, tbl_width_chars=200):
            print(schemas.head(20))
        print(f"... and {schemas.height - 20} more")


if __name__ == "__main__":
    main()

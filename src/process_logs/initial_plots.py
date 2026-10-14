# ---
# jupyter:
#   jupytext:
#     formats: py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.19.1
#   kernelspec:
#     display_name: Python 3 (ipykernel)
#     language: python
#     name: python3
# ---

# %%
# %load_ext autoreload

# %%
# %autoreload 2

# %%
from src.initial_analysis import *
import polars as pl
from polars import col as c

# %%
path = "second_log.jsonl"
df = load_json_log(path)
print(f"Loaded {df.height} rows, {df.width} columns from {path}\n")

# %%
df.sample(10)

# %%
diagnose_log(df)

# %%
records_per_entry = df.group_by("entry", "entryname", "entrytype").agg(
        pl.len().alias("record_count")
    ).sort("record_count")
records_per_entry

# %%
[en for en in records_per_entry.get_column("entryname").to_list() if "Angle" in en]

# %%
df.filter(c("entryname").str.contains("/RealOutputs/Turret/Turret Pose")).group_by("entryname").agg(count=pl.len()).get_column("entryname").to_list()

# %%
turret_poses = df.filter(c("entryname").str.contains("/RealOutputs/Turret/Turret Pose"))
turret_poses.sample(3)

# %%
df.filter(c("entryname").str.contains("/RealOutputs/Turret/Turret Pose")).filter(c("data_str").str.len_chars() != 32)

# %%
decode_struct_pose3d(df.filter(c("entryname").str.contains("/RealOutputs/Turret/Turret Pose")))

# %%
df.filter(c("entryname").str.contains("HoodAngle")).group_by("entryname").agg(count=pl.len())

# %%
robot_poses = df.filter(c("entryname").str.contains("RobotPoses"))
robot_poses.group_by("entryname", "entrytype").agg(
    count=pl.len(),
    str_data_len_min=c("data_str").str.len_chars().min(),
    str_data_len_max=c("data_str").str.len_chars().max(),
)[0].to_dict()['entryname'][0]

# %%
robot_poses.filter(c("entryname") == "/RealOutputs/Vision/Camera0/RobotPoses")

# %%
rp = robot_poses.filter(c("data_str") != "").with_columns(
    accepted=c("entryname").str.contains("RobotPosesAccepted")
)

# %%
robot_poses.with_columns(c("entryname").str.split("/").list[-1]).sort("entry", "timestamp")#.filter(c("entry") == 208)

# %%
swerve_poses = decode_struct_pose2d(df).filter(c("entryname").str.contains("CommandSwerveDrive"))

# %%
swerve_poses = swerve_poses.with_columns(
    x=c("pose2d").struct.field("x"),
    y=c("pose2d").struct.field("y"),
    rotation=c("pose2d").struct.field("rotation"),
).with_columns(
    dx=(c("x")-c("x").shift(1)),
    dy=(c("y")-c("y").shift(1)),
    dt=(c("timestamp")-c("timestamp").shift(1)),
).with_columns(
    dxdt=c("dx") / c("dt"),
    dydt=c("dy") / c("dt"),
).with_columns(
    abs_dxdt=c("dxdt").abs(),
    abs_dydt=c("dydt").abs(),
    vel_mag=(c("dxdt")**2 + c("dydt")**2).sqrt(),
)
swerve_poses.sort("abs_dxdt").tail(30)

# %%
big_x_speeds = swerve_poses.sort("dxdt").filter([
        c("timestamp") > 280,
        c("timestamp") < 390,        
])


# %%
big_x_speeds.get_column("dxdt").abs().quantile(1)

# %%
import altair as alt
alt.data_transformers.enable("vegafusion")

# %%
swerve_poses.columns

# %%
alt.Chart(big_x_speeds.filter([
    c("timestamp") > 350,
    c("timestamp") < 360,
])).mark_circle().encode(
    x=alt.X("timestamp:Q").scale(zero=False, domainMin=350),
    y="x:Q",
    tooltip="dx:Q dt:Q dxdt:Q".split()
).properties(width=800, height=600).interactive()


# %%
alt.Chart(big_x_speeds.filter([
    c("timestamp") > 350,
    c("timestamp") < 380,
])).mark_circle().encode(
    x=alt.X("timestamp:Q").scale(zero=False, domainMin=350),
    y="vel_mag:Q",
    tooltip="dx:Q dt:Q dxdt:Q vel_mag:Q".split()
).properties(width=800, height=600).interactive()

# %%

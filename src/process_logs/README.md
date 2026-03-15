# Scripts for understanding wpi logs

src/wpilog2ndjson.py - convert the first argument file into a jsonl/ndjson file. Redirect output to a file to save it.
src/initial_analysis - some helper functions
initial_plots.py - a script that loads a hardcoded file (edit it) into a polars dataframe (table), does simple filtering, and shows a couple of plots. A starting point

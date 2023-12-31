#!/usr/bin/env bash 

mtsp()
{
    EXE="build/mtsp"
    PARAM_FILE="/workspaces/path_segments/input/param.yaml"
    $EXE $PARAM_FILE
}
mtsp
PY_FILE="plotter/plotter.py"
python $PY_FILE
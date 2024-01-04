#!/usr/bin/env bash 

mtsp()
{
    EXE="cmake-build-debug/mtsp"
    PARAM_FILE="/Users/redwan/CLionProjects/path_segments/input/param.yaml"
    $EXE $PARAM_FILE
}
mtsp
PYTHON="/Users/redwan/miniconda/bin/python"
PY_FILE="plotter/plotter.py"
$PYTHON $PY_FILE
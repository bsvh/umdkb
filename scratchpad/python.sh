#!/bin/bash

# What real Python executable to use
PYVER=3.7
PYTHON=/Library/Frameworks/Python.framework/Versions/$PYVER/bin/python$PYVER

export PYTHONHOME=$(pipenv --venv)
exec $PYTHON $1


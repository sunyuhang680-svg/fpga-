"""Project-owned outputs; load installed vision dependencies without executing old pth."""
from pathlib import Path
import os
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
WORKSPACE=ROOT.parent
NATIVE_ALIAS=Path('C:/Users/34921/.codex/visualizations/2026/10/21/01a15e2f-1ad1-71a0-b96e-eda8e089ef5f/stereo_workspace')

def native_tool_path(path):
    """ASCII junction names the SAME authorized workspace, not an external staging copy."""
    relative=Path(path).resolve().relative_to(WORKSPACE.resolve())
    if NATIVE_ALIAS.exists() and NATIVE_ALIAS.resolve()==WORKSPACE.resolve():
        return NATIVE_ALIAS/relative
    return Path(path)

def configure():
    temp=ROOT/'build'/'tmp'
    temp.mkdir(parents=True,exist_ok=True)
    tool_temp=native_tool_path(temp)
    for key in ('TEMP','TMP','TMPDIR'):os.environ[key]=str(tool_temp)
    tempfile.tempdir=str(tool_temp)
    os.environ['PYTHONDONTWRITEBYTECODE']='1'
    sys.dont_write_bytecode=True

def vision():
    configure()
    # Python3.12 ABI-compatible read-only packages. Do not execute site.addsitedir/pth.
    deps=Path('D:/topic2_workspace/topic2_undistort/.venv/Lib/site-packages')
    if deps.exists():sys.path.insert(0,str(deps))
    import cv2
    import numpy as np
    cv2.setNumThreads(2)
    cv2.ocl.setUseOpenCL(False)
    return cv2,np

# -*- mode: python ; coding: utf-8 -*-
import os

ENTRY_SCRIPT = 'run_app.py'
APP_NAME = 'MPPT-App'

block_cipher = None

a = Analysis(
    [ENTRY_SCRIPT],
    pathex=[os.path.abspath('.')],   # include your project root
    binaries=[],
    datas=[],
    hiddenimports=[
        'mppt',
        'mppt.core.sensor',
        'mppt.core.collector',
        'mppt.core.parser',
        'mppt.models',
        'mppt.models.enums',
        'mppt.models.data_structures',
        'mppt.models.characteristics',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=True,   # store .pyc unpacked; helps reduce AV heuristics
    optimize=0,
)

pyz = PYZ(a.pure)

# For ONEDIR: exclude_binaries=True in EXE, then add COLLECT below.
exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,                 # <-- ONEDIR pattern
    name=APP_NAME,
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,                         # windowed app; set True for console
    disable_windowed_traceback=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon='assets/appicon.ico',
    version='windows/version_info.txt',    # version resource
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name=APP_NAME,                         # dist/MPPT-App/ folder
)

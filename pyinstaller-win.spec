# -*- mode: python ; coding: utf-8 -*-
import os

ENTRY_SCRIPT = 'run_app.py'
APP_NAME = 'MPPT-App'

block_cipher = None

a = Analysis(
    [ENTRY_SCRIPT],
    pathex=[os.path.abspath('.')],
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
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name=APP_NAME,
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon='assets/appicon.ico',
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name=APP_NAME,
)

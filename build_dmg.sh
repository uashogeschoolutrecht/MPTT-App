#!/bin/sh
# Build a macOS Disk Image (.dmg) for the app bundle using create-dmg.
# Requires: create-dmg (brew install create-dmg)

set -e

APP_NAME="MPPT-App"
APP_BUNDLE="dist/${APP_NAME}.app"
DMG_PATH="dist/${APP_NAME}.dmg"
STAGE_DIR="dist/dmg"
VOL_ICON="assets/appicon.icns"

if ! command -v create-dmg >/dev/null 2>&1; then
  echo "Error: create-dmg not found. Install with: brew install create-dmg" >&2
  exit 1
fi

if [ ! -d "$APP_BUNDLE" ]; then
  echo "Error: $APP_BUNDLE not found. Build the app first (e.g. .venv/bin/pyinstaller --noconfirm mppt.spec)." >&2
  exit 1
fi

mkdir -p "$STAGE_DIR"
rm -rf "$STAGE_DIR"/* 2>/dev/null || true

cp -R "$APP_BUNDLE" "$STAGE_DIR"/

if [ -f "$DMG_PATH" ]; then
  rm -f "$DMG_PATH"
fi

create-dmg \
  --volname "$APP_NAME" \
  --volicon "$VOL_ICON" \
  --window-pos 200 120 \
  --window-size 640 400 \
  --icon-size 120 \
  --icon "${APP_NAME}.app" 160 200 \
  --hide-extension "${APP_NAME}.app" \
  --app-drop-link 480 200 \
  "$DMG_PATH" \
  "$STAGE_DIR/"

echo "DMG created at: $DMG_PATH"

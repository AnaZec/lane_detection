#!/usr/bin/env bash
set -euo pipefail

# Run from project root (Zadatak/)
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="$ROOT_DIR/build/lane_finding"

IMG_IN="$ROOT_DIR/test_images"
VID_IN="$ROOT_DIR/test_videos"

IMG_OUT="$ROOT_DIR/output/images/overlay"
VID_OUT="$ROOT_DIR/output/videos/overlay"

mkdir -p "$IMG_OUT" "$VID_OUT"

echo "== Calibration (once) =="
"$BIN" --calibrate

echo "== Images =="
shopt -s nullglob
for img in "$IMG_IN"/*.jpg "$IMG_IN"/*.png "$IMG_IN"/*.jpeg; do
  base="$(basename "$img")"
  name="${base%.*}"
  echo "  -> $base"

  "$BIN" --image "$img"

  # Program writes: output/images/<name>/result.jpg
  SRC_IMG="$ROOT_DIR/output/images/${name}/result.jpg"
  if [[ ! -f "$SRC_IMG" ]]; then
    echo "ERROR: Expected output image not found: $SRC_IMG" >&2
    exit 1
  fi

  # Copy into overlay folder with stable naming
  cp -f "$SRC_IMG" "$IMG_OUT/${name}_out.jpg"
done

echo "== Videos =="
for vid in "$VID_IN"/*.mp4 "$VID_IN"/*.avi; do
  base="$(basename "$vid")"
  name="${base%.*}"
  echo "  -> $base"

  # Your VideoWriter works reliably with AVI+MJPG
  "$BIN" --video "$vid" "$VID_OUT/${name}_out.avi"
done

echo "== Done =="
echo "Image outputs: $IMG_OUT"
echo "Video outputs: $VID_OUT"

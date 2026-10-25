#!/usr/bin/env bash
# make_submission.sh — builds the one submission ZIP, PRS-PM-2026-09-15-AIS08-GRP-12-TheEyes.zip.
#
#   scripts/make_submission.sh --dry-run     # stage it and print the sizes, no ZIP
#   scripts/make_submission.sh               # stage it and write the ZIP
#   scripts/make_submission.sh --lean        # without the bulk data (see LEAN below), if the full one fails to upload
#
# What goes in, and where it comes from:
#   the repository at HEAD (git archive)     code, docs, report, both decks, scripts, meshes
#   assets/models/*.onnx                     the two detector models (git-ignored)
#   ProjectReport/<name>-Video.mp4           the video (runs/video/, not in git)
#   data/results/                            every scored scene's logs (runs/scored/), and the
#                                            live takes' logs; each metrics.sqlite through
#                                            sqlite3 .backup so nothing is left in a WAL file
#   data/real_v1/                            the real-frame detector set: train, val, test, labels
#   data/synth50k_v2_sample/                 300 frames of the 50,000-frame synthetic set
#   data/scored_scene_example/               one scored recording, whole, so a replay can be run
# Everything else (all recordings, the full synthetic set) is behind the link in data/DATA.md.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$PWD
NAME=PRS-PM-2026-09-15-AIS08-GRP-12-TheEyes
VIDEO=runs/video/PRS-PM-2026-AIS08-GRP-12-TheEyes-Video.mp4
SYNTH=/opt/the-eyes-data/synth50k_v2
REAL=/opt/the-eyes-data/real_v1
EXAMPLE=captures/scored/20261005/t4_cube_over
OUT=${OUT:-$ROOT/submission}
DRY=false; LEAN=false
for a in "$@"; do
  case $a in --dry-run) DRY=true ;; --lean) LEAN=true ;; *) echo "unknown argument $a" >&2; exit 2 ;; esac
done

$DRY || [[ -z $(git status --porcelain --untracked-files=no) ]] || { echo "commit first: the ZIP is built from HEAD" >&2; exit 1; }
STAGE=$(mktemp -d -p "$ROOT" .submission_stage.XXXX)
trap 'rm -rf -- "${STAGE:?}"' EXIT
D=$STAGE/$NAME
mkdir -p "$D"

git archive HEAD | tar -x -C "$D"
mkdir -p "$D/assets/models" && cp assets/models/*.onnx "$D/assets/models/"
if [[ -f $VIDEO ]]; then cp "$VIDEO" "$D/ProjectReport/"; else echo "NOTE: no video yet at $VIDEO"; fi

# results: the logs every number in the report comes from
mkdir -p "$D/data/results"
rsync -a --exclude '*.sqlite' --exclude '*.sqlite-wal' --exclude '*.sqlite-shm' runs/scored/ "$D/data/results/scored/"
while IFS= read -r db; do
  rel=${db#runs/}; mkdir -p "$D/data/results/$(dirname "$rel")"
  sqlite3 "$db" ".backup '$D/data/results/$rel'"
done < <(find runs/scored -name metrics.sqlite; ls -d runs/runtime/*_demo_* 2>/dev/null | sed 's|$|/metrics.sqlite|' | while read -r f; do [[ -f $f ]] && echo "$f"; done)

if ! $LEAN; then
  # the real-frame set, linked so it is not copied twice; zip stores what a link points to
  ln -s "$REAL" "$D/data/real_v1"
  # 250 training and 50 validation frames of the synthetic set, each with its label
  for split in train:250 val:50; do
    s=${split%%:*}; n=${split##*:}
    mkdir -p "$D/data/synth50k_v2_sample/images/$s" "$D/data/synth50k_v2_sample/labels/$s"
    ls "$SYNTH/images/$s" | grep '\.png$' | sed -n "1,${n}p" | while read -r f; do
      b=${f%.png}
      ln -s "$SYNTH/images/$s/$b.png" "$SYNTH/images/$s/$b.npy" "$D/data/synth50k_v2_sample/images/$s/"
      [[ -f $SYNTH/labels/$s/$b.txt ]] && ln -s "$SYNTH/labels/$s/$b.txt" "$D/data/synth50k_v2_sample/labels/$s/"
    done
  done
  cp "$SYNTH/generation.log" "$D/data/synth50k_v2_sample/"
  sed 's|^path: .*|path: .|' "$SYNTH/data.yaml" > "$D/data/synth50k_v2_sample/data.yaml"
  # one scored recording, whole: its stills, calibration and the session it replays
  mkdir -p "$D/data/scored_scene_example"
  ln -s "$ROOT/$EXAMPLE" "$D/data/scored_scene_example/$(basename "$(dirname "$EXAMPLE")")_$(basename "$EXAMPLE")"
fi

for x in "$D"/data/* "$D"; do du -shL "$x" | sed "s|$STAGE/||"; done
if $DRY; then echo "dry run: no ZIP written"; exit 0; fi
mkdir -p "$OUT"
Z=$OUT/$NAME$($LEAN && echo _lean).zip
rm -f -- "$Z"
(cd "$STAGE" && zip -q -r -9 "$Z" "$NAME")
ls -la "$Z"
unzip -l "$Z" | tail -1

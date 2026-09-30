#!/usr/bin/env bash
# capture_day.sh — the scored-capture routine of docs/CAPTURE_PROTOCOL.md §6,
# as commands, so capture day is mechanical and every scene is recorded the
# same way.
#
# Usage:
#   scripts/capture_day.sh begin   [--day YYYYMMDD]
#   scripts/capture_day.sh scene   <name> --tier T1|T2|T3|T4
#                                  --occluded none|cam_front|cam_over
#                                  [--objects cls,cls,...] [--target cls]
#                                  [--tv off|static|on] [--pairs 300] [--stills 10]
#                                  [--day YYYYMMDD]
#   scripts/capture_day.sh end     [--day YYYYMMDD]
#   scripts/capture_day.sh list    [--day YYYYMMDD]
#   scripts/capture_day.sh process [--day YYYYMMDD] [--force] [--replay-args "..."]
#
# begin    startup checks, re-registration (writes config/extrinsics.yaml, as
#          CALIBRATION_PROTOCOL.md does), then 30 pairs of the empty table for
#          the §4.3 plane check.
# scene    one scored scene: ground-truth stills at 1920x1080 with the occluder
#          out of view → an immediate tray check (gt_from_trays --dry-run, no
#          database) → occluder in → `the_eyes_run live --record-only` →
#          notes.yaml. Nothing downstream is run: recordings replay bit-exact.
# end      re-registers into a scratch file, NOT config: if the rig's baseline
#          moved during the day, the scenes shot after the move are suspect.
# process  needs the GPU and the FoundationPose server, not the rig. Per scene:
#          replay → ground truth from the stills → visibility → ablation, then
#          checks each scene's measured visibility against its declared tier.
#
# Layout (both trees are gitignored):
#   captures/scored/<day>/<scene>/{notes.yaml, gt_stills/, gt_check.log,
#                                  record.log, session -> recorded session}
#   runs/scored/<day>/<scene>/{metrics.sqlite, *.log, visibility.csv,
#                              visibility_overlay/}
#
# Examples:
#   scripts/capture_day.sh begin
#   scripts/capture_day.sh scene t1_cube --tier T1 --occluded none --objects red_cube --target red_cube
#   scripts/capture_day.sh scene t4_cube_over --tier T4 --occluded cam_over \
#       --objects red_cube,bowl --target red_cube
#   scripts/capture_day.sh end
#   scripts/capture_day.sh process            # the evening after, or any day
set -uo pipefail

cd "$(dirname "$0")/.."
REPO="$PWD"

RUN=build/runtime/the_eyes_run
GT=build/runtime/gt_from_trays
VIS=build/runtime/visibility_from_gt
ABL=build/runtime/ablation_from_log
CAL=build/calibration/calibrate

die() { echo "capture_day: $*" >&2; exit 1; }
say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
pause() { read -r -p "$* " _; }
now() { date -Iseconds; }

need_bins() {
  for b in "$@"; do
    [[ -x "$b" ]] || die "$b is not built -- cmake --build build -j\"\$(nproc)\""
  done
}

serial_of() {
  python3 -c "import yaml,sys; print(yaml.safe_load(open('config/cameras.yaml'))['cameras'][sys.argv[1]]['serial'])" "$1"
}

# ---- arguments ---------------------------------------------------------------
[[ $# -ge 1 ]] || { sed -n '2,40p' "$0"; exit 2; }
CMD="$1"; shift
DAY="$(date +%Y%m%d)"
NAME="" TIER="" OCCLUDED="" OBJECTS="" TARGET="" TV="off" PAIRS=300 STILLS=10
FORCE=0 REPLAY_ARGS=""
if [[ "$CMD" == "scene" ]]; then
  [[ $# -ge 1 && "$1" != --* ]] || die "scene needs a name"
  NAME="$1"; shift
fi
while [[ $# -gt 0 ]]; do
  case "$1" in
    --day) DAY="$2"; shift 2 ;;
    --tier) TIER="$2"; shift 2 ;;
    --occluded) OCCLUDED="$2"; shift 2 ;;
    --objects) OBJECTS="$2"; shift 2 ;;
    --target) TARGET="$2"; shift 2 ;;
    --tv) TV="$2"; shift 2 ;;
    --pairs) PAIRS="$2"; shift 2 ;;
    --stills) STILLS="$2"; shift 2 ;;
    --force) FORCE=1; shift ;;
    --replay-args) REPLAY_ARGS="$2"; shift 2 ;;
    *) die "unknown argument $1" ;;
  esac
done
DAYDIR="captures/scored/$DAY"
RUNDIR="runs/scored/$DAY"
log_day() { mkdir -p "$DAYDIR"; echo "$(now) $*" >> "$DAYDIR/day.log"; }

# Shoots one registration pair into $1 and re-registers against the committed
# extrinsics, writing $2. Returns reregister's exit status.
register_pair() {
  local dir="$1" out="$2"
  rm -rf "$dir"
  say "Board on its jig at the workspace centre, flat, pattern up. Nothing else on it."
  "$CAL" capture config/cameras.yaml config/calibration.yaml "$dir" --register || return 1
  "$CAL" reregister config/calibration.yaml config/extrinsics.yaml "$out" \
    "cam_front:$(serial_of cam_front):$dir/cam_front/zz_register.png" \
    "cam_over:$(serial_of cam_over):$dir/cam_over/zz_register.png"
}

# Records $2 pairs as session name $1. The runner's output goes to the terminal
# (stderr) and $3; stdout carries only the session directory, for $(...).
record() {
  local name="$1" pairs="$2" logf="$3" rc
  "$RUN" live --record-only "$name" --frames "$pairs" 2>&1 | tee "$logf" >&2
  rc=${PIPESTATUS[0]}
  [[ $rc -eq 0 ]] || return $rc
  sed -n 's/^recording to //p' "$logf" | head -1
}

# ---- begin -------------------------------------------------------------------
cmd_begin() {
  need_bins "$RUN" "$CAL"
  mkdir -p "$DAYDIR"
  say "== begin $DAY =="
  "$RUN" check --live || die "startup checks failed -- fix them before anything is shot"

  if ! register_pair "$DAYDIR/reregister_begin" config/extrinsics.yaml; then
    log_day "begin: re-registration REFUSED"
    cat <<'EOF'

Re-registration refused. If the baseline gate tripped, a TRIPOD moved, not the
desk, and re-registration cannot fix it. Before recalibrating, shoot a burst of
registration pairs with nothing touched and compare each camera's solved
position: a slipping mount scatters by 10+ mm, a rigid one repeats to < 1 mm
(CAPTURE_PROTOCOL.md 4.2f). After a full recalibration, re-run
training/shared_region.py --overlay and re-tape before any scene.
EOF
    exit 1
  fi
  log_day "begin: re-registered"
  if git diff --quiet -- config/extrinsics.yaml; then
    echo "config/extrinsics.yaml unchanged."
  else
    say "config/extrinsics.yaml was updated. Commit it BEFORE the first scene, so every run's git_rev names the extrinsics it used:"
    echo "  git commit config/extrinsics.yaml -m \"Re-register the rig for the $DAY capture\""
  fi

  pause "Take the board OFF. Desk clear, TV off, lights as for capture. ENTER to record 30 pairs of the empty table:"
  local s
  s=$(record empty_table 30 "$DAYDIR/empty_table.log") || die "empty-table recording failed"
  ln -sfn "$REPO/$s" "$DAYDIR/empty_table"
  log_day "begin: empty table -> $s"
  say "Ready. Next: scripts/capture_day.sh scene <name> --tier T1 ..."
}

# ---- scene -------------------------------------------------------------------
cmd_scene() {
  need_bins "$RUN" "$CAL" "$GT"
  [[ "$NAME" =~ ^[A-Za-z0-9_-]+$ ]] || die "scene name: letters, digits, _ and - only"
  case "$TIER" in
    T1|T2) [[ "$OCCLUDED" == "none" ]] || die "$TIER has no occluder: --occluded none" ;;
    T3|T4) [[ "$OCCLUDED" == "cam_front" || "$OCCLUDED" == "cam_over" ]] ||
             die "$TIER needs --occluded cam_front|cam_over"
           [[ -n "$TARGET" ]] || die "$TIER needs --target: the tier is a claim about one object" ;;
    *) die "--tier T1|T2|T3|T4 is required" ;;
  esac
  case "$TV" in off|static|on) ;; *) die "--tv off|static|on" ;; esac
  if [[ -n "$TARGET" && -n "$OBJECTS" && ",$OBJECTS," != *",$TARGET,"* ]]; then
    die "--target $TARGET is not in --objects $OBJECTS"
  fi
  local dir="$DAYDIR/$NAME"
  [[ ! -e "$dir" ]] || die "$dir already exists -- pick a new name; a scene is never overwritten"

  # A T3/T4 fusion gain is uninterpretable without that object's T1 floor (§9).
  if [[ "$TIER" == "T3" || "$TIER" == "T4" ]]; then
    local has_t1
    has_t1=$(python3 - "$DAYDIR" "$TARGET" <<'EOF'
import glob, sys, yaml
hits = [n for n in glob.glob(sys.argv[1] + "/*/notes.yaml")
        if (d := yaml.safe_load(open(n))) and d.get("tier") == "T1" and d.get("target") == sys.argv[2]]
print("yes" if hits else "no")
EOF
)
    if [[ "$has_t1" != "yes" ]]; then
      echo "WARNING: no T1 scene for $TARGET in $DAYDIR yet. Record its T1 first (CAPTURE_PROTOCOL.md 9)."
      pause "ENTER to continue anyway, Ctrl-C to stop:"
    fi
  fi

  local dirty=false
  git diff --quiet -- config/ || dirty=true
  if $dirty; then
    echo "WARNING: config/ differs from HEAD. Every scene in the suite must share one config"
    echo "(CAPTURE_PROTOCOL.md 9) and the runs table records only git_rev. Commit it first."
    pause "ENTER to continue anyway, Ctrl-C to stop:"
  fi

  mkdir -p "$dir"
  local started; started=$(now)
  say "== scene $NAME ($TIER, occluded: $OCCLUDED) =="
  echo "Objects seated in their trays, bottle on its tick. OCCLUDER OUT OF VIEW. TV: $TV."
  echo "Take the phone photo from above now (predicate hand labels) -- save it later as $dir/photo.jpg"
  pause "ENTER to shoot $STILLS ground-truth stills at 1920x1080:"

  local gt_check="" rc
  while true; do
    rm -rf "$dir/gt_stills"
    "$CAL" capture config/cameras.yaml config/calibration.yaml "$dir/gt_stills" \
      --auto "$STILLS" --interval 1 > "$dir/gt_stills.log" 2>&1 ||
      { tail -20 "$dir/gt_stills.log"; die "stills capture failed"; }
    "$GT" "$dir/gt_stills" - --stills --factory --dry-run 2>&1 | tee "$dir/gt_check.log"
    rc=${PIPESTATUS[0]}
    local missing=""
    if [[ -n "$OBJECTS" ]]; then
      IFS=',' read -ra objs <<< "$OBJECTS"
      for o in "${objs[@]}"; do grep -qx "$o" "$dir/gt_check.log" || missing+=" $o"; done
    fi
    if [[ $rc -eq 0 && -z "$missing" ]]; then gt_check=pass; break; fi
    [[ -n "$missing" ]] && echo "NOT FOUND in the stills:$missing"
    echo "Ground-truth check FAILED (exit $rc). Glare on a marker card, a card in the wrong"
    echo "recess, or a tray outside the shared view are the usual causes."
    local a; read -r -p "[r]eshoot stills / [c]ontinue anyway / [a]bort: " a
    case "$a" in
      r) continue ;;
      c) gt_check="failed($rc)${missing:+ missing:$missing}"; break ;;
      *) rm -rf "$dir"; die "aborted; $dir removed" ;;
    esac
  done

  if [[ "$OCCLUDED" == "none" ]]; then
    pause "No occluder for $TIER. Hands off the desk. ENTER to record $PAIRS pairs:"
  else
    pause "Place the occluder now, blocking $OCCLUDED's view of ${TARGET:-the target}. DO NOT touch the objects. ENTER to record $PAIRS pairs:"
  fi
  local session=""
  session=$(record "scored_$NAME" "$PAIRS" "$dir/record.log")
  rc=$?
  [[ -n "$session" ]] && ln -sfn "$REPO/$session" "$dir/session"

  local note=""
  read -r -p "One-line note for this scene (ENTER for none): " note
  note=${note//\\/}
  cat > "$dir/notes.yaml" <<EOF
# Written by scripts/capture_day.sh -- the run notes CAPTURE_PROTOCOL.md 6 asks for.
scene: $NAME
day: "$DAY"
tier: $TIER
occluded_camera: $OCCLUDED
target: ${TARGET:-}
objects: [${OBJECTS//,/, }]
tv: $TV
pairs_requested: $PAIRS
stills_requested: $STILLS
gt_check: "$gt_check"
session: "${session:-}"
record_exit: $rc
git_rev: $(git rev-parse HEAD)
config_dirty: $dirty
started: "$started"
finished: "$(now)"
note: "${note//\"/\'}"
EOF
  log_day "scene $NAME $TIER occluded=$OCCLUDED gt=$gt_check record_exit=$rc session=$session"
  [[ $rc -eq 0 ]] || die "recording exited $rc -- see $dir/record.log; notes.yaml records it"
  say "Scene $NAME done. Leave the rig alone between scenes."
}

# ---- end ---------------------------------------------------------------------
cmd_end() {
  need_bins "$CAL"
  say "== end $DAY =="
  if register_pair "$DAYDIR/reregister_end" "$DAYDIR/extrinsics_end.yaml"; then
    log_day "end: rig held (re-registration passed against this morning's extrinsics)"
    say "Rig held: the baseline is within the gate of this morning's. The day's scenes stand."
  else
    log_day "end: re-registration REFUSED -- the rig moved during the day"
    say "The rig moved during the day. Scenes shot after the move are suspect: check each scene's"
    echo "cross-camera disagreement in gt_check.log for the point where it jumps, and reshoot from there."
    exit 1
  fi
}

# ---- list --------------------------------------------------------------------
cmd_list() {
  [[ -d "$DAYDIR" ]] || die "no $DAYDIR"
  printf '%-22s %-4s %-10s %-15s %-10s %s\n' scene tier occluded target gt_check session
  for n in "$DAYDIR"/*/notes.yaml; do
    [[ -e "$n" ]] || continue
    python3 - "$n" <<'EOF'
import sys, yaml, os
d = yaml.safe_load(open(sys.argv[1]))
s = d.get('session') or ''
ok = 'ok' if s and os.path.isdir(s) else 'MISSING'
print(f"{d['scene']:<22} {d['tier']:<4} {d['occluded_camera']:<10} {str(d.get('target') or '-'):<15} "
      f"{str(d.get('gt_check')):<10} {ok}")
EOF
  done
  [[ -f "$DAYDIR/day.log" ]] && { echo; tail -3 "$DAYDIR/day.log"; }
}

# ---- process -----------------------------------------------------------------
# Medians per view and object from visibility.csv, judged against the tier the
# scene was declared as. The bands are the proposal's (6.4): T4 < 25% visible
# from the occluded view, T3 25-75%, and in every tier the OTHER view must see
# the target clearly, or there is nothing for fusion to recover it from.
tier_check() {
  python3 - "$1" "$2" <<'EOF'
import csv, statistics, sys, yaml
notes = yaml.safe_load(open(sys.argv[1]))
vis = {}
for r in csv.DictReader(open(sys.argv[2])):
    v = float(r['visible_fraction'])
    if v == v:
        vis.setdefault((r['camera_id'], r['cls']), []).append(v)
med = {k: statistics.median(v) for k, v in vis.items()}
tier, occ, target = notes['tier'], notes['occluded_camera'], notes.get('target')
problems = []
for (cam, cls), m in sorted(med.items()):
    if tier in ('T1', 'T2'):
        if m <= 0.75:
            problems.append(f"{cam} sees only {m:.0%} of {cls}; {tier} should be clear")
        continue
    if cls != target:
        continue  # T3/T4 are claims about the target; the rest may be partly covered
    if cam == occ and tier == 'T4' and not m < 0.25:
        problems.append(f"{cam} sees {m:.0%} of {cls}; T4 needs < 25%")
    elif cam == occ and tier == 'T3' and not 0.25 <= m <= 0.75:
        problems.append(f"{cam} sees {m:.0%} of {cls}; T3 needs 25-75%")
    elif cam != occ and m <= 0.75:
        problems.append(f"{cam} sees only {m:.0%} of {cls}; the unoccluded view must be clear")
if target and not any(cls == target for _, cls in med):
    problems.append(f"no visibility measured for the target {target}")
print("tier check: " + ("PASS" if not problems else "FAIL -- " + "; ".join(problems)))
EOF
}

cmd_process() {
  need_bins "$RUN" "$GT" "$VIS" "$ABL"
  [[ -d "$DAYDIR" ]] || die "no $DAYDIR"
  local summary=()
  for n in "$DAYDIR"/*/notes.yaml; do
    [[ -e "$n" ]] || continue
    local sdir name out session
    sdir=$(dirname "$n"); name=$(basename "$sdir"); out="$RUNDIR/$name"
    session=$(readlink -f "$sdir/session" || true)
    say "== process $name =="
    if [[ -z "$session" || ! -d "$session" ]]; then
      summary+=("$name: NO SESSION"); echo "no recorded session -- skipped"; continue
    fi
    mkdir -p "$out"
    if [[ -f "$out/metrics.sqlite" && $FORCE -eq 0 ]]; then
      echo "replay: $out/metrics.sqlite exists, reusing (--force to replay again)"
    else
      rm -f "$out/metrics.sqlite"
      # shellcheck disable=SC2086
      "$RUN" replay "$session" --pose foundationpose --db "$out/metrics.sqlite" \
        --print-every 50 $REPLAY_ARGS > "$out/replay.log" 2>&1 ||
        { tail -15 "$out/replay.log"; summary+=("$name: REPLAY FAILED"); continue; }
      grep -E "^=== latency|^pose_agent|^metrics: .* \(ok\)" "$out/replay.log" || true
    fi
    if ! "$GT" "$sdir/gt_stills" "$out/metrics.sqlite" --stills > "$out/gt.log" 2>&1; then
      tail -15 "$out/gt.log"; summary+=("$name: GROUND TRUTH REFUSED (see $out/gt.log)"); continue
    fi
    grep -E "cross-camera|REFUSED|rejected" "$out/gt.log" || true
    "$VIS" "$session" "$out/metrics.sqlite" --csv "$out/visibility.csv" \
      --overlay "$out/visibility_overlay" > "$out/visibility.log" 2>&1 ||
      { tail -10 "$out/visibility.log"; summary+=("$name: VISIBILITY FAILED"); continue; }
    sed -n '/^view \/ object/,/^$/p;/^WARNING/,$p' "$out/visibility.log"
    local tc; tc=$(tier_check "$n" "$out/visibility.csv"); echo "$tc"
    "$ABL" "$out/metrics.sqlite" > "$out/ablation.log" 2>&1 || true
    summary+=("$name: $(sed -n 's/^tier: //p' "$n") $tc")
  done
  say "== summary $DAY =="
  printf '%s\n' "${summary[@]}"
  echo
  echo "Per scene: $RUNDIR/<scene>/{gt,visibility,ablation}.log and visibility_overlay/."
  echo "A FAILED tier check means the scene is not the tier it was declared as. Reshoot it"
  echo "while the rig is still set up, or re-declare it -- never report it under the old tier."
}

case "$CMD" in
  begin) cmd_begin ;;
  scene) cmd_scene ;;
  end) cmd_end ;;
  list) cmd_list ;;
  process) cmd_process ;;
  *) die "unknown command '$CMD' (begin | scene | end | list | process)" ;;
esac

#!/bin/bash
# Usage: run.sh [N_topics=5] [width=3840] [height=2160] [rate_hz=30] [seconds=20] [mode=copy] [work_ms=0]
# Starts one subscriber and N publishers (one process per topic) and prints the subscriber's summary.
set -u
N=${1:-5}; W=${2:-3840}; H=${3:-2160}; RATE=${4:-30}; SECS=${5:-20}; MODE=${6:-copy}; WORK=${7:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
BIN=$(ros2 pkg prefix cuda_buffer_race_repro)/lib/cuda_buffer_race_repro
TOPICS=(); for i in $(seq 1 "$N"); do TOPICS+=("/race$i"); done
OUT=$(mktemp)
"$BIN/race_subscriber" $((SECS + 8)) "$MODE" "$WORK" "${TOPICS[@]}" > "$OUT" 2>&1 &
SUB=$!
sleep 2
PUBS=()
for t in "${TOPICS[@]}"; do "$BIN/race_publisher" "$t" "$W" "$H" "$RATE" "$SECS" > /dev/null 2>&1 & PUBS+=($!); done
wait "${PUBS[@]}" 2>/dev/null
wait "$SUB"
echo "== N=$N ${W}x${H} @ ${RATE} Hz, ${SECS} s, mode=$MODE, work_ms=$WORK"
grep -E "^(MISMATCH|SUMMARY|TOTAL)" "$OUT"
rm -f "$OUT"

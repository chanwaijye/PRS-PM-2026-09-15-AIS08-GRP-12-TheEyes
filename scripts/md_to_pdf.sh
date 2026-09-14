#!/usr/bin/env bash
# md_to_pdf.sh — Render a markdown file (with optional mermaid blocks) to PDF.
#
# Usage:
#   scripts/md_to_pdf.sh <input.md> [output.pdf]
#
# Examples:
#   scripts/md_to_pdf.sh ProjectReport/FinalReport.md
#   scripts/md_to_pdf.sh ProjectReport/PeerReview_ChanWaiJye.md /tmp/peer.pdf
#
# How it works:
#   1. Pre-render every ```mermaid``` block to a PNG via mmdc (mermaid-cli),
#      named <basename>-1.png, <basename>-2.png, ... next to the source.
#   2. Build a temp working copy of the markdown with each mermaid block
#      replaced by an `![…](png)` reference.
#   3. Render the working copy via `npx md-to-pdf`.
#
# Requirements (one-time):
#   npm install -g @mermaid-js/mermaid-cli   # provides `mmdc`
#   # md-to-pdf is invoked via `npx --yes`, so no global install needed
#
# Differences from PM1's copy of this script:
#   - Appends scripts/md_to_pdf.css: each major section and each diagram starts
#     a page, headings stay with their content, blocks never split.
#   - Symmetric 20 mm margins (md-to-pdf defaults to 20 mm left, 40 mm right)
#     and a "Page X of Y" footer.
#   - Removes the rendered diagram PNGs afterwards, so they do not land in the
#     submission folder beside the source.
#   - Feeds md-to-pdf an empty stdin. It reads stdin before looking at its file
#     argument and waits for EOF, so run from anything other than an
#     interactive terminal it hangs indefinitely.
#
# Notes:
#   - Uses --no-sandbox for puppeteer (required on Ubuntu 23.10+ AppArmor).

set -euo pipefail

INPUT="${1:?Usage: $0 <input.md> [output.pdf]}"
OUTPUT="${2:-${INPUT%.md}.pdf}"

if [[ ! -f "$INPUT" ]]; then
  echo "error: $INPUT not found" >&2
  exit 1
fi
command -v mmdc >/dev/null || { echo "error: mmdc not in PATH (npm i -g @mermaid-js/mermaid-cli)" >&2; exit 1; }
command -v npx  >/dev/null || { echo "error: npx not in PATH (install Node.js)" >&2; exit 1; }

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
INPUT_DIR=$(cd "$(dirname "$INPUT")" && pwd)
INPUT_BASE=$(basename "$INPUT" .md)
WORK_MD="${INPUT_DIR}/${INPUT_BASE}_pdf.md"
PUPPETEER_CFG=$(mktemp /tmp/puppeteer.XXXXXX.json)
echo '{"args":["--no-sandbox"]}' > "$PUPPETEER_CFG"

cleanup() { rm -f "$PUPPETEER_CFG" "$WORK_MD" "${INPUT_DIR}/${INPUT_BASE}"-[0-9]*.png; }
trap cleanup EXIT

# Count mermaid blocks (handles ```mermaid with or without trailing whitespace).
MERMAID_COUNT=$(grep -cE '^```mermaid[[:space:]]*$' "$INPUT" || true)

if [[ "$MERMAID_COUNT" -gt 0 ]]; then
  echo "Pre-rendering ${MERMAID_COUNT} mermaid block(s) to PNG…"
  mmdc -i "$INPUT" -o "${INPUT_DIR}/${INPUT_BASE}.png" \
       -w 3840 --scale 3 -b white -p "$PUPPETEER_CFG" >/dev/null

  # Substitute each mermaid block with its PNG reference (relative path so
  # md-to-pdf resolves it correctly from the working copy's directory).
  python3 - "$INPUT" "$WORK_MD" "$INPUT_BASE" <<'PY'
import re, sys, pathlib
src_path, dst_path, base = sys.argv[1], sys.argv[2], sys.argv[3]
src = pathlib.Path(src_path).read_text()
i = [0]
def sub(_m):
    i[0] += 1
    return f"![Diagram {i[0]}]({base}-{i[0]}.png)"
out = re.sub(r"```mermaid\n.*?\n```", sub, src, flags=re.DOTALL)
pathlib.Path(dst_path).write_text(out)
print(f"  Substituted {i[0]} mermaid block(s) in working copy")
PY
else
  cp "$INPUT" "$WORK_MD"
fi

echo "Rendering PDF via md-to-pdf…"
PDF_OPTIONS='{"format":"A4","printBackground":true,
  "margin":{"top":"20mm","right":"20mm","bottom":"22mm","left":"20mm"},
  "displayHeaderFooter":true,
  "headerTemplate":"<div></div>",
  "footerTemplate":"<div style=\"width:100%;text-align:center;font-size:8px;color:#666;font-family:sans-serif\">Page <span class=\"pageNumber\"></span> of <span class=\"totalPages\"></span></div>"}'

npx --yes md-to-pdf "$WORK_MD" \
    --launch-options "$(cat "$PUPPETEER_CFG")" \
    --pdf-options "$PDF_OPTIONS" \
    --css "$(cat "$SCRIPT_DIR/md_to_pdf.css")" < /dev/null 2>&1 | tail -3

GENERATED_PDF="${WORK_MD%.md}.pdf"
mv -f "$GENERATED_PDF" "$OUTPUT"

PAGES=$(pdfinfo "$OUTPUT" 2>/dev/null | awk '/^Pages:/ {print $2}')
SIZE=$(du -h "$OUTPUT" | awk '{print $1}')
echo "✓ ${OUTPUT}  (${PAGES:-?} pages, ${SIZE})"

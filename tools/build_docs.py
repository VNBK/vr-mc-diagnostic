#!/usr/bin/env python3
"""
build_docs.py — render docs/user_guide.md → user_guide.html → user_guide.pdf.

Self-contained: doesn't rely on vr-mc-sdk having a shared doc builder.
Auto-discovers every `docs/*.md` and renders both an HTML and a PDF
sibling for each. HTML uses the on-disk file's existing `<head>` block
when present (so hand-tuned CSS is preserved across rebuilds), or an
embedded minimal stylesheet on first render.

Requirements:
  - Python 3 with the `markdown` package (`pip install markdown`),
    OR a `markdown_py` CLI on PATH.
  - `google-chrome` / `chromium` on PATH for the PDF step.
    Pass --no-pdf to skip the PDF step (HTML-only render).

Usage:
  tools/build_docs.py                       # all docs/*.md → html + pdf
  tools/build_docs.py --no-pdf              # HTML only
  tools/build_docs.py --stem user_guide     # one stem only
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
DOCS_DIR  = REPO_ROOT / "docs"

# Fallback embedded stylesheet — used the first time a doc is rendered
# (no existing HTML file to scrape a head from). Keep the look close to
# the vr-hand guide so the two projects feel like one product family.
DEFAULT_HEAD = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>{title}</title>
<style>
  @page { size: A4; margin: 18mm 16mm; }
  body { font-family: "Source Sans Pro", "Helvetica Neue", Arial, sans-serif;
         font-size: 10.5pt; line-height: 1.45; color: #1c1c1c; max-width: 100%; }
  h1 { font-size: 22pt; margin-top: 0; border-bottom: 2px solid #444; padding-bottom: 6px; }
  h2 { font-size: 15pt; margin-top: 26px; color: #003c80;
       border-bottom: 1px solid #ccc; padding-bottom: 3px; }
  h3 { font-size: 12pt; margin-top: 18px; color: #1f3a68; }
  h2, h3 { page-break-after: avoid; }
  p, ul, ol, table, pre, blockquote { page-break-inside: avoid; }
  code { font-family: "JetBrains Mono", "Fira Mono", "Consolas", monospace;
         font-size: 9.5pt; background: #f3f4f6; padding: 1px 4px;
         border-radius: 3px; color: #b03060; }
  pre { background: #f7f7f9; border: 1px solid #e2e2e6; border-radius: 4px;
        padding: 10px 12px; font-size: 9pt; line-height: 1.35; overflow-x: auto; }
  pre code { background: transparent; color: #1c1c1c; padding: 0; font-size: 9pt; }
  table { border-collapse: collapse; margin: 10px 0; width: 100%; font-size: 9.8pt; }
  th, td { border: 1px solid #ccc; padding: 5px 8px; text-align: left; vertical-align: top; }
  th { background: #eef2f7; font-weight: 600; }
  tr:nth-child(even) td { background: #fafbfc; }
  blockquote { border-left: 4px solid #003c80; background: #f0f5fa;
               margin: 10px 0; padding: 8px 14px; color: #1f3a68; font-style: italic; }
  hr { border: none; border-top: 1px dashed #aaa; margin: 22px 0; }
</style>
</head>
<body>
"""


def render_body(md_path: Path) -> str:
    """Render a markdown file to an HTML <body> fragment."""
    src = md_path.read_text(encoding="utf-8")
    # Try Python `markdown` first (better extension support); fall back
    # to a markdown_py CLI on PATH.
    try:
        import markdown  # noqa: import-not-found in some envs
        return markdown.markdown(
            src,
            extensions=["extra", "tables", "fenced_code", "toc"],
            output_format="html5",
        )
    except ImportError:
        cli = shutil.which("markdown_py") or shutil.which("markdown")
        if not cli:
            sys.exit(
                "error: install Python `markdown` (pip install markdown) "
                "or put a markdown_py CLI on PATH"
            )
        r = subprocess.run([cli, "-x", "extra", "-x", "tables",
                            "-x", "fenced_code", "-x", "toc",
                            str(md_path)],
                           check=True, capture_output=True, text=True)
        return r.stdout


def write_html(md_path: Path, html_path: Path) -> None:
    body = render_body(md_path)
    if html_path.exists():
        # Preserve the existing <head> + everything up to <body>.
        old = html_path.read_text(encoding="utf-8")
        m = re.match(r"(.*?<body>\n?)", old, re.DOTALL)
        if m:
            head = m.group(1)
        else:
            head = DEFAULT_HEAD.format(title=md_path.stem)
    else:
        head = DEFAULT_HEAD.format(title=md_path.stem)
    html_path.write_text(head + body + "\n\n</body>\n</html>\n",
                         encoding="utf-8")
    print(f"wrote {html_path}  ({html_path.stat().st_size} bytes)")


def write_pdf(html_path: Path, pdf_path: Path) -> None:
    chrome = (shutil.which("google-chrome")
              or shutil.which("chrome")
              or shutil.which("chromium")
              or shutil.which("chromium-browser"))
    if not chrome:
        sys.exit("error: need google-chrome / chromium on PATH for PDF render"
                 " (use --no-pdf to skip)")
    cmd = [
        chrome, "--headless=new", "--disable-gpu", "--no-sandbox",
        "--no-pdf-header-footer",
        f"--print-to-pdf={pdf_path}",
        html_path.absolute().as_uri(),
    ]
    subprocess.run(cmd, check=True, capture_output=True)
    print(f"wrote {pdf_path}  ({pdf_path.stat().st_size} bytes)")


def discover_stems() -> list[str]:
    return sorted(p.stem for p in DOCS_DIR.glob("*.md"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--stem", help="render exactly this stem "
                                   "(skip auto-discovery)")
    ap.add_argument("--no-pdf", action="store_true",
                    help="skip the PDF step (HTML only)")
    args = ap.parse_args()

    stems = [args.stem] if args.stem else discover_stems()
    if not stems:
        sys.exit(f"error: no markdown files in {DOCS_DIR}")

    rc = 0
    for stem in stems:
        md   = DOCS_DIR / f"{stem}.md"
        html = DOCS_DIR / f"{stem}.html"
        pdf  = DOCS_DIR / f"{stem}.pdf"
        if not md.exists():
            print(f"skip {stem}: no .md")
            continue
        print(f"--- rendering {stem} ---")
        write_html(md, html)
        if not args.no_pdf:
            write_pdf(html, pdf)
    return rc


if __name__ == "__main__":
    sys.exit(main())

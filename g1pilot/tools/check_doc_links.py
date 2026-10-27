#!/usr/bin/env python3
"""Prueft Markdown-Links der Doku auf tote Ziele (Datei und Anker).

Aufruf aus g1pilot/:  python3 tools/check_doc_links.py
Prueft alle *.md im Repo (ohne die mitgelieferten Unitree-Baeume) und
zusaetzlich Verweise der Form ``docs/<Name>.md`` in Code, Skripten und
Launch-Dateien. Anker werden wie bei GitHub aus den Ueberschriften gebildet.
Exit-Code 1, wenn etwas nicht aufloest.
"""
import re
import sys
import unicodedata
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]          # Repo-Wurzel
# Fremdcode ueberspringen; scene_editor/ ist eigener Code und wird geprueft.
SKIP = ("unitree_sdk2_python/", "unitree_ros2/", "reference/", ".git/")
SKIP_MUJOCO_EXCEPT = "unitree_mujoco/scene_editor/"
LINK = re.compile(r"(?<!!)\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)|!\[[^\]]*\]\(([^)\s]+)\)")
# docs/<alter oder neuer Name>.md sowie nackte neue Namen (z. B. in g1_gui.py: DOCS_DIR / "11-A-Controller.md")
CODE_REF = re.compile(r"(?:docs/([0-9][0-9A-Za-z_\-]*\.md)|\b([0-9]{2}-[AT]-[A-Za-z0-9\-]+\.md|[0-9]-[A-Z][A-Za-z0-9\-]+\.md))(#[\w\-]+)?")
CODE_SUFFIX = {".py", ".sh", ".js", ".yml", ".yaml", ".html", ".txt", ".cfg", ""}


def files(pred):
    for p in ROOT.rglob("*"):
        rel = p.relative_to(ROOT).as_posix()
        if rel.startswith(SKIP) or (rel.startswith("unitree_mujoco/")
                                    and not rel.startswith(SKIP_MUJOCO_EXCEPT)):
            continue
        if p.is_file() and pred(p):
            yield p


def slug(title: str) -> str:
    title = re.sub(r"`", "", title).strip().lower()
    out = []
    for ch in title:
        cat = unicodedata.category(ch)
        if ch in " -":
            out.append("-" if ch == " " else ch)
        elif ch == "_" or cat[0] in "LN":
            out.append(ch)
    return "".join(out)


def anchors(md: Path) -> set:
    seen, res, fence = {}, set(), False
    for line in md.read_text(encoding="utf-8").splitlines():
        if line.lstrip().startswith("```"):
            fence = not fence
            continue
        m = None if fence else re.match(r"^#{1,6}\s+(.*?)\s*#*\s*$", line)
        if m:
            s = slug(m.group(1))
            n = seen.get(s, 0)
            res.add(s if n == 0 else f"{s}-{n}")
            seen[s] = n + 1
    return res


def strip_code(text: str) -> str:
    text = re.sub(r"```.*?```", "", text, flags=re.S)
    return re.sub(r"`[^`\n]*`", "", text)


def main() -> int:
    cache, bad = {}, []

    def check(src: Path, target: str, base: Path):
        if re.match(r"^[a-z]+:", target) or target.startswith("mailto:"):
            return
        path, _, frag = target.partition("#")
        dest = (base / path).resolve() if path else src
        if not dest.exists():
            bad.append(f"{src.relative_to(ROOT)}: Ziel fehlt: {target}")
            return
        if frag and dest.suffix == ".md":
            if dest not in cache:
                cache[dest] = anchors(dest)
            if frag.lower() not in cache[dest]:
                bad.append(f"{src.relative_to(ROOT)}: Anker fehlt: {target}")

    for md in files(lambda p: p.suffix == ".md"):
        for m in LINK.finditer(strip_code(md.read_text(encoding="utf-8"))):
            check(md, m.group(1) or m.group(2), md.parent)

    docs = ROOT / "g1pilot" / "docs"
    for src in files(lambda p: p.suffix in CODE_SUFFIX and p.suffix != ".md"
                     and "docs" not in p.relative_to(ROOT).parts[:2]):
        try:
            text = src.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for m in CODE_REF.finditer(text):
            check(src, (m.group(1) or m.group(2)) + (m.group(3) or ""), docs)

    for b in bad:
        print(b)
    print(f"{len(bad)} tote Verweise" if bad else "Alle Doku-Verweise loesen auf.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())

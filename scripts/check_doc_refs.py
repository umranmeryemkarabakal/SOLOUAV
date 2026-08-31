#!/usr/bin/env python3
"""
Dokümanlardaki `path/to/file.ext:L120-L145` referanslarını doğrular.

Kontroller:
  - Dosya var mı?
  - Satır aralığı dosyanın uzunluğu içinde mi?
  - Aralık geçerli mi (başlangıç <= bitiş)?

DOCS_SPEC §3'ün deterministik kapısı. Hızlı, ucuz ve en sık hatayı yakalar:
kod değişince bayatlayan referanslar. Agent denetimi yerine geçmez —
referansın DOĞRU YERİ gösterip göstermediğini technical-auditor kontrol eder.

Kullanım:
    python scripts/check_doc_refs.py                    # docs/ tamamı
    python scripts/check_doc_refs.py docs/attitude/     # tek dizin
    python scripts/check_doc_refs.py --json             # makine okunur

Çıkış kodu 0 = temiz, 1 = kırık referans var.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, asdict
from pathlib import Path

# `src/control/attitude.cpp:L120-L145` veya `src/x.py:L42`
# Backtick içinde veya çıplak halde eşleşir.
REF_PATTERN = re.compile(
    r"(?P<path>[A-Za-z0-9_./\-]+\.[A-Za-z0-9_+]+)"
    r":L(?P<start>\d+)"
    r"(?:-L?(?P<end>\d+))?"
)

# Referans taranmayacak bloklar: fenced code blocks
FENCE_PATTERN = re.compile(r"^\s*```")

SKIP_DIRS = {
    ".git", "node_modules", "__pycache__", ".venv", "venv", "build", "dist",
    "templates",  # yer tutucu referanslar içerir
}


@dataclass
class Problem:
    doc: str
    line: int
    ref: str
    reason: str


def strip_code_blocks(text: str) -> list[tuple[int, str]]:
    """Fenced code block içindeki satırları eler. (satır_no, içerik) döner."""
    out: list[tuple[int, str]] = []
    in_fence = False
    for i, line in enumerate(text.splitlines(), start=1):
        if FENCE_PATTERN.match(line):
            in_fence = not in_fence
            continue
        if not in_fence:
            out.append((i, line))
    return out


def count_lines(path: Path) -> int:
    try:
        with path.open("rb") as fh:
            return sum(1 for _ in fh)
    except OSError:
        return -1


def check_doc(doc: Path, repo_root: Path) -> tuple[list[Problem], int]:
    problems: list[Problem] = []
    checked = 0

    try:
        text = doc.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        problems.append(Problem(str(doc), 0, "", f"okunamadı: {exc}"))
        return problems, 0

    for lineno, line in strip_code_blocks(text):
        for m in REF_PATTERN.finditer(line):
            rel = m.group("path")
            start = int(m.group("start"))
            end = int(m.group("end")) if m.group("end") else start
            ref = m.group(0)
            checked += 1

            target = (repo_root / rel).resolve()

            # repo dışına çıkan yol
            try:
                target.relative_to(repo_root.resolve())
            except ValueError:
                problems.append(Problem(str(doc), lineno, ref, "yol repo kökü dışında"))
                continue

            if not target.is_file():
                problems.append(Problem(str(doc), lineno, ref, "dosya bulunamadı"))
                continue

            if start < 1:
                problems.append(Problem(str(doc), lineno, ref, "satır numarası < 1"))
                continue

            if end < start:
                problems.append(
                    Problem(str(doc), lineno, ref, f"geçersiz aralık ({start} > {end})")
                )
                continue

            total = count_lines(target)
            if total < 0:
                problems.append(Problem(str(doc), lineno, ref, "hedef dosya okunamadı"))
                continue

            if end > total:
                problems.append(
                    Problem(
                        str(doc),
                        lineno,
                        ref,
                        f"satır aralığı dosyayı aşıyor (dosya {total} satır)",
                    )
                )

    return problems, checked


def iter_docs(root: Path):
    for p in sorted(root.rglob("*.md")):
        if any(part in SKIP_DIRS for part in p.parts):
            continue
        yield p


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("target", nargs="?", default="docs", help="taranacak dizin veya dosya")
    ap.add_argument("--repo-root", default=".", help="kod referanslarının kökü")
    ap.add_argument("--json", action="store_true", help="JSON çıktı")
    args = ap.parse_args()

    repo_root = Path(args.repo_root).resolve()
    target = Path(args.target)

    if not target.exists():
        print(f"hata: '{target}' bulunamadı", file=sys.stderr)
        return 2

    docs = [target] if target.is_file() else list(iter_docs(target))

    all_problems: list[Problem] = []
    total_refs = 0
    for doc in docs:
        problems, checked = check_doc(doc, repo_root)
        all_problems.extend(problems)
        total_refs += checked

    if args.json:
        print(
            json.dumps(
                {
                    "docs_scanned": len(docs),
                    "refs_checked": total_refs,
                    "problems": [asdict(p) for p in all_problems],
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 1 if all_problems else 0

    print(f"{len(docs)} doküman tarandı, {total_refs} kod referansı kontrol edildi.")

    if not all_problems:
        print("✅ Tüm kod referansları geçerli.")
        return 0

    print(f"\n❌ {len(all_problems)} sorun bulundu:\n")
    current = None
    for p in all_problems:
        if p.doc != current:
            current = p.doc
            print(f"  {p.doc}")
        print(f"    satır {p.line}: {p.ref}  →  {p.reason}")
    return 1


if __name__ == "__main__":
    # `| head` gibi borularda BrokenPipeError yerine sessiz çıkış
    try:
        import signal
        signal.signal(signal.SIGPIPE, signal.SIG_DFL)
    except (ImportError, AttributeError, ValueError):
        pass  # Windows'ta SIGPIPE yok
    sys.exit(main())

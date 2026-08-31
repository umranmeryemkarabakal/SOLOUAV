#!/usr/bin/env python3
"""
Doküman frontmatter'larını ve YERLEŞİMİNİ DOCS_SPEC'e ve manifest.yaml'a
karşı doğrular.

Dosya adı ve dizin şeması SABİT DEĞİLDİR — manifest.yaml içindeki
meta.layout.dir_template / file_template alanlarından okunur. Böylece
düzen kararı kod tabanı görüldükten sonra (Faz 1.5) verilebilir ve
doğrulama yine de deterministik kalır.

Kontroller:
  - Zorunlu frontmatter alanları (topic, level, title, status, updated)
  - level ve status geçerli mi
  - topic manifest'te var mı, level o konu için tanımlı mı
  - DOSYA DOĞRU YERDE Mİ (layout şablonundan üretilen beklenen yol)
  - grouped/mirror düzeninde gerekli alanlar dolu mu
  - L3 için sources alanı var mı
  - Yetim dosya (manifest'te olmayan) var mı
  - Kapsam raporu: hangi konu/seviye henüz yazılmamış
  - (OVERVIEW.md gibi manifest konusu olmayan kök dokümanlar atlanır)

Kullanım:
    python scripts/check_frontmatter.py
    python scripts/check_frontmatter.py --docs docs --manifest docs/_meta/manifest.yaml
    python scripts/check_frontmatter.py --show-layout    # beklenen yolları listele

Çıkış kodu 0 = temiz, 1 = sorun var.
PyYAML yoksa gömülü minimal ayrıştırıcıya düşer.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

VALID_LEVELS = {"L1", "L2", "L3"}
VALID_STATUS = {"draft", "reviewed", "verified"}
REQUIRED = ["topic", "level", "title", "status", "updated"]

# Manifest konusu OLMAYAN, kök seviyede duran serbest dokümanlar.
# Bunlar bir konunun L1/L2/L3'ü değildir; künye/yerleşim kuralları
# onlara uygulanmaz, ama silinmezler — kapı yalnızca atlar.
NON_TOPIC_DOCS = {"OVERVIEW.md"}

DEFAULT_LAYOUT = {
    "pattern": "by-topic",
    "dir_template": "docs/{topic}/",
    "file_template": "{level}-{topic}.md",
}

try:
    import yaml  # type: ignore
    HAVE_YAML = True
except ImportError:
    HAVE_YAML = False


# --------------------------------------------------------------------------
# Ayrıştırma
# --------------------------------------------------------------------------

def parse_frontmatter(text: str) -> dict | None:
    if not text.startswith("---"):
        return None
    end = text.find("\n---", 3)
    if end == -1:
        return None
    block = text[3:end]

    if HAVE_YAML:
        try:
            data = yaml.safe_load(block)
            return data if isinstance(data, dict) else None
        except Exception:
            return None

    data: dict = {}
    key = None
    for raw in block.splitlines():
        if not raw.strip() or raw.strip().startswith("#"):
            continue
        if raw.lstrip().startswith("-") and key:
            data.setdefault(key, [])
            if isinstance(data[key], list):
                data[key].append(raw.lstrip()[1:].strip())
            continue
        if ":" in raw:
            k, _, v = raw.partition(":")
            key, v = k.strip(), v.strip()
            if v.startswith("[") and v.endswith("]"):
                inner = v[1:-1].strip()
                data[key] = [x.strip() for x in inner.split(",") if x.strip()] if inner else []
            elif v:
                data[key] = v.strip("\"'")
            else:
                data[key] = []
    return data


def load_manifest(path: Path) -> tuple[dict, dict]:
    """(layout, topics) döner. topics: id -> {levels, group, module}"""
    if not path.exists():
        return dict(DEFAULT_LAYOUT), {}

    text = path.read_text(encoding="utf-8")

    if HAVE_YAML:
        try:
            data = yaml.safe_load(text) or {}
            layout = {**DEFAULT_LAYOUT, **(data.get("meta", {}).get("layout") or {})}
            topics = {
                t["id"]: {
                    "levels": list(t.get("levels", [])),
                    "group": t.get("group"),
                    "module": t.get("module"),
                }
                for t in data.get("topics", [])
                if isinstance(t, dict) and "id" in t
            }
            return layout, topics
        except Exception:
            pass

    # Fallback ayrıştırıcı
    layout = dict(DEFAULT_LAYOUT)
    for field in ("pattern", "dir_template", "file_template"):
        m = re.search(rf"^\s*{field}:\s*(.+)$", text, re.MULTILINE)
        if m:
            layout[field] = m.group(1).strip().strip("\"'")

    topics: dict = {}
    cur = None
    for line in text.splitlines():
        s = line.strip()
        m = re.match(r"-\s*id:\s*(\S+)", s)
        if m:
            cur = m.group(1).strip("\"'")
            topics[cur] = {"levels": [], "group": None, "module": None}
            continue
        if not cur:
            continue
        m = re.match(r"levels:\s*\[(.*)\]", s)
        if m:
            topics[cur]["levels"] = [x.strip() for x in m.group(1).split(",") if x.strip()]
            continue
        m = re.match(r"(group|module):\s*(\S+)", s)
        if m:
            topics[cur][m.group(1)] = m.group(2).strip("\"'")
    return layout, topics


# --------------------------------------------------------------------------
# Yerleşim
# --------------------------------------------------------------------------

def expected_path(layout: dict, topic: str, level: str, info: dict) -> tuple[Path | None, str | None]:
    """Şablondan beklenen yolu üretir. (yol, hata) döner."""
    subs = {
        "topic": topic,
        "level": level,
        "group": info.get("group") or "",
        "module": info.get("module") or "",
    }

    combined = layout["dir_template"] + layout["file_template"]
    for var in ("group", "module"):
        if "{" + var + "}" in combined and not subs[var]:
            return None, f"layout '{layout['pattern']}' '{var}' alanı gerektiriyor ama manifest'te boş"

    try:
        d = layout["dir_template"].format(**subs)
        f = layout["file_template"].format(**subs)
    except KeyError as exc:
        return None, f"layout şablonunda bilinmeyen değişken: {exc}"

    return Path(d) / f, None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--docs", default="docs")
    ap.add_argument("--manifest", default="docs/_meta/manifest.yaml")
    ap.add_argument("--show-layout", action="store_true",
                    help="beklenen tüm yolları listele ve çık")
    args = ap.parse_args()

    docs_root = Path(args.docs)
    layout, topics = load_manifest(Path(args.manifest))

    print(f"Düzen: {layout['pattern']}  →  {layout['dir_template']}{layout['file_template']}")

    if args.show_layout:
        print()
        for tid, info in topics.items():
            for lv in info["levels"]:
                p, err = expected_path(layout, tid, lv, info)
                print(f"  {p}" if p else f"  ❌ {tid}/{lv}: {err}")
        return 0

    if not docs_root.exists():
        print(f"hata: '{docs_root}' bulunamadı", file=sys.stderr)
        return 2

    problems: list[str] = []
    seen: set[tuple[str, str]] = set()
    count = 0

    skipped: list[str] = []

    for doc in sorted(docs_root.rglob("*.md")):
        if "_meta" in doc.parts:
            continue
        if doc.parent == docs_root and doc.name in NON_TOPIC_DOCS:
            skipped.append(doc.name)
            continue
        count += 1

        fm = parse_frontmatter(doc.read_text(encoding="utf-8"))
        if fm is None:
            problems.append(f"{doc}: frontmatter yok veya ayrıştırılamadı")
            continue

        for field in REQUIRED:
            if not fm.get(field):
                problems.append(f"{doc}: zorunlu alan eksik → {field}")

        level = str(fm.get("level", ""))
        topic = str(fm.get("topic", ""))
        status = str(fm.get("status", ""))

        if level and level not in VALID_LEVELS:
            problems.append(f"{doc}: geçersiz level '{level}'")
        if status and status not in VALID_STATUS:
            problems.append(f"{doc}: geçersiz status '{status}'")

        if topic and topics and topic not in topics:
            problems.append(f"{doc}: topic '{topic}' manifest'te yok (yetim dosya)")
        elif topic and level and topics:
            info = topics[topic]
            if level not in info["levels"]:
                problems.append(
                    f"{doc}: '{topic}' için {level} manifest'te tanımlı değil "
                    f"(beklenen: {info['levels']})"
                )
            # Yerleşim kontrolü — sabit regex yerine şablondan
            exp, err = expected_path(layout, topic, level, info)
            if err:
                problems.append(f"{doc}: {err}")
            elif exp and doc != exp:
                problems.append(f"{doc}: yanlış yerde → beklenen '{exp}'")

        if level == "L3" and not fm.get("sources"):
            problems.append(f"{doc}: L3 dokümanında 'sources' alanı boş")

        if topic and level:
            seen.add((topic, level))

    print(f"{count} doküman tarandı.")
    if skipped:
        print(f"Atlandı (manifest konusu değil): {', '.join(skipped)}")

    if problems:
        print(f"\n❌ {len(problems)} sorun:\n")
        for p in problems:
            print(f"  {p}")
    else:
        print("✅ Frontmatter ve yerleşim geçerli.")

    if topics:
        missing = [
            f"{t}/{lv}" for t, i in topics.items() for lv in i["levels"] if (t, lv) not in seen
        ]
        total = sum(len(i["levels"]) for i in topics.values())
        print(f"\nKapsam: {total - len(missing)}/{total} doküman yazıldı.")
        if missing:
            print("Eksik: " + ", ".join(missing))

    return 1 if problems else 0


if __name__ == "__main__":
    # `| head` gibi borularda BrokenPipeError yerine sessiz çıkış
    try:
        import signal
        signal.signal(signal.SIGPIPE, signal.SIG_DFL)
    except (ImportError, AttributeError, ValueError):
        pass  # Windows'ta SIGPIPE yok
    sys.exit(main())

#!/usr/bin/env python3
"""backfill_tasks.py - SL-0138: propose (dry run) or insert (--write)
the `tasks:` key in run files, from the TeamEngage ids their leading
header comment already names.

    python3 tools/backfill_tasks.py            # dry run: report only
    python3 tools/backfill_tasks.py --write    # insert tasks: [...]
    python3 tools/backfill_tasks.py --write --exclude <substr> ...

For every *.yaml under tests/ and scenarios/ only the leading header
comment block is read - the 'what it proves' paragraph before the first
YAML key - never the body (a mid-file id is 'related to', not 'this
file proves it'). Ids in the new TeamEngage form ^[A-Z]{2,4}-[0-9]{4}$
(SL-0099, MP-0139, DE-0025 ...) become proposed tasks. Old-style refs
(TASK-13, TASK-P4F-07, P4B-10, P3-07 ...) are reported separately and
never converted - the human maps those by hand if wanted.

--write is line-based insertion, not a yaml round-trip: a
`tasks: [ID1, ID2]` line lands after the `description:` key's block
(falling back to after `name:`), comments and key order untouched, so
git diff adds lines only. Files already carrying a `tasks:` key are
skipped - reruns are idempotent. A file naming more than one new-style
id is listed under AMBIGUOUS: those are usually 'related to' clusters
worth a human glance; --exclude <substring> drops named files from the
write set.
"""

import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or
                 os.path.join(ROOT, "var"), "pycache"))

ROOTS = ("tests", "scenarios")

# new TeamEngage ids (convertible) vs every other dashed id-ish token
# ending in digits (old-style refs: reported, never converted)
NEW_RE = re.compile(r"\b[A-Z]{2,4}-[0-9]{4}\b")
TOKEN_RE = re.compile(r"\b[A-Z0-9]{2,}(?:-[A-Z0-9]+)+\b")


def header_block(path):
    """The leading comment block only: blank lines allowed inside it,
    stops at the first non-comment non-blank line (the first yaml
    key)."""
    out = []
    with open(path, errors="replace") as fh:
        for line in fh:
            s = line.strip()
            if not s:
                out.append("")
                continue
            if s.startswith("#"):
                out.append(s)
                continue
            break
    return out


def ids_in(lines):
    """(new-style ids, old-style tokens) found in the header lines -
    each unique, order of first appearance."""
    new, old = [], []
    for tok in TOKEN_RE.findall("\n".join(lines)):
        if not tok[-1].isdigit():
            continue
        if NEW_RE.fullmatch(tok):
            if tok not in new:
                new.append(tok)
        elif tok not in old:
            old.append(tok)
    return new, old


def insert_point(lines):
    """Line index AFTER which tasks: goes: the end of the
    `description:` block when present (its indented continuation
    lines included, so a `|` block stays whole), else the `name:`
    line."""
    desc_i = name_i = None
    for i, line in enumerate(lines):
        if re.match(r"^description\s*:", line):
            desc_i = i
        elif re.match(r"^name\s*:", line):
            name_i = i
        if desc_i is not None and name_i is not None:
            break
    if desc_i is not None:
        j = desc_i + 1
        while j < len(lines) and \
                (lines[j].startswith((" ", "\t")) or not lines[j].strip()):
            j += 1
        return j - 1
    return name_i


def main(argv):
    write = "--write" in argv
    excludes = []
    for i, a in enumerate(argv[:-1]):
        if a == "--exclude":
            excludes.append(argv[i + 1])
    files = sorted(f for r in ROOTS for f in glob.glob(
        os.path.join(ROOT, r, "**", "*.yaml"), recursive=True))
    proposed, ambiguous, already, none, old_refs = [], [], [], [], []
    for f in files:
        rel = os.path.relpath(f, ROOT)
        text = open(f, errors="replace").read().splitlines()
        if re.search(r"^tasks\s*:", "\n".join(text), re.M):
            already.append(rel)
            continue
        new, old = ids_in(header_block(f))
        if old:
            old_refs.append((rel, old))
        if not new:
            none.append(rel)
            continue
        (ambiguous if len(new) > 1 else proposed).append((rel, new))
    print(f"run files: {len(files)}   already tagged: {len(already)}   "
          f"no new-style id: {len(none)}")
    print(f"\nPROPOSED ({len(proposed)}):")
    for rel, ids in proposed:
        print(f"  {rel:64s} tasks: {ids}")
    print(f"\nAMBIGUOUS - >1 new-style id, human check ({len(ambiguous)}):")
    for rel, ids in ambiguous:
        print(f"  {rel:64s} tasks: {ids}")
    print(f"\nOLD-STYLE refs (reported, NOT converted: {len(old_refs)}):")
    for rel, ids in old_refs:
        print(f"  {rel:64s} {ids}")
    if not write:
        print("\ndry run - rerun with --write (optionally "
              "--exclude <substr>) to insert")
        return 0
    changed = skipped = 0
    for rel, ids in proposed + ambiguous:
        if any(x in rel for x in excludes):
            skipped += 1
            continue
        lines = open(os.path.join(ROOT, rel), errors="replace") \
            .read().splitlines()
        i = insert_point(lines)
        if i is None:
            print(f"  !! no name:/description: anchor in {rel} - skipped")
            skipped += 1
            continue
        lines.insert(i + 1, "tasks: [%s]" % ", ".join(ids))
        open(os.path.join(ROOT, rel), "w").write("\n".join(lines) + "\n")
        changed += 1
        print(f"  wrote {rel}: tasks: {ids}")
    print(f"\n{changed} file(s) updated, {skipped} excluded/skipped")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

#!/bin/bash
# fetch-repo.sh <git-url> <ref> <dest> [submodules...]
#
# Shallow-fetch one ref (branch, tag or full commit sha) into <dest>.
# GitHub serves fetch-by-sha, so pinning ARG refs to commit hashes keeps
# the image reproducible while staying shallow. Any further args are
# submodule paths initialised before .git is dropped.
set -euo pipefail

url="$1"; ref="$2"; dest="$3"; shift 3

mkdir -p "$dest"
git -C "$dest" init -q
git -C "$dest" remote add origin "$url"
git -C "$dest" fetch -q --depth 1 origin "$ref"
git -C "$dest" checkout -q --detach FETCH_HEAD
if [ "$#" -gt 0 ]; then
    git -C "$dest" submodule update --init --depth 1 \
        --recommend-shallow "$@"
fi
rm -rf "$dest/.git"

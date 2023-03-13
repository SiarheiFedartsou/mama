#!/usr/bin/env bash
set -e pipefail


OS=$(uname)
NPROC=$(getconf _NPROCESSORS_ONLN)

# Discover clang-format
if type clang-format-15 2> /dev/null ; then
    CLANG_FORMAT=clang-format-15
elif type clang-format 2> /dev/null ; then
    # Clang format found, but need to check version
    CLANG_FORMAT=clang-format
    V=$(clang-format --version)
    if [[ $V != *15.0* ]] ; then
        echo "clang-format is not 15.0 (returned ${V})"
        exit 1
    fi
else
    echo "No appropriate clang-format found (expected clang-format-15, or clang-format)"
    exit 1
fi

find src server test -type f -name '*.hpp' -o -name '*.cc' \
  | xargs -I{} -P ${NPROC} ${CLANG_FORMAT} -i -style=file {}

if [[ $(git ls-files --modified) ]]; then
    echo "Format failed:"
    git diff
    exit 1
else
    exit 0
fi
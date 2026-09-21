#!/usr/bin/env bash
# Run all static checks. Requires a configured build dir with compile_commands.json.
#   cmake -S . -B build -G Ninja
set -euo pipefail
cd "$(dirname "$0")/.."

SRC=(tools apps)

echo "==> clang-format"
find "${SRC[@]}" \( -name '*.hpp' -o -name '*.cpp' \) -print0 |
  xargs -0 clang-format --dry-run --Werror

echo "==> clang-tidy"
run-clang-tidy -p build -quiet

echo "==> cppcheck"
cppcheck --project=build/compile_commands.json

echo "==> all checks passed"

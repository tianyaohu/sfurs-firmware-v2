#!/bin/sh
# Shared setup keeps standalone builds and the full build on the same toolchain.
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1 &&
    [ -d /tmp/opencode/cm7-toolchain/root/usr/bin ]; then
  PATH=/tmp/opencode/cm7-toolchain/root/usr/bin:$PATH
  export PATH
fi
dry_run=0
quiet=0
fail() { echo "$*" >&2; exit 2; }
run() {
  if [ "$dry_run" = 1 ]; then
    printf 'DRY RUN:'
    printf ' [%s]' "$@"
    printf '\n'
  elif [ "$quiet" = 1 ]; then
    "$@" >/dev/null
  else
    "$@"
  fi
}
wheel_valid() {
  case "$1" in NE|SE|SW|NW) return 0 ;; *) return 1 ;; esac
}
shift_valid() {
  shift_value=$1
  case "$shift_value" in -*) shift_value=${shift_value#-} ;; +*) shift_value=${shift_value#+} ;; esac
  case "$shift_value" in ''|*[!0-9]*) return 1 ;; esac
  # Decimal conversion avoids shell arithmetic treating leading zeroes as octal.
  shift_value=$(printf '%s\n' "$shift_value" | sed 's/^0*//')
  [ ${#shift_value} -le 3 ] && [ "${shift_value:-0}" -lt 360 ]
}
require_toolchain() {
  [ "$dry_run" = 1 ] || command -v arm-none-eabi-gcc >/dev/null 2>&1 ||
    fail 'arm-none-eabi-gcc not found on PATH or in the fallback toolchain'
}

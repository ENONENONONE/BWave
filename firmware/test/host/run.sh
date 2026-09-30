#!/bin/sh
# Host unit tests for the DSP maths and pipeline. Needs a C compiler and libm.
set -e
here=$(cd "$(dirname "$0")" && pwd)
main="$here/../../main"
out="${TMPDIR:-/tmp}"
flags="-std=gnu11 -O2 -Wall -Wextra -Werror"

${CC:-cc} $flags -I"$main" \
    "$here/test_resonance.c" "$main/bwave_resonance.c" -lm -o "$out/bwave_test_resonance"
"$out/bwave_test_resonance"

${CC:-cc} $flags -Wno-unused-function -I"$here/stubs" -I"$main" \
    "$here/test_dsp_pipeline.c" "$main/bwave_resonance.c" -lm -o "$out/bwave_test_dsp"
"$out/bwave_test_dsp"

#!/usr/bin/env bash
set -euo pipefail
S28_PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
S28_RESULT_ROOT=${S28_RESULT_ROOT:-${S28_PROJECT_DIR}/out}
export S28_RESULT_ROOT
ARGS=(--case "${CASE_ID}")
if [[ "${S28_NO_VCD:-0}" == "1" ]]; then
    ARGS+=(--no-vcd)
fi
exec "${S28_PYTHON:-python3}" "${S28_PROJECT_DIR}/test/run_s28.py" "${ARGS[@]}" "$@"

#!/bin/bash

TMP_FILE="/tmp/hpa_test_out.txt"

function handle_output {
    rm -f "${TMP_FILE}"
    while read -r line
    do
        echo "${line}" >> "${TMP_FILE}"

        # make sure line is not longer than the terminal width
        width=$(tput cols) # read this again in case the terminal was resized
        width=$((width - 3)) # leave space for the "..."
        TRIMMED_LINE=$(echo "> ${line}" | sed "s/\(.\{${width}\}\).*/\1.../")
        echo -en "\033[2K\r${TRIMMED_LINE}"
        tput init # trimmed line may have messed up coloring
    done
    echo -ne "\033[2K\r";
}

function try_silent {
    echo "Running $*"
    unbuffer "$@" | handle_output
    if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
        cat "${TMP_FILE}"
        return 1
    fi
}

BASE_DIR="$(realpath "$(dirname "$0")")"
OUT_DIRS="${BASE_DIR}/test_dirs"
MSRV_DIR="${OUT_DIRS}/msrv"
MIN_VERSIONS_DIR="${OUT_DIRS}/min_versions"

for dir in "${MSRV_DIR}" "${MIN_VERSIONS_DIR}"; do
    [[ -d "${dir}" ]] && continue
    mkdir -p "${dir}"
    ln -s "${BASE_DIR}/Cargo.toml" "${dir}/Cargo.toml"
    ln -s "${BASE_DIR}/src" "${dir}/src"
    ln -s "${BASE_DIR}/tests" "${dir}/tests"
    ln -s "${BASE_DIR}/benches" "${dir}/benches"
done

# main tests
(
    cd "${BASE_DIR}" || (echo "Failed to cd to ${BASE_DIR}"; exit 1)
    try_silent cargo update || exit 1
    try_silent cargo +stable test || exit 1
    try_silent cargo +stable test --no-default-features || exit 1
    try_silent cargo +stable test --no-default-features --features parallel || exit 1
    try_silent cargo +stable test --no-default-features --features log || exit 1
    try_silent cargo +stable test --all-features || exit 1
    try_silent cargo +nightly test || exit 1
    try_silent cargo +nightly test --no-default-features || exit 1
    try_silent cargo +nightly test --no-default-features --features parallel || exit 1
    try_silent cargo +nightly test --no-default-features --features log || exit 1
    try_silent cargo +nightly test --all-features || exit 1
    export RUSTDOCFLAGS="-D warnings"
    try_silent cargo +nightly doc --all-features --no-deps || exit 1
    try_silent cargo +nightly clippy -- -D warnings || exit 1
    try_silent cargo +stable fmt --check || exit 1
) || exit 1

# # minimum supported rust version
# (
#     cd "${MSRV_DIR}" || (echo "Failed to cd to ${MSRV_DIR}"; exit 1)
#     try_silent cargo +1.56.0 test --no-default-features || exit 1
#     try_silent cargo +1.56.0 test --no-default-features --features parallel || exit 1
# ) || exit 1

# minimal versions
(
    cd "${MIN_VERSIONS_DIR}" || (echo "Failed to cd to ${MIN_VERSIONS_DIR}"; exit 1)
    try_silent cargo +nightly -Z minimal-versions update || exit 1

    try_silent cargo +stable test --no-default-features || exit 1
    try_silent cargo +stable test --no-default-features --features parallel || exit 1
    try_silent cargo +nightly test --no-default-features || exit 1
    try_silent cargo +nightly test --no-default-features --features parallel || exit 1
) || exit 1

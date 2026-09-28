#!/usr/bin/env bash

# Install the unmodified official Lichtblick web release. No source checkout,
# Node build, Docker daemon, or extra Python packages are needed by the viewer.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
version="1.29.1"
archive_sha256="75ba88e69cd05b05b7fd9ee1246a205fc3089bf1bfbbdd1e77775f92bcf6237b"
license_sha256="da67717958c4e82555a863399c875f65a1d019084d9b3b1097ef89cae4806a84"
release_url="https://github.com/lichtblick-suite/lichtblick/releases/download/v${version}/lichtblick-web.tar.gz"
source_url="https://github.com/lichtblick-suite/lichtblick/tree/v${version}"
cache_root="${repo_root}/.venv/lichtblick"
destination="${cache_root}/${version}"

# Validate the installed files, rather than trusting an interrupted download or
# a directory left behind by an earlier attempt. This also makes reruns offline.
if [[ -f "${destination}/index.html" && -f "${destination}/.rs1-sha256" ]] &&
    [[ "$(cat "${destination}/.rs1-release" 2>/dev/null || true)" == "${archive_sha256}" ]] &&
    (cd "${destination}" && sha256sum --check --quiet .rs1-sha256 2>/dev/null); then
    printf 'Lichtblick %s is already installed: %s\n' "${version}" "${destination}"
    exit 0
fi

mkdir -p "${cache_root}"
staging="$(mktemp -d "${cache_root}/.install-${version}.XXXXXX")"
trap 'rm -rf -- "${staging}"' EXIT

printf 'Downloading official Lichtblick %s web assets (approximately 40 MB)\n' "${version}"
curl --fail --silent --show-error --location --retry 3 \
    "${release_url}" -o "${staging}/lichtblick-web.tar.gz"
curl --fail --silent --show-error --location --retry 3 \
    "https://raw.githubusercontent.com/lichtblick-suite/lichtblick/v${version}/LICENSE" \
    -o "${staging}/LICENSE"
printf '%s  %s\n' "${archive_sha256}" lichtblick-web.tar.gz \
    "${license_sha256}" LICENSE > "${staging}/download.sha256"
(cd "${staging}" && sha256sum --check download.sha256)

mkdir "${staging}/assets"
python3 - "${staging}/lichtblick-web.tar.gz" "${staging}/assets" <<'PY'
from pathlib import Path
import sys
import tarfile

destination = Path(sys.argv[2]).resolve()
with tarfile.open(sys.argv[1], "r:gz") as archive:
    # Explicit checks also work on the devcontainer's Python 3.10, before
    # tarfile's extraction filters were introduced.
    for member in archive.getmembers():
        target = (destination / member.name).resolve()
        if not target.is_relative_to(destination) or not (member.isfile() or member.isdir()):
            raise ValueError(f"Unsafe web archive entry: {member.name}")
    archive.extractall(destination)
if not (destination / "index.html").is_file():
    raise ValueError("The release does not contain index.html at the expected location")
PY

# The release archive does not include a standalone license. Keep the pinned
# upstream license and source location beside the otherwise untouched assets.
cp "${staging}/LICENSE" "${staging}/assets/LICHTBLICK-LICENSE.txt"
printf 'Lichtblick %s\nSource: %s\nWeb release: %s\nArchive SHA256: %s\n' \
    "${version}" "${source_url}" "${release_url}" "${archive_sha256}" \
    > "${staging}/assets/LICHTBLICK-SOURCE.txt"
printf '%s\n' "${archive_sha256}" > "${staging}/assets/.rs1-release"
python3 - "${staging}/assets" <<'PY'
import hashlib
from pathlib import Path
import sys

root = Path(sys.argv[1])
with (root / ".rs1-sha256").open("w") as manifest:
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.name != ".rs1-sha256":
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            manifest.write(f"{digest}  {path.relative_to(root)}\n")
PY

# Publish only a complete installation; any previous incomplete cache is kept
# in staging until the new directory is in place, then removed by the trap.
if [[ -e "${destination}" ]]; then
    mv -- "${destination}" "${staging}/previous"
fi
if ! mv -- "${staging}/assets" "${destination}"; then
    if [[ -e "${staging}/previous" ]]; then
        mv -- "${staging}/previous" "${destination}"
    fi
    exit 1
fi
printf 'Installed Lichtblick %s: %s\n' "${version}" "${destination}"

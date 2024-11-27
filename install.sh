#!/usr/bin/env bash
set -euo pipefail

__wrap__() {

REPO=MIERUNE/point-tiler

VERSION=${POINT_TILER_VERSION:-latest}

if ! command -v curl > /dev/null 2>&1; then
  echo "Error: 'curl' is required but not installed. Please install it and try again."
  exit 1
fi

if [[ $VERSION == "latest" ]]; then
  VERSION=$(curl -sL "https://api.github.com/repos/${REPO}/releases/latest" | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p')
  if [[ -z $VERSION ]]; then
    echo "Error: Unable to detect the latest version. Please check your internet connection or the repository."
    exit 1
  fi
fi

PLATFORM=$(uname -s)
ARCH=$(uname -m)

if [[ $PLATFORM == "Darwin" ]]; then
  PLATFORM="apple-darwin"
elif [[ $PLATFORM == "Linux" ]]; then
  PLATFORM="unknown-linux-gnu"
else
  echo "Error: Unsupported platform $PLATFORM"
  exit 1
fi

if [[ $ARCH == armv8* ]] || [[ $ARCH == arm64* ]] || [[ $ARCH == aarch64* ]]; then
  ARCH="aarch64"
elif [[ $ARCH == i686* ]] || [[ $ARCH == x86_64 ]]; then
  ARCH="x86_64"
else
  echo "Error: Unsupported architecture $ARCH"
  exit 1
fi

BINARY="point_tiler-${VERSION}-${ARCH}-${PLATFORM}"

DOWNLOAD_URL="https://github.com/${REPO}/releases/download/${VERSION}/${BINARY}"

echo "This script will automatically download and install point_tiler (${VERSION}) for you."

if [ "x$(id -u)" == "x0" ]; then
  echo "Warning: this script is running as root. This is dangerous and unnecessary!"
fi

TEMP_DIR=$(mktemp -d)
TEMP_FILE="$TEMP_DIR/point_tiler"
cleanup() {
  rm -rf "$TEMP_DIR"
}

trap cleanup EXIT

HTTP_CODE=$(curl -SL --progress-bar "$DOWNLOAD_URL" --output "$TEMP_FILE" --write-out "%{http_code}")
if [[ ${HTTP_CODE} -lt 200 || ${HTTP_CODE} -gt 299 ]]; then
  echo "Error: platform ${PLATFORM} (${ARCH}) or version ${VERSION} is unsupported."
  exit 1
fi

chmod +x "$TEMP_FILE"

INSTALL_DIR="/usr/local/bin"
if [[ ! -w $INSTALL_DIR ]]; then
  echo "Error: The install directory $INSTALL_DIR is not writable."
  echo "Please run the script with appropriate permissions or install to a directory you have write access to."
  exit 1
fi
mv "$TEMP_FILE" "$INSTALL_DIR/point_tiler"


echo "point_tiler has been installed successfully to $INSTALL_DIR/point_tiler"
}

__wrap__

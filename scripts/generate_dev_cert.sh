#!/usr/bin/env bash
set -euo pipefail

CERT_DIR="${1:-certs}"
CERT_FILE="${CERT_DIR}/teleop-dev.crt"
KEY_FILE="${CERT_DIR}/teleop-dev.key"

mkdir -p "${CERT_DIR}"

openssl req -x509 -nodes -newkey rsa:4096 \
  -keyout "${KEY_FILE}" \
  -out "${CERT_FILE}" \
  -days 365 \
  -subj "/CN=teleop.local"

echo "Wrote cert: ${CERT_FILE}"
echo "Wrote key:  ${KEY_FILE}"

#!/usr/bin/env bash
# 
# Obtain Let's Encrypt SSL certificates for teleop.loopcow.com
# 
# Prerequisites:
#   - Domain must point to this server's public IP
#   - Port 80 must be accessible from the internet (for HTTP-01 challenge)
#   - certbot must be installed (or use Docker)
#
# Usage:
#   ./scripts/obtain_ssl_cert.sh [--staging]
#
# Options:
#   --staging    Use Let's Encrypt staging server (for testing, avoids rate limits)
#

set -euo pipefail

DOMAIN="teleop.loopcow.com"
EMAIL="${CERTBOT_EMAIL:-admin@loopcow.com}"
CERT_DIR="$(dirname "$0")/../certs"
STAGING=""

# Parse arguments
for arg in "$@"; do
    case $arg in
        --staging)
            STAGING="--staging"
            echo "Using Let's Encrypt staging server (certificates won't be trusted)"
            ;;
        --email=*)
            EMAIL="${arg#*=}"
            ;;
        *)
            echo "Unknown argument: $arg"
            exit 1
            ;;
    esac
done

mkdir -p "${CERT_DIR}"

echo "=========================================="
echo "Obtaining SSL certificate for: ${DOMAIN}"
echo "Email: ${EMAIL}"
echo "Cert directory: ${CERT_DIR}"
echo "=========================================="

# Check if certbot is installed
if command -v certbot &> /dev/null; then
    echo "Using system certbot..."
    
    # Stop any service using port 80
    echo "Note: Port 80 must be free for the HTTP-01 challenge"
    
    # Run certbot in standalone mode
    sudo certbot certonly \
        --standalone \
        --non-interactive \
        --agree-tos \
        --email "${EMAIL}" \
        --domain "${DOMAIN}" \
        ${STAGING} \
        --cert-path "${CERT_DIR}" \
        --key-path "${CERT_DIR}" \
        --fullchain-path "${CERT_DIR}/fullchain.pem" \
        --privkey-path "${CERT_DIR}/privkey.pem"
    
    # Copy certificates to our cert directory with standard names
    echo "Copying certificates to ${CERT_DIR}..."
    sudo cp "/etc/letsencrypt/live/${DOMAIN}/fullchain.pem" "${CERT_DIR}/fullchain.pem"
    sudo cp "/etc/letsencrypt/live/${DOMAIN}/privkey.pem" "${CERT_DIR}/privkey.pem"
    sudo chown "$(id -u):$(id -g)" "${CERT_DIR}/fullchain.pem" "${CERT_DIR}/privkey.pem"
    chmod 600 "${CERT_DIR}/privkey.pem"
    chmod 644 "${CERT_DIR}/fullchain.pem"
    
else
    echo "Using Docker certbot..."
    
    # Use certbot Docker image
    docker run --rm -it \
        -v "${PWD}/${CERT_DIR}:/etc/letsencrypt" \
        -v "${PWD}/${CERT_DIR}/www:/var/www/certbot" \
        -p 80:80 \
        certbot/certbot certonly \
        --standalone \
        --non-interactive \
        --agree-tos \
        --email "${EMAIL}" \
        --domain "${DOMAIN}" \
        ${STAGING}
    
    # Copy certificates to accessible location
    if [ -d "${CERT_DIR}/live/${DOMAIN}" ]; then
        cp "${CERT_DIR}/live/${DOMAIN}/fullchain.pem" "${CERT_DIR}/fullchain.pem"
        cp "${CERT_DIR}/live/${DOMAIN}/privkey.pem" "${CERT_DIR}/privkey.pem"
        chmod 600 "${CERT_DIR}/privkey.pem"
        chmod 644 "${CERT_DIR}/fullchain.pem"
    fi
fi

echo ""
echo "=========================================="
echo "Certificate obtained successfully!"
echo "=========================================="
echo ""
echo "Certificate: ${CERT_DIR}/fullchain.pem"
echo "Private key: ${CERT_DIR}/privkey.pem"
echo ""
echo "To use with Docker Compose, update your .env file:"
echo "  SSL_CERT_FILE=/certs/fullchain.pem"
echo "  SSL_KEY_FILE=/certs/privkey.pem"
echo "  PORT=443"
echo ""
echo "Certificates expire in 90 days. Set up auto-renewal:"
echo "  sudo certbot renew --dry-run"
echo ""

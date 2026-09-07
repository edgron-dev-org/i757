#!/bin/bash
# vps_mqtt_setup.sh — one-shot mosquitto public broker deployment on your VPS.
# Purpose: public-Internet OTA testing for the i757 controller.
# Baseline: anonymous disabled + password_file auth + non-standard port; TLS added later.
# Idempotent: safe to re-run.
#
# NOTE: this is the LEGACY plaintext-auth setup. Production uses mTLS (port 18884,
#       device certificate = identity). For mTLS setup see docs/Anti_Clone_and_Secure_Element_Selection.md.
#
# >>> Set your own credentials before running (never commit a real password) <<<
set -euo pipefail

MQTT_PORT=18883
MQTT_USER="${MQTT_USER:-YOUR_MQTT_USER}"
MQTT_PASS="${MQTT_PASS:-CHANGE_ME}"        # export MQTT_PASS=... before running; keep app_cfg.h in sync

echo "== 1/5 port conflict check =="
if ss -tlnp | grep -q ":${MQTT_PORT} "; then
  ss -tlnp | grep ":${MQTT_PORT} "
  echo "!! port ${MQTT_PORT} already in use, aborting"; exit 1
fi

echo "== 2/5 install mosquitto =="
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq mosquitto mosquitto-clients

echo "== 3/5 configure (auth + listen ${MQTT_PORT}) =="
touch /etc/mosquitto/passwd
mosquitto_passwd -b /etc/mosquitto/passwd "${MQTT_USER}" "${MQTT_PASS}"
chown mosquitto:mosquitto /etc/mosquitto/passwd
chmod 600 /etc/mosquitto/passwd
cat > /etc/mosquitto/conf.d/i757.conf <<EOF
# i757 industrial controller public-Internet access (OTA/telemetry); default config only listens locally, open a public port here
listener ${MQTT_PORT} 0.0.0.0
allow_anonymous false
password_file /etc/mosquitto/passwd
EOF

echo "== 4/5 ufw allow + start service =="
ufw allow ${MQTT_PORT}/tcp comment 'i757 MQTT'
systemctl enable mosquitto
systemctl restart mosquitto
sleep 1
systemctl is-active mosquitto

echo "== 5/5 local loopback acceptance (authenticated pub/sub) =="
mosquitto_sub -h 127.0.0.1 -p ${MQTT_PORT} -u "${MQTT_USER}" -P "${MQTT_PASS}" -t 'selftest' -C 1 -W 5 > /tmp/mqtt_selftest &
sleep 0.5
mosquitto_pub -h 127.0.0.1 -p ${MQTT_PORT} -u "${MQTT_USER}" -P "${MQTT_PASS}" -t 'selftest' -m 'ok'
wait
grep -q ok /tmp/mqtt_selftest && echo "PASS: authenticated send/receive OK"
# anonymous must be rejected
if mosquitto_pub -h 127.0.0.1 -p ${MQTT_PORT} -t 'selftest' -m 'x' 2>/dev/null; then
  echo "!! anonymous publish unexpectedly succeeded, config is wrong"; exit 1
else
  echo "PASS: anonymous rejected"
fi
echo "== done: mosquitto @ 0.0.0.0:${MQTT_PORT}, user ${MQTT_USER} =="

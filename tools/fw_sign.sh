#!/bin/sh
# fw_sign.sh — sign a dual-core i757 firmware image (cm7.ota.bin ‖ cm4.ota.bin) with an ECDSA-P256 key.
# Prints the DER signature as hex = the argument of the board's `ota-sign <hex>` command and the
# "signature" field of the dashboard's firmware-upload page. The private key never leaves this machine;
# the board verifies against its embedded Edgron key OR the customer key registered with `fwkey2 set`
# (OTA_and_MQTT_User_Guide.md §6.2bis).
#
# usage: fw_sign.sh <cm7.ota.bin> <cm4.ota.bin> <my_fwsign.key> [out.hex]
#   (make the .bin files with: arm-none-eabi-objcopy -O binary platform_757_CM7.elf cm7.ota.bin, same for CM4)
set -e
[ $# -ge 3 ] || { echo "usage: $0 <cm7.ota.bin> <cm4.ota.bin> <private.key> [out.hex]" >&2; exit 2; }
cm7="$1"; cm4="$2"; key="$3"; out="${4:-}"
tmp="${TMPDIR:-/tmp}/i757_fwsign_$$"
trap 'rm -f "$tmp.bin" "$tmp.sig" "$tmp.pub"' EXIT
cat "$cm7" "$cm4" > "$tmp.bin"                       # same byte order as the board: SHA256(cm7 || cm4)
[ -s "$tmp.bin" ] || { echo "empty image" >&2; exit 1; }
openssl dgst -sha256 -sign "$key" -out "$tmp.sig" "$tmp.bin"
openssl ec -in "$key" -pubout -out "$tmp.pub" 2>/dev/null
openssl dgst -sha256 -verify "$tmp.pub" -signature "$tmp.sig" "$tmp.bin" >/dev/null   # self-check before handing it out
hex=$(od -An -tx1 -v "$tmp.sig" | tr -d ' \n')
echo "cm7=$(wc -c < "$cm7" | tr -d ' ') cm4=$(wc -c < "$cm4" | tr -d ' ') bytes  sha256(cm7||cm4)=$(openssl dgst -sha256 "$tmp.bin" | sed 's/.*= //')" >&2
echo "signature: $(( ${#hex} / 2 )) bytes DER" >&2
echo "$hex"
[ -n "$out" ] && printf '%s' "$hex" > "$out" && echo "written: $out" >&2
exit 0

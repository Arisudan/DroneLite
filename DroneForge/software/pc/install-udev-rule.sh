#!/usr/bin/env bash
set -euo pipefail

RULE_PATH="/etc/udev/rules.d/99-droneforge.rules"
RULE_CONTENT='SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="0483", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="df11", MODE="0666", TAG+="uaccess"'

if [ "${EUID}" -ne 0 ]; then
	echo "This installer must run as root." >&2
	exit 1
fi

install -d -m 0755 /etc/udev/rules.d

tmp_file="$(mktemp)"
trap 'rm -f "${tmp_file}"' EXIT

printf '%s\n' "${RULE_CONTENT}" > "${tmp_file}"
install -m 0644 "${tmp_file}" "${RULE_PATH}"

udevadm control --reload-rules
udevadm trigger

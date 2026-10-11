#!/usr/bin/env bash
# 用 USB 描述符把下位机的两个 CDC-ACM 口固定成稳定名字
#
#   /dev/gimbal  <-  ATTRS{bInterfaceNumber}=="00"  ATTRS{interface}=="PCCOMM-AIM"   (自瞄)
#   /dev/nav     <-  ATTRS{bInterfaceNumber}=="02"  ATTRS{interface}=="PCCOMM-NAV"   (导航)
#
#   sudo scripts/serial_setup.sh            # 安装规则(幂等)
#   sudo scripts/serial_setup.sh --remove   # 卸载规则
#        scripts/serial_setup.sh --print    # 只打印规则,不写系统(不需要 root)
set -euo pipefail

RULES="${RULES:-/etc/udev/rules.d/70-mas-serial.rules}"
LINK_AIM="${LINK_AIM:-gimbal}"
LINK_NAV="${LINK_NAV:-nav}"
GROUP="${GROUP:-dialout}"
MODE="${MODE:-0666}"

# 描述符 = 接口号 + 接口名。iInterface 字符串由下位机固件给出,同一台车固定不变;
AIM_IFNUM="${AIM_IFNUM:-00}"
AIM_IFNAME="${AIM_IFNAME:-PCCOMM-AIM}"
NAV_IFNUM="${NAV_IFNUM:-02}"
NAV_IFNAME="${NAV_IFNAME:-PCCOMM-NAV}"

RULE_TEXT="$(cat <<EOF
# 电控串口:按 USB 描述符(接口号 + 接口名)绑定固定名与权限。由 scripts/serial_setup.sh 生成。
# 自瞄:PCCOMM-AIM
SUBSYSTEM=="tty", ATTRS{bInterfaceNumber}=="$AIM_IFNUM", ATTRS{interface}=="$AIM_IFNAME", SYMLINK+="$LINK_AIM", GROUP="$GROUP", MODE="$MODE", ENV{ID_MM_DEVICE_IGNORE}="1"
# 导航:PCCOMM-NAV
SUBSYSTEM=="tty", ATTRS{bInterfaceNumber}=="$NAV_IFNUM", ATTRS{interface}=="$NAV_IFNAME", SYMLINK+="$LINK_NAV", GROUP="$GROUP", MODE="$MODE", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
)"

case "${1:-}" in
  --print|-n)
    echo "$RULE_TEXT"
    exit 0
    ;;
  --remove|-r)
    if [ "$(id -u)" -ne 0 ]; then
      echo "需要 root,请用: sudo $0 --remove" >&2
      exit 1
    fi
    if [ -e "$RULES" ]; then
      rm -f "$RULES"
      udevadm control --reload-rules
      udevadm trigger --subsystem-match=tty
      echo "已删除 $RULES"
    else
      echo "$RULES 不存在,无需删除"
    fi
    exit 0
    ;;
  --help|-h)
    sed -n '2,13p' "$0"
    exit 0
    ;;
  "")
    ;;
  *)
    echo "未知参数: $1(可用 --print / --remove)" >&2
    exit 1
    ;;
esac

if [ "$(id -u)" -ne 0 ]; then
  echo "需要 root,请用: sudo $0" >&2
  exit 1
fi

printf '%s\n' "$RULE_TEXT" > "$RULES"
echo "已写入 $RULES:"
sed 's/^/  /' "$RULES"

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

# 等软链出现(设备没插时不会出现,不影响规则本身)
for _ in $(seq 1 20); do
  [ -e "/dev/$LINK_AIM" ] && [ -e "/dev/$LINK_NAV" ] && break
  sleep 0.1
done

echo
for link in "$LINK_AIM" "$LINK_NAV"; do
  if [ -e "/dev/$link" ]; then
    printf '/dev/%-8s -> %s\n' "$link" "$(readlink -f "/dev/$link")"
  else
    printf '/dev/%-8s 未出现(设备未插?插好后 udevadm trigger --subsystem-match=tty)\n' "$link"
  fi
done

# 打印实际命中的描述符,方便确认绑的就是想要的那个口
if [ -e "/dev/$LINK_AIM" ]; then
  echo
  echo "命中的描述符:"
  udevadm info -a -n "/dev/$LINK_AIM" 2>/dev/null | grep -m2 -E 'bInterfaceNumber|interface=' | sed 's/^/  /' || true
fi

echo
echo "把 config.yaml 的 serial.port 设为 /dev/$LINK_AIM 即可"

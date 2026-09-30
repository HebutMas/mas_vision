#!/usr/bin/env bash
#   sudo scripts/serial_setup.sh              # 交互式:列出设备,输入序号选择
#   sudo scripts/serial_setup.sh /dev/ttyACM0 # 直接指定设备
set -euo pipefail

RULES=/etc/udev/rules.d/70-mas-serial.rules
LINK="${LINK:-gimbal}"
GROUP="${GROUP:-dialout}"
MODE="${MODE:-0666}"

if [ "$(id -u)" -ne 0 ]; then
  echo "需要 root,请用: sudo $0" >&2
  exit 1
fi

# 列出候选设备
mapfile -t DEVS < <(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | sort)
if [ "${#DEVS[@]}" -eq 0 ]; then
  echo "未找到 /dev/ttyACM* 或 /dev/ttyUSB* 设备,请先插上下位机" >&2
  exit 1
fi

# 选择
if [ "$#" -ge 1 ]; then
  DEV="$1"
  [ -e "$DEV" ] || { echo "设备不存在: $DEV" >&2; exit 1; }
elif [ "${#DEVS[@]}" -eq 1 ]; then
  DEV="${DEVS[0]}"
  echo "只有一个设备,自动选择: $DEV"
else
  echo "可用串口设备:"
  for i in "${!DEVS[@]}"; do
    printf '  %d) %s\n' "$((i + 1))" "${DEVS[$i]}"
  done
  read -rp "选择序号: " n
  if ! [[ "$n" =~ ^[0-9]+$ ]] || [ "$n" -lt 1 ] || [ "$n" -gt "${#DEVS[@]}" ]; then
    echo "无效序号: $n" >&2
    exit 1
  fi
  DEV="${DEVS[$((n - 1))]}"
fi
echo "已选择: $DEV"

# 取标识。优先 厂商/产品 + 序列号;没有序列号时退回 ID_PATH(认物理口,必须插同一个口)。
prop() { udevadm info -q property -n "$DEV" 2>/dev/null | sed -n "s/^$1=//p"; }
VID="$(prop ID_VENDOR_ID)"
PID="$(prop ID_MODEL_ID)"
SER="$(prop ID_SERIAL_SHORT)"
IDPATH="$(prop ID_PATH)"

if [ -n "$SER" ]; then
  MATCH="ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", ATTRS{serial}==\"$SER\""
  echo "匹配方式: 厂商 $VID:$PID + 序列号 $SER"
elif [ -n "$IDPATH" ]; then
  MATCH="ENV{ID_PATH}==\"$IDPATH\""
  echo "匹配方式: ID_PATH $IDPATH(设备无序列号,需插回同一个 USB 口)"
else
  echo "无法获取 $DEV 的标识(idVendor/serial/ID_PATH 都为空)" >&2
  exit 1
fi

# 写规则并生效
cat > "$RULES" <<EOF
# 电控串口:固定名 /dev/$LINK + 权限。由 scripts/serial_setup.sh 生成。
SUBSYSTEM=="tty", $MATCH, SYMLINK+="$LINK", GROUP="$GROUP", MODE="$MODE"
EOF
echo "已写入 $RULES"

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

# 等软链出现
for _ in $(seq 1 20); do [ -e "/dev/$LINK" ] && break; sleep 0.1; done

if [ -e "/dev/$LINK" ]; then
  ls -l "/dev/$LINK"
  echo "把 config.yaml 的 serial.port 设为 /dev/$LINK 即可"
else
  echo "软链 /dev/$LINK 未出现:重插一次设备,或 udevadm trigger --subsystem-match=tty" >&2
fi

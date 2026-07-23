# N3 runbook — PREEMPT_RT kernel install (Trevor-driven; every step needs sudo)

Scope: install NVIDIA's prebuilt RT kernel via the OTA apt repo, keep the
stock kernel as a boot fallback, verify RT is live, then run the acceptance
gate (cyclictest under CUDA + memory-bandwidth load, target < ~100 µs
worst-case). Full core-isolation tuning (isolcpus/nohz_full/IRQ affinity) is
A1 work, not N3 — do not bundle it into this session.

**Claude cannot run any of this** (no passwordless sudo). Run the blocks in
order; paste outputs back for verification at the checkpoints marked ⏸.

---

## ⚠ Stage 0 — preflight: find the extlinux.conf the bootloader ACTUALLY reads

Discovery (2026-07-23, read-only inspection): the NVMe rootfs copy of
`/boot/extlinux/extlinux.conf` still says `root=/dev/mmcblk0p1`, yet the
system runs with / on `/dev/nvme0n1p1`, and the EFI system partition mounted
at `/boot/efi` is the SD card's (`mmcblk0p10`). Conclusion pending your
confirmation: **the bootloader reads the SD card's extlinux.conf and SD's
/boot/Image, not the NVMe's** — the NVMe copy is a stale pre-migration
artifact. If so, an apt kernel install (which writes to /boot on the NVMe
rootfs) changes NOTHING the bootloader sees until the files are synced to
the SD APP partition.

```bash
sudo mkdir -p /mnt/sd
sudo mount -o ro /dev/mmcblk0p1 /mnt/sd
echo "--- SD extlinux.conf ---"
cat /mnt/sd/boot/extlinux/extlinux.conf
echo "--- SD kernel images ---"
ls -la /mnt/sd/boot/Image* /mnt/sd/boot/initrd* 2>/dev/null
```

⏸ **Checkpoint:** expected — SD's extlinux.conf shows `root=/dev/nvme0n1p1`
(the migration edit). If instead the SD copy also says `mmcblk0p1`, stop and
report: the boot path is something else entirely (UEFI direct boot) and the
runbook needs rework before anything is installed.

Leave the SD mounted read-only for now; later stages remount it rw.

Also confirm the rescue path is intact before touching kernels:

```bash
# serial console available? (USB-C cable to the Mac, screen/minicom ready)
# This is the recovery path if a boot goes sideways — verify BEFORE rebooting.
```

## Stage 1 — add NVIDIA's rt-kernel apt repo

The device has `common`, `t234`, and `ffmpeg` r36.4 repos but NOT the
rt-kernel repo (verified 2026-07-23).

```bash
echo "deb https://repo.download.nvidia.com/jetson/rt-kernel r36.4 main" \
  | sudo tee /etc/apt/sources.list.d/nvidia-l4t-rt-kernel.list
sudo apt-get update
apt-cache search --names-only rt-kernel
```

⏸ **Checkpoint:** paste the package list. Expected (per NVIDIA JP6 docs —
verify against actual output, do not trust from memory):
`nvidia-l4t-rt-kernel`, `nvidia-l4t-rt-kernel-headers`,
`nvidia-l4t-rt-kernel-oot-modules`, `nvidia-l4t-display-rt-kernel`.

## Stage 2 — snapshot current state, then install

```bash
uname -r                                   # expect 5.15.148-tegra (stock)
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.pre-rt
sudo cp /mnt/sd/boot/extlinux/extlinux.conf /tmp/sd-extlinux.conf.pre-rt \
  && sudo cp /tmp/sd-extlinux.conf.pre-rt "$HOME/"   # SD copy backup in $HOME
sudo apt-get install -y nvidia-l4t-rt-kernel nvidia-l4t-rt-kernel-headers \
  nvidia-l4t-rt-kernel-oot-modules nvidia-l4t-display-rt-kernel
```

Then inspect what the postinst actually did — NVIDIA's packages typically
add their own extlinux entry and may flip DEFAULT:

```bash
ls -la /boot/Image* /boot/initrd*
diff /boot/extlinux/extlinux.conf.pre-rt /boot/extlinux/extlinux.conf || true
```

⏸ **Checkpoint:** paste both outputs. What we need to know: the RT image
filename, whether an RT LABEL was added, and whether DEFAULT changed. All of
this landed on the **NVMe** /boot — the bootloader hasn't seen any of it yet.

## Stage 3 — sync to the SD APP partition (the one the bootloader reads)

Adapt filenames to Stage 2's actual output. Principles: the stock `primary`
entry on the SD stays byte-for-byte untouched as the fallback; the RT entry
is added alongside; `root=` in the RT entry must point at the NVMe
(`/dev/nvme0n1p1`), matching the primary entry's migration edit.

```bash
sudo mount -o remount,rw /mnt/sd
sudo cp /boot/Image.rt   /mnt/sd/boot/Image.rt      # name per stage 2
sudo cp /boot/initrd.rt  /mnt/sd/boot/initrd.rt     # if one was generated
sudoedit /mnt/sd/boot/extlinux/extlinux.conf
```

Target shape of the SD extlinux.conf (adapt paths/names):

```
TIMEOUT 30
DEFAULT real-time

MENU TITLE L4T boot options

LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      INITRD /boot/initrd
      APPEND ${cbootargs} root=/dev/nvme0n1p1 rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 firmware_class.path=/etc/firmware fbcon=map:0 video=efifb:off console=tty0

LABEL real-time
      MENU LABEL real-time kernel (PREEMPT_RT)
      LINUX /boot/Image.rt
      INITRD /boot/initrd.rt
      APPEND ${cbootargs} root=/dev/nvme0n1p1 rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 firmware_class.path=/etc/firmware fbcon=map:0 video=efifb:off console=tty0
```

The `primary` APPEND line above must equal what Stage 0 found on the SD —
copy it verbatim, do not retype it. With `TIMEOUT 30` and serial console,
the fallback is selectable at boot even if RT panics.

⏸ **Checkpoint:** paste the final SD extlinux.conf before rebooting.

## Stage 4 — reboot and verify RT is live

```bash
sudo reboot
# after it comes back:
uname -r                     # expect a -rt suffix
cat /sys/kernel/realtime     # expect: 1
zcat /proc/config.gz | grep PREEMPT_RT   # expect CONFIG_PREEMPT_RT=y
lsmod | head -20             # oot modules (nvgpu etc.) loaded
nvidia-smi 2>/dev/null || sudo jtop --version   # GPU stack alive under RT
df -h /                      # still on nvme0n1p1
```

Rollback if it won't boot: serial console → pick `primary` at the boot menu
(or flip `DEFAULT real-time` back to `DEFAULT primary` on the SD from the
rescue boot). The stock kernel entry is never edited, only added alongside.

## Stage 5 — acceptance gate: cyclictest under combined load

```bash
sudo apt-get install -y rt-tests stress-ng
```

Three terminals (or tmux panes):

```bash
# pane 1 — GPU + memory-bandwidth load (the Orin's real threat is memory
# contention from the GPU, not CPU): sustained big matmuls
~/venvs/flatsat-ml/bin/python - <<'EOF'
"""Sustained GPU load for the N3 cyclictest gate."""
import torch
a = torch.randn(4096, 4096, device="cuda")
b = torch.randn(4096, 4096, device="cuda")
print("GPU load running — Ctrl-C to stop")
while True:
    a = (a @ b).tanh()
    torch.cuda.synchronize()
EOF

# pane 2 — CPU + memory pressure
stress-ng --cpu 4 --vm 2 --vm-bytes 1G --timeout 600s

# pane 3 — the measurement (10 min, all cores, 1 kHz)
sudo cyclictest --mlockall --smp --priority=95 --interval=1000 --duration=10m
```

⏸ **Gate:** worst-case `Max:` across all threads < ~100 µs ⇒ N3 PASSES.
Community data says 300–500 µs spikes appear when IRQ affinity is untuned —
if that happens, N3 still counts as installed-and-working; the sub-100 µs
target moves to A1 where isolcpus/nohz_full/IRQ steering get done properly
(that is where PLAN.md §4 puts the full RT strategy anyway). Record the
cyclictest summary in PLAN.md §0 either way, and re-run the same command
after A1 tuning for the before/after artifact.

## Stage 6 — cleanup + record

```bash
sudo umount /mnt/sd
cd ~/flatsat && ./tools/jetson-setup.sh   # refresh manifests (new kernel)
```

Then: PLAN.md §0 gets the result + decision-log entry (RT default vs
primary default), and jetson-setup.sh gets the rt-kernel repo + package
stanza added idempotently (Claude's job, after your outputs come back).

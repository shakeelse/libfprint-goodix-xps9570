# Goodix53x5 Fingerprint Driver — Dell XPS 15 9570

This repo is a patched checkout of upstream libfprint with the
[goodix53x5 driver](https://github.com/AndyHazz/goodix53x5-libfprint) wired in,
to support the Goodix HTK32 sensor (USB ID `27c6:5395`) found in the
Dell XPS 15 9570.

## Hardware

- Sensor: Goodix HTK32, capacitive press-type, 108x88px
- USB ID: `27c6:5395` (also covers `27c6:5335` / `27c6:5385` on related models)
- Matching: SIGFM (SIFT-based, via OpenCV) — convenience-grade auth,
  not intended as a high-security factor (e.g. don't rely on it alone
  for disk encryption).

## What's patched vs upstream

- `libfprint/meson.build`
  - Added `goodix53x5` entry to `driver_sources`
  - Added SIGFM/OpenCV static-library build block (`libsigfm`)
  - Linked `libsigfm` into `libfprint_drivers` and `libfprint`
  - Propagated `opencv_dep` into the shared `deps` list so it reaches
    both `libfprint_drivers` and `libfprint`
- `meson.build` (root)
  - Added `'goodix53x5': { 'helper': ['openssl'] }` to `drivers_info`
    (this repo's libfprint version auto-derives `default_drivers` and
    helper deps from this dict — older forks used a separate
    `default_drivers` list + `driver_helpers` dict, which no longer
    matches upstream's current structure)

Full diffs: `git log -p` in this repo, or diff against upstream:
```bash
git diff upstream/master -- meson.build libfprint/meson.build
```

## Rebuild from scratch

```bash
# 1. Build dependencies
sudo apt build-dep libfprint-2-2
sudo apt install libopencv-dev

# 2a. Either clone this repo directly (already patched):
git clone https://github.com/shakeelse/libfprint-goodix-xps9570.git ~/libfprint
cd ~/libfprint

# 2b. Or start from a fresh upstream clone and reapply the driver files
#     + this repo's meson.build patches:
git clone https://gitlab.freedesktop.org/libfprint/libfprint.git ~/libfprint
cd ~/libfprint
git clone https://github.com/AndyHazz/goodix53x5-libfprint.git ~/goodix-driver
~/goodix-driver/install.sh ~/libfprint
# then manually reapply the meson.build edits described above —
# see this repo's commit history for the exact diffs

# 3. Build
meson setup builddir --prefix=/usr -Dinstalled-tests=false -Ddoc=false
ninja -C builddir
sudo ninja -C builddir install
sudo systemctl restart fprintd

# 4. Verify the driver is actually compiled in
strings /lib/x86_64-linux-gnu/libfprint-2.so.2.0.0 | grep -i goodix53x5

# 5. Enroll
fprintd-enroll
fprintd-verify
```

## Post-install hardening (survive apt upgrades)

```bash
# Prevent apt from silently reinstalling stock libfprint/fprintd
sudo apt-mark hold libfprint-2-2 fprintd

# Self-healing check, reinstalls from builddir/ if apt ever clobbers it
# Script: ~/bin/restore-goodix-fprint.sh
# Wired into: /etc/apt/apt.conf.d/99-goodix-fprint-restore
# Logs to: journalctl -t goodix-fprint-restore
```

See `restore-goodix-fprint.sh` in this repo for the script contents.

## Sleep/resume fix

`fprintd` can break after suspend/resume (known upstream quirk, not
specific to this driver). Fix via a systemd unit that kills `fprintd`
before sleep so it reactivates cleanly on resume:

```bash
sudo systemctl enable kill-fprintd-before-sleep.service
```

See `kill-fprintd-before-sleep.service` in this repo.

## Troubleshooting

- `strings /lib/x86_64-linux-gnu/libfprint-2.so.2.0.0 | grep goodix53x5`
  returns nothing → driver wasn't actually compiled in. Check
  `driver_sources` in `libfprint/meson.build` and `drivers_info` in the
  root `meson.build`, then `rm -rf builddir` and reconfigure from
  scratch (meson caches driver list at configure time).
- `fprintd-enroll` says `NoSuchDevice` → check `lsusb | grep 27c6`,
  and confirm `fprintd` is actually linked against your custom
  `libfprint-2.so.2`, not a stale `/usr/local` or apt-packaged copy
  (`ldd $(which fprintd) | grep fprint`, `dpkg -S <path>` should
  report "no path found" for a from-source build).

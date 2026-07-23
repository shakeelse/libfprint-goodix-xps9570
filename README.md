# libfprint-goodix-xps9570

**Working Goodix HTK32 (`27c6:5395`) fingerprint reader driver for Linux — Dell XPS 15 9570**

The fingerprint sensor on the Dell XPS 15 9570 has never had official Linux support.
This repo is a patched build of [libfprint](https://gitlab.freedesktop.org/libfprint/libfprint)
with the community [goodix53x5 driver](https://github.com/AndyHazz/goodix53x5-libfprint) wired in,
plus rebuild instructions, apt-upgrade hardening, and a sleep/resume fix — everything needed to
get fingerprint login and `sudo` authentication working on Ubuntu (and other distros using
`libfprint`/`fprintd`).

If you landed here searching for things like *"Dell XPS 15 9570 fingerprint Ubuntu"*,
*"Goodix 27c6:5395 Linux"*, *"XPS 9570 fprintd NoSuchDevice"*, or
*"libfprint goodix53x5 not detected"* — this is for you.

## Hardware this covers

| | |
|---|---|
| **Laptop** | Dell XPS 15 9570 (also reported to affect Dell XPS 13 9305, XPS 13 7390 2-in-1) |
| **Sensor** | Goodix HTK32, capacitive press-type, 108×88px |
| **USB ID** | `27c6:5395` (driver also covers `27c6:5335` / `27c6:5385`) |
| **Matching method** | SIGFM — SIFT-based image matching via OpenCV |

Check if this applies to you:

```bash
lsusb | grep -E '27c6:(5335|5385|5395)'
```

> **Security note:** this is a small, low-resolution sensor using SIFT-based image matching
> rather than libfprint's usual minutiae matcher. Treat it as **convenience-grade**
> authentication (desktop login, quick `sudo` unlock) — not as your only safeguard for
> something like full-disk encryption.

## Quick start

```bash
# 1. Build dependencies
sudo apt build-dep libfprint-2-2
sudo apt install libopencv-dev

# 2. Clone this repo (already patched)
git clone https://github.com/shakeelse/libfprint-goodix-xps9570.git ~/libfprint
cd ~/libfprint

# 3. Build and install
meson setup builddir --prefix=/usr -Dinstalled-tests=false -Ddoc=false
ninja -C builddir
sudo ninja -C builddir install
sudo systemctl restart fprintd

# 4. Confirm the driver actually compiled in
strings /lib/x86_64-linux-gnu/libfprint-2.so.2.0.0 | grep -i goodix53x5

# 5. Enroll and test
fprintd-enroll
fprintd-verify
```

If `strings` returns nothing in step 4, the driver wasn't compiled in — see
[NOTES.md](./NOTES.md) for detailed troubleshooting, including common `meson.build`
integration failures on newer libfprint versions where the driver registry structure
has changed since the upstream driver project's install script was written.

## What's patched vs upstream libfprint

- `libfprint/meson.build`
  - Registered `goodix53x5` in `driver_sources`
  - Added the SIGFM/OpenCV static-library build block (`libsigfm`)
  - Linked `libsigfm` into `libfprint_drivers` and `libfprint`
  - Propagated the OpenCV dependency through the shared `deps` list
- Root `meson.build`
  - Added a `goodix53x5` entry to `drivers_info` (this repo's libfprint
    version auto-derives the default driver list and helper dependencies
    from this single dict — see [NOTES.md](./NOTES.md) for why this differs
    from older integration guides)

Full diff against upstream:

```bash
git diff upstream/master -- meson.build libfprint/meson.build
```

## Keeping it working after system updates

Because this is a from-source build, a plain `apt upgrade` can silently reinstall
the stock (non-working) `libfprint-2-2` package over your custom build. This repo
includes:

- `restore-goodix-fprint.sh` — checks whether the driver is still present after
  every apt operation, and auto-reinstalls from your local `builddir` if not
- `kill-fprintd-before-sleep.service` — fixes a known `fprintd` quirk where
  fingerprint unlock stops working after suspend/resume

See [NOTES.md](./NOTES.md) for setup of both, plus `apt-mark hold` guidance.

## Credits

- Protocol reverse-engineering and driver: [AndyHazz/goodix53x5-libfprint](https://github.com/AndyHazz/goodix53x5-libfprint)
- SIGFM matching library: [goodix-fp-linux-dev/sigfm](https://github.com/goodix-fp-linux-dev/sigfm)
- Upstream: [freedesktop.org/libfprint](https://gitlab.freedesktop.org/libfprint/libfprint)

## License

LGPL-2.1-or-later, matching upstream libfprint.

---

Full rebuild/troubleshooting/hardening details: **[NOTES.md](./NOTES.md)**

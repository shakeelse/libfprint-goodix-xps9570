#!/bin/bash
set -e
LIBPATH="/lib/x86_64-linux-gnu/libfprint-2.so.2.0.0"
BUILDDIR="$HOME/libfprint/builddir"

if [ ! -f "$LIBPATH" ] || ! strings "$LIBPATH" 2>/dev/null | grep -q goodix53x5; then
    echo "$(date): goodix53x5 driver missing from $LIBPATH — reinstalling custom build..." | systemd-cat -t goodix-fprint-restore
    if [ -d "$BUILDDIR" ]; then
        ninja -C "$BUILDDIR" install
        systemctl restart fprintd
        if strings "$LIBPATH" | grep -q goodix53x5; then
            echo "$(date): restore succeeded." | systemd-cat -t goodix-fprint-restore
        else
            echo "$(date): restore FAILED — manual rebuild needed." | systemd-cat -t goodix-fprint-restore -p err
        fi
    else
        echo "$(date): no builddir found at $BUILDDIR — cannot auto-restore." | systemd-cat -t goodix-fprint-restore -p err
    fi
fi

#!/bin/bash

OVERLAY="US_PRU-00A0"
CAPENAME="US_PRU"

echo "Installing Device Tree Overlay"

dtc -O dtb -o /lib/firmware/$OVERLAY.dtbo -b 0 -@ $OVERLAY.dts



#!/bin/bash
set -e
make clean
make
cd build/HEXFile
set +e
xfce4-terminal --title="OpenOCD" -e "openocd -f /usr/share/openocd/scripts/board/st_nucleo_f3.cfg"
sleep 1
{ echo "reset halt"; sleep 1; echo "flash write_image erase Thermometer.hex"; sleep 2; echo "reset run"; sleep 1; } | telnet localhost 4444
wmctrl -c OpenOCD









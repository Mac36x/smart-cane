#!/bin/bash
cd /home/raspberrypi/Desktop/SmartCane
source .venv/bin/activate
aconnect -x
until python detection.py; do
    aplay /home/raspberrypi/Desktop/SmartCane/audio/ระบบขัดข้อง.wav
    sleep 1
    aplay /home/raspberrypi/Desktop/SmartCane/audio/รีเซ็ตระบบ.wav
    echo "Script 'detection.py' crashed with exit code $?. Respawning.." >&2
    sleep 1
done
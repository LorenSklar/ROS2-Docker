#!/bin/bash
Xvfb :1 -screen 0 1024x768x24 &
sleep 1
mkdir -p /root/.vnc
echo "${VNC_PASSWORD:-password}" | vncpasswd -f > /root/.vnc/passwd
chmod 600 /root/.vnc/passwd
x11vnc -forever -usepw -create -display :1 -rfbport 5901 &
/usr/share/novnc/utils/launch.sh --vnc localhost:5901 --listen 6080 --web /usr/share/novnc &
export DISPLAY=:1
exec "$@" 
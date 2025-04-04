#!/bin/bash

# Iniciar Xvfb en display :99
Xvfb :99 -screen 0 1920x1080x24 +extension GLX &
export DISPLAY=:99

# Iniciar gestor de ventanas
fluxbox &

# Iniciar servidor VNC (puerto 5900)
x11vnc -forever -shared -usepw -display :99 -rfbport 5900 &

# Source ROS
source /opt/ros/jazzy/setup.bash

# Mantener el contenedor en ejecución
tail -f /dev/null
#!/bin/bash
# 1. Načtení prostředí ROS 2 (bez toho program nenastartuje)
source /opt/ros/humble/setup.bash

# 2. Nastavení upovídanosti logů, aby v journalctl něco bylo vidět
export LOG_LEVEL=1

# 3. Spuštění s ABSOLUTNÍMI cestami (nahraď cestu tou svou!)
/home/student/workspace_xtylsa03/mpc-rbt-student/build/sender_node /home/student/workspace_xtylsa03/mpc-rbt-student/config.json

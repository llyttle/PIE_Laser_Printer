#!/bin/bash
ipVar=$(/sbin/ifconfig eth0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')
title=$(hostname -I)
curl https://api.pushbullet.com/v2/pushes \
-u o.AZTAE1xMsQE8me2XLeALCJLOONG8utgw: \
-d device_iden="ujDeBh1CYyysjzyXTXRUTk" \
-d type="note" \
-d title=$title \
-d body=$ipVar \
-X POST

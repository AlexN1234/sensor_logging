#!/bin/bash
shopt -s nullglob
for file in /home/pi/SensorLog/to_upload_WiFi/*.csv
do
    sed -i.bak s/\"//g "$file"
    if curl -v --ftp-ssl -T "$file" -u mitpearl:f1r3fLy123! "ftp://beetle.gmn-usa.com/inbound_wifi/"
    then 
    	mv "$file" /home/pi/SensorLog/on_server_WiFi/
    else 
    	echo "$file failed to upload"
    fi
done
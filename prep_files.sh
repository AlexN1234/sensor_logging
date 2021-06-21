#!/bin/bash
shopt -s nullglob
for file in /home/pi/SensorLog/to_upload/*.csv
do
    sed -i.bak s/\"//g "$file"
    cp "$file" /home/pi/SensorLog/to_upload_WiFi/
    mv "$file" /home/pi/SensorLog/to_upload_Iridium/
done
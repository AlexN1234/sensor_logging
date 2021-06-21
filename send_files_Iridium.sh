#!/bin/bash
shopt -s nullglob
for file in /home/pi/SensorLog/to_upload_Iridium/*.csv
do
    sed -i.bak s/\"//g "$file"
    if curl --fail -X POST -H 'Content-Type: text/csv' --data-binary @"$file" -u superadmin:webxaccess "http://192.168.10.1/xgatev1/xfer/index.php?action=ServerDataSync&for=coast"
    then
        mv "$file" /home/pi/SensorLog/on_server_Iridium/
    else
        echo "$file failed to upload"
    fi
done

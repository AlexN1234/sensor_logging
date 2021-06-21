#!/usr/bin/python3

import sys, os, serial, csv, shutil, glob
import time, datetime
import board, busio
import adafruit_htu21d, adafruit_gps, adafruit_bmp280, adafruit_bno055
import adafruit_sht31d, adafruit_lps35hw, adafruit_veml7700
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

## INITIALIZE SENSORS
# GPS Sensor Initialization
# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
#uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)

# for a computer, use the pyserial library for uart access
import serial
# uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)

# If using I2C, we'll create an I2C interface to talk to using default pins
# i2c = board.I2C()

# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
# gps = adafruit_gps.GPS_GtopI2C(i2c, debug=False)  # Use I2C interface

# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
#   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
# Turn on just minimum info (RMC only, location):
# gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn off everything:
# gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Tuen on everything (not all of it is parsed!)
# gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

# Set update rate to once a second (1hz) which is what you typically want.
gps.send_command(b"PMTK220,1000")
# Or decrease to once every two seconds by doubling the millisecond value.
# Be sure to also increase your UART timeout above!
# gps.send_command(b'PMTK220,2000')
# You can also speed up the rate, but don't go too fast or else you can lose
# data during parsing.  This would be twice a second (2hz, 500ms delay):
# gps.send_command(b'PMTK220,500')

# Main loop runs forever printing the location, etc. every second.
last_print = time.monotonic()

# i2c sensor initialization
i2c = busio.I2C(board.SCL, board.SDA)
#sensors_connected = [hex(x) for x in i2c.scan()]

# HTU Sensor initialization - tested OK
htu_flag = 0
try:
    htu_sensor = adafruit_htu21d.HTU21D(i2c)
    htu_flag = 1
except:
    pass

# BMP Sensor initialization - tested OK
bmp_flag = 0
try:
    bmp_sensor = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
    bmp_flag = 1
except:
    pass
    
# IMU sensor initialization - tested OK
bno_flag = 0
try:
    bno_sensor = adafruit_bno055.BNO055_I2C(i2c)
    bno_flag = 1
except:
    pass
  
# VEML Sensor initialization
veml_flag = 0
try:
    veml_sensor = adafruit_veml7700.VEML7700(i2c)
    veml_sensor.light_gain = veml_sensor.ALS_GAIN_1_8
    veml_sensor.light_integration_time = veml_sensor.ALS_25MS
    veml_flag = 1
except:
    pass

# SHT-30 Sensor initialization
sht_flag = 0
try:
    sht_sensor = adafruit_sht31d.SHT31D(i2c)
    sht_flag = 1
except:
    pass

# LPS Sensor initialization - tested OK
lps_flag = 0
try:
    lps_sensor = adafruit_lps35hw.LPS35HW(i2c)
    lps_flag = 1
except:
    pass

# Wind speed sensor initialization
wind_flag = 0
try:
    ads = ADS.ADS1115(i2c)
    wind_chan = AnalogIn(ads, ADS.P0)
    wind_flag = 1
except:
    pass
    
# DS18 Sensor initialization
ds18_flag = 0
try:
    base_dir = '/sys/bus/w1/devices/'
    device_folder = glob.glob(base_dir + '28*')[0]
    device_file = device_folder + '/w1_slave'
    ds18_flag = 1
except:
    pass

## GET FILENAME TO WRITE TO
writing_path = '/home/pi/SensorLog/writing/'
completed_path = '/home/pi/SensorLog/to_upload/'

dt = datetime.datetime.now()
year = str(dt.year)
if dt.month < 10:
    month = "0" + str(dt.month)
else:
    month = str(dt.month)
if dt.day < 10:
    day = "0" + str(dt.day)
else:
    day = str(dt.day)
if dt.hour < 10:
    hour = "0" + str(dt.hour)
else:
    hour = str(dt.hour)
if dt.minute < 10:
    minute = "0" + str(dt.minute)
else:
    minute = str(dt.minute)
if dt.second < 10:
    second = "0" + str(dt.second)
else:
    second = str(dt.second)
    
filename = "SensorLog" + "_" + year + "-" + month + "-" + day + "_" + hour + "-" + minute + "-" + second + "_00.csv"
logfile = writing_path+filename
finalfile = completed_path+filename

# increment filename
if os.path.isfile(logfile):
    expand = 0
    while True:
        expand += 1
        if expand < 10:
            fileinc = "_0" + str(expand)
        else:
            fileinc = str(expand)
            new_filename = filename.split("_00.csv")[0] + fileinc + ".csv"
            new_logfile = writing_path+new_filename
        if os.path.isfile(new_logfile):
            continue
        else:
            logfile = new_logfile
            finalfile = completed_path+new_filename
            break
    print("File name is: " + logfile)

# write headers to file
with open(logfile, mode='w') as sensor_readings:
    sensor_write = csv.writer(sensor_readings, delimiter=',')
    headers = ['Date','Time (EST)',
               'GPS Date', 'GPS Time(UTC)','Fix','Quality','Location N','Location W',
               'Speed (knots)','Angle','Altitutde','Satellites',
               'X_accel (m/s^2)','Y_accel (m/s^2)','Z_accel (m/s^2)',
               'X_euler (deg)','Y_euler (deg)','Z_euler (deg)',
               'X_mag (microtesla)','Y_mag (microtesla)','Z_mag (microtesla)',
               'Sys_cal','Gyro_cal','Accel_cal','Mag_cal',
               'BMP temp(C)','BMP press(Pa)',
               'HTU temp (C)','HTU humid(%)',
               'Lux', 'White', 'Raw ALS',
               'LPS temp (C)', 'LPS press (hPa)',
               'SHTtemp (C)','SHThumid (%)','SHTheater On',
               'Wind Reading (V)', 'Wind Speed (m/s)',
               'DS18temp (C)']

    write_to_log = sensor_write.writerow(headers)
    print(headers)

# GET DATA TO WRITE
def get_timestamp():
    today = str(datetime.datetime.now().strftime("%Y-%m-%d"))
    now = str(datetime.datetime.now().strftime("%H:%M:%S"))
    timestamp = "{},{}".format(today,now)
    return(timestamp)

def get_bnodata():
    if bno_flag == 1:
        try:
            bno_accel = "{:.02f},{:.02f},{:.02f}".format(bno_sensor.acceleration[0],bno_sensor.acceleration[1],bno_sensor.acceleration[2])
            bno_euler = "{},{},{}".format(bno_sensor.euler[0],bno_sensor.euler[1],bno_sensor.euler[2])
            bno_mag = "{:.03f},{:.03f},{:.03f}".format(bno_sensor.magnetic[0],bno_sensor.magnetic[1],bno_sensor.magnetic[2])
            bno_cal = "{},{},{},{}".format(bno_sensor.calibration_status[0],bno_sensor.calibration_status[1],bno_sensor.calibration_status[2],bno_sensor.calibration_status[3])
        except:
            bno_accel = "No BNO055,No BNO055,No BNO055"
            bno_euler = "No BNO055,No BNO055,No BNO055"
            bno_mag = "No BNO055,No BNO055,No BNO055"
            bno_cal = "No BNO055,No BNO055,No BNO055,No BNO055"
    else:
        bno_accel = "No BNO055,No BNO055,No BNO055"
        bno_euler = "No BNO055,No BNO055,No BNO055"
        bno_mag = "No BNO055,No BNO055,No BNO055"
        bno_cal = "No BNO055,No BNO055,No BNO055,No BNO055"
    bno_data = "{},{},{},{}".format(bno_accel, bno_euler, bno_mag, bno_cal)
    return(bno_data)

def get_bmpdata():
    if bmp_flag == 1:
        try:
            bmp_data = "{:.02f},{:.02f}".format(bmp_sensor.temperature,bmp_sensor.pressure)
        except:
            bmp_data = "No BMP280,No BMP280"
    else:
        bmp_data = "No BMP280,No BMP280"
    return(bmp_data)

def get_htudata():
    if htu_flag == 1:
        try:
            htu_data = "{:.02f},{:.02f}".format(htu_sensor.temperature,htu_sensor.relative_humidity)
        except:
            htu_data = "No HTU21D-F,No HTU21D-F"
    else:
        htu_data = "No HTU21D-F,No HTU21D-F"
    return(htu_data)

def get_vemldata():
    if veml_flag == 1:
        try:
            veml_lux_raw = veml_sensor.lux
            veml_lux = 6.0135e-13*pow(veml_lux_raw,4)-9.3924e-9*pow(veml_lux_raw,3)+8.1488e-5*pow(veml_lux_raw,2)+1.0023*veml_lux_raw;
            veml_data = "{:.02f},{:.02f},{:.02f}".format(veml_lux,veml_sensor.white,veml_sensor.light)
        except:
            veml_data = "No VEML7700,No VEML7700,No VEML7700"
    else:
        veml_data = "No VEML7700,No VEML7700,No VEML7700"
    return(veml_data)

def get_lpsdata():
    if lps_flag == 1:
        try:
            lps_data = "{:.02f},{:.02f}".format(lps_sensor.temperature,lps_sensor.pressure)
        except:
            lps_data = "No LPS35W,No LPS35W"
    else:
        lps_data = "No LPS35W,No LPS35W"
    return(lps_data)

def get_winddata():
    if wind_flag == 1:
        wind_voltage = wind_chan.voltage
        wind_speed = wind_voltage/6; # Wind speed is proportional to the output voltage.
        wind_data = "{:.03f},{:.03f}".format(wind_voltage,wind_speed)
    else:
        wind_data = "No Wind,No Wind"
    return(wind_data)

def get_shtdata(loopcount):
    if sht_flag == 1:
        try:
            sht_data = "{:.02f},{:.02f}".format(sht_sensor.temperature,sht_sensor.relative_humidity)
            sht_data = "{},{}".format(sht_data,sht_sensor.heater)
            if loopcount == 6 and sht_sensor.heater == False:
                # if has been off for 30 seconds, turn on heater
                loopcount = 0
                sht_sensor.heater = True
            elif loopcount == 2 and sht_sensor.heater == True:
                # if it has been on for 10 seconds, turn off heater
                loopcount = 0
                sht_sensor.heater = False
        except:
            sht_data = "No SHT31-D,No SHT31-D,No SHT31-D"
    else:
        sht_data = "No SHT31-D,No SHT31-D,No SHT31-D"
    return(sht_data, loopcount)

def get_ds18data_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

def get_ds18data():
    if ds18_flag == 1:
        try:
            lines = get_ds18data_raw()
            if lines[0].strip()[-3:] == 'YES': #if there is data? do some stuff
                equals_pos = lines[1].find('t=')
                if equals_pos != -1:
                    temp_string = lines[1][equals_pos+2:]
                    ds18_temp = float(temp_string) / 1000.0
                    #temp_f = temp_c * 9.0 / 5.0 + 32.0
                    ds18_data = "{:.02f}".format(ds18_temp)
                else:
                    ds18_data ="No DS18B20"
            else:
                ds18_data ="No DS18B20"
        except:
            ds18_data ="No DS18B20"    
    else:
        ds18_data ="No DS18B20"
    return(ds18_data)
    
# WRITE DATA
def write_to_csv(last_print,loopcount):
    # get GPS data from serial - can't be done with subfunction
    # Make sure to call gps.update() every loop iteration and at least twice
    # as fast as data comes from the GPS unit (usually every second).
    # This returns a bool that's true if it parsed new data (you can ignore it
    # though if you don't care and instead look at the has_fix property).
    gps.update()
    # Every second print out current location (if there's a GPS fix).
    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        if gps.has_fix:
            gps.update()
            # Print out details about the fix like location, date, etc.
            gps_datetime = "{}/{}/{},{:02}:{:02}:{:02}".format(
                    gps.timestamp_utc.tm_year,  # Grab parts of the time from the
                    gps.timestamp_utc.tm_mon,  # struct_time object that holds
                    gps.timestamp_utc.tm_mday,  # the fix time.  Note you might
                    gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                    gps.timestamp_utc.tm_min,  # month!
                    gps.timestamp_utc.tm_sec)
            gps_lat = "{0:.6f}N".format(gps.latitude)
            gps_lon = "{0:.6f}W".format(gps.longitude)
            gps_fix = "{}".format(gps.has_fix)
            gps_quality = "{}".format(gps.fix_quality)
            gps_satellites = "{}".format(gps.satellites)
            gps_altitude = "{}".format(gps.altitude_m)
            gps_speed = "{}".format(gps.speed_knots)
            gps_angle = "{}".format(gps.track_angle_deg)
            gps_dilution = "{}".format(gps.horizontal_dilution)
            gps_geoID = "{}".format(gps.height_geoid)
        else:
            gps_datetime = "0000/00/00,00:00:00"
            gps_fix = "0"
            gps_quality = "0"
            gps_lat = "0"
            gps_lon = "0"
            gps_speed = "0"
            gps_angle = "0"
            gps_altitude = "0"
            gps_satellites = "0"
    else:
        scc_datastring = "No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC"
        gps_datetime = "0000/00/00,00:00:00"
        gps_fix = "0"
        gps_quality = "0"
        gps_lat = "0"
        gps_lon = "0"
        gps_speed = "0"
        gps_angle = "0"
        gps_altitude = "0"
        gps_satellites = "0"
    gps_datastring = "{},{},{},{},{},{},{},{},{}".format(gps_datetime,
                                                            gps_fix, gps_quality, gps_lat,
                                                            gps_lon, gps_speed,gps_angle,
                                                            gps_altitude, gps_satellites)
    #gps_datetime + "," + gps_fix + "," + gps_quality + "," + gps_lat + "," + gps_lon + "," + gps_speed + "," + gps_angle + "," + gps_altitude + "," + gps_satellites
    # the a is for append, if w for write is used then it overwrites the file
    with open(logfile, mode='a') as sensor_readings:
        sensor_write = csv.writer(sensor_readings, delimiter=',')
        sht_data = get_shtdata(loopcount)
        data_string = [get_timestamp(),
                       gps_datastring,
                       get_bnodata(),
                       get_bmpdata(),
                       get_htudata(),
                       get_vemldata(),
                       get_lpsdata(),
                       sht_data[0],
                       get_winddata(),
                       get_ds18data()]
        loopcount = sht_data[1]
        print(data_string)
        write_to_log = sensor_write.writerow(data_string)
        return(write_to_log, loopcount)
    
# RUN
start = time.time()
#print("Start timex is " + str(start))
runtime = 3570 # run program for 59.5 minutes
loopcount = 0
while True:
    print('Writing filename ' + str(filename))
    gps.update()
    written = write_to_csv(last_print,loopcount)
    loopcount = written[1]
    #log data every 1 second
    time.sleep(1)
    loopcount += 1
    # stop program if it has been 1 minute
    #print("Time is " + str(time.time()))
    #print("Time to stop is " + str(start+runtime))
    #print("Ready to stop? " + str(time.time() > (start+runtime)))
    if time.time() > (start+runtime):
        #move file to completed folder
        print("Logging complete. Moving file.")
        shutil.move(logfile,finalfile)
        break

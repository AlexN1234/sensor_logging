#!/usr/bin/python3

# updated 2021-06-29 with new NXP IMU in place of BNO
# updated 2021-07-26 for PEARL B
import sys, os, serial, csv, shutil, glob
import time, datetime
import board, busio
import adafruit_htu21d, adafruit_gps, adafruit_bmp280, adafruit_fxos8700, adafruit_fxas21002c
import adafruit_sht31d, adafruit_lps35hw, adafruit_veml7700
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

## INITIALIZE SENSORS
# SCC reading initialization
client = ModbusClient(method = 'rtu', port = '/dev/ttyXRUSB0', baudrate = 115200)

# GPS Sensor Initialization
# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
#uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)

# for a computer, use the pyserial library for uart access
import serial
uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
# uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)

# Initialize serial port for comms with navigation RPi
moos_serial = serial.Serial("/dev/ttyS0", baudrate=115200, timeout=10)

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
nxp_flag = 0
try:
    fxos_sensor = adafruit_fxos8700.FXOS8700(i2c)
    fxas_sensor = adafruit_fxas21002c.FXAS21002C(i2c)

    # Optionally create the sensor with a different accelerometer range (the
    # default is 2G, but you can use 4G or 8G values):
    # sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_4G)
    # sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_8G)

    # Optionally create the sensor with a different gyroscope range (the
    # default is 250 DPS, but you can use 500, 1000, or 2000 DPS values):
    # sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_500DPS)
    # sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_1000DPS)
    # sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_2000DPS)

    nxp_flag = 1
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
    
filename = "SensorLogB" + "_" + year + "-" + month + "-" + day + "_" + hour + "-" + minute + "-" + second + "_00.csv"
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
               'X_mag (microtesla)','Y_mag (microtesla)','Z_mag (microtesla)',
               'X_gyro (rad/s)','Y_gyro (rad/s)','Z_gyro (rad/s)',
               'BMP temp(C)','BMP press(Pa)',
               'HTU temp (C)','HTU humid(%)',
               'Lux', 'White', 'Raw ALS',
               'LPS temp (C)', 'LPS press (hPa)',
               'SHTtemp (C)','SHThumid (%)','SHTheater On',
               'Wind Reading (V)', 'Wind Speed (m/s)',
               'DS18temp (C)','PV Voltage (V)', 'PV Current (A)', 'PV Power L (W)',
               'PV Power H (W)', 'Battery Voltage (V)', 'Battery Current (A)', 'Battery Net Current (A)',
               'Battery Power L (W)', 'Battery Power H (W)',
               'Load Voltage (V)', 'Load Current (A)', 'Load Power L (W)',
               'Load Power H (W)', 'Battery SOC (%)',
               'Battery Temperature (C)', 'Device Temperature (C)']

    write_to_log = sensor_write.writerow(headers)
    print(headers)

# GET DATA TO WRITE
def get_timestamp():
    today = str(datetime.datetime.now().strftime("%Y-%m-%d"))
    now = str(datetime.datetime.now().strftime("%H:%M:%S"))
    timestamp = "{},{}".format(today,now)
    return(timestamp)

def get_nxpdata():
    if nxp_flag == 1:
        try:
            nxp_accel = "{:.02f},{:.02f},{:.02f}".format(fxos_sensor.accelerometer[0],fxos_sensor.accelerometer[1],fxos_sensor.accelerometer[2])
            nxp_mag = "{:.03f},{:.03f},{:.03f}".format(fxos_sensor.magnetometer[0],fxos_sensor.magnetometer[1],fxos_sensor.magnetometer[2])
            nxp_gyro = "{:.03f},{:.03f},{:.03f}".format(fxas_sensor.gyroscope[0],fxas_sensor.gyroscope[1],fxas_sensor.gyroscope[2])
        except:
            nxp_accel = "No NXPIMU,No NXPIMU,No NXPIMU"
            nxp_mag = "No NXPIMU,No NXPIMU,No NXPIMU"
            nxp_gyro = "No NXPIMU,No NXPIMU,No NXPIMU"
    else:
        nxp_accel = "No NXPIMU,No NXPIMU,No NXPIMU"
        nxp_mag = "No NXPIMU,No NXPIMU,No NXPIMU"
        nxp_gyro = "No NXPIMU,No NXPIMU,No NXPIMU"
    nxp_data = "{},{},{}".format(nxp_accel, nxp_mag, nxp_gyro)
    return(nxp_data)

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
        # Updated 07/08/2021 with calibration curve from wind speed test
        #wind_speed = wind_voltage*6.681+0.921; # Wind speed is proportional to the output voltage.
        # Updated 07/26/2021 with calibration curve for second anemometer from manufactuerer
        wind_speed = (wind_voltage-0.4)/1.6 * 32.4
        #wind_speed = wind_voltage*30.190-12.071; # Wind speed is proportional to the output voltage.
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
    # get GPS and SCC data from serial - can't be done with subfunction
    # Make sure to call gps.update() every loop iteration and at least twice
    # as fast as data comes from the GPS unit (usually every second).
    # This returns a bool that's true if it parsed new data (you can ignore it
    # though if you don't care and instead look at the has_fix property).
    gps.update()
    # Every second print out SCC details and current location (if there's a GPS fix).
    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        # SCC data
        client.connect()
        # result = client.read_input_registers(0x3000,9,unit=1)
        # ratedInputVoltage = float(result.registers[0] /100.0)
        # ratedInputCurrent = float(result.registers[1] /100.0)
        # ratedInputPowerL = float(result.registers[2] /100.0)
        # ratedInputPowerH = float(result.registers[3] /100.0)
        # ratedOutputVoltage = float(result.registers[4] /100.0)
        # ratedOutputCurrent = float(result.registers[5] /100.0)
        # ratedOutputPowerL = float(result.registers[6] /100.0)
        # ratedOutputPowerH = float(result.registers[7] /100.0)
        # ratedOutputCurrentLoad = float(result.registers[8] /100.0)
        # print("Rated Input Voltage: ",ratedInputVoltage)
        # print("Rated Input Current: ",ratedInputCurrent)
        # print("Rated Input Power L: ",ratedInputPowerL)
        # print("Rated Input Power H: ",ratedInputPowerH)
        # print("Rated Output Voltage: ",ratedOutputVoltage)
        # print("Rated Output Current: ",ratedOutputCurrent)
        # print("Rated Output Power L: ",ratedOutputPowerL)
        # print("Rated Output Power H: ",ratedOutputPowerH)
        # print("Rated Output Current of Load: ",ratedOutputCurrentLoad);

        #solarVoltage = float(result.registers[0] /100.0)
        #solarCurrent = float(result.registers[1] /100.0)
        #batteryVoltage = float(result.registers[4] /100.0)
        #chargeCurrent = float(result.registers[5] /100.0)
        #print("Solar Voltage: ",solarVoltage)
        #print("Solar Current: ",solarCurrent)
        #print("Battery voltage: ",batteryVoltage)
        #print("Charge current: ",chargeCurrent)
        try:
            result = client.read_input_registers(0x3100,8,unit=1)
            pvVoltage = float(result.registers[0] / 100.0)
            pvCurrent = float(result.registers[1] / 100.0)
            pvPowerL = float(result.registers[2] / 100.0)
            pvPowerH = float(result.registers[3] / 100.0)
            batteryVoltage = float(result.registers[4] / 100.0)
            batteryCurrent = float(result.registers[5] / 100.0)
            batteryPowerL = float(result.registers[6] / 100.0)
            batteryPowerH = float(result.registers[7] / 100.0)
#       print("PV array voltage: ",pvVoltage);
#       print("PV array current: ",pvCurrent);
#       print("PV array power (L): ",pvPowerL);
#       print("PV array power (H): ",pvPowerH);
#       print("Battery voltage: ",pvVoltage);
#       print("Battery current: ",pvCurrent);
#       print("Battery power (L): ",pvPowerL);
#       print("Battery power (H): ",pvPowerH);
        except:
            pvVoltage = "No SCC"
            pvCurrent = "No SCC"
            pvPowerL = "No SCC"
            pvPowerH = "No SCC"
            batteryVoltage = "No SCC"
            batteryCurrent = "No SCC"
            batteryPowerL = "No SCC"
            batteryPowerH = "No SCC"

        try:
            result = client.read_input_registers(0x310C,6,unit=1)
            loadVoltage = float(result.registers[0] / 100.0)
            loadCurrent = float(result.registers[1] / 100.0)
            loadPowerL = float(result.registers[2] / 100.0)
            loadPowerH = float(result.registers[3] / 100.0)
            batteryTemp = float(result.registers[4] / 100.0)
            deviceTemp = float(result.registers[5] / 100.0)
#       print("Load voltage: ",loadVoltage);
#       print("Load current: ",loadCurrent);
#       print("Load power (L): ",loadPowerL);
#       print("Load power (H): ",loadPowerH);
#       print("Battery temperature: ",batteryTemp);
#       print("Device temperature: ",deviceTemp);
        except:
            loadVoltage = "No SCC"
            loadCurrent = "No SCC"
            loadPowerL = "No SCC"
            loadPowerH = "No SCC"
            batteryTemp = "No SCC"
            deviceTemp = "No SCC"
            
        try:
            result = client.read_input_registers(0x311A,1,unit=1)
            batterySOC = float(result.registers[0] / 100.0)
            #print("Battery SOC: ",batterySOC);
        except:
            batterySOC = "No SCC"
        
        try:
            result = client.read_input_registers(0x331B,1,unit=1)
            batterynetCurrent = float(result.registers[0] / 100.0)
            #print("Battery Net Current (A): ",batterynetCurrent);
        except:
            batterynetCurrent = "No SCC"
            
        client.close()
        scc_datastring = "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}".format(
            pvVoltage,
            pvCurrent,
            pvPowerL,
            pvPowerH,
            batteryVoltage,
            batteryCurrent,
            batterynetCurrent,
            batteryPowerL,
            batteryPowerH,
            loadVoltage,
            loadCurrent,
            loadPowerL,
            loadPowerH,
            batterySOC,
            batteryTemp,
            deviceTemp)
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
    
    # Get data and format properly to send to navigation RPi
    scc_moos_data = scc_datastring
    veml_moos_data = get_vemldata()
    wind_moos_data = get_winddata()

    if scc_moos_data[0:2]=="No":
        scc_moos_data = "0,-,-,-,-,-,-,-,-,-,-,-,-,-,-,-,-"
    else:
        scc_moos_data = "1," + scc_moos_data
    if veml_moos_data[0:2]=="No":
        veml_moos_data = "0,-,-,-"
    else:
        veml_moos_data = "1," + veml_moos_data
    if wind_moos_data[0:2]=="No":
        wind_moos_data = "0,-,-"
    else:
        wind_moos_data = "1," + wind_moos_data

    scc_moos_string = "$PLSCC," + scc_moos_data + "*"
    veml_moos_string = "$PLLUX," + veml_moos_data + "*"
    wind_moos_string = "$PLWND," + wind_moos_data + "*"
    moos_serial.write(scc_moos_string.encode())
    moos_serial.write(veml_moos_string.encode())
    moos_serial.write(wind_moos_string.encode())
    
    # the a is for append, if w for write is used then it overwrites the file
    with open(logfile, mode='a') as sensor_readings:
        sensor_write = csv.writer(sensor_readings, delimiter=',')
        sht_data = get_shtdata(loopcount)
        data_string = [get_timestamp(),
                       gps_datastring,
                       get_nxpdata(),
                       get_bmpdata(),
                       get_htudata(),
                       get_vemldata(),
                       get_lpsdata(),
                       sht_data[0],
                       get_winddata(),
                       get_ds18data(),
                       scc_datastring]
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
    moos_serial.write(scc_moos_string.encode())
    moos_serial.write(veml_moos_string.encode())
    moos_serial.write(wind_moos_string.encode())
    #log data and send to navigation RPi every 1 second
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

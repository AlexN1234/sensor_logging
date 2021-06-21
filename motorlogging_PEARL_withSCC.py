#!/usr/bin/python3

import sys, os, serial, csv, shutil, glob
import time, datetime
import board, busio
import adafruit_htu21d, adafruit_gps, adafruit_bmp280, adafruit_bno055
import adafruit_sht31d, adafruit_lps35hw, adafruit_veml7700
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

## INITIALIZE SENSORS
# SCC reading initialization
client = ModbusClient(method = 'rtu', port = '/dev/ttyXRUSB0', baudrate = 115200)

# Main loop runs forever printing the location, etc. every second.
last_print = time.monotonic()

# i2c sensor initialization
i2c = busio.I2C(board.SCL, board.SDA)

# VEML Sensor initialization
veml_flag = 0
try:
    veml_sensor = adafruit_veml7700.VEML7700(i2c)
    veml_sensor.light_gain = veml_sensor.ALS_GAIN_1_8
    veml_sensor.light_integration_time = veml_sensor.ALS_25MS
    veml_flag = 1
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
    
filename = "MotorTestingLog" + "_" + year + "-" + month + "-" + day + "_" + hour + "-" + minute + "-" + second + "_00.csv"
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
               'Lux', 'White', 'Raw ALS',
               'Wind Reading (V)', 'Wind Speed (m/s)',
               'PV Voltage (V)', 'PV Current (A)', 'PV Power L (W)',
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
    
# WRITE DATA
def write_to_csv(last_print,loopcount):
    # get SCC data from serial - can't be done with subfunction
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

        result = client.read_input_registers(0x311A,1,unit=1)
        batterySOC = float(result.registers[0] / 100.0)
        #print("Battery SOC: ",batterySOC);
        
        result = client.read_input_registers(0x331B,1,unit=1)
        batterynetCurrent = float(result.registers[0] / 100.0)
        #print("Battery Net Current (A): ",batterynetCurrent);
        
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
    else:
        scc_datastring = "No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC, No SCC"        
    # the a is for append, if w for write is used then it overwrites the file
    with open(logfile, mode='a') as sensor_readings:
        sensor_write = csv.writer(sensor_readings, delimiter=',')
        data_string = [get_timestamp(),
                       get_vemldata(),
                       get_winddata(),
                       scc_datastring]
        print(data_string)
        write_to_log = sensor_write.writerow(data_string)
        return(write_to_log, loopcount)
    
# RUN
start = time.time()
#print("Start timex is " + str(start))
runtime = 108000 # run program for 3 hours 
loopcount = 0
while True:
    print('Writing filename ' + str(filename))
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



# SGP41-VOC-Shield
Sensor PCB for VOC, NOx, Temperature, Humidity and Barometric Pressure
The SGP41 VOC Shield is a small PCB from RJ Hegler Technologies, LLC containing Temperature, Humidity, Pressure and VOC sensors meant to connect to the XIAO series of micros. 

The sensors populated on the PCB are: 

   Sensirion AG SHT40-AD1B Temperature and Humidity.

   Sensirion AG SGP41-D-R4 VOC.

   TDK ICP-10111 Barometric Pressure and Temperature.


The board uses the XIAO 3.3VDC supply and communicates via I2C.

Code examples are for a Seeed Studio XIAO ESP32C3.

Evs-2_SGP41_VOC.ino sends data thru the serial port at a baud rate of 115,200
and also displays 4 lines on a 0.96" OLED display (please verify pinout, some OLED displays have Vcc and GND reversed).
The display reads:

	SGP41 VOC Sensor
 
	78.23F, RH 48.00%
 
	NOx = 1, VOC = 94
 
	28.97inHg
 
(Numbers in the data are an example, your readings should be the actual for your conditions)

The PCB for this is identical to the EVs-1, its populated with a SGP41 sensor rather than a SGP40 sensor. 

The schematic is almost the same between the EVs-1 and EVs-2, just change the part number of U3 to SGP41




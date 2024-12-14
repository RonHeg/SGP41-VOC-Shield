# EVs-2 SGP41-VOC-Shield
The Evs-2 VOC SGP41 OLED SuperMini is a small PCB from RJ Hegler Technologies, LLC containing Temperature, Humidity, Pressure, VOC and NOx sensors designed to connect to the ESP32C3 SuperMini microcontroller. The PCB is also configurable to easily attach to a XIAO microcontroller or, with jumper wires, you could attach it to other processors.

The sensors are: 

	Sensirion AG SHT-40-AD1B Temperature and Humidity. 
 
	Sensirion AG SGP41-D-R4 NOx and VOC. 
 
	TDK ICP-10111 Barometric pressure and temperature. 
 
	The PCB uses its hosts 3.3v supply (~11mA with the loaded firmware) and communicates via I2C.
 

Evs-2_SGP41_VOC.ino sends data thru the serial port at a baud rate of 115,200 and also displays 4 lines to the included OLED display (if you are using a different display, please verify pinout, some OLED displays have Vcc and GND reversed). 

The board uses the XIAO 3.3VDC supply and communicates via I2C.

The OLED display reads:

	SGP41 VOC Sensor
	78.23F, RH 48.00%
	NOx = 1, VOC = 94
	28.97inHg
 
(Numbers in the data are an example, your readings should be the actual for your conditions)

Example of data sent through the serial port is:

	------------------------------
	SHT40
	T: 26.32        RH: 28.41
	Tticks: 26704   RHticks: 18016

	SGP41
	compensationRh: 18016   compensationT: 26704
	raw VOC Index: 28689    raw NOx Index: 0
	VOC Index: 0    NOx Index: 0

	ICP-10111
	Temperature: 27.38C
	99919.05Pa      29.51inHg
	Altitude: 132.09m
	------------------------------



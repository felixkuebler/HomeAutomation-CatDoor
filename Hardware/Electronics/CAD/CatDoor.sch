EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:74xgxx
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ESP8266
LIBS:HiLink
LIBS:L9110
LIBS:SW_DPST_x2
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "IOT Relay + Switch"
Date "2017-05-30"
Rev "1.1.0"
Comp "Felix Kübler"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ESP-12E U4
U 1 1 592BD8DB
P 7600 1775
F 0 "U4" H 7600 2500 50  0000 C CNN
F 1 "ESP-12E" H 7600 2425 50  0000 C CNN
F 2 "ESP8266:ESP-12E_SMD" H 7600 1775 50  0001 C CNN
F 3 "" H 7600 1775 50  0001 C CNN
	1    7600 1775
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 J1
U 1 1 592BDEB8
P 3200 1525
F 0 "J1" H 3200 1950 50  0000 C CNN
F 1 "MPE 094-1-006" H 3475 1875 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Angled_1x06_Pitch2.54mm" H 3200 1525 50  0001 C CNN
F 3 "" H 3200 1525 50  0001 C CNN
	1    3200 1525
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 592C2849
P 8700 2175
F 0 "#PWR05" H 8700 1925 50  0001 C CNN
F 1 "GND" V 8600 2100 50  0000 C CNN
F 2 "" H 8700 2175 50  0001 C CNN
F 3 "" H 8700 2175 50  0001 C CNN
	1    8700 2175
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR06
U 1 1 592C2C75
P 6500 2175
F 0 "#PWR06" H 6500 2025 50  0001 C CNN
F 1 "+3.3V" V 6400 2250 50  0000 C CNN
F 2 "" H 6500 2175 50  0001 C CNN
F 3 "" H 6500 2175 50  0001 C CNN
	1    6500 2175
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR07
U 1 1 592C2CAB
P 6500 1675
F 0 "#PWR07" H 6500 1525 50  0001 C CNN
F 1 "+3.3V" V 6575 1775 50  0000 C CNN
F 2 "" H 6500 1675 50  0001 C CNN
F 3 "" H 6500 1675 50  0001 C CNN
	1    6500 1675
	0    -1   -1   0   
$EndComp
Text GLabel 8750 1450 2    60   Input ~ 0
TX
Text GLabel 8750 1600 2    60   Input ~ 0
RX
Text GLabel 8650 1875 2    60   Input ~ 0
MODE
NoConn ~ 6700 1475
NoConn ~ 7350 2675
NoConn ~ 7450 2675
NoConn ~ 7550 2675
NoConn ~ 7650 2675
NoConn ~ 7750 2675
NoConn ~ 7850 2675
$Comp
L GND #PWR08
U 1 1 592C460C
P 2850 1775
F 0 "#PWR08" H 2850 1525 50  0001 C CNN
F 1 "GND" V 2900 1575 50  0000 C CNN
F 2 "" H 2850 1775 50  0001 C CNN
F 3 "" H 2850 1775 50  0001 C CNN
	1    2850 1775
	0    1    1    0   
$EndComp
Text GLabel 2600 1475 0    60   Input ~ 0
RX
Text GLabel 2600 1325 0    60   Input ~ 0
TX
NoConn ~ 3000 1275
NoConn ~ 3000 1675
Text GLabel 3000 2400 2    60   Input ~ 0
MODE
$Comp
L GND #PWR09
U 1 1 592C5F8F
P 3025 2200
F 0 "#PWR09" H 3025 1950 50  0001 C CNN
F 1 "GND" V 3075 2050 50  0000 C CNN
F 2 "" H 3025 2200 50  0001 C CNN
F 3 "" H 3025 2200 50  0001 C CNN
	1    3025 2200
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR010
U 1 1 592C600F
P 3025 2600
F 0 "#PWR010" H 3025 2450 50  0001 C CNN
F 1 "+3.3V" V 3075 2775 50  0000 C CNN
F 2 "" H 3025 2600 50  0001 C CNN
F 3 "" H 3025 2600 50  0001 C CNN
	1    3025 2600
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR011
U 1 1 595EB4E9
P 9050 1975
F 0 "#PWR011" H 9050 1825 50  0001 C CNN
F 1 "+3.3V" V 9100 2175 50  0000 C CNN
F 2 "" H 9050 1975 50  0001 C CNN
F 3 "" H 9050 1975 50  0001 C CNN
	1    9050 1975
	0    1    1    0   
$EndComp
$Comp
L SW_DPST_x2 SW2
U 1 1 599EBC5A
P 2300 3200
F 0 "SW2" H 2300 3100 50  0000 C CNN
F 1 "SMD 4x4mm Button" H 2300 3000 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_SKQG" H 2300 3200 50  0001 C CNN
F 3 "" H 2300 3200 50  0001 C CNN
	1    2300 3200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR012
U 1 1 599EBD49
P 1800 3200
F 0 "#PWR012" H 1800 3050 50  0001 C CNN
F 1 "+3.3V" V 1800 3450 50  0000 C CNN
F 2 "" H 1800 3200 50  0001 C CNN
F 3 "" H 1800 3200 50  0001 C CNN
	1    1800 3200
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 599EC273
P 3000 3350
F 0 "R5" V 3080 3350 50  0000 C CNN
F 1 "RND 0805 1 10K" V 3150 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 2930 3350 50  0001 C CNN
F 3 "" H 3000 3350 50  0001 C CNN
	1    3000 3350
	0    1    1    0   
$EndComp
$Comp
L GND #PWR013
U 1 1 599EC69C
P 3300 3350
F 0 "#PWR013" H 3300 3100 50  0001 C CNN
F 1 "GND" V 3300 3150 50  0000 C CNN
F 2 "" H 3300 3350 50  0001 C CNN
F 3 "" H 3300 3350 50  0001 C CNN
	1    3300 3350
	0    -1   -1   0   
$EndComp
Text GLabel 3100 3200 2    60   Input ~ 0
RESET
Text GLabel 9050 1775 2    60   Input ~ 0
RESET
$Comp
L +3.3V #PWR016
U 1 1 5A1C2463
P 2850 1575
F 0 "#PWR016" H 2850 1425 50  0001 C CNN
F 1 "+3.3V" V 2775 1800 50  0000 C CNN
F 2 "" H 2850 1575 50  0001 C CNN
F 3 "" H 2850 1575 50  0001 C CNN
	1    2850 1575
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8500 2175 8700 2175
Wire Wire Line
	8500 2075 8600 2075
Wire Wire Line
	8600 2075 8600 2175
Connection ~ 8600 2175
Wire Wire Line
	6700 1675 6500 1675
Wire Wire Line
	6500 2175 6700 2175
Wire Wire Line
	8500 1475 8650 1475
Wire Wire Line
	8650 1475 8650 1450
Wire Wire Line
	8650 1450 8750 1450
Wire Wire Line
	8750 1600 8650 1600
Wire Wire Line
	8650 1600 8650 1575
Wire Wire Line
	8650 1575 8500 1575
Wire Wire Line
	8650 1875 8500 1875
Wire Wire Line
	3000 1475 2600 1475
Wire Wire Line
	3000 1375 2750 1375
Wire Wire Line
	2750 1375 2750 1325
Wire Wire Line
	2750 1325 2600 1325
Wire Wire Line
	2850 1575 3000 1575
Wire Wire Line
	3000 1775 2850 1775
Wire Wire Line
	2825 2200 3025 2200
Wire Wire Line
	2825 2600 3025 2600
Wire Wire Line
	2725 2400 3000 2400
Wire Wire Line
	8500 1975 9050 1975
Wire Wire Line
	1800 3200 1950 3200
Wire Wire Line
	3150 3350 3300 3350
Wire Wire Line
	3100 3200 2650 3200
Wire Wire Line
	2850 3350 2850 3200
Connection ~ 2850 3200
Wire Wire Line
	9050 1775 8500 1775
$Comp
L CONN_01X03 J?
U 1 1 5B8D803F
P 2525 2400
F 0 "J?" H 2525 2600 50  0000 C CNN
F 1 "MPE 094-1-003" V 2625 2400 50  0000 C CNN
F 2 "" H 2525 2400 50  0001 C CNN
F 3 "" H 2525 2400 50  0001 C CNN
	1    2525 2400
	-1   0    0    1   
$EndComp
Wire Wire Line
	2825 2600 2825 2500
Wire Wire Line
	2825 2500 2725 2500
Wire Wire Line
	2725 2300 2825 2300
Wire Wire Line
	2825 2300 2825 2200
$EndSCHEMATC

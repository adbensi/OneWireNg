EESchema Schematic File Version 4
LIBS:parasite-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 4650 4050 0    60   Input ~ 0
OW_PIN
$Comp
L parasite-rescue:R R?
U 1 1 57F11AB2
P 5400 3600
F 0 "R?" V 5480 3600 50  0001 C CNN
F 1 "R" V 5400 3600 50  0000 C CNN
F 2 "" V 5330 3600 50  0000 C CNN
F 3 "" H 5400 3600 50  0000 C CNN
	1    5400 3600
	1    0    0    -1  
$EndComp
Text GLabel 4650 3500 0    60   Input ~ 0
PWR_CTRL_PIN
$Comp
L parasite-rescue:VCC #PWR?
U 1 1 57F121C8
P 5050 3100
F 0 "#PWR?" H 5050 2950 50  0001 C CNN
F 1 "VCC" H 5050 3250 50  0000 C CNN
F 2 "" H 5050 3100 50  0000 C CNN
F 3 "" H 5050 3100 50  0000 C CNN
	1    5050 3100
	1    0    0    -1  
$EndComp
$Comp
L parasite-rescue:Q_PMOS_GDS Q?
U 1 1 57F13865
P 4950 3500
F 0 "Q?" H 5600 3450 50  0001 R CNN
F 1 "Qpu" H 5250 3550 50  0000 R CNN
F 2 "" H 5150 3600 50  0000 C CNN
F 3 "" H 4950 3500 50  0000 C CNN
	1    4950 3500
	1    0    0    -1  
$EndComp
$Comp
L parasite-rescue:DS18B20-PAR U?
U 1 1 57F13FAF
P 6150 3400
F 0 "U?" H 6000 3650 50  0001 C CNN
F 1 "DS18B20-PAR" H 6150 3150 50  0000 C CNN
F 2 "" H 6000 3650 50  0000 C CNN
F 3 "" H 6000 3650 50  0000 C CNN
	1    6150 3400
	0    -1   -1   0   
$EndComp
$Comp
L parasite-rescue:GND #PWR?
U 1 1 57F140C5
P 6250 3850
F 0 "#PWR?" H 6250 3600 50  0001 C CNN
F 1 "GND" H 6250 3700 50  0000 C CNN
F 2 "" H 6250 3850 50  0000 C CNN
F 3 "" H 6250 3850 50  0000 C CNN
	1    6250 3850
	1    0    0    -1  
$EndComp
Connection ~ 5050 4050
Wire Wire Line
	4650 3500 4750 3500
Connection ~ 6050 4050
$Comp
L parasite-rescue:DS18B20 U?
U 1 1 57F1236E
P 6900 3400
F 0 "U?" H 6750 3650 50  0001 C CNN
F 1 "DS18B20" H 6900 3150 50  0000 C CNN
F 2 "" H 6750 3650 50  0000 C CNN
F 3 "" H 6750 3650 50  0000 C CNN
	1    6900 3400
	0    -1   -1   0   
$EndComp
$Comp
L parasite-rescue:GND #PWR?
U 1 1 57F123AC
P 7000 3850
F 0 "#PWR?" H 7000 3600 50  0001 C CNN
F 1 "GND" H 7000 3700 50  0000 C CNN
F 2 "" H 7000 3850 50  0000 C CNN
F 3 "" H 7000 3850 50  0000 C CNN
	1    7000 3850
	1    0    0    -1  
$EndComp
Connection ~ 6900 4050
Wire Wire Line
	7000 3700 7000 3800
Wire Wire Line
	6900 3700 6900 4050
Wire Wire Line
	6800 3700 6800 3800
Wire Wire Line
	6800 3800 7000 3800
Connection ~ 7000 3800
Wire Wire Line
	6250 3700 6250 3850
Wire Wire Line
	6050 3700 6050 4050
Wire Wire Line
	5050 3700 5050 4050
Connection ~ 5400 4050
Wire Wire Line
	5050 3100 5050 3150
Wire Wire Line
	5400 3450 5400 3150
Wire Wire Line
	5400 3150 5050 3150
Connection ~ 5050 3150
Wire Wire Line
	5400 3750 5400 4050
Wire Wire Line
	4650 4050 5050 4050
Wire Wire Line
	5050 4050 5400 4050
Wire Wire Line
	6050 4050 6900 4050
Wire Wire Line
	6900 4050 7250 4050
Wire Wire Line
	7000 3800 7000 3850
Wire Wire Line
	5400 4050 6050 4050
Wire Wire Line
	5050 3150 5050 3300
Connection ~ 4750 3500
Wire Wire Line
	4750 3500 5000 3500
$EndSCHEMATC

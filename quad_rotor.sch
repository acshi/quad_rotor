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
LIBS:ncp5901
LIBS:SAMD21EL
LIBS:lps25hbtr‎
LIBS:icm-20948
LIBS:atsams70j19a
LIBS:quad_rotor-cache
EELAYER 25 0
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
Text Label 3850 2050 0    60   ~ 0
SDA
Text Label 3850 2150 0    60   ~ 0
SCL
Text Label 3850 2250 0    60   ~ 0
+5V
$Comp
L GND #PWR11
U 1 1 5A459E6A
P 3850 2350
F 0 "#PWR11" H 3850 2100 50  0001 C CNN
F 1 "GND" H 3850 2200 50  0000 C CNN
F 2 "" H 3850 2350 50  0001 C CNN
F 3 "" H 3850 2350 50  0001 C CNN
	1    3850 2350
	0    -1   -1   0   
$EndComp
$Comp
L R R7
U 1 1 5A45A466
P 4200 750
F 0 "R7" V 4280 750 50  0000 C CNN
F 1 "470R" V 4200 750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4130 750 50  0001 C CNN
F 3 "" H 4200 750 50  0001 C CNN
	1    4200 750 
	0    1    1    0   
$EndComp
$Comp
L R R8
U 1 1 5A45A5FA
P 4200 1050
F 0 "R8" V 4280 1050 50  0000 C CNN
F 1 "3.9k" V 4200 1050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4130 1050 50  0001 C CNN
F 3 "" H 4200 1050 50  0001 C CNN
	1    4200 1050
	0    1    1    0   
$EndComp
$Comp
L LED D1
U 1 1 5A45A63D
P 4600 750
F 0 "D1" H 4600 850 50  0000 C CNN
F 1 "Green" H 4600 650 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 4600 750 50  0001 C CNN
F 3 "" H 4600 750 50  0001 C CNN
	1    4600 750 
	-1   0    0    1   
$EndComp
$Comp
L LED D2
U 1 1 5A45A69C
P 4600 1050
F 0 "D2" H 4600 1150 50  0000 C CNN
F 1 "Yellow" H 4600 950 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 4600 1050 50  0001 C CNN
F 3 "" H 4600 1050 50  0001 C CNN
	1    4600 1050
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR9
U 1 1 5A45A753
P 4850 750
F 0 "#PWR9" H 4850 500 50  0001 C CNN
F 1 "GND" H 4850 600 50  0000 C CNN
F 2 "" H 4850 750 50  0001 C CNN
F 3 "" H 4850 750 50  0001 C CNN
	1    4850 750 
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR10
U 1 1 5A45A7C7
P 4850 1050
F 0 "#PWR10" H 4850 800 50  0001 C CNN
F 1 "GND" H 4850 900 50  0000 C CNN
F 2 "" H 4850 1050 50  0001 C CNN
F 3 "" H 4850 1050 50  0001 C CNN
	1    4850 1050
	0    -1   -1   0   
$EndComp
Text Label 3950 750  2    60   ~ 0
+5V
$Comp
L LED D3
U 1 1 5A45AE8F
P 4600 1350
F 0 "D3" H 4600 1450 50  0000 C CNN
F 1 "Red Forward" H 4600 1250 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 4600 1350 50  0001 C CNN
F 3 "" H 4600 1350 50  0001 C CNN
	1    4600 1350
	-1   0    0    1   
$EndComp
$Comp
L LED D4
U 1 1 5A45AEDA
P 4600 1650
F 0 "D4" H 4600 1750 50  0000 C CNN
F 1 "Red Backward" H 4600 1550 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 4600 1650 50  0001 C CNN
F 3 "" H 4600 1650 50  0001 C CNN
	1    4600 1650
	1    0    0    1   
$EndComp
$Comp
L R R9
U 1 1 5A45B0E3
P 4200 1350
F 0 "R9" V 4280 1350 50  0000 C CNN
F 1 "3k" V 4200 1350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4130 1350 50  0001 C CNN
F 3 "" H 4200 1350 50  0001 C CNN
	1    4200 1350
	0    1    1    0   
$EndComp
Text Label 3950 1350 2    60   ~ 0
MOT_A
Text Label 4900 2050 0    60   ~ 0
SDA
Text Label 4900 2150 0    60   ~ 0
SCL
Text Label 4900 2250 0    60   ~ 0
+5V
$Comp
L GND #PWR15
U 1 1 5A45CA87
P 4900 2350
F 0 "#PWR15" H 4900 2100 50  0001 C CNN
F 1 "GND" H 4900 2200 50  0000 C CNN
F 2 "" H 4900 2350 50  0001 C CNN
F 3 "" H 4900 2350 50  0001 C CNN
	1    4900 2350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4350 750  4450 750 
Wire Wire Line
	4750 750  4850 750 
Wire Wire Line
	4850 1050 4750 1050
Wire Wire Line
	4450 1050 4350 1050
Wire Wire Line
	4050 750  3950 750 
Wire Wire Line
	4050 1050 3950 1050
Wire Wire Line
	4350 1350 4450 1350
Wire Wire Line
	4750 1350 4900 1350
Wire Wire Line
	4750 1650 4800 1650
Wire Wire Line
	4800 1650 4800 1350
Connection ~ 4800 1350
Wire Wire Line
	3950 1350 4050 1350
Wire Wire Line
	4400 1350 4400 1650
Connection ~ 4400 1350
Wire Wire Line
	4400 1650 4450 1650
$Comp
L R R6
U 1 1 5A46FA01
P 2400 1850
F 0 "R6" V 2480 1850 50  0000 C CNN
F 1 "100k" V 2400 1850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2330 1850 50  0001 C CNN
F 3 "" H 2400 1850 50  0001 C CNN
	1    2400 1850
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 5A46FACA
P 2000 1850
F 0 "R3" V 2080 1850 50  0000 C CNN
F 1 "402k" V 2000 1850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1930 1850 50  0001 C CNN
F 3 "" H 2000 1850 50  0001 C CNN
	1    2000 1850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 1850 1850 1850
Wire Wire Line
	2150 1850 2250 1850
Wire Wire Line
	2550 1850 2650 1850
$Comp
L GND #PWR4
U 1 1 5A47082A
P 2650 1850
F 0 "#PWR4" H 2650 1600 50  0001 C CNN
F 1 "GND" H 2650 1700 50  0000 C CNN
F 2 "" H 2650 1850 50  0001 C CNN
F 3 "" H 2650 1850 50  0001 C CNN
	1    2650 1850
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 5A470DF6
P 2400 1200
F 0 "R5" V 2480 1200 50  0000 C CNN
F 1 "100k" V 2400 1200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2330 1200 50  0001 C CNN
F 3 "" H 2400 1200 50  0001 C CNN
	1    2400 1200
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 5A470DFC
P 2000 1200
F 0 "R2" V 2080 1200 50  0000 C CNN
F 1 "499k" V 2000 1200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1930 1200 50  0001 C CNN
F 3 "" H 2000 1200 50  0001 C CNN
	1    2000 1200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 1200 1850 1200
Wire Wire Line
	2150 1200 2250 1200
Wire Wire Line
	2550 1200 2650 1200
$Comp
L GND #PWR3
U 1 1 5A470E0B
P 2650 1200
F 0 "#PWR3" H 2650 950 50  0001 C CNN
F 1 "GND" H 2650 1050 50  0000 C CNN
F 2 "" H 2650 1200 50  0001 C CNN
F 3 "" H 2650 1200 50  0001 C CNN
	1    2650 1200
	0    -1   -1   0   
$EndComp
Text Label 1750 1200 2    60   ~ 0
+5V
Text Notes 950  950  0    60   ~ 0
Use 0.46mm solder!\n262-265C (510F) for rework to not damage pads\nelectrolube flux pen
$Comp
L C C1
U 1 1 5A47330C
P 2400 1350
F 0 "C1" H 2425 1450 50  0000 L CNN
F 1 "100nF" H 2425 1250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2438 1200 50  0001 C CNN
F 3 "" H 2400 1350 50  0001 C CNN
	1    2400 1350
	0    1    1    0   
$EndComp
$Comp
L C C2
U 1 1 5A4733A9
P 2400 2000
F 0 "C2" H 2425 2100 50  0000 L CNN
F 1 "100nF" H 2425 1900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2438 1850 50  0001 C CNN
F 3 "" H 2400 2000 50  0001 C CNN
	1    2400 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	2200 1350 2250 1350
Wire Wire Line
	2200 1050 2200 1350
Connection ~ 2200 1200
Wire Wire Line
	2550 1350 2600 1350
Wire Wire Line
	2600 1350 2600 1200
Connection ~ 2600 1200
Wire Wire Line
	2200 2000 2250 2000
Wire Wire Line
	2200 1700 2200 2000
Connection ~ 2200 1850
Wire Wire Line
	2550 2000 2600 2000
Wire Wire Line
	2600 2000 2600 1850
Connection ~ 2600 1850
Text Label 2200 1050 0    60   ~ 0
5V_READ
Text Label 2200 1700 0    60   ~ 0
V_MOTOR_READ
Text Label 1750 1850 2    60   ~ 0
+V_MOTOR
Text Label 3950 1050 2    60   ~ 0
+V_MOTOR
Text Notes 1300 1400 0    60   ~ 0
Compare to 1.1V
Text Notes 950  2100 0    60   ~ 0
Compare to VCC\n(as measured by 5V_READ)
$Comp
L CONN_01X04 J2
U 1 1 5A47CBA3
P 3550 2200
F 0 "J2" H 3550 2450 50  0000 C CNN
F 1 "I2C 1" V 3650 2200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 3550 2200 50  0001 C CNN
F 3 "" H 3550 2200 50  0001 C CNN
	1    3550 2200
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X04 J3
U 1 1 5A47CC54
P 4600 2200
F 0 "J3" H 4600 2450 50  0000 C CNN
F 1 "I2C 2" V 4700 2200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4600 2200 50  0001 C CNN
F 3 "" H 4600 2200 50  0001 C CNN
	1    4600 2200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4900 2350 4800 2350
Wire Wire Line
	4800 2250 4900 2250
Wire Wire Line
	4900 2150 4800 2150
Wire Wire Line
	4800 2050 4900 2050
Wire Wire Line
	3750 2050 3850 2050
Wire Wire Line
	3850 2150 3750 2150
Wire Wire Line
	3750 2250 3850 2250
Wire Wire Line
	3850 2350 3750 2350
Text Label 4900 1350 0    60   ~ 0
MOT_B
$Comp
L LPS25HBTR‎ U?
U 1 1 5B88DA5E
P 7300 1800
F 0 "U?" H 7450 1400 60  0000 C CNN
F 1 "LPS25HBTR‎" H 7200 2200 60  0000 C CNN
F 2 "" H 6900 1550 60  0001 C CNN
F 3 "" H 6900 1550 60  0001 C CNN
	1    7300 1800
	1    0    0    -1  
$EndComp
$Comp
L ICM-20948 U?
U 1 1 5B88E001
P 9400 2350
F 0 "U?" H 9650 2150 60  0000 C CNN
F 1 "ICM-20948" H 9250 3250 60  0000 C CNN
F 2 "" H 9250 2350 60  0001 C CNN
F 3 "" H 9250 2350 60  0001 C CNN
	1    9400 2350
	1    0    0    -1  
$EndComp
$Comp
L ATSAMS70J19A U?
U 1 1 5B8B731E
P 4600 5050
F 0 "U?" H 6450 3500 60  0000 C CNN
F 1 "ATSAMS70J19A" H 3000 7250 60  0000 C CNN
F 2 "" H 4650 3900 60  0001 C CNN
F 3 "" H 4650 3900 60  0001 C CNN
	1    4600 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 4550 2400 4550
Text Label 2250 4550 2    60   ~ 0
+3.3V
Wire Wire Line
	2300 4650 2400 4650
Connection ~ 2300 4550
Wire Wire Line
	2300 4750 2400 4750
Connection ~ 2300 4650
Connection ~ 2300 4750
Wire Wire Line
	2250 4850 2400 4850
Connection ~ 2300 4850
Wire Wire Line
	2250 5050 2400 5050
Connection ~ 2300 5050
Wire Wire Line
	2300 4950 2400 4950
Connection ~ 2300 4950
$Comp
L C_Small C?
U 1 1 5B8E1B1D
P 700 7250
F 0 "C?" H 710 7320 50  0000 L CNN
F 1 "100nF" H 710 7170 50  0000 L CNN
F 2 "" H 700 7250 50  0001 C CNN
F 3 "" H 700 7250 50  0001 C CNN
	1    700  7250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B8E1C7C
P 700 7450
F 0 "#PWR?" H 700 7200 50  0001 C CNN
F 1 "GND" H 700 7300 50  0000 C CNN
F 2 "" H 700 7450 50  0001 C CNN
F 3 "" H 700 7450 50  0001 C CNN
	1    700  7450
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E1D1D
P 850 7250
F 0 "C?" H 860 7320 50  0000 L CNN
F 1 "100nF" H 850 7400 50  0000 L CNN
F 2 "" H 850 7250 50  0001 C CNN
F 3 "" H 850 7250 50  0001 C CNN
	1    850  7250
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E1D6F
P 1000 7250
F 0 "C?" H 1010 7320 50  0000 L CNN
F 1 "100nF" H 1010 7170 50  0000 L CNN
F 2 "" H 1000 7250 50  0001 C CNN
F 3 "" H 1000 7250 50  0001 C CNN
	1    1000 7250
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E1DC0
P 1150 7250
F 0 "C?" H 1160 7320 50  0000 L CNN
F 1 "100nF" H 1150 7400 50  0000 L CNN
F 2 "" H 1150 7250 50  0001 C CNN
F 3 "" H 1150 7250 50  0001 C CNN
	1    1150 7250
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E1F96
P 1300 7250
F 0 "C?" H 1310 7320 50  0000 L CNN
F 1 "100nF" H 1310 7170 50  0000 L CNN
F 2 "" H 1300 7250 50  0001 C CNN
F 3 "" H 1300 7250 50  0001 C CNN
	1    1300 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  7350 700  7450
Wire Wire Line
	700  7400 1300 7400
Wire Wire Line
	1300 7400 1300 7350
Connection ~ 700  7400
Wire Wire Line
	1150 7400 1150 7350
Connection ~ 1150 7400
Wire Wire Line
	1000 7400 1000 7350
Connection ~ 1000 7400
Wire Wire Line
	850  7400 850  7350
Connection ~ 850  7400
Wire Wire Line
	700  7000 700  7150
Wire Wire Line
	700  7100 1300 7100
Wire Wire Line
	1300 7100 1300 7150
Wire Wire Line
	1150 7150 1150 7100
Connection ~ 1150 7100
Wire Wire Line
	1000 7150 1000 7100
Connection ~ 1000 7100
Wire Wire Line
	850  7150 850  7100
Connection ~ 850  7100
Connection ~ 700  7100
Text Label 700  7000 0    60   ~ 0
+3.3V
Text Notes 800  6850 0    60   ~ 0
for VDDIO
$Comp
L C_Small C?
U 1 1 5B8E2663
P 2150 4850
F 0 "C?" V 2100 4700 50  0000 L CNN
F 1 "100nF" V 2050 4700 50  0000 L CNN
F 2 "" H 2150 4850 50  0001 C CNN
F 3 "" H 2150 4850 50  0001 C CNN
	1    2150 4850
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5B8E26C9
P 1900 4850
F 0 "#PWR?" H 1900 4600 50  0001 C CNN
F 1 "GND" H 1900 4700 50  0000 C CNN
F 2 "" H 1900 4850 50  0001 C CNN
F 3 "" H 1900 4850 50  0001 C CNN
	1    1900 4850
	0    1    1    0   
$EndComp
Wire Wire Line
	1900 4850 2050 4850
Wire Wire Line
	2300 4550 2300 5050
$Comp
L R R?
U 1 1 5B8E31FE
P 1000 5150
F 0 "R?" V 1080 5150 50  0000 C CNN
F 1 "2.2R" V 1000 5150 50  0000 C CNN
F 2 "" V 930 5150 50  0001 C CNN
F 3 "" H 1000 5150 50  0001 C CNN
	1    1000 5150
	0    1    1    0   
$EndComp
$Comp
L L L?
U 1 1 5B8E3265
P 1350 5150
F 0 "L?" V 1300 5150 50  0000 C CNN
F 1 "10uH" V 1425 5150 50  0000 C CNN
F 2 "" H 1350 5150 50  0001 C CNN
F 3 "" H 1350 5150 50  0001 C CNN
F 4 "60mA" V 1500 5150 60  0000 C CNN "Current"
	1    1350 5150
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E32CC
P 1650 5250
F 0 "C?" H 1660 5320 50  0000 L CNN
F 1 "4.7uF" H 1650 5200 50  0000 L CNN
F 2 "" H 1650 5250 50  0001 C CNN
F 3 "" H 1650 5250 50  0001 C CNN
	1    1650 5250
	-1   0    0    1   
$EndComp
Connection ~ 1650 5150
Wire Wire Line
	1200 5150 1150 5150
Text Label 800  5150 2    60   ~ 0
+3.3V
Wire Wire Line
	800  5150 850  5150
$Comp
L GND #PWR?
U 1 1 5B8E3B43
P 1650 5400
F 0 "#PWR?" H 1650 5150 50  0001 C CNN
F 1 "GND" H 1650 5250 50  0000 C CNN
F 2 "" H 1650 5400 50  0001 C CNN
F 3 "" H 1650 5400 50  0001 C CNN
	1    1650 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 5400 1650 5350
Wire Wire Line
	1500 5150 2400 5150
$Comp
L C_Small C?
U 1 1 5B8E3E31
P 2150 5050
F 0 "C?" V 2100 4900 50  0000 L CNN
F 1 "100nF" V 2050 4900 50  0000 L CNN
F 2 "" H 2150 5050 50  0001 C CNN
F 3 "" H 2150 5050 50  0001 C CNN
	1    2150 5050
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 4850 1950 5050
Wire Wire Line
	1950 5050 2050 5050
Connection ~ 1950 4850
$Comp
L C_Small C?
U 1 1 5B8E46F7
P 1800 7250
F 0 "C?" H 1810 7320 50  0000 L CNN
F 1 "100nF" H 1810 7170 50  0000 L CNN
F 2 "" H 1800 7250 50  0001 C CNN
F 3 "" H 1800 7250 50  0001 C CNN
	1    1800 7250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5B8E46FD
P 1800 7450
F 0 "#PWR?" H 1800 7200 50  0001 C CNN
F 1 "GND" H 1800 7300 50  0000 C CNN
F 2 "" H 1800 7450 50  0001 C CNN
F 3 "" H 1800 7450 50  0001 C CNN
	1    1800 7450
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E4703
P 1950 7250
F 0 "C?" H 1960 7320 50  0000 L CNN
F 1 "100nF" H 1950 7400 50  0000 L CNN
F 2 "" H 1950 7250 50  0001 C CNN
F 3 "" H 1950 7250 50  0001 C CNN
	1    1950 7250
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E4709
P 2100 7250
F 0 "C?" H 2110 7320 50  0000 L CNN
F 1 "100nF" H 2110 7170 50  0000 L CNN
F 2 "" H 2100 7250 50  0001 C CNN
F 3 "" H 2100 7250 50  0001 C CNN
	1    2100 7250
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E470F
P 2250 7250
F 0 "C?" H 2260 7320 50  0000 L CNN
F 1 "100nF" H 2250 7400 50  0000 L CNN
F 2 "" H 2250 7250 50  0001 C CNN
F 3 "" H 2250 7250 50  0001 C CNN
	1    2250 7250
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E4715
P 2400 7250
F 0 "C?" H 2410 7320 50  0000 L CNN
F 1 "100nF" H 2410 7170 50  0000 L CNN
F 2 "" H 2400 7250 50  0001 C CNN
F 3 "" H 2400 7250 50  0001 C CNN
	1    2400 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 7350 1800 7450
Wire Wire Line
	1800 7400 2400 7400
Wire Wire Line
	2400 7400 2400 7350
Connection ~ 1800 7400
Wire Wire Line
	2250 7400 2250 7350
Connection ~ 2250 7400
Wire Wire Line
	2100 7400 2100 7350
Connection ~ 2100 7400
Wire Wire Line
	1950 7400 1950 7350
Connection ~ 1950 7400
Wire Wire Line
	1800 7000 1800 7150
Wire Wire Line
	1800 7100 2400 7100
Wire Wire Line
	2400 7100 2400 7150
Wire Wire Line
	2250 7150 2250 7100
Connection ~ 2250 7100
Wire Wire Line
	2100 7150 2100 7100
Connection ~ 2100 7100
Wire Wire Line
	1950 7150 1950 7100
Connection ~ 1950 7100
Connection ~ 1800 7100
Text Label 1800 7000 0    60   ~ 0
VDDCORE
Text Notes 1900 6900 0    60   ~ 0
for VDDCORE
Text Label 2300 5650 2    60   ~ 0
VDDCORE
Wire Wire Line
	2300 5650 2400 5650
Wire Wire Line
	2350 5300 2350 5650
Wire Wire Line
	2350 5450 2400 5450
Connection ~ 2350 5650
Wire Wire Line
	2350 5550 2400 5550
Connection ~ 2350 5550
Wire Wire Line
	2300 5350 2400 5350
Connection ~ 2350 5450
$Comp
L C_Small C?
U 1 1 5B8E4FAB
P 2150 5300
F 0 "C?" V 2100 5150 50  0000 L CNN
F 1 "100nF" V 2050 5150 50  0000 L CNN
F 2 "" H 2150 5300 50  0001 C CNN
F 3 "" H 2150 5300 50  0001 C CNN
	1    2150 5300
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E5029
P 2150 5500
F 0 "C?" V 2100 5350 50  0000 L CNN
F 1 "1uF" V 2050 5350 50  0000 L CNN
F 2 "" H 2150 5500 50  0001 C CNN
F 3 "" H 2150 5500 50  0001 C CNN
	1    2150 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 5300 2350 5300
Connection ~ 2350 5350
Wire Wire Line
	2250 5500 2300 5500
Wire Wire Line
	2300 5500 2300 5350
$Comp
L GND #PWR?
U 1 1 5B8E51FC
P 1950 5300
F 0 "#PWR?" H 1950 5050 50  0001 C CNN
F 1 "GND" H 1950 5150 50  0000 C CNN
F 2 "" H 1950 5300 50  0001 C CNN
F 3 "" H 1950 5300 50  0001 C CNN
	1    1950 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	2000 5500 2000 5300
Wire Wire Line
	1950 5300 2050 5300
Connection ~ 2000 5300
Wire Wire Line
	2000 5500 2050 5500
$Comp
L L L?
U 1 1 5B8E55B6
P 1900 5750
F 0 "L?" V 1850 5750 50  0000 C CNN
F 1 "470R@100MHz" V 1975 5750 50  0000 C CNN
F 2 "" H 1900 5750 50  0001 C CNN
F 3 "" H 1900 5750 50  0001 C CNN
	1    1900 5750
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E585C
P 2250 5850
F 0 "C?" V 2200 5700 50  0000 L CNN
F 1 "100nF" V 2150 5700 50  0000 L CNN
F 2 "" H 2250 5850 50  0001 C CNN
F 3 "" H 2250 5850 50  0001 C CNN
	1    2250 5850
	-1   0    0    1   
$EndComp
Wire Wire Line
	2050 5750 2400 5750
Connection ~ 2250 5750
Text Label 1700 5750 2    60   ~ 0
VDDCORE
Wire Wire Line
	1700 5750 1750 5750
$Comp
L GND #PWR?
U 1 1 5B8E5EFE
P 2300 6350
F 0 "#PWR?" H 2300 6100 50  0001 C CNN
F 1 "GND" H 2300 6200 50  0000 C CNN
F 2 "" H 2300 6350 50  0001 C CNN
F 3 "" H 2300 6350 50  0001 C CNN
	1    2300 6350
	0    1    1    0   
$EndComp
Wire Wire Line
	2300 6350 2400 6350
Wire Wire Line
	2350 6150 2350 6350
Wire Wire Line
	2350 6150 2400 6150
Connection ~ 2350 6350
Wire Wire Line
	2350 6250 2400 6250
Connection ~ 2350 6250
$Comp
L GND #PWR?
U 1 1 5B8E62BF
P 2200 5950
F 0 "#PWR?" H 2200 5700 50  0001 C CNN
F 1 "GND" H 2200 5800 50  0000 C CNN
F 2 "" H 2200 5950 50  0001 C CNN
F 3 "" H 2200 5950 50  0001 C CNN
	1    2200 5950
	0    1    1    0   
$EndComp
Wire Wire Line
	2200 5950 2250 5950
$Comp
L L L?
U 1 1 5B8E64C4
P 1450 6100
F 0 "L?" V 1400 6100 50  0000 C CNN
F 1 "10uH" V 1525 6100 50  0000 C CNN
F 2 "" H 1450 6100 50  0001 C CNN
F 3 "" H 1450 6100 50  0001 C CNN
F 4 "60mA" V 1600 6100 60  0000 C CNN "Current"
	1    1450 6100
	0    1    1    0   
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E6561
P 1700 6200
F 0 "C?" V 1650 6050 50  0000 L CNN
F 1 "100nF" V 1600 6050 50  0000 L CNN
F 2 "" H 1700 6200 50  0001 C CNN
F 3 "" H 1700 6200 50  0001 C CNN
	1    1700 6200
	-1   0    0    1   
$EndComp
$Comp
L C_Small C?
U 1 1 5B8E6600
P 1900 6200
F 0 "C?" V 1850 6050 50  0000 L CNN
F 1 "100nF" V 1800 6050 50  0000 L CNN
F 2 "" H 1900 6200 50  0001 C CNN
F 3 "" H 1900 6200 50  0001 C CNN
	1    1900 6200
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5B8E6690
P 1800 6350
F 0 "#PWR?" H 1800 6100 50  0001 C CNN
F 1 "GND" H 1800 6200 50  0000 C CNN
F 2 "" H 1800 6350 50  0001 C CNN
F 3 "" H 1800 6350 50  0001 C CNN
	1    1800 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 6300 1700 6350
Wire Wire Line
	1900 6350 1900 6300
Wire Wire Line
	1700 6350 1900 6350
Connection ~ 1800 6350
Wire Wire Line
	1600 6100 2400 6100
Connection ~ 1700 6100
Wire Wire Line
	2400 6100 2400 5950
Connection ~ 1900 6100
Text Label 1200 6100 2    60   ~ 0
+3.3V
Wire Wire Line
	1200 6100 1300 6100
$EndSCHEMATC

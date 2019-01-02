EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
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
+3.3V
$Comp
L power:GND #PWR01
U 1 1 5A459E6A
P 3850 2350
F 0 "#PWR01" H 3850 2100 50  0001 C CNN
F 1 "GND" H 3850 2200 50  0000 C CNN
F 2 "" H 3850 2350 50  0001 C CNN
F 3 "" H 3850 2350 50  0001 C CNN
	1    3850 2350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5A45A466
P 4200 750
F 0 "R1" V 4280 750 50  0000 C CNN
F 1 "470R" V 4200 750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4130 750 50  0001 C CNN
F 3 "" H 4200 750 50  0001 C CNN
	1    4200 750 
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5A45A5FA
P 4200 1050
F 0 "R2" V 4280 1050 50  0000 C CNN
F 1 "4.02k" V 4200 1050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4130 1050 50  0001 C CNN
F 3 "" H 4200 1050 50  0001 C CNN
	1    4200 1050
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5A45A63D
P 4600 750
F 0 "D1" H 4600 850 50  0000 C CNN
F 1 "Green" H 4600 650 50  0000 C CNN
F 2 "LEDs:LED_0603_HandSoldering" H 4600 750 50  0001 C CNN
F 3 "" H 4600 750 50  0001 C CNN
	1    4600 750 
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D2
U 1 1 5A45A69C
P 4600 1050
F 0 "D2" H 4600 1150 50  0000 C CNN
F 1 "Yellow" H 4600 950 50  0000 C CNN
F 2 "LEDs:LED_0603_HandSoldering" H 4600 1050 50  0001 C CNN
F 3 "" H 4600 1050 50  0001 C CNN
	1    4600 1050
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5A45A753
P 4850 750
F 0 "#PWR02" H 4850 500 50  0001 C CNN
F 1 "GND" H 4850 600 50  0000 C CNN
F 2 "" H 4850 750 50  0001 C CNN
F 3 "" H 4850 750 50  0001 C CNN
	1    4850 750 
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5A45A7C7
P 4850 1050
F 0 "#PWR03" H 4850 800 50  0001 C CNN
F 1 "GND" H 4850 900 50  0000 C CNN
F 2 "" H 4850 1050 50  0001 C CNN
F 3 "" H 4850 1050 50  0001 C CNN
	1    4850 1050
	0    -1   -1   0   
$EndComp
Text Label 3950 750  2    60   ~ 0
+3.3V
Text Notes 950  950  0    60   ~ 0
Use 0.46mm solder!\n262-265C (510F) for rework to not damage pads\nelectrolube flux pen
Text Label 3950 1050 2    60   ~ 0
+V_MOTOR
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5A47CBA3
P 3550 2150
F 0 "J1" H 3550 2400 50  0000 C CNN
F 1 "I2C 1" V 3450 2150 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3550 2150 50  0001 C CNN
F 3 "" H 3550 2150 50  0001 C CNN
	1    3550 2150
	1    0    0    -1  
$EndComp
$Sheet
S 6250 4050 1200 1400
U 5AE1F86D
F0 "bldc_unit1" 60
F1 "bldc_unit.sch" 60
F2 "+V_MOTOR" I L 6250 4200 60 
F3 "CURRENT" O R 7450 4200 60 
F4 "MOTOR_A" O R 7450 4350 60 
F5 "MOTOR_B" O R 7450 4500 60 
F6 "MOTOR_C" O R 7450 4650 60 
F7 "+3.3V" I R 7450 5350 60 
F8 "IN_HA" I L 6250 4350 60 
F9 "IN_HB" I L 6250 4650 60 
F10 "IN_HC" I L 6250 4950 60 
F11 "IN_LA" I L 6250 4500 60 
F12 "IN_LB" I L 6250 4800 60 
F13 "ENABLE" I R 7450 5200 60 
F14 "IN_LC" I L 6250 5100 60 
F15 "~FAULT" O R 7450 5050 60 
F16 "MOTOR_ZERO" O R 7450 4800 60 
$EndSheet
$Comp
L SAMD21EL:SAMD21E16L-MU U1
U 1 1 5AE30308
P 3900 4700
F 0 "U1" H 2850 6100 50  0000 C CNN
F 1 "SAMD21E16L-MU" H 4750 3300 50  0000 C CNN
F 2 "Housings_QFP:TQFP-32_7x7mm_Pitch0.8mm" H 3900 3700 50  0001 C CIN
F 3 "" H 3900 4700 50  0001 C CNN
	1    3900 4700
	1    0    0    -1  
$EndComp
Text Label 5250 4650 0    60   ~ 0
SDA
Text Label 5250 4750 0    60   ~ 0
SCL
Text Label 2550 3450 2    60   ~ 0
RESET
$Comp
L power:GND #PWR04
U 1 1 5AE35169
P 2550 6000
F 0 "#PWR04" H 2550 5750 50  0001 C CNN
F 1 "GND" H 2550 5850 50  0000 C CNN
F 2 "" H 2550 6000 50  0001 C CNN
F 3 "" H 2550 6000 50  0001 C CNN
	1    2550 6000
	1    0    0    -1  
$EndComp
Text Label 5250 5450 0    60   ~ 0
SWCLK
Text Label 5250 5550 0    60   ~ 0
SWDIO
$Comp
L Device:C C4
U 1 1 5AE3A6DB
P 2150 4600
F 0 "C4" H 2175 4700 50  0000 L CNN
F 1 "1uF" H 2175 4500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2188 4450 50  0001 C CNN
F 3 "" H 2150 4600 50  0001 C CNN
F 4 "25V" H 2250 4400 60  0000 C CNN "Voltage"
	1    2150 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5AE3A817
P 1850 4600
F 0 "C3" H 1875 4700 50  0000 L CNN
F 1 "100nF" H 1875 4500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1888 4450 50  0001 C CNN
F 3 "" H 1850 4600 50  0001 C CNN
	1    1850 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5AE3AF0C
P 2450 4600
F 0 "C5" H 2475 4700 50  0000 L CNN
F 1 "100nF" H 2475 4500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2488 4450 50  0001 C CNN
F 3 "" H 2450 4600 50  0001 C CNN
	1    2450 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5AE3C0E9
P 2150 4950
F 0 "#PWR05" H 2150 4700 50  0001 C CNN
F 1 "GND" H 2150 4800 50  0000 C CNN
F 2 "" H 2150 4950 50  0001 C CNN
F 3 "" H 2150 4950 50  0001 C CNN
	1    2150 4950
	1    0    0    -1  
$EndComp
Text Label 700  3350 0    60   ~ 0
+3.3V
$Comp
L Device:C C2
U 1 1 5AE3DEF4
P 1400 4600
F 0 "C2" H 1425 4700 50  0000 L CNN
F 1 "10uF" H 1425 4500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1438 4450 50  0001 C CNN
F 3 "" H 1400 4600 50  0001 C CNN
	1    1400 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5AE3DFED
P 700 4600
F 0 "C1" H 725 4700 50  0000 L CNN
F 1 "10uF" H 725 4500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 738 4450 50  0001 C CNN
F 3 "" H 700 4600 50  0001 C CNN
	1    700  4600
	1    0    0    -1  
$EndComp
$Comp
L Device:Ferrite_Bead_Small L2
U 1 1 5AE9408E
P 1050 4350
F 0 "L2" V 1100 4450 50  0000 L CNN
F 1 "Ferrite_Bead_Small" V 1200 3900 50  0000 L CNN
F 2 "Inductors_SMD:L_0603_HandSoldering" V 980 4350 50  0001 C CNN
F 3 "" H 1050 4350 50  0001 C CNN
	1    1050 4350
	0    -1   -1   0   
$EndComp
$Comp
L Device:L L1
U 1 1 5AE952B7
P 700 3550
F 0 "L1" V 650 3550 50  0000 C CNN
F 1 "10uH" V 775 3550 50  0000 C CNN
F 2 "Inductors_SMD:L_0805_HandSoldering" H 700 3550 50  0001 C CNN
F 3 "" H 700 3550 50  0001 C CNN
	1    700  3550
	1    0    0    -1  
$EndComp
Text Label 9950 4550 0    60   ~ 0
SWCLK
$Comp
L Device:R R4
U 1 1 5AE96F2C
P 9700 4550
F 0 "R4" V 9780 4550 50  0000 C CNN
F 1 "3k" V 9700 4550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 9630 4550 50  0001 C CNN
F 3 "" H 9700 4550 50  0001 C CNN
	1    9700 4550
	0    -1   -1   0   
$EndComp
Text Label 9450 4550 2    60   ~ 0
+3.3V
$Comp
L sam-swd-10:SAM-SWD-10 CON1
U 1 1 5AE98F5C
P 9600 3850
F 0 "CON1" H 9430 4180 50  0000 C CNN
F 1 "SAM-SWD-10" H 9300 3450 50  0000 L BNN
F 2 "Pin_Headers:Pin_Header_Straight_2x05_Pitch2.54mm" V 9030 3870 50  0001 C CNN
F 3 "" H 9850 3850 50  0001 C CNN
	1    9600 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5AE99290
P 9050 3950
F 0 "#PWR06" H 9050 3700 50  0001 C CNN
F 1 "GND" H 9050 3800 50  0000 C CNN
F 2 "" H 9050 3950 50  0001 C CNN
F 3 "" H 9050 3950 50  0001 C CNN
	1    9050 3950
	0    1    1    0   
$EndComp
Text Label 9050 3650 2    60   ~ 0
+3.3V
Text Label 10250 3650 0    60   ~ 0
SWDIO
Text Label 10250 3750 0    60   ~ 0
SWCLK
Text Label 10250 4050 0    60   ~ 0
RESET
Text Label 3950 1350 2    60   ~ 0
STATUS
$Comp
L Device:R R3
U 1 1 5A45B0E3
P 4200 1350
F 0 "R3" V 4280 1350 50  0000 C CNN
F 1 "3k" V 4200 1350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4130 1350 50  0001 C CNN
F 3 "" H 4200 1350 50  0001 C CNN
	1    4200 1350
	0    1    1    0   
$EndComp
$Comp
L Device:LED D3
U 1 1 5A45AE8F
P 4600 1350
F 0 "D3" H 4600 1450 50  0000 C CNN
F 1 "Red" H 4600 1250 50  0000 C CNN
F 2 "LEDs:LED_0603_HandSoldering" H 4600 1350 50  0001 C CNN
F 3 "" H 4600 1350 50  0001 C CNN
	1    4600 1350
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5AF14C01
P 4850 1350
F 0 "#PWR07" H 4850 1100 50  0001 C CNN
F 1 "GND" H 4850 1200 50  0000 C CNN
F 2 "" H 4850 1350 50  0001 C CNN
F 3 "" H 4850 1350 50  0001 C CNN
	1    4850 1350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4750 1350 4850 1350
Wire Wire Line
	4350 1350 4450 1350
Wire Wire Line
	3950 1350 4050 1350
Wire Wire Line
	10100 4050 10250 4050
Wire Wire Line
	10100 3750 10250 3750
Wire Wire Line
	10250 3650 10100 3650
Wire Wire Line
	9050 3650 9150 3650
Connection ~ 9050 3850
Wire Wire Line
	9050 3750 9150 3750
Wire Wire Line
	9050 3850 9150 3850
Connection ~ 9050 3950
Wire Wire Line
	9050 3750 9050 3850
Wire Wire Line
	9050 4050 9150 4050
Wire Wire Line
	9450 4550 9550 4550
Wire Wire Line
	9950 4550 9850 4550
Wire Wire Line
	700  3350 700  3400
Wire Wire Line
	700  4350 950  4350
Connection ~ 1400 4350
Wire Wire Line
	1400 4450 1400 4350
Connection ~ 1400 4850
Wire Wire Line
	1400 4750 1400 4850
Connection ~ 1850 3850
Connection ~ 700  3850
Connection ~ 700  4350
Connection ~ 2450 4350
Connection ~ 1850 4850
Wire Wire Line
	700  4750 700  4850
Wire Wire Line
	700  3700 700  3850
Connection ~ 2150 4850
Wire Wire Line
	2450 4850 2450 4750
Wire Wire Line
	2150 4750 2150 4850
Wire Wire Line
	700  4850 1400 4850
Wire Wire Line
	1850 4850 1850 4750
Wire Wire Line
	700  3850 1850 3850
Wire Wire Line
	2150 4000 2650 4000
Wire Wire Line
	2150 4450 2150 4000
Wire Wire Line
	2450 4350 2450 4450
Wire Wire Line
	1150 4350 1400 4350
Wire Wire Line
	1850 3850 1850 4450
Wire Wire Line
	5150 5550 5250 5550
Wire Wire Line
	5150 5450 5250 5450
Connection ~ 2550 5950
Wire Wire Line
	2550 5950 2650 5950
Wire Wire Line
	2550 5850 2550 5950
Wire Wire Line
	2650 5850 2550 5850
Wire Wire Line
	2650 3450 2550 3450
Wire Wire Line
	5150 4750 5250 4750
Wire Wire Line
	5150 4650 5250 4650
Wire Wire Line
	3850 2350 3750 2350
Wire Wire Line
	3750 2250 3850 2250
Wire Wire Line
	3850 2150 3750 2150
Wire Wire Line
	3750 2050 3850 2050
Wire Wire Line
	4050 1050 3950 1050
Wire Wire Line
	4050 750  3950 750 
Wire Wire Line
	4450 1050 4350 1050
Wire Wire Line
	4850 1050 4750 1050
Wire Wire Line
	4750 750  4850 750 
Wire Wire Line
	4350 750  4450 750 
Wire Wire Line
	7450 5350 7650 5350
Text Label 7650 5350 0    60   ~ 0
+3.3V
Text Notes 7350 1000 0    60   ~ 0
For reasons of space and location,\nI am thinking the main caps for the motor power should be directly soldered\nto the two power busses and not have a footprint on the board.
$Comp
L Device:C C6
U 1 1 5B01CE3F
P 7650 1500
F 0 "C6" H 7675 1600 50  0000 L CNN
F 1 "330uF" H 7675 1400 50  0000 L CNN
F 2 "" H 7688 1350 50  0001 C CNN
F 3 "" H 7650 1500 50  0001 C CNN
	1    7650 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 1350 7650 1250
Wire Wire Line
	7650 1650 7650 1750
$Comp
L power:GND #PWR08
U 1 1 5B01D481
P 7650 1750
F 0 "#PWR08" H 7650 1500 50  0001 C CNN
F 1 "GND" H 7650 1600 50  0000 C CNN
F 2 "" H 7650 1750 50  0001 C CNN
F 3 "" H 7650 1750 50  0001 C CNN
	1    7650 1750
	1    0    0    -1  
$EndComp
Text Label 7650 1250 0    60   ~ 0
+V_MOTOR
Wire Wire Line
	6250 4200 6200 4200
Wire Wire Line
	6200 4200 6200 3850
Text Label 6200 3850 0    60   ~ 0
+V_MOTOR
Wire Wire Line
	7450 4350 7550 4350
Wire Wire Line
	7550 4350 7550 3450
Wire Wire Line
	7550 3450 5150 3450
Wire Wire Line
	7450 4800 7600 4800
Wire Wire Line
	7600 4800 7600 3550
Wire Wire Line
	7600 3550 5300 3550
Wire Wire Line
	7450 4500 7650 4500
Wire Wire Line
	7650 4500 7650 3650
Wire Wire Line
	7650 3650 5150 3650
Wire Wire Line
	5200 3550 5200 3750
Wire Wire Line
	5200 3750 5150 3750
Connection ~ 5200 3550
Wire Wire Line
	7450 4650 7700 4650
Wire Wire Line
	7700 4650 7700 3700
Wire Wire Line
	7700 3700 5250 3700
Wire Wire Line
	5250 3700 5250 3850
Wire Wire Line
	5250 3850 5150 3850
Wire Wire Line
	5300 3550 5300 3950
Wire Wire Line
	5300 3950 5150 3950
Connection ~ 5300 3550
Wire Wire Line
	6250 4350 6150 4350
Wire Wire Line
	6150 4350 6150 4050
Wire Wire Line
	6150 4050 5150 4050
Wire Wire Line
	6250 4500 6100 4500
Wire Wire Line
	6100 4500 6100 4150
Wire Wire Line
	6100 4150 5150 4150
Wire Wire Line
	6250 4650 6050 4650
Wire Wire Line
	6050 4650 6050 4250
Wire Wire Line
	6050 4250 5150 4250
Wire Wire Line
	6250 4800 6000 4800
Wire Wire Line
	6000 4800 6000 4350
Wire Wire Line
	6000 4350 5150 4350
Wire Wire Line
	6250 4950 5950 4950
Wire Wire Line
	5950 4950 5950 4450
Wire Wire Line
	5950 4450 5150 4450
Wire Wire Line
	6250 5100 5900 5100
Wire Wire Line
	5900 5100 5900 4550
Wire Wire Line
	5900 4550 5150 4550
Wire Wire Line
	7450 5200 7950 5200
Wire Wire Line
	7950 5200 7950 5600
Wire Wire Line
	7950 5600 5850 5600
Wire Wire Line
	5850 5600 5850 4850
Wire Wire Line
	5850 4850 5150 4850
Wire Wire Line
	7450 5050 8000 5050
Wire Wire Line
	8000 5050 8000 5650
Wire Wire Line
	8000 5650 5800 5650
Wire Wire Line
	5800 5650 5800 4950
Wire Wire Line
	5800 4950 5150 4950
Wire Wire Line
	5150 5050 5250 5050
Text Label 5250 5050 0    60   ~ 0
STATUS
Wire Wire Line
	7450 4200 8050 4200
Wire Wire Line
	8050 4200 8050 5700
Wire Wire Line
	8050 5700 5750 5700
Wire Wire Line
	5750 5700 5750 5650
Wire Wire Line
	5750 5650 5150 5650
$Comp
L Device:R R29
U 1 1 5B84F265
P 5700 6250
F 0 "R29" V 5780 6250 50  0000 C CNN
F 1 "20k" V 5700 6250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5630 6250 50  0001 C CNN
F 3 "" H 5700 6250 50  0001 C CNN
	1    5700 6250
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R30
U 1 1 5B84F431
P 6100 6250
F 0 "R30" V 6180 6250 50  0000 C CNN
F 1 "4.02k" V 6100 6250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6030 6250 50  0001 C CNN
F 3 "" H 6100 6250 50  0001 C CNN
	1    6100 6250
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5B84F7B7
P 6300 6250
F 0 "#PWR09" H 6300 6000 50  0001 C CNN
F 1 "GND" H 6300 6100 50  0000 C CNN
F 2 "" H 6300 6250 50  0001 C CNN
F 3 "" H 6300 6250 50  0001 C CNN
	1    6300 6250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6250 6250 6300 6250
Wire Wire Line
	5850 6250 5900 6250
Wire Wire Line
	5900 5750 5900 6250
Connection ~ 5900 6250
Text Label 5500 6250 2    60   ~ 0
+V_MOTOR
Wire Wire Line
	5500 6250 5550 6250
Wire Wire Line
	5150 5750 5900 5750
Wire Wire Line
	9050 3850 9050 3950
Wire Wire Line
	9050 3950 9050 4050
Wire Wire Line
	1400 4350 2450 4350
Wire Wire Line
	1400 4850 1850 4850
Wire Wire Line
	1850 3850 2650 3850
Wire Wire Line
	700  3850 700  4350
Wire Wire Line
	700  4350 700  4450
Wire Wire Line
	2450 4350 2650 4350
Wire Wire Line
	1850 4850 2150 4850
Wire Wire Line
	2150 4850 2150 4950
Wire Wire Line
	2150 4850 2450 4850
Wire Wire Line
	2550 5950 2550 6000
Wire Wire Line
	5200 3550 5150 3550
Wire Wire Line
	5300 3550 5200 3550
Wire Wire Line
	5900 6250 5950 6250
$EndSCHEMATC

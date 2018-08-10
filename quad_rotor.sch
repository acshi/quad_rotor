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
LIBS:quad_rotor-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L GND #PWR5
U 1 1 5A458BA5
P 3900 3000
F 0 "#PWR5" H 3900 2750 50  0001 C CNN
F 1 "GND" H 3900 2850 50  0000 C CNN
F 2 "" H 3900 3000 50  0001 C CNN
F 3 "" H 3900 3000 50  0001 C CNN
	1    3900 3000
	0    -1   -1   0   
$EndComp
Text Label 3900 2800 0    60   ~ 0
UPDI
Text Label 3900 2900 0    60   ~ 0
+5V
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
	4350 1350 4400 1350
Wire Wire Line
	4400 1350 4450 1350
Wire Wire Line
	4750 1350 4800 1350
Wire Wire Line
	4800 1350 4900 1350
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
	2150 1850 2200 1850
Wire Wire Line
	2200 1850 2250 1850
Wire Wire Line
	2550 1850 2600 1850
Wire Wire Line
	2600 1850 2650 1850
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
	2150 1200 2200 1200
Wire Wire Line
	2200 1200 2250 1200
Wire Wire Line
	2550 1200 2600 1200
Wire Wire Line
	2600 1200 2650 1200
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
	2200 1050 2200 1200
Wire Wire Line
	2200 1200 2200 1350
Connection ~ 2200 1200
Wire Wire Line
	2550 1350 2600 1350
Wire Wire Line
	2600 1350 2600 1200
Connection ~ 2600 1200
Wire Wire Line
	2200 2000 2250 2000
Wire Wire Line
	2200 1700 2200 1850
Wire Wire Line
	2200 1850 2200 2000
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
$Comp
L CONN_01X03 J1
U 1 1 5A484610
P 3600 2900
F 0 "J1" H 3600 3100 50  0000 C CNN
F 1 "UPDI PROGRAM" V 3700 2900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3600 2900 50  0001 C CNN
F 3 "" H 3600 2900 50  0001 C CNN
	1    3600 2900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3800 2800 3900 2800
Wire Wire Line
	3800 2900 3900 2900
Wire Wire Line
	3800 3000 3900 3000
Text Label 4900 1350 0    60   ~ 0
MOT_B
$Sheet
S 7150 600  1200 1250
U 5AE1F86D
F0 "bldc_unit1" 60
F1 "bldc_unit.sch" 60
F2 "+V_MOTOR" I L 7150 750 60 
F3 "IN_A" I L 7150 950 60 
F4 "IN_B" I L 7150 1100 60 
F5 "IN_C" I L 7150 1250 60 
F6 "CURRENT" O R 8350 750 60 
F7 "ENABLE_A" I L 7150 1400 60 
F8 "MOTOR_A" O R 8350 900 60 
F9 "MOTOR_B" O R 8350 1050 60 
F10 "MOTOR_C" O R 8350 1200 60 
F11 "ENABLE_B" I L 7150 1550 60 
F12 "ENABLE_C" I L 7150 1700 60 
$EndSheet
$Sheet
S 7150 2100 1200 1250
U 5AE22A2E
F0 "bldc_unit2" 60
F1 "bldc_unit.sch" 60
F2 "+V_MOTOR" I L 7150 2250 60 
F3 "IN_A" I L 7150 2450 60 
F4 "IN_B" I L 7150 2600 60 
F5 "IN_C" I L 7150 2750 60 
F6 "CURRENT" O R 8350 2250 60 
F7 "ENABLE_A" I L 7150 2900 60 
F8 "MOTOR_A" O R 8350 2400 60 
F9 "MOTOR_B" O R 8350 2550 60 
F10 "MOTOR_C" O R 8350 2700 60 
F11 "ENABLE_B" I L 7150 3050 60 
F12 "ENABLE_C" I L 7150 3200 60 
$EndSheet
$Sheet
S 7150 3600 1200 1250
U 5AE22F67
F0 "bldc_unit3" 60
F1 "bldc_unit.sch" 60
F2 "+V_MOTOR" I L 7150 3750 60 
F3 "IN_A" I L 7150 3950 60 
F4 "IN_B" I L 7150 4100 60 
F5 "IN_C" I L 7150 4250 60 
F6 "CURRENT" O R 8350 3750 60 
F7 "ENABLE_A" I L 7150 4400 60 
F8 "MOTOR_A" O R 8350 3900 60 
F9 "MOTOR_B" O R 8350 4050 60 
F10 "MOTOR_C" O R 8350 4200 60 
F11 "ENABLE_B" I L 7150 4550 60 
F12 "ENABLE_C" I L 7150 4700 60 
$EndSheet
$Sheet
S 7150 5100 1200 1250
U 5AE23461
F0 "bldc_unit4" 60
F1 "bldc_unit.sch" 60
F2 "+V_MOTOR" I L 7150 5250 60 
F3 "IN_A" I L 7150 5450 60 
F4 "IN_B" I L 7150 5600 60 
F5 "IN_C" I L 7150 5750 60 
F6 "CURRENT" O R 8350 5250 60 
F7 "ENABLE_A" I L 7150 5900 60 
F8 "MOTOR_A" O R 8350 5400 60 
F9 "MOTOR_B" O R 8350 5550 60 
F10 "MOTOR_C" O R 8350 5700 60 
F11 "ENABLE_B" I L 7150 6050 60 
F12 "ENABLE_C" I L 7150 6200 60 
$EndSheet
$Comp
L SAMD21E16L-MU U?
U 1 1 5AE30308
P 2400 4850
F 0 "U?" H 1350 6250 50  0000 C CNN
F 1 "SAMD21E16L-MU" H 3250 3450 50  0000 C CNN
F 2 "TQFP32" H 2400 3850 50  0001 C CIN
F 3 "" H 2400 4850 50  0001 C CNN
	1    2400 4850
	1    0    0    -1  
$EndComp
$EndSCHEMATC
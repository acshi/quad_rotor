EESchema Schematic File Version 4
LIBS:bldc_driver-cache
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
+5V
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
F 1 "1k" V 4200 750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4130 750 50  0001 C CNN
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
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4130 1050 50  0001 C CNN
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
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4600 750 50  0001 C CNN
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
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4600 1050 50  0001 C CNN
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
+5V
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
F7 "IN_HA" I L 6250 4350 60 
F8 "IN_HB" I L 6250 4650 60 
F9 "IN_HC" I L 6250 4950 60 
F10 "IN_LA" I L 6250 4500 60 
F11 "IN_LB" I L 6250 4800 60 
F12 "ENABLE" I R 7450 5200 60 
F13 "IN_LC" I L 6250 5100 60 
F14 "~FAULT" O R 7450 5050 60 
F15 "MOTOR_ZERO" O R 7450 4800 60 
F16 "+5V" I R 7450 5350 50 
$EndSheet
Text Label 3800 5350 2    60   ~ 0
SDA
Text Label 3800 5450 2    60   ~ 0
SCL
$Comp
L power:GND #PWR04
U 1 1 5AE35169
P 4500 6350
F 0 "#PWR04" H 4500 6100 50  0001 C CNN
F 1 "GND" H 4500 6200 50  0000 C CNN
F 2 "" H 4500 6350 50  0001 C CNN
F 3 "" H 4500 6350 50  0001 C CNN
	1    4500 6350
	1    0    0    -1  
$EndComp
Text Label 4300 3250 0    60   ~ 0
+5V
Text Label 3950 1350 2    60   ~ 0
STATUS
$Comp
L Device:R R3
U 1 1 5A45B0E3
P 4200 1350
F 0 "R3" V 4280 1350 50  0000 C CNN
F 1 "3k" V 4200 1350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4130 1350 50  0001 C CNN
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
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4600 1350 50  0001 C CNN
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
	3900 5450 3800 5450
Wire Wire Line
	3900 5350 3800 5350
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
Text Label 8050 5350 0    60   ~ 0
+5V
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
Text Label 5200 5850 0    60   ~ 0
STATUS
$Comp
L Device:R R29
U 1 1 5B84F265
P 5700 6250
F 0 "R29" V 5780 6250 50  0000 C CNN
F 1 "20k" V 5700 6250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5630 6250 50  0001 C CNN
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
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6030 6250 50  0001 C CNN
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
Text Label 5500 6250 2    60   ~ 0
+V_MOTOR
Wire Wire Line
	5500 6250 5550 6250
$Comp
L Device:R R33
U 1 1 5C5B1EBF
P 3500 4000
F 0 "R33" V 3400 3950 50  0000 L CNN
F 1 "10k" V 3500 3950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3430 4000 50  0001 C CNN
F 3 "~" H 3500 4000 50  0001 C CNN
	1    3500 4000
	0    -1   -1   0   
$EndComp
$Comp
L MCU_Microchip_ATmega:ATmega328PB-AU U1
U 1 1 5D9473FC
P 4500 4850
F 0 "U1" H 4050 3350 50  0000 C CNN
F 1 "ATmega328PB-AU" H 4900 3350 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 4500 4850 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/40001906C.pdf" H 4500 4850 50  0001 C CNN
	1    4500 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 4000 3750 4000
Text Label 3750 4000 0    50   ~ 0
+5V
$Comp
L Device:L_Small L1
U 1 1 5D96D637
P 4450 2950
F 0 "L1" H 4498 2996 50  0000 L CNN
F 1 "10uH" H 4498 2905 50  0000 L CNN
F 2 "Inductor_SMD:L_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4450 2950 50  0001 C CNN
F 3 "~" H 4450 2950 50  0001 C CNN
	1    4450 2950
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5D970E21
P 4750 2950
F 0 "C1" H 4842 2996 50  0000 L CNN
F 1 "100nF" H 4842 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4750 2950 50  0001 C CNN
F 3 "~" H 4750 2950 50  0001 C CNN
	1    4750 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4550 2950 4600 2950
$Comp
L power:GND #PWR0102
U 1 1 5D97458F
P 4950 2950
F 0 "#PWR0102" H 4950 2700 50  0001 C CNN
F 1 "GND" H 4955 2777 50  0000 C CNN
F 2 "" H 4950 2950 50  0001 C CNN
F 3 "" H 4950 2950 50  0001 C CNN
	1    4950 2950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4950 2950 4850 2950
Text Label 4250 2950 1    60   ~ 0
+5V
Wire Wire Line
	4250 2950 4350 2950
Connection ~ 4600 2950
Wire Wire Line
	4600 2950 4650 2950
Wire Wire Line
	4250 3250 4500 3250
Wire Wire Line
	4500 3250 4500 3350
Wire Wire Line
	4600 2950 4600 3350
Wire Wire Line
	5100 5150 5200 5150
Text Label 5200 5150 0    50   ~ 0
~RESET
Text Label 3250 4000 2    50   ~ 0
~RESET
Wire Wire Line
	3250 4000 3350 4000
Wire Wire Line
	6250 4350 6150 4350
Wire Wire Line
	6150 4350 6150 3850
Wire Wire Line
	6250 4650 6100 4650
Wire Wire Line
	6250 4500 6050 4500
Wire Wire Line
	6050 4500 6050 4050
Wire Wire Line
	6250 4800 6000 4800
Wire Wire Line
	7450 4650 7500 4650
Wire Wire Line
	7500 4650 7500 5550
Wire Wire Line
	7500 5550 5900 5550
Wire Wire Line
	5900 5550 5900 4750
Wire Wire Line
	7450 4500 7550 4500
Wire Wire Line
	7550 4500 7550 5600
Wire Wire Line
	7550 5600 5850 5600
Wire Wire Line
	5850 5600 5850 4650
Wire Wire Line
	5850 4650 5100 4650
Wire Wire Line
	7450 4200 7650 4200
Wire Wire Line
	7650 5700 5750 5700
Wire Wire Line
	5750 5700 5750 4850
Wire Wire Line
	5750 4850 5100 4850
Wire Wire Line
	7450 5350 8050 5350
Wire Wire Line
	7650 4200 7650 5700
Wire Wire Line
	7450 4800 7700 4800
Wire Wire Line
	7700 4800 7700 5950
Wire Wire Line
	7700 5950 5100 5950
Wire Wire Line
	5850 6250 5900 6250
Wire Wire Line
	5900 6250 5900 5750
Wire Wire Line
	5900 5750 5700 5750
Wire Wire Line
	5700 5750 5700 4950
Wire Wire Line
	5700 4950 5100 4950
Connection ~ 5900 6250
Wire Wire Line
	5900 6250 5950 6250
Wire Wire Line
	7450 5200 7750 5200
Wire Wire Line
	7750 5200 7750 5850
Wire Wire Line
	7450 5050 7800 5050
Wire Wire Line
	7800 5050 7800 6050
$Comp
L Connector:AVR-ISP-6 J2
U 1 1 5D959B16
P 9200 4300
F 0 "J2" H 8920 4304 50  0000 R CNN
F 1 "AVR-ISP-6" H 8920 4395 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" V 8950 4350 50  0001 C CNN
F 3 " ~" H 7925 3750 50  0001 C CNN
	1    9200 4300
	-1   0    0    1   
$EndComp
Text Label 8600 4200 2    50   ~ 0
~RESET
Wire Wire Line
	8600 4200 8800 4200
$Comp
L power:GND #PWR0103
U 1 1 5D95BEF0
P 9300 3800
F 0 "#PWR0103" H 9300 3550 50  0001 C CNN
F 1 "GND" H 9300 3650 50  0000 C CNN
F 2 "" H 9300 3800 50  0001 C CNN
F 3 "" H 9300 3800 50  0001 C CNN
	1    9300 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	9300 3900 9300 3800
Text Label 9300 4900 2    60   ~ 0
+5V
Wire Wire Line
	9300 4800 9300 4900
Wire Wire Line
	6150 4950 6150 5350
Wire Wire Line
	6150 4950 6250 4950
Wire Wire Line
	6150 5350 5100 5350
Wire Wire Line
	6250 5100 6200 5100
Wire Wire Line
	6200 5100 6200 5450
Wire Wire Line
	6200 5450 5100 5450
Wire Wire Line
	5100 4150 5200 4150
Wire Wire Line
	5100 4250 5200 4250
Wire Wire Line
	5100 4350 5200 4350
Text Label 5200 4150 0    50   ~ 0
MOSI
Text Label 5200 4250 0    50   ~ 0
MISO
Text Label 5200 4350 0    50   ~ 0
SCK
Text Label 8600 4300 2    50   ~ 0
SCK
Text Label 8600 4500 2    50   ~ 0
MISO
Text Label 8600 4400 2    50   ~ 0
MOSI
Wire Wire Line
	8600 4300 8800 4300
Wire Wire Line
	8600 4400 8800 4400
Wire Wire Line
	8600 4500 8800 4500
$Comp
L Device:C_Small C2
U 1 1 5D979EC8
P 4150 3250
F 0 "C2" V 4050 3050 50  0000 L CNN
F 1 "100nF" V 4050 3200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4150 3250 50  0001 C CNN
F 3 "~" H 4150 3250 50  0001 C CNN
	1    4150 3250
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5D979F9C
P 4000 3250
F 0 "#PWR0104" H 4000 3000 50  0001 C CNN
F 1 "GND" H 4005 3077 50  0000 C CNN
F 2 "" H 4000 3250 50  0001 C CNN
F 3 "" H 4000 3250 50  0001 C CNN
	1    4000 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 3250 4050 3250
Wire Wire Line
	5100 4750 5900 4750
Wire Wire Line
	6100 3750 5100 3750
Wire Wire Line
	6100 3750 6100 4650
Wire Wire Line
	5100 3850 6150 3850
Wire Wire Line
	6000 3950 5100 3950
Wire Wire Line
	6000 3950 6000 4800
Wire Wire Line
	5100 4050 6050 4050
Wire Wire Line
	7750 5850 5650 5850
Wire Wire Line
	5650 5850 5650 5650
Wire Wire Line
	5650 5650 5100 5650
Wire Wire Line
	7800 6050 5600 6050
Wire Wire Line
	5600 6050 5600 5750
Wire Wire Line
	5600 5750 5100 5750
Wire Wire Line
	5100 5850 5200 5850
Text Label 5200 3750 0    50   ~ 0
HB
Text Label 5200 3850 0    50   ~ 0
HA
Text Label 5200 3950 0    50   ~ 0
LB
Text Label 5200 4050 0    50   ~ 0
LA
Text Label 5200 4750 0    50   ~ 0
MOTOR_C
Text Label 5200 4650 0    50   ~ 0
MOTOR_B
Text Label 3800 5550 2    50   ~ 0
MOTOR_A
Text Label 5200 4850 0    50   ~ 0
CURRENT
Text Label 5200 4950 0    50   ~ 0
V_MOT
Text Label 5200 5350 0    50   ~ 0
HC
Text Label 5200 5450 0    50   ~ 0
LC
Text Label 5200 5650 0    50   ~ 0
ENABLE
Text Label 5200 5750 0    50   ~ 0
~FAULT
Text Label 5200 6050 0    50   ~ 0
MOTOR_A
Wire Wire Line
	5100 6050 5200 6050
Wire Wire Line
	7450 4350 7750 4350
Text Label 7750 4350 0    50   ~ 0
MOTOR_A
Wire Wire Line
	3800 5550 3900 5550
Text Notes 2300 5550 0    50   ~ 0
just to use the pin as a wire
Wire Wire Line
	5100 3650 5200 3650
Text Label 5200 3650 0    50   ~ 0
TRIGGER
Wire Wire Line
	3900 5650 3800 5650
Text Label 3800 5650 2    50   ~ 0
TRIGGER2
$EndSCHEMATC

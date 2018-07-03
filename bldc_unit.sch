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
Sheet 5 5
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
L GND #PWR17
U 1 1 5AE1FDE2
P 4250 5550
AR Path="/5AE1F86D/5AE1FDE2" Ref="#PWR17"  Part="1" 
AR Path="/5AE23461/5AE1FDE2" Ref="#PWR17"  Part="1" 
F 0 "#PWR17" H 4250 5300 50  0001 C CNN
F 1 "GND" H 4250 5400 50  0000 C CNN
F 2 "" H 4250 5550 50  0001 C CNN
F 3 "" H 4250 5550 50  0001 C CNN
	1    4250 5550
	1    0    0    -1  
$EndComp
Text Label 2850 4550 2    60   ~ 0
GHA
Text Label 2850 5250 2    60   ~ 0
GLA
Text Label 4700 4650 0    60   ~ 0
GHB
Text Label 4700 5150 0    60   ~ 0
GLB
$Comp
L Q_NMOS_DGS Q7
U 1 1 5AE1FDEC
P 3200 4550
AR Path="/5AE1F86D/5AE1FDEC" Ref="Q7"  Part="1" 
AR Path="/5AE23461/5AE1FDEC" Ref="Q7"  Part="1" 
F 0 "Q7" H 3400 4600 50  0000 L CNN
F 1 "PSMN0R925YLC" H 3400 4500 50  0000 L CNN
F 2 "LFPAK:LFPAK_POWER_SO-8" H 3400 4650 50  0001 C CNN
F 3 "" H 3200 4550 50  0001 C CNN
	1    3200 4550
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_DGS Q8
U 1 1 5AE1FDF3
P 3200 5250
AR Path="/5AE1F86D/5AE1FDF3" Ref="Q8"  Part="1" 
AR Path="/5AE23461/5AE1FDF3" Ref="Q8"  Part="1" 
F 0 "Q8" H 3400 5300 50  0000 L CNN
F 1 "PSMN0R925YLC" H 3400 5200 50  0000 L CNN
F 2 "LFPAK:LFPAK_POWER_SO-8" H 3400 5350 50  0001 C CNN
F 3 "" H 3200 5250 50  0001 C CNN
	1    3200 5250
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_DGS Q9
U 1 1 5AE1FDFA
P 4350 4650
AR Path="/5AE1F86D/5AE1FDFA" Ref="Q9"  Part="1" 
AR Path="/5AE23461/5AE1FDFA" Ref="Q9"  Part="1" 
F 0 "Q9" H 4550 4700 50  0000 L CNN
F 1 "PSMN0R925YLC" H 4550 4600 50  0000 L CNN
F 2 "LFPAK:LFPAK_POWER_SO-8" H 4550 4750 50  0001 C CNN
F 3 "" H 4350 4650 50  0001 C CNN
	1    4350 4650
	-1   0    0    -1  
$EndComp
$Comp
L Q_NMOS_DGS Q10
U 1 1 5AE1FE01
P 4350 5150
AR Path="/5AE1F86D/5AE1FE01" Ref="Q10"  Part="1" 
AR Path="/5AE23461/5AE1FE01" Ref="Q10"  Part="1" 
F 0 "Q10" H 4550 5200 50  0000 L CNN
F 1 "PSMN0R925YLC" H 4550 5100 50  0000 L CNN
F 2 "LFPAK:LFPAK_POWER_SO-8" H 4550 5250 50  0001 C CNN
F 3 "" H 4350 5150 50  0001 C CNN
	1    4350 5150
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR21
U 1 1 5AE1FE08
P 7850 3100
AR Path="/5AE1F86D/5AE1FE08" Ref="#PWR21"  Part="1" 
AR Path="/5AE23461/5AE1FE08" Ref="#PWR21"  Part="1" 
F 0 "#PWR21" H 7850 2850 50  0001 C CNN
F 1 "GND" H 7850 2950 50  0000 C CNN
F 2 "" H 7850 3100 50  0001 C CNN
F 3 "" H 7850 3100 50  0001 C CNN
	1    7850 3100
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR22
U 1 1 5AE1FE0E
P 7850 4250
AR Path="/5AE1F86D/5AE1FE0E" Ref="#PWR22"  Part="1" 
AR Path="/5AE23461/5AE1FE0E" Ref="#PWR22"  Part="1" 
F 0 "#PWR22" H 7850 4000 50  0001 C CNN
F 1 "GND" H 7850 4100 50  0000 C CNN
F 2 "" H 7850 4250 50  0001 C CNN
F 3 "" H 7850 4250 50  0001 C CNN
	1    7850 4250
	0    -1   -1   0   
$EndComp
Text Label 7900 2800 0    60   ~ 0
GHA
Text Label 7900 3000 0    60   ~ 0
GLA
Text Label 7900 3950 0    60   ~ 0
GHB
Text Label 7900 4150 0    60   ~ 0
GLB
Text Label 3300 4950 0    60   ~ 0
MOT_A
Text Label 4250 4950 0    60   ~ 0
MOT_B
Text Label 8300 2900 0    60   ~ 0
MOT_A
Text Label 8300 4050 0    60   ~ 0
MOT_B
$Comp
L C C12
U 1 1 5AE1FE1D
P 8200 2650
AR Path="/5AE1F86D/5AE1FE1D" Ref="C12"  Part="1" 
AR Path="/5AE23461/5AE1FE1D" Ref="C12"  Part="1" 
F 0 "C12" H 8225 2750 50  0000 L CNN
F 1 "360nF" H 8225 2550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 8238 2500 50  0001 C CNN
F 3 "" H 8200 2650 50  0001 C CNN
F 4 "50V" H 8400 2650 60  0000 C CNN "Voltage"
	1    8200 2650
	1    0    0    -1  
$EndComp
Text Label 6800 2900 2    60   ~ 0
ENABLE_A
Text Label 6800 4050 2    60   ~ 0
ENABLE_B
Text Label 6800 3100 2    60   ~ 0
IN_A
Text Label 6800 4250 2    60   ~ 0
IN_B
$Comp
L C C13
U 1 1 5AE1FE29
P 8200 3800
AR Path="/5AE1F86D/5AE1FE29" Ref="C13"  Part="1" 
AR Path="/5AE23461/5AE1FE29" Ref="C13"  Part="1" 
F 0 "C13" H 8225 3900 50  0000 L CNN
F 1 "360nF" H 8225 3700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 8238 3650 50  0001 C CNN
F 3 "" H 8200 3800 50  0001 C CNN
F 4 "50V" H 8400 3800 60  0000 C CNN "Voltage"
	1    8200 3800
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 5AE1FE30
P 6250 4000
AR Path="/5AE1F86D/5AE1FE30" Ref="C10"  Part="1" 
AR Path="/5AE23461/5AE1FE30" Ref="C10"  Part="1" 
F 0 "C10" H 6275 4100 50  0000 L CNN
F 1 "100n" H 6275 3900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6288 3850 50  0001 C CNN
F 3 "" H 6250 4000 50  0001 C CNN
	1    6250 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR19
U 1 1 5AE1FE37
P 6250 4250
AR Path="/5AE1F86D/5AE1FE37" Ref="#PWR19"  Part="1" 
AR Path="/5AE23461/5AE1FE37" Ref="#PWR19"  Part="1" 
F 0 "#PWR19" H 6250 4000 50  0001 C CNN
F 1 "GND" H 6250 4100 50  0000 C CNN
F 2 "" H 6250 4250 50  0001 C CNN
F 3 "" H 6250 4250 50  0001 C CNN
	1    6250 4250
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 5AE1FE3D
P 6250 2850
AR Path="/5AE1F86D/5AE1FE3D" Ref="C9"  Part="1" 
AR Path="/5AE23461/5AE1FE3D" Ref="C9"  Part="1" 
F 0 "C9" H 6275 2950 50  0000 L CNN
F 1 "100n" H 6275 2750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6288 2700 50  0001 C CNN
F 3 "" H 6250 2850 50  0001 C CNN
	1    6250 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR18
U 1 1 5AE1FE44
P 6250 3100
AR Path="/5AE1F86D/5AE1FE44" Ref="#PWR18"  Part="1" 
AR Path="/5AE23461/5AE1FE44" Ref="#PWR18"  Part="1" 
F 0 "#PWR18" H 6250 2850 50  0001 C CNN
F 1 "GND" H 6250 2950 50  0000 C CNN
F 2 "" H 6250 3100 50  0001 C CNN
F 3 "" H 6250 3100 50  0001 C CNN
	1    6250 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 4650 4700 4650
Wire Wire Line
	3300 4750 3300 5050
Wire Wire Line
	4250 4850 4250 4950
Wire Wire Line
	3300 4350 3300 4300
Wire Wire Line
	3300 4300 5200 4300
Wire Wire Line
	4250 4000 4250 4450
Wire Wire Line
	4550 5150 4700 5150
Wire Wire Line
	3300 5450 3300 5500
Wire Wire Line
	4250 5350 4250 5550
Wire Wire Line
	2850 4550 3000 4550
Wire Wire Line
	2850 5250 3000 5250
Wire Wire Line
	7800 3100 7850 3100
Wire Wire Line
	7800 4250 7850 4250
Wire Wire Line
	7800 2800 7900 2800
Wire Wire Line
	7800 3000 7900 3000
Wire Wire Line
	7800 3950 7900 3950
Wire Wire Line
	7800 4150 7900 4150
Wire Wire Line
	7800 2900 8300 2900
Wire Wire Line
	7800 4050 8300 4050
Wire Wire Line
	8200 2800 8200 2900
Connection ~ 8200 2900
Wire Wire Line
	8200 2450 8200 2500
Wire Wire Line
	7850 2450 7850 2700
Wire Wire Line
	7850 2700 7800 2700
Wire Wire Line
	6850 2450 6850 2700
Wire Wire Line
	6800 2700 6900 2700
Wire Wire Line
	6900 2900 6800 2900
Wire Wire Line
	6800 4050 6900 4050
Wire Wire Line
	6900 3100 6800 3100
Wire Wire Line
	6900 4250 6800 4250
Connection ~ 6850 2700
Wire Wire Line
	6800 3850 6900 3850
Wire Wire Line
	8200 3950 8200 4050
Connection ~ 8200 4050
Wire Wire Line
	8200 3600 8200 3650
Wire Wire Line
	7850 3600 8200 3600
Wire Wire Line
	7850 3600 7850 3850
Wire Wire Line
	7850 3850 7800 3850
Wire Wire Line
	6850 3600 6850 3850
Connection ~ 6850 3850
Wire Wire Line
	6250 4250 6250 4150
Wire Wire Line
	6250 3850 6250 3600
Wire Wire Line
	6250 3100 6250 3000
Wire Wire Line
	6250 2700 6250 2450
Text Label 3200 4000 2    60   ~ 0
+V_MOTOR
Text Label 6800 2700 2    60   ~ 0
+V_MOTOR
Text Label 6800 3850 2    60   ~ 0
+V_MOTOR
Wire Wire Line
	6250 3600 6850 3600
Wire Wire Line
	6250 2450 6850 2450
Wire Wire Line
	7850 2450 8200 2450
$Comp
L NCP5901 U6
U 1 1 5AE1FE7C
P 7350 2900
AR Path="/5AE1F86D/5AE1FE7C" Ref="U6"  Part="1" 
AR Path="/5AE23461/5AE1FE7C" Ref="U6"  Part="1" 
F 0 "U6" H 7550 2500 60  0000 C CNN
F 1 "NCP5901" H 7300 3250 60  0000 C CNN
F 2 "SMD_Packages:SOIC-8-N" H 7350 2950 60  0001 C CNN
F 3 "" H 7350 2950 60  0001 C CNN
	1    7350 2900
	1    0    0    -1  
$EndComp
$Comp
L NCP5901 U7
U 1 1 5AE1FE83
P 7350 4050
AR Path="/5AE1F86D/5AE1FE83" Ref="U7"  Part="1" 
AR Path="/5AE23461/5AE1FE83" Ref="U7"  Part="1" 
F 0 "U7" H 7550 3650 60  0000 C CNN
F 1 "NCP5901" H 7300 4400 60  0000 C CNN
F 2 "SMD_Packages:SOIC-8-N" H 7350 4100 60  0001 C CNN
F 3 "" H 7350 4100 60  0001 C CNN
	1    7350 4050
	1    0    0    -1  
$EndComp
Text Notes 2700 2600 0    60   ~ 0
MAX4080T, 20x V/V,  0.1% accuracy, 100uV offset voltage.\nmin 4.5V supply. min output voltage 15mV sinking 10uA. (sink less for less?)\nw/ 5.0V ref and 4mR\n4mR = 0.08V/A = 62.5A range = 61.04mA/bit = 244.14uV/bit\nw/ 1.1V ref and 4mR\n4mR = 0.08V/A = 13.75A range = 13.43mA/bit = 53.71uV/bit\nw/ 2.048V ref and 4mR\n4mR = 0.08V/A = 25.6A range = 25mA/bit = 100uV/bit
$Comp
L MAX4080T U5
U 1 1 5AE1FE8B
P 4200 3300
AR Path="/5AE1F86D/5AE1FE8B" Ref="U5"  Part="1" 
AR Path="/5AE23461/5AE1FE8B" Ref="U5"  Part="1" 
F 0 "U5" H 3900 3650 50  0000 L CNN
F 1 "MAX4080T" H 4300 3650 50  0000 L CNN
F 2 "" H 4850 2600 50  0001 C CNN
F 3 "" H 4200 3700 50  0001 C CNN
	1    4200 3300
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 5AE1FE92
P 3900 4000
AR Path="/5AE1F86D/5AE1FE92" Ref="R11"  Part="1" 
AR Path="/5AE23461/5AE1FE92" Ref="R11"  Part="1" 
F 0 "R11" V 3980 4000 50  0000 C CNN
F 1 "2mR" V 3900 4000 50  0000 C CNN
F 2 "" V 3830 4000 50  0001 C CNN
F 3 "" H 3900 4000 50  0001 C CNN
	1    3900 4000
	0    1    1    0   
$EndComp
$Comp
L R R10
U 1 1 5AE1FE99
P 3500 4000
AR Path="/5AE1F86D/5AE1FE99" Ref="R10"  Part="1" 
AR Path="/5AE23461/5AE1FE99" Ref="R10"  Part="1" 
F 0 "R10" V 3580 4000 50  0000 C CNN
F 1 "2mR" V 3500 4000 50  0000 C CNN
F 2 "" V 3430 4000 50  0001 C CNN
F 3 "" H 3500 4000 50  0001 C CNN
	1    3500 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	3200 4000 3350 4000
Wire Wire Line
	3650 4000 3750 4000
Text Label 4250 4000 0    60   ~ 0
MOTOR_SHUNT
Wire Wire Line
	3800 3200 3750 3200
Text Label 3750 3200 2    60   ~ 0
+V_MOTOR
Wire Wire Line
	3800 3400 3750 3400
Text Label 3750 3400 2    60   ~ 0
MOTOR_SHUNT
Wire Wire Line
	4200 2900 4200 2850
Text Label 4200 2850 0    60   ~ 0
+V_MOTOR
Wire Wire Line
	4700 3100 4750 3100
Text Label 4750 3100 0    60   ~ 0
CURRENT
$Comp
L GND #PWR16
U 1 1 5AE1FEAB
P 4200 3650
AR Path="/5AE1F86D/5AE1FEAB" Ref="#PWR16"  Part="1" 
AR Path="/5AE23461/5AE1FEAB" Ref="#PWR16"  Part="1" 
F 0 "#PWR16" H 4200 3400 50  0001 C CNN
F 1 "GND" H 4200 3500 50  0000 C CNN
F 2 "" H 4200 3650 50  0001 C CNN
F 3 "" H 4200 3650 50  0001 C CNN
	1    4200 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3650 4200 3600
Text Label 5650 4500 0    60   ~ 0
GHC
Text Label 5650 5250 0    60   ~ 0
GLC
$Comp
L Q_NMOS_DGS Q11
U 1 1 5AE1FEB4
P 5300 4500
AR Path="/5AE1F86D/5AE1FEB4" Ref="Q11"  Part="1" 
AR Path="/5AE23461/5AE1FEB4" Ref="Q11"  Part="1" 
F 0 "Q11" H 5500 4550 50  0000 L CNN
F 1 "PSMN0R925YLC" H 5500 4450 50  0000 L CNN
F 2 "LFPAK:LFPAK_POWER_SO-8" H 5500 4600 50  0001 C CNN
F 3 "" H 5300 4500 50  0001 C CNN
	1    5300 4500
	-1   0    0    -1  
$EndComp
$Comp
L Q_NMOS_DGS Q12
U 1 1 5AE1FEBB
P 5300 5250
AR Path="/5AE1F86D/5AE1FEBB" Ref="Q12"  Part="1" 
AR Path="/5AE23461/5AE1FEBB" Ref="Q12"  Part="1" 
F 0 "Q12" H 5500 5300 50  0000 L CNN
F 1 "PSMN0R925YLC" H 5500 5200 50  0000 L CNN
F 2 "LFPAK:LFPAK_POWER_SO-8" H 5500 5350 50  0001 C CNN
F 3 "" H 5300 5250 50  0001 C CNN
	1    5300 5250
	-1   0    0    -1  
$EndComp
Text Label 5200 4950 0    60   ~ 0
MOT_C
Wire Wire Line
	5500 4500 5650 4500
Wire Wire Line
	5200 4700 5200 5050
Wire Wire Line
	5200 4300 5200 4450
Wire Wire Line
	5500 5250 5650 5250
Connection ~ 4250 4300
Connection ~ 4250 5500
Wire Wire Line
	5200 5500 5200 5450
Wire Wire Line
	3300 5500 5200 5500
Wire Wire Line
	4050 4000 4250 4000
$Comp
L GND #PWR23
U 1 1 5AE1FECC
P 7850 5350
AR Path="/5AE1F86D/5AE1FECC" Ref="#PWR23"  Part="1" 
AR Path="/5AE23461/5AE1FECC" Ref="#PWR23"  Part="1" 
F 0 "#PWR23" H 7850 5100 50  0001 C CNN
F 1 "GND" H 7850 5200 50  0000 C CNN
F 2 "" H 7850 5350 50  0001 C CNN
F 3 "" H 7850 5350 50  0001 C CNN
	1    7850 5350
	0    -1   -1   0   
$EndComp
Text Label 7900 5050 0    60   ~ 0
GHC
Text Label 7900 5250 0    60   ~ 0
GLC
Text Label 8300 5150 0    60   ~ 0
MOT_C
Text Label 6800 5150 2    60   ~ 0
ENABLE_C
Text Label 6800 5350 2    60   ~ 0
IN_C
$Comp
L C C14
U 1 1 5AE1FED8
P 8200 4900
AR Path="/5AE1F86D/5AE1FED8" Ref="C14"  Part="1" 
AR Path="/5AE23461/5AE1FED8" Ref="C14"  Part="1" 
F 0 "C14" H 8225 5000 50  0000 L CNN
F 1 "360nF" H 8225 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 8238 4750 50  0001 C CNN
F 3 "" H 8200 4900 50  0001 C CNN
F 4 "50V" H 8400 4900 60  0000 C CNN "Voltage"
	1    8200 4900
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 5AE1FEDF
P 6250 5100
AR Path="/5AE1F86D/5AE1FEDF" Ref="C11"  Part="1" 
AR Path="/5AE23461/5AE1FEDF" Ref="C11"  Part="1" 
F 0 "C11" H 6275 5200 50  0000 L CNN
F 1 "100n" H 6275 5000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6288 4950 50  0001 C CNN
F 3 "" H 6250 5100 50  0001 C CNN
	1    6250 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR20
U 1 1 5AE1FEE6
P 6250 5350
AR Path="/5AE1F86D/5AE1FEE6" Ref="#PWR20"  Part="1" 
AR Path="/5AE23461/5AE1FEE6" Ref="#PWR20"  Part="1" 
F 0 "#PWR20" H 6250 5100 50  0001 C CNN
F 1 "GND" H 6250 5200 50  0000 C CNN
F 2 "" H 6250 5350 50  0001 C CNN
F 3 "" H 6250 5350 50  0001 C CNN
	1    6250 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 5350 7850 5350
Wire Wire Line
	7800 5050 7900 5050
Wire Wire Line
	7800 5250 7900 5250
Wire Wire Line
	7800 5150 8300 5150
Wire Wire Line
	6800 5150 6900 5150
Wire Wire Line
	6900 5350 6800 5350
Wire Wire Line
	6800 4950 6900 4950
Wire Wire Line
	8200 5050 8200 5150
Connection ~ 8200 5150
Wire Wire Line
	8200 4700 8200 4750
Wire Wire Line
	7850 4700 8200 4700
Wire Wire Line
	7850 4700 7850 4950
Wire Wire Line
	7850 4950 7800 4950
Wire Wire Line
	6850 4700 6850 4950
Connection ~ 6850 4950
Wire Wire Line
	6250 5350 6250 5250
Wire Wire Line
	6250 4950 6250 4700
Text Label 6800 4950 2    60   ~ 0
+V_MOTOR
Wire Wire Line
	6250 4700 6850 4700
$Comp
L NCP5901 U8
U 1 1 5AE1FEFF
P 7350 5150
AR Path="/5AE1F86D/5AE1FEFF" Ref="U8"  Part="1" 
AR Path="/5AE23461/5AE1FEFF" Ref="U8"  Part="1" 
F 0 "U8" H 7550 4750 60  0000 C CNN
F 1 "NCP5901" H 7300 5500 60  0000 C CNN
F 2 "SMD_Packages:SOIC-8-N" H 7350 5200 60  0001 C CNN
F 3 "" H 7350 5200 60  0001 C CNN
	1    7350 5150
	1    0    0    -1  
$EndComp
Text HLabel 1450 2900 0    60   Input ~ 0
+V_MOTOR
Wire Wire Line
	1450 2900 1550 2900
Text Label 1550 2900 0    60   ~ 0
+V_MOTOR
Text Label 1600 3100 0    60   ~ 0
IN_A
Text Label 1600 3300 0    60   ~ 0
IN_B
Text Label 1600 3500 0    60   ~ 0
IN_C
Text Label 1600 3700 0    60   ~ 0
ENABLE_A
Text Label 1600 4300 0    60   ~ 0
CURRENT
Text HLabel 1450 3100 0    60   Input ~ 0
IN_A
Text HLabel 1450 3300 0    60   Input ~ 0
IN_B
Text HLabel 1450 3500 0    60   Input ~ 0
IN_C
Text HLabel 1450 3700 0    60   Input ~ 0
ENABLE_A
Text HLabel 1450 4300 0    60   Output ~ 0
CURRENT
Wire Wire Line
	1450 3100 1600 3100
Wire Wire Line
	1600 3300 1450 3300
Wire Wire Line
	1450 3500 1600 3500
Wire Wire Line
	1600 3700 1450 3700
Wire Wire Line
	1450 4300 1600 4300
Text HLabel 1450 4500 0    60   Output ~ 0
MOTOR_A
Wire Wire Line
	1450 4500 1600 4500
Text Label 1600 4500 0    60   ~ 0
MOT_A
Text HLabel 1450 4700 0    60   Output ~ 0
MOTOR_B
Wire Wire Line
	1450 4700 1600 4700
Text Label 1600 4700 0    60   ~ 0
MOT_B
Text HLabel 1450 4900 0    60   Output ~ 0
MOTOR_C
Wire Wire Line
	1450 4900 1600 4900
Text Label 1600 4900 0    60   ~ 0
MOT_C
Text Label 1600 3900 0    60   ~ 0
ENABLE_B
Text HLabel 1450 3900 0    60   Input ~ 0
ENABLE_B
Wire Wire Line
	1600 3900 1450 3900
Text Label 1600 4100 0    60   ~ 0
ENABLE_C
Text HLabel 1450 4100 0    60   Input ~ 0
ENABLE_C
Wire Wire Line
	1600 4100 1450 4100
$EndSCHEMATC

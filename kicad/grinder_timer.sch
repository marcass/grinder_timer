EESchema Schematic File Version 4
LIBS:grinder_timer-cache
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
$Comp
L grinder_timer-rescue:arduino_mini U1
U 1 1 5887E781
P 2200 3000
F 0 "U1" H 2700 2050 70  0000 C CNN
F 1 "arduino_nano" H 2950 1950 70  0000 C CNN
F 2 "arduino_nano:ArduinoNano" H 2200 2950 60  0000 C CNN
F 3 "" H 2200 3000 60  0001 C CNN
	1    2200 3000
	1    0    0    -1  
$EndComp
Text GLabel 4150 5950 2    60   Input ~ 0
Event_button
Text GLabel 3400 3400 2    60   Input ~ 0
Event_button
Text GLabel 1750 5600 0    60   Input ~ 0
Grind_button
Text GLabel 3200 3050 2    60   Input ~ 0
Grind_button
Text GLabel 2500 5900 0    60   Input ~ 0
RL
Text GLabel 3100 2450 2    60   Input ~ 0
RL
$Comp
L grinder_timer-rescue:GND #PWR01
U 1 1 588A6D00
P 4000 6750
F 0 "#PWR01" H 4000 6500 50  0001 C CNN
F 1 "GND" H 4000 6600 50  0000 C CNN
F 2 "" H 4000 6750 50  0000 C CNN
F 3 "" H 4000 6750 50  0000 C CNN
	1    4000 6750
	-1   0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:GND #PWR02
U 1 1 588A6D38
P 1800 6400
F 0 "#PWR02" H 1800 6150 50  0001 C CNN
F 1 "GND" H 1800 6250 50  0000 C CNN
F 2 "" H 1800 6400 50  0000 C CNN
F 3 "" H 1800 6400 50  0000 C CNN
	1    1800 6400
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:R R3
U 1 1 588A6D98
P 1800 6000
F 0 "R3" V 1880 6000 50  0000 C CNN
F 1 "10k" V 1800 6000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1730 6000 50  0001 C CNN
F 3 "" H 1800 6000 50  0000 C CNN
	1    1800 6000
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:R R2
U 1 1 588A6DCC
P 4000 6400
F 0 "R2" V 4080 6400 50  0000 C CNN
F 1 "10k" V 4000 6400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3930 6400 50  0001 C CNN
F 3 "" H 4000 6400 50  0000 C CNN
	1    4000 6400
	-1   0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:R R4
U 1 1 588A6FC8
P 2550 6250
F 0 "R4" V 2630 6250 50  0000 C CNN
F 1 "10K" V 2550 6250 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2480 6250 50  0001 C CNN
F 3 "" H 2550 6250 50  0000 C CNN
	1    2550 6250
	-1   0    0    -1  
$EndComp
Text GLabel 4300 5850 2    60   Input ~ 0
Button_LED
$Comp
L grinder_timer-rescue:R R5
U 1 1 588A76BD
P 4000 5850
F 0 "R5" V 4080 5850 50  0000 C CNN
F 1 "200" V 4000 5850 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3930 5850 50  0001 C CNN
F 3 "" H 4000 5850 50  0000 C CNN
	1    4000 5850
	0    1    1    0   
$EndComp
Text GLabel 3500 3300 2    60   Input ~ 0
Button_LED
$Comp
L grinder_timer-rescue:GND #PWR03
U 1 1 588A7979
P 2550 6450
F 0 "#PWR03" H 2550 6200 50  0001 C CNN
F 1 "GND" H 2550 6300 50  0000 C CNN
F 2 "" H 2550 6450 50  0000 C CNN
F 3 "" H 2550 6450 50  0000 C CNN
	1    2550 6450
	-1   0    0    -1  
$EndComp
Text GLabel 3850 5450 2    60   Input ~ 0
SCL
Text GLabel 1300 3300 0    60   Input ~ 0
SCL
Text GLabel 3850 5550 2    60   Input ~ 0
SDA
Text GLabel 1400 3200 0    60   Input ~ 0
SDA
Text GLabel 2250 5800 0    60   Input ~ 0
SW
Text GLabel 2150 5500 0    60   Input ~ 0
DT_conn
Text GLabel 2550 5100 1    60   Input ~ 0
CLK_conn
$Comp
L grinder_timer-rescue:CP C2
U 1 1 58A7F980
P 5700 2950
F 0 "C2" H 5725 3050 50  0000 L CNN
F 1 "47muF" H 5725 2850 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P3.50mm" H 5738 2800 50  0001 C CNN
F 3 "" H 5700 2950 50  0000 C CNN
	1    5700 2950
	-1   0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:CP C1
U 1 1 58A7FC41
P 7700 3100
F 0 "C1" H 7725 3200 50  0000 L CNN
F 1 "47muF" H 7725 3000 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P3.50mm" H 7738 2950 50  0001 C CNN
F 3 "" H 7700 3100 50  0000 C CNN
	1    7700 3100
	-1   0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:GND #PWR04
U 1 1 58A7FF34
P 7700 3450
F 0 "#PWR04" H 7700 3200 50  0001 C CNN
F 1 "GND" H 7700 3300 50  0000 C CNN
F 2 "" H 7700 3450 50  0000 C CNN
F 3 "" H 7700 3450 50  0000 C CNN
	1    7700 3450
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:GND #PWR05
U 1 1 58A7FF5E
P 5700 3250
F 0 "#PWR05" H 5700 3000 50  0001 C CNN
F 1 "GND" H 5700 3100 50  0000 C CNN
F 2 "" H 5700 3250 50  0000 C CNN
F 3 "" H 5700 3250 50  0000 C CNN
	1    5700 3250
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:+5V #PWR06
U 1 1 58A7FF83
P 2050 6950
F 0 "#PWR06" H 2050 6800 50  0001 C CNN
F 1 "+5V" H 2050 7090 50  0000 C CNN
F 2 "" H 2050 6950 50  0000 C CNN
F 3 "" H 2050 6950 50  0000 C CNN
	1    2050 6950
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:R R6
U 1 1 58A801DE
P 2300 6650
F 0 "R6" V 2380 6650 50  0000 C CNN
F 1 "10k" V 2300 6650 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2230 6650 50  0001 C CNN
F 3 "" H 2300 6650 50  0000 C CNN
	1    2300 6650
	-1   0    0    -1  
$EndComp
Text GLabel 3100 2550 2    60   Input ~ 0
SW
Text GLabel 3400 3600 2    60   Input ~ 0
CLK
Text GLabel 3150 3500 2    60   Input ~ 0
DT
$Comp
L grinder_timer-rescue:+5V #PWR07
U 1 1 58A80541
P 2200 1750
F 0 "#PWR07" H 2200 1600 50  0001 C CNN
F 1 "+5V" H 2200 1890 50  0000 C CNN
F 2 "" H 2200 1750 50  0000 C CNN
F 3 "" H 2200 1750 50  0000 C CNN
	1    2200 1750
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:GND #PWR08
U 1 1 58A80570
P 2200 4600
F 0 "#PWR08" H 2200 4350 50  0001 C CNN
F 1 "GND" H 2200 4450 50  0000 C CNN
F 2 "" H 2200 4600 50  0000 C CNN
F 3 "" H 2200 4600 50  0000 C CNN
	1    2200 4600
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:CONN_01X08 P1
U 1 1 58A80D49
P 2950 5750
F 0 "P1" H 2950 6200 50  0000 C CNN
F 1 "CONN_01X08" V 3050 5750 50  0000 C CNN
F 2 "Connectors_Phoenix:PhoenixContact_MCV-G_08x3.50mm_Vertical" H 2950 5750 50  0001 C CNN
F 3 "" H 2950 5750 50  0000 C CNN
	1    2950 5750
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:CONN_01X08 P2
U 1 1 58A80E00
P 3450 5700
F 0 "P2" H 3450 6150 50  0000 C CNN
F 1 "CONN_01X08" V 3550 5700 50  0000 C CNN
F 2 "Connectors_Phoenix:PhoenixContact_MCV-G_08x3.50mm_Vertical" H 3450 5700 50  0001 C CNN
F 3 "" H 3450 5700 50  0000 C CNN
	1    3450 5700
	-1   0    0    1   
$EndComp
$Comp
L grinder_timer-rescue:GND #PWR09
U 1 1 58A80FD8
P 5150 6800
F 0 "#PWR09" H 5150 6550 50  0001 C CNN
F 1 "GND" H 5150 6650 50  0000 C CNN
F 2 "" H 5150 6800 50  0000 C CNN
F 3 "" H 5150 6800 50  0000 C CNN
	1    5150 6800
	-1   0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:+5V #PWR010
U 1 1 58A81069
P 3650 5000
F 0 "#PWR010" H 3650 4850 50  0001 C CNN
F 1 "+5V" H 3650 5140 50  0000 C CNN
F 2 "" H 3650 5000 50  0000 C CNN
F 3 "" H 3650 5000 50  0000 C CNN
	1    3650 5000
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:+5V #PWR011
U 1 1 58A8161C
P 2550 6900
F 0 "#PWR011" H 2550 6750 50  0001 C CNN
F 1 "+5V" H 2550 7040 50  0000 C CNN
F 2 "" H 2550 6900 50  0000 C CNN
F 3 "" H 2550 6900 50  0000 C CNN
	1    2550 6900
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:GND #PWR012
U 1 1 58A8168E
P 2750 6850
F 0 "#PWR012" H 2750 6600 50  0001 C CNN
F 1 "GND" H 2750 6700 50  0000 C CNN
F 2 "" H 2750 6850 50  0000 C CNN
F 3 "" H 2750 6850 50  0000 C CNN
	1    2750 6850
	-1   0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:CONN_01X03 P3
U 1 1 58A81789
P 6100 2700
F 0 "P3" H 6100 2900 50  0000 C CNN
F 1 "Enc_jumperDT" V 6200 2700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 6100 2700 50  0001 C CNN
F 3 "" H 6100 2700 50  0000 C CNN
	1    6100 2700
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:CONN_01X03 P4
U 1 1 58A81C5D
P 8000 2750
F 0 "P4" H 8000 2950 50  0000 C CNN
F 1 "Enc_jumperCLK" V 8100 2750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 8000 2750 50  0001 C CNN
F 3 "" H 8000 2750 50  0000 C CNN
	1    8000 2750
	1    0    0    -1  
$EndComp
Text GLabel 5450 2800 0    60   Input ~ 0
DT
Text GLabel 7600 2850 0    60   Input ~ 0
CLK
Text GLabel 1150 2800 0    60   Input ~ 0
A_dt
Text GLabel 1150 2900 0    60   Input ~ 0
A_sck
Text GLabel 5600 2600 0    60   Input ~ 0
A_dt
Text GLabel 7450 2650 0    60   Input ~ 0
A_sck
Text GLabel 5300 2700 0    60   Input ~ 0
DT_conn
Text GLabel 7050 2750 0    60   Input ~ 0
CLK_conn
Wire Wire Line
	3100 2450 2900 2450
Wire Wire Line
	3400 3600 2900 3600
Wire Wire Line
	4000 6250 4000 5950
Wire Wire Line
	4000 6550 4000 6750
Wire Wire Line
	1800 5600 1800 5850
Wire Wire Line
	1800 6150 1800 6400
Wire Wire Line
	3400 3400 2900 3400
Wire Wire Line
	2550 5900 2550 6100
Wire Wire Line
	2550 6400 2550 6450
Wire Wire Line
	1300 3300 1500 3300
Wire Wire Line
	1400 3200 1500 3200
Wire Wire Line
	2550 5400 2750 5400
Wire Wire Line
	2050 7100 2050 6950
Wire Wire Line
	3850 5450 3650 5450
Wire Wire Line
	3850 5550 3650 5550
Wire Wire Line
	3500 3300 2900 3300
Connection ~ 2550 5900
Connection ~ 4000 5950
Wire Wire Line
	3650 6050 3700 6050
Wire Wire Line
	2300 7100 2300 6800
Wire Wire Line
	3200 3050 2900 3050
Wire Wire Line
	3100 2550 2900 2550
Wire Wire Line
	3150 3500 2900 3500
Wire Wire Line
	2200 1850 2200 1750
Wire Wire Line
	2200 4600 2200 4550
Wire Wire Line
	5150 5650 5150 6800
Wire Wire Line
	3650 5350 3650 5000
Wire Wire Line
	2250 5800 2300 5800
Wire Wire Line
	2050 7100 2300 7100
Wire Wire Line
	2500 5900 2550 5900
Connection ~ 2300 5800
Wire Wire Line
	2550 7050 2550 6900
Wire Wire Line
	2750 6000 2650 6000
Wire Wire Line
	2650 6000 2650 7050
Wire Wire Line
	2650 7050 2550 7050
Wire Wire Line
	2750 6100 2750 6850
Wire Wire Line
	5300 2700 5900 2700
Wire Wire Line
	5450 2800 5700 2800
Wire Wire Line
	7600 2850 7700 2850
Wire Wire Line
	7050 2750 7800 2750
Wire Wire Line
	1500 2800 1150 2800
Wire Wire Line
	1500 2900 1150 2900
Wire Wire Line
	3850 5850 3650 5850
Wire Wire Line
	3650 5650 5150 5650
$Comp
L grinder_timer-rescue:GND #PWR013
U 1 1 58A83412
P 4950 7100
F 0 "#PWR013" H 4950 6850 50  0001 C CNN
F 1 "GND" H 4950 6950 50  0000 C CNN
F 2 "" H 4950 7100 50  0000 C CNN
F 3 "" H 4950 7100 50  0000 C CNN
	1    4950 7100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4950 5750 4950 7100
Wire Wire Line
	4300 5850 4150 5850
Wire Wire Line
	4950 5750 3650 5750
$Comp
L grinder_timer-rescue:+5V #PWR014
U 1 1 58A83529
P 3200 5100
F 0 "#PWR014" H 3200 4950 50  0001 C CNN
F 1 "+5V" H 3200 5240 50  0000 C CNN
F 2 "" H 3200 5100 50  0000 C CNN
F 3 "" H 3200 5100 50  0000 C CNN
	1    3200 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 6300 3200 6300
Wire Wire Line
	3200 6300 3200 5100
Wire Wire Line
	3700 6050 3700 6300
Wire Wire Line
	3650 5950 4000 5950
Wire Wire Line
	1750 5600 1800 5600
Connection ~ 1800 5600
$Comp
L grinder_timer-rescue:+5V #PWR015
U 1 1 58A83B09
P 1650 6950
F 0 "#PWR015" H 1650 6800 50  0001 C CNN
F 1 "+5V" H 1650 7090 50  0000 C CNN
F 2 "" H 1650 6950 50  0000 C CNN
F 3 "" H 1650 6950 50  0000 C CNN
	1    1650 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 7100 1650 6950
Wire Wire Line
	1650 7100 1900 7100
Wire Wire Line
	2750 5700 1900 5700
Wire Wire Line
	1900 5700 1900 7100
Wire Wire Line
	7800 2650 7450 2650
Wire Wire Line
	7700 2950 7700 2850
Connection ~ 7700 2850
Wire Wire Line
	7700 3450 7700 3250
Wire Wire Line
	5700 3250 5700 3100
Wire Wire Line
	5600 2600 5900 2600
Connection ~ 5700 2800
$Comp
L grinder_timer-rescue:CONN_01X01 MH1
U 1 1 58A84A57
P 6150 4200
F 0 "MH1" H 6150 4300 50  0000 C CNN
F 1 "mounting hole" V 6250 4200 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3_DIN965_Pad" H 6150 4200 50  0001 C CNN
F 3 "" H 6150 4200 50  0000 C CNN
	1    6150 4200
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:CONN_01X01 MH2
U 1 1 58A84BDF
P 6950 4200
F 0 "MH2" H 6950 4300 50  0000 C CNN
F 1 "mounting hole" V 7050 4200 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3_DIN965_Pad" H 6950 4200 50  0001 C CNN
F 3 "" H 6950 4200 50  0000 C CNN
	1    6950 4200
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:CONN_01X01 MH3
U 1 1 58A84C2B
P 7750 4250
F 0 "MH3" H 7750 4350 50  0000 C CNN
F 1 "mounting hole" V 7850 4250 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3_DIN965_Pad" H 7750 4250 50  0001 C CNN
F 3 "" H 7750 4250 50  0000 C CNN
	1    7750 4250
	1    0    0    -1  
$EndComp
$Comp
L grinder_timer-rescue:CONN_01X01 MH4
U 1 1 58A84CD0
P 7350 5050
F 0 "MH4" H 7350 5150 50  0000 C CNN
F 1 "mounting hole" V 7450 5050 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_3.2mm_M3_DIN965_Pad" H 7350 5050 50  0001 C CNN
F 3 "" H 7350 5050 50  0000 C CNN
	1    7350 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 6500 2300 5800
Wire Wire Line
	3150 2750 2900 2750
Wire Wire Line
	2550 5900 2750 5900
Wire Wire Line
	4000 5950 4150 5950
Wire Wire Line
	2300 5800 2750 5800
Wire Wire Line
	1800 5600 2750 5600
Wire Wire Line
	7700 2850 7800 2850
Wire Wire Line
	5700 2800 5900 2800
Wire Wire Line
	2550 5100 2550 5400
Wire Wire Line
	2150 5500 2750 5500
$EndSCHEMATC

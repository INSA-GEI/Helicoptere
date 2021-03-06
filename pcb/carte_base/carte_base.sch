EESchema Schematic File Version 4
LIBS:carte_base-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Carte BASE"
Date "2019-09-26"
Rev "1.0"
Comp "Toulouse - GEI"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 5D2F2658
P 950 900
F 0 "H1" H 1050 905 50  0001 L CNN
F 1 "MountingHole_Pad" H 1050 860 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_ISO14580_Pad" H 950 900 50  0001 C CNN
F 3 "~" H 950 900 50  0001 C CNN
	1    950  900 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5D2F26FB
P 1300 900
F 0 "H2" H 1400 905 50  0001 L CNN
F 1 "MountingHole_Pad" H 1400 860 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_ISO14580_Pad" H 1300 900 50  0001 C CNN
F 3 "~" H 1300 900 50  0001 C CNN
	1    1300 900 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 5D2F2813
P 1650 900
F 0 "H3" H 1750 905 50  0001 L CNN
F 1 "MountingHole_Pad" H 1750 860 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_ISO14580_Pad" H 1650 900 50  0001 C CNN
F 3 "~" H 1650 900 50  0001 C CNN
	1    1650 900 
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 5D2F288E
P 2000 900
F 0 "H4" H 2100 905 50  0001 L CNN
F 1 "MountingHole_Pad" H 2100 860 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_ISO14580_Pad" H 2000 900 50  0001 C CNN
F 3 "~" H 2000 900 50  0001 C CNN
	1    2000 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5D2F318B
P 2300 2400
F 0 "#PWR0102" H 2300 2150 50  0001 C CNN
F 1 "GND" H 2305 2227 50  0000 C CNN
F 2 "" H 2300 2400 50  0001 C CNN
F 3 "" H 2300 2400 50  0001 C CNN
	1    2300 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 5D2F3A22
P 1750 2000
F 0 "C1" H 1868 2046 50  0000 L CNN
F 1 "10µF" H 1868 1955 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 1788 1850 50  0001 C CNN
F 3 "~" H 1750 2000 50  0001 C CNN
	1    1750 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1650 1750 1850
Connection ~ 1750 1650
Wire Wire Line
	1750 2150 1750 2350
Wire Wire Line
	950  1000 1300 1000
Connection ~ 1300 1000
Wire Wire Line
	1300 1000 1500 1000
Connection ~ 1650 1000
Wire Wire Line
	1650 1000 2000 1000
Wire Wire Line
	1500 1000 1500 1050
Connection ~ 1500 1000
Wire Wire Line
	1500 1000 1650 1000
$Comp
L power:GND #PWR0103
U 1 1 5D2F4D0C
P 1500 1050
F 0 "#PWR0103" H 1500 800 50  0001 C CNN
F 1 "GND" H 1505 877 50  0000 C CNN
F 2 "" H 1500 1050 50  0001 C CNN
F 3 "" H 1500 1050 50  0001 C CNN
	1    1500 1050
	1    0    0    -1  
$EndComp
$Comp
L Insa:STM32L031 U1
U 1 1 5D30C73C
P 8800 2100
F 0 "U1" H 8750 3165 50  0000 C CNN
F 1 "STM32L031" H 8750 3074 50  0000 C CNN
F 2 "Insa:STM32-Nucleo-32" H 8800 3100 50  0001 C CNN
F 3 "" H 8800 3100 50  0001 C CNN
	1    8800 2100
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 5D30DBA0
P 1500 4800
F 0 "R2" H 1430 4754 50  0000 R CNN
F 1 "2,7k" H 1430 4845 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 1430 4800 50  0001 C CNN
F 3 "~" H 1500 4800 50  0001 C CNN
	1    1500 4800
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 5D30F1D4
P 1500 5150
F 0 "R3" H 1570 5196 50  0000 L CNN
F 1 "1K" H 1570 5105 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 1430 5150 50  0001 C CNN
F 3 "~" H 1500 5150 50  0001 C CNN
	1    1500 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5D30F535
P 1500 6700
F 0 "R5" H 1570 6746 50  0000 L CNN
F 1 "1K" H 1570 6655 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 1430 6700 50  0001 C CNN
F 3 "~" H 1500 6700 50  0001 C CNN
	1    1500 6700
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0106
U 1 1 5D30FFD4
P 1900 4500
F 0 "#PWR0106" H 1900 4350 50  0001 C CNN
F 1 "+3.3V" H 1915 4673 50  0000 C CNN
F 2 "" H 1900 4500 50  0001 C CNN
F 3 "" H 1900 4500 50  0001 C CNN
	1    1900 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5D31018C
P 2350 5150
F 0 "C5" H 2465 5196 50  0000 L CNN
F 1 "100nF" H 2465 5105 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 2388 5000 50  0001 C CNN
F 3 "~" H 2350 5150 50  0001 C CNN
	1    2350 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5D310503
P 2350 6700
F 0 "C6" H 2465 6746 50  0000 L CNN
F 1 "100nF" H 2465 6655 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 2388 6550 50  0001 C CNN
F 3 "~" H 2350 6700 50  0001 C CNN
	1    2350 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 6500 1500 6550
Wire Wire Line
	1500 4950 1500 5000
$Comp
L power:GND #PWR0107
U 1 1 5D312AA9
P 1900 6900
F 0 "#PWR0107" H 1900 6650 50  0001 C CNN
F 1 "GND" H 1905 6727 50  0000 C CNN
F 2 "" H 1900 6900 50  0001 C CNN
F 3 "" H 1900 6900 50  0001 C CNN
	1    1900 6900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5D312B3A
P 1900 5350
F 0 "#PWR0108" H 1900 5100 50  0001 C CNN
F 1 "GND" H 1905 5177 50  0000 C CNN
F 2 "" H 1900 5350 50  0001 C CNN
F 3 "" H 1900 5350 50  0001 C CNN
	1    1900 5350
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0109
U 1 1 5D3131AD
P 1900 6050
F 0 "#PWR0109" H 1900 5900 50  0001 C CNN
F 1 "+3.3V" H 1915 6223 50  0000 C CNN
F 2 "" H 1900 6050 50  0001 C CNN
F 3 "" H 1900 6050 50  0001 C CNN
	1    1900 6050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0110
U 1 1 5D31323E
P 9900 1100
F 0 "#PWR0110" H 9900 950 50  0001 C CNN
F 1 "+3.3V" H 9915 1273 50  0000 C CNN
F 2 "" H 9900 1100 50  0001 C CNN
F 3 "" H 9900 1100 50  0001 C CNN
	1    9900 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0111
U 1 1 5D3132CF
P 6400 1650
F 0 "#PWR0111" H 6400 1500 50  0001 C CNN
F 1 "+3.3V" H 6415 1823 50  0000 C CNN
F 2 "" H 6400 1650 50  0001 C CNN
F 3 "" H 6400 1650 50  0001 C CNN
	1    6400 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 6050 1900 6200
Wire Wire Line
	1900 4500 1900 4650
Wire Wire Line
	2350 6550 2350 6500
Connection ~ 2350 6500
Wire Wire Line
	2350 6500 2650 6500
Wire Wire Line
	1900 5350 1900 5300
Wire Wire Line
	1900 6850 1900 6900
Wire Wire Line
	2350 5000 2350 4950
Connection ~ 2350 4950
Wire Wire Line
	2350 4950 2650 4950
Text Label 2450 4950 0    50   ~ 0
Consigne_3V3_Avant
Text Label 2500 6500 0    50   ~ 0
Consigne_3V3_Arriere
Wire Wire Line
	9400 1250 9550 1250
Wire Wire Line
	9550 1250 9550 1100
Wire Wire Line
	8300 1350 8300 1250
Wire Wire Line
	8300 1250 8150 1250
Connection ~ 8300 1250
$Comp
L power:GND #PWR0112
U 1 1 5D33A410
P 8150 1250
F 0 "#PWR0112" H 8150 1000 50  0001 C CNN
F 1 "GND" H 8155 1077 50  0000 C CNN
F 2 "" H 8150 1250 50  0001 C CNN
F 3 "" H 8150 1250 50  0001 C CNN
	1    8150 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 2200 7450 2200
Wire Wire Line
	8300 2300 7450 2300
Text Label 7450 2200 0    50   ~ 0
Consigne_3V3_Avant
Text Label 7450 2300 0    50   ~ 0
Consigne_3V3_Arriere
Connection ~ 1500 4950
$Comp
L Device:R R4
U 1 1 5D378003
P 1500 6350
F 0 "R4" H 1430 6304 50  0000 R CNN
F 1 "2,7K" H 1430 6395 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 1430 6350 50  0001 C CNN
F 3 "~" H 1500 6350 50  0001 C CNN
	1    1500 6350
	-1   0    0    1   
$EndComp
Connection ~ 1500 6500
Wire Wire Line
	9900 1450 9900 1100
Wire Wire Line
	9400 1450 9900 1450
$Comp
L insa-sym:LOGO #G1
U 1 1 5D38297C
P 7600 6800
F 0 "#G1" H 7600 6348 60  0001 C CNN
F 1 "LOGO" H 7600 7252 60  0001 C CNN
F 2 "" H 7600 6800 50  0001 C CNN
F 3 "" H 7600 6800 50  0001 C CNN
	1    7600 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2650 4400 2650
Wire Wire Line
	3700 2750 4400 2750
Text Label 4400 2650 2    50   ~ 0
Alim_12V
Text Label 4400 2750 2    50   ~ 0
Alim_GND
Text Label 5100 1500 2    50   ~ 0
Consigne_10V_Avant
Text Label 5100 1600 2    50   ~ 0
Consigne_10V_Arriere
$Comp
L Device:L L1
U 1 1 5D325DDF
P 2050 1650
F 0 "L1" V 2240 1650 50  0000 C CNN
F 1 "100µH" V 2149 1650 50  0000 C CNN
F 2 "Inductor_SMD:L_Bourns-SRN6028" H 2050 1650 50  0001 C CNN
F 3 "~" H 2050 1650 50  0001 C CNN
F 4 "Bourns SRR6028-102Y" V 2050 1650 50  0001 C CNN "Ref"
	1    2050 1650
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C2
U 1 1 5D325FE2
P 2300 2000
F 0 "C2" H 2415 2046 50  0000 L CNN
F 1 "100nF" H 2415 1955 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 2338 1850 50  0001 C CNN
F 3 "~" H 2300 2000 50  0001 C CNN
	1    2300 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1650 1900 1650
Wire Wire Line
	2200 1650 2300 1650
Wire Wire Line
	2300 1650 2300 1850
Wire Wire Line
	2300 2150 2300 2350
$Comp
L power:VDD #PWR0117
U 1 1 5D334033
P 2300 1650
F 0 "#PWR0117" H 2300 1500 50  0001 C CNN
F 1 "VDD" H 2317 1823 50  0000 C CNN
F 2 "" H 2300 1650 50  0001 C CNN
F 3 "" H 2300 1650 50  0001 C CNN
	1    2300 1650
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0118
U 1 1 5D334239
P 9550 1100
F 0 "#PWR0118" H 9550 950 50  0001 C CNN
F 1 "VDD" H 9567 1273 50  0000 C CNN
F 2 "" H 9550 1100 50  0001 C CNN
F 3 "" H 9550 1100 50  0001 C CNN
	1    9550 1100
	1    0    0    -1  
$EndComp
Connection ~ 2300 1650
Text Label 800  1650 0    50   ~ 0
Alim_12V
Text Label 850  2350 0    50   ~ 0
Alim_GND
Wire Wire Line
	750  4650 1500 4650
Wire Wire Line
	700  6200 1500 6200
Text Label 750  4650 0    50   ~ 0
Consigne_10V_Avant
Text Label 700  6200 0    50   ~ 0
Consigne_10V_Arriere
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 5D8C0337
P 3500 2650
F 0 "J2" H 3418 2775 50  0000 C CNN
F 1 "Conn_01x02" H 3580 2551 50  0001 L CNN
F 2 "Connector_JST:JST_VH_B2PS-VH_1x02_P3.96mm_Horizontal" H 3500 2650 50  0001 C CNN
F 3 "~" H 3500 2650 50  0001 C CNN
	1    3500 2650
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 5D8C0A62
P 4300 3050
F 0 "J3" H 4380 2996 50  0000 L CNN
F 1 "Conn_01x02" H 4380 2951 50  0001 L CNN
F 2 "Connector_JST:JST_VH_B2PS-VH_1x02_P3.96mm_Horizontal" H 4300 3050 50  0001 C CNN
F 3 "~" H 4300 3050 50  0001 C CNN
	1    4300 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 5D8C1354
P 3500 2100
F 0 "J4" H 3418 2225 50  0000 C CNN
F 1 "Conn_01x02" H 3580 2001 50  0001 L CNN
F 2 "Insa:DF3A-2P-2DS" H 3500 2100 50  0001 C CNN
F 3 "~" H 3500 2100 50  0001 C CNN
	1    3500 2100
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5D8D12A6
P 3500 850
F 0 "J1" H 3418 1075 50  0000 C CNN
F 1 "Conn_01x04" H 3418 516 50  0001 C CNN
F 2 "Insa:DF3A-4P-2DS" H 3500 850 50  0001 C CNN
F 3 "~" H 3500 850 50  0001 C CNN
	1    3500 850 
	-1   0    0    -1  
$EndComp
$Comp
L Device:CP C3
U 1 1 5D8DE5DD
P 2800 2000
F 0 "C3" H 2918 2046 50  0000 L CNN
F 1 "15µF" H 2918 1955 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 2838 1850 50  0001 C CNN
F 3 "~" H 2800 2000 50  0001 C CNN
	1    2800 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 2150 2800 2350
Wire Wire Line
	2300 2350 2800 2350
Wire Wire Line
	2800 1850 2800 1650
Wire Wire Line
	2800 1650 2300 1650
$Comp
L power:+12V #PWR0101
U 1 1 5D2F3144
P 1750 1650
F 0 "#PWR0101" H 1750 1500 50  0001 C CNN
F 1 "+12V" H 1765 1823 50  0000 C CNN
F 2 "" H 1750 1650 50  0001 C CNN
F 3 "" H 1750 1650 50  0001 C CNN
	1    1750 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3150 3450 3150
Text Label 3450 3150 0    50   ~ 0
Alim_GND
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5D8EAA98
P 1200 2850
F 0 "#FLG0101" H 1200 2925 50  0001 C CNN
F 1 "PWR_FLAG" H 1200 3023 50  0000 C CNN
F 2 "" H 1200 2850 50  0001 C CNN
F 3 "~" H 1200 2850 50  0001 C CNN
	1    1200 2850
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0104
U 1 1 5D8EB676
P 850 2850
F 0 "#PWR0104" H 850 2700 50  0001 C CNN
F 1 "+12V" H 865 3023 50  0000 C CNN
F 2 "" H 850 2850 50  0001 C CNN
F 3 "" H 850 2850 50  0001 C CNN
	1    850  2850
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0105
U 1 1 5D8ECF07
P 850 3200
F 0 "#PWR0105" H 850 3050 50  0001 C CNN
F 1 "VDD" H 867 3373 50  0000 C CNN
F 2 "" H 850 3200 50  0001 C CNN
F 3 "" H 850 3200 50  0001 C CNN
	1    850  3200
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5D8EE571
P 1200 3200
F 0 "#FLG0102" H 1200 3275 50  0001 C CNN
F 1 "PWR_FLAG" H 1200 3373 50  0000 C CNN
F 2 "" H 1200 3200 50  0001 C CNN
F 3 "~" H 1200 3200 50  0001 C CNN
	1    1200 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5D8F6FE1
P 1700 2700
F 0 "#PWR0113" H 1700 2450 50  0001 C CNN
F 1 "GND" H 1705 2527 50  0000 C CNN
F 2 "" H 1700 2700 50  0001 C CNN
F 3 "" H 1700 2700 50  0001 C CNN
	1    1700 2700
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5D8F86D5
P 2050 2700
F 0 "#FLG0103" H 2050 2775 50  0001 C CNN
F 1 "PWR_FLAG" H 2050 2873 50  0000 C CNN
F 2 "" H 2050 2700 50  0001 C CNN
F 3 "~" H 2050 2700 50  0001 C CNN
	1    2050 2700
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR0114
U 1 1 5D8F9D22
P 1700 3250
F 0 "#PWR0114" H 1700 3100 50  0001 C CNN
F 1 "+3.3V" H 1715 3423 50  0000 C CNN
F 2 "" H 1700 3250 50  0001 C CNN
F 3 "" H 1700 3250 50  0001 C CNN
	1    1700 3250
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 5D8FB40F
P 2050 3250
F 0 "#FLG0104" H 2050 3325 50  0001 C CNN
F 1 "PWR_FLAG" H 2050 3423 50  0000 C CNN
F 2 "" H 2050 3250 50  0001 C CNN
F 3 "~" H 2050 3250 50  0001 C CNN
	1    2050 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  2850 1200 2850
Wire Wire Line
	850  3200 1200 3200
Wire Wire Line
	1700 2700 2050 2700
Wire Wire Line
	1700 3250 2050 3250
$Comp
L Device:D_Schottky_x2_Serial_AKC D2
U 1 1 5D9103BC
P 1900 4950
F 0 "D2" V 2150 4800 50  0000 C CNN
F 1 "BAT754S,125" V 2050 4600 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1900 4950 50  0001 C CNN
F 3 "~" H 1900 4950 50  0001 C CNN
	1    1900 4950
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Schottky_x2_Serial_AKC D3
U 1 1 5D91123C
P 1900 6500
F 0 "D3" V 2150 6350 50  0000 C CNN
F 1 "BAT754S,125" V 2050 6150 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1900 6500 50  0001 C CNN
F 3 "~" H 1900 6500 50  0001 C CNN
	1    1900 6500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1500 5300 1900 5300
Wire Wire Line
	1500 4950 2100 4950
Wire Wire Line
	1500 6850 1900 6850
Wire Wire Line
	1500 6500 2100 6500
Connection ~ 2100 4950
Wire Wire Line
	2100 4950 2350 4950
Connection ~ 2100 6500
Wire Wire Line
	2100 6500 2350 6500
Wire Wire Line
	1900 5250 1900 5300
Connection ~ 1900 5300
Wire Wire Line
	1900 6800 1900 6850
Connection ~ 1900 6850
Wire Wire Line
	1900 5300 2350 5300
Wire Wire Line
	1900 6850 2350 6850
$Comp
L Device:L L2
U 1 1 5D92C30F
P 6800 1650
F 0 "L2" V 6990 1650 50  0000 C CNN
F 1 "10mH" V 6899 1650 50  0000 C CNN
F 2 "Inductor_SMD:L_Wuerth_WE-PD2-Typ-L" H 6800 1650 50  0001 C CNN
F 3 "~" H 6800 1650 50  0001 C CNN
F 4 "WE 74404086103" V 6800 1650 50  0001 C CNN "Ref"
	1    6800 1650
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP C7
U 1 1 5D92E6CC
P 6550 1800
F 0 "C7" H 6668 1846 50  0000 L CNN
F 1 "10µF" H 6668 1755 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 6588 1650 50  0001 C CNN
F 3 "~" H 6550 1800 50  0001 C CNN
	1    6550 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C8
U 1 1 5D930210
P 7100 1800
F 0 "C8" H 7218 1846 50  0000 L CNN
F 1 "10µF" H 7218 1755 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 7138 1650 50  0001 C CNN
F 3 "~" H 7100 1800 50  0001 C CNN
	1    7100 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5D935ECA
P 6800 2000
F 0 "#PWR0115" H 6800 1750 50  0001 C CNN
F 1 "GND" H 6805 1827 50  0000 C CNN
F 2 "" H 6800 2000 50  0001 C CNN
F 3 "" H 6800 2000 50  0001 C CNN
	1    6800 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 1650 6550 1650
Wire Wire Line
	6550 1650 6650 1650
Connection ~ 6550 1650
Wire Wire Line
	6950 1650 7100 1650
Wire Wire Line
	6550 1950 6800 1950
Wire Wire Line
	6800 1950 6800 2000
Connection ~ 6800 1950
Wire Wire Line
	6800 1950 7100 1950
Wire Wire Line
	7100 1650 8300 1650
Connection ~ 7100 1650
$Comp
L Switch:SW_Push SW1
U 1 1 5D948B58
P 8000 1950
F 0 "SW1" H 8000 2143 50  0000 C CNN
F 1 "FSMRA3JH04" H 8000 2144 50  0001 C CNN
F 2 "Button_Switch_THT:SW_Tactile_SPST_Angled_PTS645Vx58-2LFS" H 8000 2150 50  0001 C CNN
F 3 "~" H 8000 2150 50  0001 C CNN
	1    8000 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 1950 8300 1950
Wire Wire Line
	7800 1950 7800 1250
Wire Wire Line
	7800 1250 8150 1250
Connection ~ 8150 1250
NoConn ~ 8300 1850
NoConn ~ 8300 2700
NoConn ~ 9400 2900
NoConn ~ 9400 2800
NoConn ~ 9400 2700
NoConn ~ 9400 2600
NoConn ~ 9400 2500
NoConn ~ 9400 2400
NoConn ~ 9400 2300
NoConn ~ 9400 2200
NoConn ~ 9400 2100
NoConn ~ 9400 1900
NoConn ~ 9400 1600
NoConn ~ 9400 1350
Wire Wire Line
	8300 2400 7450 2400
Wire Wire Line
	8300 2900 7450 2900
Text Label 7450 2400 0    50   ~ 0
USART_TX
Text Label 7450 2900 0    50   ~ 0
USART_RX
Wire Wire Line
	4250 2050 4950 2050
Wire Wire Line
	4250 2300 4950 2300
Text Label 4950 2050 2    50   ~ 0
USART_TX
Text Label 4950 2300 2    50   ~ 0
USART_RX
$Comp
L Device:R R6
U 1 1 5D97C697
P 4100 2050
F 0 "R6" V 4000 2050 50  0000 R CNN
F 1 "100R" V 4000 2300 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 4030 2050 50  0001 C CNN
F 3 "~" H 4100 2050 50  0001 C CNN
	1    4100 2050
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5D97E2B0
P 4100 2300
F 0 "R7" V 4000 2300 50  0000 R CNN
F 1 "100R" V 4000 2550 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 4030 2300 50  0001 C CNN
F 3 "~" H 4100 2300 50  0001 C CNN
	1    4100 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	3700 2100 3700 2050
Wire Wire Line
	3700 2050 3950 2050
Wire Wire Line
	3700 2200 3700 2300
Wire Wire Line
	3700 2300 3950 2300
$Comp
L power:PWR_FLAG #FLG0105
U 1 1 5D995805
P 7100 1650
F 0 "#FLG0105" H 7100 1725 50  0001 C CNN
F 1 "PWR_FLAG" H 7100 1823 50  0000 C CNN
F 2 "" H 7100 1650 50  0001 C CNN
F 3 "~" H 7100 1650 50  0001 C CNN
	1    7100 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5DACCE16
P 8600 5650
F 0 "#PWR0119" H 8600 5400 50  0001 C CNN
F 1 "GND" H 8605 5477 50  0000 C CNN
F 2 "" H 8600 5650 50  0001 C CNN
F 3 "" H 8600 5650 50  0001 C CNN
	1    8600 5650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0121
U 1 1 5DADB64B
P 4100 4300
F 0 "#PWR0121" H 4100 4150 50  0001 C CNN
F 1 "+3.3V" H 4115 4473 50  0000 C CNN
F 2 "" H 4100 4300 50  0001 C CNN
F 3 "" H 4100 4300 50  0001 C CNN
	1    4100 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 5550 8600 5550
Wire Wire Line
	8600 5550 8600 5650
Text Label 10850 4500 2    50   ~ 0
10V_Rotation_VA
Text Label 10850 5400 2    50   ~ 0
10V_Inclinaison_Pos
Connection ~ 8900 5400
Wire Wire Line
	4200 5050 4000 5050
Wire Wire Line
	4000 5050 4000 5250
$Comp
L power:GND #PWR0124
U 1 1 5DB3E27B
P 4000 5250
F 0 "#PWR0124" H 4000 5000 50  0001 C CNN
F 1 "GND" H 4005 5077 50  0000 C CNN
F 2 "" H 4000 5250 50  0001 C CNN
F 3 "" H 4000 5250 50  0001 C CNN
	1    4000 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5DB52AAA
P 4150 6850
F 0 "C14" H 4265 6896 50  0000 L CNN
F 1 "100nF" H 4265 6805 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 4188 6700 50  0001 C CNN
F 3 "~" H 4150 6850 50  0001 C CNN
	1    4150 6850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 5DB52AB0
P 4150 7000
F 0 "#PWR0125" H 4150 6750 50  0001 C CNN
F 1 "GND" H 4155 6827 50  0000 C CNN
F 2 "" H 4150 7000 50  0001 C CNN
F 3 "" H 4150 7000 50  0001 C CNN
	1    4150 7000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0126
U 1 1 5DB56C55
P 4150 6700
F 0 "#PWR0126" H 4150 6550 50  0001 C CNN
F 1 "+3.3V" H 4165 6873 50  0000 C CNN
F 2 "" H 4150 6700 50  0001 C CNN
F 3 "" H 4150 6700 50  0001 C CNN
	1    4150 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4550 3600 4550
Wire Wire Line
	4200 4650 3600 4650
Wire Wire Line
	4200 4750 3600 4750
Text Label 3600 4550 0    50   ~ 0
DAC_DATA
Text Label 3600 4650 0    50   ~ 0
DAC_CLK
Text Label 3600 4750 0    50   ~ 0
~DAC_SYNC
Wire Wire Line
	8300 2500 7450 2500
Wire Wire Line
	8300 2600 7450 2600
Wire Wire Line
	8300 2800 7450 2800
NoConn ~ 9400 1700
NoConn ~ 9400 1800
Text Label 7450 2500 0    50   ~ 0
~DAC_SYNC
Text Label 7450 2600 0    50   ~ 0
DAC_CLK
Text Label 7450 2800 0    50   ~ 0
DAC_DATA
Wire Wire Line
	1200 1650 800  1650
Wire Wire Line
	1550 1650 1750 1650
Wire Wire Line
	1750 2350 850  2350
$Comp
L Device:L_Small L3
U 1 1 5D8E8365
P 5400 6800
F 0 "L3" V 5585 6800 50  0000 C CNN
F 1 "5.6µH" V 5494 6800 50  0000 C CNN
F 2 "Inductor_SMD:L_1812_4532Metric_Pad1.30x3.40mm_HandSolder" H 5400 6800 50  0001 C CNN
F 3 "~" H 5400 6800 50  0001 C CNN
F 4 "B82432-A1562-K" V 5400 6800 50  0001 C CNN "Ref"
	1    5400 6800
	0    1    -1   0   
$EndComp
$Comp
L Device:CP C13
U 1 1 5D8F204A
P 5700 6950
F 0 "C13" H 5818 6996 50  0000 L CNN
F 1 "10µF" H 5818 6905 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 5738 6800 50  0001 C CNN
F 3 "~" H 5700 6950 50  0001 C CNN
	1    5700 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 5D8F2050
P 5700 7150
F 0 "#PWR0122" H 5700 6900 50  0001 C CNN
F 1 "GND" H 5705 6977 50  0000 C CNN
F 2 "" H 5700 7150 50  0001 C CNN
F 3 "" H 5700 7150 50  0001 C CNN
	1    5700 7150
	1    0    0    -1  
$EndComp
Connection ~ 5700 6800
$Comp
L power:PWR_FLAG #FLG0106
U 1 1 5D8F2060
P 5700 6800
F 0 "#FLG0106" H 5700 6875 50  0001 C CNN
F 1 "PWR_FLAG" H 5700 6973 50  0000 C CNN
F 2 "" H 5700 6800 50  0001 C CNN
F 3 "~" H 5700 6800 50  0001 C CNN
	1    5700 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 6800 5500 6800
Wire Wire Line
	5300 6800 5000 6800
Wire Wire Line
	5700 7100 5700 7150
$Comp
L power:VDD #PWR0123
U 1 1 5D92B668
P 5000 6800
F 0 "#PWR0123" H 5000 6650 50  0001 C CNN
F 1 "VDD" H 5017 6973 50  0000 C CNN
F 2 "" H 5000 6800 50  0001 C CNN
F 3 "" H 5000 6800 50  0001 C CNN
	1    5000 6800
	-1   0    0    -1  
$EndComp
$Comp
L Jumper:Jumper_2_Bridged JP1
U 1 1 5D95B36C
P 2050 2350
F 0 "JP1" H 2050 2453 50  0000 C CNN
F 1 "Jumper_2_Bridged" H 2050 2454 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged2Bar_RoundedPad1.0x1.5mm" H 2050 2350 50  0001 C CNN
F 3 "~" H 2050 2350 50  0001 C CNN
	1    2050 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 2350 2300 2350
Connection ~ 2300 2350
Wire Wire Line
	2300 2400 2300 2350
Wire Wire Line
	1850 2350 1750 2350
Connection ~ 1750 2350
$Comp
L Insa:DAC7554 U2
U 1 1 5D934231
P 4600 4700
F 0 "U2" H 4600 5115 50  0000 C CNN
F 1 "DAC7554" H 4600 5024 50  0000 C CNN
F 2 "Package_SO:MSOP-10_3x3mm_P0.5mm" H 4600 4150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/dac7554.pdf" H 4600 4150 50  0001 C CNN
	1    4600 4700
	1    0    0    -1  
$EndComp
$Comp
L Insa:TS9224IDT U3
U 1 1 5D9349D4
P 8000 5000
F 0 "U3" H 8250 5900 50  0000 C CNN
F 1 "TS9224IDT" H 8100 5800 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 8000 4250 50  0001 C CNN
F 3 "https://www.mouser.fr/datasheet/2/389/ts9222-957356.pdf" H 8000 4250 50  0001 C CNN
	1    8000 5000
	1    0    0    -1  
$EndComp
Wire Notes Line
	3200 3500 3200 500 
Wire Notes Line
	5500 3500 5500 500 
Wire Notes Line
	500  3500 11200 3500
Wire Notes Line
	3500 3500 3500 7750
Text Notes 2300 3450 0    50   ~ 0
Filtrage alimentation
Text Notes 4950 3450 0    50   ~ 0
Connectique
Text Notes 10600 3450 0    50   ~ 0
Nucleo-L031
Text Notes 1550 3650 0    50   ~ 0
Entrées analogiques et mise à l'echelle (0-3,3V)
Text Notes 3550 3650 0    50   ~ 0
Sorties analogiques et mise à l'echelle (0-10V)
Wire Wire Line
	5700 6800 6050 6800
$Comp
L power:Vdrive #PWR0116
U 1 1 5DA309E1
P 6050 6800
F 0 "#PWR0116" H 5850 6650 50  0001 C CNN
F 1 "Vdrive" H 6067 6973 50  0000 C CNN
F 2 "" H 6050 6800 50  0001 C CNN
F 3 "" H 6050 6800 50  0001 C CNN
	1    6050 6800
	1    0    0    -1  
$EndComp
$Comp
L power:Vdrive #PWR0120
U 1 1 5DA3157A
P 8600 4350
F 0 "#PWR0120" H 8400 4200 50  0001 C CNN
F 1 "Vdrive" H 8617 4523 50  0000 C CNN
F 2 "" H 8600 4350 50  0001 C CNN
F 3 "" H 8600 4350 50  0001 C CNN
	1    8600 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 4350 8600 4350
Wire Wire Line
	4200 4950 4100 4950
Wire Wire Line
	4100 4300 4100 4950
$Comp
L power:+3.3V #PWR0127
U 1 1 5DACCF25
P 5050 4250
F 0 "#PWR0127" H 5050 4100 50  0001 C CNN
F 1 "+3.3V" H 5065 4423 50  0000 C CNN
F 2 "" H 5050 4250 50  0001 C CNN
F 3 "" H 5050 4250 50  0001 C CNN
	1    5050 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4250 5050 4550
Wire Wire Line
	5050 4550 5000 4550
$Comp
L Device:C C15
U 1 1 5DAEB594
P 5550 5350
F 0 "C15" H 5350 5450 50  0000 L CNN
F 1 "22nF" V 5500 4950 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 5588 5200 50  0001 C CNN
F 3 "~" H 5550 5350 50  0001 C CNN
	1    5550 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C16
U 1 1 5DAEF9B4
P 5800 5350
F 0 "C16" H 5650 5450 50  0000 L CNN
F 1 "22nF" V 5750 4950 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 5838 5200 50  0001 C CNN
F 3 "~" H 5800 5350 50  0001 C CNN
	1    5800 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C17
U 1 1 5DAF3D7B
P 6050 5350
F 0 "C17" H 5900 5450 50  0000 L CNN
F 1 "22nF" V 6150 4950 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 6088 5200 50  0001 C CNN
F 3 "~" H 6050 5350 50  0001 C CNN
	1    6050 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C18
U 1 1 5DAF8310
P 6300 5350
F 0 "C18" H 6150 5450 50  0000 L CNN
F 1 "22nF" V 6350 4950 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 6338 5200 50  0001 C CNN
F 3 "~" H 6300 5350 50  0001 C CNN
	1    6300 5350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 5DB056FB
P 5950 5550
F 0 "#PWR0128" H 5950 5300 50  0001 C CNN
F 1 "GND" H 5955 5377 50  0000 C CNN
F 2 "" H 5950 5550 50  0001 C CNN
F 3 "" H 5950 5550 50  0001 C CNN
	1    5950 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 5500 5800 5500
Connection ~ 5800 5500
Connection ~ 6050 5500
Wire Wire Line
	6050 5500 6300 5500
Wire Wire Line
	5800 5500 5950 5500
Wire Wire Line
	5000 4750 5100 4750
Wire Wire Line
	5100 4850 5000 4850
Wire Wire Line
	5000 4950 5100 4950
Wire Wire Line
	5100 5050 5000 5050
Wire Wire Line
	5550 5200 5550 5050
Wire Wire Line
	5550 5050 5500 5050
Wire Wire Line
	5800 5200 5800 4950
Wire Wire Line
	5800 4950 5500 4950
Wire Wire Line
	6050 5200 6050 4850
Wire Wire Line
	6050 4850 5500 4850
Wire Wire Line
	6300 4750 6300 5200
Wire Wire Line
	5500 4750 6300 4750
Wire Wire Line
	5950 5500 5950 5550
Connection ~ 5950 5500
Wire Wire Line
	5950 5500 6050 5500
Wire Wire Line
	7550 5350 7300 5350
Wire Wire Line
	8900 5400 8900 6250
Wire Wire Line
	7300 6250 7300 5350
Connection ~ 7300 5350
Wire Wire Line
	8450 5400 8900 5400
Wire Wire Line
	9050 5100 8850 5100
Wire Wire Line
	9800 4800 10850 4800
Wire Wire Line
	9800 5100 10850 5100
Connection ~ 7400 4750
Wire Wire Line
	7400 4750 7550 4750
Text Label 10850 4800 2    50   ~ 0
10V_Inclinaison_VA
Text Label 10850 5100 2    50   ~ 0
10V_Rotation_Pos
$Comp
L power:GND #PWR0129
U 1 1 5DC9A24D
P 6900 6000
F 0 "#PWR0129" H 6900 5750 50  0001 C CNN
F 1 "GND" H 6905 5827 50  0000 C CNN
F 2 "" H 6900 6000 50  0001 C CNN
F 3 "" H 6900 6000 50  0001 C CNN
	1    6900 6000
	1    0    0    -1  
$EndComp
Connection ~ 6300 4750
NoConn ~ 9400 2000
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 5DD43735
P 3500 1500
F 0 "J5" H 3418 1625 50  0000 C CNN
F 1 "Conn_01x02" H 3580 1401 50  0001 L CNN
F 2 "Insa:DF3A-2P-2DS" H 3500 1500 50  0001 C CNN
F 3 "~" H 3500 1500 50  0001 C CNN
	1    3500 1500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3700 750  5100 750 
Wire Wire Line
	3700 850  5100 850 
Wire Wire Line
	3700 950  5100 950 
Wire Wire Line
	3700 1050 5100 1050
Wire Wire Line
	3700 1500 5100 1500
Wire Wire Line
	3700 1600 5100 1600
Text Label 5100 750  2    50   ~ 0
10V_Rotation_VA
Text Label 5100 850  2    50   ~ 0
10V_Inclinaison_VA
Text Label 5100 950  2    50   ~ 0
10V_Rotation_Pos
Text Label 5100 1050 2    50   ~ 0
10V_Inclinaison_Pos
$Comp
L Device:R_Pack04 RN1
U 1 1 5D93A076
P 5300 4950
F 0 "RN1" V 4883 4950 50  0000 C CNN
F 1 "1K" V 4974 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Convex_4x0603" V 5575 4950 50  0001 C CNN
F 3 "~" H 5300 4950 50  0001 C CNN
F 4 "CAY16-102J4AS" V 5300 4950 50  0001 C CNN "Ref"
	1    5300 4950
	0    1    1    0   
$EndComp
$Comp
L Device:R_Pack04 RN2
U 1 1 5D979821
P 9400 5000
F 0 "RN2" V 8983 5000 50  0000 C CNN
F 1 "100" V 9074 5000 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Convex_4x0603" V 9675 5000 50  0001 C CNN
F 3 "~" H 9400 5000 50  0001 C CNN
F 4 "CAY16-101J4AS" V 9400 5000 50  0001 C CNN "Ref"
	1    9400 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	9050 4800 9050 4900
Wire Wire Line
	9050 4900 9200 4900
Wire Wire Line
	9050 5100 9050 5000
Wire Wire Line
	9050 5000 9200 5000
Wire Wire Line
	9200 5400 9200 5100
Wire Wire Line
	8900 5400 9200 5400
Wire Wire Line
	9200 4800 9200 4500
Wire Wire Line
	9800 4800 9800 4900
Wire Wire Line
	9800 4900 9600 4900
Wire Wire Line
	9600 5000 9800 5000
Wire Wire Line
	9800 5000 9800 5100
Wire Wire Line
	9600 5100 9600 5400
Wire Wire Line
	9600 5400 10850 5400
Wire Wire Line
	9600 4800 9600 4500
Wire Wire Line
	9600 4500 10850 4500
Wire Wire Line
	7200 5350 7300 5350
Wire Wire Line
	5550 5050 6450 5050
Wire Wire Line
	6450 5450 7550 5450
Connection ~ 5550 5050
Wire Wire Line
	7550 5150 6500 5150
Wire Wire Line
	6500 5150 6500 4950
Wire Wire Line
	6500 4950 5800 4950
Connection ~ 5800 4950
$Comp
L Device:R_Pack04 RN3
U 1 1 5DAA86DF
P 7100 5750
F 0 "RN3" H 6650 5850 50  0000 L CNN
F 1 "1K" H 6700 5750 50  0000 L CNN
F 2 "Resistor_SMD:R_Array_Convex_4x0603" V 7375 5750 50  0001 C CNN
F 3 "~" H 7100 5750 50  0001 C CNN
F 4 "CAY16-102J4AS" V 7100 5750 50  0001 C CNN "Ref"
	1    7100 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 5350 7200 5550
Wire Wire Line
	7100 5050 7100 5550
Wire Wire Line
	7000 4750 7000 5550
Wire Wire Line
	7000 4750 7400 4750
Wire Wire Line
	6900 4450 6900 5550
Wire Wire Line
	7200 5950 7100 5950
Connection ~ 7000 5950
Wire Wire Line
	7000 5950 6900 5950
Connection ~ 7100 5950
Wire Wire Line
	7100 5950 7000 5950
Wire Wire Line
	6900 5950 6900 6000
Connection ~ 6900 5950
Connection ~ 6050 4850
Wire Wire Line
	6450 5050 6450 5450
Wire Wire Line
	6050 4850 7550 4850
Wire Wire Line
	7550 4550 6300 4550
Wire Wire Line
	6300 4550 6300 4750
$Comp
L Device:R_Pack04 RN4
U 1 1 5DB5B6AE
P 8000 6150
F 0 "RN4" V 8200 6150 50  0000 C CNN
F 1 "2.4K" V 8300 6150 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Convex_4x0603" V 8275 6150 50  0001 C CNN
F 3 "~" H 8000 6150 50  0001 C CNN
F 4 "YC164-FR-072K4L" V 8000 6150 50  0001 C CNN "Ref"
	1    8000 6150
	0    1    1    0   
$EndComp
Wire Wire Line
	7100 5050 7350 5050
Wire Wire Line
	7300 6250 7800 6250
Wire Wire Line
	8900 6250 8200 6250
Wire Wire Line
	7800 6150 7350 6150
Wire Wire Line
	7350 6150 7350 5050
Connection ~ 7350 5050
Wire Wire Line
	7350 5050 7550 5050
Wire Wire Line
	8200 6150 8850 6150
Wire Wire Line
	8850 6150 8850 5100
Connection ~ 8850 5100
Wire Wire Line
	8450 5100 8850 5100
Wire Wire Line
	8800 4800 8800 6050
Wire Wire Line
	8800 6050 8200 6050
Connection ~ 8800 4800
Wire Wire Line
	8800 4800 9050 4800
Wire Wire Line
	7800 6050 7400 6050
Wire Wire Line
	7400 6050 7400 4750
Wire Wire Line
	8450 4800 8800 4800
Wire Wire Line
	6900 4450 7450 4450
Wire Wire Line
	8450 4500 8750 4500
Wire Wire Line
	8750 4500 8750 5950
Wire Wire Line
	8750 5950 8200 5950
Connection ~ 8750 4500
Wire Wire Line
	8750 4500 9200 4500
Wire Wire Line
	7800 5950 7450 5950
Wire Wire Line
	7450 5950 7450 4450
Connection ~ 7450 4450
Wire Wire Line
	7450 4450 7550 4450
$Comp
L power:+12V #PWR0130
U 1 1 5D9C510B
P 4000 3000
F 0 "#PWR0130" H 4000 2850 50  0001 C CNN
F 1 "+12V" H 4015 3173 50  0000 C CNN
F 2 "" H 4000 3000 50  0001 C CNN
F 3 "" H 4000 3000 50  0001 C CNN
	1    4000 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3050 4000 3050
Wire Wire Line
	4000 3050 4000 3000
$Comp
L carte_base-rescue:D_Schottky_AAK-Device D1
U 1 1 5D9FB932
P 1400 1650
F 0 "D1" H 1425 1517 50  0000 C CNN
F 1 "D_Schottky_AAK" H 1425 1516 50  0001 C CNN
F 2 "Package_TO_SOT_SMD:TO-277A" H 1400 1650 50  0001 C CNN
F 3 "~" H 1400 1650 50  0001 C CNN
F 4 "SS10P4C-M3/86A" H 1400 1650 50  0001 C CNN "Ref"
	1    1400 1650
	-1   0    0    1   
$EndComp
Wire Wire Line
	1200 1750 1200 1650
Connection ~ 1200 1650
$EndSCHEMATC

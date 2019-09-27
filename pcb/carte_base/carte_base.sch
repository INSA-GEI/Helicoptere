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
F 0 "H1" H 1050 905 50  0000 L CNN
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
F 0 "H2" H 1400 905 50  0000 L CNN
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
F 0 "H3" H 1750 905 50  0000 L CNN
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
F 0 "H4" H 2100 905 50  0000 L CNN
F 1 "MountingHole_Pad" H 2100 860 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_ISO14580_Pad" H 2000 900 50  0001 C CNN
F 3 "~" H 2000 900 50  0001 C CNN
	1    2000 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5D2F318B
P 4350 1850
F 0 "#PWR0102" H 4350 1600 50  0001 C CNN
F 1 "GND" H 4355 1677 50  0000 C CNN
F 2 "" H 4350 1850 50  0001 C CNN
F 3 "" H 4350 1850 50  0001 C CNN
	1    4350 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 5D2F3A22
P 3800 1450
F 0 "C1" H 3918 1496 50  0000 L CNN
F 1 "10µF" H 3918 1405 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 3838 1300 50  0001 C CNN
F 3 "~" H 3800 1450 50  0001 C CNN
	1    3800 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1100 3800 1300
Connection ~ 3800 1100
Wire Wire Line
	3800 1600 3800 1800
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
P 8250 2200
F 0 "U1" H 8200 3265 50  0000 C CNN
F 1 "STM32L031" H 8200 3174 50  0000 C CNN
F 2 "Insa:STM32-Nucleo-32" H 8250 3200 50  0001 C CNN
F 3 "" H 8250 3200 50  0001 C CNN
	1    8250 2200
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
P 9350 1200
F 0 "#PWR0110" H 9350 1050 50  0001 C CNN
F 1 "+3.3V" H 9365 1373 50  0000 C CNN
F 2 "" H 9350 1200 50  0001 C CNN
F 3 "" H 9350 1200 50  0001 C CNN
	1    9350 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0111
U 1 1 5D3132CF
P 5850 1750
F 0 "#PWR0111" H 5850 1600 50  0001 C CNN
F 1 "+3.3V" H 5865 1923 50  0000 C CNN
F 2 "" H 5850 1750 50  0001 C CNN
F 3 "" H 5850 1750 50  0001 C CNN
	1    5850 1750
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
	8850 1350 9000 1350
Wire Wire Line
	9000 1350 9000 1200
Wire Wire Line
	7750 1450 7750 1350
Wire Wire Line
	7750 1350 7600 1350
Connection ~ 7750 1350
$Comp
L power:GND #PWR0112
U 1 1 5D33A410
P 7600 1350
F 0 "#PWR0112" H 7600 1100 50  0001 C CNN
F 1 "GND" H 7605 1177 50  0000 C CNN
F 2 "" H 7600 1350 50  0001 C CNN
F 3 "" H 7600 1350 50  0001 C CNN
	1    7600 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 2300 6900 2300
Wire Wire Line
	7750 2400 6900 2400
Text Label 6900 2300 0    50   ~ 0
Consigne_3V3_Avant
Text Label 6900 2400 0    50   ~ 0
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
	9350 1550 9350 1200
Wire Wire Line
	8850 1550 9350 1550
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
	3100 2200 3800 2200
Wire Wire Line
	3100 2300 3800 2300
Wire Wire Line
	1150 2350 1850 2350
Wire Wire Line
	1150 2450 1850 2450
Wire Wire Line
	1150 2650 1850 2650
Wire Wire Line
	1150 2550 1850 2550
Text Label 3800 2200 2    50   ~ 0
Alim_12V
Text Label 3800 2300 2    50   ~ 0
Alim_GND
Text Label 1400 2350 0    50   ~ 0
Consigne_12V_Avant
Text Label 1400 2450 0    50   ~ 0
Consigne_12V_Arriere
Text Label 1400 2550 0    50   ~ 0
Analog_12V_Rotation
Text Label 1400 2650 0    50   ~ 0
Analog_12V_Inclinaison
$Comp
L Device:L L1
U 1 1 5D325DDF
P 4100 1100
F 0 "L1" V 4290 1100 50  0000 C CNN
F 1 "100µH" V 4199 1100 50  0000 C CNN
F 2 "Inductor_SMD:L_Bourns-SRN6028" H 4100 1100 50  0001 C CNN
F 3 "~" H 4100 1100 50  0001 C CNN
F 4 "Bourns SRR6028-102Y" V 4100 1100 50  0001 C CNN "Ref"
	1    4100 1100
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C2
U 1 1 5D325FE2
P 4350 1450
F 0 "C2" H 4465 1496 50  0000 L CNN
F 1 "100nF" H 4465 1405 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 4388 1300 50  0001 C CNN
F 3 "~" H 4350 1450 50  0001 C CNN
	1    4350 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1100 3950 1100
Wire Wire Line
	4250 1100 4350 1100
Wire Wire Line
	4350 1100 4350 1300
Wire Wire Line
	4350 1600 4350 1800
$Comp
L power:VDD #PWR0117
U 1 1 5D334033
P 4350 1100
F 0 "#PWR0117" H 4350 950 50  0001 C CNN
F 1 "VDD" H 4367 1273 50  0000 C CNN
F 2 "" H 4350 1100 50  0001 C CNN
F 3 "" H 4350 1100 50  0001 C CNN
	1    4350 1100
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0118
U 1 1 5D334239
P 9000 1200
F 0 "#PWR0118" H 9000 1050 50  0001 C CNN
F 1 "VDD" H 9017 1373 50  0000 C CNN
F 2 "" H 9000 1200 50  0001 C CNN
F 3 "" H 9000 1200 50  0001 C CNN
	1    9000 1200
	1    0    0    -1  
$EndComp
Connection ~ 4350 1100
Text Label 2900 1100 0    50   ~ 0
Alim_12V
Text Label 2900 1800 0    50   ~ 0
Alim_GND
Wire Wire Line
	750  4650 1500 4650
Wire Wire Line
	700  6200 1500 6200
Text Label 750  4650 0    50   ~ 0
Consigne_12V_Avant
Text Label 700  6200 0    50   ~ 0
Consigne_12V_Arriere
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 5D8C0337
P 2900 2200
F 0 "J2" H 2818 2325 50  0000 C CNN
F 1 "Conn_01x02" H 2980 2101 50  0001 L CNN
F 2 "Connector_JST:JST_VH_B2PS-VH_1x02_P3.96mm_Horizontal" H 2900 2200 50  0001 C CNN
F 3 "~" H 2900 2200 50  0001 C CNN
	1    2900 2200
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 5D8C0A62
P 5000 2200
F 0 "J3" H 5080 2146 50  0000 L CNN
F 1 "Conn_01x02" H 5080 2101 50  0001 L CNN
F 2 "Connector_JST:JST_VH_B2PS-VH_1x02_P3.96mm_Horizontal" H 5000 2200 50  0001 C CNN
F 3 "~" H 5000 2200 50  0001 C CNN
	1    5000 2200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 5D8C1354
P 950 3400
F 0 "J4" H 868 3525 50  0000 C CNN
F 1 "Conn_01x02" H 1030 3301 50  0001 L CNN
F 2 "Insa:DF3A-2P-2DS" H 950 3400 50  0001 C CNN
F 3 "~" H 950 3400 50  0001 C CNN
	1    950  3400
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5D8D12A6
P 950 2450
F 0 "J1" H 868 2675 50  0000 C CNN
F 1 "Conn_01x04" H 868 2116 50  0001 C CNN
F 2 "Insa:DF3A-4P-2DS" H 950 2450 50  0001 C CNN
F 3 "~" H 950 2450 50  0001 C CNN
	1    950  2450
	-1   0    0    -1  
$EndComp
$Comp
L Device:D D1
U 1 1 5D8D967B
P 3450 1100
F 0 "D1" H 3450 976 50  0000 C CNN
F 1 "D" H 3450 975 50  0001 C CNN
F 2 "Diode_SMD:D_SOD-123" H 3450 1100 50  0001 C CNN
F 3 "~" H 3450 1100 50  0001 C CNN
	1    3450 1100
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C3
U 1 1 5D8DE5DD
P 4850 1450
F 0 "C3" H 4968 1496 50  0000 L CNN
F 1 "15µF" H 4968 1405 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 4888 1300 50  0001 C CNN
F 3 "~" H 4850 1450 50  0001 C CNN
	1    4850 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 1600 4850 1800
Wire Wire Line
	4350 1800 4850 1800
Wire Wire Line
	4850 1300 4850 1100
Wire Wire Line
	4850 1100 4350 1100
$Comp
L power:+12V #PWR0101
U 1 1 5D2F3144
P 3800 1100
F 0 "#PWR0101" H 3800 950 50  0001 C CNN
F 1 "+12V" H 3815 1273 50  0000 C CNN
F 2 "" H 3800 1100 50  0001 C CNN
F 3 "" H 3800 1100 50  0001 C CNN
	1    3800 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 2200 4800 2200
Text Label 4150 2200 0    50   ~ 0
Alim_12V
Wire Wire Line
	4800 2300 4150 2300
Text Label 4150 2300 0    50   ~ 0
Alim_GND
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5D8EAA98
P 10600 950
F 0 "#FLG0101" H 10600 1025 50  0001 C CNN
F 1 "PWR_FLAG" H 10600 1123 50  0000 C CNN
F 2 "" H 10600 950 50  0001 C CNN
F 3 "~" H 10600 950 50  0001 C CNN
	1    10600 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0104
U 1 1 5D8EB676
P 10250 950
F 0 "#PWR0104" H 10250 800 50  0001 C CNN
F 1 "+12V" H 10265 1123 50  0000 C CNN
F 2 "" H 10250 950 50  0001 C CNN
F 3 "" H 10250 950 50  0001 C CNN
	1    10250 950 
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0105
U 1 1 5D8ECF07
P 10250 1300
F 0 "#PWR0105" H 10250 1150 50  0001 C CNN
F 1 "VDD" H 10267 1473 50  0000 C CNN
F 2 "" H 10250 1300 50  0001 C CNN
F 3 "" H 10250 1300 50  0001 C CNN
	1    10250 1300
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5D8EE571
P 10600 1300
F 0 "#FLG0102" H 10600 1375 50  0001 C CNN
F 1 "PWR_FLAG" H 10600 1473 50  0000 C CNN
F 2 "" H 10600 1300 50  0001 C CNN
F 3 "~" H 10600 1300 50  0001 C CNN
	1    10600 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5D8F6FE1
P 10250 1600
F 0 "#PWR0113" H 10250 1350 50  0001 C CNN
F 1 "GND" H 10255 1427 50  0000 C CNN
F 2 "" H 10250 1600 50  0001 C CNN
F 3 "" H 10250 1600 50  0001 C CNN
	1    10250 1600
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5D8F86D5
P 10600 1600
F 0 "#FLG0103" H 10600 1675 50  0001 C CNN
F 1 "PWR_FLAG" H 10600 1773 50  0000 C CNN
F 2 "" H 10600 1600 50  0001 C CNN
F 3 "~" H 10600 1600 50  0001 C CNN
	1    10600 1600
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR0114
U 1 1 5D8F9D22
P 10250 2150
F 0 "#PWR0114" H 10250 2000 50  0001 C CNN
F 1 "+3.3V" H 10265 2323 50  0000 C CNN
F 2 "" H 10250 2150 50  0001 C CNN
F 3 "" H 10250 2150 50  0001 C CNN
	1    10250 2150
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0104
U 1 1 5D8FB40F
P 10600 2150
F 0 "#FLG0104" H 10600 2225 50  0001 C CNN
F 1 "PWR_FLAG" H 10600 2323 50  0000 C CNN
F 2 "" H 10600 2150 50  0001 C CNN
F 3 "~" H 10600 2150 50  0001 C CNN
	1    10600 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 950  10600 950 
Wire Wire Line
	10250 1300 10600 1300
Wire Wire Line
	10250 1600 10600 1600
Wire Wire Line
	10250 2150 10600 2150
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
P 6250 1750
F 0 "L2" V 6440 1750 50  0000 C CNN
F 1 "10mH" V 6349 1750 50  0000 C CNN
F 2 "Inductor_SMD:L_Wuerth_WE-PD2-Typ-L" H 6250 1750 50  0001 C CNN
F 3 "~" H 6250 1750 50  0001 C CNN
F 4 "WE 74404086103" V 6250 1750 50  0001 C CNN "Ref"
	1    6250 1750
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP C7
U 1 1 5D92E6CC
P 6000 1900
F 0 "C7" H 6118 1946 50  0000 L CNN
F 1 "10µF" H 6118 1855 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 6038 1750 50  0001 C CNN
F 3 "~" H 6000 1900 50  0001 C CNN
	1    6000 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C8
U 1 1 5D930210
P 6550 1900
F 0 "C8" H 6668 1946 50  0000 L CNN
F 1 "10µF" H 6668 1855 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 6588 1750 50  0001 C CNN
F 3 "~" H 6550 1900 50  0001 C CNN
	1    6550 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5D935ECA
P 6250 2100
F 0 "#PWR0115" H 6250 1850 50  0001 C CNN
F 1 "GND" H 6255 1927 50  0000 C CNN
F 2 "" H 6250 2100 50  0001 C CNN
F 3 "" H 6250 2100 50  0001 C CNN
	1    6250 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 1750 6000 1750
Wire Wire Line
	6000 1750 6100 1750
Connection ~ 6000 1750
Wire Wire Line
	6400 1750 6550 1750
Wire Wire Line
	6000 2050 6250 2050
Wire Wire Line
	6250 2050 6250 2100
Connection ~ 6250 2050
Wire Wire Line
	6250 2050 6550 2050
Wire Wire Line
	6550 1750 7750 1750
Connection ~ 6550 1750
$Comp
L Switch:SW_Push SW1
U 1 1 5D948B58
P 7450 2050
F 0 "SW1" H 7450 2243 50  0000 C CNN
F 1 "FSMRA3JH04" H 7450 2244 50  0001 C CNN
F 2 "Button_Switch_THT:SW_Tactile_SPST_Angled_PTS645Vx58-2LFS" H 7450 2250 50  0001 C CNN
F 3 "~" H 7450 2250 50  0001 C CNN
	1    7450 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 2050 7750 2050
Wire Wire Line
	7250 2050 7250 1350
Wire Wire Line
	7250 1350 7600 1350
Connection ~ 7600 1350
NoConn ~ 7750 1950
NoConn ~ 7750 2800
NoConn ~ 8850 3000
NoConn ~ 8850 2900
NoConn ~ 8850 2800
NoConn ~ 8850 2700
NoConn ~ 8850 2600
NoConn ~ 8850 2500
NoConn ~ 8850 2400
NoConn ~ 8850 2300
NoConn ~ 8850 2200
NoConn ~ 8850 2000
NoConn ~ 8850 1700
NoConn ~ 8850 1450
Wire Wire Line
	7750 2500 6900 2500
Wire Wire Line
	7750 3000 6900 3000
Text Label 6900 2500 0    50   ~ 0
USART_TX
Text Label 6900 3000 0    50   ~ 0
USART_RX
Wire Wire Line
	1700 3350 2400 3350
Wire Wire Line
	1700 3600 2400 3600
Text Label 2400 3350 2    50   ~ 0
USART_TX
Text Label 2400 3600 2    50   ~ 0
USART_RX
$Comp
L Device:R R6
U 1 1 5D97C697
P 1550 3350
F 0 "R6" V 1450 3350 50  0000 R CNN
F 1 "100R" V 1450 3600 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 1480 3350 50  0001 C CNN
F 3 "~" H 1550 3350 50  0001 C CNN
	1    1550 3350
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 5D97E2B0
P 1550 3600
F 0 "R7" V 1450 3600 50  0000 R CNN
F 1 "100R" V 1450 3850 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 1480 3600 50  0001 C CNN
F 3 "~" H 1550 3600 50  0001 C CNN
	1    1550 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	1150 3400 1150 3350
Wire Wire Line
	1150 3350 1400 3350
Wire Wire Line
	1150 3500 1150 3600
Wire Wire Line
	1150 3600 1400 3600
$Comp
L power:PWR_FLAG #FLG0105
U 1 1 5D995805
P 6550 1750
F 0 "#FLG0105" H 6550 1825 50  0001 C CNN
F 1 "PWR_FLAG" H 6550 1923 50  0000 C CNN
F 2 "" H 6550 1750 50  0001 C CNN
F 3 "~" H 6550 1750 50  0001 C CNN
	1    6550 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5D9D1CA0
P 6900 5850
F 0 "#PWR0116" H 6900 5600 50  0001 C CNN
F 1 "GND" H 6905 5677 50  0000 C CNN
F 2 "" H 6900 5850 50  0001 C CNN
F 3 "" H 6900 5850 50  0001 C CNN
	1    6900 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 5D9E0656
P 8600 5350
F 0 "R8" V 8700 5400 50  0000 R CNN
F 1 "1K" V 8500 5400 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8530 5350 50  0001 C CNN
F 3 "~" H 8600 5350 50  0001 C CNN
	1    8600 5350
	0    -1   -1   0   
$EndComp
$Comp
L Insa:DAC7563T U2
U 1 1 5DABAEC3
P 5700 5400
F 0 "U2" H 5700 5815 50  0000 C CNN
F 1 "DAC7563T" H 5700 5724 50  0000 C CNN
F 2 "Package_SO:VSSOP-10_3x3mm_P0.5mm" H 5700 4850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/dac7563t.pdf" H 5700 4850 50  0001 C CNN
	1    5700 5400
	1    0    0    -1  
$EndComp
$Comp
L Insa:TS9222IDT U3
U 1 1 5DABB8A0
P 7800 5550
F 0 "U3" H 7800 6165 50  0000 C CNN
F 1 "TS9222IDT" H 7800 6074 50  0000 C CNN
F 2 "Package_SO:SO-8_3.9x4.9mm_P1.27mm" H 7800 5100 50  0001 C CNN
F 3 "https://www.mouser.fr/datasheet/2/389/ts9222-957356.pdf" H 7800 5100 50  0001 C CNN
	1    7800 5550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5DACCE16
P 8400 5900
F 0 "#PWR0119" H 8400 5650 50  0001 C CNN
F 1 "GND" H 8405 5727 50  0000 C CNN
F 2 "" H 8400 5900 50  0001 C CNN
F 3 "" H 8400 5900 50  0001 C CNN
	1    8400 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5DACF563
P 6200 5850
F 0 "#PWR0120" H 6200 5600 50  0001 C CNN
F 1 "GND" H 6205 5677 50  0000 C CNN
F 2 "" H 6200 5850 50  0001 C CNN
F 3 "" H 6200 5850 50  0001 C CNN
	1    6200 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5DAD1B9C
P 8600 5650
F 0 "R9" V 8700 5700 50  0000 R CNN
F 1 "1K" V 8500 5700 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 8530 5650 50  0001 C CNN
F 3 "~" H 8600 5650 50  0001 C CNN
	1    8600 5650
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0121
U 1 1 5DADB64B
P 6200 5100
F 0 "#PWR0121" H 6200 4950 50  0001 C CNN
F 1 "+3.3V" H 6215 5273 50  0000 C CNN
F 2 "" H 6200 5100 50  0001 C CNN
F 3 "" H 6200 5100 50  0001 C CNN
	1    6200 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 5200 8400 5200
Wire Wire Line
	8250 5800 8400 5800
Wire Wire Line
	8400 5800 8400 5900
Wire Wire Line
	6100 5750 6200 5750
Wire Wire Line
	6200 5750 6200 5850
Wire Wire Line
	6100 5250 6200 5250
Wire Wire Line
	6200 5250 6200 5100
Wire Wire Line
	8250 5350 8300 5350
Wire Wire Line
	8250 5650 8300 5650
Text Label 9800 5350 2    50   ~ 0
Analog_12V_Rotation
Wire Wire Line
	8750 5350 9800 5350
Wire Wire Line
	8750 5650 9800 5650
Text Label 9800 5650 2    50   ~ 0
Analog_12V_Inclinaison
$Comp
L Device:R R10
U 1 1 5DB08633
P 7050 5300
F 0 "R10" V 7150 5350 50  0000 R CNN
F 1 "1K" V 6950 5350 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 6980 5300 50  0001 C CNN
F 3 "~" H 7050 5300 50  0001 C CNN
	1    7050 5300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R12
U 1 1 5DB0B0BE
P 7050 5600
F 0 "R12" V 7150 5650 50  0000 R CNN
F 1 "1K" V 6950 5650 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 6980 5600 50  0001 C CNN
F 3 "~" H 7050 5600 50  0001 C CNN
	1    7050 5600
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R11
U 1 1 5DB0DE5B
P 7800 4700
F 0 "R11" V 7900 4750 50  0000 R CNN
F 1 "1K" V 7700 4750 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7730 4700 50  0001 C CNN
F 3 "~" H 7800 4700 50  0001 C CNN
	1    7800 4700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R13
U 1 1 5DB10BD4
P 7800 6200
F 0 "R13" V 7900 6250 50  0000 R CNN
F 1 "1K" V 7700 6250 50  0000 R CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 7730 6200 50  0001 C CNN
F 3 "~" H 7800 6200 50  0001 C CNN
	1    7800 6200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6100 5500 6750 5500
Wire Wire Line
	6750 5500 6750 5400
Wire Wire Line
	6750 5400 7350 5400
Wire Wire Line
	6100 5600 6750 5600
Wire Wire Line
	6750 5600 6750 5700
Wire Wire Line
	6750 5700 7300 5700
Wire Wire Line
	6900 5300 6900 5600
Wire Wire Line
	6900 5600 6900 5850
Connection ~ 6900 5600
Wire Wire Line
	7200 5300 7300 5300
Wire Wire Line
	7200 5600 7350 5600
Wire Wire Line
	7300 5300 7300 4700
Wire Wire Line
	7300 4700 7650 4700
Connection ~ 7300 5300
Wire Wire Line
	7300 5300 7350 5300
Wire Wire Line
	7950 4700 8300 4700
Wire Wire Line
	8300 4700 8300 5350
Connection ~ 8300 5350
Wire Wire Line
	8300 5350 8450 5350
Wire Wire Line
	7300 5700 7300 6200
Wire Wire Line
	7300 6200 7650 6200
Connection ~ 7300 5700
Wire Wire Line
	7300 5700 7350 5700
Wire Wire Line
	7950 6200 8300 6200
Wire Wire Line
	8300 6200 8300 5650
Connection ~ 8300 5650
Wire Wire Line
	8300 5650 8450 5650
Wire Wire Line
	5300 5650 5100 5650
Wire Wire Line
	5100 5650 5100 5850
$Comp
L power:GND #PWR0124
U 1 1 5DB3E27B
P 5100 5850
F 0 "#PWR0124" H 5100 5600 50  0001 C CNN
F 1 "GND" H 5105 5677 50  0000 C CNN
F 2 "" H 5100 5850 50  0001 C CNN
F 3 "" H 5100 5850 50  0001 C CNN
	1    5100 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5DB52AAA
P 6600 4900
F 0 "C14" H 6715 4946 50  0000 L CNN
F 1 "100nF" H 6715 4855 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 6638 4750 50  0001 C CNN
F 3 "~" H 6600 4900 50  0001 C CNN
	1    6600 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 5DB52AB0
P 6600 5050
F 0 "#PWR0125" H 6600 4800 50  0001 C CNN
F 1 "GND" H 6605 4877 50  0000 C CNN
F 2 "" H 6600 5050 50  0001 C CNN
F 3 "" H 6600 5050 50  0001 C CNN
	1    6600 5050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0126
U 1 1 5DB56C55
P 6600 4750
F 0 "#PWR0126" H 6600 4600 50  0001 C CNN
F 1 "+3.3V" H 6615 4923 50  0000 C CNN
F 2 "" H 6600 4750 50  0001 C CNN
F 3 "" H 6600 4750 50  0001 C CNN
	1    6600 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5250 4700 5250
Wire Wire Line
	5300 5350 4700 5350
Wire Wire Line
	5300 5500 4700 5500
Wire Wire Line
	5300 5750 4700 5750
Text Label 4700 5250 0    50   ~ 0
DAC_DATA
Text Label 4700 5350 0    50   ~ 0
DAC_CLK
Text Label 4700 5500 0    50   ~ 0
~DAC_CLEAR
Text Label 4700 5750 0    50   ~ 0
~DAC_SYNC
NoConn ~ 6100 5350
Wire Wire Line
	7750 2600 6900 2600
Wire Wire Line
	7750 2700 6900 2700
Wire Wire Line
	7750 2900 6900 2900
NoConn ~ 8850 1800
NoConn ~ 8850 1900
Text Label 6900 2600 0    50   ~ 0
~DAC_SYNC
Text Label 6900 2700 0    50   ~ 0
DAC_CLK
Text Label 6900 2900 0    50   ~ 0
DAC_DATA
Wire Wire Line
	8850 2100 9550 2100
Text Label 9550 2100 2    50   ~ 0
~DAC_CLEAR
Wire Wire Line
	3300 1100 2900 1100
Wire Wire Line
	3600 1100 3800 1100
Wire Wire Line
	3800 1800 2900 1800
$Comp
L Device:L_Small L3
U 1 1 5D8E8365
P 10500 5000
F 0 "L3" V 10685 5000 50  0000 C CNN
F 1 "5.6µH" V 10594 5000 50  0000 C CNN
F 2 "Inductor_SMD:L_1812_4532Metric_Pad1.30x3.40mm_HandSolder" H 10500 5000 50  0001 C CNN
F 3 "~" H 10500 5000 50  0001 C CNN
F 4 "B82432-A1562-K" V 10500 5000 50  0001 C CNN "Ref"
	1    10500 5000
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP C13
U 1 1 5D8F204A
P 10200 5150
F 0 "C13" H 10318 5196 50  0000 L CNN
F 1 "10µF" H 10318 5105 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-7343-31_Kemet-D" H 10238 5000 50  0001 C CNN
F 3 "~" H 10200 5150 50  0001 C CNN
	1    10200 5150
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 5D8F2050
P 10200 5350
F 0 "#PWR0122" H 10200 5100 50  0001 C CNN
F 1 "GND" H 10205 5177 50  0000 C CNN
F 2 "" H 10200 5350 50  0001 C CNN
F 3 "" H 10200 5350 50  0001 C CNN
	1    10200 5350
	-1   0    0    -1  
$EndComp
Connection ~ 10200 5000
$Comp
L power:PWR_FLAG #FLG0106
U 1 1 5D8F2060
P 10200 5000
F 0 "#FLG0106" H 10200 5075 50  0001 C CNN
F 1 "PWR_FLAG" H 10200 5173 50  0000 C CNN
F 2 "" H 10200 5000 50  0001 C CNN
F 3 "~" H 10200 5000 50  0001 C CNN
	1    10200 5000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10200 5000 10400 5000
Wire Wire Line
	10600 5000 10900 5000
Wire Wire Line
	10200 5300 10200 5350
$Comp
L power:VDD #PWR0123
U 1 1 5D92B668
P 10900 5000
F 0 "#PWR0123" H 10900 4850 50  0001 C CNN
F 1 "VDD" H 10917 5173 50  0000 C CNN
F 2 "" H 10900 5000 50  0001 C CNN
F 3 "" H 10900 5000 50  0001 C CNN
	1    10900 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 5000 8400 5000
Wire Wire Line
	8400 5000 8400 5200
$Comp
L Jumper:Jumper_2_Bridged JP1
U 1 1 5D95B36C
P 4100 1800
F 0 "JP1" H 4100 1903 50  0000 C CNN
F 1 "Jumper_2_Bridged" H 4100 1904 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Bridged2Bar_RoundedPad1.0x1.5mm" H 4100 1800 50  0001 C CNN
F 3 "~" H 4100 1800 50  0001 C CNN
	1    4100 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1800 4350 1800
Connection ~ 4350 1800
Wire Wire Line
	4350 1850 4350 1800
Wire Wire Line
	3900 1800 3800 1800
Connection ~ 3800 1800
$EndSCHEMATC

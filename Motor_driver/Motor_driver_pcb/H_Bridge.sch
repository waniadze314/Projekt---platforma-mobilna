EESchema Schematic File Version 4
LIBS:Motor_driver_pcb-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 5
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
L Transistor_FET:IRF540N Q3
U 1 1 5C9E42AA
P 3850 3000
AR Path="/5C9E3FC0/5C9E42AA" Ref="Q3"  Part="1" 
AR Path="/5C9F39FE/5C9E42AA" Ref="Q?"  Part="1" 
AR Path="/5C9F4B73/5C9E42AA" Ref="Q?"  Part="1" 
AR Path="/5C9F4B7D/5C9E42AA" Ref="Q?"  Part="1" 
AR Path="/5CA0C20C/5C9E42AA" Ref="Q9"  Part="1" 
AR Path="/5CA13815/5C9E42AA" Ref="Q?"  Part="1" 
AR Path="/5CA1B151/5C9E42AA" Ref="Q15"  Part="1" 
AR Path="/5CA227B5/5C9E42AA" Ref="Q21"  Part="1" 
F 0 "Q21" H 4056 3046 50  0000 L CNN
F 1 "IRF540N" H 4056 2955 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4100 2925 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 3850 3000 50  0001 L CNN
	1    3850 3000
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:IRF9540N Q2
U 1 1 5C9E4811
P 3850 2500
AR Path="/5C9E3FC0/5C9E4811" Ref="Q2"  Part="1" 
AR Path="/5C9F39FE/5C9E4811" Ref="Q?"  Part="1" 
AR Path="/5C9F4B73/5C9E4811" Ref="Q?"  Part="1" 
AR Path="/5C9F4B7D/5C9E4811" Ref="Q?"  Part="1" 
AR Path="/5CA0C20C/5C9E4811" Ref="Q8"  Part="1" 
AR Path="/5CA13815/5C9E4811" Ref="Q?"  Part="1" 
AR Path="/5CA1B151/5C9E4811" Ref="Q14"  Part="1" 
AR Path="/5CA227B5/5C9E4811" Ref="Q20"  Part="1" 
F 0 "Q20" H 4050 2450 50  0000 L CNN
F 1 "IRF9540N" H 4050 2550 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4050 2425 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf9540n.pdf" H 3850 2500 50  0001 L CNN
	1    3850 2500
	1    0    0    1   
$EndComp
Wire Wire Line
	3950 2700 3950 2750
$Comp
L Diode:1N4148 D1
U 1 1 5C9E549E
P 4500 2500
AR Path="/5C9E3FC0/5C9E549E" Ref="D1"  Part="1" 
AR Path="/5C9F39FE/5C9E549E" Ref="D?"  Part="1" 
AR Path="/5C9F4B73/5C9E549E" Ref="D?"  Part="1" 
AR Path="/5C9F4B7D/5C9E549E" Ref="D?"  Part="1" 
AR Path="/5CA0C20C/5C9E549E" Ref="D5"  Part="1" 
AR Path="/5CA13815/5C9E549E" Ref="D?"  Part="1" 
AR Path="/5CA1B151/5C9E549E" Ref="D9"  Part="1" 
AR Path="/5CA227B5/5C9E549E" Ref="D13"  Part="1" 
F 0 "D13" V 4454 2579 50  0000 L CNN
F 1 "1N4148" V 4545 2579 50  0000 L CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4500 2325 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 4500 2500 50  0001 C CNN
	1    4500 2500
	0    1    1    0   
$EndComp
$Comp
L Diode:1N4148 D2
U 1 1 5C9E5F82
P 4500 3000
AR Path="/5C9E3FC0/5C9E5F82" Ref="D2"  Part="1" 
AR Path="/5C9F39FE/5C9E5F82" Ref="D?"  Part="1" 
AR Path="/5C9F4B73/5C9E5F82" Ref="D?"  Part="1" 
AR Path="/5C9F4B7D/5C9E5F82" Ref="D?"  Part="1" 
AR Path="/5CA0C20C/5C9E5F82" Ref="D6"  Part="1" 
AR Path="/5CA13815/5C9E5F82" Ref="D?"  Part="1" 
AR Path="/5CA1B151/5C9E5F82" Ref="D10"  Part="1" 
AR Path="/5CA227B5/5C9E5F82" Ref="D14"  Part="1" 
F 0 "D14" V 4454 3079 50  0000 L CNN
F 1 "1N4148" V 4545 3079 50  0000 L CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 4500 2825 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 4500 3000 50  0001 C CNN
	1    4500 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	3950 2750 4500 2750
Wire Wire Line
	4500 2750 4500 2650
Connection ~ 3950 2750
Wire Wire Line
	3950 2750 3950 2800
Wire Wire Line
	4500 2750 4500 2850
Connection ~ 4500 2750
Wire Wire Line
	4500 3150 4500 3200
Wire Wire Line
	4500 2350 4500 2300
Wire Wire Line
	4500 2300 3950 2300
$Comp
L Transistor_FET:IRF9540N Q4
U 1 1 5C9E6D79
P 6150 2500
AR Path="/5C9E3FC0/5C9E6D79" Ref="Q4"  Part="1" 
AR Path="/5C9F39FE/5C9E6D79" Ref="Q?"  Part="1" 
AR Path="/5C9F4B73/5C9E6D79" Ref="Q?"  Part="1" 
AR Path="/5C9F4B7D/5C9E6D79" Ref="Q?"  Part="1" 
AR Path="/5CA0C20C/5C9E6D79" Ref="Q10"  Part="1" 
AR Path="/5CA13815/5C9E6D79" Ref="Q?"  Part="1" 
AR Path="/5CA1B151/5C9E6D79" Ref="Q16"  Part="1" 
AR Path="/5CA227B5/5C9E6D79" Ref="Q22"  Part="1" 
F 0 "Q22" H 6356 2454 50  0000 L CNN
F 1 "IRF9540N" H 6350 2550 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 6350 2425 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf9540n.pdf" H 6150 2500 50  0001 L CNN
	1    6150 2500
	-1   0    0    1   
$EndComp
$Comp
L Transistor_FET:IRF540N Q5
U 1 1 5C9E8C92
P 6150 3000
AR Path="/5C9E3FC0/5C9E8C92" Ref="Q5"  Part="1" 
AR Path="/5C9F39FE/5C9E8C92" Ref="Q?"  Part="1" 
AR Path="/5C9F4B73/5C9E8C92" Ref="Q?"  Part="1" 
AR Path="/5C9F4B7D/5C9E8C92" Ref="Q?"  Part="1" 
AR Path="/5CA0C20C/5C9E8C92" Ref="Q11"  Part="1" 
AR Path="/5CA13815/5C9E8C92" Ref="Q?"  Part="1" 
AR Path="/5CA1B151/5C9E8C92" Ref="Q17"  Part="1" 
AR Path="/5CA227B5/5C9E8C92" Ref="Q23"  Part="1" 
F 0 "Q23" H 6356 2954 50  0000 L CNN
F 1 "IRF540N" H 6300 2850 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 6400 2925 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 6150 3000 50  0001 L CNN
	1    6150 3000
	-1   0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D3
U 1 1 5C9EAA23
P 5500 2500
AR Path="/5C9E3FC0/5C9EAA23" Ref="D3"  Part="1" 
AR Path="/5C9F39FE/5C9EAA23" Ref="D?"  Part="1" 
AR Path="/5C9F4B73/5C9EAA23" Ref="D?"  Part="1" 
AR Path="/5C9F4B7D/5C9EAA23" Ref="D?"  Part="1" 
AR Path="/5CA0C20C/5C9EAA23" Ref="D7"  Part="1" 
AR Path="/5CA13815/5C9EAA23" Ref="D?"  Part="1" 
AR Path="/5CA1B151/5C9EAA23" Ref="D11"  Part="1" 
AR Path="/5CA227B5/5C9EAA23" Ref="D15"  Part="1" 
F 0 "D15" V 5450 2350 50  0000 L CNN
F 1 "1N4148" V 5550 2150 50  0000 L CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 5500 2325 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 5500 2500 50  0001 C CNN
	1    5500 2500
	0    1    1    0   
$EndComp
$Comp
L Diode:1N4148 D4
U 1 1 5C9EC3FB
P 5500 3000
AR Path="/5C9E3FC0/5C9EC3FB" Ref="D4"  Part="1" 
AR Path="/5C9F39FE/5C9EC3FB" Ref="D?"  Part="1" 
AR Path="/5C9F4B73/5C9EC3FB" Ref="D?"  Part="1" 
AR Path="/5C9F4B7D/5C9EC3FB" Ref="D?"  Part="1" 
AR Path="/5CA0C20C/5C9EC3FB" Ref="D8"  Part="1" 
AR Path="/5CA13815/5C9EC3FB" Ref="D?"  Part="1" 
AR Path="/5CA1B151/5C9EC3FB" Ref="D12"  Part="1" 
AR Path="/5CA227B5/5C9EC3FB" Ref="D16"  Part="1" 
F 0 "D16" V 5450 2850 50  0000 L CNN
F 1 "1N4148" V 5550 2650 50  0000 L CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 5500 2825 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 5500 3000 50  0001 C CNN
	1    5500 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	6050 2300 5500 2300
Wire Wire Line
	5500 2300 5500 2350
Wire Wire Line
	5500 2650 5500 2750
Wire Wire Line
	6050 2700 6050 2750
Wire Wire Line
	6050 2750 5500 2750
Connection ~ 6050 2750
Wire Wire Line
	6050 2750 6050 2800
Connection ~ 5500 2750
Wire Wire Line
	5500 2750 5500 2850
Wire Wire Line
	5500 3200 5500 3150
Wire Wire Line
	5000 3200 5000 3300
Wire Wire Line
	5000 3200 5500 3200
Connection ~ 5500 3200
Wire Wire Line
	5500 3200 6050 3200
Wire Wire Line
	3950 3200 4500 3200
Connection ~ 5000 3200
Connection ~ 4500 3200
Wire Wire Line
	4500 3200 5000 3200
Text HLabel 2500 3000 0    50   Input ~ 0
PWM_1
Text HLabel 7650 3000 2    50   Input ~ 0
PWM_2
Text HLabel 5000 3300 3    50   Input ~ 0
GND
Wire Wire Line
	4500 2300 5500 2300
Connection ~ 4500 2300
Connection ~ 5500 2300
Text HLabel 5000 2300 1    50   Input ~ 0
HV
Text HLabel 4500 2750 2    50   Input ~ 0
Motor_+
Text HLabel 5500 2750 0    50   Input ~ 0
Motor_-
$Comp
L Device:R_Small R5
U 1 1 5CA04B0F
P 6400 2400
AR Path="/5C9E3FC0/5CA04B0F" Ref="R5"  Part="1" 
AR Path="/5CA0C20C/5CA04B0F" Ref="R13"  Part="1" 
AR Path="/5CA13815/5CA04B0F" Ref="R?"  Part="1" 
AR Path="/5CA1B151/5CA04B0F" Ref="R21"  Part="1" 
AR Path="/5CA227B5/5CA04B0F" Ref="R29"  Part="1" 
F 0 "R29" H 6450 2400 50  0000 L CNN
F 1 "1k" H 6450 2300 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6400 2400 50  0001 C CNN
F 3 "~" H 6400 2400 50  0001 C CNN
	1    6400 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R6
U 1 1 5CA04F40
P 6400 3100
AR Path="/5C9E3FC0/5CA04F40" Ref="R6"  Part="1" 
AR Path="/5CA0C20C/5CA04F40" Ref="R14"  Part="1" 
AR Path="/5CA13815/5CA04F40" Ref="R?"  Part="1" 
AR Path="/5CA1B151/5CA04F40" Ref="R22"  Part="1" 
AR Path="/5CA227B5/5CA04F40" Ref="R30"  Part="1" 
F 0 "R30" H 6450 3100 50  0000 L CNN
F 1 "1k" H 6450 3000 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6400 3100 50  0001 C CNN
F 3 "~" H 6400 3100 50  0001 C CNN
	1    6400 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5CA0523E
P 3600 3100
AR Path="/5C9E3FC0/5CA0523E" Ref="R4"  Part="1" 
AR Path="/5CA0C20C/5CA0523E" Ref="R12"  Part="1" 
AR Path="/5CA13815/5CA0523E" Ref="R?"  Part="1" 
AR Path="/5CA1B151/5CA0523E" Ref="R20"  Part="1" 
AR Path="/5CA227B5/5CA0523E" Ref="R28"  Part="1" 
F 0 "R28" H 3650 3100 50  0000 L CNN
F 1 "1k" H 3650 3000 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3600 3100 50  0001 C CNN
F 3 "~" H 3600 3100 50  0001 C CNN
	1    3600 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5CA057A9
P 3600 2400
AR Path="/5C9E3FC0/5CA057A9" Ref="R3"  Part="1" 
AR Path="/5CA0C20C/5CA057A9" Ref="R11"  Part="1" 
AR Path="/5CA13815/5CA057A9" Ref="R?"  Part="1" 
AR Path="/5CA1B151/5CA057A9" Ref="R19"  Part="1" 
AR Path="/5CA227B5/5CA057A9" Ref="R27"  Part="1" 
F 0 "R27" H 3450 2400 50  0000 L CNN
F 1 "1k" H 3450 2300 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3600 2400 50  0001 C CNN
F 3 "~" H 3600 2400 50  0001 C CNN
	1    3600 2400
	1    0    0    -1  
$EndComp
Connection ~ 6400 3000
Wire Wire Line
	6400 3000 6350 3000
Wire Wire Line
	7650 3000 7600 3000
Connection ~ 6400 2500
Wire Wire Line
	6400 2500 6350 2500
Wire Wire Line
	3650 2500 3600 2500
Connection ~ 3600 2500
Wire Wire Line
	2550 3000 2500 3000
Wire Wire Line
	3650 3000 3600 3000
Connection ~ 3600 3000
Wire Wire Line
	3600 2300 3950 2300
Connection ~ 3950 2300
Wire Wire Line
	6400 2300 6050 2300
Connection ~ 6050 2300
Wire Wire Line
	3600 2500 3600 2600
Wire Wire Line
	6400 2500 6400 2600
Wire Wire Line
	2850 3200 3600 3200
Wire Wire Line
	6400 3200 6050 3200
Connection ~ 6050 3200
Wire Wire Line
	3950 3200 3600 3200
Connection ~ 3950 3200
Connection ~ 3600 3200
$Comp
L Device:R_Small R2
U 1 1 5CABFFC4
P 2850 2700
AR Path="/5C9E3FC0/5CABFFC4" Ref="R2"  Part="1" 
AR Path="/5CA0C20C/5CABFFC4" Ref="R10"  Part="1" 
AR Path="/5CA1B151/5CABFFC4" Ref="R18"  Part="1" 
AR Path="/5CA227B5/5CABFFC4" Ref="R26"  Part="1" 
F 0 "R26" H 2909 2746 50  0000 L CNN
F 1 "1k" H 2909 2655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2850 2700 50  0001 C CNN
F 3 "~" H 2850 2700 50  0001 C CNN
	1    2850 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5CAC04F6
P 2850 2500
AR Path="/5C9E3FC0/5CAC04F6" Ref="R1"  Part="1" 
AR Path="/5CA0C20C/5CAC04F6" Ref="R9"  Part="1" 
AR Path="/5CA1B151/5CAC04F6" Ref="R17"  Part="1" 
AR Path="/5CA227B5/5CAC04F6" Ref="R25"  Part="1" 
F 0 "R25" H 2909 2546 50  0000 L CNN
F 1 "220k" H 2909 2455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2850 2500 50  0001 C CNN
F 3 "~" H 2850 2500 50  0001 C CNN
	1    2850 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 2400 2850 2300
Wire Wire Line
	2850 2300 3600 2300
Connection ~ 3600 2300
Wire Wire Line
	2850 2600 3600 2600
Connection ~ 2850 2600
Connection ~ 3600 2600
Wire Wire Line
	3600 2600 3600 3000
Wire Wire Line
	7300 3200 6400 3200
Connection ~ 6400 3200
$Comp
L Device:R_Small R8
U 1 1 5CBAE7DE
P 7300 2700
AR Path="/5C9E3FC0/5CBAE7DE" Ref="R8"  Part="1" 
AR Path="/5CA0C20C/5CBAE7DE" Ref="R16"  Part="1" 
AR Path="/5CA1B151/5CBAE7DE" Ref="R24"  Part="1" 
AR Path="/5CA227B5/5CBAE7DE" Ref="R32"  Part="1" 
F 0 "R32" H 7359 2746 50  0000 L CNN
F 1 "1k" H 7359 2655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7300 2700 50  0001 C CNN
F 3 "~" H 7300 2700 50  0001 C CNN
	1    7300 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5CBAE7E4
P 7300 2500
AR Path="/5C9E3FC0/5CBAE7E4" Ref="R7"  Part="1" 
AR Path="/5CA0C20C/5CBAE7E4" Ref="R15"  Part="1" 
AR Path="/5CA1B151/5CBAE7E4" Ref="R23"  Part="1" 
AR Path="/5CA227B5/5CBAE7E4" Ref="R31"  Part="1" 
F 0 "R31" H 7359 2546 50  0000 L CNN
F 1 "220k" H 7359 2455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7300 2500 50  0001 C CNN
F 3 "~" H 7300 2500 50  0001 C CNN
	1    7300 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 2400 7300 2300
Wire Wire Line
	7300 2300 6400 2300
Connection ~ 6400 2300
Wire Wire Line
	7300 2600 6400 2600
Connection ~ 7300 2600
Connection ~ 6400 2600
Wire Wire Line
	6400 2600 6400 3000
$Comp
L Device:Q_NPN_EBC Q1
U 1 1 5CC73221
P 2750 3000
AR Path="/5C9E3FC0/5CC73221" Ref="Q1"  Part="1" 
AR Path="/5CA0C20C/5CC73221" Ref="Q7"  Part="1" 
AR Path="/5CA1B151/5CC73221" Ref="Q13"  Part="1" 
AR Path="/5CA227B5/5CC73221" Ref="Q19"  Part="1" 
F 0 "Q19" H 2941 3046 50  0000 L CNN
F 1 "2N2222" H 2941 2955 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92" H 2950 3100 50  0001 C CNN
F 3 "~" H 2750 3000 50  0001 C CNN
	1    2750 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NPN_EBC Q6
U 1 1 5CC746B3
P 7400 3000
AR Path="/5C9E3FC0/5CC746B3" Ref="Q6"  Part="1" 
AR Path="/5CA0C20C/5CC746B3" Ref="Q12"  Part="1" 
AR Path="/5CA1B151/5CC746B3" Ref="Q18"  Part="1" 
AR Path="/5CA227B5/5CC746B3" Ref="Q24"  Part="1" 
F 0 "Q24" H 7591 3046 50  0000 L CNN
F 1 "2N2222" H 7591 2955 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92" H 7600 3100 50  0001 C CNN
F 3 "~" H 7400 3000 50  0001 C CNN
	1    7400 3000
	-1   0    0    -1  
$EndComp
$EndSCHEMATC

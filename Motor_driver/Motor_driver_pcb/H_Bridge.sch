EESchema Schematic File Version 4
EELAYER 29 0
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
L Transistor_FET:IRF540N Q2
U 1 1 5C9E42AA
P 3850 3000
AR Path="/5C9E3FC0/5C9E42AA" Ref="Q2"  Part="1" 
AR Path="/5C9F39FE/5C9E42AA" Ref="Q6"  Part="1" 
AR Path="/5C9F4B73/5C9E42AA" Ref="Q10"  Part="1" 
AR Path="/5C9F4B7D/5C9E42AA" Ref="Q14"  Part="1" 
F 0 "Q14" H 4056 3046 50  0000 L CNN
F 1 "IRF540N" H 4056 2955 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4100 2925 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 3850 3000 50  0001 L CNN
	1    3850 3000
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:IRF9540N Q1
U 1 1 5C9E4811
P 3850 2500
AR Path="/5C9E3FC0/5C9E4811" Ref="Q1"  Part="1" 
AR Path="/5C9F39FE/5C9E4811" Ref="Q5"  Part="1" 
AR Path="/5C9F4B73/5C9E4811" Ref="Q9"  Part="1" 
AR Path="/5C9F4B7D/5C9E4811" Ref="Q13"  Part="1" 
F 0 "Q13" H 4056 2546 50  0000 L CNN
F 1 "IRF9540N" H 4056 2455 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4050 2425 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf9540n.pdf" H 3850 2500 50  0001 L CNN
	1    3850 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2700 3950 2750
$Comp
L Diode:1N4148 D1
U 1 1 5C9E549E
P 4500 2500
AR Path="/5C9E3FC0/5C9E549E" Ref="D1"  Part="1" 
AR Path="/5C9F39FE/5C9E549E" Ref="D5"  Part="1" 
AR Path="/5C9F4B73/5C9E549E" Ref="D9"  Part="1" 
AR Path="/5C9F4B7D/5C9E549E" Ref="D13"  Part="1" 
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
AR Path="/5C9F39FE/5C9E5F82" Ref="D6"  Part="1" 
AR Path="/5C9F4B73/5C9E5F82" Ref="D10"  Part="1" 
AR Path="/5C9F4B7D/5C9E5F82" Ref="D14"  Part="1" 
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
L Transistor_FET:IRF9540N Q3
U 1 1 5C9E6D79
P 6150 2500
AR Path="/5C9E3FC0/5C9E6D79" Ref="Q3"  Part="1" 
AR Path="/5C9F39FE/5C9E6D79" Ref="Q7"  Part="1" 
AR Path="/5C9F4B73/5C9E6D79" Ref="Q11"  Part="1" 
AR Path="/5C9F4B7D/5C9E6D79" Ref="Q15"  Part="1" 
F 0 "Q15" H 6356 2454 50  0000 L CNN
F 1 "IRF9540N" H 6300 2350 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 6350 2425 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf9540n.pdf" H 6150 2500 50  0001 L CNN
	1    6150 2500
	-1   0    0    1   
$EndComp
$Comp
L Transistor_FET:IRF540N Q4
U 1 1 5C9E8C92
P 6150 3000
AR Path="/5C9E3FC0/5C9E8C92" Ref="Q4"  Part="1" 
AR Path="/5C9F39FE/5C9E8C92" Ref="Q8"  Part="1" 
AR Path="/5C9F4B73/5C9E8C92" Ref="Q12"  Part="1" 
AR Path="/5C9F4B7D/5C9E8C92" Ref="Q16"  Part="1" 
F 0 "Q16" H 6356 2954 50  0000 L CNN
F 1 "IRF540N" H 6300 2850 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 6400 2925 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 6150 3000 50  0001 L CNN
	1    6150 3000
	-1   0    0    1   
$EndComp
$Comp
L Diode:1N4148 D3
U 1 1 5C9EAA23
P 5500 2500
AR Path="/5C9E3FC0/5C9EAA23" Ref="D3"  Part="1" 
AR Path="/5C9F39FE/5C9EAA23" Ref="D7"  Part="1" 
AR Path="/5C9F4B73/5C9EAA23" Ref="D11"  Part="1" 
AR Path="/5C9F4B7D/5C9EAA23" Ref="D15"  Part="1" 
F 0 "D15" V 5454 2579 50  0000 L CNN
F 1 "1N4148" V 5545 2579 50  0000 L CNN
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
AR Path="/5C9F39FE/5C9EC3FB" Ref="D8"  Part="1" 
AR Path="/5C9F4B73/5C9EC3FB" Ref="D12"  Part="1" 
AR Path="/5C9F4B7D/5C9EC3FB" Ref="D16"  Part="1" 
F 0 "D16" V 5454 3079 50  0000 L CNN
F 1 "1N4148" V 5545 3079 50  0000 L CNN
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
Text HLabel 3650 2500 0    50   Input ~ 0
PWM_1_High
Text HLabel 3650 3000 0    50   Input ~ 0
PWM_1_Low
Text HLabel 6350 2500 2    50   Input ~ 0
PWM_2_High
Text HLabel 6350 3000 2    50   Input ~ 0
PWM_2_Low
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
$EndSCHEMATC

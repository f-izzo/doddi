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
LIBS:adafruit+pololu
LIBS:LED_RGB
LIBS:led_breakout-cache
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
$Comp
L neopixel D1
U 1 1 58DD3308
P 5550 3350
F 0 "D1" H 5550 3700 60  0000 C CNN
F 1 "neopixel" H 5550 3600 60  0000 C CNN
F 2 "LED5_RGB:LED5_RGB" H 5550 3550 60  0001 C CNN
F 3 "" H 5550 3550 60  0001 C CNN
	1    5550 3350
	1    0    0    -1  
$EndComp
$Comp
L NeopixelConn X1
U 1 1 58DD343F
P 5000 3150
F 0 "X1" H 5000 3250 60  0000 C CNN
F 1 "NeopixelConn" H 5000 3150 60  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 5000 3150 60  0001 C CNN
F 3 "" H 5000 3150 60  0001 C CNN
	1    5000 3150
	1    0    0    -1  
$EndComp
$Comp
L NeopixelConn X2
U 1 1 58DD34D6
P 6200 3150
F 0 "X2" H 6200 3250 60  0000 C CNN
F 1 "NeopixelConn" H 6200 3150 60  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 6200 3150 60  0001 C CNN
F 3 "" H 6200 3150 60  0001 C CNN
	1    6200 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 3650 4850 3900
Wire Wire Line
	4850 3900 5400 3900
Wire Wire Line
	5400 3900 5400 3650
Wire Wire Line
	5700 3650 5700 3900
Wire Wire Line
	5700 3900 6050 3900
Wire Wire Line
	6050 3900 6050 3650
$Comp
L GND #PWR01
U 1 1 58DD37C8
P 5050 3650
F 0 "#PWR01" H 5050 3400 50  0001 C CNN
F 1 "GND" H 5050 3500 50  0000 C CNN
F 2 "" H 5050 3650 50  0001 C CNN
F 3 "" H 5050 3650 50  0001 C CNN
	1    5050 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 58DD37E8
P 5500 3650
F 0 "#PWR02" H 5500 3400 50  0001 C CNN
F 1 "GND" H 5500 3500 50  0000 C CNN
F 2 "" H 5500 3650 50  0001 C CNN
F 3 "" H 5500 3650 50  0001 C CNN
	1    5500 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 58DD380C
P 6250 3650
F 0 "#PWR03" H 6250 3400 50  0001 C CNN
F 1 "GND" H 6250 3500 50  0000 C CNN
F 2 "" H 6250 3650 50  0001 C CNN
F 3 "" H 6250 3650 50  0001 C CNN
	1    6250 3650
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR04
U 1 1 58DD381F
P 4950 3650
F 0 "#PWR04" H 4950 3500 50  0001 C CNN
F 1 "+5V" H 4950 3790 50  0000 C CNN
F 2 "" H 4950 3650 50  0001 C CNN
F 3 "" H 4950 3650 50  0001 C CNN
	1    4950 3650
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR05
U 1 1 58DD383A
P 5600 3650
F 0 "#PWR05" H 5600 3500 50  0001 C CNN
F 1 "+5V" H 5600 3790 50  0000 C CNN
F 2 "" H 5600 3650 50  0001 C CNN
F 3 "" H 5600 3650 50  0001 C CNN
	1    5600 3650
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR06
U 1 1 58DD3867
P 6150 3650
F 0 "#PWR06" H 6150 3500 50  0001 C CNN
F 1 "+5V" H 6150 3790 50  0000 C CNN
F 2 "" H 6150 3650 50  0001 C CNN
F 3 "" H 6150 3650 50  0001 C CNN
	1    6150 3650
	-1   0    0    1   
$EndComp
$EndSCHEMATC

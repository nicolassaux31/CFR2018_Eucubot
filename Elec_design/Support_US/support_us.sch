EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
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
LIBS:support_us-cache
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
L Conn_01x04 J2
U 1 1 5A84844F
P 3725 2000
F 0 "J2" H 3725 2200 50  0000 C CNN
F 1 "Conn_01x04" H 3725 1700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 3725 2000 50  0001 C CNN
F 3 "" H 3725 2000 50  0001 C CNN
	1    3725 2000
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J1
U 1 1 5A8484DC
P 2850 2100
F 0 "J1" H 2850 2300 50  0000 C CNN
F 1 "Conn_01x04" H 2850 1800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 2850 2100 50  0001 C CNN
F 3 "" H 2850 2100 50  0001 C CNN
	1    2850 2100
	-1   0    0    1   
$EndComp
Wire Wire Line
	3050 1900 3525 1900
Wire Wire Line
	3525 2000 3050 2000
Wire Wire Line
	3525 2100 3050 2100
Wire Wire Line
	3050 2200 3525 2200
Text Label 3200 1900 0    60   ~ 0
GND
Text Label 3200 2000 0    60   ~ 0
Echo
Text Label 3200 2100 0    60   ~ 0
Trig
Text Label 3200 2200 0    60   ~ 0
Vcc
$EndSCHEMATC

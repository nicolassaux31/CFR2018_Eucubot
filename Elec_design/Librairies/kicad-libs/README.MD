# Shimatta PCB Libs
Libraries for KiCad.

## ADC
* AD1974

## altera
* EP4CE6E22C8N

## lattice
* LFXP2-5E-6TN144C
* LCMXO2-1200HC-5SG32C

## Video
* ADV7513

## Power
* MCP73833
* ST1S10

## Regulators
* MC34063
* LM2596
* LTC3612 TSSOP version
* SIM2-XXXXD  (SIL) dual voltage switching regulator series.

## STM32F4
* STM32F407VGT
* STM32F407ZGT

## Connectors
* Micro SD Card
* 3.5mm TRS Jack
* HDMI-Jack

## memory
* SDRAM

## mechanic
* FTR-B4CA 4,5V compatible Relais

## analog
* UDA1330ATS 16-bit Stereo DAC

## eth-phy
* LAN8720A

## eth-mechanical
* RJ45 with transformer

## clock
* OSC (oscillator)

## Optics
* Finisar optic SFF module

## passives
* Oscillator
* potentiometer (dual Pot ALPS)

## shields
* ITEAD 3.2" Display

## ti
* OPA2134 Dual Audio OpAmp
* PCM2704 USB Audio DAC
* TFP401 DVI Receiver
* TFP410 DVI Transceiver
* TPA6120(A2) Headphone Amp

## interfaces/usbserial
* CH340G

# Shimatta Footprints
Some footprints have no 3D model. However, in some cases a model of another footprint can be used.

Legend:

| Symbol | Meaning |
| :-------| :--------- |
| Y | Available in this library |
| - | Not available in this library |
| O | Standard model used |

## SMD

| Name       | Footprint       | 3D Model       |
| :--------- | :-------------: | :------------: |
| 3.5mm-stereo-audio-jack   | Y               | -              |
| TFBGA-216 |  Y | Y |
| FBGA-96 | Y | Y |
| FBGA-484 1mm pitch | Y | Y |
| HDMI Jack | Y | Y |
| HQFP-64 | Y | - |
| HQFP-100 | Y | - |
| LPISM 13mm x 9.55mm Inductor | Y | Y |
| LQFP-64 with Thermal Pad | Y | - |
| Micro SD Slot | Y | - |
| Micro SD Slot Amphenol | Y | Y |
| Oscillator | Y | - |
| RGB-LED-LTST | Y | - |
| Resistor Pack 4 | Y | - |
| SMD DIP Switch 5 Pins | Y | Y
| SOIC 20 Thermalpad | Y | O |
| SSOP-20 Thermal Pad | Y | O |
| SuperSOT-6 | Y | - |
| Tactile Switch | Y | Y |
| TQFP-144 pitch 0.5 | Y | - |
| TSOP-54 | Y | - |
| USB Jack | - | Y |
| SOT-753 | - | Y |
| FlipChip-BGA | - | Y |
| coil WE-PD2 | - | Y |


## displays
| Name       | Footprint       | 3D Model       |
| :--------- | :-------------: | :------------: |
| LCD 4x16 Characters | Y | - |
| ITDB02-3.2 | Y | - |

## misc
| Name       | Footprint       | 3D Model       |
| :--------- | :-------------: | :------------: |
| 17 mm Buzzer | Y | Y |

## artwork
* shimatta( nuclear| audio)
* screw_art(big|small|tiny)

## led
| Name       | Footprint       | 3D Model       |
| :--------- | :-------------: | :------------: |
| 5x7 LED Matrix | Y | Y |


## tht

| Name       | Footprint       | 3D Model       |
| :--------- | :-------------: | :------------: |
|AMPHENOL-ACJS-HHDR | Y | - |
|Lumberg-1503 Audio Jack | Y | - |
|ALPS-RK09L12B0 Stereo Potentiometer | Y | - |
| SIL7 reduced | Y | - |
| RJ-45 MagJack Stewart | Y | Y |
| THT DIP Switch 4 | Y | Y |
| Diode DO-41 | - | Y |
| Finisar 2x7-SFF Fiber Modulator | Y | - |
| Pin Header Socket 2x05 | - | Y |
| Pin Header Socket 2x10 | - | Y |

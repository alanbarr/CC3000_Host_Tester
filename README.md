# CC3000 Host Start Tester


## Why?
I found that I needed to debug the SPI protocol for a STM32 Host to a CC3000 
without a scope handy (or 1.8 V level shifters) so I thought why not test it
out against something a little friendlier... a MSP430 Launchpad. The idea being 
that the debug information from the MSP430 Launchpad in conjunction with the 
output the STM32 Host should permit easier debugging. Of course if you have a 
good logic analyser or scope handy that'll likely be more informative.


## What?
This code runs on a MSP430 Launchpad (MSP430G2553, and aims to emulate a CC3000
for a call to `wlan_start(0)`. This call is made from "cold" i.e. the 
CC3000 EN line is expected to be low. The behaviour this emulates is based off
what is documented by TI
[here](http://processors.wiki.ti.com/index.php/CC3000_Serial_Port_Interface_(SPI)#CC3000_Init_Operation).
Should any step from the host (your device under test) deviate from what is 
expected you will get a (hopefully) useful error message sent over UART, 
describing the error and what step it occurred on. 


## How?
1. Connect the LP to your PC and open a UART serial connection (9600 8-N-1).
2. Connect the LP to your Host / Device Under Test (see Pins below).
2. Reset the Launchpad - it should wait until it detects pins have been initialised.
3. Start your host program.
4. Monitor the UART serial connect for output.


## Pins
The MSP430 Launchpad has the following pin set-up:

| Pin      | Connection |
|----------|------------|
| 1.0      | RED LED    |
| 1.2      | UART TX    |
| 1.4      | SPI CS     |
| 1.5      | SPI CLK    |
| 1.6      | SPI SOMI   |
| 1.7      | SPI SIMO   |
| 2.0      | CC3000 EN  |
| 2.1      | CC3000 IRQ |

*Note:* UART is hardware, so the MSP430 Launchpad jumpers should be horizontal.


## Steps
The following are the steps that `wlan_start(0)` is expected to follow:

| Step          | Action Expected                                           |
|---------------|-----------------------------------------------------------|
|      0        | Host sets EN low, CS High, CLK Low. (For least 50 ms)     |
|      1        | Host sets EN High                                         |
|      2        | We set IRQ Low                                            |
|      3        | Host Sets CS Low                                          |
|      4        | Nothing happens for 50 us                                 |
|      5        | We RX: First 4 bytes of HCI_CMND_SIMPLE_LINK_START        |
|      6        | Nothing happens for 50 us                                 |
|      7        | We RX: Last 6 bytes of HCI_CMND_SIMPLE_LINK_START Reply   |
|      8        | Host sets CS high, We set IRQ high                        |
|               |                                                           |
|      9        | We drop IRQ                                               |
|      10       | Host set CS low                                           |
|      11       | We TX: COMMAND OK                                         |
|      12       | Host pulls CS high                                        |
|      13       | We raise IRQ                                              |
|               |                                                           |
|      14       | Host sets CS Low                                          |
|      15       | We set IRQ Low                                            |
|      16       | We RX: HCI_CMND_READ_BUFFER_SIZE                          |
|      17       | Host sets CS High                                         |
|      18       | We set IRQ High                                           |
|               |                                                           |
|      19       | We set IRQ Low                                            |
|      20       | Host sets CS Low                                          |
|      21       | We TX: HCI_CMND_READ_BUFFER_SIZE Reply                    |
|               |                                                           |
|      22       | Host Pulls CS High                                        |
|      23       | We set IRQ High                                           |
|      DONE!    |                                                           |


## Testing
This code has so far only been tested against a second MSP430 Launchpad 
(MSP430G2553), running Texas Instruments Basic Wi-Fi Application. 
[Found here](http://processors.wiki.ti.com/index.php/CC3000_Wi-Fi_Downloads#Basic_Wifi_Application_and_Sample_Demos).


## Notes
This has been compiled using MSPGCC. Hopefully it should be easy to use with 
IAR or CSS. I think any likely error will be with functions setting bits in
the status register, as I wasn't sure if there was a common function.


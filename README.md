# S76G Helium US915 Example

### Example sketch from the MCCI LoRaWAN LMIC library modified to run on the AcSiP S76G SiP

**Building firmware for the AcSiP S76G SiP**

***Required software:***

* Arduino IDE https://www.arduino.cc/en/software
* STM32CubeProgrammer https://www.st.com/en/development-tools/stm32cubeprog.html
*  Arduino-LMIC library https://github.com/mcci-catena/arduino-lmic (also in Arduino Library)


* Install the Arduino IDE and STM32CubeProgrammer
* Start the Arduino IDE
    * Go to **Tools > Board > Board Manager** type _STM32_ in the filter your search... field
    * Click Install to install the current version of the STM32 Cores Board
* After the STM32 Cores Boards is installed exit and restart the Arduino IDE
    * Go to **Tools > Board > STM32 Boards > Nucleo-64**
    * Then select the microcontroller **Tools > Board Part Number > Nucleo L073RZ**
    * Then configure the environment:
        * Tools > U(S)ART Support > Enable (generic 'Serial')
        * Tools > USB Support (if available) > None
        * Tools > USB Speed (if available) > Low/Full Speed
        * Tools > Optimize > Smallest (Os default) or Fast (O1)
        * Tools > C Runtime Library > Newlib Nano (default)
        * Tools > Upload Method > STM32CubeProgrammer (serial)
        * Tools > Port > `Select the port associated with the RAK7200}`

When you have finished configuring the environment it should look similar to this:
![](https://github.com/JasonRJ/RAK7200_Helium_Mapper/blob/master/documentation/images/RAK7200%20S76G%20Arduino%20Settings.png)

## Credits

[RAK Wireless](https://www.rakwireless.com/) for
the [WisNode Track Lite RAK7200](https://store.rakwireless.com/products/rak7200-lpwan-tracker)

[Linar Yusupov](https://github.com/lyusupov) for the [POST_S76G](https://github.com/lyusupov/POST_S76G) sketch, Arduino
board configuration and S76G pin definitions which advanced the project tremendously.


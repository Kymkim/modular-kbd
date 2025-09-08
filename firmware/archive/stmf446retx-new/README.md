# How to build...
For this project we are using CMake to build our stuff. 

Using VSCode/Codium you would need...

CMake Tools by Microsoft

Just install it and it should work out of the box with no additional steps. At the bottom left there should be a "Build & Debug Microcontroller - ST-Link" button which will let you debug and... build the microcontroller

Optional: 
Install STM32Cube for Visual Studio Code by STMicroelectronics - just for generatiing the HAL initial config.

I added all of the drivers made by ST in here too, not necessary but we can remove the ones we dont need. It's just there for now - in case we have to use some of them
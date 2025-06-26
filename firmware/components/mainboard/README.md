# Mainboard

The goal of the mainboard is to take in CAN signals from the modules, and then convert these can signals to USB HID report. How the can signals would be structured is that it would be based off the Universal Serial Bus HID Usage Table. We can let the first byte be the Usage ID and the second byte be the status of that key. 


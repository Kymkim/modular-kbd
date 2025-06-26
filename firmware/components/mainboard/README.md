# Mainboard

The goal of the mainboard is to take in CAN signals from the modules, and then convert these can signals to USB HID report. 

# How is CAN Data is Transmitted

When you press a button on a module, it sends two CAN messages—one when the key is pressed down, and another when it’s released. This is in order to avoid spamming the bus with constant message of every status for every key. 

The CAN data follows a simple 2-byte pattern:
- The **first byte** is the Universal HID Keyboard Key Code
- The **second byte** reports the status of that key (pressed or released)

So, if we send something like `xx04`, it means the keycode for the letter “A” is pressed The mainboard picks that up and adds it to the report FIFO queue
## Keycode Byte

Keyboard codes can be found here https://www.toomanyatoms.com/computer/usb_keyboard_codes.html.  They are pretty self explanatory.
## Second Byte
|Bit|Meaning|
|---|---|
|0xxx xxxx|Is the key pressed? 0 = released, 1 = pressed

Of course, we have 7 bits extra, but this can be used for future proofing. For example, we might want a module that does not follow the USB Keyboard Codes. Think of it as configuration bits that maybe useful in the future. 

# N-Key Rollover
We’ll use a FIFO (First In, First Out) queue to keep track of all the keys pressed and update the USB HID report accordingly.

Read more about the HID report structure here: https://wiki.osdev.org/USB_Human_Interface_Devices

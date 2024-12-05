# Battery Scanner

## Philosophy

Inspired by [Mechanical Advantage's 2024 battery scanning utility](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2024-build-thread/442736/391), we set out to create a battery scanning that logged each battery's id in a match, which could be used to log future statistics, like [Match Count, Runtime, Power](https://www.chiefdelphi.com/uploads/default/original/3X/3/1/31864f94225aa8380e5f00228048168bbf79f656.png), but also importantly battery health degredation over time represented by voltage.

Mechanical Advantage currently provides their scanning utility of choice and code. In this, I'll more closely explain how to choose a scanner, setup the scanner with the correct configuration, and adapting Mechanical Advantage's code (or ours) for your specific needs.

## Barcode Module
- [USB C Code Reader Module](https://www.amazon.com/XICOOLEE-Barcode-Scanner-Reader-Interface/dp/B0BC1H3ZR1?dib=eyJ2IjoiMSJ9.zJoS4HL2SCC_jQ0iv8L7IgP4HZeUEEZ-leOj8f4S1n1mzAu9df3Fylb4Mp_lEbqhBbilSUYPOBrfrvlzOeCjj22Ete7uwGJvD1FS3ynFVParF8VniRxCFrvT4RAljThJ6-k8e6AAn7fM6oe-ImLurFzgKbE3-M67NXDq5Pk79pFzrigbDe-cj8ggWoy4IA9YNOacQfBIcQ5UZOyzcELk9IlDpYK0jQ8yrM4BCE6uz-k.Fk4zP7itiaLxx_wJusWfZdSOolUIe3rR6Lvtq4-P2AQ&dib_tag=se&keywords=qr+code+scanner+module&qid=1730069451&sr=8-5)
  - what 8033 currently uses
- [Micro USB Code Reader Module](https://www.amazon.com/Waveshare-Barcode-Scanner-Directly-Computer/dp/B07P3GD3XV?dib=eyJ2IjoiMSJ9.zJoS4HL2SCC_jQ0iv8L7IgP4HZeUEEZ-leOj8f4S1n1mzAu9df3Fylb4Mp_lEbqhBbilSUYPOBrfrvlzOeCjj22Ete7uwGJvD1FS3ynFVParF8VniRxCFrvT4RAljThJ6-k8e6AAn7fM6oe-ImLurFzgKbE3-M67NXDq5Pk79pFzrigbDe-cj8ggWoy4IA9YNOacQfBIcQ5UZOyzcELk9IlDpYK0jQ8yrM4BCE6uz-k.Fk4zP7itiaLxx_wJusWfZdSOolUIe3rR6Lvtq4-P2AQ&dib_tag=se&keywords=qr+code+scanner+module&qid=1730069451&sr=8-4)
  - what 6328 uses
 
Either module is fine, but this documentation goes through how to setup the USB C code reader module. If you are buying a different module, make sure that the module supports `USB Virtual Serial Port` mode and `Command Triggered Mold`. It's also nice to have audible feedback for scans and a built-in light.

#### Our Challenges and Constraints
- A PhotonVision setup coprocessor
  - comes with python 3.10 but not pip (Python's package manager)
  - **has no internet access** (or no easy way to link such interface)
  - should be able to install quickly
  - runs arm64 and Python's binary cross compilation isn't mature

The guide is designed around these challenges and constraints in mind. 

## Setup on Barcode Module

After plugging the module into your coprocessor, you will need to configure a few parameters to work as desired. First, `Scanner.py` continuously scans by sending a `Command Triggered Mold` or some commands over serial port. First, you will need to setup your module to advertise a `USB Virtual Serial Port` then find the serial port name on your device.

Advertising your module as a `USB Virtual Serial Port` and activating `Command Triggered Mold` usually involves scanning a QR code to specify a configuration. For the USB-C module mentioned above, you can find the full [PDF documentation](https://seengreat.com/upload/file/109/Barcode+Scanner+Reader+User+Manual_en.pdf). Specifically, see sections `3.3` and `4.4` in the linked pdf to set this up.

## Setup on Robot

You have two choices here. You can either setup either connect the scanner to the roboRIO or to a coprocessor, like an ornage pi. Generally, it's advised to connect the scanner module to a coprocessor, since it's known that the roboRIO might have problems with the USB port.

8033 connects the scanner module to our coprocessor, and thus have some modifications to [6328's code](https://github.com/Mechanical-Advantage/RobotCode2024/blob/face2f4f620502018fbb9715f2c1f724de799298/battery_scanner/scanner.py) for it work.

Next, scp the `orangepi-deploy` folder to your coprocessor:

```bash
scp -r ./orangepi-deploy/ pi@10.80.33.11:/home/pi/
```
> Ensure you replace the orange pi directory and the client name, address, and output directory. To find the coprocessor address, check in photonvision and find what address the backend is connected to.

In `Scanner.py`, ensure to update your team number and device name.

To find the device name, open a Python shell run:
```python
import serial.tools.list_ports
print(len(list(serial.tools.list_ports.comports()))) # Gives the numer of devices connected
print(list(serial.tools.list_ports.comports())[0]) # Gives the `device name` of a
```

To run this, execute `scan.bash` in a terminal, which links the pynetworktables module to your pythonpath. Also,
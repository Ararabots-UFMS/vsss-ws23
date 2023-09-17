# Esp-Now Message Server
## Requirements
Since the module uses the write operation on serial /dev/ttyUSB0, the current user must have access to the device.
To give permission, please run this two commands through terminal:

```sh
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER
```
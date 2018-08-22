Add `SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", SYMLINK+="ttyAcousticMagic"` to `/etc/udev/rules.d/50-usb-serial.rules`.
Do `sudo usermod -a -G audio $USER` to allow the user to use audio device even if not logged in.

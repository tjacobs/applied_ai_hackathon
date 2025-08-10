#pip install python-can[serial]
python3 -m pip install --user "python-can==4.4.2"

# Allow access to serial port
sudo usermod -aG dialout $USER
newgrp dialout
groups
ls -l /dev/ttyACM0


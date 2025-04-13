#! /bin/bash

# Debian & Ubuntu -> sudo apt install bluez* rfcomm
# Fedora -> sudo dnf install bluez bluez-tools rfcomm

# Default passwor: 1234

XM_15B_MAC=00:11:35:81:60:85

sudo systemctl enable bluetooth.service
sudo systemctl start bluetooth.service
bluetoothctl power on
bluetoothctl scan on
sleep 1
bluetoothctl pair $XM_15B_MAC
bluetoothctl trust $XM_15B_MAC
bluetoothctl connect $XM_15B_MAC
sudo rfcomm bind rfcomm0 $XM_15B_MAC
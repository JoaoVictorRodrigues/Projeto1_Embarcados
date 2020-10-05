# -*- coding: utf-8 -*-
import pyautogui
import serial
import argparse
import time
import logging
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
import math
import sys
import signal

# Get default audio device using PyCAW
devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(
    IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))

# Get current volume - volume.GetMasterVolumeLevel()
currentVolumeDb = 0.0
# volume.SetMasterVolumeLevel(currentVolumeDb - 6.0, None)
# NOTE: -6.0 dB = half volume !


class MyControllerMap:
    def __init__(self):
        self.button = {'1': 'left', '2' : 'space', '3':'right'} # Fast forward (10 seg) pro Youtube

class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay


    def update(self):
        ## Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))

        id = self.ser.read()
        status = self.ser.read()

        logging.debug("Received DATA: {}".format(status))

        if id.decode() == 'S':
            self.ser.write(b'X')
        elif id.decode() == 'v':
            print(status)
            print (currentVolumeDb)
            if status == b'0':
              #vol_inten = min(1.0, max(0.0, 0.5))
              volume.SetMasterVolumeLevel(-74.0, None)
            if status == b'1':
              #vol_inten = min(1.0, max(0.0, 0.5))
              volume.SetMasterVolumeLevel(-23.9, None)
            if status == b'2':
              #vol_inten = min(1.0, max(0.0, 0.5))
              volume.SetMasterVolumeLevel(-17.9, None)
            if status == b'3':
              #vol_inten = min(1.0, max(0.0, 0.5))
              volume.SetMasterVolumeLevel(-13.8, None)
            if status == b'4':
              #vol_inten = min(1.0, max(0.0, 0.5))
              volume.SetMasterVolumeLevel(-10.4, None)
            if status == b'5':
              #vol_inten = min(1.0, max(0.0, 0.5))
              volume.SetMasterVolumeLevel(-7.8, None)
            if status == b'6':
              #vol_inten = min(1.0, max(0.0, 0.5))
              volume.SetMasterVolumeLevel(-5.4, None)
            if status == b'7':
              #vol_inten = min(1.0, max(0.0, 0.5))
              volume.SetMasterVolumeLevel(-3.4, None)
            if status == b'8':
              #vol_inten = min(1.0, max(0.0, 0.5))
              volume.SetMasterVolumeLevel(-1.6, None)
            if status == b'9':
              #vol_inten = min(1.0, max(0.0, 0.5))
              volume.SetMasterVolumeLevel(0.0, None)
        else:
            if status == b'1':
                logging.info("KEYDOWN A")
                pyautogui.keyDown(self.mapping.button[id.decode()])
            elif status == b'0':
                logging.info("KEYUP A")
                pyautogui.keyUp(self.mapping.button[id.decode()])

        self.incoming = self.ser.read()
  

class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['A'])
        time.sleep(0.1)
        pyautogui.keyUp(self.mapping.button['A'])
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    signal.signal(signal.SIGINT, signal_handler)
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()

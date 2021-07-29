#!/usr/bin/env python

# Serial Driver for IKA RCT Digital Heater-Stirrer
# Uses USB Virtual COM communication to send commands and receive messages
# Made by Jakub Glowacki 27/07/2021

import time
import serial
import re


class IKADriver:
    serialCom = serial.Serial()  # Globally define serial communication

    def __init__(self):  # Init function starts serial communication
        global serialCom
        serialCom = serial.Serial(  # Initialize serial communication object
            port='/dev/ttyACM0',
            baudrate=9600,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.SEVENBITS,
            timeout=1
        )

    # Commands are defined in IKA datasheet, however need to be sent over serial as
    # ASCII encoded byte arrays and must end with a carriage return and line break to
    # be recognized. Received messsages can also be decoded then to unicode strings.

    def startHeat(self):
        # Turn on heating element
        global serialCom
        serialCom.write(bytearray("START_1\r\n", "ascii"))
        return True

    def stopHeat(self):
        # Turn off heating element
        global serialCom
        serialCom.write(bytearray("STOP_1\r\n", "ascii"))
        return True

    def startStir(self):
        # Turn on stirring rod motor
        global serialCom
        serialCom.write(bytearray("START_4\r\n", "ascii"))
        return True

    def stopStir(self):
        # Turn off stirring rod motor
        global serialCom
        serialCom.write(bytearray("STOP_4\r\n", "ascii"))
        return True

    def setHeat(self, heat):
        # Set hotplate temperature
        global serialCom
        serialCom.write(bytearray("OUT_SP_1 " + str(heat) + "\r\n", "ascii"))
        return True

    def setStir(self, stir):
        # set stirrer speed
        global serialCom
        serialCom.write(bytearray("OUT_SP_4 " + str(stir) + "\r\n", "ascii"))
        return True

    def getHotplateTemp(self):
        # get temperature of hotplate by pushing a serial command and then reading the response over serial and using regex to extract the number
        global serialCom
        serialCom.write(bytearray("IN_PV_2\r\n", "ascii"))
        x = serialCom.read_until("\n")  # Read response
        stringx = str(x.decode('ascii'))  # Decode response
        # Use Regex to extract only
        s = re.findall(
            "[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", stringx)
        # numbers
        return s[0]

    def getExternalTemp(self):
        # get external probe temperature by pushing a serial command and then reading the response over serial and using regex to extract the number
        global serialCom
        serialCom.write(bytearray("IN_PV_1\r\n", "ascii"))
        x = serialCom.read_until("\n")  # Read response
        stringx = str(x.decode('ascii'))  # Decode response
        # Use Regex to extract only
        s = re.findall(
            "[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", stringx)
        # numbers
        return s[0]

    def getStirringSpeed(self):
        # get stirring speed by pushing a serial command and then reading the response over serial and using regex to extract the number
        global serialCom
        serialCom.write(bytearray("IN_PV_4\r\n", "ascii"))
        x = serialCom.read_until("\n")  # Read response
        stringx = str(x.decode('ascii'))  # Decode response
        # Use Regex to extract only
        s = re.findall(
            "[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", stringx)
        # numbers
        return s[0]

    def getViscosityTrend(self):
        # get stirring viscosity trend by pushing a serial command and then reading the response over serial and using regex to extract the number
        global serialCom
        serialCom.write(bytearray("IN_PV_5\r\n", "ascii"))
        x = serialCom.read_until("\n")  # Read response
        stringx = str(x.decode('ascii'))  # Decode response
        # Use Regex to extract only
        s = re.findall(
            "[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", stringx)
        # numbers
        return s[0]

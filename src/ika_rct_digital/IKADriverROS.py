
# ROS Wrapper for Fisher Scientific PPS4102 Top Pan Balance Serial Driver
# Utilises ROS Topics to facilitate communication with balance using a serial driver
# Made by Jakub Glowacki 27/07/2021

import rospy
from ika_rct_digital.msg import IKAReading
from ika_rct_digital.msg import IKACommand
from ika_rct_digital.IKADriverSerial import IKADriver


class IKADriverROS:

    def __init__(self):
        global pub
        self.IKA = IKADriver()  # Create object of BalanceDriver class, for serial communication
        # Initialize ros subscriber of topic to which commands are published
        rospy.Subscriber("IKA_Commands", IKACommand, self.callback_commands)
        # Initialize ros published for Balance responses (weights)
        pub = rospy.Publisher("IKA_Readings", IKAReading, queue_size=10)
        rate = rospy.Rate(1)
        rospy.loginfo("IKA driver started")
        while not rospy.is_shutdown():
            tempPlate = self.IKA.getHotplateTemp()
            tempExt = self.IKA.getExternalTemp()
            stirSpeed = self.IKA.getStirringSpeed()
            visc = self.IKA.getViscosityTrend()
            pub.publish(float(tempPlate), float(tempExt), float(stirSpeed), float(visc))
            print("Hotplate Temperature: " + str(tempPlate) + "| External Temperature: " + str(tempExt) + "| Stirring Speed: " + str(stirSpeed) + "| Viscosity Trend: " + str(visc))
            rate.sleep()

        # Call upon appropriate function in driver for any possible command

    def startHeat(self):
        self.IKA.startHeat()
        rospy.loginfo("Turning on Heating")

    def stopHeat(self):
        self.IKA.stopHeat()
        rospy.loginfo("Turning off Heating")

    def startStir(self):
        self.IKA.startStir()
        rospy.loginfo("Turning on Stirring")

    def stopStir(self):
        self.IKA.stopStir()
        rospy.loginfo("Turning off Stirring")

    def setStir(self, stir):
        self.IKA.setStir(stir)
        rospy.loginfo("Setting Stirring To: " + str(stir) + "RPM")

    def setHeat(self, heat):
        self.IKA.setHeat(heat)
        rospy.loginfo("Setting Heating To: " + str(heat) + "C")

    # Callback for subscriber. Calls correct function depending on command received
    def callback_commands(self, msg):
        if(msg.ika_command == msg.HEATON):
            self.startHeat()
        elif(msg.ika_command == msg.HEATOFF):
            self.stopHeat()
        elif(msg.ika_command == msg.STIRON):
            self.startStir()
        elif(msg.ika_command == msg.STIROFF):
            self.stopStir()
        elif(msg.ika_command == msg.SETSTIR):
            self.setStir(msg.ika_param)
        elif(msg.ika_command == msg.SETHEAT):
            self.setHeat(msg.ika_param)
        elif(msg.ika_command == msg.STIRAT):
            self.setStir(msg.ika_param)
            self.startStir()
        elif(msg.ika_command == msg.HEATAT):
            self.setHeat(msg.ika_param)
            self.startHeat()
        elif(msg.ika_command == msg.ALLOFF):
            self.stopHeat()
            self.stopStir()
            self.setStir(0)
            self.setHeat(0)
        else:
            rospy.loginfo("invalid command")


print("working")

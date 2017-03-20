#!/usr/bin/env python

# Copyright (c) 2017-2018 mwlwlm. 
# All right reserved.
#

## @file controllers.py Base class and support functions for a controllers.

## @brief Controllers interact with ArbotiX hardware.
class Controller:

    ## @brief Constructs a Controller instance.
    ##
    ## @param device The arbotix instance.
    ## 
    ## @param name The controller name.
    def __init__(self, arduino, name):
        self.name = name
        self.arduino = arduino
        self.pause = False

        # output for joint states publisher
        self.joint_names = list()
        self.joint_positions = list()
        self.joint_velocities = list()

    ## @brief Start the controller, do any hardware setup needed.
    def startup(self):
        pass

    ## @brief Do any read/writes to device.
    def update(self):
        pass

    ## @brief Stop the controller, do any hardware shutdown needed.
    def shutdown(self):
        pass

    ## @brief Is the controller actively sending commands to joints?
    def active(self):
        return False
        
    ## @brief Get a diagnostics message for this joint.
    ##
    ## @return Diagnostics message. 
    def getDiagnostics(self):
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        return msg


#!/usr/bin/env python3

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import fibre
print("finding an odrive...")
my_drive = odrive.find_any()


##   FOR 12V 5A
##      current control
my_drive.axis0.motor.config.current_lim_margin = 12 
my_drive.axis0.motor.config.current_lim = 5
##      power input
my_drive.config.dc_max_positive_current = 22 ## 5A
my_drive.config.dc_max_negative_current = -1

##      brake resistance 
my_drive.config.enable_brake_resistor = True
my_drive.config.brake_resistance = 2

##      Motor config
my_drive.axis0.motor.config.pole_pairs = 20
my_drive.axis0.motor.config.torque_constant = 8.27 / 90 
my_drive.axis0.motor.config.resistance_calib_max_voltage = 4
my_drive.axis0.motor.config.requested_current_range = 40
my_drive.axis0.motor.config.current_control_bandwidth = 500
my_drive.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

##      Encoder config 
my_drive.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
my_drive.axis0.encoder.config.cpr = 4096*4
my_drive.axis0.encoder.config.bandwidth = 100

##      Controller PID
my_drive.axis0.controller.config.pos_gain = 4
my_drive.axis0.controller.config.vel_gain = 0.137
my_drive.axis0.controller.config.vel_integrator_gain = 1.37
my_drive.axis0.controller.config.vel_limit = 31


try:
    my_drive.axis0.requested_state = AXIS_STATE_IDLE
    my_drive.save_configuration()
except fibre.libfibre.ObjectLostError:
    pass # Saving configuration makes the device reboot
# my_drive.save_configuration()
my_drive = odrive.find_any()


for i in range(2):
    
    if(my_drive.axis0.motor.config.pre_calibrated == False) :
        print("starting calibration...")
        my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        my_drive.axis0.motor.config.pre_calibrated = True
        while my_drive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
    if(my_drive.axis0.encoder.config.pre_calibrated == False):
        print("starting calibration...")
        my_drive.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        while my_drive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        my_drive.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        
        while my_drive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        my_drive.axis0.encoder.config.pre_calibrated = True

    try:
        my_drive.axis0.requested_state = AXIS_STATE_IDLE
        my_drive.save_configuration()
    except fibre.libfibre.ObjectLostError:
        pass # Saving configuration makes the device reboot
    my_drive = odrive.find_any()

my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive.axis0.config.startup_closed_loop_control = True

try:
    my_drive.axis0.requested_state = AXIS_STATE_IDLE
    my_drive.reboot()
except fibre.libfibre.ObjectLostError:
    pass # Saving configuration makes the device reboot
my_drive = odrive.find_any()
print("finished")
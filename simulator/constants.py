# -*- coding: utf-8 -*-
######################### Some constants for the LoRaSim3 Simulator ##########################
######################### https://github.com/Guillaumegaillard/CANL-LoRa #####################
######################### 2023-06-30 #########################################################
######################### unguaranteed public version ########################################


import numpy as np


Ptx_2dot4GHz = 10                   # transmission power for 2.4GHz scenarii 
Ptx_subGHz = 14                     # transmission power for 868MHz scenarii

gamma = 3                           # Path loss exponent for ED-ED links
gamma_GW=2.95                       # Path loss exponent for ED-GW links

sigma_gamma_ED=.25                  # standard deviation for path loss exponent of ED-ED links

d0 = 40.0                           # Path loss ref. dist
var = 0                             # variance ignored for now
Lpld0 = 83                          # Path loss at ref. dist
GL = 0                              # ED antenna gain
GL_GW = 1.5                         # GW antenna gain (dB)

################ A different path-loss model for CAD
CAD_gamma = 2.08*1.1
CAD_d0 = 40.0
CAD_var = 0                         # variance ignored for now
CAD_Lpld0 = 127.41*1.1
CAD_GL = 0

CAD_a=26.236634985744626
CAD_b=3930.0719485605073
###############


########################## ENERGY CONSTANTS
# Transmit consumption in mA from -2 to +17 dBm
# TODO: for lora24GHz
TX = [
    22, 22, 22, 23,                                             # RFO/PA0: -2..1
    24, 24, 24, 25, 25, 25, 25, 26, 31, 32, 34, 35, 44,         # PA_BOOST/PA1: 2..14
    82, 85, 90,                                                 # PA_BOOST/PA1: 15..17
    105, 115, 125]                                              # PA_BOOST/PA1+PA2: 18..20

# SX1262_dsV2_1
TX_868_14dBm=[45]*23                                            # mA

#use 5mA for receive consumption in mA. This can be achieved by SX126X LoRa chip
RX = 5
RX_boosted_LDRO_mode=10.1                                       # mA, max value
RX_boosted_DCDC_mode=5.3                                        # mA, considered energy efficient value
V = 3.3                                                         # voltage XXX

#only for BW125, in nAh from SF7 to SF12
#based on SX1262 Semtech's AN on CAD performance
cad_consumption = {
    "SF7":{
        "BW125":{
            "1S":1.73,
            "2S":2.84,
            "4S":5.03,
            "8S":9.41,
            "16S":18.16,
        },
        "BW500":{
            "1S":0.502,
            "2S":0.81,
            "4S":1.43,
            "8S":2.62,
            "16S":4.97,
        },
    },
    "SF12":{
        "BW125":{
            "1S":64.59,
            "2S":99.57,
            "4S":169.54,
            "8S":309.50,
            "16S":589.39,
        },
        "BW500":{
            "1S":16.15,
            "2S":24.89,
            "4S":42.39,
            "8S":77.38,
            "16S":147.35,
        },        
    },
}


########################## SENSITIVITIES

# this is an array with values for sensitivity
# see SX128X Semtech doc
# BW in 203, 406, 812, 1625 kHz
sf5_2dot4GHz = np.array([5,-109.0,-107.0,-105.0,-99.0])
sf6_2dot4GHz = np.array([6,-111.0,-110.0,-118.0,-103.0])
sf7_2dot4GHz = np.array([7,-115.0,-113.0,-112.0,-106.0])
sf8_2dot4GHz = np.array([8,-118.0,-116.0,-115.0,-109.0])
sf9_2dot4GHz = np.array([9,-121.0,-119.0,-117.0,-111.0])
sf10_2dot4GHz = np.array([10,-124.0,-122.0,-120.0,-114.0])
sf11_2dot4GHz = np.array([11,-127.0,-125.0,-123.0,-117.0])
sf12_2dot4GHz = np.array([12,-130.0,-128.0,-126.0,-120.0])

#taken for spec
sf6_subGHz = np.array([6,-118.0,-115.0,-111.0])
# this is an array with measured values for sensitivity
# see Table 1 in Bor, M., Roedig, U., Voigt, T., Alonso, J. M. (2016). Do LoRa low-power wide-area networks scale? MSWiM 2016, 59–67. https://doi.org/10.1145/2988287.2989163
# BW in 125, 250, 500 kHz
sf7_subGHz = np.array([7,-126.5,-124.25,-120.75])
sf8_subGHz = np.array([8,-127.25,-126.75,-124.0])
sf9_subGHz = np.array([9,-131.25,-128.25,-127.5])
sf10_subGHz = np.array([10,-132.75,-130.25,-128.75])
sf11_subGHz = np.array([11,-134.5,-132.75,-128.75])
sf12_subGHz = np.array([12,-133.25,-132.25,-132.25])

sensi_2dot4GHz = np.array([sf5_2dot4GHz,sf6_2dot4GHz,sf7_2dot4GHz,sf8_2dot4GHz,sf9_2dot4GHz,sf10_2dot4GHz,sf11_2dot4GHz,sf12_2dot4GHz])
sensi_subGHz = np.array([sf6_subGHz,sf7_subGHz,sf8_subGHz,sf9_subGHz,sf10_subGHz,sf11_subGHz,sf12_subGHz])
        

########################## NOISE CONSTANTS
noise_mu_dB, noise_sigma_dB = 3, 3                                      # mean and standard deviation, dB
noise_mu, noise_sigma = 10**(.1*noise_mu_dB), 10**(.1*noise_sigma_dB)   # mean and standard deviation, mW

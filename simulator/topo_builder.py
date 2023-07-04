# -*- coding: utf-8 -*-
######################### Module to build topologies for the LoRaSim3 Simulator ##############
######################### https://github.com/Guillaumegaillard/CANL-LoRa #####################
######################### 2023-06-30 #########################################################
######################### unguaranteed public version ########################################

import numpy as np
import math
import constants
import sys

rng = np.random.default_rng()

# A switch for further computations
lora24GHz = False


# build a uniformly spread 2D topo based on: 
## a number of devices *nb_nodes*
## a PHY layer setting *experiment*
## an optional maximum distance to the center, a GW, *static_maxDist*
#### default is max. range of a noiseless GW-ED link
def build_topo(nb_nodes,experiment,static_maxDist=0):
    topo={}

    # no max has been given
    if static_maxDist==0:
        if lora24GHz:
            Ptx=constants.Ptx_2dot4GHz
        else:
            Ptx=constants.Ptx_subGHz

        if lora24GHz:
            if experiment in [0,1,4,6,7]:
                minsensi = constants.sensi_2dot4GHz[7,2]     # 7th row is SF12, 2nd column is BW203
            elif experiment == 2:
                minsensi = constants.sensi_2dot4GHz[0,5]     # row 0 is SF5, 5th column is BW1625
            elif experiment in [3,5]:
                minsensi = np.amin(constants.sensi_2dot4GHz) ## Experiment 3 can use any setting, so take minimum
        else:
            if experiment in [0,1,4,6,7]:
                minsensi = constants.sensi_subGHz[6,2]     # 6th row is SF12, 2nd column is BW125
            elif experiment == 2:
                minsensi = constants.sensi_subGHz[0,4]     # first row is SF6, 4th column is BW500
            elif experiment =="SF7BW500CAD4":
                minsensi = constants.sensi_subGHz[1,3]     # second row is SF7, 4th column is BW500
            elif experiment in [3,5]:
                minsensi = np.amin(constants.sensi_subGHz) ## Experiment 3 can use any setting, so take minimum
            
        Lpl = Ptx - minsensi
        print("amin", minsensi, "Lpl", Lpl)
        maxDist = constants.d0*(math.e**((Lpl-constants.Lpld0)/(10.0*constants.gamma_GW)))
        
        topo["amin"]=minsensi
        topo["Lpl"]=Lpl
        topo["maxDist"]=maxDist
    else:
        maxDist = static_maxDist 
        topo["maxDist"]=maxDist

    # base station (GW) placement
    bsx = maxDist+10
    bsy = maxDist+10

    topo["GW"]={
        "bsx":bsx,
        "bsy":bsy,
    }


    # devices get a minimum inter distance (no overlap)
    min_inter_dist=10/nb_nodes*20

    nodes=[]
    for i in range(0,nb_nodes):
        # this is a prodecure for placing nodes
        # and ensure minimum distance between each pair of nodes
        found = 0
        rounds = 0
        while (found == 0 and rounds < 100):
            a = .99*rng.random()+0.01 #avoid log10(0)/dividebyzero
            b = .99*rng.random()+0.01 #avoid log10(0)/dividebyzero
            if b<a:
                a,b = b,a
            posx = b*maxDist*math.cos(2*math.pi*a/b)+bsx
            posy = b*maxDist*math.sin(2*math.pi*a/b)+bsy
            if len(nodes) > 0:
                for index, n in enumerate(nodes):
                    dist = np.sqrt(((abs(n["x"]-posx))**2)+((abs(n["y"]-posy))**2))
                    if dist < min_inter_dist:
                        rounds = rounds + 1
                        if rounds == 100:
                            print("could not place new node, giving up")
                            exit(-1)
                        break
                if index==len(nodes)-1:
                    found = 1
                    x = posx
                    y = posy
            else:
                # print("first node")
                x = posx
                y = posy
                found = 1
        nodes.append({"id":i,"x":x,"y":y})                

    # store coordinates in the topology dict
    topo["nodes"]={}
    for node in nodes:
        topo["nodes"][node["id"]]={
            "x":node["x"],
            "y":node["y"], 
        }

    return topo


if __name__ == '__main__':
    nb_nodes=20
    experiment=4

    # topopo=build_topo(nb_nodes,experiment,static_maxDist=0)
    topopo=build_topo(nb_nodes,experiment,static_maxDist=100)
    print(topopo)
    # print(0/0)


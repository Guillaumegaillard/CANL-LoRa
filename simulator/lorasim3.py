# -*- coding: utf-8 -*-
######################### LoRaSim3 Simulator #################################################
######################### Inspired from LoRaSim https://mcbor.github.io/lorasim/ #############
######################### but in Python3, and with more features #############################
######################### https://github.com/Guillaumegaillard/CANL-LoRa #####################
######################### 2023-06-30 #########################################################
######################### unguaranteed public version ########################################


import simpy
import numpy as np
import math
import sys
import os
import pickle
import logging

import constants

if not os.path.exists('results'):
    os.makedirs('results')

rng = np.random.default_rng()




#############################################
# Simulation Core Technical Things #
#############################################

#turn on/off print
#disabling printing to stdout will make simulation much faster
print_sim = True
print_sim = False
stdout_print_target=sys.stdout

#disable printing to stdout to make simulation much faster
if print_sim==False:
    f = open('/dev/null', 'w')
    sys.stdout = f    

#turn on/off old graphics
graphics = 0


################################
##############
# Local Constants      #
##############


#packet type
dataPacketType=1
rtsPacketType=2

#node's states to control the CANL22 mechanism
schedule_tx=0
want_transmit=1

CANL_listen1=10
CANL_NAV_state=11
CANL_send_RTS=12
CANL_listen2=13
CANL_send_DATA=14
CANL_CAD=15

#node's states to control the ideal mechanism
Ideal_send_DATA=16

#node type, you can add whatever you want to customize the behavior on configuration
endDeviceType=1
relayDeviceType=2

#packet transmit interval distribution type
expoDistribType=1
uniformDistribType=2
perioDistribType=3

bsId = 1

################################
##### FUNCTIONS
################################

### If log_events is acitvated, the call to this function will initiate the logs process
def load_main_logger(start_time):
    
    ###########LOGGING MAIN EVENTS
    MainLogger = logging.getLogger('Main_Logger')
    MainLogger.setLevel(logging.INFO)
    # create file handler which logs even debug messages
    mfh = logging.FileHandler('{0}.log'.format(start_time), mode='w')
    mfh.setLevel(logging.INFO)
    #console
    mch = logging.StreamHandler()
    mch.setLevel(logging.CRITICAL)
    # create formatter and add it to the handlers
    mformatter = logging.Formatter('%(asctime)s %(message)s')
    mfh.setFormatter(mformatter)
    mch.setFormatter(mformatter)
    
    close_logger_handlers(MainLogger)
    
    # add the handlers to logger
    MainLogger.addHandler(mfh)
    MainLogger.addHandler(mch)
    
    return (MainLogger)

### shut down logging when experiment stops
def close_logger_handlers(logger_obj):
    handlers = logger_obj.handlers[:]
    for handler in handlers:
        handler.close()
        logger_obj.removeHandler(handler)



############################
############################
# check for collisions at base station and among devices
# Note: called before a packet (or rather node) is inserted into the list
def checkcollision(packet):
    col = 0 # flag needed since there might be several collisions for packet

    # if packet is detcted at GW
    if not packet.lost:
        ### packet.processed is not used in this version
        processing = 0
        for i in range(0,len(packetsOnAir)):
            if packetsOnAir[i].packet.processed == 1 :
                processing = processing + 1
        if (processing > maxBSReceives):
            # print("too long:", len(packetsOnAir))
            packet.processed = 0
        else:
            packet.processed = 1


        # if somebody else is transmitting
        if len(packetsOnAir)>0:
            in_ears=1 # packet is not in list but will be part of the mess
            for other in packetsOnAir:
                if (other.packet.rssi > get_sensitivity(other.packet.sf,other.packet.bw)):
                    in_ears+=1

            for other in packetsOnAir:
                #this other packet is different than packet and heard at GW
                if other.nodeid != packet.nodeid and (other.packet.rssi > get_sensitivity(other.packet.sf,other.packet.bw)):
                    # simple collision
                    if frequencyCollision(packet, other.packet) and sfCollision(packet, other.packet):
                        if full_collision:
                            # check who collides in the power domain
                            c = powerCollision(packet, other.packet, in_ears=in_ears)
                            # either this one, the other one, or both

                            # mark all the collided packets
                            if packet in c:
                                if timingCollision(packet, other.packet):# both_collide, or just the other?
                                    col = 1
                                    packet.collided = 1
                                    if log_events:
                                        MainLogger.info(("GW","col",packet.nodeid,packet.nodeid,env.now))
                            if other.packet in c:
                                other.packet.collided = 1
                                if log_events:
                                    MainLogger.info(("GW","col",packet.nodeid,other.packet.nodeid,env.now))
                        else:
                            packet.collided = 1
                            other.packet.collided = 1     # other also got lost, if it wasn't lost already
                            if log_events:
                                MainLogger.info(("GW","col",packet.nodeid,packet.nodeid,env.now))
                                MainLogger.info(("GW","col",packet.nodeid,other.packet.nodeid,env.now))
                            col = 1            

            #This protocol has a listening phase impaired by a local collision            
            if CANL22:
                if full_distances:
                    # here the packet can be heard at GW but unheard by some others. 
                    # so we need to check if heard by listeners and the collisions/captures there
                    #done right afterwards
                    pass
                else:
                    # here we assume heards at GW are heard everywhere. Collisions at GW impair decoding at listeners, but energy is still detected.
                    #we have to correct previous decision as previous RTS or DATA packets can be now marked as collided
                    #their state can still be listening, we just cancel the fact that they received an RTS or DATA
                    for other in packetsOnAir:
                        for node in nodes:
                            if log_events:
                                MainLogger.info((node.nodeid,"col",packet.nodeid,other.nodeid,env.now))
            

    ############### Packet has been received at GW. Let's see elsewhere
    if not full_distances:    
        #if col==1 it means that the new packet can not be decoded
        if col:
            return col    
            
    #normally, here, the packet has been correctly received at GW (not collided)
    if not col:
        if log_events:
            MainLogger.info(("GW","rx",packet.nodeid,env.now))

    # old "everywhere is the same as GW" case 
    if not full_distances:   
        # not collided at GW, packet is alone on air 
        if CANL22:
            #the trick is to assume that if the gateway received a packet
            #then all other nodes in the listening period should also have receive it
            #there might be some cases where a geographically central gw would have received a packet while a distant node,
            #far from the transmitter node might not receive the packet. But here we assume that the distances allow such reception
            for node in nodes:
                if node.nodeid != packet.nodeid:
                    #node is listenning
                    if node.ca_state in [CANL_listen1, CANL_listen2]:

                        # mark the packet in list of receptions during listenning 
                        # depending on the scenario in terms of headers, the data payload size could be known, unknown or mistaken by the receiver
                        # dataPayloadSize_in_RTS is used only in a scenario with RTS 
                        # packet could be partially captured by/capturing other packets on air - checked later wrt times & powers 
                        node.heard_frames.append({
                            "id":packet.nodeid,
                            "toa":packet.rectime,
                            "is_RTS":packet.ptype == rtsPacketType,
                            "dataPayloadSize_in_RTS":rng.integers(0,max_payload_size+1) if packet.dataPayloadSize==(CANL_rts_hdr_size+1) else packet.dataPayloadSize, # if Data of RTS size, random size
                            "dataPayloadSize_in_EH":packet.dataPayloadSize, # in explicit header
                            "start":env.now,
                            "capturing":[p.nodeid for p in packetsOnAir],
                            "captured_by":[p.nodeid for p in packetsOnAir],
                            })
                        if log_events:
                            MainLogger.info((node.nodeid,"rx",packet.nodeid,env.now))

    else:
        # # general case 
        # for each node listening
        #     if heard
        #         for other packets on air 
        #             if collide?Capture
        #                 adjust
        #         impact/or not

        if CANL22:
            for node in nodes:
                locally_collided=False
                previous_frames_impacted_by_this_one=[]
                previous_frames_impacting_this_one=[]
                if node.ca_state in [CANL_listen1, CANL_listen2]:
                    if check_heard(packet,node.nodeid):
                        in_ears=1 # packet is in ears
                        for other in packetsOnAir:
                            if check_heard(other.packet,node.nodeid):
                                in_ears+=1                        
                        for other in packetsOnAir:
                            if check_heard(other.packet,node.nodeid):
                                if frequencyCollision(packet, other.packet) and sfCollision(packet, other.packet):
                                    if full_collision:
                                        # check who collides in the power domain
                                        c = powerCollision(packet, other.packet, in_ears=in_ears,local=node.nodeid)
                                        # either this one, the other one, or both

                                        # mark all the collided packets
                                        if packet in c:
                                            if timingCollision(packet, other.packet):# both_collide, or just the other?
                                                locally_collided=True
                                                previous_frames_impacting_this_one.append(other.packet.nodeid)
                                                if log_events:
                                                    MainLogger.info((node.nodeid,"col",packet.nodeid,packet.nodeid,env.now))
                                        if other.packet in c:
                                            if log_events:
                                                MainLogger.info((node.nodeid,"col",packet.nodeid,other.packet.nodeid,env.now))
                                            previous_frames_impacted_by_this_one.append(other.packet.nodeid)
                                    else:
                                        if log_events:
                                            MainLogger.info((node.nodeid,"col",packet.nodeid,other.packet.nodeid,env.now))
                                            MainLogger.info((node.nodeid,"col",packet.nodeid,packet.nodeid,env.now))
                                        previous_frames_impacting_this_one.append(other.packet.nodeid)
                                        previous_frames_impacted_by_this_one.append(other.packet.nodeid)
                                        locally_collided=True

                        # see above
                        node.heard_frames.append({
                            "id":packet.nodeid,
                            "toa":packet.rectime,
                            "is_RTS":packet.ptype == rtsPacketType,
                            "dataPayloadSize_in_RTS":rng.integers(0,max_payload_size+1) if packet.dataPayloadSize==(CANL_rts_hdr_size+1) else packet.dataPayloadSize, # if Data of RTS size, random size
                            "dataPayloadSize_in_EH":packet.dataPayloadSize, # in explicit header
                            "start":env.now,
                            "capturing":previous_frames_impacted_by_this_one,
                            "captured_by":previous_frames_impacting_this_one,
                        })
                        
                        if not locally_collided:
                            if log_events:
                                MainLogger.info((node.nodeid,"rx",packet.nodeid,env.now))

    return col

# 
# check if a device perceives a neighbor frame above its sensitivity threshold 
def check_heard(packet,nodeid):
    rssi=packet.rx_array[nodeid]
    return (rssi >= get_sensitivity(packet.sf,packet.bw,receiver_type="DEVICE"))

#
# retrieve sensitivity according to scenario and dev type
def get_sensitivity(spreading_factor,bandwidth,receiver_type="GW"):
    if experiment == 4:
        if receiver_type=="DEVICE":
            return(-133.25)
        return(-138)
        # return(-150)
    if experiment == "SF7BW500CAD4":
        if receiver_type=="DEVICE":
            return(constants.sensi_subGHz[spreading_factor - 6, [125,250,500].index(bandwidth) + 1]) # -120.75 Why complicate things? :)
        return(-127)
        # return(-150)    
    if lora24GHz:
        sensitivity = constants.sensi_2dot4GHz[spreading_factor - 5, [203.125,406.25,812.5,1625].index(bandwidth) + 1]
    else:
        sensitivity = constants.sensi_subGHz[spreading_factor - 6, [125,250,500].index(bandwidth) + 1]

    return(sensitivity)




#
# frequencyCollision, conditions
#
#         |f1-f2| <= 120 kHz if f1 or f2 has bw 500
#         |f1-f2| <= 60 kHz if f1 or f2 has bw 250
#         |f1-f2| <= 30 kHz if f1 or f2 has bw 125
def frequencyCollision(p1,p2):
    if (abs(p1.freq-p2.freq)<=120 and (p1.bw==500 or p2.freq==500)):
        return True
    elif (abs(p1.freq-p2.freq)<=60 and (p1.bw==250 or p2.freq==250)):
        return True
    else:
        if (abs(p1.freq-p2.freq)<=30):
            return True
    # print("no frequency coll")
    return False

#
# Check if two packet are transmitted with same spreading factor
def sfCollision(p1, p2):
    if p1.sf == p2.sf:
        return True
    # print("no sf collision")
    return False

#
# Check and store the reception conditions of two packets 
# at a given device (local=devID) // at GW (default local=-1)
# depends on how many transmissions are perceived at receiver at the time
def powerCollision(p1, p2, in_ears=2,local=-1):
    # powerCaptureThreshold = 6# dB
    PCT=powerCaptureThreshold # global to local
    if in_ears>2:
        PCT+= 2*(in_ears-2)
    
    if local!=-1:
        rssi1=p1.rx_array[local]
        rssi2=p2.rx_array[local]
    else:
        rssi1=p1.rssi
        rssi2=p2.rssi

    if abs(rssi1 - rssi2) < PCT:
        # packets are too close to each other, both collide
        # return both packets as casualties
        powerCaptures.append((in_ears,False,local))
        return (p1, p2)
    elif rssi1 - rssi2 < PCT:
        # p2 overpowered p1, return p1 as casualty
        powerCaptures.append((in_ears,True,local))
        return (p1,)
    # print("p1 wins, p2 lost")
    # p2 was the weaker packet, return it as a casualty
    powerCaptures.append((in_ears,True,local))
    return (p2,)

#
# check the time concommitance of two transmissions
def timingCollision(p1, p2, ocurring_now=True):
    # assuming p1 is the freshly arrived packet and this is the last collision check
    # we know p2 is not finished (it would not be on air, if the contrary was true)
    # if we entered here, we know that p2 is in time collision

    # we've already determined that p1 is a weak packet, so the only
    # way we can win is by being late enough (only the first n - 5 preamble symbols overlap)

    # assuming minimum 3 preamble symbols to detect it
    Npream_min = 3
    Tpreamb = 2**p1.sf/(1.0*p1.bw) * (Npream_min)
    
    # check whether p2 ends in p1's critical section
    p2_end = p2.addTime + p2.rectime
    p1_cs = env.now + Tpreamb if ocurring_now else p1.addTime + Tpreamb 
    if p1_cs < p2_end:
        # p1 collided with p2 and lost
        # print("not late enough")
        return True
    # print("spared by the preamble")
    return False


#
# this function computes the airtime of a packet
# according to LoraDesignGuide_STD.pdf
def airtime(sf,cr,pl,bw,explicit=True):
    
    DE = 0         # low data rate optimization enabled (=1) or not (=0)
    Npream = 8     # number of preamble symbol (12.25     from Utz paper)

    if explicit: # Header
        H=0
    else:
        H=1

    if lora24GHz:
        Npream = 12
        H = 1         # header for variable length packet (H=1) or not (H=0)        
        if sf > 10:
            # low data rate optimization mandated for SF > 10
            DE = 1
        Tsym = (2.0**sf)/bw
        if sf < 7:
            Tpream = (Npream + 6.25)*Tsym
        else:
            Tpream = (Npream + 4.25)*Tsym
        #print("sf", sf, " cr", cr, "pl", pl, "bw", bw
        if sf >= 7:
            payloadSymbNB = 8 + math.ceil(max((8.0*pl+16-4.0*sf+8+20*H),0)/(4.0*(sf-2*DE)))*(cr+4)
        else:
            payloadSymbNB = 8 + math.ceil(max((8.0*pl+16-4.0*sf+20*H),0)/(4.0*(sf-2*DE)))*(cr+4)
        Tpayload = payloadSymbNB * Tsym
        return (Tpream + Tpayload)     
    else:
        # H = 0         # implicit header disabled (H=0) or not (H=1)
        # H = LoRa_PHY_HDR         # GG: explicit header (H=0) or implicit header (H=1)
        if bw == 125 and sf in [11, 12]:
            # low data rate optimization mandated for BW125 with SF11 and SF12
            DE = 1
        if sf == 6:
            # can only have implicit header with SF6
            # GG: I assume that with SF6, it is necessary to transmit an explicit header, aka the other way round. SX126X?
            # H = 1
            H = 0
        Tsym = (2.0**sf)/bw
        Tpream = (Npream + 4.25)*Tsym
        #print("sf", sf, " cr", cr, "pl", pl, "bw", bw
        payloadSymbNB = 8 + max(math.ceil((8.0*pl-4.0*sf+28+16-20*H)/(4.0*(sf-2*DE)))*(cr+4),0)
        Tpayload = payloadSymbNB * Tsym
        return(Tpream + Tpayload)
    
#
# the node (device) object definition
#
class myNode():
    # a node has an id, coordinates, a type (relay, endDevice...), a base station (gw), a traffic period, distribution, and payload size
    def __init__(self, nodeid, nodex, nodey, nodeType, bs, period, distrib, packetlen):
        self.nodeid = nodeid
        self.nodeType = nodeType        
        self.period = period
        self.distrib = distrib
        self.bs = bs
        self.x = nodex
        self.y = nodey

        # distance to GW
        self.dist = np.sqrt((self.x-bsx)*(self.x-bsx)+(self.y-bsy)*(self.y-bsy))
        
        # node has a packet
        self.packet = myPacket(self.nodeid, packetlen, self.dist)
        self.data_len=packetlen
        
        # packet has a time on air called rectime
        self.data_rectime = airtime(self.packet.sf,self.packet.cr,self.packet.pl,self.packet.bw,explicit=(LoRa_PHY_HDR==0))

        if CANL22:
            # if a RTS is used, its payload is 1
            self.rts_rectime = airtime(self.packet.sf,self.packet.cr,CANL_rts_hdr_size+1,self.packet.bw,explicit=(CANL_RTS_PHY_HDR==0))

            # time a device stays listening in order to obtain data length information
            if Interrupts_on_header_valid: # device is capable of using header right after the "header valid" interrupt
                self.wait_PHY_interrupt = airtime(self.packet.sf,self.packet.cr,0,self.packet.bw,explicit=(LoRa_PHY_HDR==0))
            else:
                self.wait_PHY_interrupt = self.rts_rectime + self.packet.symTime*nCadSym # I stay listening a margin to be sure to receive
        else: # RTS is not used anyway, let's consider a 5B length
            self.rts_rectime = airtime(self.packet.sf,self.packet.cr,5,self.packet.bw,explicit=(LoRa_PHY_HDR==0))
            # print("rectime for RTS packet ", self.rts_rectime)
            self.wait_PHY_interrupt = airtime(self.packet.sf,self.packet.cr,5,self.packet.bw,explicit=(LoRa_PHY_HDR==0))+131


        ################## NODE VARIABLES ##########################
        self.n_data_sent = 0
        self.n_rts_sent = 0
        self.n_data_success = 0 # successfully transmitted frames
        self.n_payload_gen = 0 # amount of generated bytes
        self.n_payload_sent = 0 # amount of sent bytes
        self.n_payload_success = 0 # amount of received bytes

        #### Variables for collision avoidance
        self.ca_state = schedule_tx # initial state, will change in transmit() function
        self.want_transmit_time=0
        self.ca_listen_start_time = 0
        self.ca_listen_end_time = 0
        self.total_listen_time = 0

        self.my_P=0
        self.backoff=0

        self.receive_rts=False
        self.receive_rts_time=0
        self.receive_rts_from=-1

        ## variables for listenning          
        self.heard_frames=[]
        self.add_rx_time_opportunities=1

        self.I_heard_preamble=False
        self.I_know_it_is_Data=False
        self.I_know_it_is_RTS=False
        self.next_payload_byte=0
        self.listened_time=-1

        self.nav=0
        self.CAD=False
        self.n_CAD=0

        # mechanism n_retry
        self.n_retry=n_retry
        self.total_retry=0

                    
        self.n_aborted=0
        self.n_collided=0
        self.n_lost=0
        self.n_dropped=0
        self.cycle=0

        self.latency=0
        self.success_latency=0
        self.min_success_latency=self.period*1000
        self.Wbusy_BE=Wbusy_BE

        self.gen_times=[0]
        
        # # graphics for node
        # # global graphics
        # if (graphics == 1):
        #     global ax
        #     ax.add_artist(plt.Circle((self.x, self.y), 2, fill=True, color='blue'))


    # node function called in CANL at beginning of listen phase
    # Check if packets on air are heard and impacting the node
    def start_listening(self):  
        # self.heard_frames=[]

        # for each packet on air oldest first 
        #     was it heard?
        #     has it been captured before even its begin?
        #     does it capture previous?



        if len(packetsOnAir)>0:
            in_ears=0
            for other in packetsOnAir:
                if check_heard(other.packet,self.nodeid):
                    in_ears+=1


            for pid in range(len(packetsOnAir)):
                packet=packetsOnAir[pid].packet
                if check_heard(packet,self.nodeid):
                    previous_frames_impacting_this_one=[]
                    previous_frames_impacted_by_this_one=[]      
                    locally_collided=False                  
                    if pid!=0: # more than one on air
                        for opid in range(pid):
                            other=packetsOnAir[opid]
                            if check_heard(other.packet,self.nodeid):
                                if frequencyCollision(packet, other.packet) and sfCollision(packet, other.packet):
                                    if full_collision:
                                        # check who collides in the power domain
                                        c = powerCollision(packet, other.packet,in_ears=in_ears,local=self.nodeid)
                                        # either this one, the other one, or both

                                        # mark all the collided packets
                                        if packet in c:
                                            if timingCollision(packet, other.packet, ocurring_now=False):# both_collide, or just the other?
                                                locally_collided=True
                                                previous_frames_impacting_this_one.append(other.packet.nodeid)
                                                if log_events:
                                                    MainLogger.info((self.nodeid,"col",packet.nodeid,packet.nodeid,env.now))
                                        if other.packet in c:
                                            if log_events:
                                                MainLogger.info((self.nodeid,"col",packet.nodeid,other.packet.nodeid,env.now))
                                            previous_frames_impacted_by_this_one.append(other.packet.nodeid)

                                    else:
                                        if log_events:
                                            MainLogger.info((self.nodeid,"col",packet.nodeid,other.packet.nodeid,env.now))
                                            MainLogger.info((self.nodeid,"col",packet.nodeid,packet.nodeid,env.now))
                                        previous_frames_impacting_this_one.append(other.packet.nodeid)
                                        previous_frames_impacted_by_this_one.append(other.packet.nodeid)
                                        locally_collided=True

                        self.heard_frames.append({
                            "id":packet.nodeid,
                            "toa":packet.rectime,
                            "is_RTS":packet.ptype == rtsPacketType,
                            "dataPayloadSize_in_RTS":rng.integers(0,max_payload_size+1) if packet.dataPayloadSize==(CANL_rts_hdr_size+1) else packet.dataPayloadSize, # if Data of RTS size, random size
                            "dataPayloadSize_in_EH":packet.dataPayloadSize, # in explicit header
                            "start":packet.addTime,
                            "capturing":previous_frames_impacted_by_this_one,
                            "captured_by":previous_frames_impacting_this_one,
                        })

                    else:
                        self.heard_frames.append({
                            "id":packet.nodeid,
                            "toa":packet.rectime,
                            "is_RTS":packet.ptype == rtsPacketType,
                            "dataPayloadSize_in_RTS":rng.integers(0,max_payload_size+1) if packet.dataPayloadSize==(CANL_rts_hdr_size+1) else packet.dataPayloadSize, # if Data of RTS size, random size
                            "dataPayloadSize_in_EH":packet.dataPayloadSize, # in explicit header
                            "start":packet.addTime,
                            "capturing":[],# what happenned prior to 0 is not on air anymore
                            "captured_by":[],# what happenned prior to 0 is not on air anymore, could have been captured just a few syms of preamble and be heard now
                        })

                    if not locally_collided:
                        if log_events:
                            MainLogger.info((self.nodeid,"rx",packet.nodeid,env.now))

    # node function called in CANL at the end of listen phase
    # Check what have been heard and if more time is needed to finish a reception of header
    def stop_listening(self):
        toyield=0 # additional time to yield before stopping, return value of stop_listening(self)
        heard_something=False
        min_preamb_heard=self.packet.symTime*3
        
        lhf=len(self.heard_frames)
        if lhf==0:#I heard nothing, go to next state (transmit)
            pass 
        else:
            for frame_heard_id in range(lhf):# in chrono order
                frame_heard=self.heard_frames[frame_heard_id]

                # was it captured from the beginning?
                if len(frame_heard["captured_by"])==0: # no
                    #did I hear it start?
                    time_since_frame_started=env.now-frame_heard["start"]
                    # (one of the two following condition is easy with the other when listentime>=preamble, otherwise TODO)
                    if (time_since_frame_started>min_preamb_heard 
                    and self.ca_listen_start_time<frame_heard["start"]+self.packet.Tpream-min_preamb_heard):#I heard it start 
                        # was it captured after the beginning? 
                        first_capturer=-1
                        for nextid in range(frame_heard_id+1,lhf): # last does not enter here
                            if frame_heard["id"] in self.heard_frames[nextid]["capturing"]:
                                first_capturer=nextid
                                break
                        if first_capturer!=-1: # yes
                            next_frame_capturing=self.heard_frames[first_capturer]
                            # but captured before preamble heard?
                            if frame_heard["start"]+self.packet.Tpream-min_preamb_heard<next_frame_capturing["start"]: #no, after
                                self.I_heard_preamble=True
                                heard_something=True
                                duration_heard=next_frame_capturing["start"]-frame_heard["start"]# it's been captured, so necessarily it's not finished before capture
                                # NB: begining of preamble has not been necessarily heard, but the following considers absolute time difference for simpler calculation
                                    # i.e. duration_heard should be called "clearly heard time + unheard portion of preamble, if any", or so...
                                # did I hear enough before capture so that I know more about data/rts?
                                if duration_heard>self.wait_PHY_interrupt:#yes (could be similarly compared if duration heard after preamble > duration waited after preamble)
                                    # if it was RTS, it would have finished before capture (case wait_interrupt>RTS, but otherwise RTS is a data for what matters)
                                    self.I_know_it_is_Data=True
                                    if Interrupts_on_header_valid:
                                        self.next_payload_byte=frame_heard["dataPayloadSize_in_EH"]
                                    self.listened_time=frame_heard["start"]-self.ca_listen_start_time+self.wait_PHY_interrupt
                                else: # I don't know if its RTS or DATA, it has been captured before
                                    # would I have waited uselessly for an RXDOne? if it was captured, i would not know, I would still wait for interrupt
                                    if time_since_frame_started<self.wait_PHY_interrupt:
                                        if self.add_rx_time_opportunities>0:
                                            self.add_rx_time_opportunities-=1
                                            toyield=self.wait_PHY_interrupt-time_since_frame_started+1/100000 #add 10 ns to avoid floating point error
                                        else:#abandon listening
                                            self.listened_time=env.now-self.ca_listen_start_time
                                    else: # already waited
                                        self.listened_time=frame_heard["start"]-self.ca_listen_start_time+self.wait_PHY_interrupt
                                break

                        else: #frame has not been captured => same as alone
                            self.I_heard_preamble=True
                            heard_something=True
                            #did I hear it finish?
                            if time_since_frame_started>frame_heard["toa"]:#yes
                                if frame_heard["toa"]>=self.wait_PHY_interrupt:
                                    self.I_know_it_is_Data=True
                                    if Interrupts_on_header_valid:
                                        self.next_payload_byte=frame_heard["dataPayloadSize_in_EH"]
                                else:
                                    if frame_heard["is_RTS"]: #I know it by CANL header differentiation (data, RTS, ACK) 
                                        self.I_know_it_is_RTS=True
                                        self.next_payload_byte=frame_heard["dataPayloadSize_in_RTS"]
                                    else:
                                        self.I_know_it_is_Data=True
                                        if Interrupts_on_header_valid:
                                            self.next_payload_byte=frame_heard["dataPayloadSize_in_EH"]
                                        
                                self.listened_time=frame_heard["start"]-self.ca_listen_start_time+self.wait_PHY_interrupt
                                break
                            else:
                                # did I hear enough?
                                if time_since_frame_started>self.wait_PHY_interrupt:#yes
                                    self.I_know_it_is_Data=True
                                    if Interrupts_on_header_valid:
                                        self.next_payload_byte=frame_heard["dataPayloadSize_in_EH"]
                                    self.listened_time=frame_heard["start"]-self.ca_listen_start_time+self.wait_PHY_interrupt
                                else:# need to wait if possible
                                    if self.add_rx_time_opportunities>0:
                                        self.add_rx_time_opportunities-=1
                                        # yield env.timeout(self.wait_PHY_interrupt-min_preamb_heard)
                                        toyield=self.wait_PHY_interrupt-time_since_frame_started+1/100000 #add 10 ns to avoid floating point error
                                    else:#abandon listening
                                        self.listened_time=env.now-self.ca_listen_start_time
                                break
                        
        if not heard_something: # full listening period + potential prolong 
            self.listened_time=env.now-self.ca_listen_start_time

        return(toyield)




#
# this class creates a packet (associated with a node)
# it also sets all parameters
#
class myPacket():
    ## node ID, packet length, distance to GW
    def __init__(self, nodeid, plen, distance):

        self.nodeid = nodeid
        self.txpow = Ptx
        self.distance_to_GW=distance

        # randomize configuration values
        if lora24GHz:
            self.sf = rng.integers(5,13)
            self.bw = rng.choice([203.125, 406.250, 812.5, 1625])
        else:    
            self.sf = rng.integers(6,13)
            self.bw = rng.choice([125, 250, 500])
        self.cr = rng.integers(1,5)

        # for certain experiments override these
        if experiment==1 or experiment == 0:
            self.sf = 12
            self.cr = 4
            if lora24GHz:
                self.bw = 203.125
            else:    
                self.bw = 125

        # for certain experiments override these
        if experiment==2:
            if lora24GHz:
                self.sf = 5
                self.cr = 1
                self.bw = 1625            
            else:
                self.sf = 6
                self.cr = 1
                self.bw = 500
        # lorawan
        if experiment in [4,6,7]:
            if lora24GHz:
                self.bw = 203.125            
            else:
                self.bw = 125
            self.sf = exp4SF
            self.cr = 1    

        if experiment == "SF7BW500CAD4":
            self.bw = 500
            self.sf = 7
            self.cr = 1    # CR is 1, 2, 3 or 4 for respective coding rates 4/5, 4/6, 4/7 or 4/8

            

        # for experiment 3 find the best setting
        # OBS, some hardcoded values
        Prx = self.txpow    ## zero path loss by default

        # log-shadow at init
        Lpl = constants.Lpld0 + 10*gamma_GW*math.log10(distance/constants.d0)
        Prx = min(self.txpow, self.txpow + constants.GL_GW - Lpl)

        #TODO for lora24GHz
        if (experiment == 3) or (experiment == 5):
            minairtime = 9999
            minsf = 0
            minbw = 0

            # print("Prx:", Prx)

            for i in range(0,6):
                for j in range(1,4):
                    if (sensi[i,j] < Prx):
                        self.sf = int(sensi[i,0])
                        if j==1:
                            self.bw = 125
                        elif j==2:
                            self.bw = 250
                        else:
                            self.bw=500
                        at = airtime(self.sf, 1, plen, self.bw)
                        if at < minairtime:
                            minairtime = at
                            minsf = self.sf
                            minbw = self.bw
                            minsensi = sensi[i, j]
            if (minairtime == 9999):
                # print("does not reach base station")
                exit(-1)
            # print("best sf:", minsf, " best bw: ", minbw, "best airtime:", minairtime)
            self.rectime = minairtime
            self.sf = minsf
            self.bw = minbw
            self.cr = 1

            if experiment == 5:
                # reduce the txpower if there's room left
                self.txpow = max(2, self.txpow - math.floor(Prx - minsensi))
                Prx = self.txpow - GL - Lpl
                # print( 'minsesi {} best txpow {}'.format(minsensi, self.txpow))

        # transmission range, needs update XXX
        self.transRange = 150
        self.pl = plen
        self.symTime = (2.0**self.sf)/self.bw
        self.arriveTime = 0
        self.rssi = Prx
        
        # Path loss exponents to neighs
        self.gamma_array = np.zeros((distance_matrix[self.nodeid].shape))
        if normal_gamma_ED:
            self.gamma_array = rng.normal(gamma_ED,sigma_gamma_ED,self.gamma_array.shape)
        else:
            self.gamma_array += gamma_ED


        # frequencies: lower bound + number of 61 Hz steps
        if lora24GHz:
            self.freq = 2403000000 + rng.integers(0,2622951)
        else:
            self.freq = 860000000 + rng.integers(0,2622951)

        # for certain experiments override these and
        # choose some random frequences
        if experiment == 1:
            if lora24GHz:
                self.freq = rng.choice([2403000000, 2425000000, 2479000000])
            else:
                self.freq = rng.choice([860000000, 864000000, 868000000])
        else:
            if lora24GHz:
                self.freq = 2403000000
            else:
                self.freq = 860000000
    

        self.ptype = dataPacketType
        #self.data_len will keep the payload length of a data packet
        #self.pl will be used to keep the current packet length which can either be 
        #data_len or 5 (the size of an RTS packet)
        self.data_len=plen
        if lora24GHz:
            Npream = 12     
            if self.sf < 7:
                self.Tpream = (Npream + 6.25)*self.symTime
            else:
                self.Tpream = (Npream + 4.25)*self.symTime                 
        else:        
            Npream = 8     # number of preamble symbol (12.25     from Utz paper) 
            self.Tpream = (Npream + 4.25)*self.symTime        
        self.rectime = airtime(self.sf,self.cr,self.pl,self.bw)

        # denote if packet is collided
        self.collided = 0
        self.processed = 0 # not in use in the current version
        self.dataPayloadSize=self.data_len

        # set a new random rx array to all neighbors
        self.repropagate()

    #
    # give payload size according to distribution
    def setDataPayloadSize(self):
        if variablePayloadSize:
            if normalPayloadSize :
                self.dataPayloadSize=rng.normal(normaldist_mean_payload_size,normaldist_sigma_payload_size,1).astype('int').clip(dist_min_payload_size,dist_max_payload_size)[0]
            else: # uniform
                self.dataPayloadSize=rng.integers(dist_min_payload_size,dist_max_payload_size+1)

            if CANL22: #depends on scenario, data length included in header or in data... 
                self.dataPayloadSize+=CANL_data_hdr_size

    #
    ## change packet type and size accordingly
    def setPacketType(self,ptype):
        self.ptype = ptype
        
        if ptype == rtsPacketType:
            self.pl=5
            if CANL22:
                self.pl=CANL_rts_hdr_size+1
                self.rectime = airtime(self.sf,self.cr,self.pl,self.bw,explicit=(CANL_RTS_PHY_HDR==0))         
            else:
                self.rectime = airtime(self.sf,self.cr,self.pl,self.bw,explicit=(LoRa_PHY_HDR==0))         
        else:
            self.pl=self.dataPayloadSize # self.data_len
            self.rectime = airtime(self.sf,self.cr,self.pl,self.bw,explicit=(LoRa_PHY_HDR==0))


    # set a new random rx array to all neighbors
    def repropagate(self):  

        noise_dB = 0 # to GW
        noise_dB_arr = np.zeros((distance_matrix[self.nodeid].shape)) # to all neighs
        if gaussian_noise:
            noise_dB = np.clip(rng.normal(constants.noise_mu_dB,constants.noise_sigma_dB),0,2*constants.noise_mu_dB)
            noise_dB_arr = np.clip(rng.normal(constants.noise_mu_dB,constants.noise_sigma_dB,noise_dB_arr.shape),0,2*constants.noise_mu_dB)

        rayleigh_dB = 0
        rayleigh_dB_arr = np.zeros((distance_matrix[self.nodeid].shape))
        
        if rayleigh_fading:
            rayleigh_dB = rng.rayleigh(scale=np.sqrt(2 / np.pi)*rayleigh_mean_dB) - rayleigh_mean_dB
            rayleigh_dB_arr = rng.rayleigh(scale=np.sqrt(2 / np.pi)*rayleigh_mean_dB,size=rayleigh_dB_arr.shape) - rayleigh_mean_dB

        # matrix of every path loss
        self.rx_array=np.clip(-1000,self.txpow,self.txpow + constants.GL - constants.Lpld0 - 10*self.gamma_array*np.log10(distance_matrix[self.nodeid]/constants.d0) - noise_dB_arr - rayleigh_dB_arr  )
        self.rssi=min(self.txpow,self.txpow + constants.GL_GW - constants.Lpld0 - 10*gamma_GW*math.log10(self.distance_to_GW/constants.d0) - noise_dB - rayleigh_dB)

        
#
# compute the CAD prob of success (true positive) 
def get_CAD_prob(distance):


    CAD_Lpl = constants.CAD_Lpld0 + 10*constants.CAD_gamma*np.log10(distance/constants.CAD_d0) #+noise_dB
    CAD_Prx = min(Ptx,Ptx - constants.CAD_GL - CAD_Lpl)
    return(min(100-distance/60, constants.CAD_a*CAD_Prx+constants.CAD_b))

#
## CAD mechanism "requires" energy is received from a transmitter during all the CAD duration, hence we need a copy of the global on-air list 
def start_CAD(node):
    return [ transmitter.nodeid for transmitter in packetsOnAir ]

#
## compute CAD success for transmissions assumed continuous during full period  
def stop_CAD(node,on_air_at_CAD_start):
    on_air_at_CAD_stop=[ transmitter.nodeid for transmitter in packetsOnAir ]

    #Hyp: no blank of less than CAD symbols between two tx of same device (if device n is tx at start and at stops => it is assumed to be during all the CAD time)
    for devid in on_air_at_CAD_stop:
        if devid in on_air_at_CAD_start:
            if var_CAD_prob:
                if full_distances:
                    if rng.random()*100 <= get_CAD_prob(distance_matrix[node.nodeid][devid]):
                        if log_events:
                            MainLogger.info((node.nodeid,"CAD+",env.now))
                        return (True)
                else:
                    if rng.random()*100 <= get_CAD_prob(node.dist):
                        if log_events:
                            MainLogger.info((node.nodeid,"CAD+",env.now))                        
                        return (True)                    
            else:
                if rng.random()*100 <= CAD_prob and CAD_prob!=0:
                    if log_events:
                        MainLogger.info((node.nodeid,"CAD+",env.now))                    
                    return (True)                    
    if log_events:
        MainLogger.info((node.nodeid,"CAD-",env.now))
    return False              

#
## Build a numpy array of distances inter devices
def build_dist_mat(topo):
    dist_mat=np.zeros((nrNodes,nrNodes))


    for i in range(nrNodes):
        for j in range(nrNodes):
            if j!=i:
                dist_mat[i][j]=(
                    (topo['nodes'][i]['x']-topo['nodes'][j]['x'])**2+
                    (topo['nodes'][i]['y']-topo['nodes'][j]['y'])**2
                    )**(1/2)
            else:
                dist_mat[i][j]=0.01 #avoid log10(0) (aka I am 1cm away from myself)

    return dist_mat



#
# main discrete event loop, runs for each node
# a global list of packet being processed while new generations occur
#
"""
the full collision avoidance mechanism is implemented here where the node can enter into several states:
schedule_tx, want_transmit, listen_1, listen_2, send_RTS, transmit and NAV

CAD can be realized (CANL22_check_busy==True)

- node.n_retry controls how many retries is allowed at want_transmit for DATA
- Exponential backoff [Wbusy_min,2**node.Wbusy_BE] is used for CAD+BAckoff
    - if Wbusy_exp_backoff==True then node.Wbusy_BE is incremented at each retry if node.Wbusy_BE < Wbusy_maxBE
- For DATA,
    - at want_transmit, after node.n_retry, packet transmission is aborted 
- A node will enter into NAV period upon reception of an RTS or a ValidHeader from DATA
    - the node will go back to want_transmit, node.n_retry is decremented and packet TX can then be aborted in want_transmit
- If eventually the data packet is transmitted, then node.latency is updated 
- If eventually the data packet is received, then node.success_latency is updated 
- If eventually the data packet is received soon, then node.min_success_latency is updated 
"""
def transmit(env,node):

    ###### No need to redeclare globals, lists are mutable
    # global packetsOnAir
    # global nodes
    # global channel_busy_rts
    # global channel_busy_data
    
    global endSim

    #### variables updated with the transmissions
    global nrLost
    global nrCollisions
    global nrReceived
    global nrProcessed
    global nrSent
    global nrScheduled

    global nrRTSLost
    global nrRTSCollisions
    global nrRTSReceived
    global nrRTSProcessed
                    
    global n_transmit
    global inter_transmit_time
    global last_transmit_time
    global lastDisplayTime  

    global ideal_latest_start,ideal_latest_time

    last_transmit_time=0

    next_gen_time=-1#ms

    #Main loop for each node
    while True:
        
        ###////////////////////////////////////////////////////////
        # CANL - Collision Avoidance - version 2022               /
        ###////////////////////////////////////////////////////////        
        if CANL22:
            ###########################################################
            # schedule_tx -> want_transmit                            #
            # want_transmit -> listen_1                               #
            # listen_1 -> NAV_state | send_RTS                        #
            # NAV_state -> want_transmit                              #
            # send_RTS -> listen_2 | send_DATA                        #
            # listen_2 -> NAV_state | send_DATA                       #
            # send_DATA -> schedule_tx                                #
            ###########################################################

            ###########################################################
            # schedule_tx -> want_transmit                            #
            ###########################################################
            if node.ca_state==schedule_tx:
                ## scheduling a new generation

                a_new_gen_has_been_done=False
                while env.now>next_gen_time:
                    # if we enter a second time here, we need to drop
                    if a_new_gen_has_been_done:
                        node.n_dropped+=1

                    # produce next packet
                    if experiment==6:
                        #normally 9 nodes with 100ms delay between each node
                        inter_gen_delay=node.cycle*node.period-env.now+node.nodeid*100
                    elif experiment==7:
                        #normally 5 nodes with 500ms delay between each node
                        inter_gen_delay=node.cycle*node.period-env.now+node.nodeid*500            
                    else:
                        if node.distrib==perioDistribType:
                            inter_gen_delay=node.period
                        if node.distrib==expoDistribType:
                            inter_gen_delay = rng.exponential(float(node.period))
                            # transmit_wait = rng.expovariate(1.0/float(node.period))
                        if node.distrib==uniformDistribType:
                            inter_gen_delay = rng.uniform(max(2000,node.period-5000),node.period+5000)        
                    
                    ### randomize first generation for each node... Not a great impact 
                    if shuffle_start and next_gen_time==-1:
                        next_gen_time+=rng.uniform(0,node.period)
                    next_gen_time+=inter_gen_delay
                    node.gen_times.append(next_gen_time)
                    a_new_gen_has_been_done=True

                    # pick a random size
                    node.packet.setDataPayloadSize()
                    node.packet.setPacketType(dataPacketType)                

                    node.cycle = node.cycle + 1
                    nrScheduled += 1
                    node.n_payload_gen += node.packet.dataPayloadSize - CANL_data_hdr_size

                # initiate backoff and change state
                node.Wbusy_BE=Wbusy_BE
                node.ca_state=want_transmit
                
                # wait the generation
                transmit_wait=next_gen_time - env.now
                yield env.timeout(transmit_wait)

            ###########################################################
            # want_transmit -> listen_1                               #
            ###########################################################
            if node.ca_state==want_transmit:# and node.packet.ptype==dataPacketType:
                if node.n_retry==0: # no more trials possible, abort
                    node.n_aborted = node.n_aborted +1
                    #reset for sending a new packet                
                    node.n_retry=n_retry
                    node.nav=0

                    node.ca_state=schedule_tx
                    node.Wbusy_BE=Wbusy_BE
                else:                    
                    if node.nav!=0:
                        #reset nav to start again a complete CA procedure
                        node.nav=0                        
                    else:
                        #this is an initial transmit attempt
                        node.want_transmit_time=env.now
                        n_transmit = n_transmit + 1
                        if n_transmit > 1:
                            current_inter_transmit_time = env.now - last_transmit_time
                            inter_transmit_time += current_inter_transmit_time
                        last_transmit_time = env.now        
                        
                    
                    if CANL22_P!=0:
                        #determine if the node transmits data right after RTS or after a listen 2 phase
                        node.my_P=rng.integers(0,101)

                    # CAD before LISTEN, optional, default not applied
                    channel_found_busy=False                    
                    CAD_time=0
                    if CANL22_check_busy:
                        node.ca_state=CANL_CAD
                        node.n_CAD = node.n_CAD + 1

                        ### Wait CAD Duration
                        CAD_time=node.packet.symTime*nCadSym

                        on_air_at_CAD_start=start_CAD(node) # simulator stores transmitters at CAD start
                        yield env.timeout(CAD_time)
                        channel_found_busy=stop_CAD(node,on_air_at_CAD_start) # determines CAD + or -
        
                    if channel_found_busy:
                        node.CAD=True
                        node.nav+=1         
                        #will go into NAV
                        node.ca_state=CANL_NAV_state  
                        
                        node.backoff=rng.integers(CANL_backoff_min,CANL_backoff_max+1)# 64 == 2**Wbusy_maxBE
                        yield env.timeout( node.backoff*node.packet.Tpream)

                    else: # go listen mode, then
                        node.ca_state=CANL_listen1
                        #change packet type to get the correct time-on-air for calculation
                        node.packet.setPacketType(rtsPacketType)
                        node.ca_listen_start_time=env.now

                        node.start_listening()

                        # switch bw softer fairness or basic fairness (reduction of listen win max wrt n_retry)
                        if CANL22_softer_fair:
                            if node.n_retry<n_retry:
                                CANL_win_max=max(CANL22_L1_min,CANL22_L1_MAX-CANL22_fair_factor*(n_retry-node.n_retry-1))
                            else:
                                CANL_win_max=max(CANL22_L1_min,CANL22_L1_MAX)
                        else:
                            CANL_win_max=max(CANL22_L1_min,CANL22_L1_MAX-CANL22_fair_factor*(n_retry-node.n_retry))

                        # compute listen window
                        listen_time=rng.integers(
                            CANL22_L1_min, 
                            CANL_win_max+1
                            )*node.packet.Tpream #+node.packet.rectime#+.131

                        node.ca_listen_end_time=env.now+listen_time
                        if log_events:
                            MainLogger.info((node.nodeid,"lis1_start",env.now))
                        yield env.timeout(listen_time)

            #############################################################
            # END or Prolong ANY listening phase                        #
            #############################################################
            #node was in listen:
            if node.ca_state in [CANL_listen1,CANL_listen2] and node.listened_time==-1:
                # compute additional time given to finish a reception of a datalength value
                toyield=node.stop_listening()
                if toyield!=0:
                    yield env.timeout(toyield)

            ###########################################################
            # listen_1 -> NAV_state | send_RTS                        #
            ###########################################################
            #node was in CANL_listen1
            #### WE stopped listening earlier! 
            if node.ca_state==CANL_listen1 and node.listened_time!=-1:
                if log_events:
                    MainLogger.info((node.nodeid,"lis1_stop",node.ca_listen_start_time+node.listened_time))
                node.total_listen_time = node.total_listen_time + node.listened_time
                
                #did we receive a DATA with a ValidHeader?
                if node.I_know_it_is_Data==True:
                    #nav period is the time-on-air of the maximum data size which is returned in node.nav or max_payload_size
                    nav_period=airtime(node.packet.sf,node.packet.cr,max_payload_size,node.packet.bw,explicit=(LoRa_PHY_HDR==0))-node.wait_PHY_interrupt
                    if Interrupts_on_header_valid:# we were able to find out the data size from its header
                        nav_period= airtime(node.packet.sf,node.packet.cr,node.next_payload_byte,node.packet.bw,explicit=(LoRa_PHY_HDR==0)) - node.wait_PHY_interrupt #node.data_rectime

                    #will go into NAV
                    node.nav+=1         
                    node.ca_state=CANL_NAV_state                

                    #it can happen that the end of the listening period is after the theoretical NAV period    for data packet
                    #in this case, it is not really possible to revert time and the end of the listening period will be the end of the nav period
                    if node.ca_listen_start_time+node.listened_time+nav_period <= env.now:
                        #in this case, there is no additional delay, we just go to end of nav state
                        pass
                    else:                        
                        #adjust to remove the extra time due to the fact that the data should have been received ealier                    
                        nav_period-=env.now-(node.ca_listen_start_time+node.listened_time)
                        yield env.timeout(nav_period)            
                elif node.I_know_it_is_RTS==True:#it did receive an RTS
                    #we process this event at the end of the listening period, normally the RTS has been received in the past
                    nav_period= airtime(node.packet.sf,node.packet.cr,node.next_payload_byte,node.packet.bw,explicit=(LoRa_PHY_HDR==0)) #node.data_rectime
                    if CANL22_P!=0:
                        nav_period+=CANL22_L2
                    #adjust to remove the extra time due to the fact that the RTS should have been received ealier (maybe, what if rts received at end of listen??)
                    delay_I_yeld_listenning_while_I_wasnt_anymore=env.now-(node.ca_listen_start_time+node.listened_time)
                    delay_short=delay_I_yeld_listenning_while_I_wasnt_anymore
                    if delay_short>0:
                        if delay_short<nav_period:
                            nav_period-=delay_short
                        else:
                            nav_period=0
                    #go into NAV
                    node.nav+=1         
                    node.ca_state=CANL_NAV_state            
                    yield env.timeout(nav_period)

                elif node.I_heard_preamble:
                    #nav period is the time-on-air of the maximum data size which is returned in node.nav
                    nav_period=airtime(node.packet.sf,node.packet.cr,max_payload_size,node.packet.bw,explicit=(LoRa_PHY_HDR==0))-node.packet.symTime*3
                    #will go into NAV
                    node.nav+=1         
                    node.ca_state=CANL_NAV_state                

                    overtime_listening=max(0,(env.now-node.ca_listen_start_time)-node.listened_time)
                    if overtime_listening>0:
                        nav_period=max(0,nav_period-overtime_listening)

                    yield env.timeout(nav_period) 

                else:# end of listening -> now send RTS (if any to send wrt scenario)

                    node.ca_state=CANL_send_RTS

                    if node.packet.dataPayloadSize>CANL_RTS_min_payload_size:# min is set very high in scenarios without RTS
                        node.n_rts_sent = node.n_rts_sent + 1
                        if (node in packetsOnAir):
                            print("ERROR: RTS packet already in",file=sys.stderr)
                        else:
                            if node.packet.rssi < get_sensitivity(node.packet.sf,node.packet.bw): # gw does not receive it, too far
                                node.packet.lost = True
                                if full_distances: # on air anyway, let's check impact on neighbors
                                    checkcollision(node.packet)
                                    packetsOnAir.append(node)
                                    node.packet.addTime = env.now
                            else:
                                node.packet.lost = False
                                checkcollision(node.packet)
                                packetsOnAir.append(node)
                                node.packet.addTime = env.now
                            if log_events:
                                MainLogger.info((node.nodeid,"TX_start",env.now))


                        channel_busy_rts[node.nodeid]=True
                        channel_log.append((env.now,node.packet.rectime,node.nodeid,node.cycle))
                        # print(node.nodeid,packetsOnAir,file=sys.stderr)
                        yield env.timeout(node.packet.rectime)
                        channel_busy_rts[node.nodeid]=False
                        if log_events:
                            MainLogger.info((node.nodeid,"TX_stop",env.now))
                        
                        if node.packet.lost:
                            nrRTSLost += 1
                        if node.packet.collided == 1:
                            nrRTSCollisions = nrRTSCollisions +1
                        if node.packet.collided == 0 and not node.packet.lost:
                            nrRTSReceived = nrRTSReceived + 1
                            # print("node {} {}: RTS packet has been correctly transmitted".format(node.nodeid, env.now))
                        if node.packet.processed == 1:
                            nrRTSProcessed = nrRTSProcessed + 1

                        # complete packet has been received by base station
                        # can remove it
                        if (node in packetsOnAir):
                            packetsOnAir.remove(node)
                        # reset the packet
                        node.packet.collided = 0
                        node.packet.processed = 0
                        node.packet.lost = False       
                        node.packet.repropagate()     
                    else: # payload is too short to consider sending RTS
                        pass


                # reinit listen params
                node.I_heard_preamble=False
                node.I_know_it_is_Data=False
                node.I_know_it_is_RTS=False
                node.next_payload_byte=0
                node.listened_time=-1

                node.heard_frames=[]
                node.add_rx_time_opportunities=1


            ###########################################################
            # NAV_state -> want_transmit                              #
            ###########################################################
            if node.ca_state==CANL_NAV_state:
                #we arrive at the end of the nav period
                #so we try again from the beginning of the CANL22 procedure
                node.ca_state=want_transmit
                node.packet.setPacketType(dataPacketType)
                # decrement retry counter
                node.n_retry = node.n_retry - 1   

            ###########################################################
            # send_RTS -> listen_2 | send_DATA                        #
            ###########################################################
            if node.ca_state==CANL_send_RTS:
                if CANL22_P==0:
                    #default: no listening phase 2
                    node.ca_state=CANL_send_DATA
                else:                    
                    #we have sent RTS, and now we go for another listening period
                    node.ca_state=CANL_listen_2   
                    #store time at which listening period began
                    node.ca_listen_start_time=env.now
                    if log_events:
                        MainLogger.info((node.nodeid,"lis2_start",env.now))

                    node.start_listening()

                    #listen period is CANL22_L2*DIFS, with DIFS=preamble duration
                    listen_duration=CANL22_L2*node.packet.Tpream
                    node.ca_listen_end_time=env.now+listen_duration
                    yield env.timeout(listen_duration)

            ###########################################################
            # listen_2 -> NAV_state | send_DATA                       #
            ###########################################################
            #### WE stopped listening earlier! See node.listened_time==-1 ###
            if node.ca_state==CANL_listen2 and node.listened_time!=-1:
                if log_events:
                    MainLogger.info((node.nodeid,"lis2_stop",node.ca_listen_start_time+node.listened_time))
                node.total_listen_time = node.total_listen_time + node.listened_time

                #did we receive a DATA with a ValidHeader?
                if node.I_know_it_is_Data==True:
                    nav_period=airtime(node.packet.sf,node.packet.cr,max_payload_size,node.packet.bw,explicit=(LoRa_PHY_HDR==0))-node.wait_PHY_interrupt

                    if Interrupts_on_header_valid:
                        nav_period= airtime(node.packet.sf,node.packet.cr,node.next_payload_byte,node.packet.bw,explicit=(LoRa_PHY_HDR==0)) - node.wait_PHY_interrupt #node.data_rectime

                    #will go into NAV
                    node.nav+=1         
                    node.ca_state=CANL_NAV_state

                    if node.ca_listen_start_time+node.listened_time+nav_period <= env.now:
                        #in this case, there is no additional delay, we just go to start_nav
                        pass
                    else:                        
                        #adjust to remove the extra time due to the fact that the data should have been received ealier                    
                        nav_period-=env.now-(node.ca_listen_start_time+node.listened_time)
                        yield env.timeout(nav_period)            

                elif node.I_know_it_is_RTS==True:#it did receive an RTS
                    nav_period= airtime(node.packet.sf,node.packet.cr,node.next_payload_byte,node.packet.bw,explicit=(LoRa_PHY_HDR==0)) #node.data_rectime
                    if CANL22_P!=0:
                        nav_period+=CANL22_L2
                    #adjust to remove the extra time due to the fact that the RTS should have been received ealier (maybe, what if rts received at end of listen??)
                    delay_I_yeld_listenning_while_I_wasnt_anymore=env.now-(node.ca_listen_start_time+node.listened_time)
                    delay_short=delay_I_yeld_listenning_while_I_wasnt_anymore
                    if delay_short>0:
                        if delay_short<nav_period:
                            nav_period-=delay_short
                        else:
                            nav_period=0
                    #go into NAV
                    node.nav+=1         
                    node.ca_state=CANL_NAV_state            
                    yield env.timeout(nav_period)


                elif node.I_heard_preamble:
                    #nav period is the time-on-air of the maximum data size which is returned in node.nav
                    nav_period=airtime(node.packet.sf,node.packet.cr,max_payload_size,node.packet.bw,explicit=(LoRa_PHY_HDR==0))-node.packet.symTime*3
                    #will go into NAV
                    node.nav+=1         
                    node.ca_state=CANL_NAV_state                

                    overtime_listening=max(0,(env.now-node.ca_listen_start_time)-node.listened_time)
                    if overtime_listening>0:
                        nav_period=max(0,nav_period-overtime_listening)

                    yield env.timeout(nav_period) 

                else:    
                    node.ca_state=CANL_send_DATA

                # reinit listen params
                node.I_heard_preamble=False
                node.I_know_it_is_Data=False
                node.I_know_it_is_RTS=False
                node.next_payload_byte=0
                node.listened_time=-1

                node.heard_frames=[]
                node.add_rx_time_opportunities=1

            ###########################################################
            # send_DATA -> schedule_tx                                #
            ###########################################################
            if node.ca_state==CANL_send_DATA:        
                #change packet type to get the correct tine-on-air
                node.packet.setPacketType(dataPacketType)                

                # DATA time sending and receiving
                # DATA packet arrives -> add to base station
                node.n_data_sent = node.n_data_sent + 1
                node.n_payload_sent += node.packet.dataPayloadSize - CANL_data_hdr_size
                nrSent+=1
                node.total_retry += n_retry - node.n_retry
                node.latency = node.latency + (env.now-node.want_transmit_time)
                if (node in packetsOnAir):
                    print("ERROR: DATA packet already in",file=sys.stderr)
                else:
                    if node.packet.rssi < get_sensitivity(node.packet.sf,node.packet.bw):
                        node.packet.lost = True
                        if full_distances:#on air anyway
                            checkcollision(node.packet)
                            packetsOnAir.append(node)
                            node.packet.addTime = env.now
                    else:
                        node.packet.lost = False
                        checkcollision(node.packet)
                        packetsOnAir.append(node)
                        node.packet.addTime = env.now
                    if log_events:
                        MainLogger.info((node.nodeid,"TX_start",env.now))


                channel_busy_data[node.nodeid]=True
                channel_log.append((env.now,node.packet.rectime,node.nodeid,node.cycle))
                # print(node.nodeid,packetsOnAir,file=sys.stderr)
                yield env.timeout(node.packet.rectime)
                channel_busy_data[node.nodeid]=False
                if log_events:
                    MainLogger.info((node.nodeid,"TX_stop",env.now))
                    
                if node.packet.lost:
                    nrLost += 1
                    node.n_lost+=1
                    # print("node {} {}: DATA packet was lost".format(node.nodeid, env.now))
                if node.packet.collided == 1:
                    nrCollisions = nrCollisions + 1
                    node.n_collided+=1
                    # print("node {} {}: DATA packet was collided".format(node.nodeid, env.now))
                if node.packet.collided == 0 and not node.packet.lost:
                    nrReceived = nrReceived + 1
                    # print("node {} {}: DATA packet has been correctly transmitted".format(node.nodeid, env.now))
                    current_latency=env.now-node.want_transmit_time
                    node.min_success_latency = min(node.min_success_latency, current_latency)
                    node.success_latency = node.success_latency + current_latency
                    node.n_data_success+=1
                    node.n_payload_success += node.packet.dataPayloadSize - CANL_data_hdr_size
                if node.packet.processed == 1:
                    nrProcessed = nrProcessed + 1

                # complete packet has been received by base station
                # can remove it
                if (node in packetsOnAir):
                    packetsOnAir.remove(node)
                # reset the packet
                node.packet.collided = 0
                node.packet.processed = 0
                node.packet.lost = False
                node.packet.repropagate()
                node.n_retry=n_retry
                node.CAD=False
                node.nav=0
                node.ca_state=schedule_tx


        ###////////////////////////////////////////////////////////
        # Ideal ideal_FIFO                                        /
        ###////////////////////////////////////////////////////////        
        elif ideal_FIFO:
            ###########################################################
            # schedule_tx -> want_transmit                            #
            # want_transmit -> send_DATA                              #
            # send_DATA -> schedule_tx                                #
            ###########################################################

            ###########################################################
            # schedule_tx -> want_transmit                            #
            ###########################################################
            if node.ca_state==schedule_tx:
                a_new_gen_has_been_done=False
                while env.now>next_gen_time:

                    # if we enter a second time here, we need to drop
                    if a_new_gen_has_been_done:
                        node.n_dropped+=1

                    # last_gen_time=next_gen_time

                    # produce next packet
                    if experiment==6:
                        #normally 9 nodes with 100ms delay between each node
                        inter_gen_delay=node.cycle*node.period-env.now+node.nodeid*100
                    elif experiment==7:
                        #normally 5 nodes with 500ms delay between each node
                        inter_gen_delay=node.cycle*node.period-env.now+node.nodeid*500            
                    else:
                        if node.distrib==perioDistribType:
                            inter_gen_delay=node.period
                        if node.distrib==expoDistribType:
                            inter_gen_delay = rng.exponential(float(node.period))
                            # transmit_wait = rng.expovariate(1.0/float(node.period))
                        if node.distrib==uniformDistribType:
                            inter_gen_delay = rng.uniform(max(2000,node.period-5000),node.period+5000)        
                    
                    # next_gen_time=last_gen_time+inter_gen_delay
                    if shuffle_start and next_gen_time==-1:
                        next_gen_time+=rng.uniform(0,node.period)
                    next_gen_time+=inter_gen_delay
                    node.gen_times.append(next_gen_time)
                    a_new_gen_has_been_done=True

                    node.packet.setDataPayloadSize()
                    node.packet.setPacketType(dataPacketType)                

                    node.cycle = node.cycle + 1
                    nrScheduled += 1
                    node.n_payload_gen += node.packet.dataPayloadSize 
                
                node.ca_state=want_transmit
                
                transmit_wait=next_gen_time - env.now
                yield env.timeout(transmit_wait)

            ###########################################################
            # want_transmit -> listen_1                               #
            ###########################################################
            if node.ca_state==want_transmit:# and node.packet.ptype==dataPacketType:

                #this is an initial transmit attempt
                node.want_transmit_time=env.now
                n_transmit = n_transmit + 1
                if n_transmit > 1:
                    current_inter_transmit_time = env.now - last_transmit_time
                    inter_transmit_time += current_inter_transmit_time
                last_transmit_time = env.now 

                my_transmission_date=ideal_latest_start+ideal_latest_time+1/100000 #add 10 ns to avoid floating point error (4104.19200000001) and collisions!
                node.ca_state=Ideal_send_DATA    
                ideal_latest_time=airtime(node.packet.sf,node.packet.cr,node.packet.dataPayloadSize,node.packet.bw,explicit=(LoRa_PHY_HDR==0))
                if node.want_transmit_time<my_transmission_date:
                    ideal_latest_start=my_transmission_date
                    #let's wait for the end of the previous tx:
                    yield env.timeout(my_transmission_date-node.want_transmit_time)
                else:
                    ideal_latest_start=node.want_transmit_time
                    

            ###########################################################
            # send_DATA -> schedule_tx                                #
            ###########################################################
            if node.ca_state==Ideal_send_DATA:        

                # DATA time sending and receiving
                # DATA packet arrives -> add to base station
                node.n_data_sent = node.n_data_sent + 1
                node.n_payload_sent += node.packet.dataPayloadSize
                nrSent+=1
                node.latency = node.latency + (env.now-node.want_transmit_time)
                if (node in packetsOnAir):
                    print("ERROR: DATA packet already in",file=sys.stderr)
                else:
                    if node.packet.rssi < get_sensitivity(node.packet.sf,node.packet.bw):
                        # print("node {}: DATA packet will be lost".format(node.nodeid))
                        node.packet.lost = True
                        if full_distances:#on air anyway
                            checkcollision(node.packet)
                            packetsOnAir.append(node)
                            node.packet.addTime = env.now
                    else:
                        node.packet.lost = False
                        checkcollision(node.packet)
                        packetsOnAir.append(node)
                        node.packet.addTime = env.now
                    if log_events:
                        MainLogger.info((node.nodeid,"TX_start",env.now))


                channel_busy_data[node.nodeid]=True
                channel_log.append((env.now,node.packet.rectime,node.nodeid,node.cycle))
                # print(node.nodeid,packetsOnAir,file=sys.stderr)
                yield env.timeout(node.packet.rectime)
                channel_busy_data[node.nodeid]=False
                if log_events:
                    MainLogger.info((node.nodeid,"TX_stop",env.now))
                    
                if node.packet.lost:
                    nrLost += 1
                    node.n_lost+=1
                    # print("node {} {}: DATA packet was lost".format(node.nodeid, env.now))
                if node.packet.collided == 1:
                    nrCollisions = nrCollisions + 1
                    node.n_collided+=1
                    # print("node {} {}: DATA packet was collided".format(node.nodeid, env.now))
                if node.packet.collided == 0 and not node.packet.lost:
                    nrReceived = nrReceived + 1
                    # print("node {} {}: DATA packet has been correctly transmitted".format(node.nodeid, env.now))
                    current_latency=env.now-node.want_transmit_time
                    node.min_success_latency = min(node.min_success_latency, current_latency)
                    node.success_latency = node.success_latency + current_latency
                    node.n_data_success+=1
                    node.n_payload_success += node.packet.dataPayloadSize
                if node.packet.processed == 1:
                    nrProcessed = nrProcessed + 1

                # complete packet has been received if not lost by base station
                # can remove it
                if (node in packetsOnAir):
                    packetsOnAir.remove(node)
                # reset the packet
                node.packet.collided = 0
                node.packet.processed = 0
                node.packet.lost = False
                node.packet.repropagate()
                node.n_retry=n_retry
                node.CAD=False
                # node.nav=0
                node.ca_state=schedule_tx


        ###////////////////////////////////////////////////////////                
        #no collision avoidance                                   /
        #original ALOHA-like behavior, with CAD+Back-Off          /
        ###//////////////////////////////////////////////////////// 
        else:
            # Schedule next tx
            #########################################################################
            a_new_gen_has_been_done=False
            while env.now>next_gen_time:

                # if we enter a second time here, we need to drop
                if a_new_gen_has_been_done:
                    node.n_dropped+=1

                # produce next packet
                if experiment==6:
                    #normally 9 nodes with 100ms delay between each node
                    inter_gen_delay=node.cycle*node.period-env.now+node.nodeid*100
                elif experiment==7:
                    #normally 5 nodes with 500ms delay between each node
                    inter_gen_delay=node.cycle*node.period-env.now+node.nodeid*500            
                else:
                    if node.distrib==perioDistribType:
                        inter_gen_delay=node.period
                    if node.distrib==expoDistribType:
                        inter_gen_delay = rng.exponential(float(node.period))
                        # transmit_wait = rng.expovariate(1.0/float(node.period))
                    if node.distrib==uniformDistribType:
                        inter_gen_delay = rng.uniform(max(2000,node.period-5000),node.period+5000)        
                
                if shuffle_start and next_gen_time==-1:
                    next_gen_time+=rng.uniform(0,node.period)                
                next_gen_time+=inter_gen_delay
                node.gen_times.append(next_gen_time)
                a_new_gen_has_been_done=True

                node.packet.setDataPayloadSize()
                node.packet.setPacketType(dataPacketType)                

                node.cycle = node.cycle + 1
                nrScheduled += 1
                node.n_payload_gen += node.packet.dataPayloadSize
            
            
            transmit_wait=next_gen_time - env.now
            yield env.timeout(transmit_wait)

            #########################################################################

            node.want_transmit_time=env.now
            
            n_transmit = n_transmit + 1
            if n_transmit > 1:
                current_inter_transmit_time = env.now - last_transmit_time
                inter_transmit_time += current_inter_transmit_time
            last_transmit_time = env.now                
            
            channel_found_busy=True
            
            while node.n_retry and channel_found_busy:
                if noCA_check_busy:
                    node.n_CAD = node.n_CAD + 1

                    ### Wait CAD Duration
                    CAD_time=node.packet.symTime*nCadSym

                    on_air_at_CAD_start=start_CAD(node) # simulator stores transmitters at CAD start
                    yield env.timeout(CAD_time)
                    channel_found_busy=stop_CAD(node,on_air_at_CAD_start) # determines CAD + or -

                else:
                    channel_found_busy=False
                        
                if channel_found_busy:
                    #here we just delay by a random backoff timer to retry again
                    #random backoff [Wbusy_min,2**Wbusy_BE]
                    node.backoff=rng.integers(Wbusy_min,2**node.Wbusy_BE+1)
                    if Wbusy_exp_backoff:
                        if node.Wbusy_BE<Wbusy_maxBE:
                            node.Wbusy_BE=node.Wbusy_BE + 1
                    node.n_retry = node.n_retry - 1
                    if Wbusy_add_max_toa:            
                        yield env.timeout(airtime(node.packet.sf,node.packet.cr,max_payload_size,node.packet.bw,explicit=(LoRa_PHY_HDR==0))+node.backoff*node.packet.Tpream)
                    else:
                        yield env.timeout(node.backoff*node.packet.Tpream)

            # exited while without transmiting => abort
            if node.n_retry==0:
                node.n_aborted = node.n_aborted +1
                node.n_retry=n_retry
                node.Wbusy_BE=Wbusy_BE
            else:    
                node.n_data_sent = node.n_data_sent + 1
                node.n_payload_sent += node.packet.dataPayloadSize
                nrSent+=1
                node.total_retry += n_retry - node.n_retry
                node.latency = node.latency + (env.now-node.want_transmit_time)
                if (node in packetsOnAir):
                    print("ERROR: DATA packet already in",file=sys.stderr)
                else:
                    if node.packet.rssi < get_sensitivity(node.packet.sf,node.packet.bw):
                        # print("node {}: DATA packet will be lost".format(node.nodeid))
                        node.packet.lost = True
                        if full_distances:#on air anyway
                            checkcollision(node.packet)
                            packetsOnAir.append(node)
                            node.packet.addTime = env.now
                    else:
                        node.packet.lost = False
                        # check collision at GW / local impacts
                        if (checkcollision(node.packet)==1):
                            node.packet.collided = 1
                        else:
                            node.packet.collided = 0
                        packetsOnAir.append(node)
                        node.packet.addTime = env.now
                    if log_events:
                        MainLogger.info((node.nodeid,"TX_start",env.now))


                channel_busy_data[node.nodeid]=True
                channel_log.append((env.now,node.packet.rectime,node.nodeid,node.cycle))
                yield env.timeout(node.packet.rectime)
                channel_busy_data[node.nodeid]=False
                if log_events:
                    MainLogger.info((node.nodeid,"TX_stop",env.now))
        
                if node.packet.lost:
                    nrLost += 1
                    node.n_lost+=1
                if node.packet.collided == 1:
                    nrCollisions = nrCollisions + 1
                    node.n_collided+=1
                if node.packet.collided == 0 and not node.packet.lost:
                    nrReceived = nrReceived + 1
                    current_latency=env.now-node.want_transmit_time
                    node.min_success_latency = min(node.min_success_latency, current_latency)
                    node.success_latency = node.success_latency + current_latency
                    node.n_data_success+=1
                    node.n_payload_success += node.packet.dataPayloadSize
                    # print("node {} {}: DATA packet has been correctly transmitted".format(node.nodeid, env.now))
                if node.packet.processed == 1:
                    nrProcessed = nrProcessed + 1
            
                # complete packet has been received by base station
                # can remove it
                if (node in packetsOnAir):
                    packetsOnAir.remove(node)
                # reset the packet
                node.packet.collided = 0
                node.packet.processed = 0
                node.packet.lost = False
                node.packet.repropagate()     
                node.n_retry=n_retry
                node.Wbusy_BE=Wbusy_BE

        ##################################
        ######## Checking end of Simu ####
        ##################################
        if nrScheduled > targetSchedPacket:
            endSim=env.now
            ## ! ##
            return

        # global lastDisplayTime    
        # if nrProcessed % 10000 == 0 and env.now!=lastDisplayTime:
        #     print( nrProcessed, "-", file=sys.stderr)
        #     lastDisplayTime=env.now    

        # if nrSent % 10000 == 0 and env.now!=lastDisplayTime:
        #     print( nrSent, "-", file=sys.stderr)
        #     lastDisplayTime=env.now          


def main_with_params(params):
    #############################################
    # Parameters
    #############################################
        #############################################
        # Parameters for this scenario
        #############################################
            ######### node behaviour ####################
    global n_retry          # maximum number of retry when transmitting a data packet
            ######### default backoff behaviour ####################
    global Wbusy_min     # minimun backoff when channel has been detected busy
    global Wbusy_BE      # maximun backoff when channel has been detected busy
    global Wbusy_maxBE
    global Wbusy_add_max_toa # indicate whether the toa of the maximum allowed payload size should be added to backoff timer. # this is to avoid retrying during the packet transmission because of unreliable CAD
    global Wbusy_exp_backoff # exponential backoff
            ######### Channel properties ################
    #CAD reliability probability
    #set to 0 to always assume that CAD indicates a free channel so that CA will be always used, or transmit immediately in ALOHA
    #set to 100 for a fully reliable CAD, normally there should not be collision at all
    #set to [1,99] to indicate a reliability percentage: i.e. (100-CAD_prob) is the probability that CAD reports a free channel while channel is busy            
    global CAD_prob
    global var_CAD_prob     # set CAD_prob variable according to distance and path loss: at 400m, uniform 20%; at -133.25dBm, 100% 
    global full_distances   # set rssi model based on distance tx-rx, as opposed to default model (based on dist. tx-gw)
    global lora24GHz        # set sensitivity for this band #set to True if LoRa 2.4Ghz is considered
    global full_collision   # do the full collision check (time overlap and capture, not just sf+bw at a given point in time)
    global gaussian_noise   # if set true each packet will be applied a gaussian noise of fixed mean (see constants) toward any receptor if $full_distances$ otherwise toward GW; otherwise no noise applied.  
    global powerCaptureThreshold # = 6# dB
    global gamma_ED         # Path Loss Exponent for ED-ED links, default in constants.py
    global gamma_GW         # Path Loss Exponent for ED-GW links, default in constants.py
    global normal_gamma_ED  # if True (default), ED-ED PLEs are normally distributed (mean= gamma_ED, stdev sigma_gamma_ED) 
    global sigma_gamma_ED   # Standard deviation of (normally distributed) ED-ED PLEs

            ######### Distribution&Traffic properties ################
    global nrNodes          # number of devices
    global maxBSReceives    # maximum number of packets the BS can receive at the same time, not in use here currently
    global avgSendTime      # average interval between packet arrival
    global packetLength     # fixed size of packets (#the selected packet length)
    global variablePayloadSize # if True, (uniform) variation of payload in e.g. (40,100) octet 
    global normalPayloadSize # if True, normal variation of payload in e.g. (clip_min=0,clip_max=255 or max_payload_size,mu=60,sigma=15) octet 
    global dist_min_payload_size # minimum payload size in a distribution.
    global dist_max_payload_size # maximum payload size in a distribution.
    global normaldist_mean_payload_size # mean payload size in a normal distribution.
    global normaldist_sigma_payload_size # std dev sigma of payload size in a normal distribution.
    global max_payload_size # maximum of allowed payload size, have impact on CAS21 NAV period. #LoRa Phy has a maximum of 255N but you can decide for smaller max value (as with SigFox for instance)
    global targetSentPacket # number of packets tried in simulation. nb of packet to be sent per node. #targetSentPacket*nrNodes will be the target total number of sent packets before we exit simulation
    global targetSchedPacket # number of packets tried in simulation. nb of packet to be sent per node. #targetSentPacket*nrNodes will be the target total number of scheduled packets before we exit simulation    
    global distribType      # type of traffic (#the selected distribution)
    global shuffle_start    # add a random uniform node.period before starting

            ######### Simulation properties ################
    # experiments:
    # 0: packet with longest airtime, aloha-style experiment
    # 0: one with 3 frequencies, 1 with 1 frequency
    # 2: with shortest packets, still aloha-style
    # 3: with shortest possible packets depending on distance
    # 4: TODO!
    # "SF7BW500CAD4": first ever "named" experiment"
    global experiment
    global exp4SF #SF value for experiment 4
    global minsensi
    global Ptx
    global nCadSym          # number of Symbols for CAD. NB: in DS_SX1261-2_V2_1 p40, Semtech mentions half a symbol to process CAD. Here assumed process made in parallel with next task. 
    global Interrupts_on_header_valid # set to true if PHY is able to stop RX mode after HeaderValid interrupt   
    global LoRa_PHY_HDR # GG: explicit header (H=0) or implicit header (H=1) for data frames
    global rayleigh_fading      # if set true, adds a rayleigh distributed dB value corrected to mean 0 
    global rayleigh_mean_dB        # mean of uncorrected rayleigh distribution. np.sqrt(2 / np.pi)*mean is then the "scale" or "mode" parameter.

    # global simtime
    global MainLogger,log_events # a node level event logger, very verbose !!!! set to False by default, around (2e6 lines, 100MB)/simu
    global keep_chan_log # same idea, but this boolean choose if you keep it in disk (in res dict), otherwise weeped out of RAM at end of simu
    global keep_Global_TT_IGTs # also a huge list to be kept or not, the network inter gen time 
            ######### Topological properties ################
    # max distance: 300m in city, 3000 m outside (5 km Utz experiment)
    # also more unit-disc like according to Utz
    global maxDist          # max dist GW device considered when building topology. defaults to sensitivity threshold.
    global distance_matrix  # numpy array with distances between devs
    # base station placement
    global bsx              # gw x coord
    global bsy              # gw y coord
    global xmax             # for old plots
    global ymax             # for old plots


        #############################################
        # Parameters for the CAD+Backoff simple comparative mechanism
        #############################################
            ######### General params ####################
    global noCA_check_busy        # check for channel busy or not, before sending Data


        #############################################
        # Parameters for the Collision_Avoidance_CANL22 mechanism
        #############################################
            ######### General params ####################
    global CANL22               # transmit() machine state checks if CA21 elif CA22 else (default) noCA
    global CANL22_P             # probability to have listen phase 2
    global CANL22_L1_min        # min DIFS for the first listening period
    global CANL22_L1_MAX        # MAX DIFS for the first listening period
    global CANL22_L2            # duration as multiple of DIFS duration for the possible second listening period
    global CANL_backoff_min     # min Tpream of backoff after positive CAD
    global CANL_backoff_max     # max Tpream of backoff after positive CAD 
    global CANL22_check_busy    # Controls whether CANL 22 performs or not CAD at end of listening period(s)
    global CANL_RTS_min_payload_size    # Controls the min data size (hdr+pl) for which CANL 22 transmits prior RTS. Default 0
    global CANL22_fair_factor   # interval for the reduction of the listen period for retrials (max -= cff*difs*n_retried)
    global CANL22_softer_fair   # starts interval reduction after second NAV
    global CANL_rts_hdr_size    # Size in bytes of upper layer CANL header for RTS frames
    global CANL_data_hdr_size   # Size in bytes of upper layer CANL header for DATA frames
    global CANL_RTS_PHY_HDR     # explicit header (H=0) or implicit header (H=1) for CANL RTS frames


        #############################################
        # Parameters for the ideal_FIFO mechanism
        #############################################
    global ideal_FIFO

    #############################################
    # Variables
    #############################################
            ######### Channel State  Vars ####################
    #indicate whether channel is busy or not, we differentiate between channel_busy_rts and channel_busy_data
    #to get more detailed statistics
    global channel_busy_rts
    global channel_busy_data
    global packetsOnAir
    global channel_log
            ######### Simu monitoring Vars ####################
    global lastDisplayTime  # print nprocessed packets only every 10000 and store time 
            ######### Simu stats on inter-transmit time Vars ####################
    global n_transmit
    global inter_transmit_time
    global max_inter_transmit_time
    global last_transmit_time
            ######### Simu control Vars ####################
    global env
    global endSim           # keep the end simultion time, at which we reach targetSentPacket
            ######### Simu components Vars ####################
    global nodes
            ######### Simu Stats Vars ####################
    global nrCollisions
    global nrRTSCollisions
    global nrReceived
    global nrRTSReceived
    global nrProcessed
    global nrSent
    global nrRTSProcessed
    global nrLost
    global nrRTSLost
    global nrScheduled
    global powerCaptures

            ######### Ideal Mechanism Vars ####################
    global ideal_latest_start,ideal_latest_time


    ######################################################################################
    ######################################################################################
    ######################################################################################
    ######################################################################################
    ######################################################################################
    ######################################################################################
    ######################################################################################
    ###### C'est Parti !
    ######################################################################################
    ######################################################################################
    ######################################################################################
    ######################################################################################
    ######################################################################################
    ######################################################################################
    ######################################################################################


    #############################################
    # Parameters
    #############################################
        #############################################
        # Parameters for this scenario
        #############################################
            ######### node behaviour ####################
    n_retry = params["n_retry"] if "n_retry" in params else 40
            ######### default backoff behaviour ####################
    Wbusy_min=1
    Wbusy_BE=3
    Wbusy_maxBE=6
    Wbusy_add_max_toa=False
    Wbusy_exp_backoff=True
            ######### Channel properties ################
    CAD_prob = params["CAD_prob"] if "CAD_prob" in params else 50
    var_CAD_prob= params["var_CAD_prob"] if "var_CAD_prob" in params else False
    full_distances = params["full_distances"] if "full_distances" in params else False
    lora24GHz = False
    full_collision = params["full_collision"] if "full_collision" in params else True
    gaussian_noise = params["gaussian_noise"] if "gaussian_noise" in params else False

    powerCaptureThreshold = params["powerCaptureThreshold"] if "powerCaptureThreshold" in params else 6# dB
    gamma_ED = params["gamma_ED"] if "gamma_ED" in params else constants.gamma
    gamma_GW = params["gamma_GW"] if "gamma_GW" in params else constants.gamma_GW

    normal_gamma_ED = params["normal_gamma_ED"] if "normal_gamma_ED" in params else True
    sigma_gamma_ED = params["sigma_gamma_ED"] if "sigma_gamma_ED" in params else constants.sigma_gamma_ED

            ######### Distribution&Traffic properties ################
    nrNodes = params["nrNodes"]
    maxBSReceives = 8
    avgSendTime = params["avgSendTime"]
    packetLength=104
    variablePayloadSize = params["variablePayloadSize"] if "variablePayloadSize" in params else False
    normalPayloadSize = params["normalPayloadSize"] if "normalPayloadSize" in params else False
    dist_min_payload_size = params["dist_min_payload_size"] if "dist_min_payload_size" in params else 40
    dist_max_payload_size = params["dist_max_payload_size"] if "dist_max_payload_size" in params else 100
    normaldist_mean_payload_size = params["normaldist_mean_payload_size"] if "normaldist_mean_payload_size" in params else 60
    normaldist_sigma_payload_size = params["normaldist_sigma_payload_size"] if "normaldist_sigma_payload_size" in params else 15

    max_payload_size = 150
    targetSentPacket = 2000
    targetSentPacket = targetSentPacket * nrNodes
    targetSchedPacket = 1000
    targetSchedPacket *= nrNodes
    # distribType=uniformDistribType
    # distribType=expoDistribType
    # distribType=perioDistribType
    distribType=expoDistribType if params["distrib"]=="expo" else uniformDistribType if params["distrib"]=="unif" else perioDistribType
    shuffle_start = params["shuffle_start"] if "shuffle_start" in params else False
            ######### Simulation properties ################
    experiment = params["experiment"]
    exp4SF=12

    # get_sensitivity(node.packet.sf,node.packet.bw) ???? minsensi was useful before existence/independance of topo_builder.py
    if lora24GHz:
        Ptx=constants.Ptx_2dot4GHz
        if experiment in [0,1,4,6,7]:
            minsensi = constants.sensi_2dot4GHz[7,2]     # 7th row is SF12, 2nd column is BW203
        elif experiment == 2:
            minsensi = constants.sensi_2dot4GHz[0,5]     # row 0 is SF5, 5th column is BW1625
        elif experiment in [3,5]:
            minsensi = np.amin(constants.sensi_2dot4GHz) ## Experiment 3 can use any setting, so take minimum
    else:
        Ptx=constants.Ptx_subGHz
        if experiment in [0,1,4,6,7]:
            minsensi = constants.sensi_subGHz[6,2]     # 6th row is SF12, 2nd column is BW125
        elif experiment == 2:
            minsensi = constants.sensi_subGHz[0,3]     # first row is SF6, 4th column is BW500
        elif experiment in [3,5]:
            minsensi = np.amin(constants.sensi_subGHz) ## Experiment 3 can use any setting, so take minimum
        elif experiment =="SF7BW500CAD4":
            minsensi = constants.sensi_subGHz[1,3]     # second row is SF7, 4th column is BW500    
    
    nCadSym=3
    # if node.packet.sf > 8:
    #     nCadSym=nCadSym+2
    #for lora24GHz we use 4 symbols for CAD    
    if lora24GHz:
        nCadSym=4 
    elif experiment==4:
        nCadSym=4 
    elif experiment =="SF7BW500CAD4":
        nCadSym=4 


    Interrupts_on_header_valid = params["Interrupts_on_header_valid"] if "Interrupts_on_header_valid" in params else False
    LoRa_PHY_HDR = params["LoRa_PHY_HDR"] if "LoRa_PHY_HDR" in params else 0             # GG: explicit header (H=0) or implicit header (H=1)

    rayleigh_fading = params["rayleigh_fading"] if "rayleigh_fading" in params else False
    rayleigh_mean_dB = params["rayleigh_mean_dB"] if "rayleigh_mean_dB" in params else 1


    # simtime = params["simtime"]
    log_events=params["log_events"]
    keep_chan_log = params["keep_chan_log"] if "keep_chan_log" in params else False
    if log_events:
        MainLogger = load_main_logger(params["start_time"])
        MainLogger.info("Started")
        MainLogger.info('start_time:{0}'.format(params["start_time"]))

    keep_Global_TT_IGTs = params["keep_Global_TT_IGTs"] if "keep_Global_TT_IGTs" in params else False

    this_topo=pickle.load(open('results/{0}_topos.dat'.format(params["start_time"]), 'rb'))[params["topo"]]
    maxDist=this_topo['maxDist']*params["topo_scale"]
    distance_matrix=build_dist_mat(this_topo)*params["topo_scale"]
    bsx = this_topo['GW']['bsx']*params["topo_scale"]
    bsy = this_topo['GW']['bsy']*params["topo_scale"]
    xmax = bsx + this_topo['maxDist']*params["topo_scale"] + 20*params["topo_scale"]
    ymax = bsy + this_topo['maxDist']*params["topo_scale"] + 20*params["topo_scale"]
    # del this_topo

        #############################################
        # Parameters for the CAD+Backoff simple comparative mechanism
        #############################################
            ######### General params ####################
    noCA_check_busy=params["with_CAD_and_back_off"] if "with_CAD_and_back_off" in params else True


        #############################################
        # Parameters for the Collision_Avoidance_CANL22 mechanism
        #############################################
            ######### General params ####################

    CANL22 = params["CANL22"] if "CANL22" in params else False
    CANL22_P = params["CANL22_P"] if "CANL22_P" in params else 0
    CANL22_L1_min = params["CANL22_L1_min"] if "CANL22_L1_min" in params else 7
    CANL22_L1_MAX = params["CANL22_L1_MAX"] if "CANL22_L1_MAX" in params else 12
    CANL22_L2 = params["CANL22_L2"] if "CANL22_L2" in params else 6
    CANL_backoff_min = params["CANL_backoff_min"] if "CANL_backoff_min" in params else 0
    CANL_backoff_max = params["CANL_backoff_max"] if "CANL_backoff_max" in params else 32
    CANL22_check_busy = params["CANL22_check_busy"] if "CANL22_check_busy" in params else False
    CANL_RTS_min_payload_size = params["CANL_RTS_min_payload_size"] if "CANL_RTS_min_payload_size" in params else 0
    CANL22_fair_factor = params["CANL22_fair_factor"] if "CANL22_fair_factor" in params else 0 # 0 means no reduction
    CANL22_softer_fair = params["CANL22_softer_fair"] if "CANL22_softer_fair" in params else False
    CANL_rts_hdr_size = params["CANL_rts_hdr_size"] if "CANL_rts_hdr_size" in params else 4 
    CANL_data_hdr_size = params["CANL_data_hdr_size"] if "CANL_data_hdr_size" in params else 4 
    CANL_RTS_PHY_HDR = params["CANL_RTS_PHY_HDR"] if "CANL_RTS_PHY_HDR" in params else 0 #(explicit)



        #############################################
        # Parameters for the ideal_FIFO mechanism
        #############################################
    ideal_FIFO = params["ideal_FIFO"] if "ideal_FIFO" in params else False

    #############################################
    # Variables
    #############################################
            ######### Channel State  Vars ####################
    channel_busy_rts = [False]*nrNodes
    channel_busy_data = [False]*nrNodes
    packetsOnAir = []
    channel_log = []
            ######### Simu monitoring Vars ####################
    lastDisplayTime=-1
            ######### Simu stats on inter-transmit time Vars ####################
    n_transmit = 0
    inter_transmit_time = 0
    max_inter_transmit_time = 40

    last_transmit_time = 0
            ######### Simu control Vars ####################
    env = simpy.Environment()
    endSim=0
            ######### Simu components Vars ####################
    nodes = []
            ######### Simu Stats Vars ####################
    nrCollisions = 0
    nrRTSCollisions = 0
    nrReceived = 0
    nrRTSReceived = 0
    nrProcessed = 0
    nrSent = 0
    nrRTSProcessed = 0
    nrLost = 0
    nrRTSLost = 0
    nrScheduled = 0
    powerCaptures = []
            ######### Ideal Mechanism Vars ####################
    ideal_latest_start = 0
    ideal_latest_time = 0



###########
###########


    # # prepare graphics and add sink
    # if (graphics == 1):
    #     plt.ion()
    #     plt.figure()
    #     ax = plt.gcf().gca()
    #     # XXX should be base station position
    #     ax.add_artist(plt.Circle((bsx, bsy), 3, fill=True, color='green'))
    #     ax.add_artist(plt.Circle((bsx, bsy), maxDist, fill=False, color='green'))

    if experiment==6:
        nrNodes=9

    if experiment==7:
        nrNodes=5

                
    for i in range(0,nrNodes):
        # myNode takes period (in ms), base station id packetlen (in Bytes)
        # 1000000 = 16 min
        # node = myNode(i, endDeviceType, bsId, avgSendTime, distribType, packetLength)
        node = myNode(
            i, 
            this_topo['nodes'][i]['x']*params["topo_scale"], 
            this_topo['nodes'][i]['y']*params["topo_scale"], 
            endDeviceType, bsId, avgSendTime, distribType, packetLength)

        nodes.append(node)
        env.process(transmit(env,node))    
        # print("-----------------------------------------------------------------------------------------------")
    
    this_topo=None
        
    # #prepare show
    # if (graphics == 1):
    #     plt.xlim([0, xmax])
    #     plt.ylim([0, ymax])
    #     plt.draw()
    #     plt.show()

    # start simulation
    # env.run(until=simtime)
    env.run()

    if log_events:
        close_logger_handlers(MainLogger)

    #########################
    ### POST SIMU ########
    #########################

    res={}

    # compute energy
    TX = constants.TX 
    RX = constants.RX
    V = constants.V

    # compute energy
    TX = constants.TX_868_14dBm
    RX = constants.RX_boosted_DCDC_mode
    V = constants.V



    #################################################################
    #################################################################
    #################################################################
    #################################################################
    #################################################################
    #################################################################
    #statistic per node
    res["nodes"]={}

    for node in nodes:
        res["nodes"][node.nodeid]={
            "number_of_CAD": node.n_CAD,
            "node_type": 'endDevice' if node.nodeType==endDeviceType else 'relayDevice',
            "node_traffic": 'expo' if node.distrib==expoDistribType else 'uniform',
            # "x":node.x,
            # "y":node.y, 
            "dist":node.dist,
        }

        #### ENERGY in CAD ####
        str_sf = "SF"+str(node.packet.sf)
        str_bw = "BW"+str(node.packet.bw)
        str_cadsym = str(nCadSym)+"S"

        #consumption must be converted into mA: cad_consumption[node.packet.sf-7]/1e6    
        energy = (node.packet.symTime * nCadSym * (constants.cad_consumption[str_sf][str_bw][str_cadsym]/3600/1e9) * V * node.n_CAD ) / 1e3
        node.CAD_energy=energy
        res["nodes"][node.nodeid]["energy_in_CAD_J"]=energy

        ##### TIME in TX 
        time_sending_data=0
        start_sending_data=0
        stop_sending_data=0
        for cl in channel_log:
            if cl[2]==node.nodeid:
                if start_sending_data==0:
                    start_sending_data=cl[0]
                time_sending_data+=cl[1]        
                stop_sending_data=cl[0]+cl[1]        
        # print(time_sending_data,start_sending_data,stop_sending_data, file=sys.stderr)
        # print(0/0)

        #### ENERGY in TX
        energy = (time_sending_data * TX[int(node.packet.txpow)+2] * V) / 1e6
        res["nodes"][node.nodeid]["energy_in_transmission_J"]=energy

        #### ENERGY in RX
        if CANL22:
            energy = (node.total_listen_time * RX * V) / 1e6
            res["nodes"][node.nodeid]["energy_in_listening_J"]=energy

        #### TOTAL ENERGY
        res["nodes"][node.nodeid]["total_energy_J"]=(
            time_sending_data * TX[int(node.packet.txpow)+2] * V  \
                                + node.total_listen_time * RX * V) / 1e6 + node.CAD_energy

        #### ENERGY per SUCCESS
        res["nodes"][node.nodeid]["energy_per_success"]=res["nodes"][node.nodeid]["total_energy_J"]/node.n_data_success if node.n_data_success>0 else -1


        res["nodes"][node.nodeid]["end_simulation_time"]=" {}ms {}h".format(endSim, float(endSim/3600000))
        res["nodes"][node.nodeid]["cumulated_TX_time_s"]=time_sending_data/1000
        res["nodes"][node.nodeid]["duty_cycle"]=time_sending_data/(stop_sending_data-start_sending_data)
        if CANL22:
            res["nodes"][node.nodeid]["cumulated_RX_time_s"]=node.total_listen_time/1000

        res["nodes"][node.nodeid]["number_of_CAD"]=sum (n.n_CAD for n in nodes)
        res["nodes"][node.nodeid]["sent_data_packets"]= node.n_data_sent
        res["nodes"][node.nodeid]["success_data_packets"]= node.n_data_success
        res["nodes"][node.nodeid]["DER"]= node.n_data_success/node.n_data_sent if node.n_data_sent>0 else -1
        res["nodes"][node.nodeid]["DER_method_2"]= (node.n_data_sent-node.n_collided)/node.n_data_sent if node.n_data_sent>0 else -1
        res["nodes"][node.nodeid]["PDR"]= node.n_data_success/(node.n_data_sent+node.n_dropped+node.n_aborted)

        res["nodes"][node.nodeid]["payload_byte_delivery_ratio"]= node.n_payload_success/node.n_payload_gen if node.n_payload_gen>0 else -1

        res["nodes"][node.nodeid]["mean_latency"]= node.latency/node.n_data_sent if node.n_data_sent>0 else -1
        res["nodes"][node.nodeid]["mean_success_latency"]= node.success_latency/node.n_data_success if node.n_data_success>0 else -1
        res["nodes"][node.nodeid]["min_success_latency"]= node.min_success_latency
        res["nodes"][node.nodeid]["aborted_packets"]= node.n_aborted
        res["nodes"][node.nodeid]["collided_packets"]= node.n_collided
        res["nodes"][node.nodeid]["lost_packets"]= node.n_lost
        res["nodes"][node.nodeid]["dropped_packets"]= node.n_dropped
        res["nodes"][node.nodeid]["mean_retry"]= node.total_retry/node.n_data_sent if node.n_data_sent>0 else -1

        if CANL22:
            res["nodes"][node.nodeid]["sent_rts_packets"]=node.n_rts_sent

    res["settings"]={
        "Nodes": nrNodes,
        "AvgSendTime": avgSendTime,
        "Distribution": 'expoDistribType' if distribType==expoDistribType else 'uniformDistribType',
        "Experiment": experiment,
        # "Simtime": simtime,
        "Full Collision": full_collision,
        "Toa DATA": nodes[0].data_rectime,
        "Toa RTS": nodes[0].rts_rectime,
        "DIFS": nodes[0].packet.Tpream,
        "n_retry": n_retry,
        "CAD_prob": CAD_prob,
        "Packet length": packetLength,
        "targetSentPacket": targetSentPacket, 
        "targetSchedPacket": targetSchedPacket, 
        "Wbusy_min": Wbusy_min,
        "Wbusy_BE": Wbusy_BE,
        "Wbusy_maxBE": Wbusy_maxBE,
        "Wbusy_exp_backoff": Wbusy_exp_backoff,
        "gaussian_noise":gaussian_noise
    }




    sent = sum(n.n_data_sent for n in nodes)
    rts_sent = sum(n.n_rts_sent for n in nodes)
    
    res["TOTAL"]={
        "energy_in_CAD_J":sum( n.CAD_energy    for n in nodes),
        "energy_in_transmission_J":sum( [res["nodes"][n.nodeid]["energy_in_transmission_J"] for n in nodes]),
        "energy_in_listening_J":sum( n.total_listen_time * RX * V for n in nodes) / 1e6,
        "total_energy_J":sum( [res["nodes"][n.nodeid]["total_energy_J"] for n in nodes]),
        "end_simulation_time":" {}ms {}h".format(endSim, float(endSim/3600000)),

        "cumulated_TX_time_s":sum( [res["nodes"][n.nodeid]["cumulated_TX_time_s"] for n in nodes]),
        "number_of_CAD":sum (n.n_CAD for n in nodes),

        "sent_data_packets": sent / nrNodes,
        "mean_latency": sum (float(n.latency)/float(n.n_data_sent) for n in nodes) / nrNodes if node.n_data_sent>0 else -1,
        "min_success_latency": sum (n.min_success_latency for n in nodes) / nrNodes,
        "aborted_packets": sum (n.n_aborted for n in nodes)  / nrNodes,
        "collided_packets": nrCollisions  / nrNodes,
        "lost_packets": nrLost  / nrNodes,
        "dropped_packets": sum (n.n_dropped for n in nodes)  / nrNodes,
        

        
        "nrCollisions":nrCollisions,
        "nrReceived":nrReceived,
        "nrProcessed":nrProcessed,
        "nrSent":nrSent,
        "nrLost":nrLost,
        "nrScheduled":nrScheduled
    }

    # "mean_success_latency":
    sum_suc_lat=0
    sum_data_suc=0
    for n in nodes:
        if n.n_data_success>0:
            sum_suc_lat+=n.success_latency
            sum_data_suc+=n.n_data_success
    if sum_data_suc>0:
        res["TOTAL"]["mean_success_latency"]=sum_suc_lat/sum_data_suc
    else:
        res["TOTAL"]["mean_success_latency"]=-1        


    res["TOTAL"]["energy_per_success"]=res["TOTAL"]["total_energy_J"]/sum_data_suc if sum_data_suc>0 else -1
    res["TOTAL"]["total_energy_J"]/=nrNodes    
    res["TOTAL"]["cumulated_TX_time_s"]/=nrNodes


    if CANL22:
        res["TOTAL"]["cumulated_RX_time_s"]=sum( (n.total_listen_time) for n in nodes)/1000

    res["TOTAL"]["mean_retry"]=sum((float(n.total_retry)/float(n.n_data_sent)) for n in nodes)/nrNodes

    
    if CANL22:
        res["TOTAL"].update({

            "sent_rts_packets":rts_sent,
            "nrRTSCollisions":nrRTSCollisions,
            "RTS_received_packets": nrRTSReceived,
            "RTS_processed_packets": nrRTSProcessed,
            "RTS_lost_packets": nrRTSLost,

        })


    if sent>0:
        # data extraction rate switched to include losses
        der = (sent-nrCollisions)/float(sent)
        res["TOTAL"]["DER_method_2"]=der
        der = (nrReceived)/float(sent)
        res["TOTAL"]["DER"]=der

    res["TOTAL"]["duty_cycle"]=sum(res["nodes"][n.nodeid]["duty_cycle"] for n in nodes)/nrNodes

    res["TOTAL"]["PDR"]= sum(n.n_data_success for n in nodes)/sum(n.n_data_sent+n.n_dropped+n.n_aborted for n in nodes)
    try:
        res["TOTAL"]["payload_byte_delivery_ratio"]= sum(n.n_payload_success for n in nodes)/sum(n.n_payload_gen for n in nodes)
    except:
        res["TOTAL"]["payload_byte_delivery_ratio"]=0

    res["TOTAL"]["n_transmit"]= n_transmit
    res["TOTAL"]["mean_inter_transmit_time_ms"]=inter_transmit_time/float(n_transmit)


    #### Newer things

    TT_gen_times = [0]
    TT_IGTs=[]
    for n in nodes:
        TT_gen_times+=n.gen_times[1:]
        npks_gen=len(n.gen_times)
        IGTs=[n.gen_times[i]-n.gen_times[i-1] for i in range(1,npks_gen)]
        # print(npks_gen, IGTs, file=sys.stderr)
        # print(0/0)
        TT_IGTs+=IGTs
    
        res["nodes"][n.nodeid]["mean_IGT"] = np.mean(IGTs)
        res["nodes"][n.nodeid]["std_dev_IGT"] = np.std(IGTs)

    res["TOTAL"]["mean_IGT"]=np.mean(TT_IGTs)
    res["TOTAL"]["std_dev_IGT"]=np.std(TT_IGTs)

    # TT_gen_times=sorted(
    TT_gen_times.sort()
    npks_gen=len(TT_gen_times)
    Global_TT_IGTs=[TT_gen_times[i]-TT_gen_times[i-1] for i in range(1,npks_gen)]
    if keep_Global_TT_IGTs:
        res["TOTAL"]["Global_TT_IGTs"]=Global_TT_IGTs
    # else:
    res["TOTAL"]["short_IGTs"]=np.count_nonzero(np.array(Global_TT_IGTs)<1000)/len(Global_TT_IGTs)

    # channel_log_sorted=sorted(channel_log, key=lambda tup: tup[0])
    channel_log.sort(key=lambda tup: tup[0])
    
    busy_dur=0
    busy_start=channel_log[0][0]
    busy_stop=max([channel_log[i][0]+channel_log[i][1] for i in range(len(channel_log))])
    last_previous_end=channel_log[0][0]+channel_log[0][1]
    prev_tx_id=0
    for tx_id in range(1,len(channel_log)):
        tx=channel_log[tx_id]
        prev_tx=channel_log[prev_tx_id]
        if (tx[0]<last_previous_end): #starts before end of previous 
            if (tx[0]+tx[1]>last_previous_end) : #ends after end of previous 
                busy_dur += (tx[0]+tx[1]) - (prev_tx[0]+prev_tx[1])
                last_previous_end=tx[0]+tx[1]
                prev_tx_id=tx_id
        else:
            busy_dur += tx[1]
            last_previous_end=tx[0]+tx[1]
            prev_tx_id=tx_id

    res["TOTAL"]["channel_occupation"]=busy_dur/(busy_stop-busy_start)

    max_idv_chan_occ_time=1000*max([res["nodes"][n.nodeid]["cumulated_TX_time_s"] for n in nodes])
    totalsum_idv_chan_occ_time=1000*sum([res["nodes"][n.nodeid]["cumulated_TX_time_s"] for n in nodes])

    res["TOTAL"]["channel_overlap_ratio"]=(totalsum_idv_chan_occ_time-busy_dur)/(totalsum_idv_chan_occ_time-max_idv_chan_occ_time)

    if keep_chan_log:
        res["TOTAL"]["chanlog"]=channel_log

    powerChecks=0#len(powerCaptures)
    # print(powerChecks, "powerChecks", file=sys.stderr)
    
    nb_caps=0
    sum_in_ears=0
    max_gw_in_ears=0
    sum_in_ears_with_capture=0

    for n in nodes:
        res["nodes"][n.nodeid]["sum_in_ears"]=0
        res["nodes"][n.nodeid]["nb_caps"]=0
        res["nodes"][n.nodeid]["sum_in_ears_with_capture"]=0
        res["nodes"][n.nodeid]["powerChecks"]=0

        res["nodes"][n.nodeid]["max_overlap_degree"]=0
        res["nodes"][n.nodeid]["max_capture_overlap_degree"]=0


    for pcheck in powerCaptures:
        if pcheck[2]==-1:
            powerChecks+=1
            sum_in_ears+=pcheck[0]
            max_gw_in_ears=max(max_gw_in_ears,pcheck[0])
            if pcheck[1]:
                nb_caps+=1
                sum_in_ears_with_capture+=pcheck[0]

        else:
            res["nodes"][pcheck[2]]["sum_in_ears"]+=pcheck[0]
            res["nodes"][pcheck[2]]["max_overlap_degree"]=max(res["nodes"][pcheck[2]]["max_overlap_degree"],pcheck[0])
            res["nodes"][pcheck[2]]["powerChecks"]+=1
            if pcheck[1]:
                res["nodes"][pcheck[2]]["nb_caps"]+=1
                res["nodes"][pcheck[2]]["sum_in_ears_with_capture"]+=pcheck[0]
                res["nodes"][pcheck[2]]["max_capture_overlap_degree"]=max(res["nodes"][pcheck[2]]["max_capture_overlap_degree"],pcheck[0])
    

    res["TOTAL"]["GW_power_capture_ratio"]=nb_caps/powerChecks if powerChecks>0 else 0
    res["TOTAL"]["GW_overlap_degree"]=sum_in_ears/powerChecks if powerChecks>0 else 0
    res["TOTAL"]["GW_max_overlap_degree"]=max_gw_in_ears
    res["TOTAL"]["GW_capture_overlap_degree"]=sum_in_ears_with_capture/nb_caps if nb_caps>0 else 0

    for n in nodes:
        res["nodes"][n.nodeid]["power_capture_ratio"]=res["nodes"][n.nodeid]["nb_caps"]/res["nodes"][n.nodeid]["powerChecks"] if res["nodes"][n.nodeid]["powerChecks"]>0 else 0
        res["nodes"][n.nodeid]["mean_overlap_degree"]=res["nodes"][n.nodeid]["sum_in_ears"]/res["nodes"][n.nodeid]["powerChecks"] if res["nodes"][n.nodeid]["powerChecks"]>0 else 0
        res["nodes"][n.nodeid]["mean_capture_overlap_degree"]=res["nodes"][n.nodeid]["sum_in_ears_with_capture"]/res["nodes"][n.nodeid]["nb_caps"] if res["nodes"][n.nodeid]["nb_caps"]>0 else 0

    res["TOTAL"]["power_capture_ratio"]=sum(res["nodes"][n.nodeid]["power_capture_ratio"] for n in nodes)/nrNodes
    res["TOTAL"]["mean_overlap_degree"]=sum(res["nodes"][n.nodeid]["mean_overlap_degree"] for n in nodes)/nrNodes
    res["TOTAL"]["mean_capture_overlap_degree"]=sum(res["nodes"][n.nodeid]["mean_capture_overlap_degree"] for n in nodes)/nrNodes
    res["TOTAL"]["max_overlap_degree"]=sum(res["nodes"][n.nodeid]["max_overlap_degree"] for n in nodes)/nrNodes
    res["TOTAL"]["max_capture_overlap_degree"]=sum(res["nodes"][n.nodeid]["max_capture_overlap_degree"] for n in nodes)/nrNodes


        
    
    # print("-- END ----------------------------------------------------------------------"    )

    """    
    # this can be done to keep graphics visible
    if (graphics == 1):
        raw_input('Press Enter to continue ...')

    # save experiment data into a dat file that can be read by e.g. gnuplot
    # name of file would be:    exp0.dat for experiment 0
    fname = "exp" + str(experiment) + ".dat"
    print fname
    if os.path.isfile(fname):
        res = "\n" + str(nrNodes) + " " + str(nrCollisions) + " "     + str(sent) + " " + str(energy)
    else:
        res = "#nrNodes nrCollisions nrTransmissions OverallEnergy\n" + str(nrNodes) + " " + str(nrCollisions) + " "    + str(sent) + " " + str(energy)
    with open(fname, "a") as myfile:
        myfile.write(res)
    myfile.close()

    """
    # with open('nodes.txt','w') as nfile:
    #        for n in nodes:
    #            nfile.write("{} {} {}\n".format(n.x, n.y, n.nodeid))
    # with open('basestation.txt', 'w') as bfile:
    #        bfile.write("{} {} {}\n".format(bsx, bsy, 0))



    return res






#
# "main" program as used as "python lorasim3.py"
#
if __name__ == '__main__':

    # module for the genration of the topology
    import topo_builder

    import time
    import json
    JSON_EXPORT = True

    raw_start=time.localtime()
    start_time=time.strftime("%Y-%m-%d-%H-%M-%S", raw_start)
    experiment=4
    # nrNodes=20
    # nrNodes=400
    nrNodes=100
    log_events=False

    topos={}
    topos[0]=topo_builder.build_topo(nrNodes,experiment)

    pickle.dump(topos, open('results/{0}_topos.dat'.format(start_time), 'wb'))



    params={
        "start_time":start_time,
        "log_events":log_events,

        "nrNodes":nrNodes,
        "avgSendTime":1500000,
        "experiment":experiment,

        "topo":0,
        "topo_scale":1,
        "distrib":"expo",

        "n_retry":40,
        "var_CAD_prob":True, # has precedence 
        "CAD_prob":100, # has not
        "full_distances":True,
        "full_collision":True,
        "gaussian_noise":True,
        "powerCaptureThreshold":6,
        # "gamma_ED":3.1,
        # "gamma_GW":2.91,
        # "normal_gamma_ED":True,
        # "sigma_gamma_ED":.25,
        
        "variablePayloadSize":True,
        "normalPayloadSize":False,
        # "dist_min_payload_size":40,
        # "dist_max_payload_size":100,
        # "normaldist_mean_payload_size":60,
        # "normaldist_sigma_payload_size":15,

        "shuffle_start":False,
        # "Interrupts_on_header_valid":False,
        # "LoRa_PHY_HDR":0,

        "rayleigh_fading":True,
        "rayleigh_mean_dB" : 4,
        "keep_chan_log":False,
        "keep_Global_TT_IGTs":False,

        "with_CAD_and_back_off":True,

        "CANL22":False,
        "CANL22_P":0,
        "CANL22_L1_min":2,
        "CANL22_L1_MAX":7,
        "CANL22_L2":6,
        # "CANL_backoff_min":3,
        # "CANL_backoff_max":64,
        "CANL22_check_busy":True,
        # "CANL_RTS_min_payload_size":12,#18,#0,#8
        "CANL22_fair_factor":4,
        "CANL22_softer_fair":False,
        # "CANL_rts_hdr_size":4,
        # "CANL_data_hdr_size":4,
        # "CANL_RTS_PHY_HDR":0,

        "ideal_FIFO":False,

    }

    # # sys.stdout=stdout_print_target
    # res=main_with_params(params)

    # sys.stdout=stdout_print_target
    params["CANL22"]=True
    # params["ideal_FIFO"]=True

    res3=main_with_params(params)

    sys.stdout=stdout_print_target

    for key in res3["TOTAL"]:
        print("{0}:".format(key))
        if isinstance(res3["TOTAL"][key], list):
            if len(res3["TOTAL"][key])>12:
                print(res3["TOTAL"][key][:12])
            else:
                print(res3["TOTAL"][key])
        else:
            print(res3["TOTAL"][key])


    pickle.dump(params, open('results/{0}_params.dat'.format(start_time), 'wb'))
    pickle.dump(res3, open('results/{0}_data.dat'.format(start_time), 'wb'))

    if JSON_EXPORT:
        json_dict = json.dumps(res3, indent=4)
        with open('results/{0}_results.json'.format(start_time), "w") as outfile:
            outfile.write(json_dict)


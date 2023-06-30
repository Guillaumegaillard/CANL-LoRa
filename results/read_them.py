########## Python script to read the data presented for conference paper:
########## IEEE ISCC 2023 /S21.2 CANL LoRa Collision Avoidance by Neighbor Listening for Dense LoRa Networks 


import pickle
import numpy as np
import json


WORK_on_topo = False
WORK_on_topo = True

WORK_some_data = False
WORK_some_data = True

EXPORT_DATA_TO_JSON = False
EXPORT_DATA_TO_JSON = True

# start dates of simulations
times=['2023-05-24-15-20-32','2023-05-23-21-01-24','2023-05-24-14-11-32']

for start_time in times:

    data_CANL=pickle.load(open('{0}_data.dat'.format(start_time), 'rb'))
    config_CANL=pickle.load(open('{0}_config.dat'.format(start_time), 'rb'))
    topos_CANL=pickle.load(open('{0}_topos.dat'.format(start_time), 'rb'))

    if WORK_on_topo:
        for repe in topos_CANL: # simulations are repeated independently over a given number of topologies
            topo=topos_CANL[repe]
            print(topo.keys())

            devices=topo["nodes"]

            for dev in devices:
                # do something
                print(devices[dev].keys())
                break
            break


    if WORK_some_data:

        n_repes=config_CANL["n_repes"]                  # times the same simulations are repeated independently over other topology instance
        tpkts=config_CANL["tpkts"]                      # sequence of inter-packet generation times
        scales=config_CANL["scales"]                    # sequence of scaling factors of the distances in topology 
        n_Nodes_tab=config_CANL["n_Nodes_tab"]          # sequence of number of active devices
        CANL_lmins=config_CANL["CANL_lmins"]            # sequence of minimum length of listen window for CANL
        CANL_lmaxes=config_CANL["CANL_lmaxes"]          # sequence of maximum length of listen window for CANL
        n_retries=config_CANL["n_retries"]              # sequence of max allowed retries for both CANL and CAD+Backoff
        experiment=config_CANL["experiment"]            # a setting in lorasim3.py, w.r.t. the chosen LoRa PHY layer
        var_CAD_prob=config_CANL["var_CAD_prob"]        # True if CAD success depends on the distance model defined in lorasim3.py
        fixed_CAD_prob=config_CANL["fixed_CAD_prob"]    # if var_CAD_prob is False, this value is taken as percentage of success of CAD  
        full_distances=config_CANL["full_distances"]    # if True, ED/ED interactions depend on the ED-ED distance, otherwise as original version (distance to GW)
        log_events=config_CANL["log_events"]            # a resource angry logger
        protos=config_CANL["protos"]                    # the list of protocols to compare in simulation
        timeline_tuples=config_CANL["timeline_tuples"]  # a sequence of short periods of communication to be graphically represented 
        maxDist_dev_gw=config_CANL["maxDist_dev_gw"]    # the maximum radius considered in the simulation
        pl_sizes=config_CANL["pl_sizes"] if "pl_sizes" in config_CANL else [(70,70)] # sequence of payload distribution of sizes
        rayleigh_means=config_CANL["rayleigh_means"] if "rayleigh_means" in config_CANL else [1] # sequence of mean impacts of rayleigh phenomenon
        normalsizes=config_CANL["normalPayloadSize"] if "normalPayloadSize" in config_CANL else False # switch uniform/normal distribution of payload sizes
        gamma_EDs=config_CANL["gamma_EDs"] if "gamma_EDs" in config_CANL else [3] # sequence of PLEs for ED-ED links

        # comparing canl performance wrt listen window's sizes 
        if len(protos)==1:
            protos_list=["CANL22_{0}/{1}".format(CANL_lmins[clmm_id],CANL_lmaxes[clmm_id]) for clmm_id in range(len(CANL_lmaxes))] #ordered list
        else: # otherwise compare different protocols
            protos_list=[proto for proto in protos]

        data_index=0

        nnodes=n_Nodes_tab[0]
        for tpkt_id in range(len(tpkts)):
            # inter-packet times and node density are related in case of evaluating a variation of node density  
            if len(n_Nodes_tab)>1:
                nnodes=n_Nodes_tab[tpkt_id]
            tpkt=tpkts[tpkt_id]
            for scale in scales:
                for n_retry in n_retries:
                    for pls in pl_sizes:
                        for rlmean in rayleigh_means:
                            for gamma_ED in gamma_EDs:
                                for repe in range(n_repes):
                                    for proto in protos_list:

                                        # here in the scope of a single independant simulation run
                                        print(data_CANL[data_index]["TOTAL"].keys())
                                        print(data_CANL[data_index]["nodes"][18].keys())
                                        print(0/0)
                                        # do something

                                        data_index+=1
        # save changes ?
        if False:
            pickle.dump(data_CANL, open('{0}_data.dat'.format(start_time), 'wb'))

    if EXPORT_DATA_TO_JSON:
        json_dict = json.dumps(data_CANL, indent=4)
        with open('{0}_results.json'.format(start_time), "w") as outfile:
            outfile.write(json_dict)

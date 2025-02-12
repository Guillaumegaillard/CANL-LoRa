## IEEE ISCC 2023 /S21.2 CANL LoRa Collision Avoidance by Neighbor Listening for Dense LoRa Networks 

The data exposed in the paper is available in zip folder **data_CANL_IEEE_ISCC_2023**.

A python script is provided to read them: extract the data, position yourself in its directory, adapt and run the python script.

### ERRATUM detected on 2025-02-11

The energy consumption needed to do a CAD was computed wrong; in order to correct the data, insert the following in [the reader script](https://github.com/Guillaumegaillard/CANL-LoRa/blob/main/results/read_them.py):

<style>
  code {
    white-space : pre-wrap !important;
    word-break: break-word;
  }
</style>


```python 
for node in data_CANL[data_index]["nodes"]:
    data_CANL[data_index]["nodes"][node]["number_of_CAD"] = round(data_CANL[data_index]["nodes"][node]["energy_in_CAD_J"] * 1e3 / (2**12/125 * 4 * 169.54/3600/1e9 * 3.3))

    data_CANL[data_index]["nodes"][node]["total_energy_J"] -= data_CANL[data_index]["nodes"][node]["energy_in_CAD_J"]

    data_CANL[data_index]["nodes"][node]["energy_in_CAD_J"] = 169.54*3600/1e9 * 3.3 * data_CANL[data_index]["nodes"][node]["number_of_CAD"]

    data_CANL[data_index]["nodes"][node]["total_energy_J"] += data_CANL[data_index]["nodes"][node]["energy_in_CAD_J"]

    data_CANL[data_index]["nodes"][node]["energy_per_success"] = data_CANL[data_index]["nodes"][node]["total_energy_J"]/data_CANL[data_index]["nodes"][node]["success_data_packets"] if data_CANL[data_index]["nodes"][node]["success_data_packets"]>0 else -1

data_CANL[data_index]["TOTAL"]["number_of_CAD"] = sum(data_CANL[data_index]["nodes"][node]["number_of_CAD"] for node in data_CANL[data_index]["nodes"]) 
data_CANL[data_index]["TOTAL"]["energy_in_CAD_J"] = sum(data_CANL[data_index]["nodes"][node]["energy_in_CAD_J"] for node in data_CANL[data_index]["nodes"]) 
data_CANL[data_index]["TOTAL"]["total_energy_J"] = np.mean([data_CANL[data_index]["nodes"][node]["total_energy_J"] for node in data_CANL[data_index]["nodes"]]) 

sum_success = sum([data_CANL[data_index]["nodes"][node]["success_data_packets"] for node in data_CANL[data_index]["nodes"]])
data_CANL[data_index]["TOTAL"]["energy_per_success"] = sum([data_CANL[data_index]["nodes"][node]["total_energy_J"] for node in data_CANL[data_index]["nodes"]])/sum_success if sum_success>0 else -1

``` 

Fig 5. in [the paper](https://hal.science/hal-04191330/file/CANL-HAL.pdf) is not really affected since the CAD consumption is neglectable compared with the transmissions and receptions. The conclusion remains identical.
The corrected version of the figure is available on this repository:
https://github.com/Guillaumegaillard/CANL-LoRa/blob/main/results/ERRATUM_CANL_DATA_ISCC_2023_fig5_energy_success.pdf 

Regards,

Guillaume Gaillard
guillaume.gaillard "at" univ-pau.fr




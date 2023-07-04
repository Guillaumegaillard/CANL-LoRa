# CANL-LoRa/LoRaSim3
This code is implementing the simulation used in the following paper that will be presented in ISCC 2023:

["Guillaume Gaillard and Congduc Pham. 2023. CANL LoRa: Collision Avoidance by Neighbor Listening for Dense LoRa Networks. In ISCC 2023, July 2023."](https://edas.info/web/ieeeiscc2023/index.html#S1569617839)


## Context and Purpose
In order to simulate CANL, a Listen-Before-Talk mechanism in LoRa, we developped an updated version of LoRaSim: ["T. Voigt and M. Bor, *LoRaSim*, 2016."](https://mcbor.github.io/lorasim/)

## Installation and Requirements

I worked on a Ubuntu 22.04.2 LTS (Xubuntu), with Anaconda Python distribution and some additional modules (installed using "pip install"):
 * Python 3.9.12 (main, Apr  5 2022, 06:56:58)

A table from ```conda list```:

 Name             |           Version          | Build          | Channel   | Install
 ---              |           ---              | ---            | ---       | ---
 anaconda         |           2022.05          |        py39_0  |           |
 python           |           3.9.12           |     h12debd9_0 |           | 
 simpy            |           4.0.1            |         pypi_0 |   pypi    | ```pip install simpy```


## Python scripts - usage
Modify the parameters and variables inline and run the scripts.
### Simulation:
```bash
python lorasim3.py
```
### Build topologies:
```bash
python topo_builder.py
```


Many comments inline should help understanding the code and its options. 
## Contact
Questions and suggestions are welcome :).

[Guillaume Gaillard](https://hal.inria.fr/search/index/?q=%2A&authIdHal_s=guillaumegaillard) [guillaume.gaillard "at" univ-pau.fr]


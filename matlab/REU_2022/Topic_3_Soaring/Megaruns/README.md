# Megaruns

## Basic Description

As described in the [Simulation](../Simulation/README.md) section, the results of this project were gathered by running large numbers of simulations in batches, called "megaruns". Over the course of this project, 7 megaruns were executed, totaling over 140,000 individual simulations. This immense computation was made possible by distributing the work across 160 threads on the West Virginia University Robotics server cluster computer.

While the first megarun's results are not included (due to being based on deprecated simulation structure), the results of megaruns 2-7 are organized as follows:
- [Megaruns 2-3](/Megarun_2-3/)
- [Megarun 4](/Megarun_4/)
- [Megarun 5](/Megarun_5/)
- [Megarun 6](/Megarun_6/)
- [Megarun 7](/Megarun_7/)

## Folder Structure
The purpose of each megarun folder is to archive the megarun simulation batch it represents. Each folder will contain the following 2 files:
- Verbose output data from one specific simulation
- Limited output data from all simulations

The verbose output data from one specific simulation includes many output metrics quantifying the success of the swarm with the simulation. Additionally, this file contains the simulation "law", labeled as 'SL', containing all parameters used to define the simulation. This data is useful for recreating an exact simulation from the megarun. 

The limited output data from all simulations includes the *varied* input parameters and the resulting output metrics of all simulations in the megarun. This data is useful for analyzing trends between input parameters and output metrics at a large scale.

Note: The individual simulation law and the database of varied input parameters can be used together to exactly recreate any simulation in the megarun. For any individual simulation, if a parameter is not listed in the batch data, then that parameter was not varied and was equal to the respective parameter in the single simulation's sim law.
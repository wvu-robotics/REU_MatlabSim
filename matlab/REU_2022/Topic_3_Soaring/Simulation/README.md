# Simulation

## Simulation Architecture
The simulation contains the following major components:
- Folders
    - Agent_Control_Functions: Contains MATLAB functions to update an agent's state.
    - Code_of_Laws: Contains parameter sets used to define a simulation or batch of simulations. An Excel file containing parameter sets is lovingly referred to as a "Law".
    - Find_Neighborhood_Functions: Contains MATLAB functions to calculate nearby agents.
- Simulation Object Classes
    - Agent: Contains the properties and rendering of an individual agent.
    - Swarm: Manages all agents in a swarm, while simulating agent decentralization.
    - Thermal: Contains the properties of an individual thermal updraft.
    - ThermalMap: Manages all thermal updrafts.
- Logistics
    - MainScriptFunction: A class used to run a simulation from start to finish.
    - MasterScriptIteration: A script used to load simulation parameters, generate all simulation combinations, and run multiple instances of MainScriptFunction (single- or multi-threaded).
    - Utility: A class containing miscellaneous, useful functions.

## Running a Simulation

To run a simulation with the desired parameter set, complete the following steps:

1. Choose a "law" file (Excel file containing parameter sets) to simulate.
2. Open "MasterScriptIteration" in MATLAB (we use R2021b).
3. Edit line 7 with the name of your chosen "law" file. For example, if your chosen "law" file is "Law_40SurvivorsExample.xlsx", lines 6-7 should read:
```MATLAB
%% Choose sim law
simLawExcel = "Law_40SurvivorsExample.xlsx";
```
4. Run "MasterScriptIteration" in MATLAB.

### Understanding the Output
If succesful, the program will generate output text in MATLAB's command window over time.

1. The program will confirm it was able to read a parameter set from the chosen "law" file.
```
Parsing Excel sheet... Done!
```

2. Next, the program will analyze the parameter set and generate all unique combinations of the specified parameters. For this example, only one value is specified for each parameter, so only one unique combination can be generated.
```
Preparing SL for 1 combinations... Done!
No variables to iterate over.
Starting simulations for 1 combination(s).
```

3. During the course of a simulation, progress markers will be printed showing simulation execution from 0-100%, eventually announcing when the simulation is finished.
```
00.5% through Run #1 
01.0% through Run #1 
01.5% through Run #1 
02.0% through Run #1 
...
98.0% through Run #1 
98.5% through Run #1 
99.0% through Run #1 
99.5% through Run #1 
100.0% through Run #1 
Finished sim 1.
```

4. Finally, the program will announce when all simulations are complete and its attempt to finalize all simulation data.
```
Finished simulations.
Combining files... Done!
Done!
```

All output data from the simulation will be located in "Topic_3_Soaring/OutputMedia", inside a folder for that day, inside a folder for that simulation batch (with a simulation code describing the date and time the batch was started).

## Editing Simulation Parameters (Micro-Changes)

Within the existing simulation architecture, all simulation parameters can be varied to produce very different results, likely displaying different swarm behavior. 

### How to Change Simulation Parameters
1. Open a "law" file.
    - Recommended: Copy "Law_40SurvivorsExample.xlsx" as a template for your own parameter set. Many parameters were handpicked to produce nice visuals and are a good starting point for your first simulations.
2. For a given parameter, change its values. Some example parameter entries:

| Units, Description        | Variable Name | Value 1      | Value 2      | Value 3 |
|---------------------------|---------------|--------------|--------------|---------|
| [s]                       | dt            | 0.5          |              |         |
| [s]                       | totalTime     | 7200         | 14400        | 10000   |
| [m], bounds of square map | mapSize       | [-4000,4000] | [-6000,6000] |         |

- If a parameter is only defined with one value, all simulations will use that value for the parameter, and the parameter will not be varied. Example: "dt" in the table above.
- However, if multiple values are listed for a parameter, the parameter will be varied and all combinations of simulations will be generated for each value of the parameter. Values should be listed starting in the left column and expanding to the right. Example: "totalTime" and "mapSize" in the table above.
- Note: A single "value" of a parameter maps directly to an entire Excel cell. Multiple cells must be used to define different values of a parameter. Example: "mapSize" in the table above. The first parameter value specifies the vector [-4000, 4000] and the second parameter value specifies the vector [-6000, 6000].

3. After defining your parameter set as desired, follow the above steps in [Running a Simulation](#running-a-simulation) to run your simulation(s). With the above example table, with one value for dt, three values for totalTime, two values for mapSize, and an assumed one value for all other parameters, then **six** simulation combinations would be generated and executed.

### Which Parameters to Change
At the time of publishing our paper, each simulation is defined by 77 parameters. Many of the parameters were made easily editable out of principle, rather than out of research motivation. Due to the large number of simulation combinations generated from changing many parameters, it is *highly* recommended to choose a small number of parameters to vary.

Within the scope of our paper, we primarily varied the following parameters to observe different swarm behaviors:
- **numAgents**: number of agents
- **numThermals**: number of thermal updrafts
- **rngSeed**: seed used to initialize the random number generator; very helpful for controlling randomness in the simulation sets
- **cohesion**: gain of cohesion towards other agents
- **separation**: gain of separation away from other agents
- **alignment**: gain of alignment with other agents
- **migration**: gain of migration towards center of map

## Running the Publication-Focused Simulation

One specific simulation was analyzed in-detail in the produced publication. This simulation was produced with the parameter set defined in [Law_40SurvivorsExample](Code_of_Laws/Law_40SurvivorsExample.xlsx).

## Adding New Simulation Features (Macro-Changes)

While the current simulation infrastructure leaves many results to be generated and analyzed, completely new features can be easily integrated into the existing architecture.

### Adding New Agent Controllers or Find Neighborhood Functions

Beyond editing simulation parameters, the easiest change to make is adding new agent controllers or new find neighborhood functions. To add a new version of either of these, it is recommended to go to their respective folders, [Simulation/Agent_Control_Functions](Agent_Control_Functions/) and [Simulation/Find_Neighborhood_Functions](Find_Neighborhood_Functions/), and to copy an existing function to preserve function syntax. Give your new function a unique name, edit the function, and use the function's name as a parameter in your "law" file. The respective paremeters are "funcName_agentControl" and "funcName_findNeighborhood".

### Adding Other Features and Parameters

The simulation's code was designed with modularity in mind, so feel free to try adding in custom classes and functionality. Additionally, adding new simulation parameters is very easy, as parameters are read in and stored in a struct, named SL ("SimLaw"). MasterScriptIteration reads in all simulation parameters, unaware of their order or how they are used in the simulation.

As such, the only requirements for adding a new parameter are to take an existing "law" file, write a new parameter name at the bottom, and give it at least one value.

When a simulation is executed in MainScriptFunction, it is given a single parameter set, stored in the SL struct. The SL struct is already passed around through most parts of the simulation. As such, if a new parameter "param" is added to the parameter set, its value will be accessible in most parts of the code via "SL.param".

## Next Steps

We have made all of our code available under an MIT license. This simulation software has a *lot* of potential and should be further explored! More results can be obtained by generating more simulations with a more extensive simulation set. Additionally, the scope of our project was supported by many assumptions, but more features could be added to explore the effects of a changing environment, different agent types, or multiple agent controllers acting concurrently within a swarm.
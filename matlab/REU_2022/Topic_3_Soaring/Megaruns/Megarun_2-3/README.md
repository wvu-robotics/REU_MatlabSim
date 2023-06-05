# Note for Megaruns 2-3

In this version of the code, finalHeightMin and finalHeightAvg were calculated slightly differently than newer code versions. The difference only has an effect when a simulation results in zero surviving agents.

| Code Version | finalHeightMin         | finalHeightAvg |
|:------------:|:----------------------:|:--------------:|
| Old          | 0                      | 0              |
| New          | SL.agentCeiling (2600) | NaN            |
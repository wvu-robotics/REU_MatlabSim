# Miscellaneous

## Explanation
Megarun data was sometimes saved incorrectly, with rngSeed sometimes duplicated for each simulation and timeStart and timeEnd being concatonated into a massive char array. The [FixData.m](FixData.m) file was used to clean some of the megarun data. 
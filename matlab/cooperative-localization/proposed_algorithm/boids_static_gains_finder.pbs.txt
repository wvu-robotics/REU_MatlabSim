#!/bin/bash
 
# Name of job:
#PBS -N generate_boids_gain_costs
 
# Where to write stderr:
#PBS -e generate_boids_gain_costs.err
 
# Where to write stdout: 
#PBS -o generate_boids_gain_costs.out
 
# Specify number of nodes, processors (really threads) per node, and the maximum allowed run time for the job
# Can also specify max memory requested with something like mem=10gb
#PBS-l nodes=1:ppn=80,walltime=24:00:00

# Keep job output and joint output and error
#PBS -k o
#PBS -j oe

# Change directory to the directory the job was submitted from
cd $PBS_O_WORKDIR

# Run the program
/localhome/trevor_smith/matlab/bin/matlab -r "run('monte_carlo_gain_finder.m');exit;"

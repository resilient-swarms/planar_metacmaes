#!/bin/bash


#SBATCH --tasks-per-node=1   # Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=24:00:00         # walltime
#SBATCH --job-name=planar_test

singularity exec planar_metacmaes_installed.sif bash run_metacmaes_test.sh $1 $2 $3 $4 $5

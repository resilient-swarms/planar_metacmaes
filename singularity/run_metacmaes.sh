#!/bin/bash


#SBATCH --exclusive   # Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=60:00:00         # walltime
#SBATCH --job-name=planar_evol

singularity exec planar_metacmaes_installed.sif bash run_metacmaes_evolution.sh "$1" "$2" "$3" "$4"




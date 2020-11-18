#!/bin/bash


#SBATCH --tasks-per-node=1   #Tasks per node
#SBATCH --nodes=1                # Number of nodes requested
#SBATCH --time=60:00:00         # walltime
#SBATCH --mem-per-cpu=10G  # actually no need to specify when nodes > 20
#SBATCH --job-name=test_task_max

singularity exec planar_metacmaes_installed.sif bash run_metacmaes_maxtask.sh $1 $2 $3 $4




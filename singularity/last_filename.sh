#!/bin/bash


shopt -s extglob
alldirs=`ls /scratch/${1}/singularity/Results`
for dir in $alldirs; do
cd /scratch/${1}/singularity/Results/$dir
for exp in `ls `; do
cd $exp

max=-1
files=`ls -d !(*.*)` # gen_files don't have extension
for gen_file in $files; do
        number=${gen_file#"gen_"}
        #echo $number
        if (( $number > $max )); then
                max=$number
        fi

done


cd ..

echo "$dir/$exp :  $max"
done
done


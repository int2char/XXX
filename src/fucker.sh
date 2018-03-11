#!/bin/bash
rm -f *.txt
nodes=(1000)
types=(0)
graphtype=("ER")
powers=(6)
declare -i edge task biao
for node in ${nodes[@]}
do
   path="../result/$node/${types[0]}/"	
   edge=`cat $path/Graph.txt |wc -l`
   edge=$edge+$edge
   echo $edge
    for power in ${powers[@]}
    do
      task=$[$power*$node]
      biao=$type
      nvcc -O3  -std=c++11 *.cpp *.cu --gpu-architecture=compute_35 --gpu-code=sm_35 -I ../include -I ../cplex_include -L ../lib -lconcert -lcplex -lilocplex -lm -lpthread -DIL_STD -DCAPACITY=200 -DNODE=$node -DEDge=$edge -DTask=$task -DTYPE="\"${graphtype[0]}\"" -DINPUTFILE=\"$path\" -DGANOEX=1
    done
done


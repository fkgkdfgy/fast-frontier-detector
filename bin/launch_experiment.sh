#!/bin/bash

RUNS=1 			# number of trials per configuration
#FREQ_LIMIT=3		# how many seconds between frontier computations
#FREQ_STEP=3		# step of frontier computations

NUM_OF_DESIRED_SAMPLES=570

i=0
j=0
DIR=
OUTPUT_DIR="results"
#$OUTPUT_DIR/
maps[0]="simulation.clf"
maps[1]="ubremen-cartesium-demo2.gfs.log"
maps[2]="fr079-sm.log"
maps[3]="fr-campus-20040714.carmen.log"
maps[4]="mit-csail-3rd-floor-2005-12-17-run4.flaser.log"
maps[5]="edmonton_3.log"

# delete old outputs
rm -rf $OUTPUT_DIR
mkdir $OUTPUT_DIR

# copy the final graph generator
cp ../octave/showResults.m $OUTPUT_DIR/
cp ../octave/showParticleTimes.m $OUTPUT_DIR/
cp ../octave/showContourLengths.m $OUTPUT_DIR/
cp ../octave/errorb.m $OUTPUT_DIR/

# copy contour and frontiers graph generators
cp ../octave/show.m .
cp ../octave/showGraph.m .


# select a configuration
for currMap in "${maps[@]}";
do
	# select map and
	#currMap=${maps[$i]}
	echo $currMap
	#currMap=${mapFile:1} 
	echo current map is: $currMap
	
	# write input sample file
	numLines=$(wc -l maps/$currMap | awk '{print $1}') 
	totalSamples=$((numLines/NUM_OF_DESIRED_SAMPLES))
	echo $totalSamples > line_skip.txt

	# create a directory for the map	
	mkdir $(printf %s/%s $OUTPUT_DIR $currMap)
	
	# create a directory for the setting
	#mkdir $(printf %s/%s $OUTPUT_DIR $currMap)

	# repeat the selected configuration
	for run in `seq 1 $RUNS`;
	do		
		# launch experiment
		#./gfs_simplegui -filename $currMap -maxUrange 5.5 -maxUrange 5.5 -generateMap
		./gfs_nogui -filename maps/$currMap -maxUrange 5.5 -maxUrange 5.5 -generateMap

		# create a directory for the test
		newDir=$(printf %s/%s $OUTPUT_DIR $currMap)
										#mkdir $newDir
		

		# create contour and frontiers images
		#octave show.m
		
		# create graphs
		octave showGraph.m

		# copy output to a separate directory
		mkdir $newDir/contours
		mv contour*.txt $newDir/contours/

		mkdir $newDir/frontiers
		mv frontier*.txt $newDir/frontiers/

		mkdir $newDir/executions		
		mv ffd_executions.txt $newDir/executions/
		mv exploration_execution_rafael.txt $newDir/executions/
		mv exploration_execution_wolfram.txt $newDir/executions/
		mv partial_ffd_executions.txt $newDir/executions/
		mv graph* $newDir/executions/
		
		mkdir $newDir/maps
		mv MAP-*.PNG $newDir/maps/
	done
done
cd results
octave showResults.m
octave showContourLengths.m
octave showParticleTimes.m
exit



echo $RUNS

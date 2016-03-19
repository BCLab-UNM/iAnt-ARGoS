#!/bin/bash
# This script runs the XML file/ argos file x100 with a new random seed each time by using
# the random_argos.py by Jeffrey Nichol.


#Generate Data for Random Distribution Stimulation
#for _ in {1..3}; do python random_argos.py; done #command to run file n times
python experiments.py
open -a TextEdit  iAntTagData.txt #open data file

mv iAntTagData.txt $1 #$1 let's you specify the name of the output file.
 
echo "DONE"

 





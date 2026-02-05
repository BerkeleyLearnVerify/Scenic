In order to run cosimulation using both METSR and Carla the user needs to supply both a SUMO and openDrive file.
To generate the associated SUMO file from an opendrive file one run the following command: 

netconvert --opendrive-files example.xodr --output-file example.net.xml --geometry.min-radius.fix --geometry.remove --opendrive.curve-resolution 1 --opendrive.import-all-lanes --output.original-names --tls.guess-signals --tls.discard-simple --tls.join


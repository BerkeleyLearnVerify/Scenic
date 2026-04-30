# To run the CoSimulator
 Start by opening and running both CARLA and METS-R 

# Setting up METSR:   
1. Create a virtual env `python -m venv metsr_venv`
2. Run `git clone clone https://github.com/umnilab/METS-R_HPC`
3. Create an instance of METSR sim 
a. An example template for this is provided in `run_blank.py`

For METSR specific details please review the its documentation: https://umnilab.github.io/METS-R_doc/`

# Setting up CARLA:
1. Ensure CARLA is open an running on your desktop

For CARLA specific details please review the CARLA examples folder or its documentation: https://carla.readthedocs.io/en/latest/python_api/

# Finally to run the scenic program first ensure
- The `globalParameter address` set at the top of your Scenic program matches your current IP address to allow Scenic to connect to CARLA
- The `globalParameter map` set at the top of your Scenic program provides the path to the corresponding `xodr` map you would like to run
- Ensure that this parameter matches the configuration file used to spin up METSR
- The `globalParameter xml_map` matches provides the path to the corresponding `xml` map. 
        `xml` map files can be generated from the provided sumo file found in `assets\maps\CARLA` using the command noted at the end fo this file

Now the simulator can be started running the following command: `scenic [your_file.scenic] --simulate --2d`  
 
In order to run cosimulation using both METSR and Carla the user needs to supply both a SUMO and openDrive file.
To generate the associated SUMO file from an opendrive file run the following command: 

```netconvert --opendrive-files example.xodr --output-file example.net.xml --geometry.min-radius.fix --geometry.remove --opendrive.curve-resolution 1 --opendrive.import-all-lanes --output.original-names --tls.guess-signals --tls.discard-simple --tls.join```

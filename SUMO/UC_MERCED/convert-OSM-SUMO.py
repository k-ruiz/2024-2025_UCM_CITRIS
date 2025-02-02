import subprocess

#path to input nd output 
osm_file = "uc_merced.osm"
net_file = "uc_merced.net.xml"

#run netconvert to create SUMO network file 
subprocess.run(["netconvert", "--osm-files", osm_file, "-o", net_file], check = True)

print("SUMO Network file created :D", net_file)

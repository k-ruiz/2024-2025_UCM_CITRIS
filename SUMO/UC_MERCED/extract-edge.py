import subprocess

osm_file = "uc_merced.osm"
net_file = "uc_merced.net.xml"
prefix = "uc_merced"

#runs netconvert with plain output
subprocess.run([
    "netconvert", 
    "--osm-files", osm_file, 
    "-o", net_file, 
    "--plain-output-prefix", prefix
], check=True)

print("Generated edge file:", prefix + ".edg.xml")


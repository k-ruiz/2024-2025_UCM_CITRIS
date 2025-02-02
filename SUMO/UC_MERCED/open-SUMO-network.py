import subprocess

net_file = "uc_merced.net.xml"

#opens netedit gui
subprocess.run(["netedit", net_file], check=True)

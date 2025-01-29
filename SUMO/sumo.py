import traci
import sumolib

# Load the network file
net = sumolib.net.readNet('path/to/your/network.xml')

# Start the simulation
sumoBinary = sumolib.checkBinary('sumo')
traci.start([sumoBinary, "-c", "path/to/your/config.sumocfg"])

# Run the simulation for 1000 steps
for step in range(1000):
    traci.simulationStep()

    # Get the position of a vehicle
    vehicle_id = "vehicle_0"
    if vehicle_id in traci.vehicle.getIDList():
        x, y = traci.vehicle.getPosition(vehicle_id)
        print(f"Vehicle {vehicle_id} is at ({x}, {y})")

# Close the simulation
traci.close()

#pip install sumolib traci

#only downloaded what i needed here, will need to figure out how this actually works later on 

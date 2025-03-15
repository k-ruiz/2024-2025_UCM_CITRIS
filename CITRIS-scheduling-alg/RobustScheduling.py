# this is code to create a robust scheduling algorithm for VTOL airmobility between the 4 northern CITRIS affiliated UC campuses.
# the goal of this algorithm is to create a scheduling program that would be able to tolerate possible time delays with little affect to the overall schedule. 
# this work is done for the CITRIS Aviation Prize 2024-2025
# written by Kyra Ruiz
import heapq
import random
from datetime import timedelta, datetime

class VTOL:
    def __init__(self, id, turnaround_time=timedelta(minutes=10)):
        self.id = id
        self.turnaround_time = turnaround_time
        self.available_at = datetime.now()
        self.location = None  # tracks where the vtol is at
    
    def schedule_mission(self, start_time, duration, destination):
        adjusted_start_time = max(self.available_at, start_time)
        self.available_at = adjusted_start_time + duration + self.turnaround_time
        self.location = destination  # updates the vtol destination
        return adjusted_start_time, self.available_at

class Scheduler:
    def __init__(self, num_vtols, campuses):
        self.vtols = [VTOL(i) for i in range(num_vtols)]
        self.event_queue = []
        self.campuses = campuses  # List of campus names
    
    def add_mission(self, mission_name, requested_start_time, duration, origin, destination):
        available_vtols = [v for v in self.vtols if v.location == origin or v.location is None]
        if not available_vtols:
            print(f"No available VTOL for {mission_name} at {origin}, delaying assignment.")
            return
        
        vtol = min(available_vtols, key=lambda v: v.available_at)
        actual_start_time, completion_time = vtol.schedule_mission(requested_start_time, duration, destination)
        heapq.heappush(self.event_queue, (actual_start_time, completion_time, mission_name, vtol.id, origin, destination))
        print(f"Scheduled {mission_name} on VTOL {vtol.id} from {origin} to {destination}, from {actual_start_time} to {completion_time}")
    
    def handle_delays(self):
        delay_types = {     # list of possible reasons for time delays, along with their maximum/minimum time that they would take 
            "refueling": timedelta(minutes=random.randint(5, 15)),
            "passenger_late": timedelta(minutes=random.randint(2, 10)), 
            "technical_issue": timedelta(minutes=random.randint(10, 30)),
            "inspection_maintinence": timedelta(minutes=random.randint(15,30)),
            "loading_unloading": timedelta(minutes=random.randint(5,10)),
            "medical_necessity": timedelta(minutes=random.randint(15,30)),
            "security_check": timedelta(minutes=random.randint(5,20))
        }
        delayed_events = []
        while self.event_queue:
            actual_start_time, completion_time, mission_name, vtol_id, origin, destination = heapq.heappop(self.event_queue)
            delay_reason, delay = random.choice(list(delay_types.items()))
            new_completion_time = completion_time + delay
            delayed_events.append((actual_start_time, new_completion_time, mission_name, vtol_id, origin, destination))
            print(f"{mission_name} delayed due to {delay_reason} by {delay}, now completing at {new_completion_time}")
        
        for event in delayed_events:
            heapq.heappush(self.event_queue, event)

# callout code
campuses = ["UC Davis", "UC Merced", "UC Santa Cruz", "UC Berkeley"]
scheduler = Scheduler(num_vtols=4, campuses=campuses) # this is assuming 4 vtols total

# the missions are arbitrarily chosen, will need to figure out what the best routing will be. 
scheduler.add_mission("Flight A", datetime.now() + timedelta(minutes=5), timedelta(minutes=20), "UC Davis", "UC Merced")
scheduler.add_mission("Flight B", datetime.now() + timedelta(minutes=12), timedelta(minutes=15), "UC Merced", "UC Santa Cruz")
scheduler.add_mission("Flight C", datetime.now() + timedelta(minutes=25), timedelta(minutes=25), "UC Santa Cruz", "UC Berkeley")
scheduler.add_mission("Flight D", datetime.now() + timedelta(minutes=35), timedelta(minutes=30), "UC Berkeley", "UC Davis")

# simulate the delays
scheduler.handle_delays()

# # things that need to be done:
# - would need to implement a weighting factor for how rnndom the possible delays are (how possible these things would happen)
# - would need to make it possible so that multiple delays could happen, or none (none would be unlikely, but still possible)
# - need to also need to take into account the actual time of the flight (need to look into the connecting this with the flight pathing + take off and landing)
# - maybe take into account emergency landing locations? and if the vtol is unable to fly for any reason? 

# things that should be considered: 
# - what is the the time frame of flights? 24-hours? 10 hours of the day? 
# - from where to where is the flights going? this is where graph theory would come in. 
# - 
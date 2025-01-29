#pip install gtfs partridge

import partridge as ptg

# Read static GTFS data
feed = ptg.read_feed('path/to/gtfs.zip')

# Access data from the feed
routes = feed.routes
stops = feed.stops
trips = feed.trips
stop_times = feed.stop_times

# Example: Print the names of all routes
for route in routes.itertuples():
    print(route.route_short_name)

#fairly certain thats the correct pip install idk i havent tested it out yet

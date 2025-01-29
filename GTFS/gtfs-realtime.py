#pip install gtfs-realtime-bindings

from google.transit import gtfs_realtime_pb2
import requests

# Fetch GTFS Realtime data from a URL
response = requests.get('https://api.example.com/gtfs-realtime')

# Parse the GTFS Realtime feed
feed = gtfs_realtime_pb2.FeedMessage()
feed.ParseFromString(response.content)

# Access updates from the feed
for entity in feed.entity:
    if entity.HasField('trip_update'):
        print(entity.trip_update)


#fairly certain thats the correct pip install idk i havent tested it out yet

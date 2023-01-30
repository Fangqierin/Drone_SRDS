'''
File used for testing purposes
'''


from mongo_dashboard_FQ import client

waypoints = list(client["waypoints"].currentWaypoints.find({"Round": 0}))
print(waypoints)

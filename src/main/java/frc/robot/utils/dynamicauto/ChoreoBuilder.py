import json

# Assuming locations is a list of dictionaries where each dictionary represents a location
# L,M, and R were change to S1, S2, and S4 with an S3 being added.
locations = [
    {"name": "Amp Side", "x": 0.71, "y": 6.72, "angle": 1.04, "allowed_destinations": ["W1", "W2", "W3", "C1", "C2", "C3", "C4", "C5"]},
    {"name": "Center", "x": 1.35, "y": 5.56, "angle": 0, "allowed_destinations": ["W1", "W2", "W3", "C1", "C2", "C3", "C4", "C5"]},
    {"name": "Stage Side", "x": 0.71, "y": 4.36, "angle": -1.04, "allowed_destinations": ["W1", "W2", "W3", "C1", "C2", "C3", "C4", "C5"]},
    {"name": "W1", "x": 2.3, "y": 6.757, "angle": 0.39, "allowed_destinations": [ "W2", "W3", "S1", "S2", "S3", "S4", "C1", "C2", "C3", "C4", "C5"]},
    {"name": "W2", "x": 2.25, "y": 5.56, "angle": 0, "allowed_destinations": ["W1", "W3", "S1", "S2", "S3", "S4", "C1", "C2", "C3", "C4", "C5"]},
    {"name": "W3", "x": 2.3, "y": 4.36, "angle": -0.39, "allowed_destinations": ["W1", "W2", "S1", "S2", "S3", "S4", "C1", "C2", "C3", "C4", "C5"]},
    {"name": "S1", "x": 5.176, "y": 6.63, "angle": 0.26, "allowed_destinations": ["W1", "W2", "W3", "C1", "C2", "C3", "C4", "C5"]},
    {"name": "S2", "x": 4.263, "y": 5.56, "angle": 0, "allowed_destinations": ["W1", "W2", "W3", "C1", "C2", "C3", "C4", "C5"]},
    {"name": "S3", "x": 4.263, "y": 3, "angle": -0.544, "allowed_destinations": ["W1", "W2", "W3", "C1", "C2", "C3", "C4", "C5"]},
    {"name": "S4", "x": 5.176, "y": 1.62, "angle": -2.89, "allowed_destinations": ["W1", "W2", "W3", "C1", "C2", "C3", "C4", "C5"]},

    {"name": "C1", "x": 7.68, "y": 7.467, "angle": 0, "allowed_destinations": ["W1", "W2", "W3", "S1", "S2", "S3", "S4", "C2", "C3", "C4", "C5"]},
    {"name": "C2", "x": 7.68, "y": 5.797, "angle": 0, "allowed_destinations": ["W1", "W2", "W3", "S1", "S2", "S3", "S4", "C1",  "C3", "C4", "C5"]},
    {"name": "C3", "x": 7.68, "y": 4.127, "angle": 0, "allowed_destinations": ["W1", "W2", "W3", "S1", "S2", "S3", "S4", "C1", "C2",  "C4", "C5"]},
    {"name": "C4", "x": 7.68, "y": 2.457, "angle": 0, "allowed_destinations": ["W1", "W2", "W3", "S3", "S4", "S3", "S4", "C1", "C2", "C3",  "C5"]},
    {"name": "C5", "x": 7.68, "y": 0.787, "angle": 0, "allowed_destinations": ["W1", "W2", "W3", "S3", "S4", "S3", "S4", "C1", "C2", "C3", "C4"]},
]

obastacles = [
    {"x": 3.42, "y": 4.04, "radius": 0.50},
    {"x": 5.61, "y": 2.81, "radius": 0.50},
    {"x": 5.61, "y": 5.44, "radius": 0.50}
]

def create_waypoint(point):
    return {
        "x": point["x"],
        "y": point["y"],
        "heading": point["angle"],
        "isInitialGuess": False,
        "translationConstrained": True,
        "headingConstrained": True,
        "controlIntervalCount": 40
    }

def create_path(allPoints):
    return {
        "waypoints": [create_waypoint(point) for point in allPoints],
        "trajectory": [],
        "constraints": [
            {
                    "scope": [
                        "first"
                    ],
                    "type": "StopPoint"
                },
                {
                    "scope": [
                        "last"
                    ],
                    "type": "StopPoint"
                }
        ],
        "usesControlIntervalGuessing": True,
        "defaultControlIntervalCount": 40,
        "usesDefaultFieldObstacles": True,
        "circleObstacles": obastacles
    }


paths = {}
def find_path(start, end_name, newPath, path_name):
    end = next((loc for loc in locations if loc["name"] == end_name), None)
    if end:
        newPath.append(end)
        path_name += f" To {end['name']}"
        paths[path_name] = create_path(newPath)
        return
    #else:
    #    midPoint = next((mid for mid in midPoints if mid["name"] == end_name), None)
    #    if midPoint:
    #        newPath.append(midPoint)
    #        path_name += f" To {midPoint['name']}"
   #         for end_name in midPoint["allowed_destinations"]:
   #             find_path(midPoint, end_name, newPath.copy(), path_name)
   #     else:
   #         break
#
for start in locations:
    newPath = [start]
    for end_name in start["allowed_destinations"]:
        path_name = start['name']
        find_path(start, end_name, newPath.copy(), path_name)

# Wrap the paths in another dictionary with the specified format
data = {
    "version": "v0.2.1",
    "robotConfiguration": {
        "mass": 74.08797700309194,
        "rotationalInertia": 6,
        "motorMaxTorque": 1.162295081967213,
        "motorMaxVelocity": 4800,
        "gearing": 6.75,
        "wheelbase": 0.5778496879611685,
        "trackWidth": 0.5778496879611685,
        "bumperLength": 0.8762995267982555,
        "bumperWidth": 0.8762995267982555,
        "wheelRadius": 0.050799972568014815
    },
    "paths": paths,
    "splitTrajectoriesAtStopPoints": False
}

#Prints names of all paths
#for i in paths:


# Write the data to a JSON file
# C:\\Users\\Robotics\\Desktop\\Crescendo\\src\\main\\java\\frc\\robot\\utils\\dynamicauto\\data.chor
with open('C:\\Users\\stoopi_poopy\\Crescendo\\src\\main\\java\\frc\\robot\\utils\\dynamicauto\\data.chor', 'w') as f:
    json.dump(data, f, indent=4)


import json

# open file in write mode
with open('src/main/java/frc/robot/utils/dynamicauto/data.chor', 'r+') as file:
    data = json.load(file)

    """
    Things to Change for each path
    1. Clear Trajectory array
    2. Reverse headings for each waypoint
    """

    for path in data["paths"]:
        path = data["paths"][path]

        waypoints = path["waypoints"]
        path["trajectory"] = []

        for waypoint in waypoints:
            if waypoint["heading"] > 0:
                waypoint["heading"] = round(waypoint["heading"] - 3.14, 3)
            else:
                waypoint["heading"] = round(waypoint["heading"] + 3.14, 3)

    with open('src/main/java/frc/robot/utils/dynamicauto/data.chor', 'w') as file:
        json.dump(data, file, indent=4)
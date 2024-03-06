import json
import os

waypointToMove = "S2"
newPose = [1,1,0] # [X,Y,Theta]
paths = {}

with open('C:\\Users\\Robotics\\Desktop\\Crescendo\\src\\main\\java\\frc\\robot\\utils\\dynamicauto\\data.chor', 'r+') as file:
    data = json.load(file)

    paths = data["paths"]


    for k in paths:
        if(waypointToMove in k):
            if(k.split()[0] == waypointToMove):
                paths[k]["waypoints"][0]["x"] = newPose[0]
                paths[k]["waypoints"][0]["y"] = newPose[1]
                paths[k]["waypoints"][0]["heading"] = newPose[2]
            elif(k.split()[-1] == waypointToMove):
                paths[k]["waypoints"][-1]["x"] = newPose[0]
                paths[k]["waypoints"][-1]["y"] = newPose[1]
                paths[k]["waypoints"][-1]["heading"] = newPose[2]
    with open('C:\\Users\\Robotics\\Desktop\\Crescendo\\src\\main\\java\\frc\\robot\\utils\\dynamicauto\\data.chor', 'w') as file:
        json.dump(data, file, indent=4)


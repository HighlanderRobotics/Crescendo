import json
import shutil

shutil.rmtree("src/main/java/frc/robot/utils/dynamicauto/deploy")

# open file in write mode
with open('src/main/java/frc/robot/utils/dynamicauto/data.chor', 'r+') as file:
    data = json.load(file)

    for path in data["paths"]:
        path = data["paths"][path]

        path["trajectory"] = []
    
    with open('src/main/java/frc/robot/utils/dynamicauto/data.chor', 'w') as file:
        json.dump(data, file, indent=4)



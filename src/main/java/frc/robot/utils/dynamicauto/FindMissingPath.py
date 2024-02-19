import json
import os

# we know that the file missing must be missing from the deploy/choreo folder

# open file in read mode

paths = {}

with open('src/main/java/frc/robot/utils/dynamicauto/data.chor', 'r') as file:
    data = json.load(file)

    paths = data["paths"]

print("Number of paths in data.chor:")
print(len(paths))

folder_path = "src/main/java/frc/robot/utils/dynamicauto/deploy/choreo"
files_in_folder = os.listdir(folder_path)

print("Files in deploy/choreo folder:")
print(len(files_in_folder) / 2)

originalFiles = []

for file in files_in_folder:
    if not file.endswith(".1.traj"):
        originalFiles.append(file[:-5])

print("Files that end with '.1.traj':")
print(len(originalFiles))

print("Finding missing files...")

pathNames = [value for value in paths.keys()]

missingFiles = []

print(originalFiles)

print(pathNames)

print("Missing files:")

print(list(set(originalFiles) ^ set(pathNames)))
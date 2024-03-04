import json
import os
import urllib.parse

folder_path = "C:\\Users\\stoopi_poopy\\Crescendo\\src\\main\\java\\frc\\robot\\utils\\dynamicauto\\deploy\choreo"
files_in_folder = os.listdir(folder_path)

print("Files in deploy/choreo folder:")
print(len(files_in_folder) / 2)

repeatFiles = []

for file in files_in_folder:
    if file.endswith(".1.traj"):
        repeatFiles.append(file)

print(f"Removing {len(repeatFiles)} files...")

for file in repeatFiles:
    os.remove(os.path.join(folder_path, file))

print("Files removed.")
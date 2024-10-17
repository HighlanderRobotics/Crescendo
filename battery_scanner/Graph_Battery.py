import matplotlib.pyplot as plt
import os
import csv

PATH = "logs"

battery_voltages = {}
counter = 0
for file in os.listdir(PATH):
    f = os.path.join(PATH, file)
    # change to include battery name later on 
    # so {name:{file/date:{timestamp:value}}}
    # alternative option: {name:{date:{time:{timestamp:value}}}}
    
    date = f[9:17]
    time = ':'.join(f[19:26].split('-'))
    key = date + " / " + time
    if(not date in battery_voltages.keys()):
        battery_voltages[date] = {}
  #  print(date)
 #   print(time)
    battery_voltages[date][time] = {}
 #   print(battery_voltages)
    if(os.path.isfile(f)):
        with open(f) as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
             #   print(row)
                if(counter == 0):
                    counter += 1
                elif(row[1] == "/SystemStats/BatteryVoltage"):
                    battery_voltages[date][time][float(row[0])] = float(row[2])
print(battery_voltages['24-09-22'].keys())
#tmp = battery_voltages['24-09-22']['3:50:18'].values()
#print(sum(tmp)/len(tmp))
#x_axis = list(battery_voltages['24-09-22']['3:50:18'].values())
for date in battery_voltages:
    plt.figure()
    for time in battery_voltages[date]:
        plt.plot(list(battery_voltages[date][time].values()), label = time)
plt.ylabel("Voltage")
plt.xlabel("'timestamp'")
plt.legend()
plt.show()
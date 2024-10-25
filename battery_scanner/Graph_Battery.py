import matplotlib.pyplot as plt
import plotly.graph_objects as go
import os
import csv
import random


RANDOM_NAMES = ["Ed", "Athena", "Lewy", "Cassie", "Dumb Idiot"]
PATH = "logs"

battery_voltages: dict[str, dict[str, dict[str, dict[str, dict[float, float]]]]] = {}

index = 0 
for file in os.listdir(PATH):
    f = os.path.join(PATH, file)
    # change to include battery name later on 
    # so {name:{file/date:{timestamp:value}}}
    # alternative option: {name:{date:{time:{timestamp:value}}}}
    counter: int = 0
    date: str = f[9:17]
    time: str = ':'.join(f[19:26].split('-'))
    key = date + " / " + time
    name = RANDOM_NAMES[index]
    index += 1
    if(index == len(RANDOM_NAMES)):
        index = 0
    print(index)
    #RANDOM_NAMES.remove(name)
    print(date + " " + name)
    if(not name in battery_voltages.keys()):
        
        battery_voltages[name] = {}
    print(battery_voltages[name].keys())
    print("d")
    if(not date in battery_voltages[name].keys()):
        print("leog")
        battery_voltages[name][date] = {}
    print(battery_voltages[name].keys())
  #  print(date)
 #   print(time)
    battery_voltages[name][date][time] = {}
 #   print(battery_voltages)
    if(os.path.isfile(f)):
        with open(f) as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
             #   print(row)
                if(counter == 0):
                    counter += 1
                elif(row[1] == "/SystemStats/BatteryVoltage"):
                    battery_voltages[name][date][time][float(row[0])] = float(row[2])
#print(battery_voltages['24-09-22'].keys())
#tmp = battery_voltages['24-09-22']['3:50:18'].values()
#print(sum(tmp)/len(tmp))
#x_axis = list(battery_voltages['24-09-22']['3:50:18'].values())
if  __name__ == '__main__':
    for name in battery_voltages:
        for date in battery_voltages[name]:
            fig = go.Figure()
            print(battery_voltages[name].keys())
            for time in battery_voltages[name][date]:
                fig.add_trace(go.Scatter(x=list(battery_voltages[name][date][time].keys()), y=list(battery_voltages[name][date][time].values()), name=time))
                print(time + " " + date + " " + name)
            fig.update_layout(title=date + " / " + name)
            fig.show()
        
def get_voltages():
    return battery_voltages

def get_graph(name: str, date, time):
    fig = go.Figure(go.Scatter(x=list(battery_voltages[name][date][time].keys()), y=list(battery_voltages[name][date][time].values()), name=time))
    fig.write_html("battery_scanner/templates/Graph.html")
    
def get_battery_rankings():
    ranking = {}
    for name in battery_voltages:
        last_date = list(battery_voltages[name].keys())[-1]
        last_time = list(battery_voltages[name][last_date].keys())[-1]
        ranking[name] = list(battery_voltages[name][last_date][last_time].values())[0]
    print(ranking)
    ranking = dict(sorted(ranking.items(), key=lambda x: x[1]))
    print(ranking)
    return ranking

        
'''


bbafor date in battery_voltages:
    plt.figure()
    print(battery_voltages.keys())
    for time in battery_voltages[date]:
        plt.plot(list(battery_voltages[date][time].values()), label = time)
plt.ylabel("Voltage")
plt.xlabel("'timestamp'")
plt.legend()
plt.show()
'''
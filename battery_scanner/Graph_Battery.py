import matplotlib.pyplot as plt
import plotly.graph_objects as go
import os
import csv
import random
from Battery import Battery


RANDOM_NAMES = ["Ed", "Athena", "Jacob"]
PATH = "logs"

battery_voltages: dict[str, dict[str, dict[str, dict[str, dict[float, float]]]]] = {}
batteries: list[Battery] = []
matches: list[str] = []
index = 0 
for file in os.listdir(PATH):
    f = os.path.join(PATH, file)
    match: str = f[27:len(f) - 4]
    name = RANDOM_NAMES[index]
    voltages: dict[float,float] = {}
    enables: dict[float, bool] = {}
    counter = 0
    index +=1 
    if (index == len(RANDOM_NAMES)):
        index = 0
    
    if(os.path.isfile(f)):
        with open(f) as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
                if(counter == 0):
                    counter += 1
                elif(row[1] == "/SystemStats/BatteryVoltage"):
                    voltages[float(row[0])] = float(row[2])
                elif(row[1] == "/DriverStation/Enabled"):
                 #   print(str(row[2]) + " / " + str(match))
                    enables[float(row[0])] = (row[2] == "true")
                   # print(float(row[0]))
           # print(enables)
            found_battery = False
            matches.append(match)
            for battery in batteries:
                if(battery.get_name() == name):
                    found_battery = True
                    battery.add_match(match)
                    battery.add_voltage(voltages)
                    battery.add_enables(enables)
                    battery.shorten_voltages()
            if(not found_battery):
                battery = Battery(name)
                battery.add_match(match)
                battery.add_voltage(voltages)
                battery.add_enables(enables)
                battery.shorten_voltages()
                batteries.append(battery)

for battery in batteries:
   # battery.process_voltages()
   pass

def get_batteries():
    return batteries;

def get_matches():
    return matches

def get_match_graph(battery: Battery, match: str):
    fig = go.Figure()
    enables = battery.enabled
    voltages = battery.voltages
    matches = battery.matches
    fig.add_trace(go.Scatter(
        x= list(voltages[matches.index(match)].keys()),
        y = list(voltages[matches.index(match)].values())
    ))
    #print(enables[matches.index(match)])
    #print(list(enables[matches.index(match)].keys())[list(enables[matches.index(match)].values()).index(True)])
    fig.add_vline(list(enables[matches.index(match)].keys())[list(enables[matches.index(match)].values()).index(True)], line_dash="dash", annotation_text = "Enabled")
    fig.add_vline(list(enables[matches.index(match)].keys())[[i for i, n in enumerate(list(enables[matches.index(match)].values())) if n == False][-1]], line_dash="dash", annotation_text = "Disabled")
    return fig.to_html(full_html = False)

# do not use
def get_candlestick_chart_all():
    x_axis = []
    high = []
    low = []

    data = []
    
    
    
    for name in battery_voltages:
        x_axis = []
        high = []
        low = []
        for date in battery_voltages[name]:
            times = list(battery_voltages[name][date].keys())
            for time in times:
                x_axis.append(date + " | " + time)
                high.append(list(battery_voltages[name][date][time].values())[0])
                low.append(list(battery_voltages[name][date][time].values())[-1])

        data.append(go.Candlestick(
            x = x_axis,
            open = high,
            high = high,
            low = low,
            close = low, 
            name = name
        ))
    print("IMPORtANT IMPORTANT IMPORTANT" + str(data[0].x))
    print(high)
    x_axis = []
    high = []
    low = []
    # this is a later petro problem
    '''
    for graph in data:
        x_axis.append(graph.x)
        high.append(graph.high)
        low.append(graph.low)
        for i in range(len(x_axis)):
            for j in range(0, len(x_axis) - i - 1):
                
                # Range of the array is from 0 to n-i-1
                # Swap the elements if the element found 
                #is greater than the adjacent element
                
                if x_axis[j] > x_axis[j + 1]:
                    x_axis[j], x_axis[j + 1] = x_axis[j + 1], x_axis[j]
    '''
    fig = go.Figure(data)
    fig.update_layout(title = name, 
                      yaxis_title = "voltatge"
                      )
    fig.show()

def get_candlestick_chart(battery: Battery):
    x_axis = []
    high = []
    low = []
    matches = battery.matches
    voltages = battery.unfiltered_voltages
    fig = go.Figure()
    for i in range(len(matches)):
        x_axis.append(matches[i])
        high.append(list(voltages[i].values())[0])
        low.append(list(voltages[i].values())[-1])
    
    fig.add_trace(go.Candlestick(
        x = x_axis,
        open = high,
        high = high,
        low = low,
        close = low
    )) 
    print(high)  
    fig.update_layout(
        title = battery.get_name(), 
        yaxis_title = "Voltage",
        xaxis_title = "Match"
    )
    return fig.to_html(full_html = False)

#get_candlestick_chart_all()
def get_battery_rankings():
    ranking = {}
    for battery in batteries:
        ranking[battery.get_name()] = battery.get_health()
    ranking = dict(sorted(ranking.items(), key=lambda x: x[1]))
    return ranking

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
for file in os.listdir(PATH):
    f = os.path.join(PATH, file)
    match: str = f[26:len(f) - 4]
    name = RANDOM_NAMES[index]
    voltages: dict[float,float] = {}
    counter = 0
    index += 1
    
    if(index == len(RANDOM_NAMES)):
        index = 0
    print(index)
    if(os.path.isfile(f)):
        with open(f) as csv_file:
            reader = csv.reader(csv_file)
            for row in reader:
             #   print(row)
                if(counter == 0):
                    counter += 1
                elif(row[1] == "/SystemStats/BatteryVoltage"):
                    voltages[float(row[0])] = float(row[2])
            found_battery = False
            for battery in batteries:
                if(battery.get_name() == name):
                    found_battery = True
                    battery.apply_match(match)
                    battery.apply_voltage(voltages)
            if(not found_battery):
                battery = Battery(name)
                battery.apply_match(match)
                battery.apply_voltage(voltages)
                batteries.append(battery)
for battery in batteries:
    print(battery.get_name() + " " + str(battery.get_health()))
if not __name__ == '__main__':
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

def get_graph_html(name: str, date: str, time: str):
    fig = go.Figure(go.Scatter(x=list(battery_voltages[name][date][time].keys()), y=list(battery_voltages[name][date][time].values()), name=time))
    return fig.to_html(full_html=False)

def get_graph(name: str, date, time):
    fig = go.Figure(go.Scatter(x=list(battery_voltages[name][date][time].keys()), y=list(battery_voltages[name][date][time].values()), name=time))
    fig.show()
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
def get_candlestick_chart(name: str):
    x_axis = []
    high = []
    low = []
    start = []
    end = []
    fig = go.Figure()
    for date in battery_voltages[name]:
        times = list(battery_voltages[name][date].keys())
        for time in times:
            x_axis.append(date + " | " + time)
            high.append(list(battery_voltages[name][date][time].values())[0])
            low.append(list(battery_voltages[name][date][time].values())[-1])
            start = high
            end = low
    print(x_axis)
    print(high)
    fig.add_trace(go.Candlestick(
        x = x_axis,
        open = start,
        high = high,
        low = low,
        close = end
    ))
    fig.update_layout(title = "todos", 
                      yaxis_title = "voltatge"
                      )
    return fig.to_html(full_html = False)
    
#get_candlestick_chart_all()
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
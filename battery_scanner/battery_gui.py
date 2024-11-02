from flask import Flask, render_template, request
import Graph_Battery
import os

app = Flask(__name__)

voltages = Graph_Battery.get_voltages()
name = ""
date = ""
time = ""

@app.route("/", methods = ['POST', 'GET'])
def battery_list():
    return render_template("Battery_Ranking.html", names = list(Graph_Battery.get_battery_rankings().keys()), ranking = Graph_Battery.get_battery_rankings(), title = "Battery Ranking")

@app.route("/name", methods = ['POST','GET'])
def name_chooser():
    
    return render_template("name_selector.html", names = list(voltages.keys()))
    
@app.route("/name/dates", methods = ["POST", "GET"])
def date_chooser():
    global name, date, time
    for key, value in request.form.items():
        if(key == "nameSelector"):
            name = value
    print(name)
    print(date)
    print(time)
    return render_template("date_selector.html", dates = list(voltages[name].keys()), title = name)

@app.route("/name/dates/times", methods = ["POST", "GET"])
def time_chooser():
    global name, date, time
    for key, value in request.form.items():
 
        if(key == "dateSelector"):
            print(value + "DWDADWDAD")
            date = value
    print(name)
    print(date)
    print(time)
    
    return render_template("time_selector.html", times = list(voltages[name][date].keys()), title = name + "/" + date)



@app.route("/name/dates/times/graphs", methods = ['POST', 'GET'])
def show_graph():
    for key, value in request.form.items():
        if(key == "timeSelector"):
            time = value
    Graph_Battery.get_graph(name, date, time)
    return "Graph opens in new window"

if __name__ == '__main__':
   app.run()
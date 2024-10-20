from flask import Flask, render_template, request
import Graph_Battery

app = Flask(__name__)

voltages = Graph_Battery.get_voltages()
name = ""
date = ""
time = ""


@app.route("/", methods = ['POST','GET'])
def name_chooser():
    
    return render_template("name_selector.html", names = list(voltages.keys()))
    
@app.route("/dates", methods = ["POST", "GET"])
def date_chooser():
    global name, date, time
    for key, value in request.form.items():
        if(key == "nameSelector"):
            name = value
    print(name)
    print(date)
    print(time)
    return render_template("date_selector.html", dates = list(voltages[name].keys()), title = name)

@app.route("/dates/times", methods = ["POST", "GET"])
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


def clear_graph():
    open('battery_scanner/templates/Graph.html', 'w').close()
    print('EIOGHSEIOGHNSIOGHIOh')
    
@app.route("/dates/times/graphs", methods = ['POST', 'GET'])
def show_graph():
    for key, value in request.form.items():
        if(key == "timeSelector"):
            time = value
    clear_graph()
    Graph_Battery.get_graph(name, date, time)
    return render_template("Graph.html")

if __name__ == '__main__':
   app.run()
from flask import Flask, render_template, request, url_for
import Graph_Battery
import os
from jinja2 import Template

app = Flask(__name__)

@app.route("/", methods = ['POST', 'GET'])
def ranking_page():
    urls = []
    for name in list(Graph_Battery.get_battery_rankings().keys()):
        urls.append(url_for("battery_rundown", name = name))
    urls.reverse()
    return render_template("Battery_Ranking.html", links = urls, names = list(Graph_Battery.get_battery_rankings().keys()), ranking = Graph_Battery.get_battery_rankings(), title = "Battery Ranking")


@app.route("/<name>", methods = ['POST', 'GET'])
def battery_rundown(name):
    for battery in Graph_Battery.get_batteries():
        if(battery.get_name() == name):
           fig = Graph_Battery.get_candlestick_chart(battery)
           print("IMPORTANT IMPORTANT IMPORTANt")
    try:
        return render_template("Battery_Rundown.html", name = name, fig = fig)
    except:
        return render_template("Battery_Rundown.html", name = name, fig = "<h2>Sorry! No battery could be found with that name. This shouldnt happen.</h2>")

@app.route("/<name>/matches", methods = ['POST', 'GET'])
def battery_matches(name):
    urls = {}
    matches = {}
    tournaments = []
    for battery in Graph_Battery.get_batteries():
        if(battery.get_name() == name):
            tournaments = battery.get_tournaments()
            for tournament in tournaments:
                if(tournament not in list(matches.keys())):
                    matches[tournament] = []
                if(tournament not in list(urls.keys())):
                    urls[tournament] = []
                matches[tournament] = battery.matches[tournament]
    for tournament in tournaments:
        for match in matches[tournament]:
            urls[tournament].append(url_for("match", name = name, match = match, tournament = tournament))
        urls[tournament].reverse()
    print(list(matches.keys()))
    print(urls.values())
    print(matches)
    
    return render_template("Battery_Matches.html", links = urls, matches = matches, title = f"Matches for {name}", tournaments = list(matches.keys()))

@app.route("/<name>/<tournament>/<match>", methods = ['POST', 'GET'])
def match(name, tournament, match):
    for battery in Graph_Battery.get_batteries():
        if(battery.get_name() == name):
           fig = Graph_Battery.get_match_graph(battery, match, tournament)
    try:
        return render_template("Match_Display.html", name = name, match = match, fig = fig, tournament = tournament)
    except:
        return render_template("Match_Display.html", name = name, match = match, fig = "<h2>Sorry! No match could be found with that name. This shouldnt happen.</h2>")


@app.route("/matches", methods = ['POST', 'GET'])
def matches():
    urls = {}
    matches = Graph_Battery.get_matches()
    batteries = {}
    for tourney in list(matches.keys()):
        if(tourney not in list(matches.keys())):
            matches[tourney] = []
        if(tourney not in list(batteries.keys())):
            batteries[tourney] = []
        if(tourney not in list(urls.keys())):
            urls[tourney] = []
        for i in range(len(matches[tourney])):
            for battery in Graph_Battery.get_batteries():
                print(matches.keys())
                if(battery.matches[tourney].count(matches[tourney][i]) > 0):
                    batteries[tourney].append(battery.get_name())
            print(str(urls) + " LINKS LONKS INLINS LINKS LONIMNKS LINKS LINKS LINKS LINKS LINKS LINKS")
            urls[tourney].append(url_for("match", name = batteries[tourney][i], tournament = tourney, match = matches[tourney][i]))
        urls[tourney].reverse()
    print(urls)
    return render_template("Matches.html", links = urls, matches = matches, title = "All Matches", touraments = list(matches.keys()))
if __name__ == '__main__':
   app.run()
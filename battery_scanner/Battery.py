import math

class Battery:
    def __init__(self, name: str):
        self.name = name
        self.voltages: list[dict[float,float]] = []
        self.unfiltered_voltages: list[dict[float,float]] = []
        self.enabled: list[dict[float,bool]] = []
        self.matches = []
        
        
    def get_health(self):
        return list(self.unfiltered_voltages[-1].values())[-2]
    
    def get_name(self):
        return self.name
    
    def add_match(self, match: str):
        self.matches.append(match)
    
    def add_voltage(self, voltage: dict[float,float]):
        self.voltages.append(voltage)
        self.unfiltered_voltages.append(voltage)
        
    def add_enables(self, enables: dict[float,bool]):
        self.enabled.append(enables)
    
    def shorten_voltages_front(self):
        
       for match in self.matches:
            time_enabled = list(self.enabled[self.matches.index(match)].keys())[list(self.enabled[self.matches.index(match)].values()).index(True)]   
            match_voltage =  list(self.voltages[self.matches.index(match)].keys())
            index = -1
            
            for i in range(len(match_voltage)):
                """if(match == "e5"):
                    if not match_voltage[i] > time_enabled - 10 and match_voltage[i] > 500 :
                        print(time_enabled - 10)
                        print(str(match_voltage[i]) + " / " + str(i) + " / " + str(math.isclose(match_voltage[i], time_enabled - 10, rel_tol = 0.1)))"""
                if(math.isclose(match_voltage[i], time_enabled , rel_tol = 0.1)):
                    index = i
                    break
            
            match_index = self.matches.index(match)
            enabled_index = list(self.enabled[self.matches.index(match)].values()).index(True)
            self.voltages[match_index] = dict(zip(list(self.voltages[match_index].keys())[index:], list(self.voltages[match_index].values())[index:]))
           # self.enabled[match_index] = dict(zip(list(self.enabled[match_index].keys())[enabled_index:], list(self.enabled[match_index].values())[enabled_index:]))
    def shorten_voltages_end(self):
        
       for match in self.matches:
            print(list(self.enabled[self.matches.index(match)].values()))
            try:
                
                time_disabled = list(self.enabled[self.matches.index(match)].keys())[[i for i, n in enumerate(list(self.enabled[self.matches.index(match)].values())) if n == False][-1]]   
            except:
                continue
            match_voltage =  list(self.voltages[self.matches.index(match)].keys())
            index = -1
            
            for i in range(len(match_voltage)):
                
                if(math.isclose(match_voltage[i], time_disabled, rel_tol = 0.1)):
                    index = i + int(time_disabled)
                    break
            
            match_index = self.matches.index(match)
            
            self.voltages[match_index] = dict(zip(list(self.voltages[match_index].keys())[:index], list(self.voltages[match_index].values())[:index]))
           # self.enabled[match_index] = dict(zip(list(self.enabled[match_index].keys())[:enabled_index], list(self.enabled[match_index].values())[:enabled_index]))
            
    def shorten_voltages(self):
        self.shorten_voltages_front()
       # self.shorten_voltages_end() # just dont use for now, needs fixing

        
    def __str__(self):
        return f"{self.name} \n {self.matches} \n {self.voltages}"
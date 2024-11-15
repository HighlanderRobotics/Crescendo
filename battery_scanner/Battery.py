import math

class Battery:
    def __init__(self, name: str):
        self.name = name
        self.voltages: list[dict[float,float]] = []
        self.enabled: list[dict[float,bool]] = []
        self.matches = []
        
    def get_health(self):
        return list(self.voltages[-1].values())[-2]
    
    def get_name(self):
        return self.name
    
    def add_match(self, match: str):
        self.matches.append(match)
    
    def add_voltage(self, voltage: dict[float,float]):
        self.voltages.append(voltage)
        
    def add_enables(self, enables: dict[float,bool]):
        self.enabled.append(enables)
    
    def process_voltages(self):
        new_voltages: list[dict[float,float]] = []
        new_enables: list[dict[float,float]] = []
        for i in range(len(self.voltages)):
            if(len(self.enabled[0]) == {}):
                self.enabled = [self.enabled[1]]
            Voltages = list(self.voltages[i].values())
            VTimestamps = list(self.voltages[i].keys())
            Enabled = list(self.enabled[i].values())
            ETimestamps = list(self.enabled[i].keys())
            print("ENABLED: " + str(self.enabled) + " MATCH: " + self.matches[i])
            
            enable_start_time = ETimestamps[Enabled.index(True)]
           # print(enable_start_time)
            enable_start_index = VTimestamps.index(enable_start_time)
            
            new_voltages.append(dict(zip(VTimestamps[enable_start_index:], Voltages[enable_start_index:])))
            new_enables.append(dict(zip(ETimestamps[enable_start_index:], Enabled[enable_start_index:])))
        self.voltages = new_voltages
        self.enabled = new_enables            
        
    def __str__(self):
        return f"{self.name} \n {self.matches} \n {self.voltages}"
import math

class Battery:
    def __init__(self, name: str):
        self.name = name
        self.voltages: list[dict[float,float]] = []
        self.matches = []
        
    def get_health(self):
        return list(self.voltages[-1].values())[-2]
    
    def get_name(self):
        return self.name
    
    def add_match(self, match: str):
        self.matches.append(match)
    
    def add_voltage(self, voltage: dict[float,float]):
        self.voltages.append(voltage)
        
    def process_voltages(self):
        new_voltages: list[dict[float,float]] = []
        
        for Voltages in self.voltages:
            voltage_list = list(Voltages.values())
            timestamps = list(Voltages.keys())
            match = {}
            start_logging = False
            
            for i in range(len(voltage_list)):
                if(not start_logging):
                    if(i % 250 == 0):
                        print(len(voltage_list))
                        print("i:" + str(i) + " / " + str(math.isclose(sum(voltage_list[i:i+20])/10, voltage_list[0])))
                    if(not math.isclose(voltage_list[i - 10], voltage_list[i], abs_tol = 0.6)):
                        start_logging = True if not 0 <= i <= 5 else False;
                        print(f"IMPORTANT IOMPORITN {i}")
                elif(start_logging):
                    if(math.isclose(sum(voltage_list[i:i+10])/10, voltage_list[-1])):
                        start_logging = False
                    else:
                        match[timestamps[i]] = voltage_list[i]
            new_voltages.append(match)
        self.voltages.clear()
        for voltage in new_voltages:
            self.voltages.append(voltage)
        
    def __str__(self):
        return f"{self.name} \n {self.matches} \n {self.voltages}"
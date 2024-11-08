class Battery:
    def __init__(self, name: str):
        self.name = name
        self.voltages: list[dict[float,float]] = []
        self.matches = []
        
    def get_health(self):
        return list(self.voltages[-1].values())[-2]
    
    def get_name(self):
        return self.name
    
    def apply_match(self, match: str):
        self.matches.append(match)
    
    def apply_voltage(self, voltage: dict[float,float]):
        self.voltages.append(voltage)
        
    def __str__(self):
        return f"{self.name} \n {self.matches} \n {self.voltages}"
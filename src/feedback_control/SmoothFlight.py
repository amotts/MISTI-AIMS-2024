import numpy as np

class SmoothFlight:
    def __init__(self, max_allowable_change = 0.4, max_n=9, min_n = 4):
        self.last_n = []
        self.running_avg = 0
        self.max_delta = max_allowable_change
        self.max_n = max_n
        self.min_n = min_n

    def run(self, value: float) -> float:
        if len(self.last_n) < self.min_n:
            self.last_n.append(value)
            self.running_avg=np.average(self.last_n)
            return(value)
        elif abs(value-self.running_avg) > self.max_delta:
            self.last_n = []
            return(value)
        else:
            return_value = self.running_avg
            if len(self.last_n) > self.max_n:
                self.last_n.pop()
            self.last_n.append(value)
            self.running_avg=np.average(self.last_n)
            return(return_value)

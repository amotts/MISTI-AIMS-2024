import numpy as np

class SmoothFlight:
    """
    Class object for flight smoothing via a sliding kernel.  
    The history of a past N values is stored and until the min N value is reached, the values are passed and stored as hx.
    The new value is compared against the stored kernel average. If it is outside the max allowable change, the history is reset and the value passed.  
    If the value is close enough to the average, the average is passed, the value is added to the hx, and a new kernel average is calculated.  
    The hx is only stored up to the maximum size of the master kernel

    Inputs
    --------------
    max_allowable_change - the change in depth from average that resets the memory (Default is 0.6 m)

    min_n - the minimum history to be stored before the kernel is used (Default is 3)

    kernel - a list or array with kernel values starting with most recent. Must be longer than min_n

    """
    def __init__(self, max_allowable_change = 0.6, min_n = 3, kernel = [1, 0.8,0.64,0.512,0.4096,0.32768,0.262144,0.2097152,0.16777216]):
        self.last_n = []
        self.kernel_avg = 0
        self.max_delta = max_allowable_change
        self.max_n = len(kernel)
        self.master_kernel = kernel
        assert (len(kernel) >= min_n)
        self.min_n = min_n

    # Function to perform smoothing
    def run(self, value: float) -> float:
        # Check if enough hx has been stored to use kernel, else add to hx and return value
        if len(self.last_n) < self.min_n:
            self.last_n.append(value)
            self.get_kernel_avg()
            return(value)
        # Else check if value is close enough to kernel average otherwise reset hx and pass value directly
        elif abs(value-self.kernel_avg) > self.max_delta:
            self.last_n = []
            return(value)
        # Else return the previous kernel average as the value, add to the hx and trim if necessary, calculate new kernel average
        else:
            return_value = self.kernel_avg
            if len(self.last_n) == self.max_n:
                self.last_n.pop(0)
            self.last_n.append(value)
            self.get_kernel_avg()
            return(return_value)

    # Function to calculate a new kernel average with a truncated version of the master kernel corresponding to the length of hx
    def get_kernel_avg(self):
        n = len(self.last_n)
        kernel = self.master_kernel[:n]
        self.kernel_avg = np.sum(np.multiply(self.last_n, kernel))/np.sum(kernel)
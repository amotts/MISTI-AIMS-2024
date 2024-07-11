import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

from DepthAnalysisFunctions import*
from KalmanFunctions import *
from FlightModelingFunctions import *
from ForwardLookFunctions import *

class App:
    """
    An application class for viewing the forward look sonar data from the perspective of the vehicle as it travels
    Displays for every iternum timesteps (default is 3Hz).
    Show the ground, the FL ground points in front of the vehicle.
    """
    def __init__(self, root, lognum):
        self.root = root
        self.root.title("Iterate and Pause with Graph")
        self.iternum = 10

        # Create and prepare photolog data
        self.pl = PhotoLogData()
        self.pl.initializeData('Deep_Photo_Logs', lognum)
        self.pl.applygaussianFilter(discard_extreme=True)
        self.ground_smooth = xy_signal(self.pl.df['cumulative_distance'].values, self.pl.df['gauss filtered data'])
        self.ground_raw = xy_signal(self.pl.df['cumulative_distance'].values, self.pl.df['total depth'].values)
        self.flight = xy_signal(*RealTimeFlightPath(self.ground_raw, 0.5, 3.5))

        # Create list to store FL data
        self.FL_x = []
        self.FL_y = []


        # Initialize the iteration counter
        self.counter = 0
        
        # Create a label to display iteration info
        self.label = tk.Label(root, text="", font=("Helvetica", 16))
        self.label.pack(pady=20)
        
        # Create a button to continue
        self.button = tk.Button(root, text="Continue", command=self.continue_iteration)
        self.button.pack(pady=20)
        
        # Create a figure and axis for the plot
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title('Altitude Data')
        self.ax.set_xlabel('Iteration')
        self.ax.set_ylabel('Altitude')
        
        # Create a canvas to embed the plot
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack()
        
        # Start the iteration
        self.iterate()
        
    def iterate(self):
        x,y = ForwardLookPing(self.ground_raw, (self.flight.x[self.counter], self.flight.y[self.counter]))
        self.FL_x.append(x)
        self.FL_y.append(y)


        # Display data and update graph every iternum iterations
        if self.counter < len(self.ground_raw.y):
            if self.counter % self.iternum == 0 and self.counter != 0:
                self.label.config(text=f"Iteration: {self.counter}")
                self.update_graph()
                self.button.config(state="normal")
            else:
                self.button.config(state="disabled")
                self.root.after(10, self.iterate)
                self.counter += 1
        else:
            self.label.config(text="Iteration Complete")
            self.button.config(state="disabled")
    
    def continue_iteration(self):
        self.button.config(state="disabled")
        self.root.after(100, self.iterate)
        self.counter += 1
    
    def update_graph(self):
        current_x = self.flight.x[self.counter]
        current_y = self.flight.y[self.counter]
        filtered_indices = [i for i in range(len(self.FL_x)) if self.FL_x[i] > current_x]
        updated_x = [self.FL_x[i] for i in filtered_indices]
        updated_y = [self.FL_y[i] for i in filtered_indices]

        self.ax.clear()
        self.ax.plot(updated_x - current_x, updated_y, 'r.', label="Forward Look View")
        self.ax.plot(self.ground_smooth.x-current_x, self.ground_smooth.y, label="Ground")
        self.ax.plot(0,current_y, marker='s', markersize=5, label="Reefscan Deep")
        self.ax.set_xlim(-3, 8)
        self.ax.set_ylim(current_y-8, current_y+3)
        self.ax.set_title('Altitude Data')
        self.ax.set_xlabel('Relative Distance')
        self.ax.set_ylabel('Altitude')
        self.ax.legend()
        self.canvas.draw()

# Create the main application window
root = tk.Tk()
app = App(root, 57)

# Run the application
root.mainloop()
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
    FOR USE WITH ARTIFICIAL GROUND NOT PHOTO LOG DATA
    An application class for viewing the forward look sonar data from the perspective of the vehicle as it travels
    Displays for every iternum timesteps (default is 3Hz).
    Show the ground, the FL ground points in front of the vehicle.
    """
    def __init__(self, root, lognum):
        self.root = root
        self.root.title("Iterate and Pause with Graph")
        self.iternum = 5

        self.alt = 2.5
        self.speed = 1

        self.numlog = lognum
        self.folder_name = r'C:\Users\amotz\Documents\Artificial_Ground'


        #Find the file with the correct log number and load it
        for filename in os.listdir(self.folder_name):
            good_str = "_" + str(self.numlog)+"."
            if good_str in filename:
                self.filename = filename
        try:
            file_path = os.path.join(self.folder_name, self.filename)
            self.df = pd.read_csv(file_path)
        except TypeError:
            print(f'Log file {lognum} does not exist in Artificial Ground folder. Cannot initialize. Try again with new file')



        # Create and prepare photolog data
        self.ground = xy_signal(self.df['cumulative_distance'].values, self.df['total depth'].values)
        self.flight = xy_signal(*RealTimeFlightPath(self.ground, self.speed, self.alt))

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
        x,y = ForwardLookPing(self.ground, (self.flight.x[self.counter], self.flight.y[self.counter]))
        self.FL_x.append(x)
        self.FL_y.append(y)


        # Display data and update graph every iternum iterations
        if self.counter < len(self.flight.y)-1:
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
        self.ax.plot(self.ground.x-current_x, self.ground.y, label="Ground")
        self.ax.plot(0,current_y, marker='s', markersize=5, label="Reefscan Deep")
        self.ax.plot(self.flight.x-current_x, self.flight.y)
        self.ax.set_xlim(-18, 15)
        self.ax.set_ylim(current_y-8, current_y+3)
        self.ax.set_title('Altitude Data')
        self.ax.set_xlabel('Relative Distance')
        self.ax.set_ylabel('Altitude')
        self.ax.legend()
        self.canvas.draw()

# Create the main application window
root = tk.Tk()
app = App(root, 2)

# Run the application
root.mainloop()
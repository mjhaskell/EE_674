#!/usr/bin/env python
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.getcwd()), 'src'))

import time
from builtins import input
from IPython.core.debugger import set_trace

import numpy as np

from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

plotter = Plotter(plotting_frequency=1)
''' Demonstrates:
 - Light theme
 - Sigma bounds
 - Scatter plots
 - Hidden plots
 '''
# Use light theme for white background (better for papers/presentations)
plotter.use_light_theme()

### Define plot names

## Simple string definitions
x = PlotboxArgs(
    title='X with sigma bounds',
    plots=[PlotArgs(name='True X', states='x_t'),
           PlotArgs(name='Estimated X', states='x_e', sigma_bounds=[1,2])]
)
y = PlotboxArgs(
    title='Y data',
    plots=[PlotArgs(name='Y position', states='y_t'),
           PlotArgs(name='Estimated Y', states='y_e')]
)
z = PlotboxArgs(
    title='Z with hidden plot',
    plots=[PlotArgs(name='Z', states='z_t'),
           PlotArgs(name='Estimated Z (hidden)', states='z_e', hidden=True)]
)
first_row = [x, y, z]

xy_plot = PlotboxArgs(
    title="XY with Scatter",
    plots=[PlotArgs(name='XY_true', states=['x_t', 'y_t']),
           PlotArgs(name='XY scatter', states=['x_scat', 'y_scat'], connect=False, symbol='o', symbol_size=0.1, px_mode=False)]
)
# Use a list of lists to specify plotting window structure (3 rows, each with 3 plots)
plots = [first_row,
         [xy_plot]]

# Add plots to the window
plotter.add_plotboxes(plots)

# Define and label vectors for more convenient/natural data input
plotter.define_input_vector('true_position', ['x_t', 'y_t', 'z_t'])
plotter.define_input_vector('est_position', ['x_e', 'y_e', 'z_e'])


# setup simulation timing
T = 5
Ts = 0.01
tvec = np.linspace(0, T, num=int((1/Ts)*T))

# run the simulation
for idx, t in enumerate(tvec):
    # Make some sinusoids and stuff
    x_e = np.sin(2*np.pi*1*t)
    y_e = np.cos(2*np.pi*0.5*t)
    z_e = t + np.cos(2*np.pi*2*t)

    # Simulate sigma values for x_e
    x_sigma = np.sin(0.2*np.pi*1*t)

    # Generate random particles for x & y
    x_scat = np.random.normal(x_e, scale=0.15, size=100)
    y_scat = np.random.normal(y_e, scale=0.15, size=100)

    x_t = 1.5*np.sin(2*np.pi*1*t)
    y_t = 1.5*np.cos(2*np.pi*0.5*t)
    z_t = t


    ## Add the state data in vectors
    plotter.add_vector_measurement('true_position', [x_t, y_t, z_t], t)
    plotter.add_vector_measurement('est_position', [x_e, y_e, z_e], t, sigma_values=[x_sigma, 0., 0.])
    plotter.set_data('x_scat', x_scat, t)
    plotter.set_data('y_scat', y_scat, t)

    # Update and display the plot
    plotter.update_plots()
    time.sleep(0.03)

# Wait so that the plot doesn't disappear
input("Press any key to end...")

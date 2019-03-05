State Plotter
=============

Uses Python and [PyQtGraph](http://www.pyqtgraph.org/) to perform real-time plotting of ROS topics.

## Usage ##

To use this package, you must add it to your working `catkin_ws`. This packages primary purpose is to expose the `state_plotter` Python package with the `Plotter` module.

In your own package, you create your own Python node that uses the `state_plotter` Python package. For example, consider the following directory tree:

```bash
catkin_ws
└── src
    ├── my_package
    │   ├── CMakeLists.txt
    │   ├── nodes
    │   │   └── state_plotter
    │   └── package.xml
    └── state_plotter
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        ├── setup.py
        ├── src
        │   └── state_plotter
        │       ├── __init__.py
        │       ├── Plotter.py
        │       ├── state_data.py
        │       ├── state_plot.py
        │       └── state_plotbox.py
        └── test
            └── test_plotting.py
```

In `my_package`, we have created a Python node called `state_plotter`. This Python node uses the `state_plotter` package to plot states:


## Example ##

For an example of how to use the `state_plotter`, see the `test/test_plotting.py` script.
There is also a ROS node example given in `nodes/mavros_plotter.py`.

## Documentation ##

See the wiki for documentation on the plotter functions and on using the argument classes to create plotbox and plot objects.


### Thanks ###

Based on the work of Jerel Nielsen and Skyler Tolman.

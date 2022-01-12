# PMD_Project_group2
Planning and Decision Making project group 2 code repository
## file overview
Testing notebook.ipynb : contains test code of bicyvlemodel.py functions and metric data evaluations 
Bezier curve testing.ipynb :  contains small experiment of bezier curve application as steering function.
bicyclemodel.py : contains class and function files relating to car kinematics and car visualisation  
main.py : not acually the main file, initial implementation of kinematic simulation of car with matplotlib visualisation  
main_pygame.py : not acually the main file, initial implementation of kinematic simulation of car with pygame  
pure_pursuit.py : copied from [https://github.com/AtsushiSakai/PythonRobotics], not actually called but used for reference  
rrt_+_simulation_results.pkl : results of set of simulations  
RRT_p_carsim.py : actual main file, RRT algotihm + path smoothing + car kinematic simulation, has toggle: Run_for_metrics = False/True  
task-  
  Bspline test.py : testing file for b-spline smoothing  
  RRT.py :imlementation of RRT algorithm + b-spline smoothing  
  RRTbasePy.py : contains RRT algorithm class and functions  
  
## installation
Install the requirements via pip
```bash
pip install -r requirements.txt
```

alternativly use the interpeter settings in pycharm to install new packages, pycharm should suggest updates autaomatically.
## creating enviroment
use
```
add interpeter > Virtualenv Enviroment > New enviroment
```
to create a local project specific enviroment for the project.
## requirements upkeep
use the 
```
Tools > Sync Python Requirements 
```
tool in pycharm to maintain the requirements document

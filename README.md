## Alex Christo presents: Flocking

### Build instructions

To build you need NGL installed as per the instructions [here](https://github.com/NCCA/NGL)

Windows

```
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=~/NGL ..
cmake --build .

```

Mac Linux


```
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=~/NGL ..
make 
```

Once NGL is installed simply build the project via CMAKE and run the "Boids" executable.

### Usage

The Program at its core is a cube shaped scene in which a number of agents "flock" mimicking the movement of groups of birds or fish, modeled after Craig Reynolds famous "Boids". There are a number of user controls to modify behaviour:

-Left click and drag: Tumble the scene.

-Right click and drag: Pan around the scene.
 
-Middle mouse: Zoom in / out.

-Maximum speed: Controls the speed at which agents move.

-Maximum steering force: Controls the speed at which agents change direction.
 
-Alignment strength: Controls the degree to which agents want to fly in the same direction as eachother.

-Seperation strength: Controls the degree to which agents want to seperate from one another.

-Cohesion strength: Controls the degree to which agents want to fly towards the flock center.

-Desired seperation: The seperation distance agents will try to achive from one another.

-Neighborhood radius: The size of the viewing radius in which agents are aware of their neigbors (a small radius will mean agents infer behaviour based on their immediate neighbors vs teh whole flock). [NOTE: a high neiborhood radius can slow the simulation with large numbers of agents]

-Flock size: The number of agents in the scene, minimum 0, maximum 3,000. 

-Teleport on edge collision: Whether agents bounce off the scene cube walls, or teleport to the other side upon collision.

-Agent size: The size of each agent.

-Obsticles: Controls whether or not a set of 4 obsticles are added to the scene for agents to avoid.


### Design
Please see "initial_class_diagram.pdf" for the initial class diagram, Many changes were made to this design throughout implimentation. For details on these changes and their reasoning, please refer to my 'CGI Techniques' final report.




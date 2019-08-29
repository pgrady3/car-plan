# car-plan

## Getting started, for dummies

On Linux with Anaconda/Miniconda installed......



```
#clone the git repo
git clone https://github.com/pia32/car-plan.git

#create a new conda environment, then activate it
conda create -n car-plan python=3
conda activate car-plan

#install the python dependencies
pip install -r requirements.txt

#now run! note it can be slow, sometimes 2-3 minutes
python main.py
```


## Theory

This planner uses a [PRM (probabilistic roadmap)](https://en.wikipedia.org/wiki/Probabilistic_roadmap) to generate a trajectory. Essentially, this works by randomly sampling intermediate points, then using a graph to search for the optimal route with no collisions.

There are superior methods, such as LazyPRM and RRT Star, which have improved performance and more optimal solutions. However I only spent a few hours coding this and am far too lazy.

This uses the [dubins car model](https://en.wikipedia.org/wiki/Dubins_path) to interpolate between the PRM points.

## Performance

The performance is bad. Like not good. These kind of algorithms must really be implemented in C, not python. I believe there are standard libraries like OMPL in ROS, which you would actually use when it's time to implement something real.

## Sample runs

Randomly generated obstacles and start/stop points

![ex1](ex1.png)
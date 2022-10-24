# Carla-Data-Collection
This repository generates a KITTI-like dataset from the POV of a traffic camera, using the CARLA simulator. It uses KITTI sensor parameters as specified [here](https://www.cvlibs.net/datasets/kitti/setup.php). 

## Setup
1. [Setup CARLA 0.9.13](https://github.com/carla-simulator/carla/releases)
2. Clone this repo into the root folder of the CARLA installation
3. Setup an environment
  i. According to [this]  (https://carla.readthedocs.io/en/latest/start_quickstart/#:~:text=CARLA%20supports%20Python%202.7%20and,version)%20version%2020.3%20or%20higher.) the version of Python 3 that is used should not matter too much.
  ii. Run `pip install -r requirements.txt` from the root of the local repo folder.


## Use
1. Run a CARLA server. Ensure TCP ports 2000 and 2001 are free!
2. Run `python main.py` in the local repo folder.

## TODOS
- Occlusion and truncation have not been implemented into KITTI labels yet (view KITTI label format [here](https://github.com/bostondiditeam/kitti/blob/master/resources/devkit_object/readme.txt)
- Implement script to properly create testing, training, and validation sets. Currently, files are saved to training and testing folders respectively by server tick odd/even parity.
- Create a more robust list of agents of varied vehicles, pedestrians, and cyclists. Currently only a single car is spawned.
- Use other CARLA maps to create more varied data.
- Add sensors to other intersections to collect more varied data.
- Add label class attributes and corresponding logic to keep track of car velocity.

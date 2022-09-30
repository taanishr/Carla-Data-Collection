# Carla-Data-Collection
This repository generates a KITTI-like dataset from the POV of a traffic camera, using the CARLA simulator. It uses KITTI sensor parameters as specified [here](https://www.cvlibs.net/datasets/kitti/setup.php). 

## TODOS
- Occlusion and truncation have not been implemented into KITTI labels yet (view KITTI label format [here](https://github.com/bostondiditeam/kitti/blob/master/resources/devkit_object/readme.txt)
- Implement script to properly create testing, training, and validation sets. Currently, files are saved to training and testing folders respectively by server tick odd/even parity.
- Create a more robust list of agents of varied vehicles, pedestrians, and cyclists. Currently only a single car is spawned.
- Use other CARLA maps to create more varied data.
- Add sensors to other intersections to collect more varied data.
- Add label class attributes and corresponding logic to keep track of car velocity.

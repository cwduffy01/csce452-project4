import yaml
import numpy as np

# read map from file
def get_occupancy_grid(file_name):
    with open(file_name) as f:
        world = yaml.safe_load(f)
    world_map = world['map'].split('\n')

    for i in range(len(world_map)):
        world_map[i] = world_map[i].replace('', ' ').split(' ')[1:-1]
    world_map = np.array(world_map[:-1])
    world_map = np.where(world_map == '#', 1, 0)
    world_map = world_map[::-1]

    world['height'] = len(world_map)
    world['width'] = len(world_map[0])
    world['map'] = world_map.flatten().tolist()

    # vectorize
    
    return world

    


import yaml

# read map from file
def get_occupancy_grid(file_name):
    # read map from file
    file_name = 'sim_config/world/brick.world'
    with open(file_name) as f:
        world = yaml.safe_load(f)
    print(world)

    


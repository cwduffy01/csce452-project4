import yaml
import numpy as np

# vectorize helper
def subtract_rows(str1, str2):
    new_arr = ""
    for i in range(len(str1)):
        a = int(str1[i])
        b = int(str2[i])
        c = a-b 
        if c == -1:
            c = 0
        new_arr += str(c)
    return new_arr

# vectorize the map
def vectorize(raw_map, resolution):
    number_map = raw_map.replace("#", "1").replace(".", "0").split('\n')
    width = len(number_map[0])
    height = len(number_map)

    dummy = np.zeros(width)
    vectors = []

    # Bottom Horizontal
    b_horizontal = []
    for i in range(height):
        if i < height-1:
            b_horizontal.append(subtract_rows(number_map[i], number_map[i+1]))
        else:
            b_horizontal.append(dummy)

    for i, row in enumerate(b_horizontal):
        idx = 0
        while(idx < len(row)):
            if row[idx] == "1":
                vector = [round(idx*resolution, 2), round(((height-i-1)*resolution), 2), 0.0, round(((height-i-1)*resolution), 2)]
                while (idx < len(row) and row[idx] == "1"):
                    idx+=1
                vector[2] = round(idx*resolution, 2)
                vectors.append(np.asarray(vector))
            else:
                idx+=1

    # Top Horizontal
    t_horizontal = []
    for i in range(height):
        if i != 0:
            t_horizontal.append(subtract_rows(number_map[i], number_map[i-1]))
        else:
            t_horizontal.append(dummy)

    for i, row in enumerate(t_horizontal):
        idx = 0
        while(idx < len(row)):
            if row[idx] == "1":
                vector = [round(idx*resolution, 2), round(((height-i-1)*resolution)+resolution, 2), 0.0, round(((height-i-1)*resolution)+resolution, 2)]
                while (idx < len(row) and row[idx] == "1"):
                    idx+=1
                vector[2] = round(idx*resolution, 2)
                vectors.append(np.asarray(vector))
            else:
                idx+=1

    for i in range(len(number_map)):
        number_map[i] = [c for c in number_map[i]]
    number_map = np.asarray(number_map)

    rotated_number_map = []
    for i in range(len(number_map[0])):
        rotated_number_map.append(''.join(map(str,number_map[:,i])))
    dummy = np.zeros(height)

    # Right Vertical
    r_vertical = []
    for i in range(width):
        if i < width-1:
            r_vertical.append(subtract_rows(rotated_number_map[i], rotated_number_map[i+1]))
        else:
            r_vertical.append(dummy)

    for i, col in enumerate(r_vertical):
        idx = 0
        while(idx < len(col)):
            if col[idx] == "1":
                vector = [round((i*resolution)+resolution, 2), round(((height-idx-1)*resolution)+resolution, 2), round((i*resolution)+resolution, 2), 0.0]
                while (idx < len(col) and col[idx] == "1"):
                    idx+=1
                vector[3] = round(((height-idx-1)*resolution)+resolution, 2)
                vectors.append(np.asarray(vector))
            else:
                idx+=1

    # Left Vertical
    l_vertical = []
    for i in range(width):
        if i != 0:
            l_vertical.append(subtract_rows(rotated_number_map[i], rotated_number_map[i-1]))
        else:
            l_vertical.append(dummy)

    for i, col in enumerate(l_vertical):
        idx = 0
        while(idx < len(col)):
            if col[idx] == "1":
                vector = [round((i*resolution), 2), round(((height-idx-1)*resolution)+resolution, 2), round((i*resolution), 2), 0.0]
                while (idx < len(col) and col[idx] == "1"):
                    idx+=1
                vector[3] = round(((height-idx-1)*resolution)+resolution, 2)
                vectors.append(np.asarray(vector))
            else:
                idx+=1

    return np.asarray(vectors)

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

    


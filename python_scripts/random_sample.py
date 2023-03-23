import gflags
import cv2
import numpy as np
import read_file as rf
import random

def is_grid_cell(im_array, i, j, rows, cols):
    for k in range(i, i + 10):
        if k >= rows: return False

        for w in range(j, j + 10):
            if w >= cols: return False

            if im_array[k][w] == 0: return False

    return True

def is_one(x):
    return x == 1

def random_sample(data):
    candidate = []
    i = 0
    for row in data:
        candidates_y = list(filter(lambda x: is_one(row[x]), range(len(row))))
        for j in range(len(candidates_y)):
            temp = [i, candidates_y[j]]
            candidate.append(temp)
        i = i + 1
    cand_count = len(candidate)

    agent_sample = []
    for i in range(2):
        rand_num = random.randint(0, cand_count+1)
        agent_sample.append(candidate[rand_num])
    
    # print("randomly sampled agents: " + str(agent_sample))
    agent_sample = [[i*10+5 for i in y] for y in agent_sample]

    return agent_sample

def convert_to_tuple(input):
    temp_tuple = []
    for ts in input:
        for t in ts:
            temp_tuple.append(t)
    temp_tuple = tuple(temp_tuple)
    return temp_tuple



if __name__ == "__main__":
    map_filepath = "/home/kai20/exercise/rdcm/envs/random_example.png"
    im = cv2.imread(map_filepath)
    im_array = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    size = 10

    rows = np.size(im_array, 0)
    cols = np.size(im_array, 1)
    map_data = [[0 for cols in range(cols//size)] for rows in range(rows//size)]

    count = 0
    for i in range(0, rows, size):
        for j in range(0, cols, size):
            if is_grid_cell(im_array, i, j, rows, cols):
                map_data[i//size][j//size] = 1
                count = count + 1
    # print(map_data)
    print(count)

    file_path = "/home/kai20/exercise/rdcm/python_scripts/input.txt"
    agent_start = []
    robot_start = []
    file_message = rf.read_input(file_path)
    agent_start = file_message[5].copy()
    agent_start_1 = [[(i-5)//10 for i in y] for y in agent_start]
    robot_start = file_message[7].copy()
    robot_start_1 = [[(i-5)//10 for i in y] for y in robot_start]

    entities = agent_start_1 + robot_start_1

    for i in range(len(entities)):
        x = entities[i][1]
        y = entities[i][0]
        map_data[x][y] = 0
        count = count - 1

    print(count)
    # print(map_data)

    agent_goal = random_sample(map_data)
    print("goal agents: " + str(agent_goal))

    agent_start_t = convert_to_tuple(agent_start)
    agent_goal_t = convert_to_tuple(agent_goal)
    robot_start_t = convert_to_tuple(robot_start)

    print(file_message)
    temp_message = list(file_message)
    temp_message[6] = agent_goal
    final_message = tuple(temp_message)
    print(final_message)
    with open("/home/kai20/exercise/rdcm/python_scripts/input2.txt", "w") as f:
        f.write("# pixel_size comm_range agents robots stump_dist start_agents[x,y] goal_agents[x,y] start_robots[x,y]" +
                 '\n'+ " ".join(map(str, final_message[0:5])) + " " + " ".join(map(str, agent_start_t)) + 
                 " " + " ".join(map(str, agent_goal_t)) + " " + " ".join(map(str, robot_start_t)))
    # f = open(file_path, "w")
    



# 依次读取pixel size、通信距离、agent数量、robot数量、stump dist、agent起始位置
def read_input(file_path):
    lines = open(file_path, mode="r")
    ag_start = []
    ag_goal = []
    rb_start = []
    for line in lines:
        if line[0] == '#':
            continue
        coord = list(map(int, line.split(" ")))
        p, comm, ag, rb = coord[0], coord[1], coord[2], coord[3]
        # agent起始位置
        ag_st, ag_g, rb_st = coord[4:8], coord[8:12], coord[12:]
        for i in range(2):
            temp = [ag_st[2*i], ag_st[2*i+1]]
            ag_start.append(temp)
            temp = [ag_g[2*i], ag_g[2*i+1]]
            ag_goal.append(temp)
            
        for i in range(7):
            temp = [rb_st[2*i], rb_st[2*i+1]]
            rb_start.append(temp)

        ### debug
        # print(p, comm, ag, rb, stump, ag_start, ag_goal, rb_start)
        return p, comm, ag, rb, ag_start, ag_goal, rb_start

### debug
# if __name__ == "__main__":
#     a = read_input("/home/kai20/exercise/rdcm/python_scripts/input.txt")
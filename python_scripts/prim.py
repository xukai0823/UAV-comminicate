class MiniTree(object):
    def __init__(self, vertex, weight):
        """
        最小生成树
        """
        self.vertex = vertex
        self.weight = weight

    def create_mini_tree(self, start):
        """
        最小生成树
        :param start:
        :return:
        """
        weightsum = 0
        visited = []
        # 标记已访问
        visited.append(start)
        v1, v2 = None, None
        while len(visited) < len(self.vertex):
            min_weight = float('inf')
            for v in visited:
                for i in range(len(self.vertex)):
                    # 边没有被访问过且 权重较小
                    if i not in visited and self.weight[v][i] < min_weight:
                        v1 = v
                        v2 = i
                        min_weight = self.weight[v][i]
            visited.append(v2)
            weightsum = weightsum + self.weight[v1][v2]
            # print('%s -> %s weight = %f' % (self.vertex[v1], self.vertex[v2], self.weight[v1][v2]))
        return weightsum


# if __name__ == '__main__':
#     a = [[10000, 5, 7, 10000, 10000, 10000, 2], 
#          [5, 10000, 10000, 9, 10000, 10000, 3],
#          [7, 10000, 10000, 10000, 8, 10000, 10000], 
#          [10000, 9, 10000, 10000, 10000, 4, 10000],
#          [10000, 10000, 8, 10000, 10000, 5, 4], 
#          [10000, 10000, 10000, 4, 5, 10000, 6],
#          [2, 3, 10000, 10000, 4, 6, 10000], ]
    
#     b = [[float(x) for x in y] for y in a]
    
#     # print(b)
#     # print(type(a))
#     mini_tree = MiniTree(['A', 'B', 'C', 'D', 'E', 'F', 'G'], b)
#     weights = mini_tree.create_mini_tree(0)
#     print(weights)

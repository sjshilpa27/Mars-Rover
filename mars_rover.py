"""
Description:
Take input, call different functions and gives output
Author: Shilpa

"""
import copy
import math
import time
from collections import namedtuple
from heapq import *

node = namedtuple('node', 'state,cost,parent')


class problem:

    def __init__(self, input_file):
        self.algo = ""
        self.sizew = 0
        self.sizeh = 0
        self.start = []
        self.zdiff = 0
        self.no_goal = 0
        self.goal = []
        self.matrix = []
        self.input_file = input_file
        self.actions = [[0, 1], [1, 0], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]]
        self.dict_solution = {}
        self.goal_copy = []

    def process_input(self):
        f = open(self.input_file)
        l = f.readlines()
        self.algo = l[0]
        temp = l[1].split(" ")
        self.sizew = int(temp[0])
        self.sizeh = int(temp[1])
        temp = l[2].split(" ")
        self.start = (int(temp[0]), int(temp[1]))
        self.zdiff = int(l[3])
        self.no_goal = int(l[4])
        for i in range(5, 5 + self.no_goal):
            temp = l[i].split(" ")
            x = (int(temp[0]), int(temp[1]))
            self.goal.append(x)

        self.goal_copy = copy.deepcopy(self.goal)

        for a in range(self.sizew):
            self.matrix.append([])

        for i in range(5 + self.no_goal, 5 + self.no_goal + self.sizeh):
            z = l[i].strip()
            temp = z.split()
            for j in range(len(temp)):
                self.matrix[j].append(int(temp[j]))

    def goal_test(self, state):
        if state in self.goal:
            self.goal.remove(state)
            return True
        return False

    def all_goal_achieved(self):
        if len(self.goal) == 0:
            return True
        return False

    def is_valid(self, node1, node2):
        # print("-----------------------------")
        # print(node1[0])
        # print(node2[0])
        # print(self.sizeh)
        # print(self.sizew)
        if (node2[0][0] >= 0 and node2[0][0] < self.sizew) and (
                node2[0][1] >= 0 and node2[0][1] < self.sizeh):
            # print(abs(self.matrix[node1[0][0]][node1[0][1]] - self.matrix[node2[0][0]][node2[0][1]]))
            if abs(self.matrix[node1[0][0]][node1[0][1]] - self.matrix[node2[0][0]][
                node2[0][1]]) <= self.zdiff:
                return True
        else:
            return False

    def solution(self, target_node):
        # print("Solution")
        t = target_node
        path = []
        while target_node is not None:
            path.append(target_node[0])
            target_node = target_node[2]
        self.dict_solution[t[0]] = [path[::-1], t[1]]

    def bfs(self):
        root = node(self.start, 0, None)
        frontier_list = []
        explored_list = set()
        inheap = set()
        count = 0
        if not self.is_valid(root, root):
            return
        if self.goal_test(root[0]):
            self.solution(root)
            if self.all_goal_achieved():
                return True

        heappush(frontier_list, (count, root))
        inheap.add(root[0])

        while not self.all_goal_achieved() and not len(frontier_list) == 0:
            newnode = heappop(frontier_list)
            #print(newnode)
            inheap.remove(newnode[1][0])
            #print(newnode[1][0])
            explored_list.add(newnode[1][0])
            for action in self.actions:
                child = node((newnode[1][0][0] + action[0], newnode[1][0][1] + action[1]), 0, newnode[1])

                if child[0] not in explored_list and self.is_valid(newnode[1], child) and child[0] not in inheap:
                    if self.goal_test(child[0]):
                        self.solution(child)
                    heappush(frontier_list, (count, child))
                    inheap.add(child[0])
                    count += 1
            #print(frontier_list)
            #print("------------")

    def ucs(self):

        root = node(self.start, 0, None)
        frontier_list = []
        explored_list = set()
        inheap = {}

        heappush(frontier_list, (root[1], root))
        inheap[root[0]] = root[1]

        while not self.all_goal_achieved() and not len(frontier_list) == 0:
            parent = heappop(frontier_list)

            if parent[1][0] in explored_list:
                continue
            explored_list.add(parent[1][0])
            # print(parent[0])
            if self.goal_test(parent[1][0]):
                self.solution(parent[1])

            for action in self.actions:
                child = None
                if abs(action[0]) == 1 and abs(action[1]) == 1:
                    child = node((parent[1][0][0] + action[0], parent[1][0][1] + action[1]), parent[1][1] + 14,
                                 parent[1])
                else:
                    child = node((parent[1][0][0] + action[0], parent[1][0][1] + action[1]), parent[1][1] + 10,
                                 parent[1])

                if child[0] not in explored_list and self.is_valid(parent[1], child):

                    if child[0] in inheap and inheap[child[0]] > child[1]:
                        inheap[child[0]] = child[1]
                    elif child[0] not in inheap:
                        heappush(frontier_list, (child[1], child))

    def heuristic(self, goal, child):
        return math.sqrt((goal[0] - child[0]) * (goal[0] - child[0]) + (goal[1] - child[1]) * (goal[1] - child[1])) * 10

    def a_star(self):

        for goal in self.goal:
            root = node(self.start, 0, None)
            frontier_list = []
            explored_list = set()
            count = 0
            inheap = {}
            heappush(frontier_list, (root[1], root))
            inheap[root[0]] = root[1]
            while not len(frontier_list) == 0:
                parent = heappop(frontier_list)
                count += 1
                if parent[1][0] in explored_list:
                    continue
                explored_list.add(parent[1][0])
                if parent[1][0] == goal:
                    self.solution(parent[1])
                    break
                for action in self.actions:
                    temp = node((parent[1][0][0] + action[0], parent[1][0][1] + action[1]), 0, parent[1])
                    if temp[0] not in explored_list and self.is_valid(parent[1], temp):
                        if abs(action[0]) == 1 and abs(action[1]) == 1:
                            child = node(temp[0], parent[1][1] + 14 + abs(
                                self.matrix[parent[1][0][0]][parent[1][0][1]] - self.matrix[temp[0][0]][temp[0][1]]),
                                         parent[1])
                        else:
                            child = node(temp[0], parent[1][1] + 10 + abs(
                                self.matrix[parent[1][0][0]][parent[1][0][1]] - self.matrix[temp[0][0]][temp[0][1]]),
                                         parent[1])
                        fn = child[1] + self.heuristic(goal, child[0])
                        if child[0] in inheap and inheap[child[0]] > fn:
                            inheap[child[0]] = fn
                        elif child[0] not in inheap:
                            heappush(frontier_list, (fn, child))


def output(solution, goal_list):
    f = open("output.txt", "w")
    for each in goal_list:
        if each in solution:
            path = solution[each][0]
            for i in path:
                if i == each:
                    f.write(str(i[0]) + "," + str(i[1]) + "\n")
                else:
                    f.write(str(i[0]) + "," + str(i[1]) + " ")
        else:
            f.write("FAIL\n")
    f.close()


if __name__ == "__main__":
    start_time = time.time()
    p = problem("huge_input (1) (1).txt")
    p.process_input()

    if p.algo.strip() == "BFS":
        p.bfs()
    elif p.algo.strip() == "UCS":
        p.ucs()
    elif p.algo.strip() == "A*":
        p.a_star()

    print(p.dict_solution)
    print(p.goal_copy)
    output(p.dict_solution, p.goal_copy)
    print(len(p.dict_solution[(99, 98)][0]))
    #print("--- %s seconds ---" % (time.time() - start_time))

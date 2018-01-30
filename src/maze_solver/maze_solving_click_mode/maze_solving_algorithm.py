import cv2
import numpy as np

def find_direction(low_pre, col_pre, low_now, col_now):

    if low_now - low_pre == 1 and col_now - col_pre == 0:
        return 'right'
    if low_now - low_pre == 1 and col_now - col_pre == -1:
        return 'upper_right'
    if low_now - low_pre == 0 and col_now - col_pre == -1:
        return 'upper'
    if low_now - low_pre == -1 and col_now - col_pre == -1:
        return 'upper_left'
    if low_now - low_pre == -1 and col_now - col_pre == 0:
        return 'left'
    if low_now - low_pre == -1 and col_now - col_pre == 1:
        return 'bottom_left'
    if low_now - low_pre == 0 and col_now - col_pre == 1:
        return 'bottom'
    if low_now - low_pre == 1 and col_now - col_pre == 1:
        return 'bottom_right'

class Solver(object):
    def __init__(self, start, goal, map_recieved):
        self.lows = abs(start[0] - goal[0]) + 51
        self.cols = abs(start[1] - goal[1]) + 51
        self.difference_low = min(start[0], goal[0]) -25
        self.difference_col = min(start[1], goal[1]) -25
        self.start = [start[0] - self.difference_low, start[1] - self.difference_col]
        self.goal = [goal[0] - self.difference_low, goal[1] - self.difference_col]
        self.distance = np.zeros((self.lows, self.cols,3), np.uint16)  # current, remain, total
        self.visited = np.zeros((self.lows, self.cols), dtype=np.bool)
        self.parent = np.zeros((self.lows, self.cols, 2), np.uint16)
        self.map = np.zeros((self.lows, self.cols), dtype=np.bool)
        self.buffer_direction = 'none'
        for i in range(self.lows):
            for j in range(self.cols):
                if map_recieved[384*(j + self.difference_col) + (i + self.difference_low)] == 100:
                    for k in [-3, -2, -1, 0, 1, 2, 3]:
                        for m in [-3, -2, -1, 0, 1, 2, 3]:
                            if i + k in range(self.lows) and j + m in range(self.cols):
                                self.map[i + k][j + m] = True
        self.shortest_path = [goal]

    def check_existance(self, arr, coordinates):
        length = len(arr)
        if length > 3:
            start = length - 3
        else:
            start = 0

        for i in range(0, length):
            if arr[i][0] == coordinates[0] and arr[i][1] == coordinates[1]:
                return i
        return False

    def solve_distance(self):
        finish = 0
        current_step = [self.start]
        next_step = []
        for i in range(0, self.lows):    # initialize
            for j in range(0, self.cols):
                self.distance[i][j][0] = 0
                self.distance[i][j][1] = 0
                self.distance[i][j][2] = 0
                self.visited[i][j] = False
                for k in range(0, 2):
                    self.parent[i][j][k] = 0
        self.visited[self.start[0]][self.start[1]] = True
        self.parent[self.start[0]][self.start[1]][0] = self.start[0]
        self.parent[self.start[0]][self.start[1]][1] = self.start[1]

        #current
        while(len(current_step) != 0):
            while(len(current_step) != 0):
                current_step_ = current_step.pop()
                for i in [-1, 0, 1]: # lows
                    for j in [-1, 0, 1]: # cols
                        if i != 0 or j != 0:
                            if (current_step_[0] + i) in range(0, self.lows) and (current_step_[1] + j) in range(0, self.cols) and self.visited[current_step_[0] + i][current_step_[1] + j] == False and self.map[current_step_[0] + i][current_step_[1] + j] == False:
                                if i !=0 and j !=0:
                                    distance_ = self.distance[current_step_[0]][current_step_[1]][0] + 15
                                else:
                                    distance_ = self.distance[current_step_[0]][current_step_[1]][0] + 10
                                existance = self.check_existance(next_step, [current_step_[0] + i, current_step_[1] + j])
                                if existance == False:
                                    next_step.append([current_step_[0] + i, current_step_[1] + j, distance_])
                                    self.parent[current_step_[0] + i][current_step_[1] + j][0] = current_step_[0]
                                    self.parent[current_step_[0] + i][current_step_[1] + j][1] = current_step_[1]

                                else:
                                    index_ = existance
                                    low_ = next_step[index_][0]
                                    col_ = next_step[index_][1]
                                    val_ = next_step[index_][2]
                                    if distance_ < val_:
                                        next_step[index_][2] = distance_
                                        self.parent[current_step_[0] + i][current_step_[1] + j][0] = current_step_[0]
                                        self.parent[current_step_[0] + i][current_step_[1] + j][1] = current_step_[1]


            while(len(next_step) !=0):
                buffer = next_step.pop()
                if buffer[0] == self.goal[0] and buffer[1] == self.goal[1]:
                    finish = 1
                current_step.append([buffer[0], buffer[1]])
                self.distance[buffer[0]][buffer[1]][0] = buffer[2]
                self.visited[buffer[0]][buffer[1]] = True


        for i in range(self.lows):
            for j in range(self.cols):
                #remains
                remain_lows = abs(self.goal[0] - i)
                remain_cols = abs(self.goal[1] - j)
                remains = np.min([remain_lows, remain_cols]) * 15 + abs(remain_lows - remain_cols) * 10
                self.distance[i, j, 1] = remains
                #total
                self.distance[i, j, 2] = self.distance[i, j, 0] + self.distance[i, j, 1]


    def find_shortest_path(self):
        current_point = self.goal
        same_value = 0


        while(current_point[0] != self.start[0] or current_point[1] != self.start[1]):
            shortest_value = 10000
            same_value = 0
            for i in [1, 0, -1]:
                for j in [1, 0, -1]:
                    if (i != 0 or j != 0) and (current_point[0] + i) in range(0, self.lows) and (current_point[1] + j) in range(0, self.cols) and self.map[current_point[0] + i][current_point[1] + j] == False:
                        if self.distance[current_point[0] + i][current_point[1] + j][2] < shortest_value:
                            shortest_value = self.distance[current_point[0] + i][current_point[1] + j][2]
                            next_point = [current_point[0] + i, current_point[1] + j]


            for i in [1, 0, -1]:
                for j in [1, 0, -1]:
                    if (i != 0 or j != 0) and (current_point[0] + i) in range(0, self.lows) and (current_point[1] + j) in range(0, self.cols) and self.map[current_point[0] + i][current_point[1] + j] == False:
                        if self.distance[current_point[0] + i][current_point[1] + j][2] == shortest_value:
                            same_value = same_value +1

            if same_value > 1:
                next_point = self.parent[current_point[0]][current_point[1]]

            if find_direction(self.shortest_path[len(self.shortest_path)-1][0], self.shortest_path[len(self.shortest_path)-1][1], next_point[0] + self.difference_low, next_point[1] + self.difference_col) == self.buffer_direction:
                _ = self.shortest_path.pop()

            else:
                self.buffer_direction = find_direction(self.shortest_path[len(self.shortest_path)-1][0], self.shortest_path[len(self.shortest_path)-1][1], next_point[0] + self.difference_low, next_point[1] + self.difference_col)
            self.shortest_path.append([next_point[0] + self.difference_low, next_point[1] + self.difference_col])
            current_point = next_point
    def collision_test(self, current_pos, test_pos):
        start = [current_pos[0] - self.difference_low, current_pos[1] - self.difference_col]
        goal = [test_pos[0] - self.difference_low, test_pos[1] - self.difference_col]
        if goal[0] != start[0]:
            a = (goal[1] - start[1]) / (goal[0] - start[0])
            b = -a*start[0] + start[1]
            for i in range(min(start[0], goal[0]), max(start[0], goal[0])):
                if self.map[i][int(a*i + b)] == True:
                    print "collision in :", i, int(a*i + b)
                    return 'danger'
        else:
            for i in range(min(start[1], goal[1]), max(start[1], goal[1])):
                if self.map[start[0]][i] == True:
                    return 'danger'
        return 'safe'

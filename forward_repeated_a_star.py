
from binaryHeap import BinaryHeap
import time
import pickle
import copy

import sys

time_start = time.time()

start_time = time.time()

UI = False
revG = False

MAZE_NUMBER = int(sys.argv[1])

mazes = pickle.load(open("mazes.pkl", "rb"))
a, start, end = mazes[MAZE_NUMBER][0], mazes[MAZE_NUMBER][1], mazes[MAZE_NUMBER][2]
f = copy.deepcopy(a)

if UI:
    import pygame
    pygame.init()
    pygame.mixer.init()
BLACK = (0, 0, 0)
WHITE = (200, 200, 200)
GREEN = (0, 255, 0,)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
WINDOW_HEIGHT = 850
WINDOW_WIDTH = 850
WIDTH = 6.3
HEIGHT = WIDTH
MARGIN = 2
FPS = 30




start_time = time.time()
counter = 0

def drawGrid():
    if UI:
        for row in range(len(a)):
            for column in range(len(a[0])):
                color = WHITE
                if a[row][column] == '*':
                    color = RED
                if a[row][column] == 'S':
                    color = BLUE
                if a[row][column] == 'T':
                    color = YELLOW
                pygame.draw.rect(SCREEN,
                                color,
                                [(MARGIN + WIDTH) * column + MARGIN,
                                (MARGIN + HEIGHT) * row + MARGIN,
                                WIDTH,
                                HEIGHT])
            
class Step:
    """
        _summary_ = "A Step object"
    """

    def __init__(self, x, y, previous) -> None:
        self.x = x
        self.y = y
        self.previous = previous

    def __str__(self) -> str:
        out = "x = " + str(self.x) + ", y= " + str(self.y)
        return out

    def getPrevious(self):
        return self.previous





block = {}
closed = {}
hval = {}
seen = {}
gval = {}
finalPath = []


def isClear(x, y):
    """
    Function to check if a cell is clear or not

    Args:
        x (int): x coordinate
        y (int): y coordinate

    Returns:
        bool: whether the cell is clear or not
    """
    if ((y >= 0 and y < len(a[0])) and (x >= 0 and x < len(a))):
        if ((not block.get((x, y)))) and not closed.get((x, y)):
            return True
    return False


def getBlockers(it):
    """Function to get obstacles in the maze that the agent can see

    Args:
        it (Step): Point to check for obstacles
    """
    if ((it.y >= 0 and it.y < len(a[0])) and (it.x+1 >= 0 and it.x+1 < len(a))):
        if (a[it.x+1][it.y] == "*"):
            block[(it.x+1, it.y)] = True

    if ((it.y >= 0 and it.y < len(a[0])) and (it.x-1 >= 0 and it.x-1 < len(a))):
        if (a[it.x-1][it.y] == "*"):
            block[(it.x-1, it.y)] = True

    if ((it.y+1 >= 0 and it.y+1 < len(a[0])) and (it.x >= 0 and it.x < len(a))):
        if (a[it.x][it.y+1] == "*"):
            block[(it.x, it.y+1)] = True

    if ((it.y-1 >= 0 and it.y-1 < len(a[0])) and (it.x >= 0 and it.x < len(a))):
        if (a[it.x][it.y-1] == "*"):
            block[(it.x, it.y-1)] = True


def calcHval(point):
    """
    Function to calculate the (h) heuristic value of a point

    Args:
        point (Step): The point to be evaluated

    Returns:
        int: h_value
    """
    if (hval.get((point.x, point.y))):
        return hval.get((point.x, point.y))

    hval[(point.x, point.y)] = abs(end[0]-point.x) + abs(end[1]-point.y)
    return hval[(point.x, point.y)]


def findAstar(point):
    """Function to run the A* algorithm

    Args:
        point (Step): starting point

    Returns:
        Step: goal point if found, else False
    """
    closed.clear()
    getBlockers(point)
    q1 = BinaryHeap()
    hval = calcHval(point)
    cost = hval + gval[(point.x, point.y)]
    
    # For reversed g priority, uncomment the following line and comment the next one
    if revG:
        q1.put((cost, -1*gval[(point.x, point.y)], point))
    else:
        q1.put((cost, gval[(point.x, point.y)], point))

    while q1.size > 0:
        if not UI:
            if time.time() - start_time > 30:
                print("Time limit exceeded", MAZE_NUMBER)
                exit()
        current = q1.pop()[2]
        closed[(current.x, current.y)] = True
        if a[current.x][current.y] == 'T':
            return current

        if (isClear(current.x+1, current.y)):
            s1 = Step(current.x+1, current.y, current)
            hval = calcHval(s1)
            cost = hval + gval[(current.x, current.y)]+1
            gval[(current.x+1, current.y)] = gval[(current.x, current.y)]+1
            if ((current.x+1, current.y) not in closed):
                # For reversed g priority, uncomment the following line and comment the next one
                if revG:
                    q1.put((cost, -1*gval[(current.x+1, current.y)], Step(current.x+1, current.y, current)))
                else:
                    q1.put((cost, gval[(current.x+1, current.y)], Step(current.x+1, current.y, current)))
                seen[(current.x+1, current.y)] = True
                closed[(current.x+1, current.y)] = True

        if (isClear(current.x-1, current.y)):
            s1 = Step(current.x-1, current.y, current)
            hval = calcHval(s1)
            cost = hval + gval[(current.x, current.y)]+1
            gval[(current.x-1, current.y)] = gval[(current.x, current.y)]+1
            if ((current.x-1, current.y) not in closed):
                # For reversed g priority, uncomment the following line and comment the next one
                if revG:
                    q1.put((cost, -1*gval[(current.x-1, current.y)], Step(current.x-1, current.y, current)))
                else: 
                    q1.put((cost, gval[(current.x-1, current.y)], Step(current.x-1, current.y, current)))
                seen[(current.x-1, current.y)] = True
                closed[(current.x-1, current.y)] = True

        if (isClear(current.x, current.y+1)):
            s1 = Step(current.x, current.y+1, current)
            hval = calcHval(s1)
            cost = hval + gval[(current.x, current.y)]+1
            gval[(current.x, current.y+1)] = gval[(current.x, current.y)]+1
            if ((current.x, current.y+1) not in closed):
                # For reversed g priority, uncomment the following line and comment the next one
                if revG:
                    q1.put((cost, -1*gval[(current.x, current.y+1)], Step(current.x, current.y+1, current)))
                else:
                    q1.put((cost, gval[(current.x, current.y+1)], Step(current.x, current.y+1, current)))
                seen[(current.x, current.y+1)] = True
                closed[(current.x, current.y+1)] = True

        if (isClear(current.x, current.y-1)):
            s1 = Step(current.x, current.y-1, current)
            hval = calcHval(s1)
            cost = hval + gval[(current.x, current.y)]+1
            gval[(current.x, current.y-1)] = gval[(current.x, current.y)]+1
            if ((current.x, current.y-1) not in closed):
                # For reversed g priority, uncomment the following line and comment the next one
                if revG:
                    q1.put((cost, -1*gval[(current.x, current.y-1)], Step(current.x, current.y-1, current)))
                else:
                    q1.put((cost, gval[(current.x, current.y-1)], Step(current.x, current.y-1, current)))
                seen[(current.x, current.y-1)] = True
                closed[(current.x, current.y-1)] = True

    return False


def printprog(res):
    li = []
    while (res.getPrevious()):
        li.append((res.x, res.y))
        res = res.getPrevious()
    li.append((res.x, res.y))
    li.reverse()
    
    if UI:
        time.sleep(0.1)
        for i in li:
            pygame.draw.rect(SCREEN,
                                GREEN,
                                [(MARGIN + WIDTH) * i[1] + MARGIN + MARGIN-2,
                                (MARGIN + HEIGHT) * i[0] + MARGIN + MARGIN-2,
                                WIDTH-8,
                                HEIGHT-8])
        pygame.display.update()

    # if UI:
    ci = 0
    for i in li:
        x = i[0]
        y = i[1]
        if (f[x][y]) != "*":
            f[x][y] = "\033[1;32;43mS"
            tu = (x, y)
            if (ci > 0):
                f[prevx][prevy] = "\033[1;32;43m "
            prevx = x
            prevy = y
        else:
            break
        ci = ci+1

    countx = 0
    # Uncomment for terminal maze
    # for e in f:
    #     county = 0
    #     for r in e:
    #         if((countx, county) in block or (countx, county) in seen):
    #             print("\033[1;34;40m",r,end="")

    #         else:
    #             print("\033[0;37;41m",r,end="")
    #         county = county+1
    #     print("\033[0;37;40m\n",end='')
    #     countx = countx+1

    
    for i in li:
        if UI:
            pygame.draw.rect(SCREEN,
                        BLUE,
                        [(MARGIN + WIDTH) * i[1] + MARGIN + MARGIN-2,
                        (MARGIN + HEIGHT) * i[0] + MARGIN + MARGIN-2,
                        WIDTH-8,
                        HEIGHT-8])
    
    
            pygame.display.update()
        #   time.sleep(0.1)
        if(i[0] == tu[0] and i[1] == tu[1]):
            break
        finalPath.append(i)

    a[new[0]][new[1]] = " "
    a[tu[0]][tu[1]] = "S"
    return tu


if UI:
    SCREEN = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    SCREEN.fill(BLACK)
    pygame.display.set_caption("Python Maze Generator")
    clock = pygame.time.Clock()

    drawGrid()
    pygame.display.update()

out = Step(start[0], start[1], None)
new = start
while (out != end):
    counter += 1
    if UI:
        drawGrid()
        pygame.display.update()
    gval.clear()
    closed.clear()
    it = Step(new[0], new[1], None)
    gval[(it.x, it.y)] = 0
    res = findAstar(it)
    if res == False:
        print("No path found")
        break
    new = printprog(res)
    out = new
if UI:
    drawGrid()
    pygame.display.update()
    finalPath.append(out)
    for i in finalPath:
        # if i != start or i != end:
        pygame.draw.rect(SCREEN,
                                GREEN,
                                [(MARGIN + WIDTH) * i[1] + MARGIN,
                                (MARGIN + HEIGHT) * i[0] + MARGIN,
                                WIDTH,
                                HEIGHT])
        pygame.display.update()
    pygame.draw.rect(SCREEN,
                                BLUE,
                                [(MARGIN + WIDTH) * start[1] + MARGIN,
                                (MARGIN + HEIGHT) * start[0] + MARGIN,
                                WIDTH,
                                HEIGHT])
    pygame.draw.rect(SCREEN,
                                YELLOW,
                                [(MARGIN + WIDTH) * end[1] + MARGIN,
                                (MARGIN + HEIGHT) * end[0] + MARGIN,
                                WIDTH,
                                HEIGHT])
    pygame.display.update()


print(str(MAZE_NUMBER)+", ", str(time.time()-start_time) +
      ","+str(len(seen)), ",", str(counter))
if UI:
    ##### pygame loop #######
    running = True
    while running:
        # keep running at the at the right speed
        clock.tick(FPS)
        # process input (events)
        for event in pygame.event.get():
            # check for closing the window
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()

import pygame
import math
from queue import PriorityQueue

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")
pygame.init()

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row*width
        self.y = col*width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_start(self):
        self.color = ORANGE

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors=[]
        if self.row < self.total_rows - 1 and not grid[self.row +  1][self.col].is_barrier(): #Down
            self.neighbors.append(grid[self.row +  1][self.col])

        if self.row > 0 and not grid[self.row -  1][self.col].is_barrier(): #up
            self.neighbors.append(grid[self.row -  1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col+ 1].is_barrier(): #Right
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0  and not grid[self.row][self.col - 1].is_barrier(): #Left
            self.neighbors.append(grid[self.row][self.col-1])

    def __lt__(self, other):
        return False

def h(p1,p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1-x2)+abs(y1-y2)

def reconstuct_path(came_from, current, draw):
    while current in came_from:
        current= came_from[current]
        current.make_path()
        draw()



def astar(draw, grid, start, end):
    count = 0
    open_set= PriorityQueue()
    open_set.put((0, count, start)) #start by putting the start node in open set
    came_from= {} #keeps track of what nodes came from where
    g_score={spot: float("inf") for row in grid for spot in row} #current shortest distance from start node to this node
    g_score[start] = 0 #initialize at zero b/c distance from start to start is 0
    f_score={spot: float("inf") for row in grid for spot in row} #predicted distance starts at infinity for all nodes
    f_score[start] = h(start.get_pos(), end.get_pos())    #sets f score to L distance (hureisitc) as we are at start

    open_set_hash = {start}  #see whats in open set
    while not open_set.empty():   #while there is stuff in open set set
        for event in pygame.event.get():  #try quit
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2] # get just the node associated with smallest f score
        open_set_hash.remove (current)

        if current == end:  #draw path
            reconstuct_path(came_from, end, draw )
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:   #look at neighbors
            temp_g_score = g_score[current]+1  #calculate temp g score add 1 because weight is 1
            if temp_g_score < g_score[neighbor]: # if neighbor less than others than update path
                came_from[neighbor]= current
                g_score[neighbor] = temp_g_score
                f_score[neighbor]= temp_g_score+ h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:  #add to open set hash
                    count+=1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()

    return False

def dijkstras(draw, grid, start, end):
    count = 0
    open_set= PriorityQueue()
    open_set.put((0, count, start)) #start by putting the start node in open set
    came_from= {} #keeps track of what nodes came from where
    length={spot: float("inf") for row in grid for spot in row} #current shortest distance from start node to this node
    length[start] = 0 #initialize at zero b/c distance from start to start is 0
    open_set_hash = {start}  #see whats in open set

    

    while not open_set.empty():   #while there is stuff in open set set

        current = open_set.get()[2] # get just the node associated with smallest f score
        open_set_hash.remove (current)

        for event in pygame.event.get():  #try quit
            if event.type == pygame.QUIT:
                pygame.quit()

        if current == end:  #draw path
            
            reconstuct_path(came_from, end, draw )
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            temp_length= length[current] +1
            if temp_length < length[neighbor]:
                came_from[neighbor]= current
                length[neighbor]= temp_length
                if neighbor not in open_set_hash:  #add to open set hash
                    count+=1
                    open_set.put((length[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()

    return False

def greedy(draw, grid, start, end):
    count = 0
    open_set= PriorityQueue()
    open_set.put((0, count, start)) #start by putting the start node in open set
    came_from= {} #keeps track of what nodes came from where
    f_score={spot: float("inf") for row in grid for spot in row} #predicted distance starts at infinity for all nodes
    f_score[start] = h(start.get_pos(), end.get_pos())    #sets f score to L distance (hureisitc) as we are at start
    open_set_hash = {start}  #see whats in open set


    while not open_set.empty():   #while there is stuff in open set set
        for event in pygame.event.get():  #try quit
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2] # get just the node associated with smallest f score
        open_set_hash.remove (current)

        if current == end:  #draw path
            reconstuct_path(came_from, end, draw )
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:   #look at neighbors
            temp_f_score = h(neighbor.get_pos(), end.get_pos()) #calculate f score of neighbor
            if temp_f_score < f_score[neighbor]: # if neighbor less than others than update path
                came_from[neighbor]= current
                f_score[neighbor] = temp_f_score
                
                if neighbor not in open_set_hash:  #add to open set hash
                    count+=1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()
        

    return False

def nextAlgo(draw, grid, start):
    count = 0
    open_set= PriorityQueue()
    open_set.put((count, start))
    came_from= {} #keeps track of what nodes came from where
    open_set_hash = {start}

    while not open_set.empty():   #while there is stuff in open set set
        for event in pygame.event.get():  #try quit
            if event.type == pygame.QUIT:
                pygame.quit()
        current = open_set.get()[1]
        open_set_hash.remove (current)

        for neighbor in current.neighbors:   #look at neighbors
            if neighbor.color == RED or neighbor.color== GREEN or neighbor.color == PURPLE:
                neighbor.reset()
                came_from[neighbor]= current
                if neighbor not in open_set_hash:  #add to open set hash
                    count+=1
                    open_set.put((count, neighbor))
                    open_set_hash.add(neighbor)
                    
        draw()


    return False






def make_grid(rows,width):
    grid= []
    gap= width//rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot= Spot(i,j,gap,rows)
            grid[i].append(spot)

    return grid


def draw_grid(win, rows,width):
    gap= width//rows
    for i in range(rows):
        pygame.draw.line(win,GREY,(0,i*gap),(width,i*gap))
    for j in range(rows):
        pygame.draw.line(win,GREY,(j*gap,0),(j*gap,width))

def draw(win,grid,rows,width):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win,rows,width)
    pygame.display.update()


def get_clicked_pos(pos,rows,width):
    gap = width // rows
    y,x = pos
    
    row = y // gap
    col=  x // gap
    return row, col



def main(win,width):
    ROWS= 50
    grid= make_grid(ROWS, width)

    start= None
    end= None

    run= True
    started= False
    while run:
        draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:
                pos= pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos,ROWS,width)
                spot= grid[row][col]

                if not start:
                    start= spot
                    start.make_start()
                elif not end and spot.get_pos() != start.get_pos():
                    end = spot
                    end.make_end()
                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:
                pos= pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos,ROWS,width)
                spot= grid[row][col]
                spot.reset()
                if spot == start:
                    start = None

                if spot == end:
                    end = None

            if event.type== pygame.KEYDOWN:
                if event.key == pygame.K_g and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    greedy(lambda: draw(win, grid, ROWS, width), grid, start, end)

                if event.key == pygame.K_a and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    astar(lambda: draw(win, grid, ROWS, width), grid, start, end)
                if event.key == pygame.K_d and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    dijkstras(lambda: draw(win, grid, ROWS, width), grid, start, end)
                if event.key == pygame.K_n and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    nextAlgo(lambda: draw(win, grid, ROWS, width), grid, start)
    


                if event.key == pygame.K_r:
                    start= None
                    end= None
                    grid= make_grid(ROWS,width)






    pygame.quit()

def intro():
    smallfont = pygame.font.SysFont(None, 30)
    intro= True
    while intro == True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            elif event.type == pygame.KEYDOWN:
                intro = False
        WIN.fill(WHITE)
        text1 = smallfont.render("Welcome to Pathfinding Visualizer",True , PURPLE)
        WIN.blit(text1, [230,160])
        text2 = smallfont.render("Use Left Click to Draw and Right Click to Erase",True , BLUE)
        WIN.blit(text2, [180,240])
        text3 = smallfont.render("Press R to Reset",True , BLUE)
        WIN.blit(text3, [180,440])
        text4 = smallfont.render("Press D for Dijkstras",True , BLUE)
        WIN.blit(text4, [180,280])
        text5= smallfont.render("Press A for A*",True , BLUE)
        WIN.blit(text5, [180,320])
        text6= smallfont.render("Press G for Greedy Best-First",True , BLUE)
        WIN.blit(text6, [180,400])
        text7= smallfont.render("Press N to Try Grid With New Algorithm",True , BLUE)
        WIN.blit(text7, [180,360])
        text8= smallfont.render("Press Any Key to Continue",True , RED)
        WIN.blit(text8, [280,600])
        pygame.display.update()
        clock = pygame.time.Clock()
        clock.tick(15)
        
intro()
main(WIN, WIDTH)



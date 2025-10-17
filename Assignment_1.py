import sys
sys.stdin = open("input.txt", "r", encoding="utf-8")

from abc import ABC, abstractmethod
from heapq import heappush, heappop


# Algorithms
# -----------------------------------------------
class PathfindingStrategy(ABC):
    @abstractmethod
    def find_path(self, map_obj, start, goal, is_walkable):
        pass

class AStarStrategy(PathfindingStrategy):
    def heuristic(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)  # Манхэттенская дистанция

    def find_path(self, map_obj, start, goal, is_walkable):
        # сброс значений, чтобы старые f/g не мешали
        for row in map_obj.grid:
            for cell in row:
                cell.reset_path()


        start_cell = map_obj.get_cell(*start)
        goal_cell = map_obj.get_cell(*goal)

        open_list = []
        closed = set()

        start_cell.g = 0
        start_cell.h = self.heuristic(start_cell, goal_cell)
        start_cell.f = start_cell.g + start_cell.h
        heappush(open_list, (start_cell.f, start_cell.x, start_cell.y, start_cell))  # <— вот так

        while open_list:
            _, _, _, current = heappop(open_list)
            if (current.x, current.y) in closed:
                continue
            closed.add((current.x, current.y))

            if current == goal_cell:
                break  # нашли цель

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = current.x + dx, current.y + dy
                neighbor = map_obj.get_cell(nx, ny)
                if not neighbor or not is_walkable(neighbor):
                    continue

                if neighbor.visited:
                    new_g = current.g + 3  # штраф за возврат в уже посещённое место
                else:
                    new_g = current.g + 1

                if new_g < neighbor.g:
                    neighbor.g = new_g
                    neighbor.h = self.heuristic(neighbor, goal_cell)
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.parent = current
                    heappush(open_list, (neighbor.f, neighbor.x, neighbor.y, neighbor))

        # восстановление пути
        path = []
        cur = goal_cell
        while cur and cur != start_cell:
            path.append((cur.x, cur.y))
            cur = cur.parent
        path.reverse()
        return path

    def print_path_values(self):
        for row in self.grid:
            for c in row:
                if c.f < float('inf'):
                    print(f"{int(c.g):2}/{int(c.h):2}/{int(c.f):2}", end=" ")
                else:
                    print(" . ", end=" ")
            print()
        print()
# -----------------------------------------------




class Character:
    def __init__(self, name="Unknown", x=None, y=None):
        self.name = name
        self.x = x
        self.y = y

# Items
# -----------------------------------------------
class Item(Character):
    pass

class MithrilMail(Item):
    def __init__(self, x=None, y=None):
        super().__init__("Mithril", x, y)

class OneRing(Item):
    def __init__(self, x=None, y=None):
        super().__init__("Ring", x, y)
# -----------------------------------------------


# Abstract Enemy
class Enemy(Character):
    def __init__(self, name, x=None, y=None):
        super().__init__(name, x, y)
        self.dirs = []

    def base_zone(self):
        zone = []
        for dx, dy in self.dirs:
            zone.append((self.x + dx, self.y + dy))
        return zone



# Enemies
# -----------------------------------------------
class OrcPatrol(Enemy):
    def __init__(self, x=None, y=None):
        super().__init__("Orc Patrol", x, y)
        self.dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]


class UrukHai(Enemy):
    def __init__(self, x=None, y=None):
        super().__init__("UrukHai", x, y)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if abs(dx) + abs(dy) <= 2 and not (dx == 0 and dy == 0):
                    self.dirs.append((dx, dy))


class Nazgul(Enemy):
    def __init__(self, x=None, y=None):
        super().__init__("Nazgul", x, y)
        self.dirs = [(dx, dy) for dx in range(-1, 2) for dy in range(-1, 2) if not (dx == 0 and dy == 0)]
        self.dirs += [(2, 2), (-2, 2), (2, -2), (-2, -2)]


class MordorWatchtower(Enemy):
    def __init__(self, x=None, y=None):
        super().__init__("Mordor Watchtower", x, y)
        self.dirs = [(dx, dy) for dx in range(-2, 3) for dy in range(-2, 3) if not (dx == 0 and dy == 0)]

# -----------------------------------------------


# Abstract Character
class Goal(Character):
    def __init__(self, name, x=None, y=None):
        super().__init__(name, x, y)

# Goal Characters
# -----------------------------------------------
class Gollum(Goal):
    def __init__(self, x=None, y=None):
        super().__init__("Gollum", x, y)

class MountDoom(Goal):
    def __init__(self, x=None, y=None):
        super().__init__("Mount Doom", x, y)
# -----------------------------------------------


# -----------------------------------------------
class FrodoAgent(Character):
    def __init__(self, radius, gollum_x, gollum_y):
        super().__init__("Frodo", 0, 0)
        # Позиция и параметры
        self.radius = radius
        self.goal = Gollum(gollum_x, gollum_y)

        self.map = Map(agent=self, goal=self.goal)
        self.pathfinder = AStarStrategy()

    # --- сенсорика ---
    def perceive(self, k):
        def choose_type(symbol, x=None, y=None):
            if symbol == 'O': return OrcPatrol(x, y)
            if symbol == 'U': return UrukHai(x, y)
            if symbol == 'N': return Nazgul(x, y)
            if symbol == 'W': return MordorWatchtower(x, y)
            if symbol == 'G': return Gollum(x, y)
            if symbol == 'M': return MountDoom(x, y)
            if symbol == 'C': return MithrilMail(x, y)
            if symbol == 'R': return OneRing(x, y)
            return None

        for _ in range(k):
            x, y, symbol = input().split()
            x, y = int(x), int(y)
            obj = choose_type(symbol, x, y)

            if obj:
                self.map.place(obj, x, y)
                self.map.place_perception(obj)
            elif symbol == 'P':
                self.map.grid[x][y].set_danger(True)

    # --- планирование ---
    def is_walkable(self, cell):
        if cell.danger or isinstance(cell.character, Enemy):
            return False
        return True  # остальное — можно

    def next_move(self):
        start = (self.x, self.y)
        goal = (self.goal.x, self.goal.y)
        path = self.pathfinder.find_path(self.map, start, goal, self.is_walkable)
        if len(path) > 0:
            nx, ny = path[0]
            # избегаем шага в то место, где только что был
            if hasattr(self, "last_pos") and (nx, ny) == self.last_pos:
                if len(path) > 1:
                    nx, ny = path[1]
            self.last_pos = (self.x, self.y)
            return nx, ny
        return self.x, self.y

    # --- движение ---
    def move(self, nx, ny):
        self.map.place(self, nx, ny)
        self.x, self.y = nx, ny
# -----------------------------------------------


class Cell:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.character = None
        self.visited = False
        self.danger = False

        self.g = float('inf')
        self.h = 0
        self.f = 0
        self.parent = None

    def reset_path(self):
        self.g = float('inf')
        self.h = 0
        self.f = 0
        self.parent = None

    def change_character(self, new_character=None):
        if isinstance(new_character, FrodoAgent):
            self.change_visited()
        self.character = new_character

    def change_visited(self):
        self.visited = True

    def set_danger(self, value=True):
        self.danger = value

    def clear(self):
        self.character = None
        self.danger = False
        self.visited = False

    def draw(self):
        if self.character:
            return self.character.name[0]
        if self.danger:
            return '#'
        if self.visited:
            return '*'
        return '.'

    def is_empty(self):
        return self.character is None


class Map:
    def __init__(self, agent=None, goal=None):
        self.size = 13
        self.grid = [[Cell(x, y) for x in range(self.size)] for y in range(self.size)]

        self.place(agent, agent.x, agent.y)

        self.place(goal, goal.x, goal.y)

    def place_perception(self, enemy):
        if not isinstance(enemy, Enemy):
            return
        for (x, y) in enemy.base_zone():
            cell = self.get_cell(x, y)
            if cell:
                cell.set_danger(True)

    def place(self, character, x, y):
        if character is None:
            return

        if character.x is not None and character.y is not None:
            self.grid[character.x][character.y].change_character(None)

        self.grid[x][y].change_character(character)
        character.x, character.y = x, y

    def get_cell(self, x, y):
        if 0<= x < self.size and 0<= y < self.size:
            return self.grid[x][y]
        return None

    def print_map(self):
        for i in range(self.size):
            row = []
            for j in range(self.size):
                row.append(self.grid[i][j].draw())
            print(" ".join(row))
        print()


if __name__ == "__main__":
    variant = int(input())  # 1 или 2
    gx, gy = map(int, input().split())
    frodo = FrodoAgent(variant, gx, gy)

    step = 0
    while True:
        k = int(input())
        if k == -1:
            break

        print(f"\n=== Step {step}: perception ===")
        frodo.perceive(k)
        # print("Map after perception:")
        # frodo.map.print_map()

        nx, ny = frodo.next_move()
        # frodo.map.print_path_values()
        print(f"Frodo decided to move to: ({nx},{ny})")

        frodo.move(nx, ny)
        print("\nAfter move:")
        frodo.map.print_map()

        step += 1

    # frodo.map.print_map()
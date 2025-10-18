import sys
sys.stdin = open("input.txt", "r", encoding="utf-8")

from abc import ABC, abstractmethod
from heapq import heappush, heappop

# Algorithms
# -----------------------------------------------
class PathfindingStrategy(ABC):
    @abstractmethod
    def find_path(self, map_obj, start, goal):
        pass

class AStarStrategy(PathfindingStrategy):
    def heuristic(self, a, b):
        # Манхэттенская эвристика
        return abs(a.x - b.x) + abs(a.y - b.y)

    def find_path(self, map_obj, agent, goal):
        # Сброс данных
        map_obj.reset_path_data()

        start = map_obj.get_cell(agent.x, agent.y)
        end = map_obj.get_cell(goal.x, goal.y)

        open_list = []

        # Инициализация старта
        start.g = 0
        start.h = self.heuristic(start, end)
        start.f = start.g + start.h
        heappush(open_list, (start.f, start))

        # Основной цикл
        while open_list:
            # 1. Берем ячейку с наименьшим f (если равны — меньший h)
            current_f, current = heappop(open_list)
            if current.is_closed:
                continue

            current.is_closed = True

            # 2. Проверка цели
            if current.x == end.x and current.y == end.y:
                # Восстановление пути: идем назад по parent
                path = []
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.reverse()
                # Следующая ячейка после старта — ответ
                next_cell = path[0] if path else end
                return next_cell.x, next_cell.y

            # 3. Перебираем соседей
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = current.x + dx, current.y + dy
                neighbor = map_obj.get_cell(nx, ny)

                if not neighbor or neighbor.is_closed:
                    continue

                # если это не цель, то проверяем проходимость
                if (nx, ny) != (end.x, end.y) and not neighbor.is_walkable():
                    continue

                tentative_g = current.g + 1

                if tentative_g < neighbor.g or neighbor.parent is None:
                    neighbor.g = tentative_g
                    neighbor.h = self.heuristic(neighbor, end)
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.parent = current
                    heappush(open_list, (neighbor.f, neighbor))

        return agent.x, agent.y
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
            row, col, symbol = input().split()
            row, col = int(row), int(col)
            obj = choose_type(symbol, col, row)
            if obj:
                self.map.place(obj, col, row)
                self.map.place_perception(obj)
            elif symbol == 'P':
                self.map.grid[col][row].set_danger(True)


    def next_move(self):
        nx, ny = self.pathfinder.find_path(self.map, self, self.goal)
        return nx, ny

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

        self.g = 0
        self.h = 0
        self.f = self.g + self.h
        self.parent = None
        self.is_closed = False

    # --- планирование ---
    def is_walkable(self):
        if self.danger or isinstance(self.character, Enemy):
            return False
        return True

    def reset_path(self):
        self.g = 0
        self.h = 0
        self.f = self.g + self.h
        self.parent = None
        self.is_closed = False

    def change_character(self, new_character=None):
        if isinstance(new_character, FrodoAgent):
            self.change_visited()
        self.character = new_character

    def change_visited(self):
        self.visited = True

    def set_danger(self, value=True):
        self.danger = value

    def __lt__(self, other):
        # если f равны, выбираем по h (ближе к цели)
        if self.f == other.f:
            return self.h < other.h
        return self.f < other.f

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
        self.grid = [[Cell(i, j) for j in range(self.size)] for i in range(self.size)]

        self.place(agent, agent.x, agent.y)

        self.place(goal, goal.x, goal.y)

    def place_perception(self, enemy):
        if not isinstance(enemy, Enemy):
            return
        for (x, y) in enemy.base_zone():
            cell = self.get_cell(x, y)
            if cell:
                cell.set_danger(True)

    # --- в Map ---
    def reset_path_data(self):
        for i in range(self.size):
            for j in range(self.size):
                self.grid[i][j].reset_path()

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
        for y in range(self.size):
            row = []
            for x in range(self.size):
                row.append(self.grid[x][y].draw())
            print(" ".join(row))
        print()


if __name__ == "__main__":
    variant = int(input())  # 1 или 2
    gx, gy = map(int, input().split())
    frodo = FrodoAgent(variant, gy, gx)

    step = 0
    while True:
        k = int(input())
        if k == -1:
            break

        print(f"\n=== Step {step}: perception ===")
        frodo.perceive(k)

        nx, ny = frodo.next_move()
        print(f"Frodo decided to move to: ({ny},{nx})")

        frodo.move(nx, ny)
        print("\nAfter move:")
        frodo.map.print_map()

        step += 1
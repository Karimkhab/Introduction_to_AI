import sys
sys.stdin = open("/Users/karimkhabib/Documents/Innopolis/Second course (B25 - B26)/Introduction_to_AI/Test/input.txt", "r", encoding="utf-8")

from abc import ABC, abstractmethod
from heapq import heappush, heappop

class Colors:
    RESET = '\033[0m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    GREY = '\033[90m'
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
                return next_cell.x, next_cell.y, len(path)

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

        return agent.x, agent.y, -1
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
        self.allowed = False

        if self.allowed:
            self.state = False

    def change_state(self, state):
        if self.allowed:
            self.state = state
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
        super().__init__("Watchtower", x, y)
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
        self.radius = radius
        self.goal = Gollum(gollum_x, gollum_y)
        self.pathfinder = AStarStrategy()

        # Кольцо и состояние
        self.has_mithril = False
        self.ring_on = False

        # Две карты
        self.map_off = Map(agent=self, goal=self.goal, ring_on=False)
        self.map_on = Map(agent=self, goal=self.goal, ring_on=True)

    # --- сенсорика ---
    def perceive(self):
        k = int(input())

        def choose_type(symbol, x=None, y=None):
            mapping = {
                'O': OrcPatrol, 'U': UrukHai, 'N': Nazgul, 'W': MordorWatchtower,
                'G': Gollum, 'M': MountDoom, 'C': MithrilMail, 'R': OneRing,
            }
            cls = mapping.get(symbol)
            return cls(x, y) if cls else None

        for _ in range(k):
            line = input().strip()
            row, col, symbol = line.split()
            row, col = int(row), int(col)
            obj = choose_type(symbol, col, row)
            if obj:
                # ставим объект на обе карты
                self.map_off.place(obj, col, row)
                self.map_off.place_perception(obj)
                self.map_on.place(obj, col, row)
                self.map_on.place_perception(obj)
            elif symbol == 'P':
                self.map_off.grid[col][row].set_danger(True)
                self.map_on.grid[col][row].set_danger(True)

        # проверка на Галллума
        if ((self.x, self.y) == (self.goal.x, self.goal.y)) and isinstance(self.goal, Gollum):
            line = input().strip()
            if line.startswith("My precious! Mount Doom is"):
                parts = line.split()
                row, col = int(parts[-2]), int(parts[-1])
                self.goal = MountDoom(col, row)
                self.map_off.place(self.goal, col, row)
                self.map_on.place(self.goal, col, row)
                print(f"Mount Doom revealed at ({row}, {col})")

    def next_move(self):
        # считаем путь на обеих картах
        x_on, y_on, len_on = self.pathfinder.find_path(self.map_on, self, self.goal)
        x_off, y_off, len_off = self.pathfinder.find_path(self.map_off, self, self.goal)

        # если оба пути не найдены
        if len_on == -1 and len_off == -1:
            return self.x, self.y

        # текущий state
        current_state = "on" if self.ring_on else "off"

        # логика выбора
        if self.ring_on:
            if len_off != -1 and (len_off < len_on):
                print(">>> Decided to REMOVE Ring (rr)")
                self.ring_on = False
                print("rr")
                return x_off, y_off
            else:
                return x_on, y_on
        else:
            if len_on != -1 and (len_on < len_off):
                print(">>> Decided to WEAR Ring (r)")
                self.ring_on = True
                print("r")
                return x_on, y_on
            else:
                return x_off, y_off

    def move(self, nx, ny):
        self.map_off.place(self, nx, ny)
        self.map_on.place(self, nx, ny)
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


    def draw(self):
        if self.character:
            name = self.character.name[0]
            if name == 'F':  # Frodo
                return f"{Colors.GREEN}{name}{Colors.RESET}"
            if name == 'G' or name == 'M':  # Frodo
                return f"{Colors.CYAN}{name}{Colors.RESET}"
            else:  # Gollum
                return f"{Colors.RED}{name}{Colors.RESET}"
        if self.danger:
            return f"{Colors.YELLOW}{'#'}{Colors.RESET}"
        if self.visited:
            return f"{Colors.BLUE}{'*'}{Colors.RESET}"
        return '.'

    def is_empty(self):
        return self.character is None


class Map:
    def __init__(self, agent=None, goal=None, ring_on=False):
        self.size = 13
        self.grid = [[Cell(i, j) for j in range(self.size)] for i in range(self.size)]
        self.ring_on = ring_on
        if agent:
            self.place(agent, agent.x, agent.y)
        if goal:
            self.place(goal, goal.x, goal.y)

    def place_perception(self, enemy):
        if not isinstance(enemy, Enemy):
            return
        for (x, y) in enemy.base_zone():
            cell = self.get_cell(x, y)
            if not cell:
                continue
            # логика зависит от состояния кольца:
            if self.ring_on:
                # Orc/Uruk становятся безопаснее, Nazgul/Watchtower опаснее
                if isinstance(enemy, (OrcPatrol, UrukHai)):
                    continue  # уменьшаем зону
                else:
                    cell.set_danger(True)
            else:
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
            print("  ".join(row))
        print()




if __name__ == "__main__":
    # === 1. Инициализация ===
    variant = int(input())  # perception radius
    g_row, g_col = map(int, input().split())
    frodo = FrodoAgent(variant, g_col, g_row)

    step = 0
    print(f"\n=== Initialized: perception radius = {variant}, goal(Gollum) = ({g_row},{g_col}) ===")

    # === 2. Главный цикл ===
    while True:
        print(f"\n=== Step {step}: perception ===")
        frodo.perceive()

        nx, ny = frodo.next_move()
        print(f"Frodo decided to move to: ({ny}, {nx})")

        # Здесь ты бы послал интерактору:
        # print(f"m {ny} {nx}")  # формат интерактора (row, col)
        # sys.stdout.flush()
        # а потом он вернёт новые данные, и цикл продолжится

        frodo.move(nx, ny)

        print("\nAfter move:")
        frodo.map_off.print_map()
        frodo.map_on.print_map()

        step += 1
import sys
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
        return abs(a.x - b.x) + abs(a.y - b.y)

    def find_path(self, map_obj, agent, goal):
        map_obj.reset_path_data()
        start = map_obj.get_cell(agent.x, agent.y)
        end = map_obj.get_cell(goal.x, goal.y)
        open_list = []

        start.g = 0
        start.h = self.heuristic(start, end)
        start.f = start.g + start.h
        heappush(open_list, (start.f, start))

        while open_list:
            current_f, current = heappop(open_list)
            if current.is_closed:
                continue
            current.is_closed = True

            if current.x == end.x and current.y == end.y:
                path = []
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.reverse()
                next_cell = path[0] if path else end
                return next_cell.x, next_cell.y, len(path)

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = current.x + dx, current.y + dy
                neighbor = map_obj.get_cell(nx, ny)
                if not neighbor or neighbor.is_closed:
                    continue
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

# Goal Characters
# -----------------------------------------------
class Gollum(Character):
    def __init__(self, x=None, y=None):
        super().__init__("Gollum", x, y)


class MountDoom(Character):
    def __init__(self, x=None, y=None):
        super().__init__("Mount Doom", x, y)


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
        self.f = 0
        self.parent = None
        self.is_closed = False

    def is_walkable(self):
        if self.danger or isinstance(self.character, Enemy):
            return False
        return True

    def reset_path(self):
        self.g = 0
        self.h = 0
        self.f = 0
        self.parent = None
        self.is_closed = False

    def change_character(self, new_character=None):
        if isinstance(new_character, FrodoAgent):
            self.visited = True
        self.character = new_character

    def set_danger(self, value=True):
        self.danger = value

    def __lt__(self, other):
        if self.f == other.f:
            return self.h < other.h
        return self.f < other.f


class Map:
    def __init__(self, agent=None, goal=None, ring_on=False):
        self.size = 13
        self.grid = [[Cell(i, j) for j in range(self.size)] for i in range(self.size)]
        if agent:
            self.place(agent, agent.x, agent.y)
        if goal:
            self.place(goal, goal.x, goal.y)

    def update_danger_zones(self, ring_on, has_mithril):
        for i in range(self.size):
            for j in range(self.size):
                self.grid[i][j].set_danger(False)

        for i in range(self.size):
            for j in range(self.size):
                cell = self.grid[i][j]
                if isinstance(cell.character, Enemy):
                    danger_zone = self.calculate_current_danger_zone(cell.character, ring_on, has_mithril)
                    for (x, y) in danger_zone:
                        danger_cell = self.get_cell(x, y)
                        if danger_cell:
                            danger_cell.set_danger(True)

    def calculate_current_danger_zone(self, enemy, ring_on, has_mithril):
        base_cell = (enemy.x, enemy.y)

        if isinstance(enemy, (OrcPatrol, UrukHai)):
            if ring_on or has_mithril:
                return [base_cell]
            else:
                return enemy.base_zone() + [base_cell]
        elif isinstance(enemy, (Nazgul, MordorWatchtower)):
            if ring_on:
                zone = []
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        if max(abs(dx), abs(dy)) <= 2:
                            zone.append((enemy.x + dx, enemy.y + dy))
                return zone
            else:
                return enemy.base_zone() + [base_cell]
        return enemy.base_zone() + [base_cell]

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
        if 0 <= x < self.size and 0 <= y < self.size:
            return self.grid[x][y]
        return None


class FrodoAgent(Character):
    def __init__(self, radius, gollum_x, gollum_y):
        super().__init__("Frodo", 0, 0)
        self.radius = radius
        self.gollum = Gollum(gollum_x, gollum_y)
        self.mount_doom = None
        self.ring_on = False
        self.has_mithril = False
        self.gollum_found = False
        self.mission_complete = False
        self.steps_to_gollum = 0
        self.total_steps = 0

        self.map_off = Map(agent=self, goal=self.gollum, ring_on=False)
        self.map_on = Map(agent=self, goal=self.gollum, ring_on=True)
        self.pathfinder = AStarStrategy()

    def process_perception(self):
        try:
            first_line = input().strip()
            if first_line.startswith("My precious!"):
                parts = first_line.split()
                x, y = int(parts[-2]), int(parts[-1])
                self.mount_doom = MountDoom(x, y)
                self.map_off.place(self.mount_doom, x, y)
                self.map_on.place(self.mount_doom, x, y)
                self.gollum_found = True
                if self.steps_to_gollum == 0:
                    self.steps_to_gollum = self.total_steps
                return 0
            return int(first_line)
        except EOFError:
            self.mission_complete = True
            return -1

    def perceive(self, k):
        if k <= 0:
            return

        def choose_type(symbol, x=None, y=None):
            mapping = {
                'O': OrcPatrol, 'U': UrukHai, 'N': Nazgul, 'W': MordorWatchtower,
                'G': Gollum, 'M': MountDoom, 'C': MithrilMail, 'R': OneRing,
            }
            cls = mapping.get(symbol)
            return cls(x, y) if cls else None

        for _ in range(k):
            line = input().strip()
            parts = line.split()
            if len(parts) < 3:
                continue

            row, col, symbol = parts[0], parts[1], parts[2]
            row, col = int(row), int(col)
            obj = choose_type(symbol, col, row)

            if obj:
                self.map_off.place(obj, col, row)
                self.map_off.update_danger_zones(False, self.has_mithril)
                self.map_on.place(obj, col, row)
                self.map_on.update_danger_zones(True, self.has_mithril)

                if isinstance(obj, Gollum) and (self.x, self.y) == (obj.x, obj.y) and not self.gollum_found:
                    self.gollum_found = True
                    self.steps_to_gollum = self.total_steps

    def should_toggle_ring(self, next_x, next_y):
        current_goal = self.mount_doom if self.gollum_found else self.gollum
        path_off = self.find_path_on_map(self.map_off, current_goal)
        path_on = self.find_path_on_map(self.map_on, current_goal)

        if path_off is None and path_on is not None and not self.ring_on:
            return True
        if path_on is None and path_off is not None and self.ring_on:
            return True

        if path_off and path_on:
            len_off = len(path_off)
            len_on = len(path_on)
            threshold = 3

            if not self.ring_on and len_on < (len_off - threshold):
                return True
            if self.ring_on and len_off < (len_on - threshold):
                return True

        target_cell_off = self.map_off.get_cell(next_x, next_y)
        target_cell_on = self.map_on.get_cell(next_x, next_y)

        if (target_cell_off and target_cell_off.danger and
                target_cell_on and not target_cell_on.danger and
                not self.ring_on):
            return True

        if (target_cell_on and target_cell_on.danger and
                target_cell_off and not target_cell_off.danger and
                self.ring_on):
            return True

        return False

    def find_path_on_map(self, map_obj, goal):
        try:
            nx, ny, path_len = self.pathfinder.find_path(map_obj, self, goal)
            if path_len != -1:
                return [(nx, ny)]
            return None
        except:
            return None

    def next_move(self):
        current_goal = self.mount_doom if self.gollum_found else self.gollum

        if self.ring_on:
            current_map = self.map_on
        else:
            current_map = self.map_off

        nx, ny, path_len = self.pathfinder.find_path(current_map, self, current_goal)

        if path_len == -1:
            return "e -1"

        if self.should_toggle_ring(nx, ny):
            return "rr" if self.ring_on else "r"

        if (nx, ny) == (current_goal.x, current_goal.y):
            if isinstance(current_goal, MountDoom):
                total_path = self.calculate_total_path_length()
                return f"e {total_path}"
            return f"m {ny} {nx}"

        return f"m {ny} {nx}"

    def calculate_total_path_length(self):
        # Вычисляем путь до Голлума + путь от Голлума до Горы
        if self.steps_to_gollum == 0:
            self.steps_to_gollum = self.total_steps

        # Путь от Голлума до Горы вычисляем через A*
        if self.gollum_found and self.mount_doom:
            start = self.map_off.get_cell(self.gollum.x, self.gollum.y)
            end = self.map_off.get_cell(self.mount_doom.x, self.mount_doom.y)
            _, _, path_len = self.pathfinder.find_path(self.map_off, start, end)
            if path_len != -1:
                return self.steps_to_gollum + path_len

        return self.total_steps  # fallback

    def execute_action(self, action):
        if action.startswith("m"):
            parts = action.split()
            x, y = int(parts[2]), int(parts[1])
            self.move(x, y)
            self.total_steps += 1
        elif action == "r":
            self.ring_on = True
        elif action == "rr":
            self.ring_on = False
        elif action.startswith("e"):
            self.mission_complete = True

    def move(self, x, y):
        self.map_off.place(self, x, y)
        self.map_on.place(self, x, y)
        self.x, self.y = x, y

        current_cell_off = self.map_off.get_cell(x, y)
        if current_cell_off and isinstance(current_cell_off.character, MithrilMail):
            self.has_mithril = True


if __name__ == "__main__":
    # ⚠️ УБРАТЬ ЭТУ СТРОКУ ДЛЯ CODEFORCES!
    # sys.stdin = open("input.txt", "r", encoding="utf-8")

    variant = int(input().strip())
    gx, gy = map(int, input().split())
    frodo = FrodoAgent(variant, gx, gy)

    k = frodo.process_perception()
    if k > 0:
        frodo.perceive(k)

    while not frodo.mission_complete:
        action = frodo.next_move()
        print(action)
        sys.stdout.flush()

        frodo.execute_action(action)

        k = frodo.process_perception()
        if k > 0:
            frodo.perceive(k)
from heapq import heappush, heappop
import sys
import select

"""Abstract base class for pathfinding algorithms"""
class PathfindingStrategy():
    def find_path(self, map_obj, start, goal):
        """Finds path from start to goal on the map"""
        pass

class Node:
    """Helper class that stores Cell reference and pathfinding data"""
    def __init__(self, cell):
        self.cell = cell          # reference to map Cell
        self.g = float("inf")     # cost from start
        self.h = 0                # heuristic cost to goal
        self.f = 0                # total cost
        self.parent = None        # previous Node in path
        self.closed = False       # visited flag

    def __lt__(self, other):
        # for heapq priority queue
        if self.f == other.f:
            return self.h < other.h
        return self.f < other.f
"""A* pathfinding algorithm implementation for finding shortest paths"""
class AStarStrategy(PathfindingStrategy):
    def heuristic(self, a, b):
        """Calculates Manhattan distance between two cells for heuristic"""
        return abs(a.x - b.x) + abs(a.y - b.y)

    def find_path(self, map_obj, agent, goal):
        """Finds shortest path from agent to goal using A* algorithm"""
        map_obj.reset_path_data()

        start_cell = map_obj.get_cell(agent.x, agent.y)
        end_cell = map_obj.get_cell(goal.x, goal.y)

        # создаём словарь Node для всех клеток (чтобы хранить состояние поиска)
        node_map = {(x, y): Node(map_obj.grid[x][y])
                    for x in range(map_obj.size)
                    for y in range(map_obj.size)}

        start = node_map[(start_cell.x, start_cell.y)]
        end = node_map[(end_cell.x, end_cell.y)]

        open_list = []
        start.g = 0
        start.h = self.heuristic(start.cell, end.cell)
        start.f = start.g + start.h
        heappush(open_list, (start.f, start))

        while open_list:
            current_f, current = heappop(open_list)
            if current.closed:
                continue
            current.closed = True

            # Check if reached goal
            if current.cell.x == end.cell.x and current.cell.y == end.cell.y:
                path = []
                while current.parent:
                    path.append(current.cell)
                    current = current.parent
                path.reverse()
                next_cell = path[0] if path else end.cell
                return next_cell.x, next_cell.y

            # Explore 4 directions
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = current.cell.x + dx, current.cell.y + dy
                if not (0 <= nx < map_obj.size and 0 <= ny < map_obj.size):
                    continue

                neighbor = node_map[(nx, ny)]
                ncell = neighbor.cell
                if neighbor.closed:
                    continue
                # allow goal even if not walkable
                if (nx, ny) != (end.cell.x, end.cell.y) and not ncell.is_walkable():
                    continue

                tentative_g = current.g + 1
                if tentative_g < neighbor.g or neighbor.parent is None:
                    neighbor.g = tentative_g
                    neighbor.h = self.heuristic(ncell, end.cell)
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.parent = current
                    heappush(open_list, (neighbor.f, neighbor))

        return agent.x, agent.y


"""Base class for all characters in the game world"""
class Object:
    def __init__(self, name="Unknown", x=None, y=None):
        self.name = name
        self.x = x
        self.y = y


class PerceptionZoneEnemy(Object):
    def __init__(self, x=None, y=None):
        super().__init__("Perception zone of enemy", x, y)

"""Base class for all collectible items in the game"""
class Item(Object):
    def __init__(self,name="Unknown", x=None, y=None):
        super().__init__(name, x, y)
        self.has_agent = False
    def change_has(self):
        self.has_agent = True

"""Mithril armor item that provides protection to Frodo"""
class MithrilMail(Item):
    def __init__(self, x=None, y=None):
        super().__init__("Mithril", x, y)

"""The One Ring item that makes Frodo invisible when worn"""
class OneRing(Item):
    def __init__(self, x=None, y=None):
        super().__init__("Ring", x, y)


"""Base class for all enemy types with detection zones"""
class Enemy(Object):
    def __init__(self, name, x=None, y=None):
        super().__init__(name, x, y)
        self.dirs = []  # Directions that define detection zone

    def base_zone(self):
        """Returns coordinates of all cells in enemy's detection zone"""
        zone = []
        for dx, dy in self.dirs:
            zone.append((self.x + dx, self.y + dy))
        return zone


"""Orc enemy that can detect Frodo in 4 adjacent cells"""
class OrcPatrol(Enemy):
    def __init__(self, x=None, y=None):
        super().__init__("Orc Patrol", x, y)
        self.dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, down, left, right


"""Uruk-hai enemy that can detect Frodo in cells within 2 steps"""
class UrukHai(Enemy):
    def __init__(self, x=None, y=None):
        super().__init__("Uruk-Hai", x, y)
        # Von Neumann neighborhood of radius 2
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if abs(dx) + abs(dy) <= 2 and not (dx == 0 and dy == 0):
                    self.dirs.append((dx, dy))


"""Nazgul enemy that can detect in 8 directions plus extended corners"""
class Nazgul(Enemy):
    def __init__(self, x=None, y=None):
        super().__init__("Nazgul", x, y)
        # Moore neighborhood of radius 1 plus extended corners
        self.dirs = [(dx, dy) for dx in range(-1, 2) for dy in range(-1, 2) if not (dx == 0 and dy == 0)]
        self.dirs += [(2, 2), (-2, 2), (2, -2), (-2, -2)]


"""Watchtower enemy that can detect in all directions within 2 cells"""
class MordorWatchtower(Enemy):
    def __init__(self, x=None, y=None):
        super().__init__("Watchtower", x, y)
        # Moore neighborhood of radius 2
        self.dirs = [(dx, dy) for dx in range(-2, 3) for dy in range(-2, 3) if not (dx == 0 and dy == 0)]


# -----------------------------------------------

# Goal Characters
# -----------------------------------------------
class Goal(Object):
    pass

"""Gollum Object that guides Frodo to Mount Doom"""
class Gollum(Goal):
    def __init__(self, x=None, y=None):
        super().__init__("Gollum", x, y)


"""Mount Doom - the final destination where Frodo must destroy the Ring"""
class MountDoom(Goal):
    def __init__(self, x=None, y=None):
        super().__init__("Mount Doom", x, y)


# -----------------------------------------------

"""Represents a single cell in the 13x13 game map"""
class Cell:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.object = None  # who occupies this cell now

        # must be a tuple for isinstance(...)
        self.__danger_objects = (
            PerceptionZoneEnemy, OrcPatrol, UrukHai, Nazgul, MordorWatchtower
        )
        self.danger = False  # this cell was observed as dangerous (P)

    def change_object(self, new_object=None):
        """Updates which character occupies this cell"""
        # mark as dangerous only when we place P or an enemy directly
        if isinstance(new_object, self.__danger_objects):
            self.danger = True
        self.object = new_object

    def is_walkable(self):
        """
        Temporary simple rule:
        - not walkable if there is an enemy on the cell
        - not walkable if it was observed as 'P' (danger) now
        - goals/items are walkable
        """
        if isinstance(self.object, (OrcPatrol, UrukHai, Nazgul, MordorWatchtower)):
            return False
        if self.danger:
            return False
        return True



"""
Class Map Environment
The map is a 13*13 grid, indexed by numbers in range of [0, 12] representing the world of Lord of the Rings.
"""
class Map:
    def __init__(self, agent):
        self.size = 13
        self.grid = [[Cell(i, j) for j in range(self.size)] for i in range(self.size)]

        self.agent = agent
        self.update_agent_position(agent.x, agent.y)
        self.objects = []  # All objects on the map

    def reset_path_data(self):
        """Resets temporary pathfinding data (used by algorithms like A*)"""
        # no persistent state yet, but this keeps compatibility
        pass

    def get_map_state(self, ring_on, has_mithril):
        """Returns map possibly updated for current Ring/Mithril state"""
        # если кольцо надето — временно игнорируем опасные зоны
        if ring_on:
            for row in self.grid:
                for cell in row:
                    cell.danger = False
        # если есть мифрил — тоже ослабляем восприятие опасности
        elif has_mithril:
            for row in self.grid:
                for cell in row:
                    if cell.danger:
                        cell.danger = False
        return self

    def place(self, x, y, object=None):
        """Places a game object at specified coordinates"""
        cell = self.grid[x][y]

        cell.change_object(object)
        self.objects.append(object)

    def update_agent_position(self, new_x, new_y):
        """Moves Frodo to new coordinates on the map"""
        # Clear old position
        old_cell = self.get_cell(self.agent.x, self.agent.y)
        if old_cell and old_cell.object == self.agent:
            old_cell.change_object()

        # Set new position
        new_cell = self.get_cell(new_x, new_y)
        if new_cell:
            new_cell.change_object(self.agent)


    def get_cell(self, x, y):
        """Returns cell at coordinates, or None if out of bounds"""
        if 0 <= x < self.size and 0 <= y < self.size:
            cell = self.grid[x][y]
            return cell
        return None


class FrodoAgent(Object):
    """
    Class FrodoAgent represents the Frodo (Baggins)
    Agent starts from the safe cell (0, 0)
    Goal is to find the Ghollum
    """
    def __init__(self, radius, gollum_x, gollum_y):
        super().__init__("Frodo", 0, 0)
        self.radius = radius
        self.mithril = MithrilMail()  # Does Frodo have a Mithril Mail-coat?
        self.ring = OneRing()  #  Does Frodo have The One Ring?

        self.total_path = 0  # Total number of moves
        
        
        self.gollum_found = False  # Has Frodo found Gollum?

        # Goals
        self.goal = Gollum(gollum_x, gollum_y)  # Current goal

        self.map = Map(self) # Map
        self.map.place(self.goal.x, self.goal.y, self.goal)
        self.pathfinder = AStarStrategy() # navigation

    def run_agent(self):
        """Main game loop that runs for up to 200 turns"""
        while True:
            try:
                # Read number of perception inputs
                count = int(input())

                self.__perceive(count)

                if self.__check_goal():
                    break

                # Checking if the goal has been completed


                # Decide and execute next action
                action = self.__decide_next_action()
                self.__execute_action(action)

            except:
                # Handle errors and exit
                print("e -1", flush=True)
                return

    def __perceive(self, count):
        """Method goes through all perceptions that interactor has sent"""
        for _ in range(count):
            perception = input().split()
            x, y, symbol = int(perception[0]), int(perception[1]), perception[2]

            self.__explore_perception(x, y, symbol) # Process new information

    def __explore_perception(self, x, y, symbol):
        """Processes one perceived object and adds it to the map"""

        def create_object(symbol):
            """Creates game object from symbol character"""
            if symbol == 'P': return PerceptionZoneEnemy(x, y)
            if symbol == 'O': return OrcPatrol(x, y)
            if symbol == 'U': return UrukHai(x, y)
            if symbol == 'N': return Nazgul(x, y)
            if symbol == 'W': return MordorWatchtower(x, y)
            if symbol == 'C': return MithrilMail(x, y)

            if symbol == 'M': return MountDoom(x, y)
            if symbol == 'G': return Gollum(x, y)
            return None

        object = create_object(symbol)
        if object:
            self.map.place(x, y, object)



    def __decide_next_action(self):
        """Decides what action Frodo should take next"""
        # Get current map state with updated danger zones
        current_map = self.map.get_map_state(self.ring.has_agent, self.mithril.has_agent)

        # Find path to current goal
        next_x, next_y = self.pathfinder.find_path(current_map, self, self.goal)

        # If no movement possible, try toggling the Ring
        if (next_x, next_y) == (self.x, self.y):
            # если реально в тупике (нет пути), только тогда пробуем кольцо
            if not self.ring.has_agent:
                return "r"
            elif self.ring.has_agent:
                return "rr"

        # Otherwise move to next cell
        return f"m {next_x} {next_y}"

    def __execute_action(self, action):
        """Executes the chosen action and updates game state"""
        print(action, flush=True)

        if action.startswith("m"):
            _, x, y = action.split()
            self.__move_to(int(x), int(y))


        elif action in ("r", "rr"):
            self.total_path += 1
            # запрет на повторное включение/выключение — иначе "failed test"

            if (action == "r" and self.ring.has_agent) or (action == "rr" and not self.ring.has_agent):
                print("e -1", flush=True)

                return

            self.ring.has_agent = (action == "r")

            # ОБЯЗАТЕЛЬНО читаем следующий блок восприятий

            count = int(input())

            self.__perceive(count)
            self.map = self.map.get_map_state(self.ring.has_agent, self.mithril.has_agent)


    def __move_to(self, x, y):
        """Moves Frodo to new coordinates and updates path length"""
        self.map.update_agent_position(x, y)
        self.x, self.y = x, y
        self.total_path += 1

        cell = self.map.get_cell(x, y)
        if isinstance(cell.object, MithrilMail):
            self.mithril.has_agent = True

    def __check_goal(self):
        # Если цель — ГОЛЛУМ, и мы пришли на его координаты, читаем координаты Ородруина
        if isinstance(self.goal, Gollum) and (self.x, self.y) == (self.goal.x, self.goal.y):
            line = input().strip()
            nums = [int(tok) for tok in line.split() if tok.lstrip('-').isdigit()]
            x, y = nums[-2], nums[-1]
            self.goal = MountDoom(x, y)
            self.map.place(x, y, self.goal)
            self.total_path += 1  # <── добавь эту строку
            return False

        # Если цель — MOUNT DOOM и мы на его координатах — финиш
        if isinstance(self.goal, MountDoom) and (self.x, self.y) == (self.goal.x, self.goal.y):
            # интерактор может прислать последнее восприятие (0)
            if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                try:
                    count = int(input().strip())
                    for _ in range(count):
                        _ = input().strip()
                except:
                    pass
            print(f"e {self.total_path}", flush=True)
            return True

        return False


if __name__ == "__main__":
    # Read initial game parameters
    variant = int(input())
    gx, gy = map(int, input().split())

    # Create and run Frodo agent
    frodo = FrodoAgent(variant, gx, gy)
    frodo.run_agent()
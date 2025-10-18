# environment.py

class Environment:
    def __init__(self, size=13):
        self.size = size
        self.grid = [[None for _ in range(size)] for _ in range(size)]
        self.classes = {}

    def register_classes(self, mapping):
        """Позволяет зарегистрировать классы персонажей"""
        self.classes = mapping

    def add_object(self, obj):
        self.grid[obj.x][obj.y] = obj

    def encode(self, obj):
        for cls, symbol in self.classes.items():
            if isinstance(obj, cls):
                return symbol
        return '.'

    def get_percepts(self, agent):
        percepts = []
        r = agent.radius
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = agent.x + dx, agent.y + dy
                if 0 <= nx < self.size and 0 <= ny < self.size:
                    obj = self.grid[nx][ny]
                    if obj:
                        percepts.append((nx, ny, self.encode(obj)))
        return percepts

    def print_real_map(self):
        print("=== REAL WORLD ===")
        for y in range(self.size):
            row = []
            for x in range(self.size):
                obj = self.grid[x][y]
                if obj is None:
                    row.append('.')
                else:
                    row.append(self.encode(obj))
            print(" ".join(row))
        print()
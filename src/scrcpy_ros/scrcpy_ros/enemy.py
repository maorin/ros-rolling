import math

class Enemy:
    def __init__(self, name, x, y, threat_level):
        self.name = name
        self.x = x
        self.y = y
        self.threat_level = threat_level

    def distance_to_player(self, player_x, player_y):
        return math.sqrt((self.x - player_x) ** 2 + (self.y - player_y) ** 2)

    def __repr__(self):
        return f"{self.name} (x: {self.x}, y: {self.y}, threat_level: {self.threat_level})"


def prioritize_enemies(enemies, player_x, player_y, distance_weight=0.5, threat_weight=0.5):
    def enemy_priority(enemy):
        distance = enemy.distance_to_player(player_x, player_y)
        threat = enemy.threat_level
        return distance_weight / distance + threat_weight * threat

    return sorted(enemies, key=enemy_priority, reverse=True)


if __name__ == "__main__":
    player_x, player_y = 0, 0

    enemies = [
        Enemy("Enemy A", 10, 20, 1),
        Enemy("Enemy B", 30, 40, 3),
        Enemy("Enemy C", 50, 60, 2),
    ]

    prioritized_enemies = prioritize_enemies(enemies, player_x, player_y)

    for enemy in prioritized_enemies:
        print(enemy)

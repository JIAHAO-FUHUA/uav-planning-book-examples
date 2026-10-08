from planning import (static_scene, dynamic_scene, search,
                      shortcut_path, length, DStarLite)

grid, start, goal = static_scene()
for name, weight, theta in [
    ("Dijkstra", 0, False), ("A*", 1, False),
    ("Weighted A*", 2, False), ("Theta*", 1, True)
]:
    result = search(grid, start, goal, weight, theta)
    print(name, round(result.cost, 3), len(result.expanded))
astar = search(grid, start, goal)
print("Shortcut", round(length(shortcut_path(grid,
                                              astar.path)), 3))

grid, start, goal = dynamic_scene()
planner = DStarLite(grid, start, goal)
planner.compute()
planner.move_start((18, 8))
events = [
    ("Close", [((24, y), True) for y in range(6, 10)]),
    ("Reopen", [((24, y), False) for y in range(6, 10)]),
    ("Remote", [((4, 28), True)])
]
for label, changes in events:
    planner.update_cells(changes)
    repaired = planner.compute()
    fresh = search(grid, (18, 8), goal)
    assert abs(repaired.cost - fresh.cost) < 1e-8
    print(label, round(repaired.cost, 3),
          len(repaired.expanded), len(fresh.expanded))

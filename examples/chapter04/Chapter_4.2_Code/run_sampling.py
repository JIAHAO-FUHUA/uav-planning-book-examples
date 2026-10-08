from sampling_dynamics import Scene, prm, rrt, kinodynamic_search

scene = Scene()
start, goal = (1.0, 3.0), (13.0, 3.0)
for result in [prm(scene, start, goal),
               rrt(scene, start, goal),
               rrt(scene, start, goal, star=True)]:
    print(result.algorithm, result.cost, bool(result.path))
result = kinodynamic_search(scene, (*start, 2.0, 0.0),
                            (*goal, 0.0, 0.0))
print(result.duration, result.cost, result.expanded)

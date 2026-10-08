from simulations import moving_replanning, Scene3D, rrt3d

for event in moving_replanning()["snapshots"]:
    print(event["step"], event["current"], event["status"],
          event["remaining_cost"])
result = rrt3d(Scene3D(), seed=17, budget=2000)
print("3D length", result.cost, "nodes", len(result.nodes))
sealed = Scene3D(high=(14.0, 10.0, 3.4))
failure = rrt3d(sealed, seed=17, budget=300)
print("Sealed ceiling has a sampled route:", bool(failure.path))

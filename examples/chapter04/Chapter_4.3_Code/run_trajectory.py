from trajectory_generation import build_cases, collision_intervals

cases = build_cases()
for name in ("through", "scaled", "safe_stop"):
    tr = cases[name]
    hit = any(collision_intervals(tr, *cases[key])
              for key in ("box", "upper_box"))
    speed = tr.maximum_norm(1)[0]
    accel = tr.maximum_norm(2)[0]
    accepted = not hit and speed <= 2.0 and accel <= 2.5
    print(name, round(tr.total_time, 3),
          round(speed, 3), round(accel, 3), accepted)

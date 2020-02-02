import pathfinder as pf
import math


points = [
    pf.Waypoint(-4, -1, math.radians(-45.0)),
    pf.Waypoint(-2, -2, 0),
    pf.Waypoint(0, 0, 0)
]

trajectory = pf.generator.generate_trajectory(points, pf.hermite.pf_fit_hermite_cubic, 1000, 0.02, 15.0, 10.0, 60.0)

individual_trajectories = pf.modifiers.tank(trajectory, 1.0)

# Plot the path
import matplotlib.pyplot as plt

mx, my = zip(*[(m.y, m.x) for m in points])
plt.scatter(mx, my, c="r")

# plot the main trajectory
x, y = zip(*[(seg.y, seg.x) for seg in trajectory])
plt.plot(x, y, color="blue")

# annotate with time
for i in range(0, len(trajectory), int(0.5 / 0.02)):
    plt.annotate(
        "t=%.2f" % (i * 0.02,),
        xy=(x[i], y[i]),
        xytext=(-20, 20),
        textcoords="offset points",
        arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0"),
    )

for t in individual_trajectories:
    xt, yt = zip(*[(seg.y, seg.x) for seg in t])
    plt.plot(xt, yt, color="red")

plt.show()

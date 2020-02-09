import pathfinder as pf
import math


points = [pf.Waypoint(0,0,0),
            pf.Waypoint(2, 0, 0)]

trajectory = pf.generator.generate_trajectory(points, pf.hermite.pf_fit_hermite_cubic, pf.SAMPLES_FAST, 0.02, 3.743381, 0.717, 120.0)
left, right = pf.modifiers.tank(trajectory, 1.0)

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
        arrowprops=dict(arrowstyle="-", connectionstyle="arc3,rad=0"),
    )


xl, yl = zip(*[(seg.y, seg.x) for seg in left])
plt.plot(xl, yl, color="red")
for i in range(0, len(left), int(len(left)/10)):
    plt.annotate(
        "v=%.2f" % left[i].velocity,
        xy=(xl[i], yl[i]),
        xytext=(10, -10),
        textcoords="offset points",
        arrowprops=dict(arrowstyle="-", connectionstyle="arc3,rad=0"),
    )

xr, yr = zip(*[(seg.y, seg.x) for seg in right])
plt.plot(xr, yr, color="red")
for i in range(0, len(left), int(len(right)/10)):
    plt.annotate(
        "a=%.2f" % right[i].acceleration,
        xy=(xr[i], yr[i]),
        xytext=(-30, 15),
        textcoords="offset points",
        arrowprops=dict(arrowstyle="-", connectionstyle="arc3,rad=0"),
    )

plt.show()

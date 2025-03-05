import matplotlib.pyplot as plt

waypoints = []

with open("projet_L3/scenarios/16room_000.map.scen", "r") as f:
    i = 2000
    for line in f:
        if(i>300):
            i-=1
            continue
        parts = line.strip().split()
        _, _, _, _, x1, y1, x2, y2, _ = parts
        waypoints.append(((int(x1), int(y1)), (int(x2), int(y2))))

# Plot waypoints
for (x1, y1), (x2, y2) in waypoints:
    plt.plot([x1, x2], [y1, y2], 'ro-')

plt.gca().invert_yaxis()  # Flip Y-axis to match grid conventions
plt.grid(True)
plt.show()

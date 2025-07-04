import json
import plotly.graph_objects as go
import plotly.express as px
from datetime import datetime

# === Load trajectory data ===
with open("trajectory_data.json") as f:
    data = json.load(f)

conflicts = data.pop("conflicts", [])
drone_ids = list(data.keys())
colors = px.colors.qualitative.Bold + px.colors.qualitative.Set1  # Extended color palette

TIME_RESOLUTION = 1.0

# === Interpolation per drone ===
def interpolate(points):
    result = {}
    if len(points) < 2:
        if points:  # Handle single-point case
            t = round(points[0]["timestamp"], 2)
            result[t] = (points[0]["x"], points[0]["y"], points[0]["z"])
        return result
    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i + 1]
        t1, t2 = p1["timestamp"], p2["timestamp"]
        if abs(t2 - t1) < 1e-6:  # Use original points if timestamps are too close
            t = round(t1, 2)
            result[t] = (p1["x"], p1["y"], p1["z"])
            continue
        steps = int((t2 - t1) / TIME_RESOLUTION)
        for s in range(steps + 1):
            t = round(t1 + s * TIME_RESOLUTION, 2)
            ratio = (t - t1) / (t2 - t1)
            x = p1["x"] + ratio * (p2["x"] - p1["x"])
            y = p1["y"] + ratio * (p2["y"] - p1["y"])
            z = p1["z"] + ratio * (p2["z"] - p1["z"])
            result[t] = (x, y, z)
    # Ensure all original points are included
    for p in points:
        t = round(p["timestamp"], 2)
        result[t] = (p["x"], p["y"], p["z"])
    return result

# === Interpolated data
interp_data = {drone: interpolate(path) for drone, path in data.items()}

# Force inclusion of all raw timestamps in all_times
all_raw_times = sorted(set().union(*[set(round(p["timestamp"], 2) for p in path) for path in data.values()]))
all_times = sorted(set().union(*[set(traj.keys()) for traj in interp_data.values()]) | set(all_raw_times))

# === Group conflicts by time
conflict_lookup = {}
for c in conflicts:
    t = round(c["timestamp"], 2)
    conflict_lookup.setdefault(t, []).append(c)

# === Create animation frames ===
frames = []

for current_time in all_times:
    frame_data = []

    for i, drone in enumerate(drone_ids):
        # Past trail
        past_x, past_y, past_z = [], [], []
        last_known_pos = None
        raw_points = [(round(p["timestamp"], 2), (p["x"], p["y"], p["z"])) for p in data[drone]]
        for t in all_times:
            if t > current_time:
                break
            if t in interp_data[drone]:
                x, y, z = interp_data[drone][t]
                past_x.append(x)
                past_y.append(y)
                past_z.append(z)
                last_known_pos = (x, y, z)
            elif raw_points:  # Fallback to nearest raw point
                nearest_t, (x, y, z) = min(raw_points, key=lambda x: abs(x[0] - t))
                past_x.append(x)
                past_y.append(y)
                past_z.append(z)
                if abs(nearest_t - t) < TIME_RESOLUTION or not last_known_pos:
                    last_known_pos = (x, y, z)

        # Draw full trail up to current time
        frame_data.append(go.Scatter3d(
            x=past_x, y=past_y, z=past_z,
            mode='lines',
            line=dict(width=2, color=colors[i % len(colors)]),
            name=drone,
            showlegend=True if current_time == all_times[0] else False
        ))

        # Current position marker
        if last_known_pos:
            x, y, z = last_known_pos
            frame_data.append(go.Scatter3d(
                x=[x], y=[y], z=[z],
                mode='markers+text',
                marker=dict(size=6, color=colors[i % len(colors)]),
                text=[drone],
                textposition="top center",
                showlegend=False
            ))

    # Conflict spheres at current time
    for t in all_times:
        if t > current_time:
            break
        if t in conflict_lookup:
            for c in conflict_lookup[t]:
                time_str = datetime.fromtimestamp(t).strftime("%H:%M:%S")
                drones = ', '.join(c["drones"])
                hover_text = (
                    f"⚠ Conflict<br>Drones: {drones}<br>"
                    f"Time: {time_str}<br>"
                    f"Location: ({c['x']}, {c['y']}, {c['z']})"
                )
                frame_data.append(go.Scatter3d(
                    x=[c["x"]], y=[c["y"]], z=[c["z"]],
                    mode='markers',
                    marker=dict(size=10, color='red'),
                    text=[hover_text],
                    hoverinfo='text',
                    showlegend=False
                ))

    frames.append(go.Frame(data=frame_data, name=str(current_time)))

# === Set up initial frame ===
initial_data = frames[0].data if frames else []

# === Layout and UI ===
layout = go.Layout(
    title="🌍 Drone Airspace Animation with Full Visibility",
    scene=dict(
        xaxis=dict(title="X", showgrid=False),
        yaxis=dict(title="Y", showgrid=False),
        zaxis=dict(title="Z (Altitude)", showgrid=False),
        aspectmode="data"
    ),
    updatemenus=[{
        "type": "buttons",
        "buttons": [dict(label="▶ Play", method="animate", args=[None])]
    }],
    sliders=[{
        "steps": [
            {"method": "animate", "args": [[str(t)]], "label": datetime.fromtimestamp(t).strftime("%H:%M:%S")}
            for t in all_times
        ],
        "transition": {"duration": 0},
        "x": 0.1, "len": 0.8
    }],
    margin=dict(l=0, r=0, t=40, b=0),
    scene_camera=dict(eye=dict(x=2, y=2, z=2))  # Adjusted for better view
)

# === Build and show the figure ===
fig = go.Figure(data=initial_data, layout=layout, frames=frames)
fig.show()
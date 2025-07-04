import json
import plotly.graph_objects as go
import plotly.express as px
from datetime import datetime

# === Load drone trajectories and conflict data from file ===
with open("trajectory_data.json") as f:
    data = json.load(f)

# Separate conflict entries from drone trajectory data
conflicts = data.pop("conflicts", [])
drone_ids = list(data.keys())
colors = px.colors.qualitative.Bold  # Custom palette for visual distinction

# === Initialize the 3D figure ===
fig = go.Figure()

# === Plot each drone's trajectory with relative timestamps ===
for i, (drone_id, points) in enumerate(data.items()):
    x, y, z, t = [], [], [], []
    t0 = points[0]["timestamp"] if points else 0

    for p in points:
        x.append(p["x"])
        y.append(p["y"])
        z.append(p["z"])
        t.append(p["timestamp"] - t0)  # Convert to relative time for display

    fig.add_trace(go.Scatter3d(
        x=x, y=y, z=z,
        mode='lines+markers',
        marker=dict(size=4),
        line=dict(width=3, color=colors[i % len(colors)]),
        name=drone_id,
        hoverinfo='text',
        text=[f"{drone_id}<br>t={ti:.1f}s" for ti in t]
    ))

# === Add conflict markers with rich contextual tooltips ===
for idx, conflict in enumerate(conflicts):
    drones_involved = conflict.get("drones", [conflict.get("drone", "Unknown")])
    drone_names = ', '.join(drones_involved)
    timestamp = conflict["timestamp"]
    time_str = datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")

    hover_text = (
        f"⚠ Conflict {idx + 1}<br>"
        f"Drones: {drone_names}<br>"
        f"Time: {time_str}<br>"
        f"Location: ({conflict['x']:.2f}, {conflict['y']:.2f}, {conflict['z']:.2f})"
    )

    fig.add_trace(go.Scatter3d(
        x=[conflict["x"]],
        y=[conflict["y"]],
        z=[conflict["z"]],
        mode='markers',
        marker=dict(size=10, color='red', symbol='circle'),
        text=[hover_text],
        hoverinfo='text',
        name=f'⚠ Conflict {idx + 1}',
        showlegend=False
    ))

# === Final layout tweaks for visual clarity ===
fig.update_layout(
    title="🛸 3D Drone Trajectories with Conflict Zones",
    margin=dict(l=0, r=0, t=40, b=0),
    scene=dict(
        xaxis=dict(title='X (meters)', showgrid=False, zeroline=False),
        yaxis=dict(title='Y (meters)', showgrid=False, zeroline=False),
        zaxis=dict(title='Altitude (Z)', showgrid=False, zeroline=False),
        aspectmode='data'
    ),
    legend=dict(x=0.01, y=0.99),
    scene_camera=dict(eye=dict(x=1.5, y=1.5, z=1.2))
)

# === Display the interactive 3D plot ===
fig.show()

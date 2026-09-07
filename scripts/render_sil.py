#!/usr/bin/env python3
"""Render recorded Gazebo ground truth; this is a data replay, not simulator camera footage."""

import argparse
import bisect
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Circle, Rectangle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("docs/assets"))
    args = parser.parse_args()
    result = json.loads((args.episode / "result.json").read_text())
    events = [
        json.loads(s) for s in (args.episode / "events.jsonl").read_text().splitlines()
    ]
    positions = {r: [] for r in range(1, result["robots"] + 1)}
    for line in (args.episode / "telemetry.jsonl").read_text().splitlines():
        row = json.loads(line)
        if row["topic"].endswith("/ground_truth/odom"):
            r = int(row["topic"].split("/")[1][5:])
            positions[r].append((row["time"], row["x"], row["y"]))
    start, end = result["mission_start_sim"], result["mission_end_sim"]
    colors = ["#2563eb", "#ea580c", "#059669"]
    fig = plt.figure(figsize=(10, 7), facecolor="#f8fafc", layout="constrained")
    grid = fig.add_gridspec(2, 1, height_ratios=[5, 1])
    ax, timeline = fig.add_subplot(grid[0]), fig.add_subplot(grid[1])
    ax.set(
        xlim=(-4.7, 4.7),
        ylim=(-3.7, 3.7),
        aspect="equal",
        xlabel="World x (m)",
        ylabel="World y (m)",
    )
    ax.set_title(
        "Three-robot warehouse execution\nRecorded Gazebo ground truth · software-in-the-loop",
        loc="left",
        fontsize=14,
        weight="bold",
    )
    for model in (
        ET.parse(args.episode / "generated/world.sdf").getroot().findall("world/model")
    ):
        if model.get("name", "").startswith("wall_"):
            x, y, *_ = map(float, model.findtext("pose").split())
            sx, sy, _ = map(
                float, model.findtext("link/collision/geometry/box/size").split()
            )
            ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color="#475569"))
    ax.text(0, 0.6, "shared passage", ha="center", fontsize=9, color="#64748b")
    tracks, bodies, labels = {}, {}, {}
    for r, rows in positions.items():
        color = colors[r - 1]
        ax.plot(
            [p[1] for p in rows],
            [p[2] for p in rows],
            color=color,
            alpha=0.16,
            linewidth=1,
        )
        (tracks[r],) = ax.plot([], [], color=color, linewidth=2, label=f"Robot {r}")
        bodies[r] = Circle(rows[0][1:], 0.24, color=color, zorder=5)
        ax.add_patch(bodies[r])
        labels[r] = ax.text(
            *rows[0][1:],
            str(r),
            color="white",
            ha="center",
            va="center",
            weight="bold",
            zorder=6,
        )
    ax.legend(loc="upper left", frameon=False, ncol=3)
    ax.grid(alpha=0.15)
    title = ax.text(
        0.98,
        0.98,
        "",
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=10,
        bbox={"facecolor": "white", "alpha": 0.95, "edgecolor": "#cbd5e1"},
    )
    for a in [e for e in events if e["kind"] == "lease_acquired"]:
        b = next(
            e
            for e in events
            if e["kind"] == "lease_released" and e["generation"] == a["generation"]
        )
        timeline.broken_barh(
            [(a["sim_time"] - start, b["sim_time"] - a["sim_time"])],
            (0, 0.65),
            facecolors=colors[a["robot"] - 1],
            alpha=0.8,
        )
    fault = next(e for e in events if e["kind"] == "fault_requested")
    timeline.axvline(fault["sim_time"] - start, color="#dc2626", linestyle="--")
    timeline.text(
        fault["sim_time"] - start + 0.8,
        0.85,
        "Controller killed → respawn / retry",
        color="#b91c1c",
        fontsize=9,
    )
    cursor = timeline.axvline(0, color="#0f172a", linewidth=2)
    timeline.set(
        xlim=(0, end - start),
        ylim=(-0.2, 1.2),
        xlabel="Elapsed simulation time (s)",
        yticks=[],
    )
    timeline.set_title("Exclusive transit ownership", loc="left", fontsize=10)
    times = {r: [p[0] for p in rows] for r, rows in positions.items()}
    samples = [min(start + i * 0.75, end) for i in range(int((end - start) / 0.75) + 2)]

    def update(frame):
        now = samples[frame]
        for r, rows in positions.items():
            index = max(0, bisect.bisect_right(times[r], now) - 1)
            tracks[r].set_data(
                [p[1] for p in rows[: index + 1]], [p[2] for p in rows[: index + 1]]
            )
            bodies[r].center = rows[index][1:]
            labels[r].set_position(rows[index][1:])
        latest = [e for e in events if e["sim_time"] <= now]
        owner = next(
            (
                e["robot"] if e["kind"] == "lease_acquired" else None
                for e in reversed(latest)
                if e["kind"] in ("lease_acquired", "lease_released")
            ),
            None,
        )
        completed = sum(e["kind"] == "task_completed" for e in latest)
        title.set_text(
            f"t = {now - start:.1f} s\nPermit owner: {owner or 'none'}\nDelivered: {completed}/3"
        )
        cursor.set_xdata([now - start, now - start])
        return []

    args.out.mkdir(parents=True, exist_ok=True)
    movie = animation.FuncAnimation(
        fig, update, frames=len(samples), interval=50, blit=False
    )
    movie.save(
        args.out / "gazebo-recovery.gif", writer=animation.PillowWriter(fps=20), dpi=95
    )
    update(
        min(
            range(len(samples)), key=lambda i: abs(samples[i] - (fault["sim_time"] + 3))
        )
    )
    fig.savefig(args.out / "gazebo-recovery.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()

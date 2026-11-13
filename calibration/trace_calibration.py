"""Régénère les graphiques à partir des mesures exportées du classeur."""

import csv
from pathlib import Path
from statistics import linear_regression, mean

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

ROOT = Path(__file__).resolve().parents[1]
COLORS = ("#14639b", "#078373", "#a25719", "#885ba2")
INK = "#162d42"
MUTED = "#50677a"


def read_table(filename):
    with (ROOT / "calibration" / filename).open(encoding="utf-8", newline="") as stream:
        return [{key: int(value) for key, value in row.items()} for row in csv.DictReader(stream)]


def style_axis(axis, xlabel, ylabel):
    axis.set_xlabel(xlabel, color=MUTED, labelpad=8)
    axis.set_ylabel(ylabel, color=MUTED, labelpad=8)
    axis.set_axisbelow(True)
    axis.grid(color="#e3ebf2", linewidth=0.8)
    axis.tick_params(length=0, pad=5, colors=INK)
    for edge in axis.spines.values():
        edge.set_visible(False)


def title(fig, headline, subtitle):
    fig.text(0.075, 0.94, headline, color=INK, fontsize=20, weight="bold", va="top")
    fig.text(0.075, 0.87, subtitle, color=MUTED, fontsize=10.5, va="top")


def save(fig, filename):
    fig.savefig(ROOT / "assets" / filename, dpi=145, facecolor="white")
    plt.close(fig)


def plot_compensation():
    data = read_table("compensation.csv")
    right = [row["commande_droite"] for row in data]
    left = [row["commande_gauche"] for row in data]
    fig, axis = plt.subplots(figsize=(8.6, 4.7))
    fig.subplots_adjust(left=0.12, right=0.94, top=0.77, bottom=0.15)
    title(fig, "Compensation du moteur gauche", "Commandes retenues pour faire avancer le robot en ligne droite")
    axis.plot([85, 315], [85, 315], "--", color="#879bad", label="Commandes identiques")
    axis.plot(right, left, "o-", color=COLORS[0], lw=1.6, label="Réglage relevé")
    for x, y in zip(right, left):
        axis.annotate(f"{y-x:+d}", (x, y), xytext=(0, 10), textcoords="offset points",
                      color=COLORS[0], ha="center", weight="bold", fontsize=10)
    axis.set(xlim=(82, 320), ylim=(82, 330), xticks=right)
    style_axis(axis, "Commande moteur droit", "Commande moteur gauche")
    axis.legend(frameon=False, loc="upper left")
    save(fig, "compensation-moteur.png")


def plot_encoders():
    data = read_table("encodeurs.csv")
    commands = sorted({row["commande_nominale"] for row in data})
    distances = [row["distance_cm"] for row in data]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), sharey=True)
    fig.subplots_adjust(left=0.075, right=0.968, top=0.75, bottom=0.22, wspace=0.16)
    title(fig, "Distance et impulsions des encodeurs", "Quatre commandes et quatre distances mesurées par roue")
    for axis, side, name in zip(axes, ("droite", "gauche"), ("Roue droite", "Roue gauche")):
        column = f"impulsions_{side}"
        for command, color in zip(commands, COLORS):
            series = [row for row in data if row["commande_nominale"] == command]
            axis.scatter([row["distance_cm"] for row in series], [row[column] for row in series],
                         color=color, s=33, label=str(command), zorder=3)
        fit = linear_regression(distances, [row[column] for row in data])
        extent = [min(distances), max(distances)]
        axis.plot(extent, [fit.slope*x + fit.intercept for x in extent],
                  "--", color=INK, lw=1.45, label="Ajustement sur 5–20 cm")
        slope = f"{fit.slope:.2f}".replace(".", ",")
        axis.text(0.04, 0.96, f"{slope} imp./cm  ·  décalage extrapolé {fit.intercept:.0f} imp.",
                  transform=axis.transAxes, ha="left", va="top", fontsize=9.3, color=MUTED)
        axis.set_title(name, loc="left", color=INK, weight="bold", fontsize=12.5)
        axis.set(ylim=(580, 2250), xticks=sorted(set(distances)), xlim=(3.6, 21.1))
        style_axis(axis, "Distance mesurée (cm)", "Impulsions" if axis is axes[0] else "")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.52, 0.025),
               title="Commande nominale", ncols=5, frameon=False, fontsize=9.3)
    save(fig, "distance-encodeurs.png")


def plot_speed():
    data = read_table("vitesse.csv")
    distances = sorted({row["distance_cm"] for row in data})
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), sharey=True)
    fig.subplots_adjust(left=0.075, right=0.968, top=0.75, bottom=0.22, wspace=0.16)
    title(fig, "Vitesse du robot selon la commande", "La vitesse est calculée en divisant la distance par le temps mesuré")
    for axis, side, name in zip(axes, ("droite", "gauche"), ("Commande droite", "Commande gauche")):
        column = f"commande_{side}"
        commands = sorted({row[column] for row in data})
        for distance, marker, color in zip(distances, ("o", "s", "^", "D"), COLORS):
            series = [row for row in data if row["distance_cm"] == distance]
            axis.scatter([row[column] for row in series], [1000*distance / row["duree_ms"] for row in series],
                         marker=marker, color=color, s=33, label=f"{distance} cm", zorder=4)
        averages = [mean(1000*row["distance_cm"] / row["duree_ms"] for row in data if row[column] == command)
                    for command in commands]
        axis.plot(commands, averages, "--", color=INK, lw=1.45, label="Moyenne")
        axis.set_title(name, loc="left", color=INK, weight="bold", fontsize=12.5)
        axis.set(ylim=(17, 52), xticks=commands)
        axis.yaxis.set_major_locator(MultipleLocator(10))
        style_axis(axis, "Commande moteur", "Vitesse mesurée (cm/s)" if axis is axes[0] else "")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.52, 0.025),
               title="Longueur de l’essai", ncols=5, frameon=False, fontsize=9.3)
    save(fig, "vitesse-commandes.png")


if __name__ == "__main__":
    plot_compensation()
    plot_encoders()
    plot_speed()

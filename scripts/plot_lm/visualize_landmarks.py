#!/usr/bin/env python3
import yaml
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

def load_mapped(path):
    """Lädt gemappte Landmarken aus YAML."""
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    out = []
    for x, y, sig in data:
        rho, theta, color = sig
        out.append([x, y, rho, theta, color])
    return out

def cluster_landmarks(landmarks, eps=0.5):
    """Clustert anhand (x,y) pro Farbe mit DBSCAN."""
    final = []
    for color in set(lm[4] for lm in landmarks):
        pts = np.array([[lm[0], lm[1]] for lm in landmarks if lm[4] == color])
        rhos   = np.array([lm[2] for lm in landmarks if lm[4] == color])
        thetas = np.array([lm[3] for lm in landmarks if lm[4] == color])
        db = DBSCAN(eps=eps, min_samples=1).fit(pts)
        for lbl in set(db.labels_):
            idx = np.where(db.labels_ == lbl)[0]
            cx, cy = pts[idx].mean(axis=0)
            avg_rho   = float(rhos[idx].mean())
            avg_theta = float(thetas[idx].mean())
            theta_deg = avg_theta * 180.0 / np.pi
            final.append([
                float(cx), float(cy),
                avg_rho, avg_theta, theta_deg,
                color
            ])
    return final

def plot_landmarks(final):
    """Erzeugt XY-, Polar- und Grad-Plot."""
    # Farb-Mapping, damit wir immer rgb-Namen verwenden
    color_map = {'red':'red', 'orange':'orange', 'green':'green'}

    fig = plt.figure(figsize=(15,5))

    # 1) XY-Plot
    ax1 = fig.add_subplot(131)
    used = set()
    for x, y, _, _, _, color in final:
        ax1.scatter(x, y,
                    color=color_map[color],
                    label=color if color not in used else "",
                    s=50)
        used.add(color)
    ax1.set_title('Landmarken in XY-Ebene')
    ax1.set_xlabel('X [m]'); ax1.set_ylabel('Y [m]')
    ax1.set_aspect('equal')
    ax1.legend()

    # 2) Polar-Plot (rad)
    ax2 = fig.add_subplot(132, projection='polar')
    used = set()
    for _, _, rho, theta, _, color in final:
        ax2.scatter(theta, rho,
                    color=color_map[color],
                    label=color if color not in used else "",
                    s=50)
        used.add(color)
    ax2.set_title('Polar-Koordinaten\n(θ [rad], ρ [m])')
    ax2.legend(loc='upper right')

    # 3) Winkel in Grad vs. rho
    ax3 = fig.add_subplot(133)
    used = set()
    for _, _, rho, _, theta_deg, color in final:
        ax3.scatter(theta_deg, rho,
                    color=color_map[color],
                    label=color if color not in used else "",
                    s=50)
        used.add(color)
    ax3.set_title('Winkel vs. Entfernung')
    ax3.set_xlabel('θ [°]'); ax3.set_ylabel('ρ [m]')
    ax3.legend()

    plt.tight_layout()
    plt.show()

def save_final(path, final):
    """Speichert finale Landmarken als YAML (mit θ in rad und °)."""
    with open(path, 'w') as f:
        yaml.dump(final, f)

def main():
    mapped_path = 'mapped_landmarks.yaml'
    final_path  = 'final_landmarks_deg.yaml'

    lms = load_mapped(mapped_path)
    final = cluster_landmarks(lms, eps=0.5)
    save_final(final_path, final)
    print(f"Final landmarks ({len(final)}) written to '{final_path}'")
    plot_landmarks(final)

if __name__ == '__main__':
    main()

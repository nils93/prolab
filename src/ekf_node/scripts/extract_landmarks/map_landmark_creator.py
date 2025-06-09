#!/usr/bin/env python3

import cv2
import matplotlib.pyplot as plt
import yaml

# === PARAMETER ANPASSEN ===
map_path = "../../maps/my_map.pgm"      # Pfad zur Kartengrafik
resolution = 0.05                       # Auflösung in m/pixel (aus .yaml)
origin = (-21.2, -21.2)                 # Ursprung in Weltkoordinaten (aus .yaml)
yaml_output_file = "landmark_map.yaml" # Ziel-YAML für Landmarken
image_output_file = "landmarks_labeled.png" # Visualisierte Ausgabe

# === BILD LADEN ===
img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
if img is None:
    print("Fehler beim Laden des Bildes.")
    exit()

height = img.shape[0]
clicked_points = []

# === MAUSKLICK-HANDLER ===
def onclick(event):
    """Callback bei Mausklick in der Karte. Konvertiert Bildkoordinaten in Weltkoordinaten."""
    if event.xdata is None or event.ydata is None:
        return
    u, v = int(event.xdata), int(event.ydata)
    x_world = origin[0] + u * resolution
    y_world = origin[1] + (height - v) * resolution
    clicked_points.append({"id": len(clicked_points), "x": round(x_world, 3), "y": round(y_world, 3)})
    ax.plot(u, v, 'ro')
    ax.text(u + 5, v - 5, str(len(clicked_points)-1), color='red', fontsize=4)
    plt.draw()

# === ANZEIGE UND INTERAKTION ===
fig, ax = plt.subplots()
ax.imshow(img, cmap='gray')
cid = fig.canvas.mpl_connect('button_press_event', onclick)
plt.title("Klicke auf Landmarken, schließe das Fenster zum Speichern")
plt.show()

# === YAML SPEICHERN ===
with open(yaml_output_file, "w") as f:
    yaml.dump({"landmarks": clicked_points}, f)

# === SPEICHERN LABEL-BILD ===
fig2, ax2 = plt.subplots()
ax2.imshow(img, cmap='gray')
for lm in clicked_points:
    u = int((lm["x"] - origin[0]) / resolution)
    v = int(height - (lm["y"] - origin[1]) / resolution)
    ax2.plot(u, v, 'ro', markersize=4)
    ax2.text(u + 5, v - 5, str(lm["id"]), color='red', fontsize=4)
plt.axis('off')
plt.savefig(image_output_file, dpi=300, bbox_inches='tight')
plt.close()

print(f"\n{len(clicked_points)} Landmarken gespeichert in '{yaml_output_file}'")
print(f"Bild mit Labels gespeichert als '{image_output_file}'")

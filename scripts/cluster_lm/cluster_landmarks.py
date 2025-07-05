
import yaml
import numpy as np
from sklearn.cluster import DBSCAN
from collections import defaultdict

# --- Konfiguration ---
# Der maximale Abstand (in Metern) zwischen Punkten, um als Nachbarn zu gelten.
# Ein guter Startwert ist etwas größer als der erwartete Drift des Roboters im Stillstand.
EPSILON = 0.5  # 50 cm

# Die Mindestanzahl von Punkten, die innerhalb des Epsilon-Radius liegen müssen,
# um einen dichten Bereich (einen Cluster-Kern) zu bilden.
MIN_SAMPLES = 5

INPUT_FILE = 'mapped_landmarks.yaml'
OUTPUT_FILE = 'mapped_landmarks_clustered.yaml'

def cluster_landmarks():
    """
    Liest eine YAML-Datei mit Landmarken, führt DBSCAN-Clustering durch,
    um verrauschte Daten zu bereinigen, und speichert die Mittelpunkte
    der gefundenen Cluster in einer neuen YAML-Datei.
    """
    print(f"Lese Landmarken aus '{INPUT_FILE}'...")
    try:
        with open(INPUT_FILE, 'r') as f:
            landmarks_raw = yaml.safe_load(f)
        if not landmarks_raw:
            print("Die Eingabedatei ist leer. Es gibt nichts zu tun.")
            return
    except FileNotFoundError:
        print(f"Fehler: Eingabedatei nicht gefunden: {INPUT_FILE}")
        return
    except yaml.YAMLError as e:
        print(f"Fehler beim Parsen der YAML-Datei: {e}")
        return

    # 1. Landmarken nach Farbe gruppieren
    #    Wir speichern nur die (x, y) Koordinaten der Landmarke selbst.
    points_by_color = defaultdict(list)
    for entry in landmarks_raw:
        # Format: [lm_x, lm_y, robot_x, robot_y, color]
        points_by_color[entry[4]].append([entry[0], entry[1]])

    print(f"Gefundene Farben: {list(points_by_color.keys())}")

    clustered_landmarks = []

    # 2. Für jede Farbgruppe DBSCAN ausführen
    for color, points in points_by_color.items():
        if len(points) < MIN_SAMPLES:
            print(f"Farbe '{color}' hat nur {len(points)} Punkte. Überspringe, da weniger als min_samples={MIN_SAMPLES}.")
            continue

        print(f"--- Clustering für Farbe: {color} ({len(points)} Punkte) ---")
        
        points_np = np.array(points)
        
        # DBSCAN-Modell erstellen und anpassen
        db = DBSCAN(eps=EPSILON, min_samples=MIN_SAMPLES).fit(points_np)
        
        # Labels: -1 für Rauschen, 0, 1, 2... für jedes Cluster
        labels = db.labels_
        
        unique_labels = set(labels)
        num_clusters = len(unique_labels) - (1 if -1 in labels else 0)
        num_noise = np.sum(labels == -1)
        
        print(f"Ergebnis: {num_clusters} Cluster gefunden, {num_noise} Rauschpunkte.")

        # 3. Mittelpunkt für jedes gefundene Cluster berechnen
        for k in unique_labels:
            if k == -1:
                # Rauschpunkte überspringen
                continue

            # Alle Punkte, die zu diesem Cluster gehören
            class_member_mask = (labels == k)
            cluster_points = points_np[class_member_mask]
            
            # Mittelpunkt berechnen
            cluster_center = np.mean(cluster_points, axis=0)
            
            # Zum Endergebnis hinzufügen
            clustered_landmarks.append([float(cluster_center[0]), float(cluster_center[1]), color])
            print(f"  - Cluster {k}: {len(cluster_points)} Punkte. Mittelpunkt: [{cluster_center[0]:.4f}, {cluster_center[1]:.4f}]")


    # 4. Bereinigte Landmarken in neue YAML-Datei schreiben
    print(f"Schreibe {len(clustered_landmarks)} bereinigte Landmarken nach '{OUTPUT_FILE}'...")
    try:
        with open(OUTPUT_FILE, 'w') as f:
            # Wir verwenden ein einfacheres Format für die bereinigte Datei: [x, y, color]
            yaml.dump(clustered_landmarks, f, default_flow_style=True)
        print("Fertig.")
    except Exception as e:
        print(f"Fehler beim Schreiben der Ausgabedatei: {e}")


if __name__ == '__main__':
    cluster_landmarks()

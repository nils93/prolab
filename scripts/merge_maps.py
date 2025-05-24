import cv2
import numpy as np
import os

# Verzeichnis mit Snapshots
map_dir = os.path.expanduser("~/turtle_ws/src/example_package/map")
snapshot_files = [os.path.join(map_dir, f) for f in os.listdir(map_dir) if f.endswith(".pgm")]
snapshot_files.sort()  # alphabetisch sortieren

# Erste Map als Basisbild laden
base_map = cv2.imread(snapshot_files[0], cv2.IMREAD_GRAYSCALE)

# Alle weiteren Snapshots einfügen (OR-Operation)
for snap_file in snapshot_files[1:]:
    snap_map = cv2.imread(snap_file, cv2.IMREAD_GRAYSCALE)
    base_map = cv2.bitwise_or(base_map, snap_map)

# Ergebnis speichern
result_file = os.path.join(map_dir, "merged_map.pgm")
cv2.imwrite(result_file, base_map)
print(f"✅ Merged map gespeichert: {result_file}")

# YAML-Datei kopieren/anpassen (z.B. von einer existierenden Map)
yaml_source = os.path.join(map_dir, "my_map_0.yaml")
yaml_target = os.path.join(map_dir, "merged_map.yaml")
if os.path.exists(yaml_source):
    with open(yaml_source, 'r') as src, open(yaml_target, 'w') as dst:
        for line in src:
            if line.startswith("image:"):
                dst.write(f"image: {os.path.basename(result_file)}\n")
            else:
                dst.write(line)
    print(f"✅ YAML-Datei angepasst: {yaml_target}")
else:
    print("⚠️ Keine YAML-Datei zum Kopieren gefunden!")

print("🚀 Fertig!")

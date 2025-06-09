#!/usr/bin/env python3

import subprocess
import os

# Speicherort
map_dir = os.path.expanduser("~/turtle_ws/src/example_package/map")

# Ordner anlegen, falls nicht vorhanden
if not os.path.exists(map_dir):
    os.makedirs(map_dir)
    print(f"📁 Ordner erstellt: {map_dir}")

counter = 0
print("🔄 Gebe 'save' ein und drücke Enter, um die aktuelle Map zu speichern. Mit 'exit' beenden.")

while True:
    user_input = input("> ").strip().lower()
    if user_input == "save":
        filename = os.path.join(map_dir, f"my_map_{counter}")
        print(f"📷 Speichere Snapshot: {filename}")
        subprocess.run(["rosrun", "map_server", "map_saver", "-f", filename])
        counter += 1
    elif user_input == "exit":
        print("👋 Beende das Speichern.")
        break
    else:
        print("❓ Befehl nicht erkannt. Gebe 'save' oder 'exit' ein.")

import cv2
import numpy as np
from apriltag import apriltag

# Tag-Einstellungen
tag_family = "tag36h11"
tag_id = 0  # ID des Tags (0-587 für tag36h11)
tag_size = 500  # Größe in Pixeln

# Tag generieren
detector = apriltag(tag_family)
tag_img = detector.generate(tag_id)

# Speichern als PNG
cv2.imwrite(f"apriltag_{tag_id}.png", tag_img)
print(f"AprilTag {tag_id} (Familie: {tag_family}) erstellt!")
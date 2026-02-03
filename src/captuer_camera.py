import cv2
import os

# Folder where images will be saved
SAVE_DIR = "captures"

# Create folder if it doesn't exist
os.makedirs(SAVE_DIR, exist_ok=True)

# Open USB camera (/dev/video0)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera")
    exit()

print("Press 'c' to capture image, 'q' to quit")

img_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("USB Camera", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        filename = os.path.join(SAVE_DIR, f"capture_{img_count}.jpg")
        cv2.imwrite(filename, frame)
        print(f"Image saved as {filename}")
        img_count += 1

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

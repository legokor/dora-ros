from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")

cap = cv2.VideoCapture(0)

threshold = 0.5

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)

    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if cls_id == 0 and conf > threshold:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                print("x1: ",x1)
                print("y1: ",y1)
                print("x2: ",x2)
                print("y2: ",y2)
                conf = float(box.conf[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"Ember {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # cv2.imshow("YOLO - Ember felismerés", frame)


    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()

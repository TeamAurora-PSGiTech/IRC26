#!/usr/bin/env python3

import sys
import os
from ultralytics import YOLO

def main():
    # Path to the model
    model_path = os.path.join(os.path.dirname(__file__), '..', 'best (2).pt')
    
    # Load the model
    try:
        model = YOLO(model_path)
        print("Model loaded successfully.")
    except Exception as e:
        print(f"Error loading model: {e}")
        sys.exit(1)
    
    # Example prediction (replace with actual image path)
    image_path = "/tmp/debug_frame.jpg"  # Change this to your test image
    if not os.path.exists(image_path):
        print(f"Test image '{image_path}' not found. Please provide a test image.")
        sys.exit(1)
    
    try:
        results = model.predict(
            source=image_path,
            imgsz=960,
            conf=0.80,
            iou=0.6,
            max_det=10,
            save=True
        )
        print("Prediction completed. Results saved"
        ".")
        print(results)
        
        # Extract and print bounding box coordinates
        if results and len(results) > 0:
            boxes = results[0].boxes
            if boxes is not None and len(boxes) > 0:
                print("\nDetected cones:")
                for i, box in enumerate(boxes):
                    coords = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                    conf = box.conf[0].cpu().numpy()    # confidence
                    cls = int(box.cls[0].cpu().numpy())  # class
                    print(f"Cone {i+1}: Class {cls}, Confidence {conf:.2f}, Box: {coords}")
            else:
                print("No cones detected.")
        else:
            print("No results.")
    except Exception as e:
        print(f"Error during prediction: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
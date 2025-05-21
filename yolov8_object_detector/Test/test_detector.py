#!/usr/bin/env python3
"""
Standalone test script for verifying YOLOv8ObjectDetector performance
Supports single image or directory input.
"""
import os
import sys
import cv2
import argparse
from glob import glob
from ultralytics import YOLO

def draw_boxes(image, results):
    for res in results:
        for box, cls, conf in zip(res.boxes.xyxy, res.boxes.cls, res.boxes.conf):
            x1, y1, x2, y2 = map(int, box)
            label = f"{res.names[int(cls)]} {conf*100:.1f}%"
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 255, 0), 2, cv2.LINE_AA)
    return image

def process_image(model, img_path, out_dir, conf=0.5, iou=0.45):
    img = cv2.imread(img_path)
    if img is None:
        print(f"Error: unable to read {img_path}")
        return
    results = model(img, conf=conf, iou=iou)
    print(f"[{os.path.basename(img_path)}] Detected {len(results[0].boxes)} objects")
    annotated = draw_boxes(img.copy(), results)

    fname = os.path.splitext(os.path.basename(img_path))[0]
    out_path = os.path.join(out_dir, f"{fname}_det.jpg")
    cv2.imwrite(out_path, annotated)
    print(f"Saved annotated image: {out_path}")

    # Optional display
    cv2.imshow("Detections", annotated)
    cv2.waitKey(1)

def main():
    parser = argparse.ArgumentParser(description="Test YOLOv8 detector on image(s)")
    parser.add_argument("--model", type=str, required=True,
                        help="Path to YOLOv8 model (ONNX or .pt)")
    parser.add_argument("--input", type=str, required=True,
                        help="Path to image file or directory of images")
    parser.add_argument("--output_dir", type=str, default="./output",
                        help="Directory to save annotated outputs")
    parser.add_argument("--conf", type=float, default=0.5,
                        help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.45,
                        help="IoU threshold")
    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f"Error: model not found at {args.model}")
        sys.exit(1)
    model = YOLO(args.model, task='detect')
    print(f"Loaded model: {args.model}")

    if not os.path.exists(args.input):
        print(f"Error: input path not found: {args.input}")
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    paths = []
    if os.path.isdir(args.input):
        # find common image files
        patterns = ["*.jpg", "*.jpeg", "*.png", "*.JPG", "*.JPEG"]
        for pat in patterns:
            paths.extend(glob(os.path.join(args.input, pat)))
        if not paths:
            print(f"No images found in directory {args.input}")
            sys.exit(1)
    else:
        paths = [args.input]

    for img_path in sorted(paths):
        process_image(model, img_path, args.output_dir, args.conf, args.iou)

    print("All done. Press any key in the image window to exit.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Standalone test script for verifying YOLOv8ObjectDetector performance
Supports single image or directory input.
"""
import os
import sys
import cv2
import argparse
from glob import glob
from ultralytics import YOLO

def draw_boxes(image, results):
    for res in results:
        for box, cls, conf in zip(res.boxes.xyxy, res.boxes.cls, res.boxes.conf):
            x1, y1, x2, y2 = map(int, box)
            label = f"{res.names[int(cls)]} {conf*100:.1f}%"
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1, cv2.LINE_AA)
    return image

def process_image(model, img_path, out_dir, conf=0.5, iou=0.45):
    img = cv2.imread(img_path)
    if img is None:
        print(f"Error: unable to read {img_path}")
        return
    results = model(img, conf=conf, iou=iou)
    print(f"[{os.path.basename(img_path)}] Detected {len(results[0].boxes)} objects")
    annotated = draw_boxes(img.copy(), results)

    fname = os.path.splitext(os.path.basename(img_path))[0]
    out_path = os.path.join(out_dir, f"{fname}_det.jpg")
    cv2.imwrite(out_path, annotated)
    print(f"Saved annotated image: {out_path}")

    # before your first imshow
    cv2.namedWindow("Detections", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Detections", 800, 600)   # whatever size you like

    # right before cv2.imshow(...)
    display = cv2.resize(annotated, (0,0), fx=0.5, fy=0.5)  # half size
    cv2.imshow("Detections", display)

    

    # Optional display
    cv2.imshow("Detections", annotated)
    cv2.waitKey(1)

def main():
    parser = argparse.ArgumentParser(description="Test YOLOv8 detector on image(s)")
    parser.add_argument("--model", type=str, required=True,
                        help="Path to YOLOv8 model (ONNX or .pt)")
    parser.add_argument("--input", type=str, required=True,
                        help="Path to image file or directory of images")
    parser.add_argument("--output_dir", type=str, default="./output",
                        help="Directory to save annotated outputs")
    parser.add_argument("--conf", type=float, default=0.5,
                        help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.45,
                        help="IoU threshold")
    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f"Error: model not found at {args.model}")
        sys.exit(1)
    model = YOLO(args.model, task='detect')
    print(f"Loaded model: {args.model}")

    if not os.path.exists(args.input):
        print(f"Error: input path not found: {args.input}")
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    paths = []
    if os.path.isdir(args.input):
        # find common image files
        patterns = ["*.jpg", "*.jpeg", "*.png", "*.JPG", "*.JPEG"]
        for pat in patterns:
            paths.extend(glob(os.path.join(args.input, pat)))
        if not paths:
            print(f"No images found in directory {args.input}")
            sys.exit(1)
    else:
        paths = [args.input]

    for img_path in sorted(paths):
        process_image(model, img_path, args.output_dir, args.conf, args.iou)

    print("All done. Press any key in the image window to exit.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

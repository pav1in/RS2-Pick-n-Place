#!/usr/bin/env python3
"""
Comprehensive test script for benchmarking YOLOv8ObjectDetector
Includes inference timing, detection metrics, resource logging, and live plotting of results.
Supports single image or directory input.
"""
import os
import sys
import cv2
import time
import argparse
import csv
from glob import glob

import psutil
import GPUtil
import numpy as np
from ultralytics import YOLO
import matplotlib.pyplot as plt

def log_resources():
    """
    Return (used_ram_bytes, used_gpu_mb).
    Falls back to gpu=0.0 if no GPU is found.
    """
    ram = psutil.virtual_memory().used
    try:
        gpus = GPUtil.getGPUs()
        gpu = gpus[0].memoryUsed if gpus else 0.0
    except Exception:
        gpu = 0.0
    return ram, gpu

def draw_boxes(image, results, font_scale=1.0, thickness=2):
    for res in results:
        for box, cls, conf in zip(res.boxes.xyxy, res.boxes.cls, res.boxes.conf):
            x1, y1, x2, y2 = map(int, box)
            label = f"{res.names[int(cls)]} {conf*100:.1f}%"
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), thickness)
            cv2.putText(image, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                        (0, 255, 0), thickness, cv2.LINE_AA)
    return image

def process_image(model, img_path, out_dir, metrics_writer, stats, conf=0.5, iou=0.45):
    img = cv2.imread(img_path)
    if img is None:
        print(f"Error: unable to read {img_path}")
        return

    # resource snapshot before
    ram_before, gpu_before = log_resources()

    # Inference timing
    start = time.time()
    results = model(img, conf=conf, iou=iou)
    elapsed_ms = (time.time() - start) * 1000

    # resource snapshot after
    ram_after, gpu_after = log_resources()
    ram_delta = (ram_after - ram_before) / 1e6    # MB
    gpu_delta = gpu_after - gpu_before            # MB

    # Detection stats
    confs = [float(c) for c in results[0].boxes.conf]
    det_count = len(confs)
    mean_conf = sum(confs) / det_count if det_count else 0.0

    # Write metrics
    metrics_writer.writerow([
        os.path.basename(img_path),
        det_count,
        f"{mean_conf*100:.2f}",
        f"{elapsed_ms:.1f}",
        f"{ram_delta:.1f}",
        f"{gpu_delta:.1f}"
    ])
    stats['times'].append(elapsed_ms)
    stats['counts'].append(det_count)
    stats['confs'].append(mean_conf*100)

    print(f"[{os.path.basename(img_path)}] det={det_count} mean_conf={mean_conf*100:.1f}% "
          f"time={elapsed_ms:.1f}ms  RAMΔ={ram_delta:.1f}MB  GPUΔ={gpu_delta:.1f}MB")

    # Annotate and display
    annotated = draw_boxes(img.copy(), results)
    cv2.namedWindow("Detections", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Detections", 800, 600)
    disp = cv2.resize(annotated, (0,0), fx=0.6, fy=0.6)
    cv2.imshow("Detections", disp)
    cv2.waitKey(1)

    # Save output image
    fname = os.path.splitext(os.path.basename(img_path))[0]
    out_path = os.path.join(out_dir, f"{fname}_det.jpg")
    cv2.imwrite(out_path, annotated)

def plot_metrics(stats):
    plt.figure(figsize=(8,4))
    plt.plot(stats['times'], marker='o')
    plt.title('YOLOv8 Inference Time per Image')
    plt.xlabel('Image Index')
    plt.ylabel('Time (ms)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(8,4))
    plt.bar(range(len(stats['counts'])), stats['counts'])
    plt.title('Detections per Image')
    plt.xlabel('Image Index')
    plt.ylabel('Number of Detections')
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(8,4))
    plt.plot(stats['confs'], marker='s', color='orange')
    plt.title('Mean Confidence per Image')
    plt.xlabel('Image Index')
    plt.ylabel('Confidence (%)')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def main():
    parser = argparse.ArgumentParser(
        description="Benchmark YOLOv8 detector on image(s) with resource logging and live plots"
    )
    parser.add_argument("--model",   required=True, help="Path to YOLOv8 model (.onnx or .pt)")
    parser.add_argument("--input",   required=True, help="Image file or directory to test")
    parser.add_argument("--output_dir", default="./output", help="Directory for annotated outputs and metrics")
    parser.add_argument("--metrics", default="metrics.csv", help="CSV filename for metrics")
    parser.add_argument("--conf",    type=float, default=0.5, help="Confidence threshold")
    parser.add_argument("--iou",     type=float, default=0.45, help="IoU threshold")
    args = parser.parse_args()

    # Validate paths
    if not os.path.exists(args.model):
        print(f"Error: model not found: {args.model}")
        sys.exit(1)
    if not os.path.exists(args.input):
        print(f"Error: input not found: {args.input}")
        sys.exit(1)

    # Load model & warm-up
    model = YOLO(args.model, task='detect')
    print(f"Loaded YOLOv8 model from {args.model}")
    blank = 255 * np.zeros((640,640,3), dtype=np.uint8)
    _ = model(blank, conf=args.conf, iou=args.iou)
    print("Warm-up inference complete.")

    # Prepare outputs and metrics
    os.makedirs(args.output_dir, exist_ok=True)
    metrics_path = os.path.join(args.output_dir, args.metrics)
    stats = {'times': [], 'counts': [], 'confs': []}

    with open(metrics_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            'image','detections','mean_conf_%','time_ms','ram_delta_MB','gpu_delta_MB'
        ])

        # Collect image paths
        if os.path.isdir(args.input):
            paths = []
            for ext in ['jpg','jpeg','png','JPG','JPEG']:
                paths += glob(os.path.join(args.input, f"*.{ext}"))
        else:
            paths = [args.input]

        if not paths:
            print("No valid images found.")
            sys.exit(1)

        cv2.namedWindow("Detections", cv2.WINDOW_NORMAL)
        for img_path in sorted(paths):
            process_image(model, img_path, args.output_dir, writer, stats, args.conf, args.iou)

    print(f"Benchmark complete. Metrics saved to {metrics_path}")
    plot_metrics(stats)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

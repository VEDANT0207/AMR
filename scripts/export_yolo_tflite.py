"""
Script to export YOLOv8 pose model to TensorFlow Lite (.tflite) for Android.

Usage:
    python scripts/export_yolo_tflite.py --model yolov8n-pose.pt --imgsz 640
"""

import argparse
import os
import shutil
from ultralytics import YOLO

def export_model(model_path: str, imgsz: int = 640, int8: bool = False):
    print(f"Loading YOLO model: {model_path}...")
    model = YOLO(model_path)

    print(f"Exporting to TFLite (imgsz={imgsz}, int8={int8})...")
    export_path = model.export(
        format="tflite",
        imgsz=imgsz,
        int8=int8
    )

    print(f"Model successfully exported to: {export_path}")

    # Determine exported .tflite filename
    # Ultralytics export generates a directory or .tflite file depending on version
    tflite_file = None
    if os.path.isfile(export_path) and export_path.endswith(".tflite"):
        tflite_file = export_path
    elif os.path.isdir(export_path):
        for fname in os.listdir(export_path):
            if fname.endswith(".tflite"):
                tflite_file = os.path.join(export_path, fname)
                break
    
    if not tflite_file:
        # Check standard default name in current or model dir
        base_name = os.path.splitext(model_path)[0]
        candidate = f"{base_name}_saved_model/{base_name}_float32.tflite"
        if os.path.exists(candidate):
            tflite_file = candidate
        else:
            candidate2 = f"{base_name}_float32.tflite"
            if os.path.exists(candidate2):
                tflite_file = candidate2

    dest_dir = os.path.join("AMRAndroid", "app", "src", "main", "assets", "models")
    os.makedirs(dest_dir, exist_ok=True)
    dest_path = os.path.join(dest_dir, "yolov8n_pose_float32.tflite")

    if tflite_file and os.path.exists(tflite_file):
        shutil.copyfile(tflite_file, dest_path)
        print(f"Copied exported model to Android assets: {dest_path} (size: {os.path.getsize(dest_path)} bytes)")
    else:
        print(f"Warning: Could not automatically locate tflite file from {export_path}")

    # Inspect TFLite tensor metadata
    inspect_tflite(dest_path if os.path.exists(dest_path) else tflite_file)

def inspect_tflite(tflite_path: str):
    if not tflite_path or not os.path.exists(tflite_path):
        print("Cannot inspect TFLite: file not found.")
        return
    try:
        import tensorflow as tf
        interpreter = tf.lite.Interpreter(model_path=tflite_path)
        interpreter.allocate_tensors()

        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        print("\n=== TFLITE TENSOR INSPECTION ===")
        print(f"File: {tflite_path}")
        print(f"Size: {os.path.getsize(tflite_path):,} bytes")
        for i, inp in enumerate(input_details):
            print(f"Input {i}: name='{inp['name']}', shape={inp['shape']}, dtype={inp['dtype']}")
        for i, out in enumerate(output_details):
            print(f"Output {i}: name='{out['name']}', shape={out['shape']}, dtype={out['dtype']}")
        print("=================================\n")
    except Exception as e:
        print(f"Could not inspect with TensorFlow: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export YOLO model to TFLite for Android AMR")
    parser.add_argument("--model", type=str, default="yolov8n-pose.pt", help="Path to .pt model")
    parser.add_argument("--imgsz", type=int, default=640, help="Model input size (default 640)")
    parser.add_argument("--int8", action="store_true", help="Enable INT8 quantization")
    args = parser.parse_args()

    export_model(args.model, args.imgsz, args.int8)


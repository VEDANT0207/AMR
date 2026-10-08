import cv2
import numpy as np
import tensorflow as tf
import os

def main():
    model_path = os.path.join("AMRAndroid", "app", "src", "main", "assets", "models", "yolov8n_pose_float32.tflite")
    
    print("===== FILE VERIFICATION =====")
    print(f"File exists: {os.path.exists(model_path)}")
    print(f"File size: {os.path.getsize(model_path):,} bytes")
    print(f"Absolute path: {os.path.abspath(model_path)}")

    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    print("\n===== TFLITE MODEL INSPECTION =====")
    print(f"Input tensor count: {len(input_details)}")
    for i, inp in enumerate(input_details):
        print(f"Input tensor name: {inp['name']}")
        print(f"Input tensor shape: {inp['shape'].tolist()}")
        print(f"Input tensor datatype: {inp['dtype']}")
        print(f"Input tensor quantization: {inp['quantization']}")

    print(f"\nOutput tensor count: {len(output_details)}")
    for i, out in enumerate(output_details):
        print(f"    Output tensor name: {out['name']}")
        print(f"    Output tensor shape: {out['shape'].tolist()}")
        print(f"    Output tensor datatype: {out['dtype']}")
        print(f"    Output tensor quantization: {out['quantization']}")

    # Load and preprocess image
    img_orig = cv2.imread("test_person.jpg")
    if img_orig is None:
        print("ERROR: Could not load test_person.jpg")
        return

    h_orig, w_orig, _ = img_orig.shape
    img_rgb = cv2.cvtColor(img_orig, cv2.COLOR_BGR2RGB)
    img_resized = cv2.resize(img_rgb, (640, 640))
    input_data = (img_resized.astype(np.float32) / 255.0)[np.newaxis, ...]

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    raw_output = interpreter.get_tensor(output_details[0]['index'])  # shape [1, 56, 8400]
    
    print("\n===== OUTPUT NUMERICAL INTEGRITY =====")
    nan_count = int(np.isnan(raw_output).sum())
    inf_count = int(np.isinf(raw_output).sum())
    print(f"NaN = {nan_count}")
    print(f"Infinity = {inf_count}")
    print(f"Min value: {np.min(raw_output):.4f}, Max value: {np.max(raw_output):.4f}")

    # Output structure analysis: [1, 56, 8400] -> [8400, 56]
    preds = raw_output[0].T

    boxes = []
    scores = []
    kps = []

    for pred in preds:
        score = float(pred[4])
        if score > 0.3:
            cx, cy, w, h = pred[0:4]
            x1 = cx - w / 2.0
            y1 = cy - h / 2.0
            boxes.append([float(x1), float(y1), float(w), float(h)])
            scores.append(score)
            kps.append(pred[5:].reshape(17, 3))

    indices = cv2.dnn.NMSBoxes(boxes, scores, score_threshold=0.3, nms_threshold=0.45)
    
    print("\n===== REAL IMAGE INFERENCE REPORT =====")
    print(f"Inference successful: YES")
    print(f"Number of raw candidates: {preds.shape[0]}")
    print(f"Number of person candidates: {len(scores)}")
    print(f"Number after NMS: {len(indices)}")

    if len(indices) > 0:
        for k, idx in enumerate(indices):
            i = idx if isinstance(idx, (int, np.integer)) else idx[0]
            box = boxes[i]
            score = scores[i]
            kp = kps[i]
            print(f"\nPerson #{k+1}:")
            print(f"  Bounding box (xywh in 640x640 coords): [{box[0]:.2f}, {box[1]:.2f}, {box[2]:.2f}, {box[3]:.2f}]")
            print(f"  Person confidence: {score:.4f}")
            print(f"  Number of keypoints: {len(kp)}")
            print(f"  Keypoint confidence range: [{kp[:, 2].min():.4f}, {kp[:, 2].max():.4f}]")
            print(f"  Nose keypoint (x, y, conf): [{kp[0,0]:.2f}, {kp[0,1]:.2f}, {kp[0,2]:.4f}]")
            print(f"  Left shoulder (x, y, conf): [{kp[5,0]:.2f}, {kp[5,1]:.2f}, {kp[5,2]:.4f}]")
            print(f"  Right shoulder (x, y, conf): [{kp[6,0]:.2f}, {kp[6,1]:.2f}, {kp[6,2]:.4f}]")
            print(f"  Left hip (x, y, conf): [{kp[11,0]:.2f}, {kp[11,1]:.2f}, {kp[11,2]:.4f}]")
            print(f"  Right hip (x, y, conf): [{kp[12,0]:.2f}, {kp[12,1]:.2f}, {kp[12,2]:.4f}]")

if __name__ == "__main__":
    main()

import os
os.environ["DISABLE_MODEL_SOURCE_CHECK"] = "True"
os.environ["PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK"] = "True"
import time
import cv2
import numpy as np
from paddleocr import PaddleOCR

# ================= IMX298 CAMERA SETTINGS =================
DEVICE = "/dev/video4"
COLOR_W, COLOR_H, FPS = 4656, 3496, 10
FLIP_CODE = 1  # Horizontal flip if needed

# Preview settings
PREVIEW_W, PREVIEW_H = 1280, 960

# ROI for book spines
ROI_X0, ROI_Y0 = 0.25, 0.50
ROI_X1, ROI_Y1 = 0.80, 0.75

# OCR settings
OCR_PERIOD_S = 1.0
FRAME_BUFFER_SIZE = 5  # Multi-frame averaging

# ================= INITIALIZE CAMERA =================
print("Initializing IMX298 camera...")
cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, COLOR_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, COLOR_H)
cap.set(cv2.CAP_PROP_FPS, FPS)

if not cap.isOpened():
    raise RuntimeError("Failed to open IMX298 camera")

print(f"Camera initialized: {COLOR_W}x{COLOR_H} @ {cap.get(cv2.CAP_PROP_FPS):.0f}fps")

# Warm up
print("Warming up camera (30 frames)...")
for i in range(30):
    cap.read()
    if i % 10 == 0:
        print(f"  {i}/30...")
print("Camera ready!\n")

# ================= INITIALIZE PADDLE OCR =================
print("Initializing PaddleOCR v3 mobile (lightweight)...")
ocr = PaddleOCR(
    lang='en',
    det_model_dir=None,  # Will auto-download mobile model
    rec_model_dir=None,
    use_angle_cls=False,  # Disable angle classifier for speed
    use_gpu=False,
    show_log=False,
    det_db_box_thresh=0.3,  # Lower threshold for better detection
    det_db_unclip_ratio=1.5,  # Adjust text box expansion
    rec_batch_num=6,  # Batch size for recognition
    max_text_length=25,  # Max chars per detection
    # Use mobile models (lightweight)
    det_algorithm='DB',
    rec_algorithm='CRNN'
)
print("PaddleOCR v3 mobile initialized\n")

# ================= HELPERS =================
frame_buffer = []
last_ocr_t = 0
cached_results = []
cached_clusters = []
cached_call_numbers = []

t0 = time.time()
frame_count = 0
disp_fps = 0

def crop_roi(img):
    h, w = img.shape[:2]
    x0 = int(w * ROI_X0)
    y0 = int(h * ROI_Y0)
    x1 = int(w * ROI_X1)
    y1 = int(h * ROI_Y1)
    roi = img[y0:y1, x0:x1].copy()
    return roi, (x0, y0, x1, y1)

def average_frames(buffer):
    if len(buffer) == 0: return None
    return np.mean(buffer, axis=0).astype(np.uint8)

def preprocess_for_ocr(roi_bgr):
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 9, 75, 75)
    clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8,8))
    gray = clahe.apply(gray)
    gray = cv2.fastNlMeansDenoising(gray, None, h=10, templateWindowSize=7, searchWindowSize=21)
    gaussian = cv2.GaussianBlur(gray, (0,0), 2.0)
    gray = cv2.addWeighted(gray, 2.0, gaussian, -1.0, 0)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
    gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

def box_points(box):
    """Return list of 4 (x, y) points from PaddleOCR box in any format"""
    if len(box) == 4 and all(isinstance(p, (list, tuple)) for p in box):
        return box
    elif len(box) == 8 and all(isinstance(p, (int, float)) for p in box):
        return [(box[i], box[i+1]) for i in range(0, 8, 2)]
    elif len(box) == 4 and all(isinstance(p, (int, float)) for p in box):
        x0, y0, x1, y1 = box
        return [(x0,y0), (x1,y0), (x1,y1), (x0,y1)]
    else:
        return [(0,0),(0,0),(0,0),(0,0)]

def box_area(box):
    pts = box_points(box)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (max(xs) - min(xs)) * (max(ys) - min(ys))

def box_center(box):
    pts = box_points(box)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (sum(xs)/4, sum(ys)/4)

def box_bounds(box):
    pts = box_points(box)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)

def cluster_detections_by_book(detections):
    clusters = []
    for box, text, conf in detections:
        cx, cy = box_center(box)
        x_min, y_min, x_max, y_max = box_bounds(box)
        placed = False
        for c in clusters:
            cluster_cx = c["x_pos"]
            cluster_x_min = c["x_min"]
            cluster_x_max = c["x_max"]
            horizontal_overlap = not (x_max < cluster_x_min - 50 or x_min > cluster_x_max + 50)
            close_enough = abs(cx - cluster_cx) < 250
            if horizontal_overlap or close_enough:
                c["items"].append((box, text, conf, cx, cy))
                c["x_pos"] = sum(item[3] for item in c["items"]) / len(c["items"])
                c["x_min"] = min(cluster_x_min, x_min)
                c["x_max"] = max(cluster_x_max, x_max)
                c["all_boxes"].append(box)
                placed = True
                break
        if not placed:
            clusters.append({"x_pos": cx, "x_min": x_min, "x_max": x_max, "items":[(box,text,conf,cx,cy)], "all_boxes":[box]})
    clusters.sort(key=lambda c: c["x_pos"])
    return clusters

def build_call_number_from_cluster(items):
    items = sorted(items, key=lambda x: (x[4], x[3]))
    text_parts = [t.strip().upper() for _, t, _, _, _ in items if t.strip()]
    return " ".join(text_parts) if text_parts else "UNKNOWN"

# ================= MAIN LOOP =================
try:
    cv2.namedWindow("IMX298 Book Reading", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("IMX298 Book Reading", PREVIEW_W, PREVIEW_H)
    cv2.moveWindow("IMX298 Book Reading", 100, 100)
    
    print("\nStarting IMX298 Book Shelf Reading (PaddleOCR v3 mobile)...")
    print("="*60)
    print("Controls: 'q'/ESC quit, 's' save shelf, 'c' capture frame")
    print("="*60 + "\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame!")
            time.sleep(0.1)
            continue

        img = frame.copy()  # Apply flip if needed: cv2.flip(frame, FLIP_CODE)
        frame_buffer.append(img.copy())
        if len(frame_buffer) > FRAME_BUFFER_SIZE: frame_buffer.pop(0)
        now = time.time()

        # OCR
        if (now - last_ocr_t) >= OCR_PERIOD_S and len(frame_buffer) == FRAME_BUFFER_SIZE:
            ocr_start = time.time()
            print(f"[{time.strftime('%H:%M:%S')}] Running OCR...")
            avg_img = average_frames(frame_buffer)
            roi, (x0,y0,x1,y1) = crop_roi(avg_img)
            roi_proc = preprocess_for_ocr(roi)
            
            # Run OCR
            result = ocr.ocr(roi_proc, cls=False)
            
            new_results = []

            if result and result[0]:  # Check if results exist
                for line in result[0]:  # result[0] contains the detections
                    try:
                        box = line[0]  # Bounding box coordinates
                        text_conf = line[1]  # (text, confidence)

                        # Unpack text and confidence
                        if isinstance(text_conf, (tuple, list)) and len(text_conf) >= 2:
                            text, conf = text_conf
                            if isinstance(conf, list):
                                conf = float(conf[0])
                            else:
                                conf = float(conf)
                        else:
                            text = str(text_conf)
                            conf = 1.0

                        text = str(text).strip().upper()
                        if not text or len(text) < 1:
                            continue

                        # Convert box to proper format
                        if isinstance(box[0], (list, tuple)):
                            box_pts = [(p[0], p[1]) for p in box]
                        else:
                            box_pts = [(box[i], box[i+1]) for i in range(0, len(box), 2)]

                        # Filter by area
                        area = box_area(box_pts)
                        roi_area = roi.shape[0] * roi.shape[1]
                        if area < 0.0003 * roi_area or area > 0.2 * roi_area:
                            continue

                        x_min, y_min, x_max, y_max = box_bounds(box_pts)
                        width = x_max - x_min
                        height = y_max - y_min
                        if height > width * 2.0:
                            continue

                        new_results.append((box_pts, text, conf))

                    except Exception as e:
                        print(f"  Skipping line due to error: {e}")
                        continue

            cached_results = new_results
            cached_clusters = cluster_detections_by_book(new_results)
            cached_call_numbers = [(build_call_number_from_cluster(c["items"]), c["x_pos"],
                                    sum(i[2] for i in c["items"])/len(c["items"]))
                                   for c in cached_clusters]
            last_ocr_t = now
            
            ocr_time = time.time() - ocr_start
            print(f"  OCR took {ocr_time:.2f}s, detected {len(cached_clusters)} books:")
            for i, (call, _, conf) in enumerate(cached_call_numbers):
                print(f"    {i+1}. {call} (conf: {conf:.2f})")

        # Visualize
        roi, (x0,y0,x1,y1) = crop_roi(img)
        disp = img.copy()
        cv2.rectangle(disp, (x0,y0),(x1,y1),(0,255,0),3)
        colors = [(255,0,0),(0,255,255),(255,0,255),(0,165,255),(255,255,0),(128,0,128)]
        for idx, cluster in enumerate(cached_clusters):
            color = colors[idx % len(colors)]
            for box in cluster["all_boxes"]:
                pts = np.array([[int(p[0]+x0), int(p[1]+y0)] for p in box], np.int32)
                cv2.polylines(disp,[pts],True,color,3)
        
        # Scale coordinates for preview
        scale_x = PREVIEW_W / COLOR_W
        scale_y = PREVIEW_H / COLOR_H
        
        preview = cv2.resize(disp,(PREVIEW_W,PREVIEW_H))
        
        y_offset = 50
        cv2.putText(preview,"Shelf Order (Left to Right):",(20,y_offset),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2)
        for i, (call, _, _) in enumerate(cached_call_numbers):
            y_offset += 35
            cv2.putText(preview,f"{i+1}. {call}",(20,y_offset),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)

        # FPS
        frame_count += 1
        if now - t0 >= 1.0:
            disp_fps = frame_count/(now-t0)
            t0 = now
            frame_count = 0
        cv2.putText(preview,f"FPS: {disp_fps:.1f} | Books: {len(cached_clusters)}",(PREVIEW_W-400,30),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2)
        cv2.putText(preview,f"Buffer: {len(frame_buffer)}/{FRAME_BUFFER_SIZE}",(PREVIEW_W-400,70),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,0),2)

        cv2.imshow("IMX298 Book Reading", preview)

        key = cv2.waitKey(1) & 0xFF
        if key in [27, ord('q')]:
            break
        elif key == ord('s'):
            print("\n" + "="*60)
            print("SAVED SHELF ORDER")
            print("="*60)
            for i, (call, _, _) in enumerate(cached_call_numbers):
                print(f"{i+1}. {call}")
            print("="*60 + "\n")
        elif key == ord('c'):
            ts = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"capture_{ts}.jpg", disp)
            print(f"📸 Captured: capture_{ts}.jpg")

finally:
    cap.release()
    cv2.destroyAllWindows()
    print("\n" + "="*60)
    print("FINAL SHELF ORDER")
    print("="*60)
    for i, (call, _, _) in enumerate(cached_call_numbers):
        print(f"{i+1}. {call}")
    print("="*60)
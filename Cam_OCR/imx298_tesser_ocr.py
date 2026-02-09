import os
import time
import cv2
import numpy as np
import pytesseract

# ========== IMX298 CAMERA CONFIGURATION ==========
DEVICE = "/dev/video4"
COLOR_W, COLOR_H, FPS = 4656, 3496, 10  # Full 16MP resolution
FLIP_CODE = 1  # Horizontal flip

# Display preview settings
PREVIEW_W, PREVIEW_H = 1280, 960

# ROI Configuration
ROI_X0, ROI_Y0 = 0.25, 0.50
ROI_X1, ROI_Y1 = 0.80, 0.75

# OCR Configuration
OCR_PERIOD_S = 2.0  # Increased to 2 seconds (Tesseract is slower than PaddleOCR)

# Multi-frame averaging
FRAME_BUFFER_SIZE = 3  # Reduced from 5 to speed up initial detection

# ========== INITIALIZE CAMERA ==========
print("Initializing IMX298 16MP camera...")
cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, COLOR_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, COLOR_H)
cap.set(cv2.CAP_PROP_FPS, FPS)

# Camera controls
cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
cap.set(cv2.CAP_PROP_AUTO_WB, 1)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
# cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
# cap.set(cv2.CAP_PROP_CONTRAST, 128)
# cap.set(cv2.CAP_PROP_SATURATION, 128)
# cap.set(cv2.CAP_PROP_SHARPNESS, 5)

if not cap.isOpened():
    raise RuntimeError("Failed to open IMX298 camera")

actual_fps = cap.get(cv2.CAP_PROP_FPS)
print(f"Camera initialized: {COLOR_W}x{COLOR_H} @ {actual_fps:.0f}fps")

# Warm up camera
print("Warming up camera (30 frames)...")
for i in range(30):
    ret, _ = cap.read()
    if i % 10 == 0:
        print(f"  {i}/30...")
print("Camera ready!\n")

# ========== INITIALIZE OCR ==========
print("Initializing Tesseract OCR...")
print("OCR initialized\n")

# ========== HELPER FUNCTIONS ==========
frame_buffer = []
last_ocr_t = 0.0
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
    if len(buffer) == 0:
        return None
    return np.mean(buffer, axis=0).astype(np.uint8)

def preprocess_for_ocr(roi_bgr):
    """CRITICAL: Match PaddleOCR preprocessing - keep it simple for Tesseract"""
    # Convert to grayscale
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    
    #  Simple binarization (often works best for Tesseract)
    # Otsu's thresholding
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    #  If above doesn't work, try adaptive threshold
    # binary = cv2.adaptiveThreshold(
    #     gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
    #     cv2.THRESH_BINARY, 11, 2
    # )
    
    return binary

def box_area(box):
    xs = [p[0] for p in box]
    ys = [p[1] for p in box]
    return (max(xs) - min(xs)) * (max(ys) - min(ys))

def box_center(box):
    xs = [p[0] for p in box]
    ys = [p[1] for p in box]
    return (sum(xs)/4, sum(ys)/4)

def box_bounds(box):
    xs = [p[0] for p in box]
    ys = [p[1] for p in box]
    return min(xs), min(ys), max(xs), max(ys)

def cluster_detections_by_book(detections):
    if not detections:
        return []

    x_clusters = []

    for box, text, conf in detections:
        cx, cy = box_center(box)
        x_min, y_min, x_max, y_max = box_bounds(box)
        placed = False

        for cluster in x_clusters:
            cluster_cx = cluster["x_pos"]
            cluster_x_min = cluster["x_min"]
            cluster_x_max = cluster["x_max"]

            horizontal_overlap = not (
                x_max < cluster_x_min - 50 or x_min > cluster_x_max + 50
            )
            close_enough = abs(cx - cluster_cx) < 250

            if horizontal_overlap or close_enough:
                cluster["items"].append((box, text, conf, cx, cy))
                cluster["x_pos"] = sum(item[3] for item in cluster["items"]) / len(cluster["items"])
                cluster["x_min"] = min(cluster_x_min, x_min)
                cluster["x_max"] = max(cluster_x_max, x_max)
                cluster["all_boxes"].append(box)
                placed = True
                break

        if not placed:
            x_clusters.append({
                "x_pos": cx,
                "x_min": x_min,
                "x_max": x_max,
                "items": [(box, text, conf, cx, cy)],
                "all_boxes": [box]
            })

    x_clusters.sort(key=lambda c: c["x_pos"])
    return x_clusters

def build_call_number_from_cluster(cluster_items):
    sorted_items = sorted(cluster_items, key=lambda item: (item[4], item[3]))

    text_parts = []
    for box, text, conf, cx, cy in sorted_items:
        cleaned = text.strip().upper()
        if cleaned:
            text_parts.append(cleaned)

    full_call_number = " ".join(text_parts)
    return full_call_number if full_call_number else "UNKNOWN"

# ========== MAIN LOOP ==========
try:
    print("\nStarting IMX298 16MP Book Shelf Reading...")
    print("=" * 60)
    print("Controls:")
    print("  'q' or ESC - Quit")
    print("  's' - Save current shelf order")
    print("  'c' - Capture current frame")
    print("  'd' - Save debug images (ROI + preprocessed)")
    print("=" * 60 + "\n")

    cv2.namedWindow("IMX298 Book Reading", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("IMX298 Book Reading", PREVIEW_W, PREVIEW_H)
    cv2.moveWindow("IMX298 Book Reading", 100, 100)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from camera!")
            time.sleep(0.1)
            continue

        #img = cv2.flip(frame, FLIP_CODE)
        img = frame.copy()  # Use original frame

        frame_buffer.append(img.copy())
        if len(frame_buffer) > FRAME_BUFFER_SIZE:
            frame_buffer.pop(0)

        now = time.time()

        if (now - last_ocr_t) >= OCR_PERIOD_S and len(frame_buffer) >= FRAME_BUFFER_SIZE:
            ocr_start = time.time()
            print(f"[{time.strftime('%H:%M:%S')}] Running OCR on averaged frame...")
            
            averaged_img = average_frames(frame_buffer)
            roi, (x0, y0, x1, y1) = crop_roi(averaged_img)
            roi_proc = preprocess_for_ocr(roi)

            # TESSERACT CONFIG - CRITICAL SETTINGS
            # PSM 3: Fully automatic page segmentation (works better than PSM 11 for structured text)
            # PSM 6: Assume a single uniform block of text
            # PSM 11: Sparse text - find as much text as possible
            # Try PSM 3 first, if no results, the issue is preprocessing not PSM
            tess_config = (
                "--oem 3 "  # Default + LSTM
                "--psm 3 "  # Fully automatic page segmentation
                "-c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789./- "
            )

            # Run OCR
            data = pytesseract.image_to_data(
                roi_proc,
                output_type=pytesseract.Output.DICT,
                config=tess_config
            )

            new_results = []
            roi_area = roi.shape[0] * roi.shape[1]

            for i in range(len(data["text"])):
                text = data["text"][i].strip()
                conf = int(data["conf"][i]) if data["conf"][i] != '-1' else 0

                # MUCH LOWER confidence threshold to see what Tesseract is detecting
                if conf < 20 or not text or len(text) < 2:
                    continue

                x = data["left"][i]
                y = data["top"][i]
                w = data["width"][i]
                h = data["height"][i]

                area = w * h
                # More lenient area filtering
                if area < 0.00005 * roi_area or area > 0.30 * roi_area:
                    continue

                # More lenient aspect ratio
                if h > 0 and h > w * 5.0:
                    continue

                box = [
                    [x, y], 
                    [x + w, y], 
                    [x + w, y + h], 
                    [x, y + h]
                ]
                new_results.append((box, text.upper(), conf / 100.0))

            ocr_time = time.time() - ocr_start
            
            clusters = cluster_detections_by_book(new_results)

            cached_results = new_results
            cached_clusters = clusters
            cached_call_numbers = [
                (build_call_number_from_cluster(c["items"]), c["x_pos"],
                 sum(item[2] for item in c["items"]) / len(c["items"]))
                for c in clusters
            ]

            # Print results
            print(f"  OCR took {ocr_time:.2f}s")
            print(f"  Raw detections: {len(new_results)}")
            print(f"  Detected {len(clusters)} books:")
            for i, (call_num, x_pos, conf) in enumerate(cached_call_numbers):
                print(f"    {i+1}. {call_num} (conf: {conf:.2f})")
            
            if len(new_results) == 0:
                print("     NO TEXT DETECTED! Try:")
                print("     - Press 'd' to save debug images")
                print("     - Check if books are in the GREEN ROI box")
                print("     - Adjust lighting or camera focus")

            last_ocr_t = now

        # Visualize on full resolution frame
        roi, (x0, y0, x1, y1) = crop_roi(img)
        disp = img.copy()
        
        # Draw ROI rectangle (GREEN)
        cv2.rectangle(disp, (x0, y0), (x1, y1), (0, 255, 0), 3)
        
        # Draw detection boxes with different colors for each book cluster
        colors = [(255,0,0), (0,255,255), (255,0,255), (0,165,255), (255,255,0), (128,0,128)]
        
        for idx, cluster in enumerate(cached_clusters):
            color = colors[idx % len(colors)]
            for box in cluster["all_boxes"]:
                # Convert box coordinates from ROI space to full image space
                pts = np.array([[int(p[0] + x0), int(p[1] + y0)] for p in box], dtype=np.int32)
                cv2.polylines(disp, [pts], isClosed=True, color=color, thickness=3)
        
        # Draw shelf order text overlay
        y_offset = 50
        cv2.putText(disp, "Shelf Order (Left to Right):", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        for i, (call_num, x_pos, conf) in enumerate(cached_call_numbers):
            y_offset += 35
            display_text = f"{i+1}. {call_num}"
            cv2.putText(disp, display_text, (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Calculate and display FPS
        frame_count += 1
        if (now - t0) >= 1.0:
            disp_fps = frame_count / (now - t0)
            t0 = now
            frame_count = 0
        
        cv2.putText(disp, f"FPS: {disp_fps:.1f} | Books: {len(cached_clusters)}", 
                   (COLOR_W - 400, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Show buffer status
        cv2.putText(disp, f"Buffer: {len(frame_buffer)}/{FRAME_BUFFER_SIZE}", 
                   (COLOR_W - 400, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Resize for display
        preview = cv2.resize(disp, (PREVIEW_W, PREVIEW_H))
        cv2.imshow("IMX298 Book Reading", preview)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break
        elif key == ord('s'):
            print("\n" + "="*60)
            print("SAVED SHELF ORDER")
            print("="*60)
            for i, (call_num, x_pos, conf) in enumerate(cached_call_numbers):
                print(f"{i+1}. {call_num}")
            print("="*60 + "\n")
        elif key == ord('c'):
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            capture_path = f"capture_{timestamp}.jpg"
            cv2.imwrite(capture_path, disp)
            print(f"\nFrame captured: {capture_path}\n")
        elif key == ord('d'):
            # Save debug images
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            if len(frame_buffer) > 0:
                averaged_img = average_frames(frame_buffer)
                roi, _ = crop_roi(averaged_img)
                roi_proc = preprocess_for_ocr(roi)
                
                cv2.imwrite(f"debug_roi_{timestamp}.jpg", roi)
                cv2.imwrite(f"debug_preprocessed_{timestamp}.jpg", roi_proc)
                print(f"\n  Debug images saved:")
                print(f"   - debug_roi_{timestamp}.jpg")
                print(f"   - debug_preprocessed_{timestamp}.jpg\n")

finally:
    cap.release()
    cv2.destroyAllWindows()
    
    print("\n" + "="*60)
    print("FINAL SHELF ORDER")
    print("="*60)
    for i, (call_num, x_pos, conf) in enumerate(cached_call_numbers):
        print(f"{i+1}. {call_num}")
    print("="*60)
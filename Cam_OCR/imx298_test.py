import cv2

cap = cv2.VideoCapture("/dev/video4", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 4656)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 3496)
cap.set(cv2.CAP_PROP_FPS, 10)

if not cap.isOpened():
    raise RuntimeError("Camera open failed")

PREVIEW_SIZE = (1280, 960)
FLIP_CODE = 1   # change if needed

cv2.namedWindow("IMX298 Preview", cv2.WINDOW_NORMAL)
cv2.resizeWindow("IMX298 Preview", 1280, 960)
cv2.moveWindow("IMX298 Preview", 100, 100)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Correct orientation
    frame = cv2.flip(frame, FLIP_CODE)

    # Resize ONLY for preview
    preview = cv2.resize(frame, PREVIEW_SIZE,
                          interpolation=cv2.INTER_AREA)

    cv2.imshow("IMX298 Preview", preview)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

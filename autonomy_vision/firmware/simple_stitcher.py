import cv2
import glob

imgs = [cv2.imread(f) for f in sorted(glob.glob("panorama_frames/*.jpg"))]

stitcher = cv2.Stitcher_create()
status, pano = stitcher.stitch(imgs)

if status == cv2.Stitcher_OK:
    cv2.imwrite("panorama.jpg", pano)

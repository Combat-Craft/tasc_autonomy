#!/usr/bin/env python3

"""
Standalone OpenCV Panorama Stitching Script

This script stitches together panorama sweep images that were previously captured by panorama_capture.py.

Expected folder structure:
    ~/panorama_images/
    └── test_sweep_01/
        ├── frame_000.jpg
        ├── frame_001.jpg
        ├── frame_002.jpg
        ├── ...
        ├── metadata.csv
        └── panorama.jpg   # created by this script (panorama_opencv_stitcher.py)

Input:
    - A folder path containing frame_*.jpg images.

Output:
    - panorama.jpg
        The final stitched panorama image, saved inside the same sweep folder.

Metadata:
    - metadata.csv
        This script reads gps_latitude, gps_longitude, camera_heading, and camera_cardinal from metadata.csv.
        The GPS latitude/longitude is drawn on the top left of the stitched image. Also, the camera cardinal 
        directions and headings are drawn approximately across the top based on frame order.

Notes:
    - This script is intentionally separate from ROS2 so that stitching can be
      run after image capture without causing CPU spikes during the live sweep.
    - Cardinal label positions are approximate because OpenCV's stitcher warps
      and crops the input images internally. The labels are placed based on the
      original frame order from left to right.
"""

import cv2
from pathlib import Path
import csv

def load_images(folder_path):
    """
    Loads all frame_*.jpg images from the provided sweep folder.

    Returns a list of valid OpenCV image frames and the expanded pathlib.Path object for the sweep folder.
    """

    # Expand into the full home directory path
    folder = Path(folder_path).expanduser()

    # Load images in sorted order
    image_paths = sorted(folder.glob("frame_*.jpg"))

    if len(image_paths) < 2:
        print(f"Need at least 2 images to stitch. Found {len(image_paths)}.")
        return [], folder

    frames = []

    for image_path in image_paths:
        frame = cv2.imread(str(image_path))

        # Skip unreadable or corrupted images instead of crashing.
        if frame is None:
            print(f"Warning: could not read {image_path}")
            continue

        frames.append(frame)

    print(f"Loaded {len(frames)} valid images from {folder}")
    
    return frames, folder

def load_metadata(folder):
    """
    Loads metadata.csv from the sweep folder if it exists.

    Expected metadata columns:
        - frame
        - servo_angle
        - gps_latitude
        - gps_longitude
        - rover_heading
        - camera_heading
        - camera_cardinal
        - image_path

    Returns a list of metadata rows (as dictionaries) or an empty list if no metadata is found.
    """

    metadata_path = folder / "metadata.csv"

    if not metadata_path.exists():
        print("No metadata.csv found. Stitching without metadata overlay.")
        return []

    rows = []

    with open(metadata_path, mode="r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            rows.append(row)

    print(f"Loaded metadata for {len(rows)} frames")
    return rows

def draw_gps_label(pano, metadata):
    """
    Draws the GPS latitude and longitude in the top-left corner
    of the final panorama.
    """

    if len(metadata) == 0:
        print("No metadata available for GPS label.")
        return pano

    first_row = metadata[0]

    lat = first_row.get("gps_latitude", "")
    lon = first_row.get("gps_longitude", "")

    if lat != "" and lon != "":
        gps_text = f"GPS: {lat}, {lon}"
    else:
        gps_text = "GPS: unavailable"

    font = cv2.FONT_HERSHEY_SIMPLEX
    color = (255, 255, 255)
    shadow_color = (0, 0, 0)

    # Draw black shadow with white text
    cv2.putText(pano, gps_text, (20, 30), font, 0.6, shadow_color, 3, cv2.LINE_AA)
    cv2.putText(pano, gps_text, (20, 30), font, 0.6, color, 1, cv2.LINE_AA)

    return pano

def draw_cardinal_labels(pano, metadata):
    """
    Draws cardinal labels across the top of the final panorama.
    Positions are approximate based on frame order.
    """

    if len(metadata) == 0:
        print("No metadata available for cardinal labels.")
        return pano

    # White text with black shadow
    font = cv2.FONT_HERSHEY_SIMPLEX
    color = (255, 255, 255) 
    shadow_color = (0, 0, 0)

    w = pano.shape[1]  # width of the panorama image

    count = len(metadata)

    for i, row in enumerate(metadata):
        # Place the first label near the left, last label near the right,
        # and all other labels evenly between them.
        if count == 1:
            x = w // 2
        else:
            x = int((i / (count - 1)) * (w - 40)) + 20

        # Read camera-facing direction from metadata
        cardinal = row.get("camera_cardinal", "?")
        heading = row.get("camera_heading", "")

        # Format heading as a whole number with degree symbol when possible
        if heading != "":
            try:
                heading_label = f"{float(heading):.0f}"
            except ValueError:
                heading_label = heading
        else:
            heading_label = "N/A"

        # Draw cardinal direction label
        cv2.putText(pano, cardinal, (x - 15, 55), font, 0.6, shadow_color, 3, cv2.LINE_AA)
        cv2.putText(pano, cardinal, (x - 15, 55), font, 0.6, color, 1, cv2.LINE_AA)

        # Draw numeric heading below cardinal direction
        cv2.putText(pano, heading_label, (x - 20, 78), font, 0.45, shadow_color, 3, cv2.LINE_AA)
        cv2.putText(pano, heading_label, (x - 20, 78), font, 0.45, color, 1, cv2.LINE_AA)

        # Draw a small tick mark under each label
        cv2.line(pano, (x, 90), (x, 110), color, 1)

    return pano

def stitch_images(folder_path):
    """
    Loads saved frames from a sweep folder, stitches them into a panorama,
    draws metadata overlays if available, and saves as panorama.jpg.
    """
     
    # Load images
    frames, folder = load_images(folder_path)

    # At least two valid frames are needed for stitching.
    if len(frames) < 2:
        print(f"Not enough valid images to stitch. Found {len(frames)}.")
        return

    # Load metadata if available
    metadata = load_metadata(folder)

    print(f"Stitching {len(frames)} images from: {folder}")

    # Create OpenCV panorama stitcher and attempt to stitch the frames.
    stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    status, pano = stitcher.stitch(frames)

    print(f"Called OpenCV stitcher → status {status}")

    if status != cv2.Stitcher_OK:
        print(f"Stitch failed with status {status}")
        return

    # Draw GPS (latitude, longitude) label if metadata was loaded.
    pano = draw_gps_label(pano, metadata)

    # Draw cardinal labels only if metadata was loaded.
    pano = draw_cardinal_labels(pano, metadata)

    # Save the panorama inside the same folder as the source frames
    output_path = folder / "panorama.jpg"

    saved = cv2.imwrite(str(output_path), pano)

    if not saved:
        print(f"Failed to save panorama to: {output_path}")
        return

    print(f"Panorama saved → {output_path}")


def main():
    folder_path = input("Enter folder path containing frame_*.jpg images: ").strip()

    if folder_path == "":
        print("No folder path entered.")
        return

    stitch_images(folder_path)


if __name__ == "__main__":
    main()

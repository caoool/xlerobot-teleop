import cv2
import sys

def list_cameras(max_cameras=10):
    """
    Test and list all available cameras.
    """
    available_cameras = []
    
    print("Scanning for available cameras...\n")
    
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = int(cap.get(cv2.CAP_PROP_FPS))
                
                print(f"✓ Camera {i}: Available")
                print(f"  Resolution: {width}x{height}")
                print(f"  FPS: {fps}")
                print()
                
                available_cameras.append({
                    'id': i,
                    'resolution': f"{width}x{height}",
                    'fps': fps
                })
            cap.release()
    
    if available_cameras:
        print(f"\nFound {len(available_cameras)} camera(s):")
        for cam in available_cameras:
            print(f"  - Camera {cam['id']}: {cam['resolution']} @ {cam['fps']}fps")
    else:
        print("No cameras found!")
    
    return available_cameras

if __name__ == "__main__":
    list_cameras()

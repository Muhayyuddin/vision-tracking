import cv2 as cv

# Path to your video file
video_path = "T2-3.mp4"

# Open the video file
cap = cv.VideoCapture(video_path)

# Check if the video was successfully opened
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Loop through each frame in the video
while cap.isOpened():
    ret, frame = cap.read()  # Capture frame-by-frame
    if not ret:
        print("Reached the end of the video or cannot fetch the frame.")
        break
    
    # Display the current frame
    cv.imshow('Video Playback', frame)
    
    # Press 'q' to exit the video early
    if cv.waitKey(25) & 0xFF == ord('q'):
        break

# Release the video capture object
cap.release()

# Close all OpenCV windows
cv.destroyAllWindows()

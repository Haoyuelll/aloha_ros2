import cv2
import zarr
import numpy as np

def play_video_from_zarr(zarr_file_path):
    # Open the Zarr file in read mode ('r')
    zarr_file = zarr.open(zarr_file_path, mode='r')

    # Retrieve the 'video_frames' dataset from Zarr
    # if 'video_frames' not in zarr_file:
    #     print(f"Error: No 'video_frames' dataset found in {zarr_file_path}")
    #     return

    video_frames = zarr_file['episode_0']['video_frames']
    
    # Check if the dataset contains any frames
    if video_frames.shape[0] == 0:
        print(f"Error: No video frames found in the Zarr dataset")
        return

    # Loop through the frames and display them using OpenCV
    for i in range(video_frames.shape[0]):
        frame = video_frames[i]  # Get the i-th frame
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # Convert from RGB to BGR for OpenCV

        # Display the frame in a window
        cv2.imshow('Video Playback', frame_bgr)

        # Wait for a key press, and stop if 'q' is pressed
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    # Clean up and close the window
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Define the path to the Zarr file
    zarr_file_path = '/home/kyutae/test.zarr'

    # Call the function to play the video
    play_video_from_zarr(zarr_file_path)

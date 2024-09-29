from lib.test.evaluation.tracker import Tracker

# Directly set your parameters here
tracker_name = 'seqtrack'  # Replace with your tracker name
tracker_param = 'seqtrack_b256_got'  # Replace with your parameter file name
videofile = 'T2-3.mp4'  # Replace with the path to your video file
optional_box = [354,394,327,97]  # Set to [x, y, w, h] if you have an optional bounding box, otherwise None
debug = None  # Set debug level if needed, otherwise None
save_results = False  # Set to True if you want to save the results

def run_video(tracker_name, tracker_param, videofile, optional_box=None, debug=None, save_results=False):
    """Run the tracker on your video file."""
    tracker = Tracker(tracker_name, tracker_param,'got10k_test')
    tracker.run_video(videofilepath=videofile, optional_box=optional_box, debug=debug, save_results=save_results)

def main():
    run_video(tracker_name, tracker_param, videofile, optional_box, debug, save_results)

if __name__ == '__main__':
    main()
    
    




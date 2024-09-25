import cv2
import time

class ThetaVideoRecorder:
    def __init__(self):
        # GStreamer pipeline
        self.pipeline = (
            "thetauvcsrc ! decodebin ! autovideoconvert ! video/x-raw,format=BGRx ! queue ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink"
        )

        # OpenCV VideoCapture to receive images from the GStreamer pipeline
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError('Error: Video capture could not be opened.')

        # Create a VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'avc1')
        self.out = cv2.VideoWriter('/root/test.avi', fourcc, 30.0, (1920, 960))

    def record_video(self):
        start_time = time.time()
        while (time.time() - start_time) < 20:
            ret, frame = self.cap.read()
            print(frame.shape)

            if not ret:
                print('Error: Could not read frame.')
                break

            # Write the frame into the file
            self.out.write(frame)

        # Release everything if job is finished
        self.cap.release()
        self.out.release()
        print("Video saved successfully.")

def main():
    try:
        recorder = ThetaVideoRecorder()
        recorder.record_video()
    except RuntimeError as e:
        print(e)

if __name__ == '__main__':
    main()

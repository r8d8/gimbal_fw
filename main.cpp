#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>

using namespace std;
using namespace cv;

// skip alligment movement if ROI offset lower than threshold
#define ALLIGMENT_ERROR_THRESHOLD 3.0

// global structure to remember some states changed
// by mouse action and move
struct initRoi
{
    // on off
    bool init;
    // updated ROI rectangle
    bool updated;

    // initial coordination based on EVENT_LBUTTONDOWN
    int initX;
    int initY;

    // actual coordination
    int actualX;
    int actualY;

    // Selected Rect
    Rect roiRect;
} SelectedRoi;

// event int is compared to determine action of the mouse EVENT_RBUTTONDOWN
//  EVENT_LBUTTONDOWN, EVENT_LBUTTONUP, EVENT_MOUSEMOVE.
//  If the event is evauated as happened the global structure SelectedRoi
//  is updated.
static void MouseCallBack(int event, int x, int y, int flags, void *img)
{
    // Mouse Right button down
    if (event == EVENT_RBUTTONDOWN)
    {
        return;
    }
    // Mouse Left button down
    if (event == EVENT_LBUTTONDOWN)
    {
        cout << "Mouse down lbut" << endl;
        SelectedRoi.initX = x;
        SelectedRoi.initY = y;
        return;
    }
    // Mouse Left button up
    if (event == EVENT_LBUTTONUP)
    {
        cout << "Mouse up lbut" << endl;
        SelectedRoi.actualX = x;
        SelectedRoi.actualY = y;
        int width = x - SelectedRoi.initX;
        int height = y - SelectedRoi.initY;
        SelectedRoi.roiRect = Rect(
            SelectedRoi.initX, SelectedRoi.initY,
            width, height);

        SelectedRoi.init = true;
        SelectedRoi.updated = true;
        return;
    }
    // Mouse move coordinates update
    if (event == EVENT_MOUSEMOVE)
    {
        SelectedRoi.actualX = x;
        SelectedRoi.actualY = y;
        return;
    }
}

Point2i calcROIOffset(Mat &frame, Rect &roi)
{
    // frame center
    Point2i fCenter = Point2i(frame.size().height / 2, frame.size().width / 2);
    // ROI center
    Point2i rCenter = Point2i(roi.x + roi.width / 2, roi.y + roi.height / 2);

    return fCenter - rCenter;
}

std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width,
                               int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) 
        + ", height=(int)" + std::to_string(capture_height) 
        + ", framerate=(fraction)" + std::to_string(framerate) 
        + "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) 
        + " ! video/x-raw, width=(int)" + std::to_string(display_width) 
        + ", height=(int)" + std::to_string(display_height) 
        + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char **argv)
{
    SelectedRoi.init = false;
    SelectedRoi.updated = false;
    Mat frame;
    VideoCapture cap;
    Ptr<Tracker>  tracker;

    int capture_width = 1920;
    int capture_height = 1080;
    int display_width = 1920;
    int display_height = 1080;
    int framerate = 60;
    int flip_method = 2;

    std::string pipeline = gstreamer_pipeline(capture_width,
                                              capture_height,
                                              display_width,
                                              display_height,
                                              framerate,
                                              flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";

    cap.open(pipeline, cv::CAP_GSTREAMER);
    // check if we succeeded
    if (!cap.isOpened())
    {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    namedWindow("tracker", WINDOW_AUTOSIZE);
    setMouseCallback("tracker", MouseCallBack, 0);

    // perform the tracking process
    printf("Start the tracking process, press ESC to quit.\n");


    bool lf = true;
    for (;;)
    {
        cap.read(frame);
        if (frame.empty())
        {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }


        if (SelectedRoi.init && lf) 
        {
            tracker = TrackerCSRT::create();
            tracker->init(frame, SelectedRoi.roiRect);
            lf = false;
        }

        if (SelectedRoi.updated) 
        {
            tracker = TrackerCSRT::create();
            tracker->init(frame, SelectedRoi.roiRect);
            SelectedRoi.updated = false;
        }

        if (SelectedRoi.init)
        {
            cout <<  "Rect width: " + std::to_string(SelectedRoi.roiRect.width) 
                    + " height: " + std::to_string(SelectedRoi.roiRect.height) << endl;

            tracker->update(frame, SelectedRoi.roiRect);

            Point2i ROIoffset = calcROIOffset(frame, SelectedRoi.roiRect);
            double alligmentError = cv::sqrt(ROIoffset.ddot(ROIoffset));
            if (alligmentError > ALLIGMENT_ERROR_THRESHOLD)
            {
                // do gimbal alligment
                // calculate pitch, yaw angles for correction
            }

            rectangle(frame, SelectedRoi.roiRect, Scalar(0, 0, 255), 2, 1);
        }

        // show image with the tracked object
        imshow("tracker", frame);

        // quit on ESC button
        if (waitKey(1) == 27)
        {
            destroyAllWindows();
            break;
        }
    }

    return 0;
}
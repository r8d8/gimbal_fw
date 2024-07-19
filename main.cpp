#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
#include <cassert>
#include <cmath>
#include <fstream>

using namespace std;
using namespace cv;

// This video stablisation smooths the global trajectory using a sliding average window

// const int SMOOTHING_RADIUS = 15; // In frames. The larger the more stable the video, but less reactive to sudden panning
const int HORIZONTAL_BORDER_CROP = 20; // In pixels. Crops the border to reduce the black borders from stabilisation being too noticeable.

// 1. Get previous to current frame transformation (dx, dy, da) for all frames
// 2. Accumulate the transformations to get the image trajectory
// 3. Smooth out the trajectory using an averaging window
// 4. Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
// 5. Apply the new transformation to the video

struct TransformParam
{
    TransformParam() {}
    TransformParam(double _dx, double _dy, double _da)
    {
        dx = _dx;
        dy = _dy;
        da = _da;
    }

    double dx;
    double dy;
    double da; // angle
};

struct Trajectory
{
    Trajectory() {}
    Trajectory(double _x, double _y, double _a)
    {
        x = _x;
        y = _y;
        a = _a;
    }
    // "+"
    friend Trajectory operator+(const Trajectory &c1, const Trajectory &c2)
    {
        return Trajectory(c1.x + c2.x, c1.y + c2.y, c1.a + c2.a);
    }
    //"-"
    friend Trajectory operator-(const Trajectory &c1, const Trajectory &c2)
    {
        return Trajectory(c1.x - c2.x, c1.y - c2.y, c1.a - c2.a);
    }
    //"*"
    friend Trajectory operator*(const Trajectory &c1, const Trajectory &c2)
    {
        return Trajectory(c1.x * c2.x, c1.y * c2.y, c1.a * c2.a);
    }
    //"/"
    friend Trajectory operator/(const Trajectory &c1, const Trajectory &c2)
    {
        return Trajectory(c1.x / c2.x, c1.y / c2.y, c1.a / c2.a);
    }
    //"="
    Trajectory operator=(const Trajectory &rx)
    {
        x = rx.x;
        y = rx.y;
        a = rx.a;
        return Trajectory(x, y, a);
    }

    double x;
    double y;
    double a; // angle
};

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
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" + std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) + "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" + std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char **argv)
{
    SelectedRoi.init = false;
    SelectedRoi.updated = false;
    Mat frame;
    VideoCapture cap;
    Ptr<Tracker> tracker;

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

    ofstream out_transform("prev_to_cur_transformation.txt");
    ofstream out_trajectory("trajectory.txt");
    ofstream out_smoothed_trajectory("smoothed_trajectory.txt");
    ofstream out_new_transform("new_prev_to_cur_transformation.txt");

    Mat cur, cur_grey;
    Mat prev, prev_grey;

    cap >> prev; // get the first frame.ch
    cvtColor(prev, prev_grey, COLOR_BGR2GRAY);

    // Step 1 - Get previous to current frame transformation (dx, dy, da) for all frames
    vector<TransformParam> prev_to_cur_transform; // previous to current
    // Accumulated frame to frame transform
    double a = 0;
    double x = 0;
    double y = 0;
    // Step 2 - Accumulate the transformations to get the image trajectory
    vector<Trajectory> trajectory; // trajectory at all frames
    //
    // Step 3 - Smooth out the trajectory using an averaging window
    vector<Trajectory> smoothed_trajectory; // trajectory at all frames
    Trajectory X;                           // posteriori state estimate
    Trajectory X_;                          // priori estimate
    Trajectory P;                           // posteriori estimate error covariance
    Trajectory P_;                          // priori estimate error covariance
    Trajectory K;                           // gain
    Trajectory z;                           // actual measurement
    double pstd = 4e-3;                     // can be changed
    double cstd = 0.25;                     // can be changed
    Trajectory Q(pstd, pstd, pstd);         // process noise covariance
    Trajectory R(cstd, cstd, cstd);         // measurement noise covariance
    // Step 4 - Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
    vector<TransformParam> new_prev_to_cur_transform;
    //
    // Step 5 - Apply the new transformation to the video
    // cap.set(CV_CAP_PROP_POS_FRAMES, 0);
    Mat T(2, 3, CV_64F);

    int vert_border = HORIZONTAL_BORDER_CROP * prev.rows / prev.cols; // get the aspect ratio correct
    //
    int k = 1;
    int max_frames = cap.get(CAP_PROP_FRAME_COUNT);
    Mat last_T;
    Mat prev_grey_, cur_grey_;

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
            cout << "Rect width: " + std::to_string(SelectedRoi.roiRect.width) + " height: " + std::to_string(SelectedRoi.roiRect.height) << endl;

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

        // ============ Stabilize =================
        cur = frame;
        cvtColor(cur, cur_grey, COLOR_BGR2GRAY);

        // vector from prev to cur
        vector<Point2f> prev_corner, cur_corner;
        vector<Point2f> prev_corner2, cur_corner2;
        vector<uchar> status;
        vector<float> err;

        goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
        calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err);

        // weed out bad matches
        for (size_t i = 0; i < status.size(); i++)
        {
            if (status[i])
            {
                prev_corner2.push_back(prev_corner[i]);
                cur_corner2.push_back(cur_corner[i]);
            }
        }

        // translation + rotation only
        Mat T = estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

        // in rare cases no transform is found. We'll just use the last known good transform.
        if (T.data == NULL)
        {
            last_T.copyTo(T);
        }

        T.copyTo(last_T);

        // decompose T
        double dx = T.at<double>(0, 2);
        double dy = T.at<double>(1, 2);
        double da = atan2(T.at<double>(1, 0), T.at<double>(0, 0));
        //
        // prev_to_cur_transform.push_back(TransformParam(dx, dy, da));

        out_transform << k << " " << dx << " " << dy << " " << da << endl;
        //
        // Accumulated frame to frame transform
        x += dx;
        y += dy;
        a += da;
        // trajectory.push_back(Trajectory(x,y,a));
        //
        out_trajectory << k << " " << x << " " << y << " " << a << endl;
        //
        z = Trajectory(x, y, a);
        //
        if (k == 1)
        {
            // intial guesses
            X = Trajectory(0, 0, 0); // Initial estimate,  set 0
            P = Trajectory(1, 1, 1); // set error variance,set 1
        }
        else
        {
            // time update（prediction）
            X_ = X;     // X_(k) = X(k-1);
            P_ = P + Q; // P_(k) = P(k-1)+Q;
            // measurement update（correction）
            K = P_ / (P_ + R);                  // gain;K(k) = P_(k)/( P_(k)+R );
            X = X_ + K * (z - X_);              // z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k));
            P = (Trajectory(1, 1, 1) - K) * P_; // P(k) = (1-K(k))*P_(k);
        }
        // smoothed_trajectory.push_back(X);
        out_smoothed_trajectory << k << " " << X.x << " " << X.y << " " << X.a << endl;
        //-
        // target - current
        double diff_x = X.x - x; //
        double diff_y = X.y - y;
        double diff_a = X.a - a;

        dx = dx + diff_x;
        dy = dy + diff_y;
        da = da + diff_a;

        // new_prev_to_cur_transform.push_back(TransformParam(dx, dy, da));
        //
        out_new_transform << k << " " << dx << " " << dy << " " << da << endl;
        //
        T.at<double>(0, 0) = cos(da);
        T.at<double>(0, 1) = -sin(da);
        T.at<double>(1, 0) = sin(da);
        T.at<double>(1, 1) = cos(da);

        T.at<double>(0, 2) = dx;
        T.at<double>(1, 2) = dy;

        Mat cur2;
        warpAffine(prev, cur2, T, cur.size());
        cur2 = cur2(Range(vert_border, cur2.rows - vert_border), Range(HORIZONTAL_BORDER_CROP, cur2.cols - HORIZONTAL_BORDER_CROP));

        // ==================== Display =================

        // Resize cur2 back to cur size, for better side by side comparison
        resize(cur2, cur2, cur.size());

        // Now draw the original and stablised side by side for coolness
        Mat canvas = Mat::zeros(cur.rows, cur.cols * 2 + 10, cur.type());

        prev.copyTo(canvas(Range::all(), Range(0, cur2.cols)));
        cur2.copyTo(canvas(Range::all(), Range(cur2.cols + 10, cur2.cols * 2 + 10)));

        // If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
        if (canvas.cols > 1920)
        {
            resize(canvas, canvas, Size(canvas.cols / 2, canvas.rows / 2));
        }
        // outputVideo<<canvas;
        imshow("before and after", canvas);

        waitKey(10);
        prev = cur.clone(); // cur.copyTo(prev);
        cur_grey.copyTo(prev_grey);

        cout << "Frame: " << k << "/" << max_frames << " - good optical flow: " << prev_corner2.size() << endl;
        k++;

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
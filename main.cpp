#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <iostream>
#include <cstring>
#include <cassert>
#include <cmath>
#include <fstream>

using namespace std;
using namespace cv;

// #define USE_DIS_OPTICALFLOW

// Calculate new transformation once in interval. Use old  transformation in between calculation.
const int OPTFLOW_FRAME_CAP_INTERVAL = 0;

// In pixels. Crops the border to reduce the black borders from stabilisation being too noticeable.
const int HORIZONTAL_BORDER_CROP = 20;

// Fraction of captured frame width and height
// Skip frame transformation if exceeds, to avoid big movement artifacts
const float WARP_THRESHOLD = 0.25;

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
    double da;
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
    double a; // anglele
};

void fixBorder(Mat &frame_stabilized)
{
    Mat T = getRotationMatrix2D(Point2f(frame_stabilized.cols / 2, frame_stabilized.rows / 2), 0, 1.04);
    warpAffine(frame_stabilized, frame_stabilized, T, frame_stabilized.size());
}

std::string gst_cap_pipeline(int cap_width, int cap_height, int cap_fps)
{
    return "libcamerasrc\
            ! video/x-raw,width=" +
           std::to_string(cap_width) + ",height=" + std::to_string(cap_height) + ",framerate=" + std::to_string(cap_fps) + "/1,format=BGR \
            ! appsink";
}

std::string gst_out_pipeline()
{
    // ! rtph264pay config-interval=3 pt=96\
    // ! udpsink host=192.168.191.18 port=5000";

    //  ! rtspclientsink location=rtsp://localhost:8554/videostab debug=true";
    return "appsrc \
        ! video/x-raw, format=BGR\
        ! v4l2convert \
        ! v4l2h264enc ! video/x-h264,level=(string)4,profile=main \
        ! h264parse \
        ! rtph264pay \
        ! udpsink host=192.168.191.18 port=5600 sync=false auto-multicast=0";
}

int main(int argc, char **argv)
{
    // cout << cv::getBuildInformation() << endl;
    Mat frame;
    VideoCapture cap;
    VideoWriter out;
    Ptr<Tracker> tracker;
    int capture_width = 1920;
    int capture_height = 1080;
    int capture_fps = 30;
    int roi_width = 512;
    int roi_height = 512;
    Rect roi = cv::Rect((capture_width - roi_width) / 2, (capture_height - roi_height) / 2, roi_width, roi_height);

    Mat prevgray, gray, bgr;
    Mat flow, flow_uv[2];
    Mat magnitude, angle;
    Mat hsv_split[3], hsv, hsv8;
    Ptr<DenseOpticalFlow> dis_optflow = DISOpticalFlow::create(DISOpticalFlow::PRESET_FAST);

    std::cout << cv::getBuildInformation();
    std::string pipeline = gst_cap_pipeline(capture_width, capture_height, capture_fps);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";

    cap.open(pipeline, cv::CAP_GSTREAMER);
    // check if we succeeded
    if (!cap.isOpened())
    {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    int codec = VideoWriter::fourcc('H', '2', '6', '4');
    out.open(gst_out_pipeline(),
             cv::CAP_GSTREAMER, // apiPreference
             codec,             // fourcc
             capture_fps,       // fps
             cv::Size{capture_width, capture_height},
             true);
    if (!out.isOpened())
    {
        cerr << "ERROR! Unable to open video writer\n";
        return -1;
    }

#ifdef USE_DIS_OPTICALFLOW
    Mat prevgray, gray, bgr;
    Mat flow, flow_uv[2];
    Mat magnitude, angle;
    Mat hsv_split[3], hsv, hsv8;
    Ptr<DenseOpticalFlow> dis_optflow = DISOpticalFlow::create(DISOpticalFlow::PRESET_FAST);
#else
    Mat cur, cur_grey;
    Mat prev, prev_grey;
    int frame_skip_count = 0;

    cap >> frame; // get the first frame.ch

    prev = frame(roi);
    resize(prev, prev, Size(roi_width, roi_height), 0, 0, INTER_AREA);
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
    double pstd = 4e-3;                     // can be changleed
    double cstd = 0.25;                     // can be changleed
    Trajectory Q(pstd, pstd, pstd);         // process noise covariance
    Trajectory R(cstd, cstd, cstd);         // measurement noise covariance
    // Step 4 - Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
    vector<TransformParam> new_prev_to_cur_transform;

    Mat T(2, 3, CV_64F);
    int vert_border = HORIZONTAL_BORDER_CROP * frame.rows / frame.cols; // get the aspect ratio correct

    //
    int k = 1;
    Mat last_T;
    Mat prev_grey_, cur_grey_;
#endif

    for (;;)
    {
        cap.read(frame);
        if (frame.empty())
        {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }

#ifdef USE_DIS_OPTICALFLOW
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        if (!prevgray.empty())
        {
            // visualization
            dis_optflow->calc(prevgray, gray, flow);
            split(flow, flow_uv);
            multiply(flow_uv[1], -1, flow_uv[1]);
            cartToPolar(flow_uv[0], flow_uv[1], magnitude, angle, true);
            normalize(magnitude, magnitude, 0, 1, NORM_MINMAX);
            angle *= ((1.f / 360.f) * (180.f / 255.f));

            // build  image
            hsv_split[0] = angle;
            hsv_split[1] = magnitude;
            hsv_split[2] = Mat::ones(angle.size(), angle.type());
            merge(hsv_split, 3, hsv);
            hsv.convertTo(hsv8, CV_8U, 255.0);
            cvtColor(hsv8, bgr, COLOR_HSV2BGR);

            out << bgr;
        }
        std::swap(prevgray, gray);
#else
        if (frame_skip_count > 0)
        {
            frame_skip_count--;

            Mat frame_stabilized;
            warpAffine(frame, frame_stabilized, last_T, frame.size());
            fixBorder(frame_stabilized);
            out << frame_stabilized;
        }
        else
        {
            frame_skip_count = OPTFLOW_FRAME_CAP_INTERVAL;

            cur = frame(roi);
            resize(cur, cur, Size(roi_width, roi_height), 0, 0, INTER_AREA);
            cvtColor(cur, cur_grey, COLOR_BGR2GRAY);
            // vector from prev to cur
            vector<Point2f> prev_corner, cur_corner;
            vector<Point2f> prev_corner2, cur_corner2;
            vector<uchar> status;
            vector<float> err;

            goodFeaturesToTrack(prev_grey, prev_corner, 100, 0.02, 25);
            Size winSize = Size(21, 21);
            int maxLevel = 2;
            TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 25, 0.02);
            calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_corner, cur_corner, status, err, winSize, maxLevel, criteria);

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
            T = estimateRigidTransform(prev_corner2, cur_corner2, false); // false = rigid transform, no scaling/shearing

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

            // Accumulated frame to frame transform
            x += dx;
            y += dy;
            a += da;

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

            // target - current
            double diff_x = X.x - x;
            double diff_y = X.y - y;
            double diff_a = X.a - a;

            dx = dx + diff_x;
            dy = dy + diff_y;
            da = da + diff_a;

            T.at<double>(0, 0) = cos(da);
            T.at<double>(0, 1) = -sin(da);
            T.at<double>(1, 0) = sin(da);
            T.at<double>(1, 1) = cos(da);

            T.at<double>(0, 2) = dx;
            T.at<double>(1, 2) = dy;

            if (diff_x > capture_width * WARP_THRESHOLD || diff_y > capture_height * WARP_THRESHOLD)
            {
                out << frame;
            }
            else
            {
                Mat frame_stabilized;
                warpAffine(frame, frame_stabilized, T, frame.size());
                fixBorder(frame_stabilized);
                out << frame_stabilized;
            }

            cur.copyTo(prev);
            cur_grey.copyTo(prev_grey);
            k++;
        }
#endif

        // quit on ESC button
        if (waitKey(1) == 27)
        {
            cap.release();
            out.release();
            destroyAllWindows();
            break;
        }
    }

    return 0;
}
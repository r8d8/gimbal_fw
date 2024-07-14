#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
 
using namespace std;
using namespace cv;


// global structure to remember some states changed
// by mouse action and move
struct initRoi {
  // on off
  int init;

  //initial coordination based on EVENT_LBUTTONDOWN
  int initX;
  int initY;

  // actual coordination 
  int actualX;
  int actualY;

  //Selected Rect
  Rect roiRect; 

  //Selected Mat roi
  Mat takenRoi;
} SelectedRoi;

//event int is compared to determine action of the mouse EVENT_RBUTTONDOWN
// EVENT_LBUTTONDOWN, EVENT_LBUTTONUP, EVENT_MOUSEMOVE.
// If the event is evauated as happened the global structure SelectedRoi
// is updated.
static void MouseCallBack(int event, int x, int y, int flags, void* img) {
//Mouse Right button down
  if (event == EVENT_RBUTTONDOWN) {
    cout << "right button " << endl;
    return;
  }
//Mouse Left button down
  if (event == EVENT_LBUTTONDOWN) {
    SelectedRoi.initX = x;
    SelectedRoi.initY = y;
    SelectedRoi.init = 1;
    cout << "left button DOWN" << endl; 
    return;
  }
//Mouse Left button up
  if (event == EVENT_LBUTTONUP) {
    SelectedRoi.actualX = x;
    SelectedRoi.actualX = y;
    cout << "left button UP" << endl;
    return;
  }
//Mouse move coordinates update
  if (event == EVENT_MOUSEMOVE) {
  
     cout << "event mouse move"<< endl; 
      SelectedRoi.actualX = x;
      SelectedRoi.actualY = y;
      SelectedRoi.roiRect = Rect(SelectedRoi.initX, SelectedRoi.initY,
   SelectedRoi.actualX,  SelectedRoi.actualY);
    return;
  }
}

bool isROISelected(initRoi* r) {
    return (r->actualX-r->initX > 0) && (r->actualY-r->initY > 0); 
}

int main( int argc, char** argv ){
 // show help
 if(argc<2) {
        cout<<
        " Usage: tracker <video_name>\n"
        " examples:\n"
        " example_tracking_kcf Bolt/img/%04d.jpg\n"
        " example_tracking_kcf faceocc2.webm\n"
        << endl;
        return 0;
    }  
 
    // declares all required variables
    SelectedRoi.init = 0;
    Mat frame;
    
    // create a tracker object
    Ptr<Tracker> tracker = TrackerKCF::create();
    
    VideoCapture cap;
    int deviceID = 0; // 0 = open default camera
    int apiID = cv::CAP_ANY; // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }   
    namedWindow("tracker", WINDOW_AUTOSIZE);
    setMouseCallback("tracker", MouseCallBack, 0);
    
    // perform the tracking process
    printf("Start the tracking process, press ESC to quit.\n");
    for ( ;; ) {
        
        // get bounding box
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        //quit if ROI was not selected
        if (isROISelected(&SelectedRoi)) {
            // initialize the tracker
            tracker->init(frame, SelectedRoi.roiRect);
            
            // update the tracking result
            tracker->update(frame, SelectedRoi.roiRect);
            
            // draw the tracked object
            rectangle( frame, SelectedRoi.roiRect, Scalar( 255, 0, 0 ), 2, 1 );
        }
        
        // show image with the tracked object
        imshow("tracker", frame);
        
        //quit on ESC button
        if (waitKey(1) == 27) break;
    }
    
    return 0;
}
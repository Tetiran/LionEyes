#include "MNN_UltraFace.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <dlib/opencv.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h> 
#include <unistd.h> 
#include <math.h>
#include <stdlib.h>
#include <sys/time.h> 

//using namespace dlib;

#define FOV_X 62.2
#define FOV_Y 48.8
#define XLEN 854.0
#define YLEN 480.0
using namespace std;
#define MAX_DROP 20
#define FACE_DISTANCE 50.0

#define FRAMESTILLSLEEP 200
int MissedFrames = 0;

float lastpos[2];
float TargetFace[4];
int tracking_state = 0;

void calculate_angles(dlib::full_object_detection det, int serial);
void serial_send(float x, float y, float r);
void change_event(int eventcode);

int64_t LastEventTime;
int NextBlink = 0;

int FramesSinceSeen = 0;

int64_t currentTimeMillis();

int rollrand(int lower, int upper);

void UpdateSleep();

bool Sleep = false;
bool LastSleepState = false;

int serial_port;

int main(int argc, char **argv) {
    float f =0;
    float FPS[16];
    int i, Fcnt=0;
    int framerate = 60;
    cv::Mat frame;

    dlib::shape_predictor pose_model;
    dlib::deserialize("shape_predictor_5_face_landmarks.dat") >> pose_model;

    chrono::steady_clock::time_point Tbegin, Tend, Tloop, TlastEvent;
    for(i=0;i<16;i++) FPS[i]=0.0;


    serial_port = open("/dev/ttyUSB0", O_RDWR);

    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }
    // Create new termios struct, we call it 'tty' for convention
    // No need for "= {0}" at the end as we'll immediately write the existing
    // config to this struct
    struct termios tty;

    // Read in existing settings, and handle any error
    // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
    // must have been initialized with a call to tcgetattr() overwise behaviour
    // is undefined
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    UltraFace ultraface("slim-320-quant-ADMM-50.mnn", 320, 240, 4, 0.65); // config model input
    const char* gst = "nvarguscamerasrc  ! video/x-raw(memory:NVMM), format=(string)NV12, width=(int)854, height=(int)480, framerate=(fraction)30/1 ! \
			nvvidconv         ! video/x-raw,              format=(string)BGRx !  \
            videoflip method=rotate-180 ! \
			videoconvert      ! video/x-raw,              format=(string)BGR  ! \
			appsink";

    cv::VideoCapture cap(gst);
    // internal buffer will now store only 3 frames
    //nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! video/x-raw,width=(int)960, height=(int)616, ! appsink
    //nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)I420, framerate=(fraction)24/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink
    //gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! 'video/x-raw(memory:NVMM),width=3820, height=2464, framerate=21/1, format=NV12' ! nvvidconv flip-method=0 ! 'video/x-raw,width=960, height=616' ! nvvidconv ! nvegltransform ! nveglglessink -e
    if (!cap.isOpened()) {
        cerr << "ERROR: Unable to open the camera" << endl;
        return 0;
    }
    cout << "Start grabbing, press ESC on Live window to terminate" << endl;
    Tloop = chrono::steady_clock::now();

    LastEventTime = currentTimeMillis();


    srand(time(0));

    change_event(10);

    while(1){
        char read_buf [256];

        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
        int n = read(serial_port, &read_buf, sizeof(read_buf));
        printf("%.*s",n,read_buf);
        float freq = 1000.0/(chrono::duration_cast <chrono::milliseconds> (Tloop - Tbegin).count());
        //printf("freq %0.2f\n", freq);
        if (freq > 1.0 && freq < framerate){
            int drop = 1;
            while(drop * freq < framerate){
                cap >> frame;
                drop++;
            }
            //printf("dropping %d frames\n", drop-1);
        } else{
            cap >> frame;
        }
        //cap >> frame;
        
        Tbegin = chrono::steady_clock::now();

        if (frame.empty()) {
            cerr << "ERROR: Unable to grab from the camera" << endl;
            break;
        }

        vector<FaceInfo> face_info;
        ultraface.detect(frame, face_info);
        dlib::cv_image<dlib::bgr_pixel> cimg(frame);
        float lastdist = 10000;

        bool FoundTarget = false;
        
        //printf("lastpos at x: %0.2f, y: %0.2f\n", lastpos[0], lastpos[1]);

        for (auto face : face_info) {
            FramesSinceSeen = 0;
            Sleep = false;
            cv::Point pt1(face.x1, face.y1);
            cv::Point pt2(face.x2, face.y2);
            cv::rectangle(frame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
            float x = (face.x1+face.x2)/2.0;
            float y = (face.y1+face.y2)/2.0;
            //printf("face at x: %0.2f, y: %0.2f\n", x, y);
            if (tracking_state == 0){
                tracking_state = 1;
                lastpos[0] = x;
                lastpos[1] = y;
                FoundTarget = true;
                TargetFace[0] = face.x1;
                TargetFace[1] = face.y1;
                TargetFace[2] = face.x2;
                TargetFace[3] = face.y2;

            } else if (tracking_state == 1){
                float dist = sqrt( pow((lastpos[0] - x),2) + pow((lastpos[1] - y),2));
                if (dist < lastdist && dist < FACE_DISTANCE){
                    //printf("chosing targetface at distnace %0.2f x: %0.2f y: %0.2f\n", dist, x, y);
                    FoundTarget = true;
                    TargetFace[0] = face.x1;
                    TargetFace[1] = face.y1;
                    TargetFace[2] = face.x2;
                    TargetFace[3] = face.y2;
                    lastdist = dist;
                }
            }
        }

        if(FoundTarget == false){
            FramesSinceSeen++;
            if(FramesSinceSeen > FRAMESTILLSLEEP){
                Sleep = true;
            }
        }
        UpdateSleep();
        

        if (tracking_state == 1 && FoundTarget == false){
            MissedFrames++;
        }

        if (MissedFrames >= MAX_DROP){
            MissedFrames = 0;
            tracking_state = 0;
        }
        //printf("missed frames %d\n", MissedFrames);
        //printf("tracking state %d\n", tracking_state);
        

        if (FoundTarget){
            //printf("target face x: %0.2f, y: %0.2f\n", (TargetFace[0]+TargetFace[2])/2.0, (TargetFace[1]+TargetFace[3])/2.0);
            MissedFrames = 0;
            lastpos[0] = (TargetFace[0]+TargetFace[2])/2.0;
            lastpos[1] = (TargetFace[1]+TargetFace[3])/2.0;
            dlib::rectangle rect { (long int) TargetFace[0], (long int) TargetFace[1], 
            (long int) TargetFace[2], (long int) TargetFace[3]};
            dlib::full_object_detection det = pose_model(cimg, rect);
            cv::circle(frame, cv::Point(det.part(1).x(), det.part(1).y()), 2, cv::Scalar(0,255,0), 1, 8, 0);
            cv::circle(frame, cv::Point(det.part(3).x(), det.part(3).y()), 2, cv::Scalar(0,255,0), 1, 8, 0);
            calculate_angles(det, serial_port);
        }
        if(!Sleep){
            if (LastEventTime + NextBlink < currentTimeMillis()){
                int event = 10;
                int prob = rollrand(0,100);
                if(prob > 90){
                    event = 13;
                } else if (prob <= 90 && prob > 80){
                    event = 14;
                } else if (prob <= 80 && prob > 60){
                    event = 12;
                } else {
                    event = 11;
                }

                NextBlink = rollrand(3000, 8000);
                
                LastEventTime = currentTimeMillis();
                change_event(event);
            }
        }

        /*
        f = chrono::duration_cast <chrono::milliseconds> (Tend - Tbegin).count();
        if(f>0.0) FPS[((Fcnt++)&0x0F)]=1000.0/f;
        for(f=0.0, i=0;i<16;i++){ f+=FPS[i]; }
        printf("FPS %0.2f\n", f/16);
        cv::putText(frame, cv::format("FPS %0.2f", f/16),cv::Point(10,20),cv::FONT_HERSHEY_SIMPLEX,0.6, cv::Scalar(0, 0, 255));
        */
        cv::imshow("Capture", frame);
        char esc = cv::waitKey(1);
        if(esc == 27) break;
        Tloop = chrono::steady_clock::now();
    }
    cv::destroyAllWindows();
    return 0;
}

void UpdateSleep(){
    if(LastSleepState != Sleep){
        LastSleepState = Sleep;
        printf("Changing sleep state to %d", Sleep);
        if(Sleep){
            change_event(2);
        } else{
            change_event(3);
        }
    }
}

int64_t currentTimeMillis() {
  struct timeval time;
  gettimeofday(&time, NULL);
  int64_t s1 = (int64_t)(time.tv_sec) * 1000;
  int64_t s2 = (time.tv_usec / 1000);
  return s1 + s2;
}

int rollrand(int lower, int upper) {
    return (rand() % (upper - lower + 1)) + lower;
}


void calculate_angles(dlib::full_object_detection det, int serial){
    //printf("left_eye %ld  %ld\n", det.part(1).x(), det.part(1).y());
    //printf("right_eye %ld  %ld\n", det.part(3).x(), det.part(3).y());
    float center_x = (det.part(1).x() + det.part(3).x())/2.0;
    float center_y = (det.part(1).y() + det.part(3).y())/2.0;

    float theta_x = (center_x/XLEN) * FOV_X - FOV_X/2.0;
    float theta_y = (center_y/YLEN) * FOV_Y - FOV_Y/2.0;
    float rotation = atan2( det.part(1).y()- det.part(3).y(), det.part(1).x() -det.part(3).x() ) * 180 / M_PI;
    //printf("theta_x %0.2f theta_y %0.2f rotation %0.2f\n", theta_x, theta_y, rotation);
    serial_send(theta_x, theta_y, rotation);
}

void change_event(int eventcode){
    printf("sending event code %d \n", eventcode);
    char msg[10];
    int n = sprintf(msg, "e%d", eventcode);
    write(serial_port, msg, n+1);
}

void serial_send(float x, float y, float r){
    char msg[10];
    int n = sprintf(msg, "x%.2f", -x);
    write(serial_port, msg, n+1);
    n = sprintf(msg, "y%.2f", -y);
    write(serial_port, msg, n+1);
    n = sprintf(msg, "f%.2f", -r);
    write(serial_port, msg, n+1);
}

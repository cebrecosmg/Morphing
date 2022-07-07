#pragma once

#include "ofMain.h"
#include "ofxCvHaarFinder.h"
#include "ofxOpenCv.h"
#include "ofxPS3EyeGrabber.h"

class FaceDetection {
public:
    FaceDetection();
    void setup();
    void update();
    void setupInstallation();
    void installation();
    void nFacesDetected();//No. of faces detetcted at installation
    void detection(bool _drawRect);// loop over found faces and draw a square
    void test();
    vector<ofPoint> facePosition();
    vector<float>faceSize();
    
    // set up object instances
    ofImage img;
    ofVideoGrabber videoGrabber;
    ofxCvHaarFinder finder;
    ofRectangle cur;
    
    //array of centre points
    vector<float>faceSizes;
//    float faceSize;
    
    //array of face sizes
    vector<ofPoint>points;
    ofPoint point;
    
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage init;
    ofxCvGrayscaleImage reSize;
    
    //Program variables
    float millis;
    bool faceDet;
    int proximity;
    int nFacesPresent;
    int counter;
  
    
};

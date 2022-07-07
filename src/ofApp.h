#pragma once

/*
This code creates video morphing, through the use of ulploaded images, ofxOpenCv addon, OpenCV, and ofxKinect.
 
It computes optical flow between two images, and use them to morphing some other image.
You can choose what image to morph:
- first input image (key '1')
- checkerboard image (key '2'). This one is use to see the area affected or distorted
Images in the kinect are been translated to pixels and the closest point is calculated by and average of the brightest pixels capture. The code gets the location of it (x,y) and uses the x-position to set morphing values.

--------------------------------------
This code is partially based in the the example 05-VideoMorphing from the book
"Mastering openFrameworks: Creative Coding Demystified",
Chapter 9 - Computer Vision with OpenCV

*/

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "FaceDetection.hpp"

using namespace cv;

class ofApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();
    void installationSetup();
    void mousePressed(int x, int y, int button);
    
    ofImage imageOf1, imageOf2;    


	ofxCvColorImage color1, color2;		//First and second original images
	ofxCvGrayscaleImage gray1, gray2;	//Decimated grayscaled images
	ofxCvFloatImage flowX, flowY;		//Resulted optical flow in x and y axes

	ofxCvColorImage colorTest;			//Input image "checkerboard" for morphing

	int w, h;							//Decimated size of input images

	ofxCvGrayscaleImage planeX, planeY;

	ofxCvFloatImage idX, idY;	//idX(x,y) = x, idY(x,y) = y
	ofxCvFloatImage mapX, mapY;
	ofxCvFloatImage bigMapX, bigMapY;

	ofxCvFloatImage fx, fy;
	ofxCvFloatImage weight;

	float morphValue;	//[0, 1]
	ofxCvColorImage morph;	    //Resulted morphed image
	int morphImageIndex;		//What to morph: 1 - first input image, 2 - checkerboard image

	//Inverting the mapping (mapX, mapY), with antialiasing.
	void inverseMapping( ofxCvFloatImage &mapX, ofxCvFloatImage &mapY );

	//Making image morphing
	void updateMorph( float morphValue, int morphImageIndex );

	void keyPressed(int key);
	void location(int x, int y );
    
    
    //KINECT
//    void exit();
//
//    ofxKinect kinect;
//    
//    ofxCvColorImage colorImg;
//    ofxCvGrayscaleImage grayImage; // grayscale depth image
//    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
//    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
//    
//    ofxCvContourFinder contourFinder;
    
    bool bThreshWithOpenCV;
    //bool bDrawPointCloud;
    
    int nearThreshold;
    int farThreshold;
    
    int angle; // angle of the camera

    int threshold;
    float closestColorX, closestColorY; //declatering variables for the closest pixel
    int count; // to calculate an averge
    
    // face detection
    FaceDetection faceDetection;
    
    // installation / run toggle
    bool debug;

};


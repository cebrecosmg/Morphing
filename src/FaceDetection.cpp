//
//  FaceDetection program referenced from OFX computer vision
//
//  Created by Chris  on 16/04/2021.
//

#include "FaceDetection.hpp"



//--------------------------------------------------------------
FaceDetection::FaceDetection(){

    finder.setup("haarcascade_frontalface_default.xml");
    
    videoGrabber.setGrabber(std::make_shared<ofxPS3EyeGrabber>());
    //videoGrabber.setup(640, 480);

    //videoGrabber.setDeviceID(0);
    videoGrabber.setDesiredFrameRate(30);
    videoGrabber.initGrabber(320, 240);
    videoGrabber.setUseTexture(false);  // add then you won't see the video, pi optimisation
    finder.setScaleHaar(1.1);
    bool faceDet = false;
    init.allocate(320, 240);// resize for pi optimisation
    reSize.allocate(160, 120);// resize for pi optimisation
}

//--------------------------------------------------------------
void FaceDetection::update(){
    
    
    
    videoGrabber.update();

    if(videoGrabber.isFrameNew()){
        colorImg.setFromPixels(videoGrabber.getPixels());
        init = colorImg;
        
        //assumes reSize is allocated to be a smaller size than init
        reSize.scaleIntoMe(init);// resize video grabber image to optimise raspberry pi
        img.setFromPixels(reSize.getPixels());
        finder.findHaarObjects(img); // face recognition update frames
        nFacesPresent = finder.blobs.size(); // update faces recognised
    }
    

}

//setup installation --------------------------------------------
void FaceDetection::setupInstallation(){ // draws box around face for setup purposes and debug
    ofSetColor(255);
    ofNoFill();
    img.draw(0, 0);
    detection(1);
    
   
    
    
    if(finder.blobs.size() > 0 ) {
        
        faceDet = true;
        millis = ofGetElapsedTimeMillis();
    }
    
    if (ofGetElapsedTimeMillis() > (millis + 4000))
    {
        faceDet = false;
        proximity = 0;
    }
    
    
    
   // cout<<faceDet<<endl;
    


}

//--------------------------------------------------------------
void FaceDetection::installation(){// Installation function and no box drawn
    ofSetColor(255);
    ofNoFill();
    detection(0);
    
    
    // is there a face present boolean
    if(finder.blobs.size() > 0 ) {
        faceDet = true;
        millis = ofGetElapsedTimeMillis();
    }
    
    
    if (ofGetElapsedTimeMillis() > (millis + 2000))
    {
        faceDet = false;
    }
    
    
    
//    cout<<faceDet<<endl;
    
}


//--------------------------------------------------------------
void FaceDetection::detection(bool _drawRect){
    ofSetLineWidth(3);
    ofSetColor(0);
    for(int i = 0; i < finder.blobs.size(); i++) { // loop over all the found faces/blobs
        cur = finder.blobs[i].boundingRect;     // and put a rectangle around the face
        proximity = ofMap (finder.blobs[i].boundingRect.width, 15, 80, 0,10); // map closeness to installation
        

        cout <<finder.blobs[0].boundingRect.width<<endl;//log face size and no. of faces
        cout<<finder.blobs.size()<<endl;
        if( _drawRect) ofDrawRectangle(cur);
       
    }
}


//No. of faces detetcted at installation------------------------
void FaceDetection::nFacesDetected(){
    
    if (counter%60==0){ //using modulo to occasionally check for faces
        nFacesPresent = finder.blobs.size();
    }
    counter++;
}
//function to return centre points as an array-------------------
vector<ofPoint> FaceDetection::facePosition(){
    for(int i = 0; i < finder.blobs.size(); i++) { // loop over all the found faces/blobs
    point = finder.blobs[i].centroid;     // and put a rectangle around the face
        points.push_back(point);
    }
    return points;
}

//function to return proxitiy/facesize as an array--------------------
vector<float> FaceDetection::faceSize(){
    for(int i = 0; i < finder.blobs.size(); i++) { // loop over all the found faces/blobs
        // map closeness to installation
        proximity = ofMap (finder.blobs[i].boundingRect.width, 15, 80, 0,10);
        faceSizes.push_back(proximity);
    }
    return faceSizes;
}




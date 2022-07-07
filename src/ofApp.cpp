#include "ofApp.h"
//#include "tracking.hpp"
#include "opencv2/opencv.hpp"
#include "ofxKinect.h"

//--------------------------------------------------------------
void ofApp::setup()
{
        ofSetFrameRate(30);
    
    debug = true; // initialise sketch in debug mode

   
    //MORPHING IMAGE
		
	imageOf1.loadImage("hands1.png");
	imageOf2.loadImage("hands2.png");

	color1.setFromPixels( imageOf1 );	//Convert to ofxCv images
	color2.setFromPixels( imageOf2 );

	float decimate = 0.3;              //Decimate images to 30%
	ofxCvColorImage imageDecimated1;
	imageDecimated1.allocate( color1.width * decimate,
                          color1.height * decimate );
	//High-quality resize
	imageDecimated1.scaleIntoMe( color1, CV_INTER_AREA );
	gray1 = imageDecimated1;

	ofxCvColorImage imageDecimated2;
	imageDecimated2.allocate( color2.width * decimate,
		                      color2.height * decimate );
	//High-quality resize
	imageDecimated2.scaleIntoMe( color2, CV_INTER_AREA );
	gray2 = imageDecimated2;


	Mat img1 = cvarrToMat( gray1.getCvImage() );  //Create OpenCV images
	Mat img2 = cvarrToMat( gray2.getCvImage() );  //Create OpenCV images
	Mat flow;                        //Image for flow
	//Computing optical flow
	  calcOpticalFlowFarneback( img1, img2, flow, 0.7, 3, 11, 5, 5, 1.1, 0 );
	//Split flow into separate images
	vector<Mat> flowPlanes;
	split( flow, flowPlanes );
	//Copy float planes to ofxCv images flowX and flowY
	IplImage iplX( flowPlanes[0] );
	flowX = &iplX;
	IplImage iplY( flowPlanes[1] );
	flowY = &iplY;

	//--------------------------------------------------------------------------
	//ATTENTION: Lines flowX = &iplX; and flowY = &iplY; can raise runtime error,
	//caused by small bug in ofxOpenCV.
	//So before running the example, fix it, as it described in ofApp.h file
	//--------------------------------------------------------------------------

	w = gray1.width;
	h = gray1.height;

	//Flow image
	planeX = flowX;
	planeY = flowY;

	//create idX, idy
	idX.allocate( w, h );
	idY.allocate( w, h );
	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			idX.getPixelsAsFloats()[ x + w * y ] = x;
			idY.getPixelsAsFloats()[ x + w * y ] = y;
		}
	}

	//Load checkerboard image
	ofImage imageTest;
	imageTest.loadImage("checkerBoard.png");
	colorTest.setFromPixels( imageTest );

	//Make morphing at first time
	morphValue = 0;
	morphImageIndex = 1;
	updateMorph( morphValue, morphImageIndex );
}

//--------------------------------------------------------------
void multiplyByScalar( ofxCvFloatImage &floatImage, float value )
{
	int w = floatImage.width;
	int h = floatImage.height;
	float *floatPixels = floatImage.getPixelsAsFloats();
	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			floatPixels[ x + w * y ] *= value;
		}
	}
	floatImage.flagImageChanged();
}

//--------------------------------------------------------------
void ofApp::update(){
//Update face detection
    faceDetection.update(); // Update the face detector
    
    
    vector<ofPoint>points = faceDetection.facePosition();
    for(int i = 0; i<points.size(); i++){
        closestColorX =  ofMap(points[i].x, 0, 160, 0, ofGetWidth());
        closestColorY =  ofMap(points[i].y, 0, 120, 0, ofGetHeight());
        
        // ++closestColorX, closestColorY change these++ add xy positon of blob into location
                location(closestColorX, closestColorY); // REVISAR ACA ESTA LA JARANA

    }
  




}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground( 255, 255, 255);	//Set the background color
    //KINECT CODE
    if (debug){ // debug mode for setting up installation,
        installationSetup();
    } else {
      
    // draw instructions
    ofSetColor(255, 255, 255);
   
    //___END KINECT
    //MORPH IMAGE CODE
	int w = gray1.width;
	int h = gray1.height;

        //Input images + optical flow:
        morph.draw( 0, 0 );         //This draws the final Morphed IMAGE

	//Output text with current morhping value
	ofSetColor( 0, 0, 0 );
	ofDrawBitmapString( "Morphing value: " + ofToString( morphValue * 100, 1 ) + "%", morph.getWidth() + 20, ofGetHeight() - 20 ); // prints out the Morphing value
    
    ofDrawCircle(closestColorX, closestColorY, 20); // draw red dot to test, debug
}

}

//--------------------------------------------------------------
//Making image morphing
void ofApp::updateMorph( float morphValue, int morphImageIndex )
{
	mapX.allocate( w, h );			//w and h is size of gray1 image
	mapY.allocate( w, h );

	//Get pointers to pixels data
	float *flowXPixels = flowX.getPixelsAsFloats();
	float *flowYPixels = flowY.getPixelsAsFloats();
	float *mapXPixels = mapX.getPixelsAsFloats();
	float *mapYPixels = mapY.getPixelsAsFloats();
	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			int i = x + w * y;		//pixels' index
			mapXPixels[ i ] = x + flowXPixels[ i ] * morphValue;
			mapYPixels[ i ] = y + flowYPixels[ i ] * morphValue;
		}
	}
	mapX.flagImageChanged();		//Notify that pixels values were changed
	mapY.flagImageChanged();

	inverseMapping( mapX, mapY );

	//Create big maps
	int W = color1.width;
	int H = color1.height;
	if ( !bigMapX.bAllocated ) {
		bigMapX.allocate( W, H );
		bigMapY.allocate( W, H );
	}
	//bigMapX and bigMapY have type ofxCvFloatImage
	bigMapX.scaleIntoMe( mapX, CV_INTER_LINEAR );
	bigMapY.scaleIntoMe( mapY, CV_INTER_LINEAR );
	multiplyByScalar( bigMapX, 1.0 * W / w );
	multiplyByScalar( bigMapY, 1.0 * H / h );

	//Do warping

	//Select image to morph
	if ( morphImageIndex == 1 ) {
		morph = color1;		//First input image
	}
	else {
		morph = colorTest;	//Checkerboard image
	}
	morph.remap( bigMapX.getCvImage(), bigMapY.getCvImage() );

}

//--------------------------------------------------------------
//Inverting the mapping (mapX, mapY), with antialiasing.
//TODO: probably there is a simpler way to do this.
void ofApp::inverseMapping( ofxCvFloatImage &mapX, ofxCvFloatImage &mapY ){
	if ( !fx.bAllocated ) {
		fx.allocate( w, h );
		fy.allocate( w, h );
		weight.allocate( w, h );
	}
	fx.set( 0 );
	fy.set( 0 );
	weight.set( 0 );

	float *mapXPixels = mapX.getPixelsAsFloats();
	float *mapYPixels = mapY.getPixelsAsFloats();
	float *fxPixels = fx.getPixelsAsFloats();
	float *fyPixels = fy.getPixelsAsFloats();
	float *weightPixels = weight.getPixelsAsFloats();

	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			float MX = mapXPixels[ x + w * y ];
			float MY = mapYPixels[ x + w * y ];

			int mx0 = int( MX );
			int my0 = int( MY );
			int mx1 = mx0 + 1;
			int my1 = my0 + 1;
			float weightX = MX - mx0;
			float weightY = MY - my0;

			mx0 = ofClamp( mx0, 0, w-1 );	//Bound
			my0 = ofClamp( my0, 0, h-1 );
			mx1 = ofClamp( mx1, 0, w-1 );
			my1 = ofClamp( my1, 0, h-1 );
			for (int b=0; b<2; b++) {
				for (int a=0; a<2; a++) {
					int x1 = ( a == 0 ) ? mx0 : mx1;
					int y1 = ( b == 0 ) ? my0 : my1;
					int i1 = x1 + w * y1;
					float wgh = ( ( a == 0 ) ? ( 1 - weightX ) : weightX )
						* ( ( b == 0 ) ? ( 1 - weightY ) : weightY );
					fxPixels[ i1 ] += x * wgh;
					fyPixels[ i1 ] += y * wgh;
					weightPixels[ i1 ] += wgh;
				}
			}
		}
	}

	//Compute map for non-zero weighted pixels
	int zeros = 0;		//Count of zeros pixels
	for (int y=0; y<h; y++) {
		for (int x=0; x<w; x++) {
			int i = x + w * y;
			float X = fxPixels[ i ];
			float Y = fyPixels[ i ];
			float Weight = weightPixels[ i ];
			if ( Weight > 0 ) {
				X /= Weight;
				Y /= Weight;
			}
			else {
				X = x;
				Y = y;
				zeros++;
			}
			mapXPixels[ i ] = X;
			mapYPixels[ i ] = Y;
			weightPixels[ i ] = Weight;
		}
	}

	//Fill zero-weighted pixels by weighting of near non-zero weighted pixels
	const int rad = 2; //ERA 2
	const int diam = 2 * rad + 1;
	float filter[ diam * diam ];
	float sum = 0;
	for (int b=-rad; b<=rad; b++) {
		for (int a=-rad; a<=rad; a++) {
			float wgh = rad + 1 - max( abs( a ), abs( b ) );
			filter[ a+rad + diam * (b+rad) ] = wgh;
			sum += wgh;
		}
	}
	for (int i=0; i<diam*diam; i++) {
		filter[ i ] /= sum;
	}

	int zeros0 = -1;
	while ( zeros > 0 && (zeros0 == -1 || zeros0 > zeros) ) {
		zeros0 = zeros;
		zeros = 0;
		for (int y=0; y<h; y++) {
			for (int x=0; x<w; x++) {
				int i = x + w * y;
				if (weightPixels[ i ] < 0.0001 ) {
					float mX = 0;
					float mY = 0;
					float mWeight = 0;
					int x1, y1, i1;
					float wgh;
					for (int b = -rad; b<=rad; b++) {
						for (int a = -rad; a<=rad; a++) {
							x1 = x + a;
							y1 = y + b;
							if ( ofInRange( x1, 0, w-1 ) && ofInRange( y1, 0, h-1 ) ) {
								i1 = x1 + w * y1;
								if ( weightPixels[ i1 ] >= 0.0001 ) {
									wgh = filter[ a+rad + diam * (b+rad) ] * weightPixels[ i1 ];
									mX += mapXPixels[i1] * wgh;
									mY += mapYPixels[i1] * wgh;
									mWeight += wgh;
								}
							}
						}
					}
					if ( mWeight > 0 ) {
						mapXPixels[ i ] = mX / mWeight;
						mapYPixels[ i ] = mY / mWeight;
						weightPixels[ i ] = mWeight;
					}
					else {
						zeros++;
					}
				}
			}
		}
	}

	mapX.flagImageChanged();
	mapY.flagImageChanged();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if ( key == '1' ) {		//Select first image to morph
		morphImageIndex = 1;
		updateMorph( morphValue, morphImageIndex );
	}
	if ( key == '2' ) {		//Select checkerboard image to morph
		morphImageIndex = 2;
		updateMorph( morphValue, morphImageIndex );
	}
}

//--------------------------------------------------------------
void ofApp::location(int x, int y ){
	morphValue = ofMap(x, 0, ofGetWidth(), 0, 2 ); // Last number will distort more the image, the bigger it is to 1. X now need to be the brightness
    // 10 is an interesting distortion
    cout << "value: " << x << endl;
    //0 o 60 a 560
	updateMorph( morphValue, morphImageIndex );
}



//Set up function click mouse to exit ----------------------------
void ofApp::installationSetup(){
    
    faceDetection.setupInstallation(); //face detect call drawing video
  //  counter ++;         // Counter for modulo
}

// Toggle between setup and Installation----------------------------
void ofApp::mousePressed(int x, int y, int button){
    debug = !debug;
}


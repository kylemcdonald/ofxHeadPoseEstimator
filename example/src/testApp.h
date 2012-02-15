#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxHeadPoseEstimator.h"

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void updateCloud();
		void draw();
		//------------------------
		void setupEstimator();
		//-----------------------
		void drawPointCloud();
        void drawPoses();
		//-----------------------
        void calcAvgFPS();
        void drawReport();
		//-----------------------
		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		//-----------------------
		ofxKinect kinect;
		ofEasyCam easyCam;
};

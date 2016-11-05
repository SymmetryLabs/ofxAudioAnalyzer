#pragma once

#include "ofMain.h"
#include "ofxAudioAnalyzer.h"

#include "ofSoundPlayerExtended.h"
#include "ofxGui.h"

#include "ofxFilterbank.h"      //for filterbank

//---- #defs for filterbank
#define BANDWIDTH  1.0
#define BUFFER_SIZE 512         //changed to 1024
#define LIN2dB(x) (double)(20. * log10(x))
#define SR 44100
#define bufferSize 512


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();
    
        void audioIn(ofSoundBuffer &inBuffer);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

        ofSoundStream soundStream;
        ofxAudioAnalyzer audioAnalyzer;
        ofxFilterbank filterBank; //for filterbank
    
        float rms_l, rms_r;
        float smooth;
    
        float rms;
        float power;
        float pitchFreq;
        float pitchFreqNorm;
        float pitchConf;
        float pitchSalience;
        float hfc;
        float hfcNorm;
        float specComp;
        float specCompNorm;
        float centroid;
        float centroidNorm;
        float inharmonicity;
        float dissonance;
        float rollOff;
        float rollOffNorm;
        float oddToEven;
        float oddToEvenNorm;
        float strongPeak;
        float strongPeakNorm;
        float strongDecay;
        float strongDecayNorm;
    
        vector<float> spectrum;
        vector<float> melBands;
        vector<float> mfcc;
        vector<float> hpcp;
    
        vector<float> tristimulus;
    
        bool isOnset;
    
    
        ofxFloatSlider smoothing;
    
		
};

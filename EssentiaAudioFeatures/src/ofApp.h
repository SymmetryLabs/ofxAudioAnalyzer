#pragma once

#include "ofMain.h"
#include "ofxAudioAnalyzer.h"

#include "ofSoundPlayerExtended.h"
#include "ofxGui.h"

#include "ofxFilterbank.h"      //for filterbank
#include "ofxAubio.h"           //for aubio

//---- #defs for filterbank
#define BANDWIDTH  1.0
#define BUFFER_SIZE 512         //changed to 1024
#define LIN2dB(x) (double)(20. * log10(x))
#define SR 44100
#define bufferSize 512


class ofApp : public ofBaseApp{
    
    //-----scaling
    
    float logScale(float scaledOutput, float minVal, float maxVal);
    float inverseLogScale(float scaledOutput, float minVal, float maxVal);
    float normalize(float old, float minVal, float maxVal);
    float trigger(float input);

    //-------essentia
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
        float danceability;
    
        vector<float> spectrum;
        vector<float> melBands;
        vector<float> mfcc;
        vector<float> hpcp;
        vector<float> tristimulus;
    
        bool isOnset;
    
    
        ofxPanel gui;
    
    
        //-----filterbank
    
        ofxFilterbank filterBank;
        ofxFloatSlider smoothing;
    
        //-----aubio
    
        void onsetEvent(float & time);//for aubio
        void beatEvent(float & time); //for aubio
    
    
        private:
            ofxAubioOnset onset;
            ofxAubioPitch pitch;
            ofxAubioBeat beat;
            ofxAubioMelBands bands;
            
            ofxPanel pitchGui;
            ofxFloatSlider midiPitch;
            ofxFloatSlider pitchConfidence;
            
            ofxPanel beatGui;
            bool gotBeat;
            ofxFloatSlider bpm;
            
            ofxPanel onsetGui;
            bool gotOnset;
            ofxFloatSlider onsetThreshold;
            ofxFloatSlider onsetNovelty;
            ofxFloatSlider onsetThresholdedNovelty;
            
            ofxPanel bandsGui;
            ofPolyline bandPlot;

		
};

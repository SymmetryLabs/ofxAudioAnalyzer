
#include "ofApp.h"
#include "ofEventUtils.h"   //for Aubio
#include "asio.hpp"         //for socket integration
#include "oscpkt.hh"        //for OSC Serialzation
#include "ofxAubio.h"       //for aubio
#include <string.h>         //for aubio
#include "ofSoundStream.h"
#include "ofxGui.h"
#include <math.h> 

//---------------------Global Var: Memory Frames ----------------------------

// Each feature has a memory buffer, whose length needs to be manually specified below.
int numMemoryFrames = 30; // the size of the memory array (number of frames it holds, not physical size in bits)
int currFrame; // the frame we are currently on

// There is probably a neater way of initializing these.
float rmsMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float powerMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float pitchFreqMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float pitchConfMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float pitchSaliMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float inharmonicityMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float hfcMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float centroidMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float strongPeakMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float rollOffMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float specCompMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float dissonanceMemory[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//---------------------Global Var: Trigger Function ------------------------------------

// Right now only HFC is being used as a trigger.
// These variables indicate whether the value that is sent out should be the actual
// raw value, or a decayed version of the previous frame's value. 
bool isHFCDelay = false;
float nextHFCVal = 0.7;
// Different features require different threshold values.
float triggerThreshold = 0.8;

//---------------------Global Var: ASIO------------------------------------

std::string s;
using boost::asio::ip::udp;
boost::asio::io_service io_service;
#define PORT "1331"


//---------------------Global Var: OSC-------------------------------------

using namespace oscpkt;
Message msg;
const void * message;
int size;

//---------------------UDP Classes-----------------------------------------

class UDPClient
{
public:
    UDPClient(
              boost::asio::io_service& io_service,
              const std::string& host,
              const std::string& port
              ) : io_service_(io_service), socket_(io_service, udp::endpoint(udp::v4(), 0)) {
        udp::resolver resolver(io_service_);
        udp::resolver::query query(udp::v4(), host, port);
        udp::resolver::iterator iter = resolver.resolve(query);
        endpoint_ = *iter;
    }
    
    ~UDPClient()
    {
        socket_.close();
    }
    
    void send(const std::string& msg) {
        socket_.send_to(boost::asio::buffer(msg, msg.size()), endpoint_);
    }
    
    void send_osc(const void *msg, int size) {
        socket_.send_to(boost::asio::buffer(msg, size), endpoint_);
    }
    
private:
    boost::asio::io_service& io_service_;
    udp::socket socket_;
    udp::endpoint endpoint_;
};


UDPClient client(io_service, "localhost", PORT);


//------------- Normalizing, Log-Scaling, and Trigger Utility Functions ----------------

float ofApp::logScale(float input, float minVal, float maxVal) {
    // For converting from linear to logarithmic scale
    
    // These commented out lines are previous attempts at log-scaling.
    //    float logScaled = (log(input) - log(minVal))/(log(maxVal) - log(minVal));
    //    float logScaled = abs((19980/(log(2)))*log(input) + 20);
    //    float logScaled = abs((maxVal - minVal)*log(input) + maxVal);
    //    float logScaled = 996.015*exp(3*input);
    
    float logScaled = abs((1/log(maxVal/minVal))*log(input) - log(minVal)/log(maxVal/minVal));
    return logScaled;
}

float ofApp::inverseLogScale(float input, float minVal, float maxVal) {
    // For converting from logarithmic to linear scale
    
    // Once again, commented out lines are previous attempts...
    //    float logScaled = abs((maxVal - minVal)*log(input) + maxVal);
    float inverse = pow(10, input);
    return inverse;
}

float ofApp::normalize(float input, float minVal, float maxVal) {
    /*
     * input: audio value, may or may not already be transformed (e.g. log-scaled)
     * minVal: current local min value of input feature
     * maxVal: current local max value of input feature
     */
    if (input == 0) {
        // this ensures that we don't get NaN when no music playing
        return 0;
    }
    float result = (input - minVal) / (maxVal - minVal);
    cout<<"input: "<<input<<endl;
    cout<<"minVal: "<<minVal<<endl;
    cout<<"maxVal: "<<maxVal<<endl;
    cout<<"result: "<<result<<endl;
    return result;
}

//
//float ofApp::trigger(str feature, float input, float featureTriggerThreshold) {
//    enum triggerableFeatures {"hfc" = 1, "centroid" = 2};
//    switch(feature) {
//        case hfc:
//            if (input > featureTriggerThreshold) {
//                isHFCDelay = true;
//                nextHFCVal = hfc * 0.9;
//            } else {
//                if (isHFCDelay) {
//                    hfc = nextHFCVal;
//                    nextHFCVal = hfc * 0.9;
//                } else {
//                    isHFCDelay = false;
//                }
//            }
//            break;
//        case centroid:
//            if (input > featureTriggerThreshold) {
//                isHFCDelay = true;
//                nextHFCVal = hfc * 0.9;
//            } else {
//                if (isHFCDelay) {
//                    hfc = nextHFCVal;
//                    nextHFCVal = hfc * 0.9;
//                } else {
//                    isHFCDelay = false;
//                }
//            }
//            
//    }
//
//
//}

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofBackground(34, 34, 34);
    ofSetFrameRate(20);
    
    int sampleRate = 44100;
    int outChannels = 0;
    int inChannels = 2;
    
    gui.setup();
    gui.setPosition(20, 150);
    gui.add(smoothing.setup  ("Smoothing", 0.0, 0.0, 1.0));
    
    

    //--------------aubio setup
    
    // setup onset object
    //onset.setup();
    onset.setup("mkl", 2 * bufferSize, bufferSize, sampleRate);
    // listen to onset event
    ofAddListener(onset.gotOnset, this, &ofApp::onsetEvent);
    
    // setup pitch object
    //pitch.setup();
    pitch.setup("yinfft", 8 * bufferSize, bufferSize, sampleRate);
    
    
    // setup beat object
    //beat.setup();
    beat.setup("default", 2 * bufferSize, bufferSize, sampleRate);
    // listen to beat event

    // setup mel bands object
    bands.setup();
    ofAddListener(beat.gotBeat, this, &ofApp::beatEvent);
    
    //--------------essentia setup
    
    // setup the sound stream
    soundStream.setup(this, outChannels, inChannels, sampleRate, bufferSize, 3);
    
    //setup ofxAudioAnalyzer with the SAME PARAMETERS
    audioAnalyzer.setup(sampleRate, bufferSize, inChannels);
    
    
    //--------------filterbank setup
    
    ofSetVerticalSync(true);
    ofBackground(54, 54, 54);
    ofSetFrameRate(20);
    
    int ticksPerBuffer = 8;
    
    int midiMin = 21;
    int midiMax = 108;
    
    filterBank.setup(bufferSize, midiMin, midiMax, inChannels, BANDWIDTH, sampleRate, 1.0);
    filterBank.setColor(ofColor::orange);


    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    
    
    smooth = ofClamp(ofGetMouseX() / (float)ofGetWidth(), 0.0, 1.0);
    
    //get the analysis values
    rms_l = audioAnalyzer.getValue(RMS, 0, smooth);
    rms_r = audioAnalyzer.getValue(RMS, 1, smooth);

    
    //-:get Values:
    rms     = audioAnalyzer.getValue(RMS, 0, smoothing);
    power   = audioAnalyzer.getValue(POWER, 0, smoothing);
    pitchFreq = audioAnalyzer.getValue(PITCH_FREQ, 0, smoothing);
    pitchConf = audioAnalyzer.getValue(PITCH_CONFIDENCE, 0, smoothing);
    pitchSalience  = audioAnalyzer.getValue(PITCH_SALIENCE, 0, smoothing);
    inharmonicity   = audioAnalyzer.getValue(INHARMONICITY, 0, smoothing);
    hfc = audioAnalyzer.getValue(HFC, 0, smoothing);
    specComp = audioAnalyzer.getValue(SPECTRAL_COMPLEXITY, 0, smoothing);
    centroid = audioAnalyzer.getValue(CENTROID, 0, smoothing);
    rollOff = audioAnalyzer.getValue(ROLL_OFF, 0, smoothing);
    oddToEven = audioAnalyzer.getValue(ODD_TO_EVEN, 0, smoothing);
    strongPeak = audioAnalyzer.getValue(STRONG_PEAK, 0, smoothing);
    strongDecay = audioAnalyzer.getValue(STRONG_DECAY, 0, smoothing);
    //Normalized values for graphic meters:
    pitchFreqNorm   = audioAnalyzer.getValue(PITCH_FREQ, 0, smoothing, TRUE);
    hfcNorm     = audioAnalyzer.getValue(HFC, 0, smoothing, TRUE);
    specCompNorm = audioAnalyzer.getValue(SPECTRAL_COMPLEXITY, 0, smoothing, TRUE);
    centroidNorm = audioAnalyzer.getValue(CENTROID, 0, smoothing, TRUE);
    rollOffNorm  = audioAnalyzer.getValue(ROLL_OFF, 0, smoothing, TRUE);
    oddToEvenNorm   = audioAnalyzer.getValue(ODD_TO_EVEN, 0, smoothing, TRUE);
    strongPeakNorm  = audioAnalyzer.getValue(STRONG_PEAK, 0, smoothing, TRUE);
    strongDecayNorm = audioAnalyzer.getValue(STRONG_DECAY, 0, smoothing, TRUE);
    dissonance = audioAnalyzer.getValue(DISSONANCE, 0, smoothing);
    
    //Vector Parameters
    spectrum = audioAnalyzer.getValues(SPECTRUM, 0, smoothing);
    melBands = audioAnalyzer.getValues(MEL_BANDS, 0, smoothing);
    mfcc = audioAnalyzer.getValues(MFCC, 0, smoothing);
    hpcp = audioAnalyzer.getValues(HPCP, 0, smoothing);
    tristimulus = audioAnalyzer.getValues(TRISTIMULUS, 0, smoothing);
    
    //Boolean Parameter
    isOnset = audioAnalyzer.getOnsetValue(0);
    
    //For Testing
    std::cout<<"centroidNorm: "<<centroidNorm<<endl ;
    std::cout<<"strongDecayNorm: "<<strongDecayNorm<<endl ;
    std::cout<<"HFC Essentia Norm: "<<hfcNorm<<endl;
    
    //Adding current audio frame to each feature's MemoryFrame
    rmsMemory[currFrame] = rms;
    powerMemory[currFrame] = power;
    pitchConfMemory[currFrame] = pitchConf;
    pitchFreqMemory[currFrame] = pitchFreq;
    pitchSaliMemory[currFrame] = pitchSalience;
    inharmonicityMemory[currFrame] = inharmonicity;
    hfcMemory[currFrame] = hfc;
    specCompMemory[currFrame] = specComp;
    centroidMemory[currFrame] = centroid;
    strongPeakMemory[currFrame] = strongPeak;
    rollOffMemory[currFrame] = rollOff;
    dissonanceMemory[currFrame] = dissonance;

    // Incrementing the current frame
    if ((currFrame + 1) == numMemoryFrames) {
        currFrame = 0;
    } else {
        currFrame++;
    }
    
    // ------------------ RMS ------------------
    float rmsLocalMax = 0.0;
    float rmsLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (rmsMemory[i] < rmsLocalMin) {
            rmsLocalMin = rmsMemory[i];
        }
        if (rmsMemory[i] > rmsLocalMax) {
            rmsLocalMax = rmsMemory[i];
        }
    }
    cout<<"rms min and rms max: "<<rmsLocalMin<<", "<<rmsLocalMax<<endl;
    float normalizedRMS = normalize(rms, rmsLocalMin, rmsLocalMax);
    cout<<"normalizedRMS: "<<normalizedRMS<<endl;
    rms = normalizedRMS;
    
    // ------------------ POWER ------------------
    float powerLocalMax = 0.0;
    float powerLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (powerMemory[i] < powerLocalMin) {
            powerLocalMin = powerMemory[i];
        }
        if (powerMemory[i] > powerLocalMax) {
            powerLocalMax = powerMemory[i];
        }
    }
    
    float normalizedPower = normalize(power, powerLocalMin, powerLocalMax);
    cout<<"normalizedPower: "<<normalizedPower<<endl;
    power = normalizedPower;
    
    // ------------------ PITCH FREQUENCY ------------------
    float pitchFreqLocalMax = 0.0;
    float pitchFreqLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (pitchFreqMemory[i] < pitchFreqLocalMin) {
            pitchFreqLocalMin = pitchFreqMemory[i];
        }
        if (pitchFreqMemory[i] > pitchFreqLocalMax) {
            pitchFreqLocalMax = pitchFreqMemory[i];
        }
    }
    
    float normalizedPitchFreq = normalize(pitchFreq, pitchFreqLocalMin, pitchFreqLocalMax);
    cout<<"normalizedPitchFreq: "<<normalizedPitchFreq<<endl;
    pitchFreq = normalizedPitchFreq;
    
    // ------------------ PITCH CONFIDENCE ------------------
    float pitchConfLocalMax = 0.0;
    float pitchConfLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (pitchConfMemory[i] < pitchConfLocalMin) {
            pitchConfLocalMin = pitchConfMemory[i];
        }
        if (pitchConfMemory[i] > pitchConfLocalMax) {
            pitchConfLocalMax = pitchConfMemory[i];
        }
    }
    
    float normalizedPitchConf = normalize(pitchConf, pitchConfLocalMin, pitchConfLocalMax);
    cout<<"normalizedPitchConf: "<<normalizedPitchConf<<endl;
    pitchConf = normalizedPitchConf;
    
    // ------------------ PITCH SALIENCE ------------------
    float pitchSaliLocalMax = 0.0;
    float pitchSaliLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (pitchSaliMemory[i] < pitchSaliLocalMin) {
            pitchSaliLocalMin = pitchSaliMemory[i];
        }
        if (pitchSaliMemory[i] > pitchSaliLocalMax) {
            pitchSaliLocalMax = pitchSaliMemory[i];
        }
    }
    
    float normalizedPitchSali = normalize(pitchSalience, pitchSaliLocalMin, pitchSaliLocalMax);
    cout<<"normalizedPitchSali: "<<normalizedPitchSali<<endl;
    pitchSalience = normalizedPitchSali;
    
    // ------------------ INHARMONICITY ------------------
    float inharmonicityLocalMax = 0.0;
    float inharmonicityLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (inharmonicityMemory[i] < inharmonicityLocalMin) {
            inharmonicityLocalMin = inharmonicityMemory[i];
        }
        if (inharmonicityMemory[i] > inharmonicityLocalMax) {
            inharmonicityLocalMax = inharmonicityMemory[i];
        }
    }
    
    float normalizedInharmonicity = normalize(inharmonicity, inharmonicityLocalMin, inharmonicityLocalMax);
    cout<<"normalizedInharmonicity: "<<normalizedInharmonicity<<endl;
    inharmonicity = normalizedInharmonicity;


    // ------------------ HFC ------------------
    float hfcLocalMax = 0.0;
    float hfcLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (hfcMemory[i] < hfcLocalMin) {
            hfcLocalMin = hfcMemory[i];
        }
        if (hfcMemory[i] > hfcLocalMax) {
            hfcLocalMax = hfcMemory[i];
        }
    }
    
    float normalizedHFC = normalize(hfc, hfcLocalMin, hfcLocalMax);
    hfc = normalizedHFC;
    // Makes HFC a trigger
    // Working on implementing this for later
//    hfc = trigger(normalizedHFC);
    
    if (hfc > triggerThreshold) {
        isHFCDelay = true;
        nextHFCVal = hfc * 0.9;
    } else {
        if (isHFCDelay) {
            hfc = nextHFCVal;
            nextHFCVal = hfc * 0.9;
        } else {
            isHFCDelay = false;
        }
    }
    
    // ------------------ SPECTRAL COMPLEXITY ------------------
    // note that there already exists a normalized spectral complexity feature
    // does the current LX app pick up the normalized or non-normalized feature?
    
    float specCompLocalMax = 0.0;
    float specCompLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (specCompMemory[i] < specCompLocalMin) {
            specCompLocalMin = specCompMemory[i];
        }
        if (specCompMemory[i] > specCompLocalMax) {
            specCompLocalMax = specCompMemory[i];
        }
    }
    
    float normalizedSpecComp = normalize(specComp, specCompLocalMin, specCompLocalMax);
    cout<<"normalizedSpecComp: "<<normalizedSpecComp<<endl;
//    float logSpecComp = inverseLogScale(normalizedSpecComp, hfcLocalMin, hfcLocalMax);
    specComp = normalizedSpecComp;
    
    
    // ------------------ CENTROID ------------------
    float centroidLocalMax = 0.0;
    float centroidLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (centroidMemory[i] < centroidLocalMin) {
            centroidLocalMin = centroidMemory[i];
        }
        if (centroidMemory[i] > centroidLocalMax) {
            centroidLocalMax = centroidMemory[i];
        }
    }
    
    float normalizedCentroid = normalize(centroid, centroidLocalMin, centroidLocalMax);
    cout<<"normalizedCentroid: "<<normalizedCentroid<<endl;
    centroid = normalizedCentroid;

    
    // ------------------ DISSONANCE ------------------
    float dissonanceLocalMax = 0.0;
    float dissonanceLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (dissonanceMemory[i] < dissonanceLocalMin) {
            dissonanceLocalMin = dissonanceMemory[i];
        }
        if (dissonanceMemory[i] > dissonanceLocalMax) {
            dissonanceLocalMax = dissonanceMemory[i];
        }
    }
    
    float normalizedDissonance = normalize(dissonance, dissonanceLocalMin, dissonanceLocalMax);
    cout<<"normalizedDissonance: "<<normalizedDissonance<<endl;
    dissonance = normalizedDissonance;
    
    // ------------------ ROLL OFF ------------------
    float rollOffLocalMax = 0.0;
    float rollOffLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (rollOffMemory[i] < rollOffLocalMin) {
            rollOffLocalMin = rollOffMemory[i];
        }
        if (rollOffMemory[i] > rollOffLocalMax) {
            rollOffLocalMax = rollOffMemory[i];
        }
    }
    
    float normalizedRollOff = normalize(rollOff, rollOffLocalMin, rollOffLocalMax);
    cout<<"normalizedRollOff: "<<normalizedRollOff<<endl;
    rollOff = normalizedRollOff;
    
    // ------------------ STRONG PEAK ------------------
    float strongPeakLocalMax = 0.0;
    float strongPeakLocalMin = 10000;
    for (int i = 0; i < numMemoryFrames; i++) {
        if (strongPeakMemory[i] < strongPeakLocalMin) {
            strongPeakLocalMin = strongPeakMemory[i];
        }
        if (strongPeakMemory[i] > strongPeakLocalMax) {
            strongPeakLocalMax = strongPeakMemory[i];
        }
    }
    
    float normalizedStrongPeak = normalize(strongPeak, strongPeakLocalMin, strongPeakLocalMax);
    cout<<"normalizedStrongPeak: "<<normalizedStrongPeak<<endl;
    strongPeak = normalizedStrongPeak;
    
  
    
    
    // this is the PACKAGING section
    // Make  & Send OSC Object
    PacketWriter pkt = PacketWriter();
    pkt.startBundle();
    pkt.addMessage(msg.init("/essentia/rms").pushFloat(rms));
    pkt.addMessage(msg.init("/essentia/power").pushFloat(power));
    pkt.addMessage(msg.init("/essentia/pitchFreq").pushFloat(pitchFreq));
    pkt.addMessage(msg.init("/essentia/pitchConf").pushFloat(pitchConf));
    pkt.addMessage(msg.init("/essentia/pitchSalience").pushFloat(pitchSalience));
    pkt.addMessage(msg.init("/essentia/inharmonicity").pushFloat(inharmonicity));
    pkt.addMessage(msg.init("/essentia/hfc").pushFloat(hfc));
    pkt.addMessage(msg.init("/essentia/specComp").pushFloat(specComp));
    pkt.addMessage(msg.init("/essentia/centroid").pushFloat(centroid));
    pkt.addMessage(msg.init("/essentia/rollOff").pushFloat(rollOff));
    pkt.addMessage(msg.init("/essentia/oddToEven").pushFloat(oddToEven));
    pkt.addMessage(msg.init("/essentia/strongPeak").pushFloat(strongPeak));
    pkt.addMessage(msg.init("/essentia/strongDecay").pushFloat(strongDecay));
    //Normalized values for graphic meters:
    pkt.addMessage(msg.init("/essentia/pitchFreqNorm").pushFloat(pitchFreqNorm));
    pkt.addMessage(msg.init("/essentia/hfcNorm").pushFloat(hfcNorm));
    pkt.addMessage(msg.init("/essentia/specCompNorm").pushFloat(specCompNorm));
    pkt.addMessage(msg.init("/essentia/centroidNorm").pushFloat(centroidNorm));
    pkt.addMessage(msg.init("/essentia/rollOffNorm").pushFloat(rollOffNorm));
    pkt.addMessage(msg.init("/essentia/oddToEvenNorm").pushFloat(oddToEvenNorm));
    pkt.addMessage(msg.init("/essentia/strongPeakNorm").pushFloat(strongPeakNorm));
    pkt.addMessage(msg.init("/essentia/strongDecayNorm").pushFloat(strongDecayNorm));
    
    // Boolean, mapped to onset.
    pkt.addMessage(msg.init("/essentia/isOnset").pushFloat(isOnset));
    
    
    // Vector Parameters
    Message temp_msg_pter;
    int melBands_size=melBands.size(); //24
    temp_msg_pter = msg.init("/essentia/melBands");
    for(int n=0; n<melBands_size; n++)
    {   temp_msg_pter.pushFloat((float)melBands[n]);
    }
    pkt.addMessage(temp_msg_pter);
    
    int mfcc_size=mfcc.size();  //13
    temp_msg_pter = msg.init("/essentia/mfcc");
    for(int n=0; n<mfcc_size; n++)
    {   temp_msg_pter.pushFloat((float)mfcc[n]);
    }
    pkt.addMessage(temp_msg_pter);
    
    int tristimulus_size=tristimulus.size(); //3
    temp_msg_pter = msg.init("/essentia/tristimulus");
    for(int n=0; n<tristimulus_size; n++)
    {   temp_msg_pter.pushFloat((float)tristimulus[n]);
    }
    pkt.addMessage(temp_msg_pter);
    
    // Polyphonic Pitch from Filterbank
    float * polyphonic_pitch_pointer;
    float log_smth_energy;
    
    polyphonic_pitch_pointer=filterBank.getSmthEnergies();
    
    log_smth_energy = LIN2dB (polyphonic_pitch_pointer[filterBank.midiMinVar]);
    temp_msg_pter= msg.init("/essentia/PolyphonicPitch");
    std::cout<<"Midi Min= "<<filterBank.midiMinVar<<endl;
    std::cout<<"Midi Min= "<<filterBank.midiMaxVar<<endl;
    for(int n=filterBank.midiMinVar; n<filterBank.midiMaxVar; n++)  // 87
    {   log_smth_energy = LIN2dB (polyphonic_pitch_pointer[n]);
        std::cout<<log_smth_energy<<" ";
        temp_msg_pter.pushFloat(log_smth_energy);
    }
    pkt.addMessage(temp_msg_pter);
    
    std::cout<<endl;

    pkt.endBundle();
    if (pkt.isOk()) {
        message=pkt.packetData();
        size= pkt.packetSize();
        client.send_osc(message, size);
    }
    msg.clear();
    
    // Make  & Send Second OSC Object (first bundle was full)
    PacketWriter pkt2 = PacketWriter();
    pkt2.startBundle();
    
    int spectrum_size=spectrum.size(); //257
    temp_msg_pter = msg.init("/essentia/spectrum");
    for(int n=0; n<spectrum_size; n++)
    {   temp_msg_pter.pushFloat((float)spectrum[n]);
    }
    pkt2.addMessage(temp_msg_pter);

    int hpcp_size=hpcp.size();  //12
    std::cout<<endl<<"Hpcp Size: "<<hpcp_size<<endl;
    temp_msg_pter = msg.init("/essentia/hpcp");
    for(int n=0; n<hpcp_size; n++)
    {   temp_msg_pter.pushFloat((float)hpcp[n]);
    }
    pkt2.addMessage(temp_msg_pter);
    
    pkt2.addMessage(msg.init("/aubio/onset").pushFloat(onset.thresholdedNovelty));
    pkt2.addMessage(msg.init("/aubio/midiPitch").pushFloat(pitch.latestPitch));
    pkt2.addMessage(msg.init("/aubio/bpm").pushFloat(beat.bpm));
    
    //Close the second bundle
    pkt2.endBundle();
    if (pkt2.isOk()) {
        message=pkt2.packetData();
        size= pkt2.packetSize();
        client.send_osc(message, size);
    }
    msg.clear();
   
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    //-Single value Algorithms:
    
    ofPushMatrix();
    ofTranslate(350, 0);
    int mw = 250;
    int xpos = 0;
    int ypos = 30;
    
    float value, valueNorm;
    
    ofSetColor(255);
    value = rms;
    string strValue = "RMS: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, value * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = power;
    strValue = "Power: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, value * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = pitchFreq;
    valueNorm = pitchFreqNorm;
    strValue = "Pitch Frequency: " + ofToString(value, 2) + " hz.";
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    // if we normalize it ourselves (as opposed to sending out essentia's normalized version) then (cont.)
    // we actually get visible results!!!
    ofDrawRectangle(xpos, ypos+5, value * mw, 10);

    
    ypos += 50;
    ofSetColor(255);
    value = pitchConf;
    strValue = "Pitch Confidence: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, value * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = pitchSalience;
    strValue = "Pitch Salience: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, value * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = inharmonicity;
    strValue = "Inharmonicity: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, value * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = hfc;
    valueNorm = hfcNorm;
    strValue = "HFC: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    // if we normalize it ourselves (as opposed to sending out essentia's normalized version) then (cont.)
    // we actually get visible results!!!
    ofDrawRectangle(xpos, ypos+5, value * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = specComp;
    valueNorm = specCompNorm;
    strValue = "Spectral Complexity: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, valueNorm * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = centroid;
    valueNorm = centroidNorm;
    strValue = "Centroid: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, valueNorm * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = dissonance;
    strValue = "Dissonance: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, value * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = rollOff;
    valueNorm = rollOffNorm;
    strValue = "Roll Off: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, value * mw , 10);
    
    ypos += 50;
    ofSetColor(255);
    value = oddToEven;
    valueNorm = oddToEvenNorm;
    strValue = "Odd To Even Harmonic Energy Ratio: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, valueNorm * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = strongPeak;
    valueNorm = strongPeakNorm;
    strValue = "Strong Peak: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, value * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = strongDecay;
    valueNorm = strongDecayNorm;
    strValue = "Strong Decay: " + ofToString(value, 2);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, valueNorm * mw, 10);
    
    ypos += 50;
    ofSetColor(255);
    value = isOnset;
    strValue = "Onsets: " + ofToString(value);
    ofDrawBitmapString(strValue, xpos, ypos);
    ofSetColor(ofColor::cyan);
    ofDrawRectangle(xpos, ypos+5, value * mw, 10);
    
    ofPopMatrix();
    
    //-Vector Values Algorithms:
    
    ofPushMatrix();
    
    ofTranslate(700, 0);
    
    int graphH = 75;
    int yoffset = graphH + 50;
    ypos = 30;
    
    ofSetColor(255);
    ofDrawBitmapString("Spectrum: ", 0, ypos);
    ofPushMatrix();
    ofTranslate(0, ypos);
    ofSetColor(ofColor::cyan);
    float bin_w = (float) mw / spectrum.size();
    for (int i = 0; i < spectrum.size(); i++){
        float scaledValue = ofMap(spectrum[i], DB_MIN, DB_MAX, 0.0, 1.0, true);//clamped value
        float bin_h = -1 * (scaledValue * graphH);
        ofDrawRectangle(i*bin_w, graphH, bin_w, bin_h);
    }
    ofPopMatrix();
    
    ypos += yoffset;
    ofSetColor(255);
    ofDrawBitmapString("Mel Bands: ", 0, ypos);
    ofPushMatrix();
    ofTranslate(0, ypos);
    ofSetColor(ofColor::cyan);
    bin_w = (float) mw / melBands.size();
    for (int i = 0; i < melBands.size(); i++){
        float scaledValue = ofMap(melBands[i], DB_MIN, DB_MAX, 0.0, 1.0, true);//clamped value
        float bin_h = -1 * (scaledValue * graphH);
        ofDrawRectangle(i*bin_w, graphH, bin_w, bin_h);
    }
    ofPopMatrix();
    
    ypos += yoffset;
    ofSetColor(255);
    ofDrawBitmapString("MFCC: ", 0, ypos);
    ofPushMatrix();
    ofTranslate(0, ypos);
    ofSetColor(ofColor::cyan);
    bin_w = (float) mw / mfcc.size();
    for (int i = 0; i < mfcc.size(); i++){
        float scaledValue = ofMap(mfcc[i], 0, MFCC_MAX_ESTIMATED_VALUE, 0.0, 1.0, true);//clamped value
        float bin_h = -1 * (scaledValue * graphH);
        ofDrawRectangle(i*bin_w, graphH, bin_w, bin_h);
    }
    ofPopMatrix();
    
    ypos += yoffset;
    ofSetColor(255);
    ofDrawBitmapString("HPCP: ", 0, ypos);
    ofPushMatrix();
    ofTranslate(0, ypos);
    ofSetColor(ofColor::cyan);
    bin_w = (float) mw / hpcp.size();
    for (int i = 0; i < hpcp.size(); i++){
        //float scaledValue = ofMap(hpcp[i], DB_MIN, DB_MAX, 0.0, 1.0, true);//clamped value
        float scaledValue = hpcp[i];
        float bin_h = -1 * (scaledValue * graphH);
        ofDrawRectangle(i*bin_w, graphH, bin_w, bin_h);
    }
    ofPopMatrix();
    
    ypos += yoffset;
    ofSetColor(255);
    ofDrawBitmapString("Tristimulus: ", 0, ypos);
    ofPushMatrix();
    ofTranslate(0, ypos);
    ofSetColor(ofColor::cyan);
    bin_w = (float) mw / tristimulus.size();
    for (int i = 0; i < tristimulus.size(); i++){
        //float scaledValue = ofMap(hpcp[i], DB_MIN, DB_MAX, 0.0, 1.0, true);//clamped value
        float scaledValue = tristimulus[i];
        float bin_h = -1 * (scaledValue * graphH);
        ofDrawRectangle(i*bin_w, graphH, bin_w, bin_h);
    }
    ofPopMatrix();
    
    
    
    ofPopMatrix();
    
    //-Gui & info:
    
    gui.draw();
    ofSetColor(255);
    ofDrawBitmapString("ofxAudioAnalyzer\n\nALL ALGORITHMS EXAMPLE", 10, 32);
    ofSetColor(ofColor::hotPink);
    ofDrawBitmapString("Keys 1-6: Play audio tracks", 10, 100);
    
    


   /* ofSetColor(ofColor::cyan);
    
    float xpos = ofGetWidth() *.5;
    float ypos = ofGetHeight() - ofGetHeight() * rms_r;
    float radius = 5 + 100*rms_l;
    
    ofDrawCircle(xpos, ypos, radius);
    
    //----------------
    
    ofSetColor(225);
    ofDrawBitmapString("ofxAudioAnalyzer - RMS SMOOTHING INPUT EXAMPLE", 32, 32);
    
    
    string infoString = "RMS Left: " + ofToString(rms_l) +
                        "\nRMS Right: " + ofToString(rms_r) +
                        "\nSmoothing (mouse x): " + ofToString(smooth);
    
    ofDrawBitmapString(infoString, 32, 579);*/
    
    
    /*
    ofSetColor(225);
    ofNoFill();
    
    float chSz = bufferSize/3;
    // draw the left input channel:
    {
        ofPushStyle();
        ofPushMatrix();
        
        ofTranslate(100, 15, 0);
        ofSetColor(225);
        ofDrawBitmapString("Left Channel", 4, 18);
        ofSetLineWidth(1);
        ofRect(0, 0, chSz, 200);
        ofSetColor(ofColor::orange);
        ofSetLineWidth(3);
        ofBeginShape();
        for (int i = 0; i < bufferSize; i++){
            ofVertex(i/(bufferSize/chSz), 100 - filterBank.getLeftBuffer()[i]*45);
        }
        ofEndShape(false);
        ofPopMatrix();
        ofPopStyle();
    }
    // draw the right input channel:
    {
        ofPushStyle();
        ofPushMatrix();
        ofTranslate(200+chSz, 15, 0);
        ofSetColor(225);
        ofDrawBitmapString("Right Channel", 4, 18);
        ofSetLineWidth(1);
        ofRect(0, 0, chSz, 200);
        ofSetColor(ofColor::orange);
        ofSetLineWidth(3);
        ofBeginShape();
        for (int i = 0; i < bufferSize; i++){
            ofVertex(i/(bufferSize/chSz), 100 - filterBank.getRightBuffer()[i]*45);
        }
        ofEndShape(false);
        ofPopMatrix();
        ofPopStyle();
    }
    
    //Draw FilterBank
    {
        ofPushStyle();
        ofPushMatrix();
        ofTranslate (100,250,0);
        filterBank.draw(800,400);
        ofPopMatrix();
        ofPopStyle();
    }
    ofSetColor(225);
    
    string reportString =  "Sampling Rate: "+ ofToString(SR) +"\nBuffer size: "+ ofToString(bufferSize);
    ofDrawBitmapString(reportString, 10, 700);
    
    // Onsets, BPM and monophonic pitch from Aubio
    
    onset.setThreshold(onsetThreshold);
    onsetNovelty = onset.novelty;
    onsetThresholdedNovelty = onset.thresholdedNovelty;
    std::cout<<"Onset Threshold Novelty:"<<onsetThresholdedNovelty<<endl ;
    
    // update pitch info
    pitchConfidence = pitch.pitchConfidence;
    if (pitch.latestPitch) midiPitch = pitch.latestPitch;
    std::cout<<"Pitch:"<<midiPitch<<endl ;
    
    // update BPM
    bpm = beat.bpm;
    std::cout<<"BPM:"<<bpm<<endl ;
*/
}
//--------------------------------------------------------------
void ofApp::audioIn(ofSoundBuffer &inBuffer){
    //ANALYZE SOUNDBUFFER:
    audioAnalyzer.analyze(inBuffer);

    //Analyze Input Buffer with ofxFilterbank
    vector <float> temp;
    temp = inBuffer.getBuffer(); // this spits out <vector &>
    float *p = &temp[0];
    filterBank.analyze(p);  //if the gui is working, then this is working, and the pointer works.
    
    //Aubio
    
    // compute onset detection
    
    onset.audioIn(p, inBuffer.getNumFrames(), inBuffer.getNumChannels() );
    // compute pitch detection
    pitch.audioIn(p, inBuffer.getNumFrames(), inBuffer.getNumChannels());
    // compute beat location
    beat.audioIn(p, inBuffer.getNumFrames(), inBuffer.getNumChannels());
    // compute bands
    bands.audioIn(p, inBuffer.getNumFrames(), inBuffer.getNumChannels());
    
}


//--------------------------------------------------------------
void ofApp::exit(){
    ofSoundStreamStop();
    audioAnalyzer.exit();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

//==========Aubio

//----
void ofApp::onsetEvent(float & time) {
    //ofLog() << "got onset at " << time << " s";
    gotOnset = true;
}

//----
void ofApp::beatEvent(float & time) {
    //ofLog() << "got beat at " << time << " s";
    gotBeat = true;
}





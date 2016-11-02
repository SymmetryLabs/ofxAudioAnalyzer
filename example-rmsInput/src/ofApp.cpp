#include "ofApp.h"
#include "asio.hpp"     //For socket integration
#include "oscpkt.hh"    //For OSC Serialzation


//---------------------Global Var: ASIO------------------------------------

std::string s;
using asio::ip::udp;
asio::io_service io_service;
#define PORT "1330"

//---------------------Global Var: OSC-------------------------------------

using namespace oscpkt;
PacketWriter pkt;
Message msg;
const void * message;
int size;

//---------------------UDP Classes-----------------------------------------

class UDPClient
{
public:
    UDPClient(
              asio::io_service& io_service,
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
        socket_.send_to(asio::buffer(msg, msg.size()), endpoint_);
    }
    
    void send_osc(const void *msg, int size) {
        socket_.send_to(asio::buffer(msg, size), endpoint_);
    }
    
private:
    asio::io_service& io_service_;
    udp::socket socket_;
    udp::endpoint endpoint_;
};


UDPClient client(io_service, "localhost", PORT);



//--------------------------------------------------------------
void ofApp::setup(){
    
    ofBackground(34, 34, 34);
    ofSetFrameRate(60);
    
    int sampleRate = 44100;
    int bufferSize = 512;
    int outChannels = 0;
    int inChannels = 2;
    
    // setup the sound stream
    soundStream.setup(this, outChannels, inChannels, sampleRate, bufferSize, 3);
    
    //setup ofxAudioAnalyzer with the SAME PARAMETERS
    audioAnalyzer.setup(sampleRate, bufferSize, inChannels);
   
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
    
    spectrum = audioAnalyzer.getValues(SPECTRUM, 0, smoothing);
    melBands = audioAnalyzer.getValues(MEL_BANDS, 0, smoothing);
    mfcc = audioAnalyzer.getValues(MFCC, 0, smoothing);
    hpcp = audioAnalyzer.getValues(HPCP, 0, smoothing);
    
    tristimulus = audioAnalyzer.getValues(TRISTIMULUS, 0, smoothing);
    
    isOnset = audioAnalyzer.getOnsetValue(0);
    
    std::cout<<"centroidNorm:"<<centroidNorm<<endl ;
    std::cout<<"rms:"<<rms<<endl ;
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    ofSetColor(ofColor::cyan);
    
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
    
    ofDrawBitmapString(infoString, 32, 579);
}
//--------------------------------------------------------------
void ofApp::audioIn(ofSoundBuffer &inBuffer){
    //ANALYZE SOUNDBUFFER:
    audioAnalyzer.analyze(inBuffer);
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

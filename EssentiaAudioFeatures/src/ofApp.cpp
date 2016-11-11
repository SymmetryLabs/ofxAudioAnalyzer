#include "ofApp.h"
#include "asio.hpp"     //For socket integration
#include "oscpkt.hh"    //For OSC Serialzation


//---------------------Global Var: ASIO------------------------------------

std::string s;
using asio::ip::udp;
asio::io_service io_service;
#define PORT "1331"

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
    //int bufferSize = 512;
    int outChannels = 0;
    int inChannels = 2;
    
    // setup the sound stream
    soundStream.setup(this, outChannels, inChannels, sampleRate, bufferSize, 3);
    
    //setup ofxAudioAnalyzer with the SAME PARAMETERS
    audioAnalyzer.setup(sampleRate, bufferSize, inChannels);
    
    
    
    
    //--------------filterbank setup
    
    ofSetVerticalSync(true);
    ofBackground(54, 54, 54);
    ofSetFrameRate(60);
    
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
    
    spectrum = audioAnalyzer.getValues(SPECTRUM, 0, smoothing);
    melBands = audioAnalyzer.getValues(MEL_BANDS, 0, smoothing);
    mfcc = audioAnalyzer.getValues(MFCC, 0, smoothing);
    hpcp = audioAnalyzer.getValues(HPCP, 0, smoothing);
    tristimulus = audioAnalyzer.getValues(TRISTIMULUS, 0, smoothing);
    
    isOnset = audioAnalyzer.getOnsetValue(0);
    
    std::cout<<"centroidNorm:"<<centroidNorm<<endl ;
    std::cout<<"strongDecayNorm:"<<strongDecayNorm<<endl ;
    
    // Make  & Send OSC Object
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
    
    int spectrum_size=spectrum.size();
    pkt.addMessage(msg.init("/essentia/spectrum"));
    for(int n=0; n<spectrum_size; n++)
    {   msg.pushFloat(spectrum[n]);
    }

    int melBands_size=melBands.size();
    pkt.addMessage(msg.init("/essentia/melBands"));
    for(int n=0; n<melBands_size; n++)
    {   msg.pushFloat(melBands[n]);
    }
    
    int mfcc_size=mfcc.size();
    pkt.addMessage(msg.init("/essentia/mfcc"));
    for(int n=0; n<mfcc_size; n++)
    {   msg.pushFloat(mfcc[n]);
    }
    
    int hpcp_size=hpcp.size();
    pkt.addMessage(msg.init("/essentia/hpcp"));
    for(int n=0; n<hpcp_size; n++)
    {   msg.pushFloat(hpcp[n]);
    }
    
    int tristimulus_size=tristimulus.size();
    pkt.addMessage(msg.init("/essentia/tristimulus"));
    for(int n=0; n<tristimulus_size; n++)
    {   msg.pushFloat(tristimulus[n]);
    }
    
    // Polyphonic Pitch from Filterbank
    float * polyphonic_pitch_pointer;
    float log_smth_energy;
    polyphonic_pitch_pointer=filterBank.getSmthEnergies();
    
    log_smth_energy = LIN2dB (polyphonic_pitch_pointer[filterBank.midiMinVar]);
    pkt.addMessage(msg.init("/essentia/PolyphonicPitch").pushFloat(log_smth_energy));
    std::cout<<"Midi Min= "<<filterBank.midiMinVar<<endl;
    std::cout<<"Midi Min= "<<filterBank.midiMaxVar<<endl;
    for(int n=filterBank.midiMinVar+1; n<filterBank.midiMaxVar; n++)
    {   log_smth_energy = LIN2dB (polyphonic_pitch_pointer[n]);
        std::cout<<log_smth_energy<<" ";
        msg.pushFloat(log_smth_energy);
    }
    std::cout<<endl;
    
    
    
    pkt.endBundle();
    if (pkt.isOk()) {
        message=pkt.packetData();
        size= pkt.packetSize();
        client.send_osc(message, size);
    }
    msg.clear();
    pkt.Reset();
}

//--------------------------------------------------------------
void ofApp::draw(){
    
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

}
//--------------------------------------------------------------
void ofApp::audioIn(ofSoundBuffer &inBuffer){
    //ANALYZE SOUNDBUFFER:
    audioAnalyzer.analyze(inBuffer);

    //Analyze Input Buffer with ofxFilterbank
    vector <float> temp;
    temp = inBuffer.getBuffer();
    float *p = &temp[0];
    filterBank.analyze(p);
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

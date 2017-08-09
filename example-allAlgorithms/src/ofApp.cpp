#include "ofApp.h"




//--------------------------------------------------------------
void ofApp::setup(){
    
    ofBackground(34);
    ofSetFrameRate(60);
    
    sampleRate = 44100;
    bufferSize = 512; //careful, as nomenclature changed from size to frames where size=frames*channel_count
    int channels = 1;
    
    audioAnalyzer.setup(sampleRate, bufferSize, channels);
    
//    player.load("beatTrack.wav");
    
    
    
    gui.setup();
    gui.setPosition(20, 150);
    gui.add(smoothing.setup  ("Smoothing", 0.0, 0.0, 1.0));
    
    //----------------Make Buffers for the audio
    /*int nOutputChannels=0;
    int nInputChannels=1;
    int nBuffers=4;
    int numFrames=512;
    MonoAudioInput = new short[bufferSize]; //two channels
    soundBuffer = ofSoundBuffer( MonoAudioInput, numFrames, nInputChannels, (unsigned int) sampleRate);
    for (int i = 0; i<bufferSize; i++) {
        MonoAudioInput[i] = 0;
    }
    
    left.assign(bufferSize, 0.0);
    right.assign(bufferSize, 0.0);
    bufferCounter	= 0;
    
    */
    
    //----------------Setup the Sound Stream
    
    //int sampleRate = 44100;
    //int bufferSize = 512;
    int outChannels = 0;
    int inChannels = 1;
    
    // setup the sound stream
    soundStream.setup(this, outChannels, inChannels, sampleRate, bufferSize, 4);
    
    //setup ofxAudioAnalyzer with the SAME PARAMETERS
    audioAnalyzer.setup(sampleRate, bufferSize, inChannels);

    //----------------Connect the Sound Stream to an Input Device
    soundStream.printDeviceList();
    std::vector<ofSoundDevice> devices;
    devices = soundStream.getMatchingDevices("default");
    if(!devices.empty()){
            soundStream.setDeviceID(devices[0].deviceID);
    }
// The audio analyzer accepts the soundBuffer Object.
    
    soundStream.start();
}
//--------------------------------------------------------------
void ofApp::update(){
    
    ofSetWindowTitle(ofToString(ofGetFrameRate()));
    
    
    /*
    //-:Get buffer from sound player:
    soundBuffer = player.getCurrentSoundBuffer(bufferSize);
    
    //-:ANALYZE SOUNDBUFFER:
    audioAnalyzer.analyze(soundBuffer);
    */
     
    //-:get Values:
    
    //  float
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
    // float ... Normalized values for graphic meters:
    pitchFreqNorm   = audioAnalyzer.getValue(PITCH_FREQ, 0, smoothing, TRUE);
    hfcNorm     = audioAnalyzer.getValue(HFC, 0, smoothing, TRUE);
    specCompNorm = audioAnalyzer.getValue(SPECTRAL_COMPLEXITY, 0, smoothing, TRUE);
    centroidNorm = audioAnalyzer.getValue(CENTROID, 0, smoothing, TRUE);
    rollOffNorm  = audioAnalyzer.getValue(ROLL_OFF, 0, smoothing, TRUE);
    oddToEvenNorm   = audioAnalyzer.getValue(ODD_TO_EVEN, 0, smoothing, TRUE);
    strongPeakNorm  = audioAnalyzer.getValue(STRONG_PEAK, 0, smoothing, TRUE);
    strongDecayNorm = audioAnalyzer.getValue(STRONG_DECAY, 0, smoothing, TRUE);
    dissonance = audioAnalyzer.getValue(DISSONANCE, 0, smoothing);
    
    //  vector<float>
    spectrum = audioAnalyzer.getValues(SPECTRUM, 0, smoothing);
    melBands = audioAnalyzer.getValues(MEL_BANDS, 0, smoothing);
    mfcc = audioAnalyzer.getValues(MFCC, 0, smoothing);
    hpcp = audioAnalyzer.getValues(HPCP, 0, smoothing);
    tristimulus = audioAnalyzer.getValues(TRISTIMULUS, 0, smoothing);
    
    //  bool
    isOnset = audioAnalyzer.getOnsetValue(0);
  
    std::cout<<"centroidNorm:"<<centroidNorm<<endl ;
    std::cout<<"rms:"<<rms<<endl ;
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
    ofDrawRectangle(xpos, ypos+5, valueNorm * mw, 10);
    
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
    ofDrawRectangle(xpos, ypos+5, valueNorm * mw, 10);
    
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
    ofDrawRectangle(xpos, ypos+5, valueNorm * mw , 10);
    
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
    ofDrawRectangle(xpos, ypos+5, valueNorm * mw, 10);
    
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
    

    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    /*player.stop();
    soundStream.stop();

    switch (key) {
        case '0':
            soundStream.start();
            break;
        case '1':
            player.load("test440mono.wav");
                player.play();
            break;
        case '2':
            player.load("flute.wav");
                player.play();
            break;
        case '3':
            player.load("chord.wav");
                player.play();
            break;
        case '4':
            player.load("cadence.wav");
                player.play();
            break;
        case '5':
            player.load("beatTrack.wav");
                player.play();
            break;
        case '6':
            player.load("noise.wav");
                player.play();
            break;
            
            
        default:
            break;
    }
*/
    
}
//--------------------------------------------------------------
void ofApp::exit(){
    audioAnalyzer.exit();
    //player.stop();
    ofSoundStreamStop();
    ofSoundStreamClose();
}

//-----------------------Audio Stream Classes--------------------------


//void ofApp::audioIn( float * input, int bufferSize, int nChannels ) {
//void ofApp::audioIn(ofSoundBuffer & input);
//    short x;
 //   for (int i = 0; i<bufferSize; i++) {
  //      MonoAudioInput[i] = (short) input[i];
   //  }
    
//}

void ofApp::audioIn(ofSoundBuffer &inBuffer){
    //ANALYZE SOUNDBUFFER:
    audioAnalyzer.analyze(inBuffer);
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
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

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

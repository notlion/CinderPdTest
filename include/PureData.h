/*
 CinderPd by Ryan Alexander
 adapted from ofxPd Copyright 2010 by Damian Stewart, Zach Liebermann.

 this code uses code from pdlib "AudioOutput.h", so pdlib license is included here:

 AudioOutput.h
 PdLib v0.3

 Copyright 2010 Bryan Summersett. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY BRYAN SUMMERSETT ``AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BRYAN SUMMERSETT OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Bryan Summersett.
 */

#pragma once

#include "cinder/Thread.h"
#include "cinder/Utilities.h"
#include "cinder/CinderMath.h"

#include "boost/algorithm/string/join.hpp"

using std::string;
using std::vector;

namespace cinder { namespace pd {

class PureData {
public:

    PureData();
    ~PureData();

	// lib_dir is the directory in which to look for pd files
	// in_chans, out_chans, bitrate and block_size are passed to Pd engine
	// if in_chans is >0, you should call the input+output version of renderAudio to pass
	// input audio to the Pd engine
	void setup(
        const string &lib_dir,
        const int _n_channels_in = 0, const int _n_channels_out = 2,
        const int _sample_rate = 44100,
        const int _chunk_size = 64
    );

	// add the given file to the list to be opened on startup
	void addOpenFile(const string &file_path);

	// add the given path to the search path
	void addSearchPath(const string &dir_path);

	// start and stop pd core; DSP (sound processing) is started separately
	void start();
	void stop();

	// start DSP (sends 'pd dsp 1' message)
	void startDSP(){
        sendRawMessage("; pd dsp 1" );
    }
	// stop DSP (sends 'pd dsp 0' message)
	void stopDSP(){
        sendRawMessage("; pd dsp 0" );
    }

	// send a message to the pd engine
	void sendRawMessage(const string& message );
	// send the given float to the given receive target in pd
	void sendFloat(const string& receive_target, float number);


	// callback for audio rendering (output only)
	void renderAudio(float* output, int bufferSize, int nChannels);
	// callback for audio rendering (input and output)
	void renderAudio(float * input, float* output, int bufferSize, int nChannels);

    int getSampleRate();

private:

    std::shared_ptr<std::thread> pd_thread;
    bool                         pd_thread_running;

	// the thing to run in a thread
	void pdThreadFunction();

	string lib_dir;

	vector<string> externs;
	vector<string> search_paths;
	vector<string> open_files;

	int n_channels_out, n_channels_in, sample_rate, chunk_size;
};

}}

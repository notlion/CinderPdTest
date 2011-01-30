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

#include "PureData.h"
#include <assert.h>

extern "C"
{
#include "m_pd.h"
#include "m_imp.h"
#include "s_main.h"
#include "s_stuff.h"

extern void sched_audio_callbackfn(void);
};


using namespace ci::pd;


PureData::PureData()
{
    pd_thread_running = false;
}
PureData::~PureData()
{
    stop();
}


void PureData::setup(const string &_lib_dir, int _n_channels_in, int _n_channels_out, int _sample_rate, int _chunk_size)
{
	lib_dir        = _lib_dir;
	n_channels_in  = _n_channels_in;
	n_channels_out = _n_channels_out;
	sample_rate    = _sample_rate;
	chunk_size     = _chunk_size;
}


void PureData::addOpenFile(const string &file_path)
{
	open_files.push_back(file_path);
}

void PureData::addSearchPath(const string &dir_path)
{
	search_paths.push_back(dir_path);
}


void PureData::start()
{
    pd_thread = std::shared_ptr<std::thread>(new std::thread(&PureData::pdThreadFunction, this));

    pd_thread_running = true;

	// sleep until pd engine has fully started
	while(!sys_hasstarted)
		ci::sleep(500);
}

void PureData::stop()
{
	if(pd_thread_running){
		sys_exit();
        pd_thread->join();
    }
}


void PureData::pdThreadFunction()
{
	// concatenate string vectors to single strings
	string externs_cat      = boost::algorithm::join(externs, ":");
    string open_files_cat   = boost::algorithm::join(open_files, ":");
    string search_paths_cat = boost::algorithm::join(search_paths, ":");

	sys_main(lib_dir.c_str(), externs_cat.c_str(), open_files_cat.c_str(), search_paths_cat.c_str(), sample_rate, chunk_size, n_channels_out, n_channels_in);
}


void PureData::renderAudio(float* output, int bufferSize, int nChannels)
{
	// adapted from AudioOutputController by bryan summerset

    // sys_schedblocksize is How many frames we have per PD dsp_tick
    // inNumberFrames is how many frames have been requested
	int inNumberFrames = bufferSize;

    // Make sure we're dealing with evenly divisible numbers between
    // the number of frames requested vs the block size for a given PD dsp tick.
    //Otherwise this looping scheme we're doing below doesn't make much sense.
    assert(inNumberFrames % sys_schedblocksize == 0);

    // How many times to generate a DSP event in PD
    int times = inNumberFrames / sys_schedblocksize;

    for(int i = 0; i < times; i++)
    {
		// do one Pd DSP block
        sys_lock();
		sched_audio_callbackfn();
        sys_unlock();

        // This should cover sys_noutchannels channels. Turns non-interleaved into
        // interleaved audio.
        for(int n = 0; n < sys_schedblocksize; n++){
            for(int ch = 0; ch < sys_noutchannels; ch++){
                t_sample fsample = math<t_sample>::clamp(sys_soundout[n + sys_schedblocksize * ch], -1, 1);
                output[(n * sys_noutchannels + ch) + // offset in iteration
					    i * sys_schedblocksize * sys_noutchannels] // iteration starting address
					= fsample;
            }
        }

        // After loading the samples, we need to clear them for the next iteration
        memset(sys_soundout, 0, sizeof(t_sample) * sys_noutchannels * sys_schedblocksize);
    }

}

void PureData::renderAudio(float *input, float *output, int bufferSize, int nChannels)
{
	// adapted from AudioOutputController by bryan summerset

	// sys_schedblocksize is How many frames we have per PD dsp_tick
    // inNumberFrames is how many frames have been requested
	int inNumberFrames = bufferSize;

    // Make sure we're dealing with evenly divisible numbers between
    // the number of frames requested vs the block size for a given PD dsp tick.
    //Otherwise this looping scheme we're doing below doesn't make much sense.
    assert(inNumberFrames % sys_schedblocksize == 0);

    // How many times to generate a DSP event in PD
    int times = inNumberFrames / sys_schedblocksize;

    for(int i = 0; i < times; i++){
		for(int n = 0; n < sys_schedblocksize; n++){
            for(int ch = 0; ch < sys_ninchannels; ch++){
				sys_soundin[n + sys_schedblocksize * ch] = input[(n * sys_ninchannels + ch) + // offset in iteration
                                                                 i * sys_schedblocksize * sys_ninchannels]; // iteration starting address
            }
        }

		// do one Pd DSP block
        sys_lock();
		sched_audio_callbackfn();
        sys_unlock();

        // This should cover sys_noutchannels channels.
        // Turns non-interleaved into interleaved audio.
        for(int n = 0; n < sys_schedblocksize; n++){
            for(int ch = 0; ch < sys_noutchannels; ch++){
                t_sample fsample = math<t_sample>::clamp(sys_soundout[n + sys_schedblocksize * ch], -1, 1);
                output[(n * sys_noutchannels + ch) + // offset in iteration
                       i * sys_schedblocksize * sys_noutchannels] // iteration starting address
					= fsample;
			}
        }

        // After loading the samples, we need to clear them for the next iteration
        memset(sys_soundout, 0, sizeof(t_sample) * sys_noutchannels * sys_schedblocksize);
    }
}


void PureData::sendFloat(const string &receive_target, float the_float)
{
	sendRawMessage(";" + receive_target + " " + toString(the_float));
}

void PureData::sendRawMessage(const string &message)
{
	// senda  message to pd
	t_binbuf *b = binbuf_new();
	static char msg_buf[MAXPDSTRING + 1];
	strncpy(msg_buf, message.c_str(), message.size());
	binbuf_text(b, msg_buf, message.size() );
	sys_lock();
	binbuf_eval(b, 0, 0, 0);
	sys_unlock();
	binbuf_free(b);
}


int PureData::getSampleRate()
{
    return sample_rate;
}

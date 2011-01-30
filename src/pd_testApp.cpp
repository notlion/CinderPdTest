#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/audio/Output.h"
#include "cinder/audio/Callback.h"

#include "OscSender.h"
#include "PureData.h"

using namespace ci;
using namespace ci::app;
using namespace std;


class pd_testApp : public AppBasic {
public:

	void setup();
	void mouseMove(MouseEvent event);
	void update();
	void draw();

    void onAudioRequested(uint64_t in_sample_offset, uint32_t io_sample_count, audio::Buffer32f *io_buffer);

    osc::Sender osc_out;

    pd::PureData pd;

    static const int SAMPLE_RATE  = 44100;
    static const int CHANNELS_IN  = 0;
    static const int CHANNELS_OUT = 2;
};


void pd_testApp::setup()
{
    pd.setup(getResourcePath(), CHANNELS_IN, CHANNELS_OUT, SAMPLE_RATE);
    pd.addOpenFile(getResourcePath("sine.pd"));
    pd.start();

    audio::Output::play(audio::createCallback(this, &pd_testApp::onAudioRequested, false, SAMPLE_RATE, CHANNELS_OUT));

    pd.startDSP();
}

void pd_testApp::mouseMove(MouseEvent event)
{
	pd.sendFloat("osc_l_frequency", event.getX());
	pd.sendFloat("osc_r_frequency", event.getY());
}

void pd_testApp::onAudioRequested(uint64_t in_sample_offset, uint32_t io_sample_count, audio::Buffer32f *io_buffer)
{
	pd.renderAudio(io_buffer->mData, io_sample_count, 2);
}

void pd_testApp::update()
{
}

void pd_testApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) );
}


CINDER_APP_BASIC( pd_testApp, RendererGl )

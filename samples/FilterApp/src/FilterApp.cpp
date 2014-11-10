#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"

#include "

class FilterApp : public ci::app::AppNative
{
public:
	void draw();
	void mouseDrag( ci::app::MouseEvent event );
	void setup();
	void update();
};

using namespace ci;
using namespace ci::app;
using namespace std;

void FilterApp::draw()
{
	gl::clear( Color::black() );
}

void FilterApp::mouseDrag( MouseEvent event )
{
}

void FilterApp::setup()
{
}

void FilterApp::update()
{
}

CINDER_APP_NATIVE( FilterApp, RendererGl )

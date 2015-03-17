
/*
 *
 * Copyright (c) 2015, Wieden+Kennedy
 * Stephen Schieberl
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of the Ban the Rewind nor the names of its
 * contributors may be used to endorse or promote products
 * derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "cinder/app/App.h"
#include "cinder/params/Params.h"

#include "Sampling.h"

/*
 * This app demonstrates how to use Sampling to apply a 
 * variety of smoothing algorithms without the need for 
 * classes or cumbersome setup.
 * 
 * This sample was inspired by Jonathan Aceituno's 1euro 
 * demo. The colors were lifted from the his demo, but the 
 * filter implementations were not.
 * http://www.lifl.fr/~casiez/1euro/InteractiveDemo/
 */

enum : size_t
{
	FilterType_None,
	FilterType_Noise,
	FilterType_ExponentialDouble,
	FilterType_ExponentialSingle,
	FilterType_Kalman,
	FilterType_MovingAverage,
	FilterType_OneEuro,
	FilterType_Count
} typedef FilterType;

typedef std::map<size_t, std::function<float()> >	ProcessMap;
typedef sampling::SamplerT<float, float>			Sampler;

class FilterApp : public ci::app::App
{
public:
	void						draw() override;
	void						setup() override;
	void						update() override;
private:
	typedef std::map<size_t, std::vector<float> > SignalMap;
	
	float						mSignalAmplitude;
	float						mSignalFrequency;
	float						mSignalNoise;
	float						mSignalRawValue;
	
	float						mAlphaExponentialDouble;
	float						mAlphaExponentialSingle;
	float						mKalmanProcessErrorCovariance;
	float						mKalmanMeasurementErrorCovariance;
	int32_t						mMovingAverageWindowSize;
	float						mOneEuroCutoffSlope;
	float						mOneEuroMinimumCutoff;
	float						mOneEuroCutoffForDerivative;
	
	Sampler						mInput;
	int32_t						mNumSamples;
	SignalMap					mOutput;
	
	std::map<size_t, bool>		mDraw;
	
	float						mFrameRate;
	ci::params::InterfaceGlRef	mParams;
};

#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Rand.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void FilterApp::draw()
{
	gl::viewport( getWindowSize() );
	gl::clear( Color8u::hex( 0x888888 ) );
	gl::setMatricesWindow( getWindowSize() );
	
	for ( const auto& iter : mOutput ) {
		if ( mDraw.at( iter.first ) ) {
		
			// Choose signal color
			switch ( (FilterType)iter.first ) {
				case FilterType_None:
					gl::color( Colorf::black() );
					break;
				case FilterType_Noise:
					gl::color( Color8u::hex( 0xEEEEEE ) );
					break;
				case FilterType_ExponentialDouble:
					gl::color( Color8u::hex( 0x0FA5F2 ) );
					break;
				case FilterType_ExponentialSingle:
					gl::color( Color8u::hex( 0xFF9752 ) );
					break;
				case FilterType_Kalman:
					gl::color( Color8u::hex( 0x0EBB52 ) );
					break;
				case FilterType_MovingAverage:
					gl::color( Color8u::hex( 0xFF5252 ) );
					break;
				case FilterType_OneEuro:
					gl::color( Color8u::hex( 0x920BFE ) );
					break;
				default:
					break;
			}
			
			// Draw signal
			gl::begin( GL_LINE_STRIP );
			float h						= (float)getWindowHeight() * 0.5f;
			float x						= (float)getWindowWidth();
			const vector<float>& signal	= mOutput.at( iter.first );
			size_t numSamples			= signal.size();
			if ( numSamples > 0 ) {
				float d = x / (float)( mNumSamples - 1 );
				for ( size_t i = 0; i < numSamples; ++i, x -= d ) {
					float y = signal.at( i ) * h * 0.5f + h;
					gl::vertex( x, y );
				}
			}
			gl::end();
		}
	}
	
	mParams->draw();
}

void FilterApp::setup()
{
	gl::enableVerticalSync();
	
	// Initialize members
	mAlphaExponentialDouble				= 0.06f;
	mAlphaExponentialSingle				= 0.11f;
	mFrameRate							= 60.0f;
	mKalmanMeasurementErrorCovariance	= 0.617f;
	mKalmanProcessErrorCovariance		= 0.03f;
	mMovingAverageWindowSize			= 14;
	mNumSamples							= 256;
	mOneEuroCutoffSlope					= 0.007f;
	mOneEuroMinimumCutoff				= 1.0f;
	mOneEuroCutoffForDerivative			= 1.0f;
	mSignalAmplitude					= 1.0f;
	mSignalFrequency					= 2.0f;
	mSignalNoise						= 0.2f;
	
	// Set up filters. By using lamdba methods with size_t keys,
	// we can just slam in filter logic without setting up 
	// classes or buffering routines.
	for ( size_t i = 0; i < (size_t)FilterType_Count; ++i ) {
		mOutput[ i ]	= {};
		mDraw[ i ]		= true;
		switch ( (FilterType)i ) {
			case FilterType_None:
				
				// NONE: Amplify the raw signal
				mInput.setProcess( i, [ & ]() -> float
				{
					return mSignalRawValue * mSignalAmplitude;
				} );
				break;
			case FilterType_Noise:
				
				// NOISE: Copy the last input value
				mInput.setProcess( i, [ & ]() -> float
				{
					float f = 0.0f;
					if ( !mInput.getSamples().empty() ) {
						f = mInput.getSamples().back();
					}
					return f;
				} );
				break;
			case FilterType_ExponentialDouble:
				
				// EXPO DOUBLE: Continue where expo single leaves off
				//				for smoother, but slower, filtering
				mInput.setProcess( i, [ & ]() -> float
				{
					float f = 0.0f;
					if ( !mInput.getSamples().empty() ) {
						static float v = 0.0f;

						const vector<float>& signal = mOutput.at( (size_t)FilterType_ExponentialSingle );
						if ( !signal.empty() ) {
							f = signal.back();
						}

						float invAlpha	= 1.0f - mAlphaExponentialSingle;
						v				= mAlphaExponentialSingle * f + invAlpha * v;
						float a			= 2.0f * f - v;
						float b			= ( mAlphaExponentialSingle / invAlpha ) * ( f - v );
						f				= a + b;
					}
					return f;
				} );
				break;
			case FilterType_ExponentialSingle:
				
				// EXPO SINGLE: Filter from last calculation
				mInput.setProcess( i, [ & ]() -> float
				{
					float f = 0.0f;
					if ( !mInput.getSamples().empty() ) {
						const vector<float>& signal = mOutput.at( (size_t)FilterType_ExponentialSingle );
						
						if ( !signal.empty() ) {
							f = signal.back();
						}
						f = mAlphaExponentialSingle * mInput.getSamples().back() + ( 1.0f - mAlphaExponentialSingle ) * f;
					}
					return f;
				} );
				break;
			case FilterType_Kalman:
				
				// KALMAN: Simple Kalman filter uses input as control vector
				mInput.setProcess( i, [ & ]() -> float
				{
					float f = 0.0f;
					if ( !mInput.getSamples().empty() ) {
						static float estimateProbability = 0.0f;

						const vector<float>& signal = mOutput.at( (size_t)FilterType_Kalman );
						if ( !signal.empty() ) {
							f = signal.back();
						}

						float v				= mInput.getSamples().back();
						float p				= estimateProbability + mKalmanProcessErrorCovariance;
						float kalmanGain	= p * ( 1.0f / (p + mKalmanMeasurementErrorCovariance ) );
						f					= f + kalmanGain * ( v - f );
						estimateProbability	= ( 1.0f- kalmanGain ) * p;
					}
					return f;
				} );
				break;
			case FilterType_MovingAverage:
				
				// MOVING AVERAGE: Set value to average of samples in window size
				mInput.setProcess( i, [ & ]() -> float
				{
					float f = 0.0f;
					if ( !mInput.getSamples().empty() ) {
						int32_t count = (int32_t)mInput.getNumSamples();
						int32_t start = max( count - mMovingAverageWindowSize, 0 );
						for ( int32_t i = start; i < count; ++i ) {
							f += (float)mInput.getSamples().at( i );
						}
						f /= (float)mMovingAverageWindowSize;
					}
					return f;
				} );
				break;
			case FilterType_OneEuro:
				
				// ONE EURO: Classless implemenetation of the One Euro filter
				mInput.setProcess( i, [ & ]() -> float
				{
					float f = 0.0f;
					if ( !mInput.getSamples().empty() ) {
						static float value				= 0.0f;
						static float valueDerivative	= 0.0f;
						static float valueRaw			= 0.0f;
						
						function<float( float, float )> getAlpha = []( float cutoff, float step )
						{
							return 1.0f / ( 1.0f + ( 1.0f / ( 2.0f * (float)M_PI * cutoff ) ) / step );
						};
					
						f						= mInput.getSamples().back();
						float m					= f < 0.0f ? -1.0f : 1.0f;
						f						= abs( f );
						float d					= ( f - valueRaw ) * mFrameRate;
						valueRaw				= f;
						float step				= 1.0f / mFrameRate;
						float alphaDerivative	= getAlpha( mOneEuroCutoffForDerivative, step );
						valueDerivative			= alphaDerivative * d + ( 1.0f - alphaDerivative ) * valueDerivative;
						float alpha				= getAlpha( mOneEuroMinimumCutoff + mOneEuroCutoffSlope * fabs( valueDerivative ), step );
						value					= alpha * f + ( 1.0f - alpha ) * value;
						f						= value * m;
					}
					return f;
				} );
				break;
			default:
				break;
		}
	}
	
	// Set up the parameters window
	mParams = params::InterfaceGl::create( "Params", ivec2( 300, 450 ) );
	mParams->addParam( "Frame rate",	&mFrameRate,			"", true );
	mParams->addButton( "Quit",			[ & ]() { quit(); },	"key=q" );
	
	mParams->addParam( "Amplitude",		&mSignalAmplitude ).max( 2.0f ).min( 0.0f ).step( 0.01f ).group( "Signal" );
	mParams->addParam( "Frequency",		&mSignalFrequency ).max( 44100.0f ).min( 1.0f ).step( 1.0f ).group( "Signal" );
	mParams->addParam( "Noise",			&mSignalNoise ).max( 1.0f ).min( 0.0f ).step( 0.001f ).group( "Signal" );
	mParams->addParam( "Num samples",	&mNumSamples ).max( 1024 ).min( 2 ).step( 1 ).group( "Signal" );
	
	mParams->addParam( "Alpha single",	&mAlphaExponentialSingle ).max( 1.0f ).min( 0.0f ).step( 0.001f ).group( "Exponential" );
	mParams->addParam( "Alpha double",	&mAlphaExponentialDouble ).max( 1.0f ).min( 0.0f ).step( 0.0001f ).group( "Exponential" );
	
	mParams->addParam( "Process error covariance",		&mKalmanProcessErrorCovariance ).max( 1.0f ).min( 0.0f ).step( 0.001f ).group( "Kalman" );
	mParams->addParam( "Measurement error covariance",	&mKalmanMeasurementErrorCovariance ).max( 1.0f ).min( 0.0f ).step( 0.001f ).group( "Kalman" );
	
	mParams->addParam( "Window size",			&mMovingAverageWindowSize ).max( 1024 ).min( 1 ).step( 1 ).group( "Moving average" );
	
	mParams->addParam( "Cutoff slope",			&mOneEuroCutoffSlope ).max( 1.0f ).min( 0.001f ).step( 0.001f ).group( "One euro" );
	mParams->addParam( "Minimum cutoff",		&mOneEuroMinimumCutoff ).max( 10.0f ).min( 0.01f ).step( 0.01f ).group( "One euro" );
	mParams->addParam( "Cutoff for derivative",	&mOneEuroCutoffForDerivative ).max( 10.0f ).min( 0.01f ).step( 0.01f ).group( "One euro" );
	
	mParams->addParam( "Draw raw",				&mDraw.at( FilterType_None ) ).group( "Draw" );
	mParams->addParam( "Draw noise",			&mDraw.at( FilterType_Noise ) ).group( "Draw" );
	mParams->addParam( "Draw expo double",		&mDraw.at( FilterType_ExponentialDouble ) ).group( "Draw" );
	mParams->addParam( "Draw expo single",		&mDraw.at( FilterType_ExponentialSingle ) ).group( "Draw" );
	mParams->addParam( "Draw Kalman",			&mDraw.at( FilterType_Kalman ) ).group( "Draw" );
	mParams->addParam( "Draw moving average",	&mDraw.at( FilterType_MovingAverage ) ).group( "Draw" );
	mParams->addParam( "Draw one euro",			&mDraw.at( FilterType_OneEuro ) ).group( "Draw" );
}

void FilterApp::update()
{
	mFrameRate = getAverageFps();
	
	// Resize sampler as needed
	if ( mInput.getNumSamples() != (size_t)mNumSamples ) {
		mInput.setNumSamples( (size_t)mNumSamples );
	}
	
	// Moving average window size cannot exceed sample count
	mMovingAverageWindowSize = min( mMovingAverageWindowSize, mNumSamples );
	
	// Generate a value for the input signal
	mSignalRawValue	= sinf( (float)getElapsedSeconds() * mSignalFrequency );
	float v			= mSignalRawValue;
	v				+= randFloat( -mSignalNoise, mSignalNoise );
	v				*= mSignalAmplitude;
	mInput.pushBack( v );
	
	// Add filtered values to each output signal
	for ( size_t i = 0; i < (size_t)FilterType_Count; ++i ) {
		float f = mInput.runProcess( i );
		mOutput[ i ].push_back( f );
		while ( mOutput[ i ].size() > mInput.getNumSamples() ) {
			mOutput[ i ].erase( mOutput[ i ].begin() );
		}
	}
}

CINDER_APP( FilterApp, RendererGl, []( App::Settings* settings )
{
	settings->disableFrameRate();
	settings->setWindowSize( 1280, 720 );
} )

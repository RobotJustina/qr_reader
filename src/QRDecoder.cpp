/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** QRDecoder.cpp
** Class implementation for wrapping over the libdecodeqr library running
** asynchronously along with a ros node.
** 
** Author: Mauricio Matamoros
** -------------------------------------------------------------------------*/

#include "QRDecoder.h"
using namespace qr_reader;

QRDecoder::QRDecoder(): mainThread(NULL){
	decoder = qr_decoder_open();
}

QRDecoder::~QRDecoder(){
	qr_decoder_close(decoder);
}

void QRDecoder::addTextRecognizedHandler(const stringFunctionType& handler){
	textRecognized.connect(handler);
}

void QRDecoder::beginRecognize(cv_bridge::CvImageConstPtr& imgPtr){
	// std::cout << "." << std::flush;
	boost::mutex::scoped_lock lock(mutex, boost::try_to_lock);
	{
		sImgPtr = imgPtr;
		condition.notify_one();
	}
}

std::string QRDecoder::info(){
	std::string s("libdecodeqr version: ");
	s+= qr_decoder_version();
	return s;
}

void QRDecoder::mainThreadTask(){
	run();
}


bool QRDecoder::decode(cv_bridge::CvImageConstPtr& imgPtr, QrCodeHeader& header){
	if( qr_decoder_is_busy(decoder) )
		return false;

	short result;
	short size = 25;
	try{
		IplImage* iplImg = new IplImage(imgPtr->image);
		qr_decoder_set_image_buffer(decoder, iplImg);
		do{
			// result = qr_decoder_decode_image(decoder, iplImg, size);
			result = qr_decoder_decode(decoder, size);
			result&= QR_IMAGEREADER_DECODED;
			size -= 2;
		}while( (size > 1) && (result == 0) );
		// delete iplImg;

		if( !qr_decoder_get_header(decoder, &header) )
			return false;
	}
	catch ( ... ){
		qr_decoder_close(decoder);
		decoder = qr_decoder_open();
		return false;
	}
	return true;
}

bool QRDecoder::fetchText(const QrCodeHeader& header, std::string& text){
	bool readable = true;
	char *buffer = NULL;
	try{
		size_t bufferSize = header.byte_size + 1;
		char *buffer = new char[bufferSize];
		qr_decoder_get_body(decoder, (unsigned char*)buffer, bufferSize);
		text.assign(buffer, bufferSize);
		/*
		size_t ix;
		for(ix = 0; (ix < bufferSize-1) && (buffer[ix] != 0); ++ix){
			if((buffer[ix] < 32) || (buffer[ix] > 126)){
				readable = false;
				break;
			}
		}
		if(ix < 1)
			readable = 0;
		*/
	}
	catch ( ... ){
		if(buffer != NULL)
			delete[] buffer;
		return false;
	}
	delete[] buffer;
	return readable;
}

bool QRDecoder::recognize(cv_bridge::CvImageConstPtr& imgPtr, std::string& text){
	QrCodeHeader header;
	if(!decode(imgPtr, header))
		return false;
	if(!fetchText(header, text))
		return false;
	std::cout << "QR Text: " << text << std::flush << std::endl;
	return true;
}

void QRDecoder::run(){
	while(true){
		boost::mutex::scoped_lock lock(mutex);
		{
			condition.wait(lock);
			std::string text;
			if(recognize(sImgPtr, text) && !textRecognized.empty())
				textRecognized(text);
		}
	}
}

void QRDecoder::runAsync(){
	if(mainThread != NULL)
		return;
	mainThread = new boost::thread(&QRDecoder::mainThreadTask, this);
}
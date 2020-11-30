#include "KinectSkeletonApplication.h"


KinectSkeletonApplication::KinectSkeletonApplication() {
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return;
	}

	data = new std::vector<GLubyte>(WIDTH * HEIGHT * 4);
	if (sensor) {
		sensor->Open();

		IColorFrameSource* frameSource = NULL;
		sensor->get_ColorFrameSource(&frameSource);
		frameSource->OpenReader(&reader);
		if (frameSource) {
			frameSource->Release();
			frameSource = NULL;
		}
		getKinectData();
	}
}


void KinectSkeletonApplication::run() {
	while (true) {

	}
}

void KinectSkeletonApplication::printData() {
	for (int i = 0; i < data->size(); i++) {
		//std::cout << << std::endl;
	}
}
void KinectSkeletonApplication::retrieveFrameSettings(IColorFrame* frame) {
	IColorCameraSettings* cameraSettings;
	frame->get_ColorCameraSettings(&cameraSettings);
	float gain;
	TIMESPAN exposure;
	cameraSettings->get_Gain(&gain);
	cameraSettings->get_ExposureTime(&exposure);

}
void KinectSkeletonApplication::getKinectData() {
	IColorFrame* frame = NULL;
	if (SUCCEEDED(reader->AcquireLatestFrame(&frame))) {
		frame->CopyConvertedFrameDataToArray(WIDTH * HEIGHT * 4, data->data(), ColorImageFormat_Bgra);
	}
	if (frame) frame->Release();
}
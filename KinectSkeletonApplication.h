#pragma once
#include <Kinect.h>
#include <Windows.h>
#include "utils.h"
#include <iostream>
class KinectSkeletonApplication
{
private:
	IKinectSensor* sensor;
	IColorFrameReader* reader;
	std::vector<GLubyte> *data;
	void retrieveFrameSettings(IColorFrame* frame);

public:
	void printData();

	std::vector<GLubyte>* getData() { return data; }
	KinectSkeletonApplication();
	void run();
	void getKinectData();
};


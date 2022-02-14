#ifndef KINECT_WRAPPER_HPP
#define KINECT_WRAPPER_HPP

#include "../Wrapper.h"
#include <iostream>
using namespace std;

class KinectWrapper : public Wrapper
{
	public:
		bool LoadSingleFrame(Frame &fc, MultiViewGeometry::CameraPara& camera){}
	private:
		bool init(MultiViewGeometry::CameraPara& camera){std::cout << "Sorry, Kinect Wrapper isn't done yet." << std::endl;}
};

#endif /*KINECT_WRAPPER_HPP*/
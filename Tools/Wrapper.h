#ifndef WRAPPER_H
#define WRAPPER_H

#include "GCSLAM/MultiViewGeometry.h"
#include "GCSLAM/frame.h"
#include <opencv2/core.hpp>

#define DEVIGNETTING 0
#define UNDISTORTION 0

class Wrapper
{
	public:
		virtual bool LoadSingleFrame(Frame &fc, MultiViewGeometry::CameraPara& camera)=0;
		virtual bool init(MultiViewGeometry::CameraPara &camera)=0;
};

#endif WRAPPER_H

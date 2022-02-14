#define OPENNI2_FOUND 1
#define LIBREALSENSE_FOUND 0
#define LIBFREENECT2_FOUND 0

#include "Wrapper.h"
#include "DatasetWrapper.hpp"

#if OPENNI2_FOUND
#include "OpenNI2/OpenNI2Wrapper.hpp"
#endif

#if LIBREALSENSE_FOUND
#include "RealSense/RealSenseWrapper.hpp"
#endif

#if LIBFREENECT2_FOUND
#include "Kinect/KinectWrapper.hpp"
#endif

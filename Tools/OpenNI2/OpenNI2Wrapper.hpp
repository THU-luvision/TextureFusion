#ifndef OPENNI2_WRAPPER_HPP
#define OPENNI2_WRAPPER_HPP
#include "LiveLogReader.h"
#include "../Wrapper.h"
#include <radical/radiometric_response.h>
#include <radical/vignetting_response.h>

class OpenNI2Wrapper : public Wrapper
{
	public:
		OpenNI2Wrapper();
		~OpenNI2Wrapper();
		bool LoadSingleFrame(Frame &fc, MultiViewGeometry::CameraPara& camera);
	private:
		bool init(MultiViewGeometry::CameraPara& camera);
		void framePreprocess(Frame &fc, MultiViewGeometry::CameraPara& camera);
		LiveLogReader* logReader;
#if DEVIGNETTING
		radical::RadiometricResponse rr;
		radical::VignettingResponse vr;
#endif
		int frame_index;
};

#if DEVIGNETTING
OpenNI2Wrapper::OpenNI2Wrapper():rr("ps1080.crf"),vr("ps1080.vgn"){}
#else
OpenNI2Wrapper::OpenNI2Wrapper(){}
#endif

OpenNI2Wrapper::~OpenNI2Wrapper(){delete logReader;}

bool OpenNI2Wrapper::init(MultiViewGeometry::CameraPara& camera)
{
	frame_index = 0;
	logReader = new LiveLogReader("", 0, LiveLogReader::CameraType::OpenNI2);
    if (!logReader->cam->ok()) exit(1);
	if(logReader != NULL)
	{
		// for xtion, sensors need to be calibrated
		camera.c_fx = 533;
		camera.c_fy = 533;
		camera.c_cx = 315;
		camera.c_cy = 236;
		camera.height = 480;
		camera.width = 640;
		camera.depth_scale = 1000;
		camera.maximum_depth = 8;
		camera.d[0]=0.0423;
		camera.d[1]=-0.1110;
		camera.d[2]=0.0011;
		camera.d[3]=0.0005;
		camera.d[4]=0.0423;
	}
	return true;
}

bool OpenNI2Wrapper::LoadSingleFrame(Frame &fc, MultiViewGeometry::CameraPara& camera)
{
    if (logReader == nullptr) return false;
    int width = camera.width;
    int height = camera.height;

    logReader->getNext();

    fc.frame_index = frame_index++;

    fc.rgb.release();
    fc.rgb.create(height,width,CV_8UC3);
    memcpy(fc.rgb.data,logReader->rgb,width*height*3);

    fc.depth.release();
    fc.depth.create(height,width,CV_16UC1);
    memcpy(fc.depth.data,logReader->depth,width*height*sizeof(short));

    framePreprocess(fc, camera);
    if(fc.frame_index > 30) logReader ->setAuto(false);
	return true;
}

void OpenNI2Wrapper::framePreprocess(Frame &t, MultiViewGeometry::CameraPara& camera)
{
	int height = t.depth.rows;
    int width = t.depth.cols;
    t.refined_depth.create(height,width,CV_32FC1);
    t.weight.create(height,width,CV_32FC1);
	
#if UNDISTORTION
	cv::Mat tmp_rgb;
	cv::Mat intrinsics = cv::Mat::eye(3,3,CV_32FC1);
	intrinsics.at<float>(0,0) = camera.c_fx;
	intrinsics.at<float>(1,1) = camera.c_fy;
	intrinsics.at<float>(0,2) = camera.c_cx;
	intrinsics.at<float>(1,2) = camera.c_cy;
	cv::Mat distorts = cv::Mat::zeros(5,1,CV_32FC1);
	distorts.at<float>(0,0) = camera.d[0];
	distorts.at<float>(0,0) = camera.d[1];
	distorts.at<float>(0,0) = camera.d[2];
	distorts.at<float>(0,0) = camera.d[3];
	distorts.at<float>(0,0) = camera.d[4];

	
	cv::undistort(t.rgb, tmp_rgb, intrinsics, distorts);
	t.rgb = tmp_rgb;
	

	
#endif
	
    #pragma omp parallel for
    for(int i = 0; i < height * width ; i++)
    {
        if(t.depth.at<unsigned short>(i) > camera.maximum_depth * camera.depth_scale)
        {
            t.depth.at<unsigned short>(i) = 0;
        }
        t.refined_depth.at<float>(i) = float(t.depth.at<unsigned short>(i)) / camera.depth_scale;
        t.weight.at<float>(i) = 0;
    }

    /***************bilateral filter***************/
#if 1
    cv::Mat filteredDepth;
    int bilateralFilterRange = 9;
#if MobileCPU
    bilateralFilterRange = 7;
#endif

    cv::bilateralFilter(t.refined_depth, filteredDepth, bilateralFilterRange, 0.03,4.5);
    t.refined_depth = filteredDepth;
    /***************remove boundary***************/

    float *refined_depth_data = (float *)t.refined_depth.data;
    unsigned short *depth_data = (unsigned short*)t.depth.data;
//    for(int i = 0; i < height * width; i++)
//    {
//        if(fabs(refined_depth_data[i] - float(depth_data[i]) / camera.depth_scale) > 0.02)
//        {
//             refined_depth_data[i] = 0;
//             depth_data[i] = 0;
//        }
//    }
//    removeBoundary(t.refined_depth);
#endif

#if DEVIGNETTING
    for(int i = 0; i < height * width ; i++)
    {
        t.depth.at<unsigned short>(i) = t.refined_depth.at<float>(i) * camera.depth_scale;
    }
    t.depth_scale = camera.depth_scale;
	
	cv::Mat irradiance, radiance;
	rr.inverseMap(t.rgb, irradiance);
	vr.remove(irradiance, radiance);
	t.rgb.release();
	rr.directMap(radiance, t.rgb);
#endif
}

#endif
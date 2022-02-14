#ifndef REALSENSE_WRAPPER_HPP
#define REALSENSE_WRAPPER_HPP
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include "RealSenseInterface.h"
#include "../Wrapper.h"

class RealSenseWrapper : public Wrapper
{
	public:
		RealSenseWrapper();
		~RealSenseWrapper();
		bool LoadSingleFrame(Frame &fc, MultiViewGeometry::CameraPara& camera);
	private:
		bool init(MultiViewGeometry::CameraPara& camera);
		void framePreprocess(Frame &fc, MultiViewGeometry::CameraPara& camera);
		rs2::pipeline pipe;
		int frame_index;
};

RealSenseWrapper::RealSenseWrapper(){}

RealSenseWrapper::~RealSenseWrapper(){}

bool RealSenseWrapper::init(MultiViewGeometry::CameraPara& camera)
{
	frame_index = 0;
	
	rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);

    try{pipe.start(cfg);}
    catch(rs2::error){return false;}
    rs2::stream_profile stream = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH);
    auto video_stream = stream.as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics = video_stream.get_intrinsics();
    auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
    auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
    rs2_distortion model = intrinsics.model;

    camera.c_fx = focal_length.first;
    camera.c_fy = focal_length.second;
    camera.c_cx = principal_point.first;
    camera.c_cy = principal_point.second;
    camera.height = 480;
    camera.width = 640;
    camera.depth_scale = 1000;
    camera.maximum_depth = 8;
    memset(camera.d,0,sizeof(float) * 5);
	return true;
}

bool RealSenseWrapper::LoadSingleFrame(Frame &fc, MultiViewGeometry::CameraPara& camera)
{
    rs2::pipeline_profile profile = pipe.get_active_profile();
    float depth_scale = get_depth_scale(profile.get_device());

    rs2::frameset frameset = pipe.wait_for_frames();

    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH);
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR);
    rs2_extrinsics e = depth_stream.get_extrinsics_to(color_stream);
    rs2_intrinsics depthI = depth_stream.as<rs2::video_stream_profile>().get_intrinsics();
    rs2_intrinsics colorI = color_stream.as<rs2::video_stream_profile>().get_intrinsics();
    Eigen::Matrix3f R;
    Eigen::Vector3f t;


    R << e.rotation[0],e.rotation[1],e.rotation[2],
            e.rotation[3],e.rotation[4],e.rotation[5],
            e.rotation[6],e.rotation[7],e.rotation[8];
    t << e.translation[0], e.translation[1], e.translation[2];
    int width = depthI.width;
    int height = depthI.height;


    rs2::video_frame rs2color = frameset.get_color_frame();
    rs2::depth_frame rs2depth = frameset.get_depth_frame();
    const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(rs2depth.get_data());
    uint8_t* p_rgb = reinterpret_cast<uint8_t*>(const_cast<void*>(rs2color.get_data()));


    cv::Mat color_data;
    color_data.create(colorI.height,colorI.width,CV_8UC3);
    memcpy(color_data.data,p_rgb,width*height*3);

    fc.rgb.release();
    fc.rgb.create(height,width,CV_8UC3);
    for(int i = 0; i < height; i++)
    {
      for(int j = 0; j < width; j++)
      {
        fc.rgb.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
      }
    }
    for(int i = 0; i < height; i++)
    {
      for(int j = 0; j < width; j++)
      {

        float d = (float) (p_depth_frame[i * width + j]) / camera.depth_scale;
        Eigen::Vector3f v = Eigen::Vector3f((j - depthI.ppx) / depthI.fx * d,
                             (i - depthI.ppy) / depthI.fy * d,
                             d);
        Eigen::Vector3f new_v = R * v + t;
        int x = new_v(0) * colorI.fx / new_v(2) + colorI.ppx;
        int y = new_v(1) * colorI.fy / new_v(2) + colorI.ppy;
        if(x < width && x > 0 && y < height && y > 0)
        {
           cv::Vec3b c = color_data.at<cv::Vec3b>(y,x);
           fc.rgb.at<cv::Vec3b>(i,j)[0] = c[0];
           fc.rgb.at<cv::Vec3b>(i,j)[1] = c[1];
           fc.rgb.at<cv::Vec3b>(i,j)[2] = c[2];
        }
      }
    }
    fc.depth.release();
    fc.depth.create(height,width,CV_16UC1);
    memcpy(fc.depth.data,p_depth_frame,width*height*sizeof(short));

    if (!rs2depth || !rs2color)
    {
        cout << "warning! no camera observations!" << endl;
        return false;
    }

    fc.frame_index = frame_index++;
	
    framePreprocess(fc,camera);
    return true;
}

void RealSenseWrapper::framePreprocess(Frame &t, MultiViewGeometry::CameraPara& camera)
{
	int height = t.depth.rows;
    int width = t.depth.cols;
    t.refined_depth.create(height,width,CV_32FC1);
    t.weight.create(height,width,CV_32FC1);
    
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

    for(int i = 0; i < height * width ; i++)
    {
        t.depth.at<unsigned short>(i) = t.refined_depth.at<float>(i) * camera.depth_scale;
    }
    t.depth_scale = camera.depth_scale;
}

#endif /*REALSENSE_WRAPPER_HPP*/



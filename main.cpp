#include <omp.h>
#include <stdio.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#include "GCSLAM/GCSLAM.h"
#include "GCSLAM/MILD/BayesianFilter.hpp"
#include "GCSLAM/MILD/loop_closure_detector.hpp"
#include "GCSLAM/MultiViewGeometry.h"
#include "GCSLAM/frame.h"

#include "GCFusion/MapMaintain.hpp"
#include "GCFusion/MobileFusion.h"
#include "GCFusion/MobileGUI.hpp"

#include "Structure/Chisel.h"
#include "open_chisel/Stopwatch.h"
#include "open_chisel/camera/PinholeCamera.h"
#include "open_chisel/utils/ProjectionIntegrator.h"

#include "BasicAPI.h"
#include "Tools/config.h"

using namespace std;
using namespace cv;

#define MULTI_THREAD 1

#define INPUT_SOURCE_DATABASE 0
#define INPUT_SOURCE_OPENNI 1
#define INPUT_SOURCE_REALSENSE 2
#define INPUT_SOURCE_KINECT 3

int main(int argc, char *argv[]) {
  int showCaseMode = 0;
  string basepath;
  float inputVoxelResolution = 0.005;
  int sensorType = 0;

  BasicAPI::parseInput(argc, argv, showCaseMode, inputVoxelResolution, basepath,
                       MultiViewGeometry::g_para, sensorType);

  vector<float> processingTimePerFrame;

  MultiViewGeometry::CameraPara camera;

  Wrapper *wrap;

  if (sensorType == INPUT_SOURCE_DATABASE) wrap = new DatasetWrapper(basepath);
#if OPENNI2_FOUND
  if (sensorType == INPUT_SOURCE_OPENNI) wrap = new OpenNI2Wrapper();
#endif
#if LIBREALSENSE_FOUND
  if (sensorType == INPUT_SOURCE_REALSENSE) wrap = new RealSenseWrapper();
#endif
#if LIBFREENECT2_FOUND
  if (sensorType == INPUT_SOURCE_REALSENSE) wrap = new KinectWrapper();
#endif
  // choose sensor type according to input
  if (!wrap->init(camera)) {
    std::cout << "fail to init camera" << std::endl;
    exit(1);
  }

  cout << "begin init rendering" << endl;
  MobileGUI gui(showCaseMode);
  MobileFusion gcFusion;
  gcFusion.initChiselMap(camera, inputVoxelResolution,
                         MultiViewGeometry::g_para.far_plane_distance);

  cout << "begin init gcSLAM" << endl;
  int maxFrameNum = 20000;
  gcFusion.initGCSLAM(maxFrameNum, MultiViewGeometry::g_para, camera);

  pangolin::GlTexture imageTexture(640, 480, GL_RGB, false, 0, GL_RGB,
                                   GL_UNSIGNED_BYTE);

#if MULTI_THREAD
  boost::thread map_thread(
      boost::bind(&MobileFusion::MapManagement, &gcFusion));
#endif

  float integrateLocalFrameNum = 6;

  int portableDeviceFlag = 0;
  // for portable devices
  if (thread::hardware_concurrency() < 4) {
    portableDeviceFlag = 1;
    integrateLocalFrameNum = 3;
  }

  double localization_time = 0;

  while (!pangolin::ShouldQuit() && (wrap != NULL) && !gcFusion.is_stopped)
  //**********************************************************the main
  // loop*********************************************************
  {
    if (!gui.pause->Get() || pangolin::Pushed(*gui.step)) {
      Frame f;
      TICK("System::1::FrameTime");
      if (!wrap->LoadSingleFrame(f, camera)) {
        gcFusion.is_stopped = true;
        break;
      }

      int feature_num = MultiViewGeometry::g_para.max_feature_num;
      if (portableDeviceFlag) feature_num = 600;
      BasicAPI::detectAndExtractFeatures(f, feature_num, camera);
      BasicAPI::extractNormalMapSIMD(f.refined_depth, f.normal_map, camera.c_fx,
                                     camera.c_fy, camera.c_cx, camera.c_cy);

      gcFusion.gcSLAM.update_frame(f);

      Frame &frame_current = gcFusion.gcSLAM.globalFrameList.back();

      if (frame_current.tracking_success &&
          !frame_current.is_keyframe)  // if not keyframe
      {
        int keyframeIndex =
            gcFusion.gcSLAM.GetKeyframeDataList().back().keyFrameIndex;
        BasicAPI::refineKeyframesSIMD(
            gcFusion.gcSLAM.globalFrameList[keyframeIndex], frame_current,
            camera);
        BasicAPI::refineNewframesSIMD(
            gcFusion.gcSLAM.globalFrameList[keyframeIndex], frame_current,
            camera);
      }
      BasicAPI::refineDepthUseNormalSIMD(
          (float *)frame_current.normal_map.data,
          (float *)frame_current.refined_depth.data, camera.c_fx, camera.c_fy,
          camera.c_cx, camera.c_cy, camera.width, camera.height);

      if (frame_current.is_keyframe) {
        BasicAPI::checkColorQuality(frame_current.normal_map,
                                    frame_current.colorValidFlag, camera.c_fx,
                                    camera.c_fy, camera.c_cx, camera.c_cy);
        BasicAPI::estimateColorQuality(
            frame_current.refined_depth, frame_current.normal_map,
            frame_current.observationQualityMap, frame_current.rgb, camera.c_fx,
            camera.c_fy, camera.c_cx, camera.c_cy);

        gcFusion.clearRedudentFrameMemory(integrateLocalFrameNum);
#if MULTI_THREAD
        gcFusion.updateGlobalMap(gcFusion.gcSLAM.globalFrameList.size(),
                                 gcFusion.gcSLAM.globalFrameList.size() - 1);

#else
        gcFusion.tsdfFusion(gcFusion.gcSLAM.globalFrameList,
                            gcFusion.gcSLAM.globalFrameList.size() - 1,
                            gcFusion.gcSLAM.GetKeyframeDataList(),
                            gcFusion.gcSLAM.GetKeyframeDataList().size() - 2);

#endif
      }

      imageTexture.Upload(gcFusion.gcSLAM.globalFrameList.back().rgb.data,
                          GL_RGB, GL_UNSIGNED_BYTE);
      // imageTexture显示实际看到的场景，非贴图

      float memoryConsumption = 0;
      for (int k = 0; k < gcFusion.gcSLAM.globalFrameList.size(); k++) {
        memoryConsumption +=
            gcFusion.gcSLAM.globalFrameList[k].GetOccupiedMemorySize();
      }
      //            cout << "memory for frames: " << memoryConsumption / 1024 /
      //            1024 << " " << memoryConsumption / 1024 / 1024
      //            /gcFusion.gcSLAM.globalFrameList.size() << endl;
      TOCK("System::1::FrameTime");
      if (frame_current.is_keyframe)
        printf("frame %d is key frame!\n",
               gcFusion.gcSLAM.globalFrameList.back().frame_index);
      printf("frame %d time: %fms\r\n",
             gcFusion.gcSLAM.globalFrameList.back().frame_index,
             Stopwatch::getInstance().getTiming("System::1::FrameTime"));
      processingTimePerFrame.push_back(
          Stopwatch::getInstance().getTiming("System::1::FrameTime"));
    }

    TICK("System::2::GUI");
    if (gui.followPose->Get()) {
      Eigen::Matrix4f currPose;
      if (gcFusion.gcSLAM.globalFrameList.back().tracking_success &&
          gcFusion.gcSLAM.globalFrameList.back().origin_index == 0) {
        currPose = gcFusion.gcSLAM.globalFrameList.back()
                       .pose_sophus[0]
                       .matrix()
                       .cast<float>();
      }
      gui.setModelView(currPose, camera.c_fy < 0);
    }
    gui.PreCall();
    if (gui.drawGlobalModel->Get()) {
      gcFusion.MobileShow(gui.s_cam.GetProjectionModelViewMatrix(),
                          VERTEX_WEIGHT_THRESHOLD, gui.drawUnstable->Get(),
                          gui.drawNormals->Get(), gui.drawColors->Get(),
                          gui.drawPoints->Get(), gui.drawOrigin->Get(),
                          gcFusion.gcSLAM.globalFrameList.size(), 1,
                          gcFusion.gcSLAM.globalFrameList);
    }
    // gui.DisplayImg(gui.RGB,&imageTexture);
    gui.PostCall();
    TOCK("System::2::GUI");
  }

  std::ofstream timeout("time.txt", std::ios::out | std::ios::trunc);
  for (int i = 0; i < processingTimePerFrame.size(); i++) {
    timeout << i << " " << processingTimePerFrame[i] << std::endl;
  }
  std::ofstream chunkout("chunk.txt", std::ios::out | std::ios::trunc);
  for (int i = 0; i < gcFusion.chunk_log.size(); i++) {
    chunkout << gcFusion.chunk_log[i] << " " << gcFusion.chunk_scale[i] << " "
             << gcFusion.chunk_complex[i] << std::endl;
  }

  std::ofstream statout("stat.txt", std::ios::out | std::ios::trunc);
  statout
      << localization_time / gcFusion.gcSLAM.globalFrameList.size() << " "
      << gcFusion.integration_time /
             gcFusion.gcSLAM.GetKeyframeDataList().size()
      << " "
      << gcFusion.meshing_time / gcFusion.gcSLAM.GetKeyframeDataList().size()
      << " " << gcFusion.mrf_time / gcFusion.gcSLAM.GetKeyframeDataList().size()
      << " "
      << gcFusion.adjustment_time / gcFusion.gcSLAM.GetKeyframeDataList().size()
      << " "
      << gcFusion.rendering_time / gcFusion.gcSLAM.GetKeyframeDataList().size()
      << " " << std::endl;

  char fileName[2560];
  memset(fileName, 0, 2560);
  sprintf(fileName, "%s/trajectory.txt", basepath.c_str());
  BasicAPI::saveTrajectoryFrameList(gcFusion.gcSLAM.globalFrameList, fileName);

  float total_time = 0;
  for (int i = 0; i < processingTimePerFrame.size(); i++) {
    total_time += processingTimePerFrame[i];
  }
  cout << "average processing time per frame: "
       << total_time / processingTimePerFrame.size() << endl;

  /*size_t pixel_num = 0;
  for (auto patchit: gcFusion.chiselMap->atlas.allPatches)
  {
          pixel_num+=
  patchit.second->boundingbox.height*patchit.second->boundingbox.width;
  }
  std::cout << "Memory Storage Stats:"
                    << (gcFusion.tsdf_vertice_num*sizeof(float)*9 +
  gcFusion.tsdf_indice_num*sizeof(unsigned int)
                    + pixel_num*sizeof(unsigned char)*3)/1024.0f/1024
                    <<"/"<<
  gcFusion.tsdf_indice_num*sizeof(float)*7/1024.0f/1024 << std::endl;
*/
  memset(fileName, 0, 2560);
  sprintf(fileName, "%s/OnlineModel_%dmm.ply", basepath.c_str(),
          (int)(1000 * (gcFusion.GetVoxelResolution())));
  cout << "saving online model to:    " << fileName << endl;
  gcFusion.chiselMap->SaveAllMeshesToPLY(fileName);
  gcFusion.chiselMap->Reset();

  cout << "saving textured model" << endl;
  gcFusion.chiselMap->atlas.SaveTexturedModel("./output");

  cout << "offline re-integrating all frames" << endl;
  TICK("Final::IntegrateAllFrames");
  for (int i = 0; i < gcFusion.gcSLAM.globalFrameList.size(); i++) {
    gcFusion.IntegrateFrame(gcFusion.gcSLAM.globalFrameList[i]);
  }
  TOCK("Final::IntegrateAllFrames");
  gcFusion.chiselMap->UpdateMeshes(gcFusion.cameraModel);

  memset(fileName, 0, 2560);
  sprintf(fileName, "%s/finalModelAllframes_%dmm.ply", basepath.c_str(),
          (int)(1000 * (gcFusion.GetVoxelResolution())));
  cout << "saving offline model to:    " << fileName << endl;
  gcFusion.chiselMap->SaveAllMeshesToPLY(fileName);
  gcFusion.chiselMap->Reset();

  BasicAPI::makeDir("texture");
  for (int i = 0; i < gcFusion.gcSLAM.KeyframeDataList.size(); i++) {
    MultiViewGeometry::KeyFrameDatabase kfd =
        gcFusion.gcSLAM.KeyframeDataList[i];
    Frame &f = gcFusion.gcSLAM.globalFrameList[kfd.keyFrameIndex];
    if (!f.tracking_success) continue;

    Eigen::MatrixXf transform = f.pose_sophus[0].matrix().cast<float>();
    transform = transform.inverse();
    Eigen::Matrix3f r = transform.block<3, 3>(0, 0);
    Eigen::MatrixXf t = transform.block<3, 1>(0, 3);

    memset(fileName, 0, 2560);
    sprintf(fileName, "%s/texture/%06d.cam", basepath.c_str(), i);
    FILE *fp = fopen(fileName, "w+");
    fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f\r\n", t(0), t(1), t(2),
            r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0),
            r(2, 1), r(2, 2));
    fprintf(fp, "%f %f %f %f %f %f", camera.c_fx / camera.width, camera.d[0],
            camera.d[1], camera.c_fx / camera.c_fy, camera.c_cx / camera.width,
            camera.c_cy / camera.height);
    fclose(fp);
    memset(fileName, 0, 2560);
    sprintf(fileName, "%s/texture/%06d.png", basepath.c_str(), i);
    cv::cvtColor(f.rgb, f.rgb, cv::COLOR_RGB2BGR);
    cv::imwrite(fileName, f.rgb);
  }

  Stopwatch::getInstance().printAll();

  cout << "program finish" << endl;

  gcFusion.is_stopped = true;
#if MULTI_THREAD
  gcFusion.render_barr.wait();
  map_thread.join();
#endif
}

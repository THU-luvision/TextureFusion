#ifndef DATASET_WRAPPER_HPP
#define DATASET_WRAPPER_HPP

#include <radical/radiometric_response.h>
#include <radical/vignetting_response.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "Wrapper.h"

using namespace std;

class DatasetWrapper : public Wrapper {
 public:
  DatasetWrapper(string base_path);
  ~DatasetWrapper();
  bool LoadSingleFrame(Frame &fc, MultiViewGeometry::CameraPara &camera);

 private:
  bool init(MultiViewGeometry::CameraPara &camera);
  void framePreprocess(Frame &fc, MultiViewGeometry::CameraPara &camera);
  void spilt_word(string ori, vector<string> &res);

  string work_folder;
  vector<string> rgb_files;
  vector<string> depth_files;
  vector<double> time_stamp;
  int frame_index;

#if DEVIGNETTING
  radical::RadiometricResponse rr;
  radical::VignettingResponse vr;
#endif
};

#if DEVIGNETTING
DatasetWrapper::DatasetWrapper(string base_path)
    : rr("Params/ps1080.crf"), vr("Params/ps1080.vgn") {
#else
DatasetWrapper::DatasetWrapper(string base_path) {
#endif
  work_folder = base_path;
}

DatasetWrapper::~DatasetWrapper() {}

void DatasetWrapper::spilt_word(string ori, vector<string> &res) {
  string buf;            // Have a buffer string
  stringstream ss(ori);  // Insert the string into a stream
  while (ss >> buf) res.emplace_back(buf);
}

bool DatasetWrapper::init(MultiViewGeometry::CameraPara &camera) {
  frame_index = 0;
  cout << "working folder: " << work_folder << endl;
  rgb_files.clear();
  depth_files.clear();
  string rgb_file, depth_file;
  char fileName[256];
  char line[1000];
  memset(fileName, '\0', 256);
  sprintf(fileName, "%s/associate.txt", work_folder.c_str());

  fstream fin;

  float average_time_delay = 0;
  int count = 0;
  fin.open(fileName, ios::in);
  while (fin.getline(line, sizeof(line), '\n')) {
    string input = line;
    vector<string> input_data;
    spilt_word(line, input_data);
    if (input_data.size() == 4) {
      double tRGB = stod(input_data[0]);
      double tDepth = stod(input_data[0]);
      average_time_delay += tRGB + tDepth;
      count++;
    }
  }
  fin.close();
  average_time_delay /= count;

  fin.open(fileName, ios::in);
  while (fin.getline(line, sizeof(line), '\n')) {
    string input = line;
    vector<string> input_data;
    spilt_word(line, input_data);
    if (input_data.size() == 4) {
      double tRGB = stod(input_data[0]);
      double tDepth = stod(input_data[2]);

      double time = (tRGB + tDepth) / 2;
      rgb_file = work_folder + "/" + input_data[1];
      depth_file = work_folder + "/" + input_data[3];
      rgb_files.emplace_back(rgb_file);
      depth_files.emplace_back(depth_file);
      time_stamp.emplace_back(time);
    }
  }
  fin.close();

  /*memset(fileName, '\0', 256);
  sprintf(fileName, "%s/groundtruth.txt", work_folder.c_str());

  fin.open(fileName, ios::in);
  int lineCnt = 0;
  while (fin.getline(line, sizeof(line), '\n'))
  {

      lineCnt++;
  }
  fin.close();
  fin.open(fileName, ios::in);
  ground_truth = Eigen::MatrixXd(lineCnt,8);
  lineCnt = 0;
  while (fin.getline(line, sizeof(line), '\n'))
  {
      string input = line;
      vector<string> input_data;
      spilt_word(line, input_data);
      if (input_data.size() == 8)
      {
          for (int cnt = 0; cnt < 8; cnt++)
          {
              ground_truth(lineCnt, cnt) = stod(input_data[cnt]);
          }
      }
      lineCnt++;
  }
  fin.close();
      */

  memset(fileName, '\0', 256);
  sprintf(fileName, "%s/calib.txt", work_folder.c_str());
  fin.open(fileName, ios::in);
  while (fin.getline(line, sizeof(line), '\n')) {
    string input = line;
    vector<string> input_data;
    spilt_word(line, input_data);
    if (input_data.size() != 13) {
      cout << "error in loading parameters" << endl;
    } else {
      camera.width = stod(input_data[0]);
      camera.height = stod(input_data[1]);
      camera.c_fx = stod(input_data[2]);
      camera.c_fy = stod(input_data[3]);
      camera.c_cx = stod(input_data[4]);
      camera.c_cy = stod(input_data[5]);
      camera.d[0] = stod(input_data[6]);
      camera.d[1] = stod(input_data[7]);
      camera.d[2] = stod(input_data[8]);
      camera.d[3] = stod(input_data[9]);
      camera.d[4] = stod(input_data[10]);
      camera.depth_scale = stod(input_data[11]);
      camera.maximum_depth = stod(input_data[12]);
    }
  }
  fin.close();
  return true;
}

bool DatasetWrapper::LoadSingleFrame(Frame &t,
                                     MultiViewGeometry::CameraPara &camera) {
  if (rgb_files.size() < frame_index + 1 ||
      depth_files.size() < frame_index + 1) {
    cout << "end of dataset: " << rgb_files.size() << endl;
    return false;
  }
  t.frame_index = frame_index;
  int image_id = frame_index++;
  t.time_stamp = time_stamp[image_id];
  t.rgb = cv::imread(rgb_files[image_id].c_str(), cv::IMREAD_UNCHANGED);
  t.depth = cv::imread(depth_files[image_id].c_str(), cv::IMREAD_UNCHANGED);
  if (t.rgb.rows <= 0 || t.depth.rows <= 0)
    cout << "load image error ! " << rgb_files[image_id] << " "
         << depth_files[image_id] << endl;
  cv::cvtColor(t.rgb, t.rgb, cv::COLOR_BGR2RGB);

  assert(t.rgb.rows > 0);
  assert(t.depth.rows > 0);

  framePreprocess(t, camera);
  return true;
}

void DatasetWrapper::framePreprocess(Frame &t,
                                     MultiViewGeometry::CameraPara &camera) {
  int height = t.depth.rows;
  int width = t.depth.cols;
  t.refined_depth.create(height, width, CV_32FC1);
  t.weight.create(height, width, CV_32FC1);

#if UNDISTORTION
  cv::Mat tmp_rgb;
  cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_32FC1);
  intrinsics.at<float>(0, 0) = camera.c_fx;
  intrinsics.at<float>(1, 1) = camera.c_fy;
  intrinsics.at<float>(0, 2) = camera.c_cx;
  intrinsics.at<float>(1, 2) = camera.c_cy;
  cv::Mat distorts = cv::Mat::zeros(5, 1, CV_32FC1);
  distorts.at<float>(0, 0) = camera.d[0];
  distorts.at<float>(0, 0) = camera.d[1];
  distorts.at<float>(0, 0) = camera.d[2];
  distorts.at<float>(0, 0) = camera.d[3];
  distorts.at<float>(0, 0) = camera.d[4];
  cv::undistort(t.rgb, tmp_rgb, intrinsics, distorts);
  t.rgb = tmp_rgb;
#endif

#pragma omp parallel for
  for (int i = 0; i < height * width; i++) {
    if (t.depth.at<unsigned short>(i) >
        camera.maximum_depth * camera.depth_scale) {
      t.depth.at<unsigned short>(i) = 0;
    }
    t.refined_depth.at<float>(i) =
        float(t.depth.at<unsigned short>(i)) / camera.depth_scale;
    t.weight.at<float>(i) = 0;
  }

  /***************bilateral filter***************/
#if 1
  cv::Mat filteredDepth;
  int bilateralFilterRange = 9;
#if MobileCPU
  bilateralFilterRange = 7;
#endif

  cv::bilateralFilter(t.refined_depth, filteredDepth, bilateralFilterRange,
                      0.03, 10);
  t.refined_depth = filteredDepth;
  /***************remove boundary***************/

  float *refined_depth_data = (float *)t.refined_depth.data;
  unsigned short *depth_data = (unsigned short *)t.depth.data;
//    for(int i = 0; i < height * width; i++)
//    {
//        if(fabs(refined_depth_data[i] - float(depth_data[i]) /
//        camera.depth_scale) > 0.02)
//        {
//             refined_depth_data[i] = 0;
//             depth_data[i] = 0;
//        }
//    }
//    removeBoundary(t.refined_depth);
#endif

  for (int i = 0; i < height * width; i++) {
    t.depth.at<unsigned short>(i) =
        t.refined_depth.at<float>(i) * camera.depth_scale;
  }
  t.depth_scale = camera.depth_scale;

#if DEVIGNETTING
  cv::Mat irradiance, radiance;
  rr.inverseMap(t.rgb, irradiance);
  vr.remove(irradiance, radiance);
  t.rgb.release();
  rr.directMap(radiance, t.rgb);
#endif
}

#endif
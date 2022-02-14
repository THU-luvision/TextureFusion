#include "MobileFusion.h"
#include <algorithm>
#include <cmath>
#include "MapMaintain.hpp"

using namespace std;

#define DEBUG_MODE 0
#define VERTEX_HASHING 1

// for range 1 only.

void GetMapDynamics(const std::vector<Frame> &frame_list,
                    std::vector<int> &keyframesToUpdate,
                    int CurrKeyframeIndex) {
  keyframesToUpdate.clear();
  std::vector<int> keyframeIDList;
  std::vector<float> keyframeCostList;
  keyframeIDList.clear();
  keyframeCostList.clear();
  for (int i = 0; i < CurrKeyframeIndex; i++) {
    if (frame_list[i].is_keyframe && frame_list[i].tracking_success &&
        frame_list[i].origin_index == 0) {
      Eigen::Matrix4f prePose =
          frame_list[i].pose_sophus[1].matrix().cast<float>();
      Eigen::Matrix4f curPose =
          frame_list[i].pose_sophus[0].matrix().cast<float>();

      float cost = GetPoseDifference(prePose, curPose);
      keyframeIDList.emplace_back(i);
      keyframeCostList.emplace_back(cost);
      //            cout << "frames: " << i << " " << cost << endl;
    }
  }

  // only consider dynamic map when keyframe number is larger than
  // movingAveregaeLength Deintegrate 10 keyframes at each time slot
  int movingAverageLength = 5;

  SelectLargestNValues(movingAverageLength, keyframeIDList, keyframeCostList,
                       keyframesToUpdate);
#if MobileCPU
  SelectLargestNValues(movingAverageLength, keyframeIDList, keyframeCostList,
                       keyframesToUpdate);
  SelectLargestNValues(movingAverageLength, keyframeIDList, keyframeCostList,
                       keyframesToUpdate);
#endif
  movingAverageLength = 2;
  SelectLargestNValues(movingAverageLength, keyframeIDList, keyframeCostList,
                       keyframesToUpdate);
  SelectLargestNValues(movingAverageLength, keyframeIDList, keyframeCostList,
                       keyframesToUpdate);
  if (keyframesToUpdate.size() < 5) {
    movingAverageLength = 1;
    SelectLargestNValues(movingAverageLength, keyframeIDList, keyframeCostList,
                         keyframesToUpdate);
    SelectLargestNValues(movingAverageLength, keyframeIDList, keyframeCostList,
                         keyframesToUpdate);
    SelectLargestNValues(movingAverageLength, keyframeIDList, keyframeCostList,
                         keyframesToUpdate);
  }
  //    for(int i = 0; i < keyframesToUpdate.size();i++)
  //    {
  //        cout << "reintegrating keyframe: " << i << " " <<
  //        keyframesToUpdate[i] << endl;
  //    }
}

// clear redudent memory stored in frames, including depth, normal, color,
// features.
void MobileFusion::clearRedudentFrameMemory(int integrateLocalFrameNum) {
  int keyFrameNum = gcSLAM.GetKeyframeDataList().size();
  if (keyFrameNum > 1) {
    MultiViewGeometry::KeyFrameDatabase kd =
        gcSLAM.GetKeyframeDataList()[keyFrameNum - 2];

    float inc = 0;
    for (int k = 0; k < kd.corresponding_frames.size(); k++) {
      if (k < inc - 1e-4) {
        gcSLAM.globalFrameList[kd.corresponding_frames[k]].clear_memory();
        continue;
      }
      inc += kd.corresponding_frames.size() / integrateLocalFrameNum;
      gcSLAM.globalFrameList[kd.corresponding_frames[k]]
          .clear_redudent_memoery();
    }

    gcSLAM.globalFrameList[kd.keyFrameIndex].clear_keyframe_memory();
  }
}

void MobileFusion::updateGlobalMap(int inputValidFrameNum,
                                   int inputFuseKeyframeId) {
  validFrameNum = inputValidFrameNum;
  fuseKeyframeId = inputFuseKeyframeId;
  render_barr.wait();
}

void MobileFusion::MapManagement() {
  while (!is_stopped) {
    TICK("MobileFusion::TSDFFusion()");
    if (tsdfFusion(gcSLAM.globalFrameList, fuseKeyframeId,
                   gcSLAM.GetKeyframeDataList(),
                   gcSLAM.GetKeyframeDataList().size() - 2) < 0)
      is_stopped = true;

    TOCK("MobileFusion::TSDFFusion()");
    Stopwatch::getInstance().printAll();
    render_barr.wait();
  }
  // UpdateGlobalMap_semaphore.post();
}

void MobileFusion::ReIntegrateKeyframe(
    std::vector<Frame> &frame_list,
    const MultiViewGeometry::KeyFrameDatabase &kfDatabase,
    const int integrateFlag) {
  Frame &kf = frame_list[kfDatabase.keyFrameIndex];

  int totalPixelNum = cameraModel.GetWidth() * cameraModel.GetHeight();
  float cx = cameraModel.GetCx();
  float cy = cameraModel.GetCy();
  float fx = cameraModel.GetFx();
  float fy = cameraModel.GetFy();
  int width = cameraModel.GetWidth();
  int height = cameraModel.GetHeight();
  chisel::Transform lastPose;
  ChunkIDList localChunksIntersecting;
  std::vector<void *> localChunksPtr;
  std::vector<bool> localNeedsUpdateFlag;
  std::vector<bool> localNewChunkFlag;
  if (integrateFlag == 1) {
    lastPose = kf.pose_sophus[0].matrix().cast<float>();
    kf.pose_sophus[1] = kf.pose_sophus[0];
  } else if (integrateFlag == 0) {
    lastPose = kf.pose_sophus[1].matrix().cast<float>();
    localChunksIntersecting = kf.validChunks;
    localChunksPtr = kf.validChunksPtr;
    for (int i = 0; i < localChunksIntersecting.size(); i++) {
      localNeedsUpdateFlag.emplace_back(true);
      localNewChunkFlag.emplace_back(false);
    }
  }
  float *depthImageData;
  static unsigned char *colorImageData = new unsigned char[totalPixelNum * 4];

  unsigned char *colorValid = (unsigned char *)kf.colorValidFlag.data;
  depthImageData = (float *)kf.refined_depth.data;
  float *observationQualityPointer = (float *)kf.observationQualityMap.data;
#if 1
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int pos = i * width + j;
      colorImageData[pos * 4 + 0] =
          colorValid[pos] > 0 ? kf.rgb.at<unsigned char>(pos * 3 + 0) : 0;
      colorImageData[pos * 4 + 1] =
          colorValid[pos] > 0 ? kf.rgb.at<unsigned char>(pos * 3 + 1) : 0;
      colorImageData[pos * 4 + 2] =
          colorValid[pos] > 0 ? kf.rgb.at<unsigned char>(pos * 3 + 2) : 0;
      colorImageData[pos * 4 + 3] = colorValid[pos] > 0 ? 1 : 0;
    }
  }
#endif

  if (integrateFlag == 1) {
    // TICK("CHISEL::Reintegration::1::prepareIntersectChunks");
    chiselMap->PrepareIntersectChunks(
        projectionIntegrator, depthImageData, lastPose, cameraModel,
        localChunksIntersecting, localNeedsUpdateFlag, localNewChunkFlag);
    //        chiselMap->GetSearchRegion(searchArea,cameraModel,lastPose);
    // TOCK("CHISEL::Reintegration::1::prepareIntersectChunks");
  }

  // TICK("CHISEL::Reintegration::2::IntegrateKeyDepthAndColor");

  chiselMap->IntegrateDepthScanColor(
      projectionIntegrator, depthImageData, colorImageData, lastPose,
      cameraModel, localChunksIntersecting, localNeedsUpdateFlag, integrateFlag,
      kf.frame_index, observationQualityPointer);
  // TOCK("CHISEL::Reintegration::2::IntegrateKeyDepthAndColor");

#if INTEGRATE_ALL
  // TICK("CHISEL::Reintegration::3::IntegrateLocalDepth");

  // only integrate ten frames evenly distributed in this keyframe

  for (int i = 0; i < kfDatabase.corresponding_frames.size(); i++) {
    Frame &local_frame = frame_list[kfDatabase.corresponding_frames[i]];
    if (local_frame.refined_depth.empty()) continue;
    //            printf("integrating frame: %d %d\r\n",local_frame.frame_index,
    //            integrateFlag);
    if (integrateFlag == 1) {
      lastPose = local_frame.pose_sophus[0].matrix().cast<float>();
      local_frame.pose_sophus[1] = local_frame.pose_sophus[0];
    } else if (integrateFlag == 0) {
      lastPose = local_frame.pose_sophus[1].matrix().cast<float>();
    }
    depthImageData = (float *)local_frame.refined_depth.data;

    chiselMap->IntegrateDepthScanColor(
        projectionIntegrator, depthImageData, NULL, lastPose, cameraModel,
        localChunksIntersecting, localNeedsUpdateFlag, integrateFlag);
  }
  // TOCK("CHISEL::Reintegration::3::IntegrateLocalDepth");
#endif

  // TICK("CHISEL::Reintegration::4::FinalizeIntegrateChunks");
  if (integrateFlag == 1) {
    chiselMap->FinalizeIntegrateChunks(localChunksIntersecting,
                                       localNeedsUpdateFlag, localNewChunkFlag,
                                       kf.validChunks);
  } else if (integrateFlag == 0) {
    std::vector<void *> localChunksPtrValid;
    ChunkIDList localValidChunks;
    chiselMap->FinalizeIntegrateChunks(localChunksIntersecting,
                                       localNeedsUpdateFlag, localNewChunkFlag,
                                       localValidChunks);
    kf.validChunks.clear();
  }
  // TOCK("CHISEL::Reintegration::4::FinalizeIntegrateChunks");
}

void MobileFusion::IntegrateFrame(const Frame &frame_ref) {
  int totalPixelNum = cameraModel.GetWidth() * cameraModel.GetHeight();

  chisel::Transform lastPose;
  lastPose = frame_ref.pose_sophus[0].matrix().cast<float>();
  if (frame_ref.refined_depth.empty()) {
    return;
  }
  float *depthImageData = (float *)frame_ref.refined_depth.data;
  unsigned char *colorImageData;
  if (frame_ref.rgb.empty()) {
    colorImageData = NULL;
  } else {
    colorImageData = new unsigned char[totalPixelNum * 4];
    for (int j = 0; j < totalPixelNum; j++) {
      colorImageData[j * 4 + 0] = frame_ref.rgb.at<unsigned char>(j * 3 + 0);
      colorImageData[j * 4 + 1] = frame_ref.rgb.at<unsigned char>(j * 3 + 1);
      colorImageData[j * 4 + 2] = frame_ref.rgb.at<unsigned char>(j * 3 + 2);
      colorImageData[j * 4 + 3] = 1;
    }
  }

  if (frame_ref.tracking_success && frame_ref.origin_index == 0) {
    chiselMap->IntegrateDepthScanColor(projectionIntegrator, depthImageData,
                                       colorImageData, lastPose, cameraModel);
  }
  free(colorImageData);
}

void MobileFusion::RetractObservations(chisel::ChunkManager &manager, Frame &kf,
                                       std::vector<int> &lookup) {
  std::cout << "Retracting observations..." << std::endl;
  int frame_id = kf.frame_index;
  int cnt = 0;
  for (int i = 0; i < kf.validChunks.size(); i++) {
    if (!manager.HasChunk(kf.validChunks[i])) continue;
    chisel::ChunkPtr chunk = manager.GetChunk(kf.validChunks[i]);
    chunk->observations.erase(frame_id);
    if (texManager->chunkGraph.chunks.find(kf.validChunks[i]) ==
        texManager->chunkGraph.chunks.end())
      continue;
    size_t chunkindex =
        texManager->chunkGraph.chunks.find(kf.validChunks[i])->second;
    size_t frameid = lookup[frame_id];
    texManager->dataCost.remove_observation(chunkindex, frameid);
    // texManager->labels[chunkindex] = 0;
    cnt++;
  }
  // if (cnt > 0) std::cout << "Retracting observations..." << std::endl;
}

int MobileFusion::tsdfFusion(
    std::vector<Frame> &frame_list, int CurrKeyframeIndex,
    const std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist,
    int integrateKeyframeID) {
  //    printf("begin refine keyframes %d
  //    %d\r\n",CurrKeyframeIndex,frame_list.size());
  Frame &frame_ref = frame_list[CurrKeyframeIndex];

  if (CurrKeyframeIndex == 0) return 0;

  //    IntegrateFrameChunks(frame_ref);
  int integrateKeyframeIndex = kflist[integrateKeyframeID].keyFrameIndex;
  if (frame_ref.is_keyframe) {
    // TICK("MobileFusion::TSDFFusion::GetMapDynamics");
    // get dynamic info
    std::vector<int> keyframesToUpdate;
    GetMapDynamics(frame_list, keyframesToUpdate, integrateKeyframeIndex);

    // create look up tables for kflist
    std::vector<int> frameIndexToKeyframeDB(CurrKeyframeIndex + 1, -1);
    for (int i = 0; i < kflist.size(); i++) {
      frameIndexToKeyframeDB[kflist[i].keyFrameIndex] = i;
    }
    // TOCK("MobileFusion::TSDFFusion::GetMapDynamics");

#if 1
    // TICK("MobileFusion::TSDFFusion::Reintegration::ReintegrateAll");
    for (int i = 0; i < keyframesToUpdate.size(); i++) {
      int frame_index = keyframesToUpdate[i];
      TICK("MobileFusion::TSDFFusion::Reintegration::Deintegration");
      RetractObservations(chiselMap->chunkManager, frame_list[frame_index],
                          frameIndexToKeyframeDB);
      ReIntegrateKeyframe(frame_list,
                          kflist[frameIndexToKeyframeDB[frame_index]], 0);
      ReIntegrateKeyframe(frame_list,
                          kflist[frameIndexToKeyframeDB[frame_index]], 1);
      TOCK("MobileFusion::TSDFFusion::Reintegration::Deintegration");
      cout << "finish reintegrate frame: " << frame_index << " time: "
           << Stopwatch::getInstance().getTiming(
                  "MobileFusion::TSDFFusion::Reintegration::Deintegration")
           << "ms" << endl;
    }
    // TOCK("MobileFusion::TSDFFusion::Reintegration::ReintegrateAll");
#endif
    // begin update

    if (integrateKeyframeID >= 0) {
      Frame &kf = frame_list[kflist[integrateKeyframeID].keyFrameIndex];
      //    OptimizeKeyframeVoxelDomain(frame_list,kflist[integrateKeyframeID]);
      if (kf.tracking_success && kf.origin_index == 0) {
        ReIntegrateKeyframe(frame_list, kflist[integrateKeyframeID], 1);
      }
    }
    chiselMap->UpdateMeshes(cameraModel);

#if 1
    if (integrateKeyframeIndex > 3) {
      for (auto meshit : chiselMap->chunkManager.GetAllMeshes()) {
        chisel::PatchPtr patch = meshit.second->m_patch;
        if (patch != nullptr && patch->wrong_mapping) {
          size_t chunkindex =
              texManager->chunkGraph.chunks.find(meshit.second->chunkID)
                  ->second;
          size_t frameid = frameIndexToKeyframeDB[patch->frameid];
          texManager->dataCost.remove_observation(chunkindex, frameid);
          // texManager->labels[chunkindex] = 0;
        }
      }
    }
#endif

    chunksToUpdate.clear();
    const chisel::MeshMap &allMeshes = chiselMap->chunkManager.GetAllMeshes();
    for (auto it : chiselMap->meshesToUpdate) {
      if (!it.second) continue;
      if (allMeshes.find(it.first) == allMeshes.end()) {
        continue;
      }
      chunksToUpdate.emplace_back(it.first);
    }

    chiselMap->CompressMeshes(chiselMap->meshesToUpdate);
    texManager->update_chunkgraph(chunksToUpdate, chiselMap->chunkManager);
    texManager->update_datacost(chunksToUpdate, chiselMap->chunkManager,
                                frameIndexToKeyframeDB, integrateKeyframeIndex,
                                keyframesToUpdate);
    if (!keyframesToUpdate.empty())
      texManager->check_graph(chiselMap->chunkManager);
    if (texManager->chunkGraph.num_nodes() > 0) {
      try {
        texManager->view_selection(kflist);
      } catch (std::runtime_error &e) {
        std::cout << "WARNING: unexpected error when solving MRF!" << std::endl;
        return -1;
      }
    }
    chunk_log.emplace_back(kflist[integrateKeyframeID].keyFrameIndex);
    chunk_complex.emplace_back(Stopwatch::getInstance().getTiming(
        "MobileFusion::TSDFFusion::MRFSolver::Solve"));
    chunk_scale.emplace_back(texManager->chunkGraph.num_nodes());
    int eot = chiselMap->GeneratePatches(chunksToUpdate, texManager->chunkGraph,
                                         frame_list, cameraModel);
    if (eot < 0) {
      std::cout << "WARNING: texture buffer is full!" << std::endl;
      return -1;
    }
    chiselMap->CompensateColor();

    chiselMap->UpdateAtlas(chunksToUpdate);

    chiselMap->DrawMeshes(tsdf_vertices_buffer, tsdf_indices_buffer,
                          tsdf_indice_num, tsdf_vertice_num);
    /*
    integration_time += Stopwatch::getInstance().getTiming(
        "MobileFusion::TSDFFusion::IntegrateKeyFrame");
    meshing_time += Stopwatch::getInstance().getTiming(
        "MobileFusion::TSDFFusion::MESHING::UpdateMeshes");
    mrf_time += Stopwatch::getInstance().getTiming(
        "MobileFusion::TSDFFusion::MRFSolver::Solve");
    adjustment_time += Stopwatch::getInstance().getTiming(
        "MobileFusion::TSDFFusion::Meshing::CompensateColor");
    rendering_time += Stopwatch::getInstance().getTiming(
                          "MobileFusion::TSDFFusion::Meshing::Shading") +
                      Stopwatch::getInstance().getTiming(
                          "MobileFusion::TSDFFusion::Meshing::UpdateAtlas") +
                      Stopwatch::getInstance().getTiming(
                          "MobileFusion::TSDFFusion::Meshing::GeneratePatches");
    */
    vertex_data_updated = 1;
  }

  return 1;
}

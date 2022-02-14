#ifndef MOBILEFUSION_H
#define MOBILEFUSION_H

#include <sys/stat.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "../GCSLAM/GCSLAM.h"
#include "../GCSLAM/MultiViewGeometry.h"
#include "../GCSLAM/frame.h"
#include "MapMaintain.hpp"
#include "Structure/TexMap.h"

#include "../Shaders/Shaders.h"

#include <GL/freeglut.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/pangolin.h>

#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/utils/ProjectionIntegrator.h>
#include <open_chisel/weighting/ConstantWeighter.h>
#include <boost/filesystem.hpp>
#include <boost/thread/barrier.hpp>
#include "Structure/Chisel.h"

// 30 M * 12 * 4 = 1.5G
#define GLOBLA_MODLE_VERTEX_NUM (1024 * 1024 * 30)
#define VERTEX_WEIGHT_THRESHOLD 3

typedef float DepthData;
typedef uint8_t ColorData;

typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >
    Mat4List;
typedef std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >
    Mat3List;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >
    Vec3List;

#define INTEGRATE_ALL 1

struct VertexElement {
  float loc[4];
  float color[4];
  float normal[4];
};

class MobileFusion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GLuint vbo;
  GLuint vbo_point_cloud;
  GLuint vbo_data_transfer;
  GLuint ibo;
  GLuint pbo;

  std::shared_ptr<Shader> drawProgram;
  std::shared_ptr<Shader> drawPhongLighting;
  std::shared_ptr<Shader> drawVoxelHashingStyle;

  std::vector<unsigned int> chunk_log;
  std::vector<unsigned int> chunk_scale;
  std::vector<float> chunk_complex;

  float *tsdf_vertices_buffer;        // vertex buffer
  unsigned int *tsdf_indices_buffer;  // index buffer
  unsigned int tsdf_vertice_num;      // indicate the number of valid vertices
  unsigned int tsdf_indice_num;
  int vertex_data_updated;  // update vertex data;

  cv::Mat texture_mask;
  cv::Mat texture_buffer;  // texture buffer
  int texture_buffer_width;
  int texture_buffer_height;
  int keyframe_height;
  GLuint texture_model;

  chisel::ChunkIDList chunksToUpdate;
  chisel::ChunkIDList chunksAtBorder;

  chisel::ChiselPtr chiselMap;
  std::shared_ptr<chisel::DepthImage<DepthData> > lastDepthImage;
  std::shared_ptr<chisel::ColorImage<ColorData> > lastColorImage;
  chisel::ProjectionIntegrator projectionIntegrator;
  chisel::PinholeCamera cameraModel;

  boost::barrier render_barr;

  double integration_time;
  double meshing_time;
  double mrf_time;
  double adjustment_time;
  double rendering_time;

  int validFrameNum;
  int fuseKeyframeId;
  bool is_stopped;

  GCSLAM gcSLAM;
  std::unique_ptr<TexMap> texManager;

  MobileFusion() : render_barr(2) {
    int argc = 1;
    char ProjectName[256] = "MobileFusion";
    char *argv = ProjectName;
    glutInit(&argc, &argv);
    glutInitDisplayMode(GLUT_SINGLE);
    GLenum err = glewInit();
    if (err != GLEW_OK) {
      // Problem: glewInit failed, something is seriously wrong.
      std::cout << "glewInit failed: " << glewGetErrorString(err) << std::endl;
      exit(1);
    }

    tsdf_vertice_num = 0;
    tsdf_indice_num = 0;
    validFrameNum = 0;
    fuseKeyframeId = 0;
    is_stopped = false;

    // allocate memory for vertices&indices buffers
    tsdf_vertices_buffer = new float[GLOBLA_MODLE_VERTEX_NUM *
                                     sizeof(VertexElement) / sizeof(float)];
    memset(tsdf_vertices_buffer, 0,
           GLOBLA_MODLE_VERTEX_NUM * sizeof(VertexElement));
    tsdf_indices_buffer = new unsigned int[GLOBLA_MODLE_VERTEX_NUM];
    memset(tsdf_indices_buffer, 0,
           GLOBLA_MODLE_VERTEX_NUM * sizeof(unsigned int));
    vertex_data_updated = 0;

    // allocate memory for texture whose size is MAX_PATCH_WIDTH*
    // MAX_PATCH_HEIGHT
    glGenTextures(1, &texture_model);
    glBindTexture(GL_TEXTURE_2D, texture_model);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, MAX_PATCH_WIDTH, MAX_PATCH_HEIGHT, 0,
                 GL_RGB, GL_UNSIGNED_BYTE, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenBuffersARB(1, &pbo);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 GLOBLA_MODLE_VERTEX_NUM * sizeof(VertexElement),
                 &tsdf_vertices_buffer[0], GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 GLOBLA_MODLE_VERTEX_NUM * sizeof(unsigned int),
                 &tsdf_indices_buffer[0], GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glGenBuffers(1, &vbo_data_transfer);
    glGenBuffers(1, &vbo_point_cloud);

    drawVoxelHashingStyle =
        loadProgramFromFile("draw_mesh.vert", "draw_mesh.frag");
    texManager = std::make_unique<TexMap>();

    integration_time = 0;
    meshing_time = 0;
    mrf_time = 0;
    adjustment_time = 0;
    rendering_time = 0;
  }

  // clear redudent memory stored in frames, including depth, normal, color,
  // features.
  void clearRedudentFrameMemory(int integrateLocalFrameNum);
  void updateGlobalMap(int inputValidFrameNum, int inputFuseKeyframeId);
  void MapManagement();
  void ReIntegrateKeyframe(
      std::vector<Frame> &frame_list,
      const MultiViewGeometry::KeyFrameDatabase &kfDatabase,
      const int integrateFlag);
  void IntegrateFrame(const Frame &frame_ref);
  int tsdfFusion(std::vector<Frame> &frame_list, int CorrKeyframeIndex,
                 const std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist,
                 int integrateKeyframeID);
  void RetractObservations(chisel::ChunkManager &manager, Frame &kf,
                           std::vector<int> &lookup);

  void initGCSLAM(const int maximum_frame_num,
                  const MultiViewGeometry::GlobalParameters para,
                  const MultiViewGeometry::CameraPara &camera) {
    gcSLAM.init(maximum_frame_num, camera);
    gcSLAM.SetMinimumDisparity(para.minimum_disparity);
    gcSLAM.SetSalientScoreThreshold(para.salient_score_threshold);
    gcSLAM.SetMaxCandidateNum(para.maximum_keyframe_match_num);
  }

  void initChiselMap(const MultiViewGeometry::CameraPara &camera,
                     float ipnutVoxelResolution, float farPlaneDist = 3) {
    float fx = camera.c_fx;
    float fy = camera.c_fy;
    float cx = camera.c_cx;
    float cy = camera.c_cy;
    int width = camera.width;
    int height = camera.height;

#if 1
    float truncationDistConst = 0.001504;
    float truncationDistLinear = 0.00152;
    float truncationDistQuad = 0.0019;
    float truncationDistScale = 6.0;
#else
    float truncationDistConst = 0.01;
    float truncationDistLinear = 0.01;
    float truncationDistQuad = 0.01;
    float truncationDistScale = 1.0;
#endif
    float weight = 1;
    bool useCarving = true;
    float carvingDist = 0.05;
    float nearPlaneDist = 0.01;

    std::cout << "far plane dist: " << farPlaneDist << std::endl;
    chunkSizeX = 8;
    chunkSizeY = 8;
    chunkSizeZ = 8;
    voxelResolution = ipnutVoxelResolution;
    useColor = true;

    chisel::Vec4 truncation(truncationDistQuad, truncationDistLinear,
                            truncationDistConst, truncationDistScale);
    chiselMap = chisel::ChiselPtr(
        new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ),
                           voxelResolution, useColor));

    projectionIntegrator.SetCentroids(
        chiselMap->GetChunkManager().GetCentroids());
    projectionIntegrator.SetTruncator(
        chisel::TruncatorPtr(new chisel::QuadraticTruncator(
            truncation(0), truncation(1), truncation(2), truncation(3))));
    projectionIntegrator.SetWeighter(
        chisel::WeighterPtr(new chisel::ConstantWeighter(weight)));
    projectionIntegrator.SetCarvingDist(carvingDist);
    projectionIntegrator.SetCarvingEnabled(useCarving);

    cameraModel.SetIntrinsics(fx, fy, cx, cy);
    cameraModel.SetNearPlane(nearPlaneDist);
    cameraModel.SetFarPlane(farPlaneDist);
    cameraModel.SetWidth(width);
    cameraModel.SetHeight(height);
  }

  ~MobileFusion() {
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ibo);
    glDeleteBuffers(1, &pbo);
    glDeleteBuffers(1, &vbo_data_transfer);
    glDeleteBuffers(1, &vbo_point_cloud);
    glDeleteTextures(1, &texture_model);
    delete tsdf_vertices_buffer;
    delete tsdf_indices_buffer;
  }

  void DrawCube(float *vertex_list, GLint *index_list) {
    int i, j;

    glBegin(GL_LINES);
    for (i = 0; i < 12; ++i)  // 12 条线段

    {
      for (j = 0; j < 2; ++j)  // 每条线段 2个顶点

      {
        //                Eigen::Vector4f vertex(vertex_list[index_list[i*2 + j]
        //                * 3],
        //                        vertex_list[index_list[i*2 + j] * 3 + 1],
        //                        vertex_list[index_list[i*2 + j] * 3 + 2],
        //                        1);
        //                vertex = t * vertex;
        glVertex3fv(&vertex_list[index_list[i * 2 + j] * 3]);
      }
    }
    glEnd();
  }

  void GetColor(double v, double vmin, double vmax, int &r, int &g, int &b) {
    double dv;

    if (v < vmin) v = vmin;
    if (v > vmax) v = vmax;
    dv = vmax - vmin;

    r = 0;
    g = 0;
    b = 0;
    if (v < (vmin + 0.25 * dv)) {
      r = 0;
      g = (4 * (v - vmin) / dv) * 255;
    } else if (v < (vmin + 0.5 * dv)) {
      r = 0;
      b = (1 + 4 * (vmin + 0.25 * dv - v) / dv) * 255;
    } else if (v < (vmin + 0.75 * dv)) {
      r = (4 * (v - vmin - 0.5 * dv) / dv) * 255;
      b = 0;
    } else {
      g = (1 + 4 * (vmin + 0.75 * dv - v) / dv) * 255;
      b = 0;
    }
  }

  inline int MobileShow(pangolin::OpenGlMatrix mvp, const float threshold,
                        const bool drawUnstable, const bool drawNormals,
                        const bool drawColors, const bool drawPoints,
                        const bool drawOrigin, const int time,
                        const int timeDelta, std::vector<Frame> &frame_list) {
    std::shared_ptr<Shader> program = drawVoxelHashingStyle;
    program->Bind();  // set this program as current program
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    program->setUniform(Uniform("MVP", mvp));
    program->setUniform(Uniform("pose", pose));
    program->setUniform(Uniform("threshold", threshold));
    program->setUniform(Uniform(
        "colorType",
        (drawNormals ? 1
                     : drawColors ? 2 : drawPoints ? 3 : drawOrigin ? 4 : 0)));
    program->setUniform(Uniform("unstable", drawUnstable));
    program->setUniform(Uniform("time", time));
    program->setUniform(Uniform("timeDelta", timeDelta));

    if (program == drawPhongLighting) {
      program->setUniform(Uniform("view_matrix", pose));
      program->setUniform(Uniform("proj_matrix", mvp));
      Eigen::Vector3f Lightla(0.2f, 0.2f, 0.2f);
      Eigen::Vector3f Lightld(1.0f, 1.0f, 1.0f);
      Eigen::Vector3f Lightls(1.0f, 1.0f, 1.0f);
      Eigen::Vector3f Lightldir(0.0, 0.0, 1.0f);
      Eigen::Vector3f fma(0.26f, 0.26f, 0.26f);
      Eigen::Vector3f fmd(0.35f, 0.35f, 0.35f);
      Eigen::Vector3f fms(0.30f, 0.30f, 0.30f);
      float fss = 16.0f;
      Eigen::Vector3f bma(0.85f, 0.85f, 0.85f);
      Eigen::Vector3f bmd(0.85f, 0.85f, 0.85f);
      Eigen::Vector3f bms(0.60f, 0.60f, 0.60f);
      float bss = 16.0f;

      Eigen::Matrix4f user_view_matrix = Eigen::Matrix4f::Identity();
      Eigen::Matrix4f user_light_matrix = Eigen::Matrix4f::Identity();
      Eigen::Vector4f user_rot_center = Eigen::Vector4f(0, 0, 0, 1);
      program->setUniform(Uniform("Lightla", Lightla));
      program->setUniform(Uniform("Lightld", Lightld));
      program->setUniform(Uniform("Lightls", Lightls));
      program->setUniform(Uniform("Lightldir", Lightldir));
      program->setUniform(Uniform("fma", fma));
      program->setUniform(Uniform("fmd", fmd));
      program->setUniform(Uniform("fms", fms));
      program->setUniform(Uniform("bma", bma));
      program->setUniform(Uniform("bmd", bmd));
      program->setUniform(Uniform("bms", bms));
      program->setUniform(Uniform("bss", bss));
      program->setUniform(Uniform("fss", fss));
      program->setUniform(Uniform("user_view_matrix", user_view_matrix));
      program->setUniform(Uniform("user_light_matrix", user_light_matrix));
      program->setUniform(Uniform("user_rot_center", user_rot_center));
    }

    if (program == drawVoxelHashingStyle) {
      float s_materialShininess = 16.0f;
      Eigen::Vector4f s_materialAmbient =
          Eigen::Vector4f(0.75f, 0.65f, 0.5f, 1.0f);
      Eigen::Vector4f s_materialDiffuse =
          Eigen::Vector4f(1.0f, 0.9f, 0.7f, 1.0f);
      Eigen::Vector4f s_materialSpecular =
          Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
      Eigen::Vector4f s_lightAmbient = Eigen::Vector4f(0.4f, 0.4f, 0.4f, 1.0f);
      Eigen::Vector4f s_lightDiffuse =
          Eigen::Vector4f(0.6f, 0.52944f, 0.4566f, 0.6f);
      Eigen::Vector4f s_lightSpecular = Eigen::Vector4f(0.3f, 0.3f, 0.3f, 1.0f);
      Eigen::Vector3f lightDir = Eigen::Vector3f(0.0f, -1.0f, 2.0f);

      program->setUniform(Uniform("materialShininess", s_materialShininess));
      program->setUniform(Uniform("materialAmbient", s_materialAmbient));
      program->setUniform(Uniform("materialDiffuse", s_materialDiffuse));
      program->setUniform(Uniform("materialSpecular", s_materialSpecular));
      program->setUniform(Uniform("lightAmbient", s_lightAmbient));
      program->setUniform(Uniform("lightDiffuse", s_lightDiffuse));
      program->setUniform(Uniform("lightSpecular", s_lightSpecular));
      program->setUniform(Uniform("lightDir", lightDir));
      program->setUniform(Uniform("ftex", 0));
    }
    // This is for the point shader
    // setup a uniform:

    //    GLuint loc = glGetUniformLocation(program->programId(),
    //    "camera_array"); glUniformMatrix4fv(loc, 20, false,
    //    camera_array_matrices, 0);

    if (vertex_data_updated) {
      // update texture
      if (chiselMap->atlas.hot_end > chiselMap->atlas.hot_start) {
        glBindBufferARB(GL_PIXEL_UNPACK_BUFFER, pbo);
        glBufferDataARB(
            GL_PIXEL_UNPACK_BUFFER,
            (chiselMap->atlas.hot_end - chiselMap->atlas.hot_start) * 3,
            &chiselMap->atlas.texture_buffer
                 .data[chiselMap->atlas.hot_start * 3],
            GL_STREAM_COPY_ARB);

        glBindTexture(GL_TEXTURE_2D, texture_model);
        glTexSubImage2D(
            GL_TEXTURE_2D, 0, 0, chiselMap->atlas.hot_start / MAX_PATCH_WIDTH,
            MAX_PATCH_WIDTH,
            (chiselMap->atlas.hot_end - chiselMap->atlas.hot_start) /
                MAX_PATCH_WIDTH,
            GL_RGB, GL_UNSIGNED_BYTE, 0);
        //   reinterpret_cast<GLvoid*>(chiselMap->atlas.hot_start*3));
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindBufferARB(GL_PIXEL_UNPACK_BUFFER, 0);
        // may be further improved by using two pbo? or handle pointer to
        // another thread?
      }
      // update data
      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      //          glBufferData(GL_ARRAY_BUFFER, (tsdf_vertice_num + 3) *
      //          sizeof(VertexElement), &tsdf_vertices_buffer[0],
      //          GL_STREAM_DRAW);
      glBufferSubData(GL_ARRAY_BUFFER, 0,
                      tsdf_vertice_num * sizeof(VertexElement),
                      &tsdf_vertices_buffer[0]);
      glBindBuffer(GL_ARRAY_BUFFER, 0);

      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
      glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
                      tsdf_indice_num * sizeof(unsigned int),
                      &tsdf_indices_buffer[0]);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

      vertex_data_updated = 0;
      // begin draw signed distance fields
    }

    glBindTexture(GL_TEXTURE_2D, texture_model);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(VertexElement), 0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1, 4, GL_FLOAT, GL_FALSE, sizeof(VertexElement),
        reinterpret_cast<GLvoid *>(sizeof(Eigen::Vector4f) * 1));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(
        2, 4, GL_FLOAT, GL_FALSE, sizeof(VertexElement),
        reinterpret_cast<GLvoid *>(sizeof(Eigen::Vector4f) * 2));

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

    // glDrawArrays(GL_TRIANGLES,0,tsdf_vertice_num);

    glDrawElements(GL_TRIANGLES, tsdf_indice_num, GL_UNSIGNED_INT, 0);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    program->Unbind();
#if 0
        float vertex_list[24] =
        {
            -0.5f, -0.5f, -0.5f,
            0.5f, -0.5f, -0.5f,
            -0.5f, 0.5f, -0.5f,
            0.5f, 0.5f, -0.5f,
            -0.5f, -0.5f, 0.5f,
            0.5f, -0.5f, 0.5f,
            -0.5f, 0.5f, 0.5f,
            0.5f, 0.5f, 0.5f,
        };
         GLint index_list[24] =
        {
            0, 1,
            2, 3,
            4, 5,
            6, 7,
            0, 2,
            1, 3,
            4, 6,
            5, 7,
            0, 4,
            1, 5,
            7, 3,
            2, 6
        };
        DrawCube(searchArea,index_list);
        std::vector<float> corners = chiselMap->candidateCubes;
        int cornersNum = corners.size() / 24;
        float *corner_pointer = corners.data();
        for(int i = 0; i < cornersNum; i++)
        {
            DrawCube(&corner_pointer[i*24],index_list);
        }
#endif
    return 0;
  }

  float GetVoxelResolution() { return voxelResolution; }

  float searchArea[24];

 private:
  int chunkSizeX;
  int chunkSizeY;
  int chunkSizeZ;
  float voxelResolution;
  bool useColor;
};

#endif  // MOBILEFUSION_H

#pragma once
#include "MeshViewer.h"
#include <vector>
#include <climits>

#define _USE_MATH_DEFINES

using namespace std;
typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;

struct PossibleVertex
{
    OpenMesh::Vec2f vparam_uv_;
    Mesh::Point vpos_;
    Mesh::Normal vnorm_;
};
class SphereParametrization
{
public:
    SphereParametrization(const char *_meshfilename, SoSeparator *_root, int diskParamMode, bool &successfulProcess);
    SphereParametrization(const char *_meshfilename, SoSeparator *_root, bool &successfulProcess);
    ~SphereParametrization();
    bool open_mesh(const char *_meshfilename);

private:

    void find_bounding_sphere();
    void transform_to_unit_sphere();
    void set_point_to_center_of_one_ring_vertices();
    bool find_interior_point();

    bool lineIntersection(float x11, float x12, float y11, float y12, 
                            float x21, float x22, float y21, float y22, 
                            float& intersection_X, float& intersection_Y);
    SoSeparator *getShapeSep();
    void split_mesh(float zCut, float zInsert, bool marginalTrisInLeft, bool marginalTrisInRight);
    bool process_link(float zCut, vector<Mesh::VertexHandle> &leftInd, vector<Mesh::VertexHandle> &rightInd,
                      Mesh::VertexHandle v1, Mesh::VertexHandle v2, Mesh::VertexHandle &mid);
    PossibleVertex getPossibleVertex(Mesh::VertexHandle vleft, Mesh::VertexHandle vright, float scale);
    vector<PossibleVertex>      chopLink(Mesh::VertexHandle left, Mesh::VertexHandle right, int numNewPoints);
    void                        insertPoints(vector<PossibleVertex> &shape, PossibleVertex left, PossibleVertex right, vector<PossibleVertex> const &source);
    void                        addEdgeLinks(vector<PossibleVertex> const &shape, set<pair<PossibleVertex, PossibleVertex>> &links, float zCut);
    int compareZ(int index, float zCut);
    int compareZ(PossibleVertex vert, float zCut);

private:
    Mesh mesh_;
    SoSeparator *root;

    bool youWantToPaintEachVertexDifferently;
    bool isCutProcess;

    OpenMesh::VPropHandleT<OpenMesh::Vec2f> vparam_uv_;
    OpenMesh::VPropHandleT<OpenMesh::Vec3f> vparam_sph_;

    OpenMesh::VPropHandleT<Mesh::Point> vpos_;
    OpenMesh::FPropHandleT<Mesh::Point> fcent_;

    int ninterior;
    int nboundary;
    Mesh::Point _bbMin3D, _bbMax3D, _bbMin2D, _bbMax2D;
    Mesh::Point sphereCenterPoint;
    double sphereRad;
};

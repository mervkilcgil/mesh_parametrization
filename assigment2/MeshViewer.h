#ifndef MESH_VIEWER_WIDGET_HH
#define MESH_VIEWER_WIDGET_HH

#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>

#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoTextureCoordinate2.h>
#include <Inventor/nodes/SoSceneTexture2.h>
#include <Inventor/nodes/SoTranslation.h>

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMeshT.hh>
#include <OpenMesh/Core/Utils/BaseProperty.hh>
#include <OpenMesh\Core\IO\MeshIO.hh>

#ifndef PI
#define PI 3.141592653589793238462643383279
#endif

using namespace std;

class MeshViewer
{
public:
    MeshViewer(int _drawMode, int _taskNumber);

    virtual bool open_mesh(const char *_filename);

    void update_face_indices();

    void generateSphere(SoSeparator *root);

    void setPaintEachVertexDifferently() { youWantToPaintEachVertexDifferently = true; }

    SoSceneTexture2* getTexturedScene(){ return texturedScene; }
    SoSeparator *getShapeSep();

protected:
    typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;
    typedef Mesh::Point point3d;

    point3d MidPoint(point3d p1, point3d p2);
    void NormalizeVector(point3d &p);
    int CreateUnitSphere(int iterations);
    void get_one_ring_vertices(int vert_i, vector<int> &neighb_verts, vector<int> &neighb_vedges);
    float distance(point3d a, point3d b);
    float tan2(point3d a, point3d b);

protected:
    Mesh mesh_;
    std::vector<unsigned int> indices_;
    string fileName;

    int ninterior;
    int nboundary;
    int drawMode;
    int taskNumber;
    bool youWantToPaintEachVertexDifferently;

    OpenMesh::VPropHandleT<OpenMesh::Vec2f> vparam_uv_;

    OpenMesh::VPropHandleT<Mesh::Point> vpos_;

    float *_TextureCoordinates_u, *_TextureCoordinates_h;
    SoSceneTexture2* texturedScene = new SoSceneTexture2();
};

#endif // MESH_VIEWER_WIDGET_HH

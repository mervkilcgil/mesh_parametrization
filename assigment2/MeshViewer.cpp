#include "MeshViewer.h"
#include <fstream>
#include <iostream>
#include <ostream>

MeshViewer::MeshViewer(int _drawMode, int _taskNumber)
{
    ninterior = 0;
    nboundary = 0;
    taskNumber = _taskNumber;
    drawMode = _drawMode;
    youWantToPaintEachVertexDifferently = false;
    fileName = ".off";
}

//-----------------------------------------------------------------------------

bool MeshViewer::open_mesh(const char *_filename)
{
    // load mesh
    fileName = string(_filename);
    if (OpenMesh::IO::read_mesh(mesh_, _filename))
    {
        mesh_.request_face_normals();
        mesh_.request_vertex_normals();
        mesh_.request_vertex_status();
        mesh_.request_vertex_texcoords2D();
        mesh_.request_edge_status();
        mesh_.request_edge_colors();
        mesh_.request_face_colors();
        mesh_.request_face_status();
        mesh_.request_face_texture_index();
        mesh_.request_halfedge_status();
        mesh_.request_halfedge_normals();
        mesh_.request_halfedge_texcoords1D();
        mesh_.request_halfedge_texcoords2D();
        mesh_.request_halfedge_texcoords3D();
        mesh_.request_vertex_colors();
        mesh_.request_vertex_texcoords1D();
        mesh_.request_vertex_texcoords3D();

        mesh_.add_property(vpos_);
        mesh_.add_property(vparam_uv_);
        // set center and radius
        Mesh::ConstVertexIter v_it(mesh_.vertices_begin()),
            v_end(mesh_.vertices_end());
        Mesh::Point bbMin, bbMax;

        bbMin = bbMax = mesh_.point(*v_it);
        for (; v_it != v_end; ++v_it)
        {
            bbMin.minimize(mesh_.point(*v_it));
            bbMax.maximize(mesh_.point(*v_it));
        }

        // compute face & vertex normals
        mesh_.update_normals();
        // update face indices for faster rendering
        update_face_indices();

        Mesh::VertexIter viter;
        for (viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
        {
            if (mesh_.is_boundary(*viter))
                ++nboundary;
            else
                ++ninterior;
        }
        return true;
    }

    return false;
}

//-----------------------------------------------------------------------------

SoSeparator *MeshViewer::getShapeSep()
{
    SoSeparator *res = new SoSeparator();

    SoMaterial *mat = new SoMaterial();
    mat->diffuseColor.setValue(0, 1, 0);
    mat->transparency.setValue(0.5f);
    Mesh::VertexIter v_it_c = mesh_.vertices_begin(), v_endc(mesh_.vertices_end());

    if (mesh_.has_vertex_colors() && youWantToPaintEachVertexDifferently)
    {
        for (v_it_c = mesh_.vertices_begin(); v_it_c != v_endc; ++v_it_c)
        {
            int idx = (*v_it_c).idx();
            SbVec3f vec(mesh_.color(*v_it_c)[0], mesh_.color(*v_it_c)[1], mesh_.color(*v_it_c)[2]);
            mat->diffuseColor.set1Value(idx, vec);
        }
    }
    res->addChild(mat);

    SoShapeHints *hints = new SoShapeHints;
    hints->creaseAngle = 3.14;
    res->addChild(hints);

    if (youWantToPaintEachVertexDifferently)
    {
        SoMaterialBinding *materialBinding = new SoMaterialBinding;
        materialBinding->value = SoMaterialBinding::PER_VERTEX_INDEXED;
        res->addChild(materialBinding);
    }

    //shape
    SoCoordinate3 *coords = new SoCoordinate3();
    SoTextureCoordinate2* textcoords = new SoTextureCoordinate2();

    for (Mesh::VertexIter v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it)
    {
        coords->point.set1Value((*v_it).idx(), mesh_.property(vpos_, *v_it)[0], mesh_.property(vpos_, *v_it)[1], mesh_.property(vpos_, *v_it)[2]);
        if (taskNumber != 2)
          textcoords->point.set1Value((*v_it).idx(), mesh_.property(vparam_uv_, *v_it)[0], mesh_.property(vparam_uv_, *v_it)[1]);
    }

    SoIndexedFaceSet *faceSet = new SoIndexedFaceSet();

    for (Mesh::FaceIter f_it = mesh_.faces_begin(); f_it != mesh_.faces_end(); ++f_it)
    {
        int c2 = 0;

        for (Mesh::FaceVertexIter fvit = mesh_.fv_iter(*f_it); fvit.is_valid(); ++fvit)
        {
          int indx = (*f_it).idx() * 4 + c2;
          faceSet->coordIndex.set1Value(indx, (*fvit).idx());
            faceSet->textureCoordIndex.set1Value(indx, (*fvit).idx());
            /* if (taskNumber != 2)
                faceSet->materialIndex.set1Value(indx, (*fvit).idx()); */
            ++c2;
        }

        faceSet->coordIndex.set1Value((*f_it).idx() * 4 + c2, -1);
        faceSet->textureCoordIndex.set1Value((*f_it).idx() * 4 + c2, -1);
    }

    res->addChild(coords);
    res->addChild(textcoords);
    texturedScene->ref();
    texturedScene->scene.setValue(textcoords);
    res->addChild(faceSet);
    res->addChild(texturedScene);

    return res;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void MeshViewer::update_face_indices()
{
    Mesh::ConstFaceIter f_it,
        f_end(mesh_.faces_end());
    Mesh::ConstFaceVertexIter fv_it;

    indices_.clear();
    indices_.reserve(mesh_.n_faces() * 3);
    std::cout << "mesh indices updated" << std::endl;

    for (f_it = mesh_.faces_begin(); f_it != f_end; ++f_it)
    {
        fv_it = mesh_.cfv_iter(*f_it);
        indices_.push_back((*fv_it).idx());
        indices_.push_back((*(++fv_it)).idx());
        indices_.push_back((*(++fv_it)).idx());
    }
}
//-----------------------------------------------------------------------------

void MeshViewer::generateSphere(SoSeparator *root)
{
    int n = 7;
    n = CreateUnitSphere(n);
    root->addChild(getShapeSep());
    OpenMesh::IO::write_mesh(mesh_, "spheregeneration.off");
}

//-----------------------------------------------------------------------------

int MeshViewer::CreateUnitSphere(int iterations)
{
    int n, nstart;
    point3d p1(1.0, 1.0, 1.0), p2(-1.0, -1.0, 1.0);
    point3d p3(1.0, -1.0, -1.0), p4(-1.0, 1.0, -1.0);
    NormalizeVector(p1);
    NormalizeVector(p2);
    NormalizeVector(p3);
    NormalizeVector(p4);

    vector<vector<point3d>> facesets = vector<vector<point3d>>(4);
    vector<point3d> f1;
    f1.push_back(p1);
    f1.push_back(p2);
    f1.push_back(p3);
    facesets[0] = f1;
    vector<point3d> f2;
    f2.push_back(p2);
    f2.push_back(p1);
    f2.push_back(p4);
    facesets[1] = f2;
    vector<point3d> f3;
    f3.push_back(p2);
    f3.push_back(p4);
    f3.push_back(p3);
    facesets[2] = f3;
    vector<point3d> f4;
    f4.push_back(p1);
    f4.push_back(p3);
    f4.push_back(p4);
    facesets[3] = f4;

    n = 4;

    for (int i = 1; i < iterations; i++)
    {
        nstart = n;

        for (int j = 0; j < nstart; ++j)
        {
            facesets.resize(n + 3);
            facesets[n] = facesets[j];
            facesets[n + 1] = facesets[j];
            facesets[n + 2] = facesets[j];

            point3d p1n = MidPoint(facesets[j][0], facesets[j][1]);
            point3d p2n = MidPoint(facesets[j][1], facesets[j][2]);
            point3d p3n = MidPoint(facesets[j][2], facesets[j][0]);

            facesets[j][1] = p1n;
            facesets[j][2] = p3n;

            facesets[n][0] = p1n;
            facesets[n][2] = p2n;

            facesets[n + 1][0] = p3n;
            facesets[n + 1][1] = p2n;

            facesets[n + 2][0] = p1n;
            facesets[n + 2][1] = p2n;
            facesets[n + 2][2] = p3n;
            n += 3;
        }
    }

    for (int j = 0; j < n; j++)
    {
        NormalizeVector(facesets[j][0]);
        NormalizeVector(facesets[j][1]);
        NormalizeVector(facesets[j][2]);
    }

    for (int i = 1; i < facesets.size(); i++)
    {
        Mesh::VertexHandle v1 = mesh_.add_vertex(facesets[i][0]);
        Mesh::VertexHandle v2 = mesh_.add_vertex(facesets[i][1]);
        Mesh::VertexHandle v3 = mesh_.add_vertex(facesets[i][2]);
        mesh_.add_face(v1, v2, v3);
    }

    return n;
}

//-----------------------------------------------------------------------------

MeshViewer::point3d MeshViewer::MidPoint(point3d p1, point3d p2)
{
    point3d p((p1.data()[0] + p2.data()[0]) / 2, (p1.data()[1] + p2.data()[1]) / 2, (p1.data()[2] + p2.data()[2]) / 2);
    return p;
}

//-----------------------------------------------------------------------------

void MeshViewer::NormalizeVector(point3d &p)
{
    double length;
    length = sqrt(p.data()[0] * p.data()[0] + p.data()[1] * p.data()[1] + p.data()[2] * p.data()[2]);
    if (length != 0)
    {
        p.data()[0] = p.data()[0] / length;
        p.data()[1] = p.data()[1] / length;
        p.data()[2] = p.data()[2] / length;
    }
    else
    {
        p.data()[0] = 0;
        p.data()[1] = 0;
        p.data()[2] = 0;
    }
}

//-----------------------------------------------------------------------------

void MeshViewer::get_one_ring_vertices(int vert_i, vector<int> &neighb_verts, vector<int> &neighb_vedges)
{
    Mesh::VertexHandle vert = mesh_.vertex_handle((unsigned int)vert_i);
    Mesh::VertexOHalfedgeIter voh_it = mesh_.voh_begin(vert);
    while (voh_it != mesh_.voh_end(vert))
    {
        neighb_verts.push_back(mesh_.to_vertex_handle(*voh_it).idx());
        neighb_vedges.push_back(mesh_.s_edge_handle(*voh_it).idx());
        ++voh_it;
    }
}

//-----------------------------------------------------------------------------

float MeshViewer::distance(point3d a, point3d b)
{
    point3d p = (a - b);
    float dist = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    return dist;
}

//-----------------------------------------------------------------------------

float MeshViewer::tan2(point3d a, point3d b)
{
    float modusa = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float modusb = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    float d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (modusa * modusb);
    float angle = std::acos(d);
    return tan(angle / 2.0);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//=============================================================================

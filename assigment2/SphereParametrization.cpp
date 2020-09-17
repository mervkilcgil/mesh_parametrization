#include "SphereParametrization.h"
#include "HarmonicMapViewer.h"

SphereParametrization::SphereParametrization(const char *_meshfilename, SoSeparator *_root, int diskParamMode, bool &successfulProcess)
{
    root = _root;
    isCutProcess = true;
    successfulProcess = open_mesh(_meshfilename);
    //split_mesh(zCut, zInsert, true, false);
    HarmonicMapViewer hmwindow(mesh_, 0, diskParamMode, 1);

    root = getShapeSep();
}

SphereParametrization::SphereParametrization(const char *_meshfilename, SoSeparator *_root, bool &successfulProcess)
{
    root = _root;
    isCutProcess = false;
    successfulProcess = open_mesh(_meshfilename);
    //if (!find_interior_point())
        find_bounding_sphere();
    transform_to_unit_sphere();
    set_point_to_center_of_one_ring_vertices();
    //transform_to_unit_sphere();

    mesh_.update_normals();
    std::string fname = _meshfilename;
    std::string newmeshname = "sphericparam_" + fname;
    OpenMesh::IO::write_mesh(mesh_, newmeshname.c_str());

    root = getShapeSep();
}

SphereParametrization::~SphereParametrization()
{
}

bool SphereParametrization::open_mesh(const char *_meshfilename)
{
    // load mesh
    if (OpenMesh::IO::read_mesh(mesh_, _meshfilename))
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
        mesh_.add_property(vparam_sph_);
        mesh_.add_property(fcent_);

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
        //mesh_.update_normals();
        // update face indices for faster rendering
        //update_face_indices();
        Mesh::VertexIter viter;
        for (viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
        {
            if (mesh_.is_boundary(*viter))
                ++nboundary;
            else
                ++ninterior;
            mesh_.property(vpos_, *viter) = mesh_.point(*viter);
        }

        // store vertex initial positions and 3D mesh bounding box
        Mesh::VertexIter v_it3(mesh_.vertices_begin()), v_end3(mesh_.vertices_end());
        if (mesh_.vertices_begin() != v_end3)
        {
            _bbMin3D = _bbMax3D = mesh_.point(*v_it3);
            for (v_it3 = mesh_.vertices_begin(); v_it3 != v_end3; ++v_it3)
            {
                mesh_.property(vpos_, *v_it3) = mesh_.point(*v_it3);
                _bbMin3D.minimize(mesh_.point(*v_it3));
                _bbMax3D.maximize(mesh_.point(*v_it3));
            }
        }
        for(Mesh::FaceIter fiter = mesh_.faces_begin(); fiter != mesh_.faces_end(); ++fiter){
            mesh_.calc_face_normal(*fiter);
            mesh_.property(fcent_, *fiter) = mesh_.calc_face_centroid(*fiter);
        }
        return true;
    }
    return false;
}
//-----------------------------------------------------------------------------

void SphereParametrization::split_mesh(float zCut, float zInsert, bool marginalTrisInLeft, bool marginalTrisInRight)
{
    /* IMeshBuffer *oldMeshBuf = oldMesh->getMeshBuffer(0);
    int *oldInd = oldMeshBuf->getIndices();
    SMeshBuffer *leftMeshBuf = new SMeshBuffer();
    SMeshBuffer *rightMeshBuf = new SMeshBuffer();

    OpenMesh::Vec3f offset(0.f, 0.f, zInsert);

    set<pair<Mesh::VertexHandle, Mesh::VertexHandle>> leftEdgeLinks;
    set<pair<Mesh::VertexHandle, Mesh::VertexHandle>> rightEdgeLinks;
    Mesh::FaceIter fiter;

    for (fiter = mesh_.faces_begin(); fiter != mesh_.faces_end(); ++fiter)
    {
        vector<Mesh::VertexHandle> leftShape;
        vector<Mesh::VertexHandle> rightShape;

        Mesh::FaceVertexIter fv_it = mesh_.fv_iter(*fiter);
        Mesh::VertexHandle v1 = (*fv_it);
        ++fv_it;
        Mesh::VertexHandle v2 = (*fv_it);
        ++fv_it;
        Mesh::VertexHandle v3 = (*fv_it);

        Mesh::VertexHandle mid1, mid2, mid3;
        bool mid1exist = false;
        bool mid2exist = false;
        bool mid3exist = false;

        if (mesh_.point(v1)[3] == zCut && mesh_.point(v2)[3] == zCut && mesh_.point(v3)[3] == zCut)
        {
            if (marginalTrisInLeft)
            {
                leftShape.push_back(v1);
                leftShape.push_back(v2);
                leftShape.push_back(v3);
            }

            if (marginalTrisInRight)
            {
                rightShape.push_back(v1);
                rightShape.push_back(v2);
                rightShape.push_back(v3);
            }
        }
        else
        {
            mid1exist = process_link(leftShape, rightShape, v1, v2, mid1);
            mid2exist = process_link(leftShape, rightShape, v2, v3, mid2);
            mid3exist = process_link(leftShape, rightShape, v3, v1, mid3);
        }

        // If a triangle was split then two of those three midpoints are inhabited by a vertex.
        // We want to get them both in mid1 and mid2 AND we need them in correct order.
        Mesh::VertexHandle cut1 = (mid1exist && mid2exist) ? mid1 : mid2;
        Mesh::VertexHandle cut2 = (mid2exist && mid3exist) ? mid3 : mid2;

        if (cut1 && cut2)
        {
            vector<Mesh::VertexHandle> chain = chopLink(cut1, cut2, 1);
            insertPoints(leftShape, cut1, cut2, chain);
            insertPoints(rightShape, cut1, cut2, chain);
        }

        addConvexShape(leftShape, leftMeshBuf, -offset / 2);
        addConvexShape(rightShape, rightMeshBuf, offset / 2);

        // Add any edges of the left shape that lie along the border.
        addEdgeLinks(leftShape, leftEdgeLinks);

        std::reverse(rightShape.begin(), rightShape.end());
        addEdgeLinks(rightShape, rightEdgeLinks);
    }

    SMesh *leftMesh = new SMesh();
    leftMeshBuf->recalculateBoundingBox();
    leftMeshBuf->setHardwareMappingHint(EHM_STATIC);
    leftMesh->addMeshBuffer(leftMeshBuf);
    leftMesh->recalculateBoundingBox();
    leftMeshBuf->drop(); // we drop the buf, mesh obj has it now

    SMesh *rightMesh = new SMesh();
    rightMeshBuf->recalculateBoundingBox();
    rightMeshBuf->setHardwareMappingHint(EHM_STATIC);
    rightMesh->addMeshBuffer(rightMeshBuf);
    rightMesh->recalculateBoundingBox();
    rightMeshBuf->drop(); // we drop the buf, mesh obj has it now

    SMesh *middleMesh = NULL;
    if (zInsert > 0)
    {
        set<pair<Mesh::VertexHandle, Mesh::VertexHandle>> result;
        std::set_intersection(
            leftEdgeLinks.begin(), leftEdgeLinks.end(),
            rightEdgeLinks.begin(), rightEdgeLinks.end(),
            std::inserter(result, result.begin()));

        size_t debugsize = result.size();

        if (result.size() > 0)
        {
            SMeshBuffer *middleMeshBuf = new SMeshBuffer();

            vector<Mesh::VertexHandle> shape(4);
            for (auto it = result.begin(); it != result.end(); ++it)
            {
                shape[0] = it->second;
                shape[1] = it->first;
                shape[2] = it->first->duplicate(offset);
                shape[3] = it->second->duplicate(offset);
                addConvexShape(shape, middleMeshBuf, -offset / 2);
            }

            middleMesh = new SMesh();
            middleMeshBuf->recalculateBoundingBox();
            middleMeshBuf->setHardwareMappingHint(EHM_STATIC);
            middleMesh->addMeshBuffer(middleMeshBuf);
            middleMesh->recalculateBoundingBox();
            middleMeshBuf->drop(); // we drop the buf, mesh obj has it now
        }
    } */
}
//-----------------------------------------------------------------------------

bool SphereParametrization::process_link(float zCut, vector<Mesh::VertexHandle> &leftInd, vector<Mesh::VertexHandle> &rightInd,
                                         Mesh::VertexHandle v1, Mesh::VertexHandle v2, Mesh::VertexHandle &mid)
{
    bool midExist = false;
    // Add a to left side if it is left or in both
    if (mesh_.point(v1)[3] <= zCut)
    {
        leftInd.push_back(v1);

        if (mesh_.point(v1)[3] == zCut)
        {
            mid = v1;
            midExist = true;
        }
    }

    // Add a to right side if it is right or in both
    if (mesh_.point(v1)[3] >= zCut)
    {
        rightInd.push_back(v1);

        if (mesh_.point(v1)[3] == zCut)
        {
            mid = v1;
            midExist = true;
        }
    }
    // Link crosses like this:  a --|--> b
    if (mesh_.point(v1)[3] < zCut && mesh_.point(v2)[3] > zCut)
    {
        if (fabs(mesh_.point(v1)[3] - zCut) < fabs(mesh_.point(v2)[3] < zCut))
        {
            mid = v1;
            rightInd.push_back(mid);
            midExist = true;
        }
        else
        {
            mid = v2;
            leftInd.push_back(mid);
            midExist = true;
        }
        leftInd.push_back(v1);
        rightInd.push_back(v2);
    }
    // Link crosses like this: b <--|-- a
    else if (mesh_.point(v2)[3] < zCut && mesh_.point(v1)[3] > zCut)
    {
        if (fabs(mesh_.point(v1)[3] - zCut) < fabs(mesh_.point(v2)[3] < zCut))
        {
            mid = v1;
            leftInd.push_back(mid);
            midExist = true;
        }
        else
        {
            mid = v2;
            rightInd.push_back(mid);
            midExist = true;
        }
        leftInd.push_back(v2);
        rightInd.push_back(v1);
    }

    return midExist;
}

//-----------------------------------------------------------------------------

vector<PossibleVertex> SphereParametrization::chopLink(Mesh::VertexHandle left, Mesh::VertexHandle right, int numNewPoints)
{
    vector<PossibleVertex> result;

    for (int i = 1; i <= numNewPoints; ++i)
    {
        float scale = ((float)i) / (numNewPoints + 1.0f);
        PossibleVertex pv = getPossibleVertex(left, right, scale);
        result.push_back(pv);
    }

    return result;
}

//-----------------------------------------------------------------------------

PossibleVertex SphereParametrization::getPossibleVertex(Mesh::VertexHandle vleft, Mesh::VertexHandle vright, float scale)
{

    float leftCoeff = (1.f - scale);
    OpenMesh::Vec2f UVCoord;
    UVCoord[0] = (mesh_.property(vparam_uv_, vleft)[0] * leftCoeff) + (mesh_.property(vparam_uv_, vright)[0] * scale);
    UVCoord[1] = (mesh_.property(vparam_uv_, vleft)[1] * leftCoeff) + (mesh_.property(vparam_uv_, vright)[1] * scale);

    Mesh::Normal vnorm;
    vnorm[0] = mesh_.calc_vertex_normal(vleft)[0] * leftCoeff + mesh_.calc_vertex_normal(vright)[0] * scale;
    vnorm[1] = mesh_.calc_vertex_normal(vleft)[1] * leftCoeff + mesh_.calc_vertex_normal(vright)[1] * scale;
    vnorm[2] = mesh_.calc_vertex_normal(vleft)[2] * leftCoeff + mesh_.calc_vertex_normal(vright)[2] * scale;

    Mesh::Point pos;
    pos[0] = (mesh_.property(vpos_, vleft)[0] * leftCoeff) + (mesh_.property(vpos_, vright)[0] * scale);
    pos[1] = (mesh_.property(vpos_, vleft)[1] * leftCoeff) + (mesh_.property(vpos_, vright)[1] * scale);
    pos[2] = (mesh_.property(vpos_, vleft)[2] * leftCoeff) + (mesh_.property(vpos_, vright)[2] * scale);

    PossibleVertex vert;
    vert.vnorm_ = vnorm;
    vert.vparam_uv_ = UVCoord;
    vert.vpos_ = pos;

    return vert;
}

//-----------------------------------------------------------------------------

void SphereParametrization::insertPoints(vector<PossibleVertex> &shape, PossibleVertex left, PossibleVertex right, vector<PossibleVertex> const &source)
{
    /* auto iterL = std::find(shape.begin(), shape.end(), left);
    auto iterR = std::find(shape.begin(), shape.end(), right);

    bool leftToRight = ((iterR - iterL) == 1) || (iterR == shape.begin() && iterL == (shape.end() - 1));

    if (leftToRight)
        shape.insert(iterL + 1, source.begin(), source.end());
    else
        shape.insert(iterR + 1, source.rbegin(), source.rend()); */
}
//-----------------------------------------------------------------------------

/* void SphereParametrization::addConvexShape(vector<PossibleVertex> const &newShape, SMeshBuffer *newMeshBuf, vector3df offset)
{
    int numCorners = newShape.size();
    switch (numCorners)
    {
    case 0:
        break; // no triangle
    case 1:
        break; // incomplete triangle
    case 2:
        break; // incomplete triangle
    case 3:
        for (int j = 0; j < 3; ++j)
            newShape[j]->addToMeshBuf(newMeshBuf, offset);
        break;
    default:
    {
        unsigned int bestIndex = 0;
        float bestRatio = 1.99f;
        for (unsigned int i = 0; i < numCorners; ++i)
        {
            float a = newShape[i]->dist(newShape[(i + numCorners - 1) % numCorners]);
            float b = newShape[i]->dist(newShape[(i + 1) % numCorners]);
            float c = newShape[(i + numCorners - 1) % numCorners]->distSQ(newShape[(i + 1) % numCorners]);

            float ratio = c / (a + b);

            if (ratio < bestRatio)
                bestIndex = i;
        }

        newShape[(bestIndex + numCorners - 1) % numCorners]->addToMeshBuf(newMeshBuf, offset);
        newShape[bestIndex]->addToMeshBuf(newMeshBuf, offset);
        newShape[(bestIndex + 1) % numCorners]->addToMeshBuf(newMeshBuf, offset);

        vector<PsblVertPtr> smallerShape;
        for (unsigned int i = 0; i < numCorners; ++i)
            if (i != bestIndex)
                smallerShape.push_back(newShape[i]);

        addConvexShape(smallerShape, newMeshBuf, offset);
    }
    }
} */

//-----------------------------------------------------------------------------

void SphereParametrization::addEdgeLinks(vector<PossibleVertex> const &shape, set<pair<PossibleVertex, PossibleVertex>> &links, float zCut)
{
    /* if (shape.size() >= 2)
    {
        for (size_t i = 0; i < shape.size(); ++i)
        {
            PossibleVertex a = shape[i];
            PossibleVertex b = shape[(i + 1) % shape.size()];
            if (compareZ(a, zCut) == 0 && compareZ(b, zCut) == 0)
                links.insert(make_pair(a, b));
        }
    } */
}

//-----------------------------------------------------------------------------
int SphereParametrization::compareZ(int index, float zCut)
{
    /* PossibleVertex vert = oldMeshBuf[index];
    float tolerance = 0.000001f;
    float z = vert.vpos_[2];
    if ((z + tolerance >= zCut) && (z - tolerance <= zCut))
        return 0;
    else if (z < zCut)
        return -1;
    else*/
    return 1;
}
//-----------------------------------------------------------------------------
int SphereParametrization::compareZ(PossibleVertex vert, float zCut)
{
    float tolerance = 0.000001f;
    float z = vert.vpos_[2];
    if ((z + tolerance >= zCut) && (z - tolerance <= zCut))
        return 0;
    else if (z < zCut)
        return -1;
    else
        return 1;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

SoSeparator *SphereParametrization::getShapeSep()
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
    Mesh::VertexIter v_it = mesh_.vertices_begin(), v_end(mesh_.vertices_end());
    int c = 0.;
    for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it)
    {
        coords->point.set1Value((*v_it).idx(), mesh_.property(vpos_, *v_it)[0], mesh_.property(vpos_, *v_it)[1], mesh_.property(vpos_, *v_it)[2]);
        c++;
    }

    SoIndexedFaceSet *faceSet = new SoIndexedFaceSet();
    int c1 = 0;
    Mesh::FaceIter f_it = mesh_.faces_begin(), f_end(mesh_.faces_end());
    for (f_it = mesh_.faces_begin(); f_it != f_end; ++f_it)
    {
        int c2 = 0;
        Mesh::FaceVertexIter fvit = mesh_.fv_iter(*f_it);

        for (fvit = mesh_.fv_iter(*f_it); fvit.is_valid(); ++fvit)
        {
            faceSet->coordIndex.set1Value(c1 * 4 + c2, (*fvit).idx());
            faceSet->textureCoordIndex.set1Value(c1 * 4 + c2, (*fvit).idx());
            if (youWantToPaintEachVertexDifferently)
                faceSet->materialIndex.set1Value(c1 * 4 + c2, (*fvit).idx());
            ++c2;
        }
        faceSet->coordIndex.set1Value(c1 * 4 + 3, -1);
        ++c1;
    }

    res->addChild(coords);
    res->addChild(faceSet);

    return res;
}

//-----------------------------------------------------------------------------

void SphereParametrization::find_bounding_sphere()
{
    register double dx, dy, dz;
    register double rad_sq, xspan, yspan, zspan, maxspan;
    double old_to_p, old_to_p_sq, old_to_new;
    Mesh::Point xmin, xmax, ymin, ymax, zmin, zmax, dia1, dia2;

    /* FIRST PASS: find 6 minima/maxima points */
    xmin[0] = ymin[1] = zmin[2] = INT_MAX; /* initialize for min/max compare */
    xmax[0] = ymax[1] = zmax[2] = -INT_MAX;

    for (Mesh::VertexIter viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
    {
        Mesh::Point caller_p = mesh_.point(*viter);

        if (caller_p[0] < xmin[0])
            xmin = caller_p; /* New xminimum point */
        if (caller_p[0] > xmax[0])
            xmax = caller_p;
        if (caller_p[1] < ymin[1])
            ymin = caller_p;
        if (caller_p[1] > ymax[1])
            ymax = caller_p;
        if (caller_p[2] < zmin[2])
            zmin = caller_p;
        if (caller_p[2] > zmax[2])
            zmax = caller_p;
    }
    /* Set xspan = distance between the 2 points xmin & xmax (squared) */
    dx = xmax[0] - xmin[0];
    dy = xmax[1] - xmin[1];
    dz = xmax[2] - xmin[2];
    xspan = dx * dx + dy * dy + dz * dz;

    /* Same for y & z spans */
    dx = ymax[0] - ymin[0];
    dy = ymax[1] - ymin[1];
    dz = ymax[2] - ymin[2];
    yspan = dx * dx + dy * dy + dz * dz;

    dx = zmax[0] - zmin[0];
    dy = zmax[1] - zmin[1];
    dz = zmax[2] - zmin[2];
    zspan = dx * dx + dy * dy + dz * dz;

    /* Set points dia1 & dia2 to the maximally separated pair */
    dia1 = xmin;
    dia2 = xmax; /* assume xspan biggest */
    maxspan = xspan;
    if (yspan > maxspan)
    {
        maxspan = yspan;
        dia1 = ymin;
        dia2 = ymax;
    }
    if (zspan > maxspan)
    {
        dia1 = zmin;
        dia2 = zmax;
    }

    sphereCenterPoint[0] = (dia1[0] + dia2[0]) / 2.0;
    sphereCenterPoint[1] = (dia1[1] + dia2[1]) / 2.0;
    sphereCenterPoint[2] = (dia1[2] + dia2[2]) / 2.0;
    /* calculate initial radius**2 and radius */
    dx = dia2[0] - sphereCenterPoint[0]; /* x component of radius vector */
    dy = dia2[1] - sphereCenterPoint[1]; /* y component of radius vector */
    dz = dia2[2] - sphereCenterPoint[2]; /* z component of radius vector */
    rad_sq = dx * dx + dy * dy + dz * dz;
    sphereRad = sqrt(rad_sq);

    /* SECOND PASS: increment current sphere */

    for (Mesh::VertexIter viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
    {
        Mesh::Point caller_p = mesh_.point(*viter);
        dx = caller_p[0] - sphereCenterPoint[0];
        dy = caller_p[1] - sphereCenterPoint[1];
        dz = caller_p[2] - sphereCenterPoint[2];
        old_to_p_sq = dx * dx + dy * dy + dz * dz;
        if (old_to_p_sq > rad_sq) /* do r**2 test first */
        {                         /* this point is outside of current sphere */
            old_to_p = sqrt(old_to_p_sq);
            /* calc radius of new sphere */
            sphereRad = old_to_p;           //(sphereRad + old_to_p) / 2.0;
            rad_sq = sphereRad * sphereRad; /* for next r**2 compare */
            old_to_new = old_to_p - sphereRad;
            /* calc center of new sphere */
            sphereCenterPoint[0] = (sphereRad * sphereCenterPoint[0] + old_to_new * caller_p[0]) / old_to_p;
            sphereCenterPoint[1] = (sphereRad * sphereCenterPoint[1] + old_to_new * caller_p[1]) / old_to_p;
            sphereCenterPoint[2] = (sphereRad * sphereCenterPoint[2] + old_to_new * caller_p[2]) / old_to_p;
        }
    }
}

//-----------------------------------------------------------------------------

void SphereParametrization::transform_to_unit_sphere()
{
    for (Mesh::VertexIter viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
    {
        double v_x = mesh_.point(*viter)[0] - sphereCenterPoint[0],
               v_y = mesh_.point(*viter)[1] - sphereCenterPoint[1],
               v_z = mesh_.point(*viter)[2] - sphereCenterPoint[2];

        double rho = sqrt(v_x * v_x + v_y * v_y + v_z * v_z);
        if (rho != 0)
        {
            v_x = v_x / rho;
            v_y = v_y / rho;
            v_z = v_z / rho;
        }

        double phi = acos(v_z);
        double theta = atan2(v_y , v_x);

        if (phi < 0)
            phi = PI + phi;
        if (phi > PI)
            phi = PI - phi;
        if (theta < 0)
            theta = 2 * PI + theta;

        OpenMesh::Vec3f sphCoorV;
        sphCoorV[0] = 1.;
        sphCoorV[1] = theta;
        sphCoorV[2] = phi;
        double xp = sin(phi) * cos(theta);
        if (theta > PI/2. && theta < 3*PI/2.)
        {
            double s1 = sin(phi);
            double c1 = (-1)*fabs(cos(theta));
            xp = s1*c1;
            if (s1 > 0 && c1 < 0 && xp > 0)
                xp = - xp;
        }     
        Mesh::Point pnt = Mesh::Point(xp, sin(phi) * sin(theta), cos(phi));
        mesh_.property(vparam_sph_, *viter) = sphCoorV;
        mesh_.property(vpos_, *viter) = pnt;
        mesh_.set_point(*viter, pnt);
    }
}

//-----------------------------------------------------------------------------

void SphereParametrization::set_point_to_center_of_one_ring_vertices()
{
    Mesh::VertexIter          v_it, v_end(mesh_.vertices_end());
    Mesh::VertexVertexIter    vv_it;
    Mesh::Point               cog, cogpos;
    Mesh::Scalar              valence;
    for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it)
    {
        cog[0] = cog[1] = cog[2] = cogpos[0] = cogpos[1] = cogpos[2] = valence = 0.0;
        
        for (vv_it=mesh_.vv_iter( *v_it ); vv_it.is_valid(); ++vv_it)
        {
            cog += mesh_.property(vpos_, *v_it);
            Mesh::Point pnt = Mesh::Point(mesh_.property(vparam_sph_, *vv_it)[0], mesh_.property(vparam_sph_, *vv_it)[1], mesh_.property(vparam_sph_, *vv_it)[2]);
            cogpos += pnt;
            ++valence;
        }

        if ( !mesh_.is_boundary( *v_it ) && valence != 0.)
        {
            mesh_.set_point( *v_it, cog / valence);
            mesh_.property(vparam_sph_, *v_it) = cogpos / valence;
            mesh_.property(vpos_, *v_it) = cog / valence;
        }
    }
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

bool SphereParametrization::lineIntersection(float x11, float x12, float y11, float y12,
                                             float x21, float x22, float y21, float y22,
                                             float &intersection_X, float &intersection_Y)
{
    float m1, c1, m2, c2;
    float dx, dy;

    dx = x12 - x11;
    dy = y12 - y11;
    m1 = dy / dx;

    c1 = y11 - m1 * x11;

    dx = x22 - x21;
    dy = y22 - y21;
    m2 = dy / dx;

    c2 = y22 - m2 * x22;

    if ((m1 - m2) == 0)
        return false;
    else
    {
        intersection_X = (c2 - c1) / (m1 - m2);
        intersection_Y = m1 * intersection_X + c1;
        return true;
    }
}

//-----------------------------------------------------------------------------

bool SphereParametrization::find_interior_point()
{
    sphereRad = 1.; 

    vector<vector<Mesh::Point>> lines, lines2;
    double distnc = 0;
    double maxdist = 0.;
    for(Mesh::FaceIter fiter = mesh_.faces_begin(); fiter != mesh_.faces_end(); ++fiter)
    {
        Mesh::Normal normFace = mesh_.calc_face_normal(*fiter);
        double x2 = normFace[0], y2 = normFace[1], z2 = normFace[2];
        for(Mesh::FaceIter fiter2 = mesh_.faces_begin(); fiter2 != mesh_.faces_end(); ++fiter2)
        {
            if ((*fiter).idx() != (*fiter2).idx())
            {
                vector<Mesh::Point> line = vector<Mesh::Point>(2);
                line[0] =  mesh_.property(fcent_, *fiter);
                line[1] = mesh_.property(fcent_, *fiter2);
                double x1 = line[1][0] - line[0][0], y1 = line[1][1] - line[0][1], z1 = line[1][2] - line[0][2];
                double dot = x1*x2 + y1*y2 + z1*z2;
                double det = (y1*z2 - z1*y2) - (x1*z2 - z1*x2) + (x1*y2 - y1*x2);
                double angle = atan2(det, dot);
                double dist = sqrt(x1*x1 + y1*y1 + z1*z1);
                maxdist = max(maxdist, dist);
                if (angle > PI/2. && angle < 3*PI/2. ){
                    distnc = distnc + dist;
                    if (dist >= maxdist)
                        lines2.push_back(line);
                }
            }
        }
    }
    if (lines2.size() > 10000) return false;
    distnc = distnc/lines2.size();
    for(int i = 0; i < lines2.size(); ++i)
    {
        double x1 = lines2[i][1][0] - lines2[i][0][0], y1 = lines2[i][1][1] - lines2[i][0][1], z1 = lines2[i][1][2] - lines2[i][0][2];
        if (sqrt(x1*x1 + y1*y1 + z1*z1) >= maxdist)
            lines.push_back(lines2[i]);
    }
    cout << "size of lines: " << lines.size() << endl;

    vector<Mesh::Point> interior_points;
    for(int indx = 0; indx < lines.size(); ++indx)
    {
        for(int indx2 = 0; indx2 < lines.size(); ++indx2)
        {
            if (indx == indx2) continue;
            float inty = 0., intz = 0., intx = 0.;

            if (lineIntersection(lines[indx][0][0], lines[indx][1][0], lines[indx][0][2], lines[indx][1][2], 
                                 lines[indx2][0][0], lines[indx2][1][0], lines[indx2][0][2], lines[indx2][1][2], intx, intz))
            {
                float dummy = 0.;
                if (lineIntersection(lines[indx][0][1], lines[indx][1][1], lines[indx][0][2], lines[indx][1][2], 
                                    lines[indx2][0][1], lines[indx2][1][1], lines[indx2][0][2], lines[indx2][1][2], inty, dummy))
                {
                    if (intz == 0. && dummy != 0.)
                        intz = dummy;
                }
            }
            else if (lineIntersection(lines[indx][0][0], lines[indx][1][0], lines[indx][0][1], lines[indx][1][1], 
                                     lines[indx2][0][0], lines[indx2][1][0], lines[indx2][0][1], lines[indx2][1][1],intx, inty))
            {
                float dummy = 0.;
                if (lineIntersection(lines[indx][0][1], lines[indx][1][1], lines[indx][0][2], lines[indx][1][2], 
                                     lines[indx2][0][1], lines[indx2][1][1], lines[indx2][0][2], lines[indx2][1][2], dummy, intz))
                {
                    if (inty == 0. && dummy != 0.)
                        inty = dummy;
                }
            }
            
            if (inty != 0. && intz != 0. && intx != 0.)
            {
                Mesh::Point interior_point;
                interior_point[0] = intx;
                interior_point[1] = inty;
                interior_point[2] = intz;
                interior_points.push_back(interior_point);
            }
        }
    }
    cout << "size of int points: " << interior_points.size() << endl;
    if (interior_points.size() == 0)
        return false;
    else 
    {
        double minEnergy = INT_MAX;
        sphereCenterPoint = interior_points[0];
        for (int i = 0; i < interior_points.size(); ++i)
        {
            vector<double> distances;
            for (Mesh::VertexIter viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
            {
                Mesh::Point caller_p = mesh_.point(*viter);
                double dx = caller_p[0] - interior_points[i][0];
                double dy = caller_p[1] - interior_points[i][1];
                double dz = caller_p[2] - interior_points[i][2];
                double dist = sqrt(dx * dx + dy * dy + dz * dz);
                distances.push_back(dist);
            }
            double distEnergy = 0.;
            for (int j = 0; j < distances.size(); ++j)
            {
                for (int k = 0; k < distances.size(); ++k)
                {
                    distEnergy = fabs(distEnergy) + fabs(0.5*distances[j]*distances[k]);
                }
            }
            if (i > 0)
            {
                if (distEnergy < minEnergy){
                    sphereCenterPoint = interior_points[i];
                    minEnergy = distEnergy;
                }
            }
        }

        return true;
    }
}

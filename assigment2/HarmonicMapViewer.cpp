#include "HarmonicMapViewer.h"
#include "Eigen/Sparse"

HarmonicMapViewer::HarmonicMapViewer(int iRepeat, int _drawMode, int _taskNumber)
    : MeshViewer(_drawMode, _taskNumber)
{
    _Repeat = iRepeat;

    drawMode = _drawMode;
    _ParameterizationMode_ = (ParameterizationMode)_drawMode;
    taskNumber = _taskNumber;
    init();
}

HarmonicMapViewer::HarmonicMapViewer(Mesh _mesh_, int iRepeat, int _drawMode, int _taskNumber)
  : MeshViewer(_drawMode, _taskNumber)
{
     _Repeat = iRepeat;
     _Repeat = iRepeat;

    drawMode = _drawMode;
    _ParameterizationMode_ = (ParameterizationMode)_drawMode;
    taskNumber = _taskNumber;
    init();
}

HarmonicMapViewer::~HarmonicMapViewer()
{
    if (_TextureCoordinates_u)
        delete[] _TextureCoordinates_u;

    if (_TextureCoordinates_h)
        delete[] _TextureCoordinates_h;
}

void HarmonicMapViewer::init()
{
}

bool HarmonicMapViewer::open_mesh(const char *_meshfilename)
{
    // load mesh
    if (MeshViewer::open_mesh(_meshfilename))
    {
        // store vertex initial positions and 3D mesh bounding box
        Mesh::VertexIter v_it(mesh_.vertices_begin()), v_end(mesh_.vertices_end());
        if (mesh_.vertices_begin() != v_end)
        {
            _bbMin3D = _bbMax3D = mesh_.point(*v_it);
            for (v_it = mesh_.vertices_begin(); v_it != v_end; ++v_it)
            {
                mesh_.property(vpos_, *v_it) = mesh_.point(*v_it);
                _bbMin3D.minimize(mesh_.point(*v_it));
                _bbMax3D.maximize(mesh_.point(*v_it));
            }
        }
        Mesh::VertexIter viter, v_end2(mesh_.vertices_end());
        for (viter = mesh_.vertices_begin(); viter != v_end2; ++viter)
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
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void HarmonicMapViewer::calc_parameterization()
{
    std::map<int, float> edgeWeights = compute_weights();
    std::vector<Eigen::Triplet<double> > A_coefficients;
	std::vector<Eigen::Triplet<double> > B_coefficients;

    for (Mesh::VertexIter viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
    {
        int vid = (*viter).idx();
        if (mesh_.is_boundary(*viter))
            continue;

        double sw = 0;
        for (Mesh::VertexVertexIter witer = mesh_.vv_begin(*viter); witer != mesh_.vv_end(*viter); ++witer)
        {
            int wid = (*witer).idx();

            int edgeId = mesh_.edge_handle(mesh_.find_halfedge((*viter), (*witer))).idx();
            std::map<int, float>::iterator it;
            it = edgeWeights.find(edgeId);
            if (it == edgeWeights.end())
                continue;
            double w = (*it).second;

            if (mesh_.is_boundary(*witer))
            {
                Eigen::Triplet<double> e(vid,wid,w);
				B_coefficients.push_back( e );
            }
            else
            {
                A_coefficients.push_back( Eigen::Triplet<double>(vid,wid, -w) );
            }
            sw += w;
        }
        A_coefficients.push_back( Eigen::Triplet<double>(vid,vid, sw ) );
        
    }

	Eigen::SparseMatrix<double> A( mesh_.n_vertices(), mesh_.n_vertices() );
	A.setZero();

	Eigen::SparseMatrix<double> B( mesh_.n_vertices(), mesh_.n_vertices() );
	B.setZero();
	A.setFromTriplets(A_coefficients.begin(), A_coefficients.end());
	B.setFromTriplets(B_coefficients.begin(), B_coefficients.end());


	Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> solver;
	solver.compute(A);
	if( solver.info() != Eigen::Success )
		return;

    Eigen::VectorXd x1, x2;

	for( int k = 0; k < 2; k ++ )
	{
		Eigen::VectorXd b(mesh_.n_vertices());
		//set boundary constraints b vector
		for (Mesh::VertexIter viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
        {
            int vid = (*viter).idx();
            if (!mesh_.is_boundary(*viter)) continue;
                b(vid) = mesh_.property(vparam_uv_, *viter)[k];
		}

		Eigen::VectorXd c(mesh_.n_vertices());
		c = B * b;
        if (k == 0)
		    x1 = solver.solve(c);
        else 
            x2 = solver.solve(c);
		if( solver.info() != Eigen::Success )
		    return;
	}

    for (Mesh::VertexIter viterh = mesh_.vertices_begin(); viterh != mesh_.vertices_end(); ++viterh)
    {
        if (mesh_.is_boundary(*viterh))
            continue;
        int id = (*viterh).idx();
        OpenMesh::Vec2f UVCoors;
        UVCoors[0] = x1[id];
        UVCoors[1] = x2[id];
        mesh_.set_texcoord2D(*viterh, UVCoors);
        mesh_.property(vparam_uv_, *viterh) = UVCoors;
    }
    mesh_.update_vertex_normals();
    mesh_.update_face_normals();
}
//-----------------------------------------------------------------------------

void HarmonicMapViewer::compute_boundary_uvcoordinates()
{
    Mesh::HalfedgeIter hiter;
    for (hiter = mesh_.halfedges_begin(); hiter != mesh_.halfedges_end(); ++hiter)
    {
        double sum = 0;
        Mesh::HalfedgeLoopIter hliter;
        for (hliter = mesh_.hl_begin(*hiter); hliter != mesh_.hl_end(*hiter); hliter++)
        {
            sum += mesh_.calc_edge_length(*hliter);
        }
        double l = 0;
        Mesh::HalfedgeLoopIter hliter2;
        for (hliter2 = mesh_.hl_begin(*hiter); hliter2 != mesh_.hl_end(*hiter); hliter2++)
        {
            if (!mesh_.is_boundary(*hliter2))
                continue;
            l += mesh_.calc_edge_length(*hliter2);
            double ang = l / sum * 2.0 * PI;
            OpenMesh::Vec2f texCoorV;
            texCoorV[0] = (cos(ang) + 1.0) / 2.0;
            texCoorV[1] = (sin(ang) + 1.0) / 2.0;
            mesh_.set_texcoord2D(mesh_.to_vertex_handle(*hliter2), texCoorV);
            mesh_.property(vparam_uv_, mesh_.to_vertex_handle(*hliter2)) = texCoorV;
        }
    }
}

//-----------------------------------------------------------------------------

std::map<int, float> HarmonicMapViewer::compute_weights()
{
    std::map<int, float> edgeWeights;
    if (_ParameterizationMode_ == 0 || _ParameterizationMode_ == 1)
    {
        Mesh::EdgeIter eiter, e_end(mesh_.edges_end());
        for (eiter = mesh_.edges_begin(); eiter != e_end; ++eiter)
        {
            float wt = 1;
            if (_ParameterizationMode_ == 1)
            {
                Mesh::HalfedgeHandle he = mesh_.halfedge_handle(*eiter, 0);
                float sangle = mesh_.calc_sector_angle(he);
                wt = (cos(sangle) / sin(sangle)) / 2.;
                Mesh::HalfedgeHandle sh = mesh_.halfedge_handle(*eiter, 1);
                if (sh.is_valid())
                {
                    float sangle2 = mesh_.calc_sector_angle(sh);
                    wt += (cos(sangle2) / sin(sangle2)) / 2.;
                }
            }
            std::pair<int, float> p1;
            p1.first = (*eiter).idx();
            p1.second = wt;
            edgeWeights.insert(p1);
        }
    }
    else if (_ParameterizationMode_ == 2)
    {
        Mesh::VertexIter viter, v_end(mesh_.vertices_end());
        for (viter = mesh_.vertices_begin(); viter != v_end; ++viter)
        {
            int vid = (*viter).idx();
            if (mesh_.is_boundary(*viter))
                continue;
            vector<int> neighbors;
            vector<int> edgesofneighbors;
            get_one_ring_vertices(vid, neighbors, edgesofneighbors);
            Mesh::Point center_p = mesh_.point(mesh_.vertex_handle(vid));

            for (size_t j = 0; j < neighbors.size(); j++)
            {
                size_t current = neighbors[j];
                size_t pre = neighbors[(j - 1 + neighbors.size()) % neighbors.size()];
                size_t next = neighbors[(j + 1) % neighbors.size()];

                Mesh::Point current_p = mesh_.point(mesh_.vertex_handle(current));
                Mesh::Point pre_p = mesh_.point(mesh_.vertex_handle(pre));
                Mesh::Point next_p = mesh_.point(mesh_.vertex_handle(next));

                float dist = distance(center_p, current_p);
                float t1 = tan2(current_p - center_p, pre_p - center_p);
                float t2 = tan2(current_p - center_p, next_p - center_p);
                float wt = (t1 + t2) / dist;

                std::pair<int, float> p1;
                p1.first = edgesofneighbors[j];
                p1.second = wt;
                edgeWeights.insert(p1);
            }
        }
    }
    return edgeWeights;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void HarmonicMapViewer::draw(SoSeparator *root)
{
    root->ref();
    for(Mesh::VertexIter viter = mesh_.vertices_begin(); viter != mesh_.vertices_end(); ++viter)
    {
        OpenMesh::Vec2f UVCoord;
        UVCoord[0] = mesh_.texcoord2D(*viter)[0];
        UVCoord[1] = mesh_.texcoord2D(*viter)[1];
        mesh_.property(vpos_, *viter) =  Mesh::Point(UVCoord[0], UVCoord[1], 0.);
        mesh_.set_point(*viter, Mesh::Point(UVCoord[0], UVCoord[1], 0.));
    }
    mesh_.update_normals();
    update_face_indices();
    std::string newmeshname = "param_" + std::to_string(_ParameterizationMode_) +"_" + fileName;
    OpenMesh::IO::write_mesh(mesh_, newmeshname.c_str());
    root->addChild(getShapeSep());
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void HarmonicMapViewer::mapping()
{
    compute_boundary_uvcoordinates();
    calc_parameterization();
}

//=============================================================================
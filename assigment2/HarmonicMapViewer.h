#ifndef HARMONIC_MAP_VIEWER_HH
#define HARMONIC_MAP_VIEWER_HH

#include "MeshViewer.h"
#include <map>
#include <iostream>

class HarmonicMapViewer : public MeshViewer
{
public:
    enum ParameterizationMode
    {
        NoParameterization,
        Uniform,
        Harmonic,
        MeanValue
    };

    HarmonicMapViewer(int iRepeat, int _drawMode, int _taskNumber);
    HarmonicMapViewer(Mesh _mesh_, int iRepeat, int _drawMode, int _taskNumber);
    ~HarmonicMapViewer();

    virtual bool open_mesh(const char *_meshfilename);
    void draw(SoSeparator *root);
    void mapping();

private:
    void compute_boundary_uvcoordinates();
    void calc_parameterization();

    virtual void init();
    std::map<int, float> compute_weights();

private:
    int _Repeat;
    Mesh::Point _bbMin3D, _bbMax3D, _bbMin2D, _bbMax2D;
    ParameterizationMode _ParameterizationMode_;
};

#endif // HARMONIC_MAP_VIEWER_HH

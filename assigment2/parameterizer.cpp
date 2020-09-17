//=============================================================================
#include "HarmonicMapViewer.h"
#include "Windows.h"
#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include "SphereParametrization.h"

void showWindow(HWND sowinwindow, int& newWin);

int main(int argc, char **argv)
{
    HWND sowinwindow = SoWin::init(argv[0]);
    int newWin = 1;
    while(newWin == 1){
        newWin = -1;
        showWindow(sowinwindow, newWin);
    }
    return 0;
}

void showWindow(HWND sowinwindow, int& newWin)
{
    int paintV = 0;
    std::cout << "Paint each vertex differently?(0: No, 1: Yes): ";
    std::cin >> paintV;

    int taskNumber = 0;
    std::cout << "Task (0->Disk Topology, 1->Sphere Topology, 2->Sphere Generation, 3->Show Mesh): ";
    std::cin >> taskNumber;
    
    SoWinExaminerViewer *viewer = new SoWinExaminerViewer(sowinwindow);
    viewer->setSize(SbVec2s(640, 480));

    SoSeparator *root = new SoSeparator;
    root->ref();
    SoSceneTexture2* scntx2 = NULL;
    std::string meshPath; //= "..\\meshes2\\";

    bool showWindow = false;
    if (taskNumber == 0 || taskNumber == 1 || taskNumber == 3)
    {
        std::string meshName = "facem-low.off";
        std::cout << "Write mesh file full name (with extension): ";
        std::string scanname = "";
        std::cin >> scanname;
        if (scanname != " " && scanname != "" && scanname != "0")
            meshName = scanname;
        meshName = meshPath + meshName;
        const char *fileName = meshName.c_str();
        if (taskNumber == 0)
        {
            int paramMode = 0;
            std::cout << "Parameterization (0->Uniform, 1->Harmonic, 2->Mean-Value): ";
            std::cin >> paramMode;

            HarmonicMapViewer mapWindow(0, paramMode, taskNumber);
            if (paintV)
                mapWindow.setPaintEachVertexDifferently();

            if (!mapWindow.open_mesh(fileName))
            {
                cout << "mesh is not open!" << endl;
                cout << "mesh file path is: " << fileName << endl;
            }
            else
            {
                mapWindow.mapping();
                mapWindow.draw(root);

                scntx2 = mapWindow.getTexturedScene();
                /* viewer->setSceneGraph(scntx2);
                //scntx2->unref(); */
                showWindow = true;
                viewer->setSceneGraph(root);
                viewer->show();

                SoWin::show(sowinwindow);
                SoWin::mainLoop();
            }
        }
        else if (taskNumber == 1)
        {
            int topglMode = 0;
            std::cout << "Parameterization with (0->Cutting Close Shape, 1->Sphere Parameterization): ";
            std::cin >> topglMode;

            if (topglMode == 0)
            {
                int paramMode = 0;
                std::cout << "Parameterization (0->Uniform, 1->Harmonic, 2->Mean-Value): ";
                std::cin >> paramMode;
                SphereParametrization spwin(fileName, root, paramMode, showWindow);
            }
            else if (topglMode == 1)
            {
                SphereParametrization spwin(fileName, root, showWindow);
            }
        }
        else if (taskNumber == 3)
        {
            MeshViewer meshWin(-1, 3);
            if (meshWin.open_mesh(fileName))
            {
                if (paintV)
                    meshWin.setPaintEachVertexDifferently();
                root = meshWin.getShapeSep();
                showWindow = true;
            }
        }
    }
    else if (taskNumber == 2)
    {
        MeshViewer sphereWindow(-1, 2);
        if (paintV)
            sphereWindow.setPaintEachVertexDifferently();
        sphereWindow.generateSphere(root);
        showWindow = true;
    }

    if (showWindow)
    {
        viewer->setSceneGraph(root);
        viewer->show();

        SoWin::show(sowinwindow);
        SoWin::mainLoop();
    }
    root->unref();

    delete viewer;

    std::cout << "New Window: (0: No, 1: Yes): ";
    std::cin >> newWin;
}


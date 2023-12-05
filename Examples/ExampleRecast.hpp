#ifndef __CFORGE_EXAMPLERECAST_HPP__
#define __CFORGE_EXAMPLERECAST_HPP__

#include <iostream>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include "ExampleSceneBase.hpp"


using namespace Eigen;
using namespace std;

namespace CForge {

    struct NavMeshSetHeader {
        int magic;
        int version;
        int numTiles;
        dtNavMeshParams params;
    };

    struct NavMeshTileHeader {
        dtTileRef tileRef;
        int dataSize;
    };

    static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
    static const int NAVMESHSET_VERSION = 1;

    class ExampleRecast : public ExampleSceneBase {
    protected:
        SGNTransformation m_RootSGN;
        StaticActor m_Terrain;
        SGNGeometry m_TerrainGeo;
        SGNTransformation m_TerrainTransform;
        StaticActor m_NavMeshActor;
        SGNGeometry m_NavMeshGeo;
        SGNTransformation m_NavmeshTransform;
        // dtNavMeshQuery m_NavMeshQuery;
        dtNavMesh *navMesh;
        T3DMesh<float> navMeshT3DMesh;
        StaticActor actor;

        dtNavMesh *loadNavMesh(const char *path);

    public:
        ExampleRecast();

        ~ExampleRecast();

        void init() override;

        // void clear() override;
        void mainLoop() override;
    };

    ExampleRecast::ExampleRecast() {
        m_WindowTitle = "CrossForge Example - Recast";
        m_WinWidth = 1280;
        m_WinHeight = 720;
    }

    ExampleRecast::~ExampleRecast() {
        clear();
    }

    void ExampleRecast::init() {

        initWindowAndRenderDevice();
        initCameraAndLights();
        initFPSLabel();

        m_RootSGN.init(nullptr);
        m_SG.init(&m_RootSGN);

        T3DMesh<float> TerrainMesh;

        // initGroundPlane(&m_RootSGN, 100.0f, 20.0f);

        SAssetIO::load("Assets/Scene/end_mvp.gltf", &TerrainMesh);
        for (uint32_t i = 0; i < TerrainMesh.materialCount(); ++i)
            CForgeUtility::defaultMaterial(TerrainMesh.getMaterial(i), CForgeUtility::STONE_GREY);

        TerrainMesh.computePerVertexNormals();
        m_Terrain.init(&TerrainMesh);
        TerrainMesh.clear();
        Eigen::Quaternionf quat;
        quat = AngleAxisf(3.1415f, Eigen::Vector3f::UnitY());
        m_TerrainTransform.init(&m_RootSGN, Vector3f(0.0f, 0.0f, 0.0f), quat);
        m_TerrainGeo.init(&m_TerrainTransform, &m_Terrain);

        navMesh = loadNavMesh("Assets/Navmesh/solo_navmesh.bin");
        dtQueryFilter *filter = new dtQueryFilter();

        Eigen::Vector3f startPos(-15, 50, -15);
        Eigen::Vector3f endPos(30, 5, 30);
        Eigen::Vector3f search(5, 5, 5);
        Eigen::Vector3f nearest(0, 0, 0);
        dtPolyRef start;
        dtPolyRef end;

        dtNavMeshQuery query;
        query.init(navMesh, 16000);
        query.findNearestPoly(startPos.data(), search.data(), filter, &start, nearest.data());
        cout << "Ref: " << start << " Nearest: " << nearest.x() << " " << nearest.y() << " " << nearest.z() << endl;
        query.findNearestPoly(endPos.data(), search.data(), filter, &end, nearest.data());
        cout << "Ref: " << end << " Nearest: " << nearest.x() << " " << nearest.y() << " " << nearest.z() << endl;

        int maxPath = 100;
        int pathCount = 0;
        dtPolyRef *path = new dtPolyRef[maxPath];


        query.findPath(start, end, startPos.data(), endPos.data(), filter, path, &pathCount, maxPath);
        cout << "PathCount: " << pathCount << endl;

        float *vec = new float[maxPath * 3];
        dtStraightPathFlags flags;
        dtStraightPathOptions options = DT_STRAIGHTPATH_ALL_CROSSINGS;
        int outputPathCount;

        SAssetIO::load("Assets/placeholder/zylinder.glb", &TerrainMesh);
        query.findStraightPath(startPos.data(), endPos.data(), path, pathCount, vec, 0, 0, &outputPathCount,
                               maxPath, options);

        cout << "ActualPath: " << outputPathCount << endl;

        TerrainMesh.computePerVertexNormals();
        StaticActor *actor1 = new StaticActor();
        actor1->init(&TerrainMesh);
        TerrainMesh.clear();
        for (int i = 0; i < outputPathCount; i++) {
            SGNTransformation *pos = new SGNTransformation();
            SGNGeometry *geo = new SGNGeometry();
            pos->init(&m_RootSGN, Vector3f(vec[i * 3], vec[i * 3 + 1], vec[i * 3 + 2]));
            geo->init(pos, actor1);
        }
    }

    void ExampleRecast::mainLoop() {
        m_RenderWin.update();
        m_SG.update(60.0f / m_FPS);

        defaultCameraUpdate(&m_Cam, m_RenderWin.keyboard(), m_RenderWin.mouse());

        m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
        m_RenderDev.activeCamera(const_cast<VirtualCamera *>(m_Sun.camera()));
        m_SG.render(&m_RenderDev);

        m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
        m_RenderDev.activeCamera(&m_Cam);
        m_SG.render(&m_RenderDev);

        m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);
        if (m_FPSLabelActive) m_FPSLabel.render(&m_RenderDev);
        if (m_DrawHelpTexts) drawHelpTexts();

        m_RenderWin.swapBuffers();

        updateFPS();

        defaultKeyboardUpdate(m_RenderWin.keyboard());
    }

    dtNavMesh *ExampleRecast::loadNavMesh(const char *path) {
        FILE *fd = fopen(path, "rb");
        if (!fd) throw CForgeExcept("Can't open file");

        try {
            NavMeshSetHeader header;
            size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fd);

            if (
                    readLen != 1 ||
                    header.magic != NAVMESHSET_MAGIC ||
                    header.version != NAVMESHSET_VERSION
                    )
                throw runtime_error("Header Error");

            dtNavMesh *navMesh = dtAllocNavMesh();
            if (!navMesh)
                throw bad_alloc();

            dtStatus status = navMesh->init(&header.params);
            if (dtStatusFailed(status))
                throw runtime_error("Init Error");

            for (unsigned i = 0; i < header.numTiles; i++) {
                NavMeshTileHeader tileHeader;
                readLen = fread(&tileHeader, sizeof(tileHeader), 1, fd);

                if (readLen != 1)
                    throw runtime_error("Tile Header Error");

                if (!tileHeader.tileRef || !tileHeader.dataSize)
                    break;

                unsigned char *data = (unsigned char *) dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
                if (!data)
                    throw bad_alloc();
                readLen = fread(data, tileHeader.dataSize, 1, fd);
                if (readLen != 1) {
                    dtFree(data);
                    throw runtime_error("Insufficient tiles in provided file");
                }

                navMesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
            }
            fclose(fd);
            return navMesh;
        } catch (const exception &e) {
            fclose(fd);
            cout << e.what() << endl;
            throw runtime_error("Something went wrong");
        }
    }

}
#endif
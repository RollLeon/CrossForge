#ifndef __CFORGE_EXAMPLERECAST_HPP__
#define __CFORGE_EXAMPLERECAST_HPP__

#include <iostream>
#include <DetourNavMesh.h>
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

    static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
    static const int NAVMESHSET_VERSION = 1;

    class ExampleRecast : public ExampleSceneBase {
    protected:
        SGNTransformation m_RootSGN;
        StaticActor m_Terrain;
        SGNGeometry m_TerrainGeo;
        SGNTransformation m_TerrainTransform;
        StaticActor m_NavMesh;
        SGNGeometry m_NavMeshGeo;
        SGNTransformation m_NavmeshTransform;
        // dtNavMeshQuery m_NavMeshQuery;
        void loadNavMesh(const char *path);
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

        SAssetIO::load("Assets/ExampleScenes/Recast/Terrain.gltf", &TerrainMesh);
        for (uint32_t i = 0; i < TerrainMesh.materialCount(); ++i)
            CForgeUtility::defaultMaterial(TerrainMesh.getMaterial(i), CForgeUtility::STONE_GREY);

        TerrainMesh.computePerVertexNormals();
        m_Terrain.init(&TerrainMesh);
        TerrainMesh.clear();

        m_TerrainTransform.init(&m_RootSGN, Vector3f(0.0f, 0.0f, 0.0f));
        m_TerrainGeo.init(&m_TerrainTransform, &m_Terrain);

        loadNavMesh("solo_navmesh.bin");



        m_NavmeshTransform.init(&m_TerrainTransform, Vector3f(0.0f, 0.0f, 0.0f));
        m_NavMeshGeo.init(&m_NavmeshTransform, &m_NavMesh);
    }

    void ExampleRecast::mainLoop() {
        m_RenderWin.update();
        m_SG.update(60.0f / m_FPS);

        defaultCameraUpdate(&m_Cam, m_RenderWin.keyboard(), m_RenderWin.mouse());

        m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
        m_RenderDev.activeCamera(const_cast<VirtualCamera*>(m_Sun.camera()));
        m_SG.render(&m_RenderDev);

        m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
        m_RenderDev.activeCamera(&m_Cam);
        m_SG.render(&m_RenderDev);

        m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);
        if(m_FPSLabelActive) m_FPSLabel.render(&m_RenderDev);
        if (m_DrawHelpTexts) drawHelpTexts();

        m_RenderWin.swapBuffers();

        updateFPS();

        defaultKeyboardUpdate(m_RenderWin.keyboard());
    }

    void ExampleRecast::loadNavMesh(const char *path) {
        FILE* fd = fopen(path, "rb");
        if (!fd) return;

        try {
            NavMeshSetHeader header;
            size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fd);

            if (
                    readLen != 1 ||
                    header.magic != NAVMESHSET_MAGIC ||
                    header.version != NAVMESHSET_VERSION
                    )
                throw runtime_error("Header Error");

            dtNavMesh* navMesh = dtAllocNavMesh();
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

                unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
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

        } catch(const exception &e) {
            fclose(fd);
            cout << e.what() << endl;
            return;
        }
    }

}
#endif
//
// Created by Linus on 29.06.23.
//

#ifndef CFORGESANDBOX_PATHSYSTEM_H
#define CFORGESANDBOX_PATHSYSTEM_H

#include <flecs.h>
#include <iostream>
#include <DetourNavMesh.h>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "SteeringSystem.h"
#include "Components.h"
#include "Examples/ExampleRecast.hpp"

namespace CForge {

    class PathSystem {
    public:
        void addPathSystem(flecs::world &world) {

            auto navMesh = loadNavMesh("Assets/Navmesh/solo_navmesh.bin");
            auto filter = new dtQueryFilter();

            world.system<PathRequestComponent>("PathSystem")
                    .iter([navMesh, filter](flecs::iter it, PathRequestComponent *p) {
                        for (auto i: it) {
                            flecs::entity e = it.entity(i);
                            auto startPos = p[i].start;
                            auto destination = p[i].destination;
                            e.remove<PathRequestComponent>();
                            e.add<PathComponent>();
                            auto pathComponent = e.get_mut<PathComponent>();
                            findPath(navMesh, filter, startPos, destination, pathComponent->path);
                        }
                    });
            this->filter=filter;
            this->navMesh=navMesh;
        }
        ~PathSystem(){
            delete filter;
            dtFreeNavMesh(navMesh);
        }
    private:
        dtNavMesh *navMesh;
        dtQueryFilter *filter;
        static void
        findPath(dtNavMesh *navMesh, dtQueryFilter *filter, Eigen::Vector3f startPos, Eigen::Vector3f endPos,
                 std::queue<Eigen::Vector3f> &target) {
            std::cout << "Start: " << startPos.x() << " " << startPos.y() << " " << startPos.z() << std::endl;
            std::cout << "end: " << endPos.x() << " " << endPos.y() << " " << endPos.z() << std::endl;

            Eigen::Quaternionf rotate180;
            rotate180 = AngleAxisf(0/*3.141592f*/, Eigen::Vector3f::UnitY());
            startPos = rotate180 * startPos;
            endPos = rotate180 * endPos;

            Eigen::Vector3f search(5, 5, 5);
            Eigen::Vector3f nearest(0, 0, 0);
            dtPolyRef start;
            dtPolyRef end;

            dtNavMeshQuery query;
            query.init(navMesh, 16000);
            query.findNearestPoly(startPos.data(), search.data(), filter, &start, nearest.data());
            query.findNearestPoly(endPos.data(), search.data(), filter, &end, nearest.data());

            int maxPath = 100;
            int pathCount = 0;
            dtPolyRef *path = new dtPolyRef[maxPath];


            query.findPath(start, end, startPos.data(), endPos.data(), filter, path, &pathCount, maxPath);

            float *vec = new float[maxPath * 3];
            dtStraightPathFlags flags;
            dtStraightPathOptions options = DT_STRAIGHTPATH_ALL_CROSSINGS;
            int outputPathCount;

            query.findStraightPath(startPos.data(), endPos.data(), path, pathCount, vec, 0, 0, &outputPathCount,
                                   maxPath, options);

            cout << "ActualPath: " << outputPathCount << endl;
            // clear queue
            target = {};
            for (int i = 0; i < outputPathCount; i++) {
                target.push(rotate180 * Eigen::Vector3f(vec[i * 3], vec[i * 3 + 1], vec[i * 3 + 2]));
            }
            target.push(rotate180 * endPos);
            delete[] path;
            delete[] vec;
        }

        static dtNavMesh *loadNavMesh(const char *path) {
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

    };

} // CForge

#endif //CFORGESANDBOX_PATHSYSTEM_H

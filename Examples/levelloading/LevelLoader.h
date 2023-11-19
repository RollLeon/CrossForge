//
// Created by private on 27.07.23.
//

#ifndef CFORGESANDBOX_LEVELLOADER_H
#define CFORGESANDBOX_LEVELLOADER_H

#include <string>
#include <flecs/addons/cpp/world.hpp>
#include <json/reader.h>
#include <json/value.h>
#include <iostream>
#include <regex>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include "crossforge/AssetIO/T3DMesh.hpp"
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "crossforge/AssetIO/SAssetIO.h"
#include "crossforge/Graphics/SceneGraph/SGNGeometry.h"
#include "crossforge/Graphics/Actors/StaticActor.h"
#include "Examples/edt/PhysicsComponent.h"

namespace CForge {
    class LevelLoader {
    public:
        void loadLevel(std::string filePath, SGNTransformation *rootNode, flecs::world *world) {
            std::string content = SAssetIO::readTextFile(filePath);
            Json::Reader reader;
            Json::Value level;
            reader.parse(content, level);
            const Json::Value &entities = level["entities"];
            for (int i = 0; i < entities.size(); i++) {
                Eigen::Vector3f position(entities[i]["position"]["x"].asFloat(),
                                         entities[i]["position"]["y"].asFloat(),
                                         entities[i]["position"]["z"].asFloat());
                Eigen::Quaternionf rotation;
                rotation = Eigen::AngleAxisf(entities[i]["rotation"]["x"].asFloat(), Eigen::Vector3f::UnitX()) *
                           Eigen::AngleAxisf(entities[i]["rotation"]["y"].asFloat(), Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(entities[i]["rotation"]["z"].asFloat(), Eigen::Vector3f::UnitZ());
                Eigen::Vector3f scale(entities[i]["scale"]["x"].asFloat(),
                                      entities[i]["scale"]["y"].asFloat(),
                                      entities[i]["scale"]["z"].asFloat());
                auto entity = world->entity();
                entity.add<PositionComponent>();
                entity.add<GeometryComponent>();
                PositionComponent *entityPosition = entity.get_mut<PositionComponent>();
                entityPosition->init();
                entityPosition->rotation(rotation);
                entityPosition->scale(scale);
                entityPosition->translation(position);
                GeometryComponent *obstacle_geom = entity.get_mut<GeometryComponent>();
                obstacle_geom->init(getStaticActor("Assets/Models/" + entities[i]["path"].asString()));

                initEntityWithType(entity, entities[i]["name"].asString());
            }
            //load static geometry
            SGNTransformation *static_geom_position = new SGNTransformation();
            static_geom_position->init(rootNode);
            SGNGeometry *static_geom = new SGNGeometry();
            std::string static_mesh_path = std::regex_replace(filePath, std::regex("json"), "gltf");
            loadStaticCollisionMesh(world, static_mesh_path);
            static_geom->init(static_geom_position, getStaticActor(static_mesh_path));
        }

        btVector3 btVec(Eigen::Vector3f vec) {
            return btVector3(vec.x(), vec.y(), vec.z());
        }

        void copyTriangles(btTriangleMesh *btMesh, T3DMesh<float> &mesh, T3DMesh<float>::Submesh *subMesh) {
            for (auto face: subMesh->Faces) {
                btMesh->addTriangle(
                        btVec(mesh.vertex(face.Vertices[0])),
                        btVec(mesh.vertex(face.Vertices[1])),
                        btVec(mesh.vertex(face.Vertices[2]))
                );
            }
            for (auto sm: subMesh->Children) {
                copyTriangles(btMesh, mesh, sm);
            }
        }

        void loadStaticCollisionMesh(flecs::world *world, std::string filePath) {
            auto striding = new btTriangleMesh();

            T3DMesh<float> mesh = loadMesh(filePath);
            for (int i = 0; i < mesh.submeshCount(); ++i) {
                auto sm = mesh.getSubmesh(i);
                copyTriangles(striding, mesh, sm);
            }

            btCollisionShape *map = new btBvhTriangleMeshShape(striding, true);

            btTransform groundTransform;
            groundTransform.setIdentity();
            btRigidBody::btRigidBodyConstructionInfo rbInfo(0, new btDefaultMotionState(), map);
            btRigidBody *body = new btRigidBody(rbInfo);
            auto staticSzeneEntity = world->entity();
            staticSzeneEntity.emplace<PhysicsComponent>(body);
        }

        T3DMesh<float> loadMesh(std::string filePath) {
            T3DMesh<float> M;
            SAssetIO::load(filePath, &M);
            setMeshShader(&M);
            M.computePerVertexNormals();
            M.computeAxisAlignedBoundingBox();
            return M;
        }

        StaticActor *getStaticActor(std::string filePath) {
            if (models.find(filePath) != models.end()) {
                return models.find(filePath)->second;
            }
            auto M = loadMesh(filePath);
            StaticActor *actor = new StaticActor();
            actor->init(&M);
            M.clear();
            models.insert({filePath, actor});
            return actor;
        }


    protected:
        std::map<std::string, StaticActor *> models;

        static void setMeshShader(T3DMesh<float> *pM) {
            for (uint32_t i = 0; i < pM->materialCount(); ++i) {
                T3DMesh<float>::Material *pMat = pM->getMaterial(i);

                pMat->VertexShaderGeometryPass.push_back("Shader/BasicGeometryPass.vert");
                pMat->FragmentShaderGeometryPass.push_back("Shader/BasicGeometryPass.frag");

                pMat->VertexShaderShadowPass.push_back("Shader/ShadowPassShader.vert");
                pMat->FragmentShaderShadowPass.push_back("Shader/ShadowPassShader.frag");

                pMat->VertexShaderForwardPass.push_back("Shader/ForwardPassPBS.vert");
                pMat->FragmentShaderForwardPass.push_back("Shader/ForwardPassPBS.frag");
            }
        }

        void initEntityWithType(flecs::entity &entity, string name) {
            if (name.find("robot") != std::string::npos) {
                entity.set_name(name.c_str());
                entity.add<SteeringComponent>();
                entity.add<AIComponent>();

                auto steering = entity.get_mut<SteeringComponent>();
                steering->securityDistance = 1;
                steering->mass = 500;
                steering->max_force = 36;
                steering->max_speed = 3;

                btCollisionShape *groundShape = new btBoxShape(btVector3(3, 3, 3));

                btTransform groundTransform;
                groundTransform.setIdentity();
                groundTransform.setOrigin(btVector3(0, 56, 0));

                btScalar mass(1.);

                //rigidbody is dynamic if and only if mass is non zero, otherwise static
                bool isDynamic = (mass != 0.f);

                btVector3 localInertia(0, 0, 0);
                if (isDynamic)
                    groundShape->calculateLocalInertia(mass, localInertia);

                //using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
                btDefaultMotionState *myMotionState = new btDefaultMotionState(groundTransform);
                btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
                btRigidBody *body = new btRigidBody(rbInfo);
                entity.emplace<PhysicsComponent>(body);

                auto aic = entity.get_mut<AIComponent>();
                for (int i = 0; i < 10; i++) {
                    aic->path.push(Eigen::Vector3f(-10, 0, 1));
                    aic->path.push(Eigen::Vector3f(10, 0, -1));
                }
            }
        }

    };
}
#endif //CFORGESANDBOX_LEVELLOADER_H

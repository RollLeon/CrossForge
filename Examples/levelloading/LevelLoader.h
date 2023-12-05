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
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include "crossforge/AssetIO/T3DMesh.hpp"
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "crossforge/AssetIO/SAssetIO.h"
#include "crossforge/Graphics/SceneGraph/SGNGeometry.h"
#include "crossforge/Graphics/Actors/StaticActor.h"
#include "Examples/edt/Components.h"
#include "Examples/edt/RobotActionNodes.h"
#include "behaviortree_cpp/bt_factory.h"


namespace CForge {
    class LevelLoader {
    public:
        void loadLevel(std::string filePath, SGNTransformation *rootNode, flecs::world *world) {
            std::string content = SAssetIO::readTextFile(filePath);
            Json::Reader reader;
            srand(10);
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

                initEntityWithType(entity, entities[i]["name"].asString(), world);
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

        static btCollisionShape *createCapsuleCollider(float radius, float height) {
            btCompoundShape *pCompoundShape = new btCompoundShape();
            btCollisionShape *cylinderShape = new btCapsuleShape(radius, height - 2 * radius);
            auto transform = btTransform();
            transform.setIdentity();
            transform.setOrigin(btVector3(0, height / 2.0f, 0));
            pCompoundShape->addChildShape(transform, cylinderShape);
            return pCompoundShape;
        }

        static btCollisionShape *createCylinderCollider(float radius, float height) {
            btCompoundShape *pCompoundShape = new btCompoundShape();
            btCollisionShape *cylinderShape = new btCylinderShape(btVector3(radius, height / 2.0f, radius));
            auto transform = btTransform();
            transform.setIdentity();
            transform.setOrigin(btVector3(0, height / 2.0f, 0));
            pCompoundShape->addChildShape(transform, cylinderShape);
            return pCompoundShape;
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

        static void initEntityWithType(flecs::entity &entity, string name, flecs::world *world) {
            if (name.find("robot") != std::string::npos) {
                entity.set_name(name.c_str());
                entity.add<SteeringComponent>();
                entity.add<PathComponent>();
                entity.add<AIComponent>();

                BT::BehaviorTreeFactory factory;

                // The recommended way to create a Node is through inheritance.
                factory.registerNodeType<FindPlant>("FindPlant");
                factory.registerNodeType<FindWay>("FindWay");
                factory.registerNodeType<DriveToPlant>("DriveToPlant");
                factory.registerNodeType<Watering>("Watering");
                auto aic = entity.get_mut<AIComponent>();
                aic->tree = factory.createTreeFromText(
                        SAssetIO::readTextFile("Assets/Behaviors/WateringBehaviorTree.xml"));
                // visitor will initialize the instances of
                auto visitor = [entity, world](BT::TreeNode *node) mutable {
                    if (auto action_B_node = dynamic_cast<EntityAwareNode *>(node)) {
                        action_B_node->initialize(entity, world);
                    }
                };


                // Apply the visitor to ALL the nodes of the tree
                aic->tree.applyVisitor(visitor);

                auto steering = entity.get_mut<SteeringComponent>();
                steering->securityDistance = 1;
                steering->mass = 500;
                steering->max_force = 36;
                steering->max_speed = 3;


                btRigidBody::btRigidBodyConstructionInfo rbInfo(steering->mass, new btDefaultMotionState(),
                                                                createCapsuleCollider(1.5f, 5.0f));
                btRigidBody *body = new btRigidBody(rbInfo);
                entity.emplace<PhysicsComponent>(body);

            } else if (name.find("monstera") != std::string::npos || name.find("small_plant") != std::string::npos) {
                entity.set_name(name.c_str());
                entity.add<PlantComponent>();
                entity.add<ObstacleComponent>();

                auto plant = entity.get_mut<PlantComponent>();
                float LO = 1.0;
                float HI = 10.0;
                float random = LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HI - LO)));
                plant->waterLevel = random;
                btRigidBody::btRigidBodyConstructionInfo rbInfo(10, new btDefaultMotionState(),
                                                                createCylinderCollider(0.5f, 1.0f));
                btRigidBody *body = new btRigidBody(rbInfo);
                entity.emplace<PhysicsComponent>(body);
            }
        }

    };
}
#endif //CFORGESANDBOX_LEVELLOADER_H

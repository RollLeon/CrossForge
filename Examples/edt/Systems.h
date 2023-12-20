#ifndef CFORGESANDBOX_SYSTEMS_H
#define CFORGESANDBOX_SYSTEMS_H

#include <flecs.h>
#include <iostream>
#include "Components.h"
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include "crossforge/Graphics/Actors/StaticActor.h"
#include "crossforge/AssetIO/SAssetIO.h"


namespace CForge {

    class Systems {
    public:
        std::map<std::string, StaticActor*> models;

        static void setMeshShader(T3DMesh<float>* pM) {
            for (uint32_t i = 0; i < pM->materialCount(); ++i) {
                T3DMesh<float>::Material* pMat = pM->getMaterial(i);

                pMat->VertexShaderGeometryPass.push_back("Shader/BasicGeometryPass.vert");
                pMat->FragmentShaderGeometryPass.push_back("Shader/BasicGeometryPass.frag");

                pMat->VertexShaderShadowPass.push_back("Shader/ShadowPassShader.vert");
                pMat->FragmentShaderShadowPass.push_back("Shader/ShadowPassShader.frag");

                pMat->VertexShaderForwardPass.push_back("Shader/ForwardPassPBS.vert");
                pMat->FragmentShaderForwardPass.push_back("Shader/ForwardPassPBS.frag");
            }
        }

        T3DMesh<float> loadMesh(std::string filePath) {
            T3DMesh<float> M;
            SAssetIO::load(filePath, &M);
            setMeshShader(&M);
            M.computePerVertexNormals();
            M.computeAxisAlignedBoundingBox();
            return M;
        }

        StaticActor* getStaticActor(std::string filePath) {
            if (models.find(filePath) != models.end()) {
                return models.find(filePath)->second;
            }
            auto M = loadMesh(filePath);
            StaticActor* actor = new StaticActor();
            actor->init(&M);
            M.clear();
            models.insert({ filePath, actor });
            return actor;
        }

        static void addSimpleSystems(flecs::world &world) {
            world.system<AIComponent>("AISystem")
                    .iter([&world](flecs::iter it, AIComponent *ai) {
                        for (int i: it) {
                            ai[i].tree.tickExactlyOnce();
                        }
                    });

            float waterDecreaseRate = 0.1;
            world.system<PlantComponent>("PlantSystem")
                    .iter([waterDecreaseRate](flecs::iter it, PlantComponent *p) {
                        for (int i: it) {
                            if (p[i].waterLevel > 0) {
                                p[i].waterLevel -= waterDecreaseRate * it.delta_time();
                            } else {
                                p[i].waterLevel = 0; // Ensure the water level doesn't go negative
                            }
                        }
                    });
            
            world.system<ParticleComponent>("ParticleRemoverSystem")
                    .iter([&world](flecs::iter it, ParticleComponent* pc) {
                        for (int i : it) {
                            ParticleComponent* particleComp = it.entity(i).get_mut<ParticleComponent>();
                            if (particleComp->lifeTime > 0.1){
                                particleComp->lifeTime -= 0.1;
                            }
                            else{
                                it.entity(i).destruct();
                                std::cout << "Deleting "  << particleComp->lifeTime<< std::endl;
                            }
                        }
                    });

            world.system<EmitterComponent, PositionComponent>("WaterEmitterSystem")
                    .iter([&world](flecs::iter it, EmitterComponent* em, PositionComponent* pc) {
                        for (int i : it) {


                            for (int i = 0; i < em[i].numParticles; i++) {

                                auto entity = world.entity();
                                entity.add<PositionComponent>();
                                entity.add<GeometryComponent>();
                                entity.add<ParticleComponent>();
                                auto particle = entity.get_mut<ParticleComponent>();
                                particle->lifeTime = 3.0;
                                std::cout << "New Entity " << entity.id() << std::endl;
                                
                                PositionComponent* entityPosition = entity.get_mut<PositionComponent>();
                                entityPosition->init();
                                   
                                GeometryComponent* obstacle_geom = entity.get_mut<GeometryComponent>();
                                std::string path = "CrossForge/Assets/Models/Drop.gltf";
                                obstacle_geom->init(getStaticActor(path)); 

                                
                                entityPosition->translation(em[i].relativePosition + pc[i].m_Translation);
                            }
                        }
                    });
                    
        }
        
    };

    

} // CForge

#endif //CFORGESANDBOX_SYSTEMS_H
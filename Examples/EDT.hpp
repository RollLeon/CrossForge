/*****************************************************************************\
*                                                                           *
* File(s): EDT.hpp                                            *
*                                                                           *
* Content: Example scene that shows how to use the scene graph to create    *
*          dynamic scene descriptions.                                      *
*                                                                           *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_EDT_HPP__
#define __CFORGE_EDT_HPP__

#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>
#include "ExampleSceneBase.hpp"
#include "Examples/edt/PathSystem.h"
#include "Examples/levelloading/LevelLoader.h"
#include <flecs.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_glfw.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include "DialogGraph.hpp"
#include "Examples/edt/PhysicsSystem.h"
#include <fstream>
#include <json/json.h>
#include <tinyfsm.hpp>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include "Examples/edt/Components.h"
#include "Examples/edt/Systems.h"
#include "Examples/edt/PlayerSystem.h"

namespace CForge {
    class EDT : public ExampleSceneBase {
    public:
        EDT(void) {

        }//Constructor

        ~EDT(void) {
            clear();
        }//Destructor

        void init(void) override {
            initWindowAndRenderDevice();
            initCameraAndLights();

            initSkybox();
            initFPSLabel();

            m_FPSLabel.color(1.0f, 1.0f, 1.0f, 1.0f);

            m_RootSGN.init(nullptr);
            m_SG.rootNode(&m_RootSGN);

            // initialize ground plane
            T3DMesh<float> M;

            // load the ground model
            //SAssetIO::load("Assets/ExampleScenes/TexturedGround.gltf", &M);
            PrimitiveShapeFactory::plane(&M, Vector2f(1250.0f, 1250.0f), Vector2i(10, 10));
            setMeshShader(&M, 0.6f, 0.2f);
            M.changeUVTiling(Vector3f(250.0f, 250.0f, 1.0f));
            M.computePerVertexNormals();
            M.computePerVertexTangents();
            M.getMaterial(0)->TexAlbedo = "Assets/ExampleScenes/Textures/Ground003/Ground003_2K_Color.webp";
            M.getMaterial(0)->TexNormal = "Assets/ExampleScenes/Textures/Ground003/Ground003_2K_NormalGL.webp";
            m_Ground.init(&M);
            BoundingVolume BV;
            m_Ground.boundingVolume(BV);
            M.clear();

            SAssetIO::load("Assets/ExampleScenes/Duck/Duck.gltf", &M);
            for (uint32_t i = 0; i < M.materialCount(); ++i)
                CForgeUtility::defaultMaterial(M.getMaterial(i), CForgeUtility::PLASTIC_YELLOW);
            M.computePerVertexNormals();
            m_duck.init(&M);
            M.clear();

            // initialize ground transformation and geometry scene graph node
            m_GroundTransformSGN.init(&m_RootSGN);
            m_GroundSGN.init(&m_GroundTransformSGN, &m_Ground);
            debugDraw = new DebugDraw();
            dynamicsWorld = PhysicsSystem::addPhysicsSystem(world);
            dynamicsWorld->setDebugDrawer(debugDraw);
            SteeringSystem::addSteeringSystem(world);
            PathSystem::addPathSystem(world);
            PlayerSystem::addPlayerSystem(world);
            Systems::addSimpleSystems(world);
            // load level
            LevelLoader levelLoader;
            levelLoader.loadLevel("Assets/Scene/end_mvp.json", &m_RootSGN, &world);

            flecs::entity player = world.entity();
            player.emplace<PlayerComponent>(&m_Cam, m_RenderWin.keyboard(), m_RenderWin.mouse());

            btRigidBody::btRigidBodyConstructionInfo rbInfo(10, new btDefaultMotionState(),
                                                            LevelLoader::createCapsuleCollider(0.5f,
                                                                                               PlayerComponent::HEIGHT));
            btRigidBody *body = new btRigidBody(rbInfo);
            player.emplace<PhysicsComponent>(body);
            player.add<PositionComponent>();
            auto pos = player.get_mut<PositionComponent>();
            pos->init();
            pos->translation(Vector3f(15, 4, 0));

            // change sun settings to cover this large area
            m_Sun.position(Vector3f(100.0f, 1000.0f, 500.0f));
            m_Sun.initShadowCasting(2048 * 2, 2048 * 2, Vector2i(1000, 1000), 1.0f, 5000.0f);

            // create help text
            LineOfText *pKeybindings = new LineOfText();
            pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18),
                               "Movement:(Shift) + W,A,S,D  | Rotation: LMB/RMB + Mouse | F1: Toggle help text");
            m_HelpTexts.push_back(pKeybindings);
            pKeybindings->color(0.0f, 0.0f, 0.0f, 1.0f);
            m_DrawHelpTexts = true;

            IMGUI_CHECKVERSION();
            ImGui::CreateContext();

            ImGuiIO &io = ImGui::GetIO();
            (void) io;

            io.Fonts->AddFontFromFileTTF(
                    "Assets/Fonts/NotoSerif/NotoSerif-Regular.ttf",
                    24.0f,
                    NULL,
                    NULL
            );

            // setup platform/renderer bindings
            if (!ImGui_ImplGlfw_InitForOpenGL(glfwGetCurrentContext(), true)) {
                std::cout << "Failed to init imGUI for window" << std::endl;
            }
            if (!ImGui_ImplOpenGL3_Init()) {
                std::cout << "Failed to init imGUI for OpenGL" << std::endl;
            }

            dialog.init("Assets/Dialogs/conversation.json");
        }//initialize

        void clear(void) override {
            for (auto &i: m_TreeSGNs) if (nullptr != i) delete i;
            for (auto &i: m_TreeTransformSGNs) if (nullptr != i) delete i;

            ExampleSceneBase::clear();
        }//clear

        void mainLoop(void) override {
            m_RenderWin.update();

            m_SkyboxSG.update(60.0f / m_FPS);
            m_SG.update(60.0f / m_FPS);

            m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
            m_RenderDev.activeCamera(const_cast<VirtualCamera *>(m_Sun.camera()));
            m_SG.render(&m_RenderDev);
            renderEntities(&m_RenderDev);

            m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
            m_RenderDev.activeCamera(&m_Cam);
            m_SG.render(&m_RenderDev);
            renderEntities(&m_RenderDev);

            m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);

            m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
            m_SkyboxSG.render(&m_RenderDev);
            if (m_FPSLabelActive) m_FPSLabel.render(&m_RenderDev);
            if (m_DrawHelpTexts) drawHelpTexts();

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();

            bool test = true;
            ImVec2 size = {0, 0};

            debugDraw->updateUniform(m_Cam.projectionMatrix(), m_Cam.cameraMatrix());
            dynamicsWorld->debugDrawWorld();
            m_RenderWin.swapBuffers();

            updateFPS();
            world.progress(1.0f / m_FPS);
            // change between flying and walking mode
            defaultKeyboardUpdate(m_RenderWin.keyboard());

        }//run

    protected:

        void renderEntities(RenderDevice *pRDev) {
            IRenderableActor *duckCopy = &m_duck;
            world.query<PositionComponent, GeometryComponent>()
                    .iter([pRDev](flecs::iter it, PositionComponent *p, GeometryComponent *geo) {
                        for (int i: it) {
                            pRDev->requestRendering(geo[i].actor, p[i].m_Rotation, p[i].m_Translation, p[i].m_Scale);
                        }
                    });
            world.query<PathComponent>()
                    .iter([pRDev, duckCopy](flecs::iter it, PathComponent *p) {
                        for (int i: it) {
                            auto copy = p[i].path;
                            while (!copy.empty()) {
                                auto pos = copy.front();
                                copy.pop();
                                pRDev->requestRendering(duckCopy, Quaternionf(), pos,
                                                        Vector3f(0.003, 0.003, 0.003));
                            }
                        }
                    });
        }


        flecs::world world;
        SGNTransformation m_RootSGN;

        StaticActor m_Ground;
        StaticActor m_duck;
        SGNGeometry m_GroundSGN;
        SGNTransformation m_GroundTransformSGN;

        std::vector<SGNTransformation *> m_TreeTransformSGNs;
        std::vector<SGNGeometry *> m_TreeSGNs;

        SGNTransformation m_TreeGroupSGN;

        Dialoggraph dialog;
        vector<int> conversationProgress;
        DebugDraw *debugDraw;
        std::shared_ptr<btDiscreteDynamicsWorld> dynamicsWorld;
    };//EDT

}//name space

#endif 
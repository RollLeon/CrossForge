//
// Created by private on 04.12.23.
//

#ifndef CFORGESANDBOX_DEBUGDRAW_H
#define CFORGESANDBOX_DEBUGDRAW_H

#include <LinearMath/btIDebugDraw.h>
#include <vector>
#include <glad/glad.h>
#include <iostream>
#include "crossforge/Graphics/Shader/GLShader.h"
#include "crossforge/Graphics/GLVertexArray.h"
#include "crossforge/Graphics/Shader/SShaderManager.h"

namespace CForge {
    class DebugDraw : public btIDebugDraw {
    public:
        DebugDraw() {
            shader.init();
            shader.addVertexShader("#version 330 core\n"
                                   "\n"
                                   "uniform mat4 ViewMatrix;\n"
                                   "uniform mat4 ProjectionMatrix;\n"
                                   "\n"
                                   "uniform vec3 lineStart;\n"
                                   "uniform vec3 lineEnd;\n"
                                   "\n"
                                   "void main(){\n"
                                   "    vec3 Position = lineStart;\n"
                                   "    if(gl_VertexID == 1){\n"
                                   "        Position = lineEnd;\n"
                                   "    }\n"
                                   "\tgl_Position = ProjectionMatrix * ViewMatrix * vec4(Position, 1.0);\n"
                                   "}");
            shader.addFragmentShader("#version 330 core \n"
                                     "out vec4 col;\n"
                                     "void main(){\n"
                                     "col = vec4(vec3(1,0,0),1);\n"
                                     "}");
            std::string ErrorLog;
            shader.build(&ErrorLog);

            if (!ErrorLog.empty()) {
                std::cout << ("Shader compilation failed: " + ErrorLog) << std::endl;
            }

            camLocation = shader.uniformLocation("ViewMatrix");
            projLocation = shader.uniformLocation("ProjectionMatrix");
            lineStartLocation = shader.uniformLocation("lineStart");
            lineEndLocation = shader.uniformLocation("lineEnd");
            dummyVao.init();
        }

        void updateUniform(Eigen::Matrix4f proj, Eigen::Matrix4f cam) {
            shader.bind();
            glUniformMatrix4fv(camLocation, 1, GL_FALSE, cam.data());
            glUniformMatrix4fv(projLocation, 1, GL_FALSE, proj.data());
            shader.unbind();
        }

        void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color) override {
            dummyVao.bind();
            shader.bind();
            glUniform3f(lineStartLocation, from.x(), from.y(), from.z());
            glUniform3f(lineEndLocation, to.x(), to.y(), to.z());
            glDrawArrays(GL_LINES, 0, 2);
            shader.unbind();
        }

        void drawContactPoint(const btVector3 &PointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime,
                              const btVector3 &color) override {}

        void reportErrorWarning(const char *warningString) override {}

        void draw3dText(const btVector3 &location, const char *textString) override {}

        void setDebugMode(int debugMode) override {}

        [[nodiscard]] int getDebugMode() const override {
            return DebugDrawModes::DBG_DrawWireframe;
        }

    private:
        GLShader shader;
        uint32_t camLocation;
        uint32_t projLocation;
        uint32_t lineStartLocation;
        uint32_t lineEndLocation;
        GLVertexArray dummyVao;
    };
}

#endif //CFORGESANDBOX_DEBUGDRAW_H

#ifndef SHADERPIPELINE_H_
#define SHADERPIPELINE_H_
#include "Shader.h"

#include <glm/gtc/type_ptr.hpp>

class ShaderPipeline{
    private:

    std::vector<Shader> shaders;
    GLint programID;
    bool link();
    void detach();
    void deleteShaders();
    public:
    ShaderPipeline(std::vector<std::string> paths, std::vector<GLenum> types);
    GLint getProgram();
    //implement later, for freeing shaders

    bool deleteProgram();
   // void modelViewProjection(Camera *c);
    
   // void sendLights(std::string name, std::vector<Light> lights);
    void sendMatrix(std::string name, glm::mat4 matrix);
    void sendFloat(std::string name, float l);
    void sendVec2(std::string name, glm::vec2 vector);
    void sendVec3(std::string name, glm::vec3 vector);
    void sendVec4(std::string name, glm::vec4 vector);
    void sendInt(std::string name, int t);


    void use();
};

#endif
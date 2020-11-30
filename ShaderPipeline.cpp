#include "ShaderPipeline.h"

ShaderPipeline::ShaderPipeline(std::vector<std::string> paths, std::vector<GLenum> types){
    assert(paths.size() == types.size());
    int counter = 0;
    Shader *shader;
    for (std::string path : paths){
        std::cout << path << std::endl;
        shader = new Shader(path, types[counter]);
        shaders.push_back(*shader);
        counter++;
    }

    link();
}


void ShaderPipeline::use(){
    glUseProgram(programID);
}

bool ShaderPipeline::link(){
    this->programID = glCreateProgram();
    for(Shader shader : shaders){
        glAttachShader(programID, shader.getShader());
    }
    glLinkProgram(programID);

    GLint linked;

    glGetProgramiv(programID, GL_LINK_STATUS, &linked);

    if(!linked){
        std::cout << "Linking failed" << std::endl;
        return false;
    }

    detach();
    deleteShaders();

    return true;
}

void ShaderPipeline::detach(){
    for(Shader shader : shaders){
        glDetachShader(programID, shader.getShader());
    }
}

void ShaderPipeline::deleteShaders(){
    for(Shader shader : shaders){
        glDeleteShader(shader.getShader());
    }
}
GLint ShaderPipeline::getProgram(){
    return programID;
}

/*void ShaderPipeline::modelViewProjection(Camera *c){
        sendMatrix("model", c->getModel());
        sendMatrix("view", c->getView());
        sendMatrix("projection", c->getProjection());
}**/

void ShaderPipeline::sendVec4(std::string name, glm::vec4 vector){
    GLint location = glGetUniformLocation(programID, name.c_str());
    // std::cout << "Vector: " << name << " location : " << location << std::endl;

    glUniform4fv(location, 1, glm::value_ptr(vector));
}


void ShaderPipeline::sendVec2(std::string name, glm::vec2 vector){
    GLint location = glGetUniformLocation(programID, name.c_str());
    // std::cout << "Vector: " << name << " location : " << location << std::endl;

    glUniform2fv(location, 1, glm::value_ptr(vector));
}

void ShaderPipeline::sendVec3(std::string name, glm::vec3 vector){
    GLint location = glGetUniformLocation(programID, name.c_str());
    // std::cout << "Vector: " << name << " location : " << location << std::endl;

    // std::cout << glm::to_string(vector) << std::endl;
    glUniform3fv(location, 1, glm::value_ptr(vector));
}

void ShaderPipeline::sendFloat(std::string name, float f){
    GLint location = glGetUniformLocation(programID, name.c_str());
    glUniform1f(location, f);
}

/*void ShaderPipeline::sendLights(std::string name, std::vector<Light> lights){
    int i = 0;
    for(Light l : lights){
        std::string nPos= name +"[" + std::to_string(i) + "].pos";
        std::string nColour = name +"[" + std::to_string(i) + "].colour";
        
        GLuint positionLocation = glGetUniformLocation(programID, nPos.c_str());
        GLuint colourLocation = glGetUniformLocation(programID, nColour.c_str());
       
        glUniform3fv(positionLocation, 1, glm::value_ptr(l.getPosition()));
        glUniform4fv(colourLocation, 1, glm::value_ptr(l.getColour()));
        i+=1;
    }

}*/
void ShaderPipeline::sendMatrix(std::string name, glm::mat4 matrix){
    GLint location = glGetUniformLocation(programID, name.c_str());
    // std::cout << "Matrix " << name << " location : " << location << std::endl;
    glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(matrix));
}

void ShaderPipeline::sendInt(std::string name, int t){
    GLint location = glGetUniformLocation(programID, name.c_str());
    // std::cout << "Matrix " << name << " location : " << location << std::endl;
    glUniform1i(location, t);
}
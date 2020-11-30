#include "Shader.h"


Shader::Shader(std::string path, GLenum shaderType){
    instantiate(path, shaderType);
}

std::string Shader::read(std::string path){
    std::string shaderCode;
    std::ifstream shader;
    shader.open(path);
    std::string line;
    while(!shader.eof()){
     std::getline(shader, line);
    // std::cout << line << std::endl;
     shaderCode.append(line + "\n"); 
    }
    shader.close();
   // std::cout << shaderCode << std::endl;
    return shaderCode;
    }


bool Shader::instantiate(std::string path, GLenum shaderType){
    shader = glCreateShader(shaderType);
    std::cout << "Shader value: " << shader << std::endl;
    std::string shaderCode = read(path);
    std::cout << "Read shaders " << std::endl;
    const char * code = shaderCode.c_str();
    glShaderSource(shader, 1, &code, NULL);
    std::cout << code << std::endl;
    glCompileShader(shader);
    GLint compiled = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    std::cout << "Compiled shader: " << compiled << std::endl;
    if(!compiled){
        generateLog();
        return false;
    }
    return true;
}

void Shader::generateLog(){
    GLsizei infoLength;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLength);


    // The maxLength includes the NULL character
    std::vector<GLchar> errorLog(infoLength);
  //  std::string infoLog;

    glGetShaderInfoLog(shader, infoLength, &infoLength, &errorLog[0]);
    if(infoLength){
      printf("Compilation failed: %s\n", errorLog[0]);
    }

}

GLuint Shader::getShader(){
    return shader;
}
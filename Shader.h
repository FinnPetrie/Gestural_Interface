#ifndef SHADER_H_
#define SHADER_H_

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include "utils.h"
class Shader{

private:
// GLchar *code;
GLuint programID;
GLuint shader;
// GLuint shader;
bool instantiate(std::string path, GLenum shaderType);
void generateLog();
std::string read(std::string path);
public:

Shader(std::string path, GLenum shaderType);
Shader();

GLuint getShader();

};

#endif
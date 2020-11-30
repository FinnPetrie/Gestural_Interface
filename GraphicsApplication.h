#pragma once
#include "utils.h"

#include <gl/GLU.h>
#include <gl/glut.h>
#include "ShaderPipeline.h"
#include "KinectSkeletonApplication.h"
struct Triangle {
	GLuint meshVAO;
	GLuint vertexBuffer;
	std::vector<glm::vec3> vertices;
};

void draw();

class GraphicsApplication
{

public:
	GraphicsApplication(int *argc, char **argv, uint32_t width, uint32_t height, KinectSkeletonApplication* skele);
	GraphicsApplication(uint32_t width, uint32_t height, KinectSkeletonApplication *skele = nullptr);
	void attachKinectData();
	void setupForKinect();
	void drawKinect();
	void run(glm::vec3 cursor,bool kinect);
	void drawTriangle(glm::vec3 position);
	void attachTriangle();
	void drawKinectData();
private:
	ShaderPipeline* shaders;
	KinectSkeletonApplication* skeleton;

	unsigned int texture1, texture2;
	unsigned int VBO, VAO, EBO;

	uint32_t width;
	uint32_t height;
	GLuint textureId;
	Triangle t;
	GLenum err;
	GLFWwindow* window;
};


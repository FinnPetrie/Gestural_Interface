#include "GraphicsApplication.h"



GraphicsApplication::GraphicsApplication(int *argc, char** argv, uint32_t width, uint32_t height, KinectSkeletonApplication* skeleton) : width(width), height(height), skeleton(skeleton) {
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(width, height);
	glutCreateWindow("Kinect");
	glutDisplayFunc(draw);
	glutIdleFunc(draw);

	attachKinectData();
}

GraphicsApplication::GraphicsApplication(uint32_t width, uint32_t height, KinectSkeletonApplication* skele) : width(width), height(height), skeleton(skele) {
	//setup window


	glfwInit();
	window = glfwCreateWindow(width, height, "standard", nullptr, nullptr);

	glfwMakeContextCurrent(window);

	err = glewInit();

	std::vector<std::string> paths = { "/Users/finn/source/repos/Gestural_Interface/shader.vert", "/Users/finn/source/repos/Gestural_Interface/shader.frag" };
	std::vector<GLenum> types = { GL_VERTEX_SHADER, GL_FRAGMENT_SHADER };
	shaders = new ShaderPipeline(paths, types);


	if (skeleton != nullptr) {
		setupForKinect();
		//attachKinectData();
	}
	else {
		attachTriangle();
	}

	
}
void GraphicsApplication::attachKinectData() {
	
	std::vector<GLubyte>* data = skeleton->getData();
	glGenTextures(1, &textureId);
	glBindTexture(GL_TEXTURE_2D, textureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height,
		0, GL_BGRA, GL_UNSIGNED_BYTE, (GLvoid*)data->data());
	glBindTexture(GL_TEXTURE_2D, 0);

	// OpenGL setup
	glClearColor(0, 0, 0, 0);
	glClearDepth(1.0f);
	glEnable(GL_TEXTURE_2D);

	// Camera setup
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, width, height, 0, 1, -1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

}

void GraphicsApplication::setupForKinect() {
	float vertices[] = {
		// positions          // colors           // texture coords
		 0.5f,  0.5f, 0.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f, // top right
		 0.5f, -0.5f, 0.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f, // bottom right
		-0.5f, -0.5f, 0.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f, // bottom left
		-0.5f,  0.5f, 0.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f  // top left 
	};
	unsigned int indices[] = {
		0, 1, 3, // first triangle
		1, 2, 3  // second triangle
	};
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// color attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	// texture coord attribute
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);

	// texture 1
	// ---------
	glGenTextures(1, &texture1);
	glBindTexture(GL_TEXTURE_2D, texture1);
	// set the texture wrapping parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	// set texture filtering parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)skeleton->getData()->data());
	glGenerateMipmap(GL_TEXTURE_2D);

	shaders->sendInt("texture1", 0);
	//shaders->sendInt("texture2", 1);
}

void GraphicsApplication::drawKinect() {
//	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
///	glClear(GL_COLOR_BUFFER_BIT);
	skeleton->getKinectData();
	glUseProgram(shaders->getProgram());
	// bind textures on corresponding texture units
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, texture1);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGRA, GL_UNSIGNED_BYTE, (GLvoid*)skeleton->getData()->data());

	//glActiveTexture(GL_TEXTURE1);
	//glBindTexture(GL_TEXTURE_2D, texture2);
	std::cout << "Doing the bind " << std::endl;
	// render container
	//ourShader.use();
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

	// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
	// -------------------------------------------------------------------------------
	glfwSwapBuffers(window);
	glfwPollEvents();
}


void GraphicsApplication::drawKinectData() {
	glBindTexture(GL_TEXTURE_2D, textureId);
	skeleton->getKinectData();
	//kinect.getKinectData();
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGRA, GL_UNSIGNED_BYTE, (GLvoid*)skeleton->getData()->data());
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(0, 0, 0);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(width, 0, 0);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(width, height, 0.0f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(0, height, 0.0f);
	glEnd();

}

void draw() {

	//drawKinectData();
	glutSwapBuffers();
}
void GraphicsApplication::run(glm::vec3 cursor, bool kinect) {
	if (GLEW_OK != err) {
		fprintf(stderr, "Error : %s'\n", glewGetErrorString(err));
	}



	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (kinect) {
		//std::cout << "Drawing " << std::endl;

		drawKinect();
	//	drawKinectData(*skeleton);
	}
	else {
		drawTriangle(cursor);
	}
	glfwSwapBuffers(window);
	glFlush();
}

void GraphicsApplication::attachTriangle() {
	
	t.vertices = { glm::vec3(0.1, 0, 0), glm::vec3(0, 0.1, 0), glm::vec3(-0.1, 0, 0) };
	glGenVertexArrays(1, &t.meshVAO);
	glBindVertexArray(t.meshVAO);

	glGenBuffers(1, &t.vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, t.vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, t.vertices.size() * sizeof(glm::vec3), &t.vertices[0].x, GL_DYNAMIC_DRAW);

	int numVertices = t.vertices.size();
	glBindBuffer(GL_ARRAY_BUFFER, t.vertexBuffer);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(0);


}

void GraphicsApplication::drawTriangle(glm::vec3 position) {
	//bind buffers, upload to GPU
	t.vertices = { glm::vec3( 0.1 + position.x, 0, 0), glm::vec3(0, 0.1 + position.y, 0), glm::vec3(-0.1+ position.x, 0, 0) };
	glBindBuffer(GL_ARRAY_BUFFER, t.vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, t.vertices.size() * sizeof(glm::vec3), &t.vertices[0].x, GL_DYNAMIC_DRAW);
	glBindVertexArray(t.meshVAO);
	//draw triangle
	glDrawArrays(GL_TRIANGLES, 0, t.vertices.size());
}
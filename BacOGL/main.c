#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/SOIL.h>
#include "Render.h"
#include <fstream>
#include <sstream>
#include <ctime>
#define WINDOW_TITLE_PREFIX "Schwarzschild"

const int RASTER_RES = 2000;

int
CurrentWidth = 800,
CurrentHeight = 600,
WindowHandle = 0;

bool leftMouseButtonActive = false;
int mousePosX = 0, mousePosY = 0;
float rotationX = 0, rotationY = 0;

unsigned FrameCount = 0;

GLuint
VertexShaderId,
FragmentShaderId,
ProgramId,
VaoId,
VboId,
ColorBufferId,
Texture,
RasterTex;


//Move to Render!!
float* mat1, *mat2, *mat3, *psiFactor, *rasterFun, *fov, *ratio;
Render* rayTracer;

double R = 10;
double FOV = M_PI_2;
double rStart = 25;
double surfaceR = 500;

bool needsReset = false;

const GLchar* VertexShader =
{
	"#version 330\n"\

	"layout(location=0) in vec4 in_Position;\n"\
	"//layout(location=1) in vec4 in_Color;\n"\
	"//out vec4 v_Color;\n"\
	"out vec2 v_screenPosition;\n"\

	"void main(void)\n"\
	"{\n"\
	"  gl_Position = in_Position;\n"\
	"  v_screenPosition = in_Position.xy;\n"\
	"  //v_Color = in_Color;\n"\
	"}\n"
};

void Initialize(int, char*[]);
void InitWindow(int, char*[]);
void ResizeFunction(int, int);
void RenderFunction(void);
void TimerFunction(int);
void IdleFunction(void);
void Cleanup(void);
void CreateVBO(void);
void DestroyVBO(void);
void CreateShaders(void);
void DestroyShaders(void);
void initRayTracer(void);
std::string loadFile(const char *fname);
void loadTexture(const char *picName);
void mouse(int button, int state, int x, int y);
void mouseMotion(int x, int y);
void keyboard(unsigned char key, int x, int y);
bool DRMCheck();

int main(int argc, char* argv[])
{
	//if (!DRMCheck())
	//	return 1;
	initRayTracer();
	Initialize(argc, argv);

	glutMainLoop();

	exit(EXIT_SUCCESS);
}

void initRayTracer()
{
	RasterFunction180::NO_VALUE = -10000.;
	rayTracer = new Render(RASTER_RES, M_PI / 100, R, surfaceR, rStart, FOV);
	mat1 = new float[9];
	mat2 = new float[9];
	mat3 = new float[9];
	rasterFun = new float[RASTER_RES*2];
	psiFactor = new float;
	*psiFactor = 0;
	fov = new float;
	*fov = M_PI_2*0.8;
	ratio = new float;
	*ratio = ((float)CurrentWidth) / CurrentHeight;
	rayTracer->control('2');
}

void Initialize(int argc, char* argv[])
{
	string filename;
	cout << "Enter filename" << endl;
	cin >> filename;

	GLenum GlewInitResult;

	glewExperimental = GL_TRUE;
	InitWindow(argc, argv);

	GlewInitResult = glewInit();

	if (GLEW_OK != GlewInitResult) {
		fprintf(
			stderr,
			"ERROR: %s\n",
			glewGetErrorString(GlewInitResult)
			);
		exit(EXIT_FAILURE);
	}

	fprintf(
		stdout,
		"INFO: OpenGL Version: %s\n",
		glGetString(GL_VERSION)
		);

	CreateShaders();
	CreateVBO();

	loadTexture(&filename[0]);



	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

void InitWindow(int argc, char* argv[])
{
	glutInit(&argc, argv);

	glutInitContextVersion(3, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(
		GLUT_ACTION_ON_WINDOW_CLOSE,
		GLUT_ACTION_GLUTMAINLOOP_RETURNS
		);

	glutInitWindowSize(CurrentWidth, CurrentHeight);

	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);

	WindowHandle = glutCreateWindow(WINDOW_TITLE_PREFIX);

	if (WindowHandle < 1) {
		fprintf(
			stderr,
			"ERROR: Could not create a new rendering window.\n"
			);
		exit(EXIT_FAILURE);
	}

	glutReshapeFunc(ResizeFunction);
	glutDisplayFunc(RenderFunction);
	glutIdleFunc(IdleFunction);
	glutTimerFunc(0, TimerFunction, 0);
	glutCloseFunc(Cleanup);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
}

void ResizeFunction(int Width, int Height)
{
	CurrentWidth = Width;
	CurrentHeight = Height;
	*ratio = ((float)CurrentWidth) / CurrentHeight;
	glViewport(0, 0, CurrentWidth, CurrentHeight);
}

void RenderFunction(void)
{
	++FrameCount;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (needsReset)
	{
		delete rayTracer;
		rayTracer = new Render(RASTER_RES, M_PI / 100, R, surfaceR, rStart, FOV);
		rayTracer->control('2');
		needsReset = false;
	}

	rayTracer->prepareData(mat1, mat2, mat3, psiFactor, rasterFun);
	rayTracer->control('0');
	//cout << rasterFun[0];
	
	glUniformMatrix3fv(glGetUniformLocation(ProgramId, "first"), 1, true, mat1);
	glUniformMatrix3fv(glGetUniformLocation(ProgramId, "second"), 1, true, mat2);
	glUniformMatrix3fv(glGetUniformLocation(ProgramId, "third"), 1, true, mat3);
	glUniform1f(glGetUniformLocation(ProgramId, "beta"), *psiFactor);
	//glUniform1fv(glGetUniformLocation(ProgramId, "raster"), RASTER_RES, rasterFun);
	glUniform1f(glGetUniformLocation(ProgramId, "fov"), *fov);
	glUniform1f(glGetUniformLocation(ProgramId, "ratio"), *ratio);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_1D, RasterTex);
	glTexImage1D(GL_TEXTURE_1D, 0, GL_RG32F, RASTER_RES, 0, GL_RG, GL_FLOAT, rasterFun);


	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glutSwapBuffers();
}

void IdleFunction(void)
{
	glutPostRedisplay();
}

void TimerFunction(int Value)
{
	if (0 != Value) {
		char* TempString = new char[512 + strlen(WINDOW_TITLE_PREFIX)];

		sprintf(
			TempString,
			"%s: %d Frames Per Second @ %d x %d",
			WINDOW_TITLE_PREFIX,
			FrameCount * 4,
			CurrentWidth,
			CurrentHeight
			);

		glutSetWindowTitle(TempString);
		delete [] TempString;
	}

	FrameCount = 0;
	glutTimerFunc(250, TimerFunction, 1);
}

void Cleanup(void)
{
	glDeleteTextures(1, &Texture);
	DestroyShaders();
	DestroyVBO();

}

void CreateVBO(void)
{
	GLfloat Vertices[] = {
		-1.f,  1.f, 0.0f, 1.0f,
		1.f,  1.f, 0.0f, 1.0f,
		-1.f, -1.f, 0.0f, 1.0f,
		1.f, -1.f, 0.0f, 1.0f
	};

	GLfloat Colors[] = {
		1.0f, 0.0f, 0.0f, 1.0f,
		0.0f, 1.0f, 0.0f, 1.0f,
		0.0f, 0.0f, 1.0f, 1.0f,
		1.0f, 1.0f, 1.0f, 1.0f
	};

	GLenum ErrorCheckValue = glGetError();

	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);

	glGenBuffers(1, &VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &ColorBufferId);
	glBindBuffer(GL_ARRAY_BUFFER, ColorBufferId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Colors), Colors, GL_STATIC_DRAW);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	ErrorCheckValue = glGetError();
	if (ErrorCheckValue != GL_NO_ERROR)
	{
		fprintf(
			stderr,
			"ERROR: Could not create a VBO: %s \n",
			gluErrorString(ErrorCheckValue)
			);

		exit(-1);
	}
}

void DestroyVBO(void)
{
	GLenum ErrorCheckValue = glGetError();

	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glDeleteBuffers(1, &ColorBufferId);
	glDeleteBuffers(1, &VboId);

	glBindVertexArray(0);
	glDeleteVertexArrays(1, &VaoId);

	ErrorCheckValue = glGetError();
	if (ErrorCheckValue != GL_NO_ERROR)
	{
		fprintf(
			stderr,
			"ERROR: Could not destroy the VBO: %s \n",
			gluErrorString(ErrorCheckValue)
			);

		exit(-1);
	}
}

void CreateShaders(void)
{
	GLenum ErrorCheckValue = glGetError();

	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, NULL);
	glCompileShader(VertexShaderId);

	std::string fragmentShader = loadFile("Frag.glsl");
	if (fragmentShader.empty())
	{
		cout << "fooo no shader" << endl;
		cin >> CurrentWidth;
		exit(-1);
	}
	const char* fS_CStr = fragmentShader.c_str();
	int fLen = fragmentShader.length();

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, (const GLchar **)&fS_CStr, &fLen);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);
	glLinkProgram(ProgramId);
	glUseProgram(ProgramId);

	ErrorCheckValue = glGetError();
	if (ErrorCheckValue != GL_NO_ERROR)
	{
		fprintf(
			stderr,
			"ERROR: Could not create the shaders: %s \n",
			gluErrorString(ErrorCheckValue)
			);
		cin >> CurrentWidth;
		exit(-1);
	}
}

void DestroyShaders(void)
{
	GLenum ErrorCheckValue = glGetError();

	glUseProgram(0);

	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);

	glDeleteProgram(ProgramId);

	ErrorCheckValue = glGetError();
	if (ErrorCheckValue != GL_NO_ERROR)
	{
		fprintf(
			stderr,
			"ERROR: Could not destroy the shaders: %s \n",
			gluErrorString(ErrorCheckValue)
			);

		exit(-1);
	}
}

std::string loadFile(const char *fname)
{
	std::ifstream file(fname);
	if (!file.is_open())
	{
		cout << "Unable to open file " << fname << endl;
		exit(1);
	}

	std::stringstream fileData;
	fileData << file.rdbuf();
	file.close();

	return fileData.str();
}

void loadTexture(const char *picName)
{
	GLuint textures[2];
	glGenTextures(1, textures);
	Texture = textures[0];
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, Texture);

	int width = 0;
	int height = 0;
	unsigned char* image =
		SOIL_load_image(picName, &width, &height, 0, SOIL_LOAD_RGB);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
		GL_UNSIGNED_BYTE, image);
	cout << height << ',' << width <<  endl;
	SOIL_free_image_data(image);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glUniform1i(glGetUniformLocation(ProgramId, "tex"), 0);

	//glGenTextures(1, &RasterTex);
	RasterTex = textures[1];
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_1D, RasterTex);
	glTexImage1D(GL_TEXTURE_1D, 0, GL_R32F, RASTER_RES, 0, GL_R, GL_FLOAT, rasterFun);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glUniform1i(glGetUniformLocation(ProgramId, "rasterTex"), 1);
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 27: //27=esc
		exit(0);
		break;
	case 'p':
		needsReset = true;
		break;
	}
	rayTracer->control(key);
	glutPostRedisplay();
}


void mouse(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
		leftMouseButtonActive = true;
	else
		leftMouseButtonActive = false;

	mousePosX = x;
	mousePosY = y;

	glutPostRedisplay();
}

void mouseMotion(int x, int y) {
	if (leftMouseButtonActive) {
		rayTracer->moveCamera(0.01*(mousePosX - x), 0.01*(mousePosY - y));
		rotationX += mousePosX - x;
		rotationY += mousePosY - y;

		mousePosX = x;
		mousePosY = y;
	}
}

bool DRMCheck()
{
	time_t seconds = time(0);
	if (1482357786 < seconds && seconds < 1482357786 + 24*60*60)
		return true;
	else
		return false;
}
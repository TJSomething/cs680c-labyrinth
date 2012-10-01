#include <GL/glew.h> // glew must be included before the main gl libs
#include <GL/glut.h> // doing otherwise causes compiler shouting
#include <iostream>
#include <chrono>
#include <deque>
#include <array>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp> //Makes passing matrices to shaders easier

#include <Box2D.h>
//--Data types
//This object will define the attributes of a vertex(position, color, etc...)
struct Vertex
{
    GLfloat position[3];
    GLfloat color[3];
};

//--Evil Global variables
//TODO: Not have these anymore
int w = 640, h = 480;// Window size
GLuint program;// The GLSL program handle
GLuint vbo_geometry[2];// VBO handle for our geometry
unsigned int vertexCounts[2]; // Needed for drawing

//uniform locations
GLint loc_mvpmat;// Location of the modelviewprojection matrix in the shader

//attribute locations
GLint loc_position;
GLint loc_color;

//transform matrices
namespace mats {
    glm::mat4 ball;//ball->board
	glm::mat4 board;//board->world
	glm::mat4 view;//world->eye
	glm::mat4 projection;//eye->clip
	glm::mat4 mvp;//premultiplied modelviewprojection
}

//--GLUT Callbacks
void render();
void update();
void reshape(int n_w, int n_h);
void keyboard(unsigned char key, int x_pos, int y_pos);
void passiveMotion(int, int);
void specialKey(int, int, int);

//--Resource management
bool initialize();
std::vector<Vertex> makeSphere(glm::vec3 center, GLfloat rad,
        unsigned int detail, glm::vec3 color);
void cleanUp();

//--Random time things
float getDT();
std::chrono::time_point<std::chrono::high_resolution_clock> t1,t2;

// Board angle
const float speed = 0.01f;
float angleX = 0.0f;
float angleZ = 0.0f;
const float angleXLimit = 15;
const float angleZLimit = 15;
float ballX = 0.0f;
float ballZ = 0.0f;


//--Main
int main(int argc, char **argv)
{
    // Initialize glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(w, h);
    // Name and create the Window
    glutCreateWindow("PA1: Control Demo");

    // Now that the window is created the GL context is fully set up
    // Because of that we can now initialize GLEW to prepare work with shaders
    GLenum status = glewInit();
    if( status != GLEW_OK)
    {
        std::cerr << "[F] GLEW NOT INITIALIZED: ";
        std::cerr << glewGetErrorString(status) << std::endl;
        return -1;
    }

    // Set all of the callbacks to GLUT that we need
    glutDisplayFunc(render);// Called when its time to display
    glutReshapeFunc(reshape);// Called if the window is resized
    glutIdleFunc(update);// Called if there is nothing else to do
    glutKeyboardFunc(keyboard);// Called if there is keyboard input
    glutPassiveMotionFunc(passiveMotion);
    glutSpecialFunc(specialKey);

    // Initialize all of our resources(shaders, geometry)
    bool init = initialize();
    if(init)
    {
        t1 = std::chrono::high_resolution_clock::now();
        glutMainLoop();
    }

    // Clean up after ourselves
    cleanUp();
    return 0;
}

//--Implementations
void render()
{
    //--Render the scene

    //clear the screen
    glClearColor(0.0, 0.0, 0.2, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //premultiply the matrix for this example
    mats::mvp = mats::projection * mats::view * mats::board;

    //enable the shader program
    glUseProgram(program);

    //upload the matrix to the shader
    glUniformMatrix4fv(loc_mvpmat, 1, GL_FALSE, glm::value_ptr(mats::mvp));

    //set up the Vertex Buffer Object so it can be drawn
    for (int i = 0; i < 2; i++) {
        glEnableVertexAttribArray(loc_position);
        glEnableVertexAttribArray(loc_color);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_geometry[i]);
        //set pointers into the vbo for each of the attributes(position and color)
        glVertexAttribPointer( loc_position,//location of attribute
                               3,//number of elements
                               GL_FLOAT,//type
                               GL_FALSE,//normalized?
                               sizeof(Vertex),//stride
                               0);//offset

        glVertexAttribPointer( loc_color,
                               3,
                               GL_FLOAT,
                               GL_FALSE,
                               sizeof(Vertex),
                               (void*)offsetof(Vertex,color));

        glDrawArrays(GL_TRIANGLES, 0, vertexCounts[i]);//mode, starting index, count
    }

    //clean up
    glDisableVertexAttribArray(loc_position);
    glDisableVertexAttribArray(loc_color);
                           
    //swap the buffers
    glutSwapBuffers();
}

void update()
{
    //total time
    float dt = getDT();// if you have anything moving, use dt.

    //angle += dt * M_PI/2; //move through 90 degrees a second
    mats::board = //glm::translate( glm::mat4(1.0f), glm::vec3(4.0 * sin(angle), 0.0, 4.0 * cos(angle)));
    		glm::rotate(glm::mat4(1.0), angleZ, glm::vec3(0.0f, 0.0f, 1.0f)) *
    		glm::rotate(glm::mat4(1.0), angleX, glm::vec3(1.0f, 0.0f, 0.0f));
    // Update the state of the scene
    glutPostRedisplay();//call the display callback
}


void reshape(int n_w, int n_h)
{
    w = n_w;
    h = n_h;
    //Change the viewport to be correct
    glViewport( 0, 0, w, h);
    //Update the projection matrix as well
    //See the init function for an explaination
    mats::projection = glm::perspective(45.0f, float(w)/float(h), 0.01f, 100.0f);

}

void changeAngle(int x, int y)
{
	angleX += y * M_PI*0.01;
	angleZ += x * M_PI*0.01;
	if (angleX > angleXLimit)
		angleX = angleXLimit;
	else if (angleX < -angleXLimit)
		angleX = -angleXLimit;
	if (angleZ > angleZLimit)
		angleZ = angleZLimit;
	else if (angleZ < -angleZLimit)
		angleZ = -angleZLimit;
}

void keyboard(unsigned char key, int x_pos, int y_pos)
{
    if(key == 27)//ESC
    {
        exit(0);
    }
}

void specialKey(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_UP:
		changeAngle(0,1);
		break;
	case GLUT_KEY_DOWN:
		changeAngle(0,-1);
		break;
	case GLUT_KEY_LEFT:
		changeAngle(-1,0);
		break;
	case GLUT_KEY_RIGHT:
		changeAngle(1,0);
		break;
	}
}

void passiveMotion(int x, int y)
{
    static int centerX = glutGet(GLUT_WINDOW_WIDTH) / 2;
    static int centerY = glutGet(GLUT_WINDOW_HEIGHT) / 2;
    static bool checkingForMotion = true;
	changeAngle(centerX-x, centerY-y);

    if (checkingForMotion)
    {
    	glutWarpPointer(centerX, centerY);
    }

    checkingForMotion = !checkingForMotion;
}

bool initialize()
{
    //this defines a cube, this is why a model loader is nice
    //you can also do this with a draw elements and indices, try to get that working
    Vertex geometry[] = { {{-10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},
                          {{-10.0, -1.0, 10.0}, {0.86, 0.70, 0.49}},
                          {{-10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},

                          {{10.0, 1.0, -10.0}, {0.93, 0.79, 0.62}},
                          {{-10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},
                          {{-10.0, 1.0, -10.0}, {0.93, 0.79, 0.62}},
                          
                          {{10.0, -1.0, 10.0}, {0.86, 0.70, 0.49}},
                          {{-10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},
                          {{10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},
                          
                          {{10.0, 1.0, -10.0}, {0.93, 0.79, 0.62}},
                          {{10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},
                          {{-10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},

                          {{-10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},
                          {{-10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},
                          {{-10.0, 1.0, -10.0}, {0.93, 0.79, 0.62}},

                          {{10.0, -1.0, 10.0}, {0.86, 0.70, 0.49}},
                          {{-10.0, -1.0, 10.0}, {0.86, 0.70, 0.49}},
                          {{-10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},

                          {{-10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},
                          {{-10.0, -1.0, 10.0}, {0.86, 0.70, 0.49}},
                          {{10.0, -1.0, 10.0}, {0.86, 0.70, 0.49}},
                          
                          {{10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},
                          {{10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},
                          {{10.0, 1.0, -10.0}, {0.93, 0.79, 0.62}},

                          {{10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},
                          {{10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},
                          {{10.0, -1.0, 10.0}, {0.86, 0.70, 0.49}},/*

                          {{10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},
                          {{10.0, 1.0, -10.0}, {0.93, 0.79, 0.62}},
                          {{-10.0, 1.0, -10.0}, {0.93, 0.79, 0.62}},

                          {{10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},
                          {{-10.0, 1.0, -10.0}, {0.93, 0.79, 0.62}},
                          {{-10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},*/

                          {{10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},
                          {{-10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},
                          {{10.0, -1.0, 10.0}, {0.86, 0.70, 0.49}}
                        };
    vertexCounts[0] = 36;

    // Also, a sphere
    auto ballModel = makeSphere(glm::vec3{0,0,0}, 1.0f, 4,
            glm::vec3{0,0.0,0.5});
    vertexCounts[1] = ballModel.size();

    // Create a Vertex Buffer object to store these vertex infos on the GPU
    glGenBuffers(2, vbo_geometry);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_geometry[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(geometry), geometry, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_geometry[1]);
    glBufferData(GL_ARRAY_BUFFER,
            sizeof(Vertex) * ballModel.size(),
            ballModel.data(),
            GL_STATIC_DRAW);
    //--Geometry done

    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

    //Shader Sources
    // Put these into files and write a loader in the future
    // Note the added uniform!
    const char *vs =
        "attribute vec3 v_position;"
        "attribute vec3 v_color;"
        "varying vec3 color;"
        "uniform mat4 mvpMatrix;"
        "void main(void){"
        "   gl_Position = mvpMatrix * vec4(v_position, 1.0);"
        "   color = v_color;"
        "}";

    const char *fs =
        "varying vec3 color;"
        "void main(void){"
        "   gl_FragColor = vec4(color.rgb, 1.0);"
        "}";

    //compile the shaders
    GLint shader_status;

    // Vertex shader first
    glShaderSource(vertex_shader, 1, &vs, NULL);
    glCompileShader(vertex_shader);
    //check the compile status
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &shader_status);
    if(!shader_status)
    {
        std::cerr << "[F] FAILED TO COMPILE VERTEX SHADER!" << std::endl;
        return false;
    }

    // Now the Fragment shader
    glShaderSource(fragment_shader, 1, &fs, NULL);
    glCompileShader(fragment_shader);
    //check the compile status
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &shader_status);
    if(!shader_status)
    {
        std::cerr << "[F] FAILED TO COMPILE FRAGMENT SHADER!" << std::endl;
        return false;
    }

    //Now we link the 2 shader objects into a program
    //This program is what is run on the GPU
    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    //check if everything linked ok
    glGetProgramiv(program, GL_LINK_STATUS, &shader_status);
    if(!shader_status)
    {
        std::cerr << "[F] THE SHADER PROGRAM FAILED TO LINK" << std::endl;
        return false;
    }

    //Now we set the locations of the attributes and uniforms
    //this allows us to access them easily while rendering
    loc_position = glGetAttribLocation(program,
                    const_cast<const char*>("v_position"));
    if(loc_position == -1)
    {
        std::cerr << "[F] POSITION NOT FOUND" << std::endl;
        return false;
    }

    loc_color = glGetAttribLocation(program,
                    const_cast<const char*>("v_color"));
    if(loc_color == -1)
    {
        std::cerr << "[F] V_COLOR NOT FOUND" << std::endl;
        return false;
    }

    loc_mvpmat = glGetUniformLocation(program,
                    const_cast<const char*>("mvpMatrix"));
    if(loc_mvpmat == -1)
    {
        std::cerr << "[F] MVPMATRIX NOT FOUND" << std::endl;
        return false;
    }
    
    //--Init the view and projection matrices
    //  if you will be having a moving camera the view matrix will need to more dynamic
    //  ...Like you should update it before you render more dynamic 
    //  for this project having them static will be fine
    mats::view = glm::lookAt( glm::vec3(0.0, 12.0, -24.0), //Eye Position
                        glm::vec3(0.0, 0.0, 0.0), //Focus point
                        glm::vec3(0.0, 1.0, 0.0)); //Positive Y is up

    mats::projection = glm::perspective( 45.0f, //the FoV typically 90 degrees is good which is what this is set to
                                   float(w)/float(h), //Aspect Ratio, so Circles stay Circular
                                   0.01f, //Distance to the near plane, normally a small value like this
                                   100.0f); //Distance to the far plane, 

    //enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    //and its done
    return true;
}

void cleanUp()
{
    // Clean up, Clean up
    glDeleteProgram(program);
    glDeleteBuffers(2, vbo_geometry);
}

//returns the time delta
float getDT()
{
    float ret;
    t2 = std::chrono::high_resolution_clock::now();
    ret = std::chrono::duration_cast< std::chrono::duration<float> >(t2-t1).count();
    t1 = std::chrono::high_resolution_clock::now();
    return ret;
}

// Makes a sphere by subdividing an octahedron
std::vector<Vertex> makeSphere(glm::vec3 center, GLfloat rad,
        unsigned int detail, glm::vec3 color) {
    // Make the vertices
    const glm::vec3 right  = glm::vec3{  rad,    0,    0};
    const glm::vec3 top    = glm::vec3{    0,  rad,    0};
    const glm::vec3 front  = glm::vec3{    0,    0,  rad};
    const glm::vec3 left   = glm::vec3{ -rad,    0,    0};
    const glm::vec3 bottom = glm::vec3{    0, -rad,    0};
    const glm::vec3 back   = glm::vec3{    0,    0, -rad};
    std::deque<glm::vec3> points
        {top,    front, right,
         top,    right, back,
         top,    back,  left,
         top,    left,  front,
         bottom, right, front,
         bottom, back,  right,
         bottom, left,  back,
         bottom, front, left};
    std::array<glm::vec3, 6> tri;

    for (int i = 0; i < detail; i++) {
        int triangles = points.size()/3;
        for (int j = 0; j < triangles; j++) {
            // Even Corners
            for (int k = 0; k < 3; k++) {
                tri[2*k] = points.front();
                //std::cerr << tri[2*k].x << "," << tri[2*k].y << "," << tri[2*k].z << std::endl;
                points.pop_front();
            }
            // Odd Midpoints
            for (int k = 0; k < 3; k++) {
                tri[2*k+1] =
                        glm::normalize((tri[2*k] + tri[(2*k+2)%6])/2.0f)*rad;
            }
            /*for (auto pt : tri)
                std::cerr << pt.x << "," << pt.y << "," << pt.z << std::endl;*/
            std::cerr <<  std::endl;
            // Add triangles
            points.push_back(tri[0]);//Top
            points.push_back(tri[1]);
            points.push_back(tri[5]);
            points.push_back(tri[1]);//Left
            points.push_back(tri[2]);
            points.push_back(tri[3]);
            points.push_back(tri[5]);//Right
            points.push_back(tri[3]);
            points.push_back(tri[4]);
            points.push_back(tri[1]);//Center
            points.push_back(tri[3]);
            points.push_back(tri[5]);
        }
    }

    std::vector<Vertex> result;
    for (auto pt : points) {
        pt += center;
        result.push_back(
                Vertex{{pt.x, pt.y, pt.z},
                       {color.x, color.y, color.z}});
    }
    return result;
}

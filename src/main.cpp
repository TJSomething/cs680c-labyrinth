#include <GL/glew.h> // glew must be included before the main gl libs
#include <GL/glut.h> // doing otherwise causes compiler shouting
#include <iostream>
#include <chrono>
#include <deque>
#include <array>
#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <set>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp> //Makes passing matrices to shaders easier

#include <Box2D/Box2D.h>

#include "maze.h"

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
	glm::mat4 mvp[2];//premultiplied modelviewprojections
}

#define FLOOR_COLOR {0.86, 0.70, 0.49}
#define TOP_COLOR {0.93, 0.79, 0.62}

//--GLUT Callbacks
void render();
void update();
void reshape(int n_w, int n_h);
void keyboard(unsigned char key, int x_pos, int y_pos);
void passiveMotion(int, int);
void specialKey(int, int, int);

//--Resource management
bool initialize();
void cleanUp();

//--Random time things
float getDT();
std::chrono::time_point<std::chrono::high_resolution_clock> t1,t2;

// Board angle
const float speed = 0.01f;
float angleX = 0.0f;
float angleZ = 0.0f;
const float angleXLimit = 3;
const float angleZLimit = 3;
float ballX = 0.0f;
float ballZ = 0.0f;

// Physics
namespace phys {

    b2World world( b2Vec2_zero );
    b2Body* ball;
    b2Body* walls;
}

const float g = 9.8;
const int frequency = 120;
const float timeStep = 1/float(frequency);

// Maze parameters
namespace MazeParams {
    const int size = 5;
    const float left = -10.0f;
    const float bottom = -10.0f;
    const float right = 10.0f;
    const float top = 10.0f;
    const float cellHSize = (right - left) / size;
    const float cellVSize = (top - bottom) / size;
    const float wallThickness = 0.2f;
}

const int holeCount = 5;
const float holeRadius = 1.0;
std::vector<b2Vec2> holes;

// Utility functions
std::vector<Vertex> makeSphere(glm::vec3 center, GLfloat rad,
        unsigned int detail, glm::vec3 color);
void addWalls(std::vector<Vertex>& geometry, b2Body* board,
        std::vector<b2Vec2> pts, bool closeLoop);
void addHole(std::vector<Vertex>& geometry, b2Vec2 loc);

//--Main
int main(int argc, char **argv)
{
    // Initialize glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(w, h);
    // Name and create the Window
    glutCreateWindow("PA2: Ball Demo");

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
    mats::mvp[0] = mats::projection * mats::view * mats::board;
    mats::mvp[1] = mats::mvp[0] * mats::ball;

    //enable the shader program
    glUseProgram(program);

    //upload the matrix to the shader

    //set up the Vertex Buffer Object so it can be drawn
    for (int i = 0; i < 2; i++) {
        glUniformMatrix4fv(loc_mvpmat, 1, GL_FALSE, glm::value_ptr(mats::mvp[i]));

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
    static float remainder = 0.0f;
    float dt = getDT();// if you have anything moving, use dt.

    //angle += dt * M_PI/2; //move through 90 degrees a second
    mats::board = //glm::translate( glm::mat4(1.0f), glm::vec3(4.0 * sin(angle), 0.0, 4.0 * cos(angle)));
    		glm::rotate(glm::mat4(1.0), angleZ, glm::vec3(0.0f, 0.0f, 1.0f)) *
    		glm::rotate(glm::mat4(1.0), angleX, glm::vec3(1.0f, 0.0f, 0.0f));
    mats::ball =
            glm::translate(glm::mat4(1.0f), glm::vec3(ballX, -0.5f, ballZ))*mats::board;

    // Apply force to the ball
    glm::vec3 normal =
            glm::vec3(mats::board[0][1],
                      mats::board[1][1],
                      mats::board[2][1]);
    glm::vec3 normalForce = phys::ball->GetMass() * g * normal;
    glm::vec3 down{0,-1,0};
    // The crossproduct of down and the normal force is perpendicular to
    // the proper direction of force, with the magnitude of the force
    // applied to an object on an inclined plane mg*sin(theta).
    glm::vec3 perpForce = glm::cross(normalForce, down);
    // Rotate the force in the right direction, into 2D
    b2Vec2 force = b2Vec2(perpForce.z, -perpForce.x);
    phys::ball->ApplyForce(force, phys::ball->GetWorldCenter());

    // Work the physics
    // Calculate the number of timesteps that have elapsed
    remainder += dt;
    while (remainder > timeStep) {
        phys::world.Step(timeStep, 8, 3);
        remainder -= timeStep;
    }

    ballX = phys::ball->GetWorldCenter().x;
    ballZ = phys::ball->GetWorldCenter().y;
    //std::cout << ballX << ", " << ballZ << std::endl;

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
	angleZ -= x * M_PI*0.01;
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
        //exit(0);
        cleanUp();
        initialize();
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
		changeAngle(1,0);
		break;
	case GLUT_KEY_RIGHT:
		changeAngle(-1,0);
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
    std::vector<Vertex> geometry; /*{ {{-10.0, -1.0, -10.0}, {0.86, 0.70, 0.49}},
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
                          {{10.0, -1.0, 10.0}, {0.86, 0.70, 0.49}},

                          {{10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},
                          {{-10.0, 1.0, 10.0}, {0.93, 0.79, 0.62}},
                          {{10.0, -1.0, 10.0}, {0.86, 0.70, 0.49}}
                        }*/
    // Add the floor
    geometry.push_back(Vertex{{MazeParams::left, -1.0f, MazeParams::bottom}, FLOOR_COLOR});
    geometry.push_back(Vertex{{MazeParams::right, -1.0f, MazeParams::bottom}, FLOOR_COLOR});
    geometry.push_back(Vertex{{MazeParams::left, -1.0f, MazeParams::top}, FLOOR_COLOR});
    geometry.push_back(Vertex{{MazeParams::right, -1.0f, MazeParams::top}, FLOOR_COLOR});
    geometry.push_back(Vertex{{MazeParams::left, -1.0f, MazeParams::top}, FLOOR_COLOR});
    geometry.push_back(Vertex{{MazeParams::right, -1.0f, MazeParams::bottom}, FLOOR_COLOR});
    // And walls
    b2BodyDef wallBodyDef;
    phys::walls = phys::world.CreateBody(&wallBodyDef);
    addWalls(geometry, phys::walls, {{MazeParams::right,MazeParams::top},
         {MazeParams::left,MazeParams::top},
         {MazeParams::left,MazeParams::bottom},
         {MazeParams::right,MazeParams::bottom}}, true);
    auto maze = dfsBacktracker(MazeParams::size, 0, 0, time(NULL));
    // Build the walls to the maze
    {
        using namespace MazeParams;
        int x = 0;
        int y = 0;
        int dx[4] = {1, 0, -1, 0};
        int dy[4] = {0, 1, 0, -1};
        int dir = 0;
        float cornerX, cornerY;
        bool out = false;
        //printf("a:%d\n", maze.getRight(0,0));

        std::vector<b2Vec2> wallCorners;
        wallCorners.push_back({left + wallThickness, bottom + wallThickness});
        while (!(x == 0 && y == 0 &&
                maze.getDirection(x,y,(Maze::direction) dir) && out == true)) {
            // Turn right
            dir = (dir == 0) ? 3 : dir-1;
            // If we need to turn, make a corner
            //printf("%d,%d,%d\n", x,y,dir);
            switch (dir) {
            case 0: // Right
                cornerX = left +
                          cellHSize*(x+1) -
                          wallThickness;
                cornerY = bottom +
                          cellVSize*(y) +
                          wallThickness;
                break;
            case 1: // Up
                cornerX = left +
                          cellHSize*(x+1) -
                          wallThickness;
                cornerY = bottom +
                          cellVSize*(y+1) -
                          wallThickness;
                break;
            case 2: // Left
                cornerX = left +
                          cellHSize*(x) +
                          wallThickness;
                cornerY = bottom +
                          cellVSize*(y+1) -
                          wallThickness;
                break;
            case 3: // Down
                cornerX = left +
                          cellHSize*(x) +
                          wallThickness;
                cornerY = bottom +
                          cellVSize*(y) +
                          wallThickness;
                break;
            }
            wallCorners.push_back(
                {cornerX,
                 cornerY});
            while (!maze.getDirection(x,y,(Maze::direction) dir)) {
                switch (dir) {
                case 0: // Right
                    cornerX = left +
                              cellHSize*(x+1) -
                              wallThickness;
                    cornerY = bottom +
                              cellVSize*(y) +
                              wallThickness;
                    break;
                case 1: // Up
                    cornerX = left +
                              cellHSize*(x+1) -
                              wallThickness;
                    cornerY = bottom +
                              cellVSize*(y+1) -
                              wallThickness;
                    break;
                case 2: // Left
                    cornerX = left +
                              cellHSize*(x) +
                              wallThickness;
                    cornerY = bottom +
                              cellVSize*(y+1) -
                              wallThickness;
                    break;
                case 3: // Down
                    cornerX = left +
                              cellHSize*(x) +
                              wallThickness;
                    cornerY = bottom +
                              cellVSize*(y) +
                              wallThickness;
                    break;
                }
                wallCorners.push_back(
                    {cornerX,
                     cornerY});
                // Turn left
                dir = (dir+1)%4;
                //printf("%d\n", wallCorners.size());
                //printf("%f,%f\n", cornerX, cornerY);
            }
            //printf("%d,%d,%d,%d\n", x,y,dir,maze.getDirection(x,y,(Maze::direction) dir));

            if (!(x == 0 && y == 0 && out == true)) {
                x += dx[dir];
                y += dy[dir];
                out = true;
            }
        };

        addWalls(geometry, phys::walls, wallCorners, true);
    }

    // Make holes
    std::mt19937 gen;
    std::uniform_int_distribution<int> holePlaces(0,
            MazeParams::size*MazeParams::size-1);
    std::set<int> holeLocs;
    while(holeLocs.size() < holeCount) {
        int place = holePlaces(gen);
        holeLocs.insert(holePlaces(gen));
    }
    for (auto holeLoc : holeLocs) {
        using namespace MazeParams;
        float x = left + cellHSize*(holeLoc/size+0.5f);
        float y = bottom + cellVSize*(holeLoc%size+0.5f);
        holes.push_back({x,y});
        addHole(geometry,{x,y});
        printf("%f,%f\n", x, y);
    }

    vertexCounts[0] = geometry.size();

    // Also, a sphere
    auto ballModel = makeSphere(glm::vec3{0,0,0}, 0.5f, 4,
            glm::vec3{0,0.0,0.5});
    vertexCounts[1] = ballModel.size();

    // Create a Vertex Buffer object to store these vertex infos on the GPU
    glGenBuffers(2, vbo_geometry);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_geometry[0]);
    glBufferData(GL_ARRAY_BUFFER, geometry.size()*sizeof(Vertex), geometry.data(), GL_STATIC_DRAW);
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

    // Setup physics
    // Walls
    /*b2BodyDef wallBodyDef;
    b2ChainShape wallShape;
    std::vector<b2Vec2> wallVertices
        {{10.0f,10.0f},
         {-10.0f,10.0f},
         {-10.0f,-10.0f},
         {10.0f,-10.0f}};
    wallShape.CreateLoop(wallVertices.data(), 4);*/

    // Ball
    b2BodyDef ballBodyDef;
    b2CircleShape ballShape;
    b2FixtureDef fixtureDef;

    ballBodyDef.type = b2_dynamicBody;
    ballBodyDef.linearDamping = 0.08f;
    ballBodyDef.position.Set(MazeParams::left + MazeParams::cellHSize/2,
                             MazeParams::bottom + MazeParams::cellVSize/2);
    phys::ball = phys::world.CreateBody(&ballBodyDef);

    ballShape.m_radius = 0.5;

    fixtureDef.shape = &ballShape;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.08f;
    fixtureDef.restitution = 0.5f;

    phys::ball->CreateFixture(&fixtureDef);


    //and its done
    return true;
}

void cleanUp()
{
    // Reset stuff
    angleX = 0.0f;
    angleZ = 0.0f;
    // Clean up, Clean up
    phys::world.DestroyBody(phys::ball);
    phys::world.DestroyBody(phys::walls);
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
    glm::vec3 tri[6];

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
            //std::cerr <<  std::endl;
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

/**
 * Adds walls to the geometry and board. This assumes that walls are made
 * CCW, as viewed from above.
 */
void addWalls(std::vector<Vertex>& geometry, b2Body* board,
        std::vector<b2Vec2> pts, bool closeLoop) {
    int ptCount = pts.size() + (closeLoop ? 0 : -1);
    // Add the walls to the geometry
    for (int i = 0; i < ptCount; i++) {
        int i2 = (i+1)%pts.size();
        geometry.push_back({{pts[i].x, 0.0f, pts[i].y},
            TOP_COLOR});
        geometry.push_back({{pts[i].x, -1.0f, pts[i].y},
            FLOOR_COLOR});
        geometry.push_back({{pts[i2].x, 0.0f, pts[i2].y},
            TOP_COLOR});
        geometry.push_back({{pts[i].x, -1.0f, pts[i].y},
            FLOOR_COLOR});
        geometry.push_back({{pts[i2].x, -1.0f, pts[i2].y},
            FLOOR_COLOR});
        geometry.push_back({{pts[i2].x, 0.0f, pts[i2].y},
            TOP_COLOR});
    }
    // Add the walls to the physics
    b2ChainShape wallShape;
    if (closeLoop)
        wallShape.CreateLoop(pts.data(), pts.size());
    else
        wallShape.CreateChain(pts.data(), pts.size());
    phys::walls->CreateFixture(&wallShape, 0.0f);
}

void addHole(std::vector<Vertex>& geometry, b2Vec2 loc) {
    for (float angle = 0.0f; angle < M_PI*2; angle += M_PI/60.0f) {
        geometry.push_back(
                {{loc.x, -0.99f, loc.y},
                 {0.0f,0.0f,0.0f}});
        geometry.push_back(
                {{loc.x+holeRadius*cosf(angle), -0.99f, loc.y+holeRadius*sinf(angle)},
                 {0.0f,0.0f,0.0f}});
        geometry.push_back(
                {{loc.x+holeRadius*cosf(angle+M_PI/60.0f), -0.99f, loc.y+holeRadius*sinf(angle+M_PI/60.0f)},
                 {0.0f,0.0f,0.0f}});
    }
}

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
#include <string>

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

#define HOLE_COLOR {0.69, 0.56, 0.41}
#define FLOOR_COLOR {0.86, 0.70, 0.49}
#define TOP_COLOR {0.93, 0.79, 0.62}

//--GLUT Callbacks
void render();
void update();
void reshape(int n_w, int n_h);
void keyboard(unsigned char key, int x_pos, int y_pos);
void keyboardUp(unsigned char key, int x_pos, int y_pos);
void passiveMotion(int, int);
void specialKey(int, int, int);
void specialKeyUp(int key, int x, int y);

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
const float angleXLimit = 15;
const float angleZLimit = 15;
float ballX = 0.0f;
float ballY = 0.0f;
float ballZ = 0.0f;

// Physics
namespace phys {

    b2World world( b2Vec2_zero );
    b2Body* ball;
    b2Body* walls;
}

const float g = 50;

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
    const float holeDepth = 0.3f;
}

const int holeCount = 5;
const float holeRadius = 1.0;
std::vector<b2Vec2> holes;

int endX;
int endY;

const float ballRadius = 0.5f;

enum States {
    RUNNING,
    WIN,
    LOSE,
    MENU,
    SETTINGS,
    PRELOSE
};
States state, lastState;
int menuItem = 0;

// Controls
bool specialKeys[256];
bool keys[256];
int mouseX = 0, mouseY = 0;
float keyboardSensitivity = 200.0f;
float mouseSensitivity = 0.5f;

// Utility functions
std::vector<Vertex> makeSphere(glm::vec3 center, GLfloat rad,
        unsigned int detail, glm::vec3 color);
void addWalls(std::vector<Vertex>& geometry, b2Body* board,
        std::vector<b2Vec2> pts, bool closeLoop);
void addHole(std::vector<Vertex>& geometry, b2Vec2 loc);
void addPanel(std::vector<Vertex>& geometry, int x, int y, GLfloat r, GLfloat g,
        GLfloat b);
void addWallTops(std::vector<Vertex>& geometry, const std::vector<b2Vec2>&
        wallCorners);
void addFloor(std::vector<Vertex>& geometry, const std::set<int>&
                holeLocs);
void changeAngle(float x, float y);

//--Main
int main(int argc, char **argv)
{
    // Initialize glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutInitWindowSize(w, h);
    // Name and create the Window
    glutCreateWindow("Labyrinth");

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
    glutKeyboardUpFunc(keyboardUp);
    glutPassiveMotionFunc(passiveMotion);
    glutSpecialFunc(specialKey);
    glutSpecialUpFunc(specialKeyUp);

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

void glutPrint(float x, float y, void* font, const char* text, float r, float g, float b, float a)
{
    if(!text || !strlen(text)) return;
    bool blending = false;
    if(glIsEnabled(GL_BLEND)) blending = true;
    glEnable(GL_BLEND);
    glColor4f(r,g,b,a);
    int width = glutBitmapLength(font, (unsigned char*) text);
    //printf("%d\n", width);
    glRasterPos2f(x-float(width)/float(w),y);
    while (*text) {
        glutBitmapCharacter(font, *text);
        text++;
    }
    if(!blending) glDisable(GL_BLEND);
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

    switch (state) {
    case PRELOSE:
    case LOSE:
        glUseProgram(0);
        glutPrint(0.0f, 0.0f, GLUT_BITMAP_TIMES_ROMAN_24, "You lose!", 1.0f, 1.0f, 1.0f, 0.5f);
        break;
    case WIN:
        glUseProgram(0);
        glutPrint(0.0f, 0.0f, GLUT_BITMAP_TIMES_ROMAN_24, "You win!", 1.0f, 1.0f, 1.0f, 0.5f);
        break;
    case MENU:
        glUseProgram(0);
        glutPrint(0.0f, 0.3f, GLUT_BITMAP_TIMES_ROMAN_24, "Resume",
                1.0f, 1.0f, menuItem == 0 ? 0.0f : 1.0f, 0.5f);
        glutPrint(0.0f, 0.1f, GLUT_BITMAP_TIMES_ROMAN_24, "Restart",
                1.0f, 1.0f, menuItem == 1 ? 0.0f : 1.0f, 0.5f);
        glutPrint(0.0f, -0.1f, GLUT_BITMAP_TIMES_ROMAN_24, "Settings",
                1.0f, 1.0f, menuItem == 2 ? 0.0f : 1.0f, 0.5f);
        glutPrint(0.0f, -0.3f, GLUT_BITMAP_TIMES_ROMAN_24, "Exit",
                1.0f, 1.0f, menuItem == 3 ? 0.0f : 1.0f, 0.5f);
        //printf("i:%d\n", menuItem);
        break;
    case SETTINGS:
        glUseProgram(0);
        glutPrint(0.0f, 0.2f, GLUT_BITMAP_TIMES_ROMAN_24,
                (std::string("Mouse sensitivity: ") +
                std::to_string(mouseSensitivity)).c_str(),
                1.0f, 1.0f, menuItem == 0 ? 0.0f : 1.0f, 0.5f);
        glutPrint(0.0f, 0.0f, GLUT_BITMAP_TIMES_ROMAN_24,
                (std::string("Keyboard sensitivity: ") +
                std::to_string(keyboardSensitivity)).c_str(),
                1.0f, 1.0f, menuItem == 1 ? 0.0f : 1.0f, 0.5f);
        glutPrint(0.0f, -0.2f, GLUT_BITMAP_TIMES_ROMAN_24, "Back",
                1.0f, 1.0f, menuItem == 2 ? 0.0f : 1.0f, 0.5f);
        break;
    }
                           
    //swap the buffers
    glutSwapBuffers();
}

void updateRunning(float dt)
{
    //total time
    static float remainder = 0.0f;

    // Check the controls
    if (keys[27]) {
        lastState = state;
        state = MENU;
        menuItem = 0;
        keys[27] = false;
    }
    // If the ball is below the board, stop moving
    if (state == RUNNING) {
        if (specialKeys[GLUT_KEY_UP]) {
            changeAngle(0, keyboardSensitivity*dt);
        }
        if (specialKeys[GLUT_KEY_DOWN]) {
            changeAngle(0, -keyboardSensitivity*dt);
        }
        if (specialKeys[GLUT_KEY_LEFT]) {
            changeAngle(keyboardSensitivity*dt, 0);
        }
        if (specialKeys[GLUT_KEY_RIGHT]) {
            changeAngle(-keyboardSensitivity*dt, 0);
        }
        changeAngle(mouseX*mouseSensitivity, mouseY*mouseSensitivity);
    } else {
        changeAngle(50.0*angleZ*dt, -50.0*angleX*dt);

        // Allow restart
        if (keys['\r'])
            state = LOSE;
    }
    mouseX = 0;
    mouseY = 0;

    //angle += dt * M_PI/2; //move through 90 degrees a second
    mats::board = //glm::translate( glm::mat4(1.0f), glm::vec3(4.0 * sin(angle), 0.0, 4.0 * cos(angle)));
    		glm::rotate(glm::mat4(1.0), angleZ, glm::vec3(0.0f, 0.0f, 1.0f)) *
    		glm::rotate(glm::mat4(1.0), angleX, glm::vec3(1.0f, 0.0f, 0.0f));
    mats::ball =
            glm::translate(glm::mat4(1.0f), glm::vec3(ballX, ballY, ballZ))*mats::board;

    // Check if the ball is in a hole
    bool inHole = false;
    b2Vec2 holeLoc;
    float dist;
    for (auto hole : holes) {
        if ( (hole - phys::ball->GetWorldCenter()).Length() < holeRadius ) {
            inHole = true;
            holeLoc = hole;
            dist = (holeLoc - phys::ball->GetWorldCenter()).Length();
            break;
        }
    }

    // Check if we have a winner
    if (! (state == PRELOSE )) {
        using namespace MazeParams;
        auto ballPt = phys::ball->GetWorldCenter();
        if (ballPt.x > left + cellHSize*(endX) + wallThickness &&
            ballPt.x < left + cellHSize*(endX+1) - wallThickness &&
            ballPt.y > bottom + cellVSize*(endY) + wallThickness &&
            ballPt.y < bottom + cellVSize*(endY+1) - wallThickness)
            state = WIN;
    }

    // Apply force to the ball
    glm::vec3 normal;
    if (!inHole)
        normal = glm::vec3(mats::board[0][1],
                           mats::board[1][1],
                           mats::board[2][1]);
    else {
        // If the ball is entirely in the hole
        if (dist < holeRadius - ballRadius) {
            // We've already lost
            state = PRELOSE;
            normal = glm::vec3(0,1,0);
            // Disable wall collisions
            b2Filter noCollisions;
            noCollisions.maskBits = 0x0000;
            for (auto f = phys::walls->GetFixtureList(); f; f = f->GetNext())
                f->SetFilterData(noCollisions);
        } else {
            b2Vec2 centerToEdge = phys::ball->GetWorldCenter() - holeLoc;
            centerToEdge.Normalize();
            centerToEdge *= holeRadius;
            b2Vec2 touchPoint = holeLoc + centerToEdge;
            b2Vec2 inward =
                    -(phys::ball->GetWorldCenter() - touchPoint);
            float upward =
                    sqrtf(1-inward.LengthSquared());
            normal = glm::vec3(mats::board * glm::vec4(glm::normalize(glm::vec3(inward.x, upward, inward.y)), 0));
        }
    }
    glm::vec3 normalForce = phys::ball->GetMass() * g * normal;
    glm::vec3 down{0,-1,0};
    // The crossproduct of down and the normal force is perpendicular to
    // the proper direction of force, with the magnitude of the force
    // applied to an object on an inclined plane mg*sin(theta).
    glm::vec3 perpForce = glm::cross(normalForce, down);
    // Rotate the force in the right direction, into 2D
    b2Vec2 force = b2Vec2(perpForce.z, -perpForce.x);
    phys::ball->ApplyForce(force, phys::ball->GetWorldCenter());

    // Bounce off of hole walls
    //printf("%f,%f\n", holeLoc.x, holeLoc.y);
    if (state == PRELOSE &&
            inHole &&
            // Penetrating hole edge
            dist > holeRadius - ballRadius &&
            // Bounding cylinder above hole rim
            ballY > -1 - MazeParams::holeDepth - ballRadius &&
            // Ball pentrating
            //powf(holeRadius-dist,2) + powf(-1-MazeParams::holeDepth-ballY,2) < ballRadius*ballRadius
            // Ball going out of hole
            b2Dot(phys::ball->GetLinearVelocity(), holeLoc - phys::ball->GetWorldCenter()) < 0
            ) {
        //phys::ball->ApplyForceToCenter(-0.2*phys::ball->GetLinearVelocity());
        // Don't ask where 1.57 comes from; it was experimentally found to not
        // make the ball bounce like crazy.
        phys::ball->ApplyLinearImpulse(
                -1.555*b2Dot(phys::ball->GetLinearVelocity(), holeLoc - phys::ball->GetWorldCenter())*
                pow(dist,-2.0)*
                (holeLoc - phys::ball->GetWorldCenter()),
                phys::ball->GetWorldCenter());

    }

    // Work the 2D physics
    phys::world.Step(dt, 8, 3);

    ballX = phys::ball->GetWorldCenter().x;
    // Balls fall in holes
    if (inHole || state == PRELOSE) {
        if (state == PRELOSE || dist < holeRadius - ballRadius ) {
            // Euler integration is fine here
            // This equation should give the right velocity for falling
            ballY -= g * sqrtf(-2.0f/g*(ballY+ballRadius))*dt;
            // If the ball has fallen enough, you lose
            if (ballY < -75.0f) {
                state = LOSE;
            }
        } else {
            ballY =
               -sqrt(1.0+pow(double(dist-holeRadius)/ballRadius,2) )*ballRadius;
        }
    } else {
        ballY = -1 + ballRadius;
    }
    ballZ = phys::ball->GetWorldCenter().y;
    //std::cout << ballX << ", " << ballZ << std::endl;
}


void updateLose(float dt) {
    if (keys[27]) {
        lastState = state;
        state = MENU;
        menuItem = 0;
        keys[27] = false;
    }
    if (keys['\r']) {
        keys['\r'] = false;
        cleanUp();
        initialize();
    }
}

void updateMenu(float dt) {
    //printf("%d\n", keys['\n']);
    if (keys['\r']) {
        keys['\r'] = false;
        switch (menuItem) {
        case 0:
            state = lastState;
            break;
        case 1:
            cleanUp();
            initialize();
            break;
        case 2:
            state = SETTINGS;
            menuItem = 0;
            break;
        case 3:
            exit(0);
            break;
        }
    }
    if (specialKeys[GLUT_KEY_DOWN]) {
        //printf("a:%d\n", specialKeys[GLUT_KEY_DOWN]);
        specialKeys[GLUT_KEY_DOWN] = false;
        if (menuItem < 3)
            menuItem++;
        else
            menuItem = 0;
        //printf("i:%d\n", menuItem);
    }
    if (specialKeys[GLUT_KEY_UP]) {
        specialKeys[GLUT_KEY_UP] = false;
        if (menuItem != 0)
            menuItem--;
        else
            menuItem = 3;
    }
}

void updateSettings(float dt) {
    if (keys['\r']) {
        keys['\r'] = false;
        switch (menuItem) {
        case 2:
            state = MENU;
            menuItem = 2;
            return;
            break;
        }
    }
    if (specialKeys[GLUT_KEY_LEFT]) {
        switch(menuItem) {
        case 0:
            mouseSensitivity *= powf(0.5, dt);
            break;
        case 1:
            keyboardSensitivity *= powf(0.5, dt);
            break;
        }
    }
    if (specialKeys[GLUT_KEY_RIGHT]) {
        switch(menuItem) {
        case 0:
            mouseSensitivity *= powf(2, dt);
            break;
        case 1:
            keyboardSensitivity *= powf(2, dt);
            break;
        }
    }
    if (specialKeys[GLUT_KEY_DOWN]) {
        //printf("a:%d\n", specialKeys[GLUT_KEY_DOWN]);
        specialKeys[GLUT_KEY_DOWN] = false;
        if (menuItem < 2)
            menuItem++;
        else
            menuItem = 0;
        //printf("i:%d\n", menuItem);
    }
    if (specialKeys[GLUT_KEY_UP]) {
        specialKeys[GLUT_KEY_UP] = false;
        if (menuItem != 0)
            menuItem--;
        else
            menuItem = 2;
    }
}


void update() {
    float dt = getDT();// if you have anything moving, use dt.

    switch(state) {
    case RUNNING:
    case PRELOSE:
        updateRunning(dt);
        break;
    case LOSE:
    case WIN:
        updateLose(dt);
        break;
    case MENU:
        updateMenu(dt);
        break;
    case SETTINGS:
        updateSettings(dt);
        break;
    }
    glutPostRedisplay();
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

void changeAngle(float x, float y)
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
    /*if(key == 27)//ESC
    {
        //exit(0);
        //cleanUp();
        //initialize();
        state = MENU;
        menuItem = 0;
    }*/
    //printf("%d\n", key);
    keys[key] = true;
}

void keyboardUp(unsigned char key, int x_pos, int y_pos)
{
    keys[key] = false;
}

void specialKey(int key, int x, int y)
{
    specialKeys[key] = true;
    //printf("%d\n", specialKeys[GLUT_KEY_DOWN]);
}

void specialKeyUp(int key, int x, int y)
{
    specialKeys[key] = false;
    //printf("%d\n", specialKeys[GLUT_KEY_DOWN]);
}

void passiveMotion(int x, int y)
{
    static int centerX = glutGet(GLUT_WINDOW_WIDTH) / 2;
    static int centerY = glutGet(GLUT_WINDOW_HEIGHT) / 2;
    static bool checkingForMotion = true;
    mouseX += centerX-x;
    mouseY += centerY-y;

    if (checkingForMotion)
    {
    	glutWarpPointer(centerX, centerY);
    }

    checkingForMotion = !checkingForMotion;
}

bool initialize()
{
    state = RUNNING;
    // Reset all of the keys
    for ( auto &key : specialKeys ) {
        key = false;
    }
    for ( auto &key : keys ) {
        key = false;
    }

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

    // Add the walls
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
        addWallTops(geometry, wallCorners);
    }

    // Place the start and finish
    endX = maze.getEndX();
    endY = maze.getEndY();
    addPanel(geometry, 0, 0, 0, 1, 0);
    addPanel(geometry, endX, endY, 1, 0, 0);

    // Make holes
    std::mt19937 gen(time(0));
    std::uniform_int_distribution<int> holePlaces(0,
            MazeParams::size*MazeParams::size-1);
    std::set<int> holeLocs;
    while(holeLocs.size() < holeCount) {
        int place = holePlaces(gen);
        int x = place/MazeParams::size;
        int y = place%MazeParams::size;
        if (!((x == 0 && y == 0) || (x == endX && y == endY)))
           holeLocs.insert(place);
    }
    // Put the holes in the floor
    addFloor(geometry, holeLocs);

    vertexCounts[0] = geometry.size();

    // Also, a sphere
    auto ballModel = makeSphere(glm::vec3{0,0,0}, ballRadius, 4,
            glm::vec3{0,0.0,0.8});
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
    mats::view = glm::lookAt( glm::vec3(0.0, 21.0, -21.0), //Eye Position
                        glm::vec3(0.0, 0.0, 0.0), //Focus point
                        glm::vec3(0.0, 1.0, 0.0)); //Positive Y is up

    mats::projection = glm::perspective( 45.0f, //the FoV typically 90 degrees is good which is what this is set to
                                   float(w)/float(h), //Aspect Ratio, so Circles stay Circular
                                   0.01f, //Distance to the near plane, normally a small value like this
                                   100.0f); //Distance to the far plane, 

    //enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Antialiasing
    glEnable(GL_MULTISAMPLE);

    // Allow better keyboard control
    glutIgnoreKeyRepeat(1);

    // Disable the cursor
    glutSetCursor(GLUT_CURSOR_NONE);

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

    ballY = -0.5f;


    //and its done
    return true;
}

void cleanUp()
{
    // Reset stuff
    angleX = 0.0f;
    angleZ = 0.0f;
    mouseX = 0;
    mouseY = 0;
    holes.clear();
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

void addPanel(std::vector<Vertex>& geometry, int x, int y, GLfloat r, GLfloat g,
        GLfloat b) {
    using namespace MazeParams;
    // The p is for panel
    GLfloat pLeft = left + x*cellHSize + wallThickness;
    GLfloat pRight = left + (x+1)*cellHSize - wallThickness;
    GLfloat pBottom = bottom + y*cellVSize + wallThickness;
    GLfloat pTop = bottom + (y+1)*cellVSize - wallThickness;
    geometry.push_back({{pLeft, -0.95f, pBottom}, {r, g, b}});
    geometry.push_back({{pRight, -0.95f, pBottom}, {r, g, b}});
    geometry.push_back({{pLeft, -0.95f, pTop}, {r, g, b}});
    geometry.push_back({{pRight, -0.95f, pTop}, {r, g, b}});
    geometry.push_back({{pLeft, -0.95f, pTop}, {r, g, b}});
    geometry.push_back({{pRight, -0.95f, pBottom}, {r, g, b}});
}

#ifndef CALLBACK
#define CALLBACK
#endif

std::vector<Vertex> wallTopVertices;
void CALLBACK addWallTopVertex(GLvoid *vertex) {
    Vertex v = *(Vertex*) vertex;
    wallTopVertices.push_back(v);
}

void CALLBACK combineCallback(GLdouble coords[3],
                     Vertex *vertex_data[4],
                     GLfloat weight[4], Vertex **dataOut )
{
   int i;
   Vertex *vertex = new Vertex;

   vertex->position[0] = coords[0];
   vertex->position[1] = coords[1];
   vertex->position[2] = coords[2];
   for (i = 0; i < 3; i++)
      vertex->color[i] = vertex_data[0]->color[i];
   if (*dataOut != nullptr)
       delete *dataOut;
   *dataOut = vertex;
}

// Needed to force triangles
void CALLBACK edgeCallback(){return;}

void addWallTops(std::vector<Vertex>& geometry, const std::vector<b2Vec2>&
        wallCorners) {
    using namespace MazeParams;
    std::vector<std::unique_ptr<Vertex>> innerVertices;
    std::vector<std::unique_ptr<double[]>> innerCoords;

    wallTopVertices.clear();

    // Run the tesselator
    auto tess = gluNewTess();
    gluTessCallback(tess, GLU_TESS_VERTEX, (GLvoid (*) ()) addWallTopVertex);
    gluTessCallback(tess, GLU_TESS_COMBINE, (GLvoid (*) ()) combineCallback);
    gluTessCallback(tess, GLU_TESS_EDGE_FLAG_DATA,
            (GLvoid (*) ()) edgeCallback);
    gluTessProperty(tess, GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_POSITIVE);

    gluTessBeginPolygon(tess, nullptr);

    // Add the outer walls
    gluTessBeginContour(tess);
    std::vector<std::vector<double>> outsideCoords{{left, 0.0, top},
        {right, 0.0, top},
        {right, 0.0, bottom},
        {left, 0.0, bottom}};
    for (auto coords : outsideCoords) {
        innerVertices.emplace_back(new Vertex{{float(coords[0]), float(coords[1]), float(coords[2])},
                         TOP_COLOR});
        gluTessVertex(tess, coords.data(), innerVertices.back().get());
    }
    gluTessEndContour(tess);

    // Add the inner walls
    gluTessBeginContour(tess);
    for (auto corner : wallCorners) {
        innerCoords.emplace_back(new double[3]);
        innerCoords.back()[0] = corner.x;
        innerCoords.back()[1] = 0;
        innerCoords.back()[2] = corner.y;
        innerVertices.emplace_back(
                new Vertex{{float(innerCoords.back()[0]),
                    float(innerCoords.back()[1]),
                    float(innerCoords.back()[2])},
                         TOP_COLOR});
        gluTessVertex(tess, innerCoords.back().get(), innerVertices.back().get());
    }
    gluTessEndContour(tess);

    gluTessEndPolygon(tess);
    gluDeleteTess(tess);

    // Shove the new vertices into the geometry
    geometry.insert(geometry.end(), wallTopVertices.begin(),
            wallTopVertices.end());
}

std::vector<Vertex> floorVertices;
void CALLBACK addFloorVertex(GLvoid *vertex) {
    Vertex v = *(Vertex*) vertex;
    floorVertices.push_back(v);
}

void addFloor(std::vector<Vertex>& geometry, const std::set<int>&
        holeLocs) {
    using namespace MazeParams;
    std::vector<std::unique_ptr<std::vector<GLdouble>>> innerPoints;
    std::vector<std::unique_ptr<Vertex>> innerVertices;

    // Run the tesselator
    auto tess = gluNewTess();
    gluTessCallback(tess, GLU_TESS_VERTEX, (GLvoid (*) ()) addFloorVertex);
    gluTessCallback(tess, GLU_TESS_COMBINE, (GLvoid (*) ()) combineCallback);
    gluTessCallback(tess, GLU_TESS_EDGE_FLAG_DATA,
            (GLvoid (*) ()) edgeCallback);
    gluTessProperty(tess, GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_NONZERO);

    gluTessBeginPolygon(tess, nullptr);

    // Add the floor
    gluTessBeginContour(tess);
    std::vector<std::vector<double>> outsideCoords{{left, -1.0, top},
        {right, -1.0, top},
        {right, -1.0, bottom},
        {left, -1.0, bottom}};
    for (auto coords : outsideCoords) {
        innerVertices.push_back(
                std::unique_ptr<Vertex>(
                        new Vertex{{float(coords[0]),
                                    float(coords[1]),
                                    float(coords[2])},
                                   FLOOR_COLOR}));
        gluTessVertex(tess, coords.data(), innerVertices.back().get());
    }
    gluTessEndContour(tess);

    // Punch out the holes
    for (auto holeLoc : holeLocs) {
        float centerX = left + cellHSize*(holeLoc/size+0.5f);
        float centerY = bottom + cellVSize*(holeLoc%size+0.5f);
        holes.emplace_back(centerX, centerY);

        gluTessBeginContour(tess);
        for (float angle = 0.0f; angle < M_PI*2; angle += M_PI/60.0f) {
            geometry.push_back(Vertex{{centerX+holeRadius*cosf(angle),
                                        -1.0,
                                        centerY+holeRadius*sinf(angle)},
                                       HOLE_COLOR});
            geometry.push_back(Vertex{{centerX+holeRadius*cosf(angle+M_PI/60.0f),
                                        -1.0,
                                        centerY+holeRadius*sinf(angle+M_PI/60.0f)},
                                       HOLE_COLOR});
            geometry.push_back(Vertex{{centerX+holeRadius*cosf(angle+M_PI/60.0f),
                                        -1.0-holeDepth,
                                        centerY+holeRadius*sinf(angle+M_PI/60.0f)},
                                       HOLE_COLOR});
            geometry.push_back(Vertex{{centerX+holeRadius*cosf(angle),
                                        -1.0,
                                        centerY+holeRadius*sinf(angle)},
                                       HOLE_COLOR});
            geometry.push_back(Vertex{{centerX+holeRadius*cosf(angle+M_PI/60.0f),
                                        -1.0-holeDepth,
                                        centerY+holeRadius*sinf(angle+M_PI/60.0f)},
                                       HOLE_COLOR});
            geometry.push_back(Vertex{{centerX+holeRadius*cosf(angle),
                                        -1.0-holeDepth,
                                        centerY+holeRadius*sinf(angle)},
                                       HOLE_COLOR});

            // Add tesselation vertex
            innerPoints.push_back(
                    std::unique_ptr<std::vector<GLdouble>>(
                            new std::vector<GLdouble>
                                      {centerX+holeRadius*cos(angle),
                                       -1.0,
                                       centerY+holeRadius*sin(angle)}));
            innerVertices.push_back(
                    std::unique_ptr<Vertex>(
                            new Vertex{{float((*innerPoints.back())[0]),
                                        float((*innerPoints.back())[1]),
                                        float((*innerPoints.back())[2])},
                                       FLOOR_COLOR}));

            //printf("%f,%f\n", x, y);
            gluTessVertex(tess, innerPoints.back().get()->data(), innerVertices.back().get());
        }
        gluTessEndContour(tess);
    }

    gluTessEndPolygon(tess);
    gluDeleteTess(tess);

    // Shove the new vertices into the geometry
    geometry.insert(geometry.end(), floorVertices.begin(),
            floorVertices.end());
    floorVertices.clear();
}


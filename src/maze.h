/*
 * maze.h
 *
 *  Created on: Oct 6, 2012
 *      Author: tommy
 */

#ifndef MAZE_H_
#define MAZE_H_

#include <vector>

class Maze {
public:
    enum direction { RIGHT, UP, LEFT, DOWN };
    static const int dx[4];/*={1,0,-1,0}*/
    static const int dy[4];/*={0,1,0,-1}*/

    Maze(int size);
    virtual ~Maze();
    bool getTop(int,int) const;
    bool getLeft(int,int) const;
    bool getRight(int,int) const;
    bool getBottom(int,int) const;
    void setTop(int,int,bool);
    void setLeft(int,int,bool);
    void setRight(int,int,bool);
    void setBottom(int,int,bool);
    void clear();
    void fill();
    int getSize() const;
    bool getDirection(int,int,direction);
    void setDirection(int,int,direction,bool);
    int getEndX() const;
    void setEndX(int endX);
    int getEndY() const;
    void setEndY(int endY);
    int getStartX() const;
    void setStartX(int startX);
    int getStartY() const;
    void setStartY(int startY);

private:
    std::vector<std::vector<bool> > openTop;
    std::vector<std::vector<bool> > openLeft;
    const static int offsetX[4]; // = {1,0,0,0};
    const static int offsetY[4]; // = {0,0,0,1};
    int startX;
    int startY;
    int endX;
    int endY;
};

Maze dfsBacktracker(int, int, int, int);

#endif /* MAZE_H_ */

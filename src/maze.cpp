/*
 * maze.cpp
 *
 *  Created on: Oct 6, 2012
 *      Author: tommy
 */

#include "maze.h"
#include <bitset>
#include <array>
#include <random>
#include <functional>

const int Maze::dx[4] = {1,0,-1,0};
const int Maze::dy[4] = {0,1,0,-1};
const int Maze::offsetX[4] = {1,0,0,0};
const int Maze::offsetY[4] = {0,0,0,1};

/**
 * Initializes the maze to some size
 */
Maze::Maze(int size) {
    for (int i = 0; i < size + 1; i++) {
        openTop.push_back(std::vector<bool>(size + 1));
        openLeft.push_back(std::vector<bool>(size + 1));
    }
}

Maze::~Maze() {
    // TODO Auto-generated destructor stub
}

bool Maze::getTop(int x, int y) const {
    return openTop[y+1][x];
}

bool Maze::getLeft(int x, int y) const {
    //printf("%d,%d", x, y);
    return openLeft[y][x];
}

bool Maze::getRight(int x, int y) const {
    return openLeft[y][x+1];
}

bool Maze::getBottom(int x, int y) const {
    return openTop[y][x];
}

void Maze::setTop(int x, int y, bool setting) {
    openTop[y+1][x] = setting;
}

void Maze::setLeft(int x, int y, bool setting) {
    openLeft[y][x] = setting;
}

void Maze::setRight(int x, int y, bool setting) {
    openLeft[y][x+1] = setting;
}

void Maze::setBottom(int x, int y, bool setting) {
    openTop[y][x] = setting;
}

void Maze::clear() {
    for (int y = 0; y < getSize()+1; y++) {
        for (int x = 0; x < getSize()+1; x++) {
            openTop[y][x] = true;
            openLeft[y][x] = true;
            if (y == 0 || y == getSize())
                openTop[y][x] = false;
            if (x == 0 || x == getSize())
                openLeft[y][x] = false;
        }
    }
}

void Maze::fill() {
    for (int i = 0; i < getSize()+1; i++) {
        for (int j = 0; j < getSize()+1; j++) {
            openTop[i][j] = false;
            openLeft[i][j] = false;
        }
    }
}

int Maze::getSize() const {
    return openTop.size()-1;
}

Maze dfsBacktracker(int size, int startX, int startY, int seed) {
    std::mt19937 gen(seed);
    Maze result(size);
    auto index =
        [size](int x, int y) {
            if (x >= 0 && x < size && y >= 0 && y < size)
                return int(x*size + y);
            else
                return size*size;
        };
    auto unindex =
        [size](int i) {
                return std::array<int,2>{{i/size, i%size}};
        };
    std::vector<int> moveStack;
    std::vector<bool> usedCells(size*size+1, false);
    // This is a dummy value that we use out of bounds
    usedCells[size*size] = true;
    moveStack.push_back(index(startX, startY));
    usedCells[index(startX,startY)] = true;

    result.fill();
    //printf("a:%d\n", result.getBottom(2,0));
    while (!moveStack.empty()) {
        std::vector<int> nearby;
        std::array<int,2> pos = unindex(moveStack.back());
        if (!usedCells[index(pos[0]+1, pos[1])] &&
                pos[0] != size)
            nearby.push_back(0);
        if (!usedCells[index(pos[0]-1, pos[1])] &&
                pos[0] != 0)
            nearby.push_back(1);
        if (!usedCells[index(pos[0], pos[1]+1)] &&
                pos[1] != size)
            nearby.push_back(2);
        if (!usedCells[index(pos[0], pos[1]-1)] &&
                pos[1] != 0)
            nearby.push_back(3);
        if (nearby.empty())
            moveStack.pop_back();
        else {
            int i = std::uniform_int_distribution<int>(0, nearby.size()-1)(gen);
            //printf("%d,%d,%d\n", pos[0], pos[1], nearby[i]);
            //printf("%d\n",nearby[i]);
            switch(nearby[i]) {
            case 0:
                result.setRight(pos[0], pos[1], true);
                moveStack.push_back(index(pos[0]+1, pos[1]));
                usedCells[index(pos[0]+1, pos[1])] = true;
                break;
            case 1:
                result.setLeft(pos[0], pos[1], true);
                moveStack.push_back(index(pos[0]-1, pos[1]));
                usedCells[index(pos[0]-1, pos[1])] = true;
                break;
            case 2:
                result.setTop(pos[0], pos[1], true);
                moveStack.push_back(index(pos[0], pos[1]+1));
                usedCells[index(pos[0], pos[1]+1)] = true;
                break;
            case 3:
                result.setBottom(pos[0], pos[1], true);
                moveStack.push_back(index(pos[0], pos[1]-1));
                usedCells[index(pos[0], pos[1]-1)] = true;
                break;
            }
        }
    }
    //printf("a:%d\n", result.getBottom(2,0));
    return result;
}

bool Maze::getDirection(int x, int y, direction dir) {
    switch (dir) {
    case UP:
        return getTop(x,y);
    case LEFT:
        return getLeft(x,y);
    case RIGHT:
        return getRight(x,y);
    case DOWN:
        return getBottom(x,y);
    }
}

void Maze::setDirection(int x, int y, direction dir, bool setting) {
    switch (dir) {
    case UP:
        setTop(x,y,setting);
        break;
    case LEFT:
        setLeft(x,y,setting);
        break;
    case RIGHT:
        setRight(x,y,setting);
        break;
    case DOWN:
        setBottom(x,y,setting);
        break;
    }
}

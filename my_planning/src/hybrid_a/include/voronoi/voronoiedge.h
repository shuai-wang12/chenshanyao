#pragma once

#include "voronoi/dynamicvoronoi.h"

class voronoiedge
{
public:
    struct node{
        int parentx;
        int parenty;
        int status=0;
        double dist;
        node* parent;
    };
    typedef node* nodePtr;
    enum{
        open=0,
        close
    };
    int sizeX;
    int sizeY;
    nodePtr** nodeMap;
    
public:
    voronoiedge(){};
    ~voronoiedge(){}
    void initializeMap(int width,int height);
    void calVEdgeDist(bool** binMap);
    inline double getVEdgeDist(int x,int y){
        return nodeMap[x][y]->dist;
    }
    void visualize(const char *filename="result2.pgm");
};


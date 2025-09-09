#include "voronoi/voronoiedge.h"

#include <math.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <queue>

const int dx[4] = {0, 0, 1, -1};
const int dy[4] = {1, -1, 0, 0};

void voronoiedge::initializeMap(int width, int height) {
  sizeX = width;
  sizeY = height;
  nodeMap = new nodePtr*[sizeX];
  for (int i = 0; i < sizeX; i++) {
    nodeMap[i] = new nodePtr[sizeY];
    for (int j = 0; j < sizeY; j++) {
      nodeMap[i][j] = new node{0, 0, open, 0, nullptr};
    }
  }
}

void voronoiedge::calVEdgeDist(DynamicVoronoi& voronoiDiagram) {
  std::queue<std::pair<int, int>> q;
  for (int i = 0; i < sizeX; i++) {
    for (int j = 0; j < sizeY; j++) {
      if (voronoiDiagram.isVoronoi(i, j)) {
        q.push(std::make_pair(i, j));
        nodeMap[i][j]->parentx = i;
        nodeMap[i][j]->parenty = j;
        nodeMap[i][j]->parent = nullptr;
        nodeMap[i][j]->status = close;
        nodeMap[i][j]->dist = 0;
      } else if (voronoiDiagram.isOccupied(i, j)) {
        nodeMap[i][j]->parentx = i;
        nodeMap[i][j]->parenty = j;
        nodeMap[i][j]->parent = nullptr;
        nodeMap[i][j]->status = close;
        nodeMap[i][j]->dist = 0;
      }
    }
  }

  while (!q.empty()) {
    int x = q.front().first;
    int y = q.front().second;
    q.pop();
    //上下左右遍历即可，已经遍历的不需要再遍历
    for (int i = 0; i < 4; i++) {
      int nx = x + dx[i];
      int ny = y + dy[i];
      if (nx < 0 || nx >= sizeX || ny < 0 || ny >= sizeY ||
          nodeMap[nx][ny]->status == close)
        continue;
      nodeMap[nx][ny]->parentx = nodeMap[x][y]->parentx;
      nodeMap[nx][ny]->parenty = nodeMap[x][y]->parenty;
      nodeMap[nx][ny]->parent = nodeMap[x][y];
      nodeMap[nx][ny]->status = close;
      nodeMap[nx][ny]->dist =
          hypot(nodeMap[x][y]->parentx - nx, nodeMap[x][y]->parenty - ny);
      q.push(std::make_pair(nx, ny));
    }
  }
}

void voronoiedge::visualize(const char* filename) {
  // write pgm files
  std::ofstream outfile(filename);
  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  char* path;
  if ((path = getcwd(NULL, 0)) == NULL) {
    std::cerr << "Error message : _getcwd error" << std::endl;
  } else {
    std::cout << path << std::endl;
  }

  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  for (int y = sizeY - 1; y >= 0; y--) {
    for (int x = 0; x < sizeX; x++) {
      unsigned char c = 0;
      double f = nodeMap[x][y]->dist;
      if (f > 255) f = 255;
      if (f < 0) f = 0;
      c = (unsigned char)f;
      fputc(c, F);
      fputc(c, F);
      fputc(c, F);
    }
  }
  fclose(F);
}

#include <GL/glew.h>
#include <GL/glut.h>

#include <IL/il.h>
#include <IL/ilut.h>

#include <vector>
#include <fstream>
#include <float4x4.h>
#include <float3x3.h>
#include "Raytracer.h"

int meshcount = 0;

int getFreshMeshNumber() {
  meshcount++;
  return meshcount;
}

void translateMesh(float x, float y, float z, int meshid, std::vector<polygon>& polygons,   std::vector<float3>& vertex) {
  float4x4 transmatrix = make_translation(make_vector(x,y,z));
  for(int i=0;i<polygons.size();i++) {
    if(polygons[i].mesh==meshid) {
      for(int j=0;j<4;j++){
        float4 newpos = transmatrix * make_vector(vertex[polygons[i].indices[j]].x, vertex[polygons[i].indices[j]].y, vertex[polygons[i].indices[j]].z ,1.0f);
        vertex[polygons[i].indices[j]] = make_vector(newpos.x,newpos.y,newpos.z);
      }
    }
  }
}

void rotateMesh(float x, float y, float z, int meshid, std::vector<polygon>& polygons, std::vector<float3>& vertex) {
  float3x3 rotmatrix = make_rotation_x<float3x3>(x) * make_rotation_y<float3x3>(y) * make_rotation_z<float3x3>(z);
  for(int i=0;i<polygons.size();i++) {
    if(polygons[i].mesh==meshid) {
      for(int j=0;j<4;j++){
        vertex[polygons[i].indices[j]] = rotmatrix * make_vector(vertex[polygons[i].indices[j]].x, vertex[polygons[i].indices[j]].y, vertex[polygons[i].indices[j]].z);
        //polygons[i].normal = rotmatrix * polygons[i].normal;
      }
    }
  }
}

void rotateLight(float x, float y, float z, light& l) {
  float3x3 rotmatrix = make_rotation_x<float3x3>(x) * make_rotation_y<float3x3>(y) * make_rotation_z<float3x3>(z);
  l.pos = rotmatrix * make_vector(l.pos.x, l.pos.y, l.pos.z);
}

void moveLightPos(float x, float y, float z, light& l) {
  l.pos += make_vector(x, y, z);
}

GLuint generateNoiseTexture() {
  GLuint texture;
  int size=1024;
  float* data = new float[size*size*3];
  srand(time(NULL));
  for(int i=0;i<size*size*3;i++) {
    data[i] = ((float)(rand()%100)/100);
  }
 
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, size, size, 0, GL_RGB, GL_FLOAT, data);
  glBindTexture(GL_TEXTURE_2D, texture);
  delete data;
  return texture;
}

int loadModel(const char* filename, std::vector<polygon>& polygons, std::vector<float3>& vertex) {
  std::vector<std::string*> coord;
  std::vector<float3> normals;
  std::ifstream ins(filename);
  int offset = vertex.size();
  int offsetn = normals.size();
  if(!ins.is_open())
  {
    printf("File is open");
    return -1;
  }
  char buf[256];
  while(!ins.eof())
  {
    ins.getline(buf,256);
    coord.push_back(new std::string(buf));
  }
  
  int meshid = getFreshMeshNumber();
  polygon p;
  for(int i=0;i<coord.size();i++)
  {
    if(coord[i]->c_str()[0]=='#')  {
      continue;
    } else if(coord[i]->c_str()[0]=='v' && coord[i]->c_str()[1]==' ') {
      float x, y, z;
      sscanf_s(coord[i]->c_str(),"v %f %f %f",&x,&y,&z);
      vertex.push_back(make_vector(x, y, z));
    } else if(coord[i]->c_str()[0]=='v' && coord[i]->c_str()[1]=='n') {
      float x, y, z;
      sscanf_s(coord[i]->c_str(),"v %f %f %f",&x,&y,&z);
      normals.push_back(make_vector(x, y, z));   
    }else if(coord[i]->c_str()[0]=='f') {
      int v[4], n[4], t[4];
      if(count(coord[i]->begin(),coord[i]->end(),' ')==4) {
        sscanf_s(coord[i]->c_str(),"f %d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d",&v[0],&n[0],&t[0],&v[1],&n[1],&t[1],&v[2],&n[2],&t[2],&v[3],&n[3],&t[3]);
        p.type = 1;
        p.color = make_vector(0.0f,0.0f,0.6f,1.0f);
        p.mesh = meshid;
        for(int j=0;j<4;j++) {
          p.indices[j] = offset + v[j]-1;
        }
        polygons.push_back(p);
      }
    }
  }
  return meshid;
}
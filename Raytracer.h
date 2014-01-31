#include <float4.h>
#include <float3.h>
#include <time.h>

using namespace chag;

struct polygon {
	int mesh;
  float4 color;
  int type;
	int indices[4];
  int normals[4];
	float reflect;
  int texture;
};

struct light {
  float4 color;
  float3 pos;
};

int getFreshMeshNumber();

void translateMesh(float x, float y, float z, int meshid, std::vector<polygon>& polygons,   std::vector<float3>& vertex);
void rotateMesh(float x, float y, float z, int meshid, std::vector<polygon>& polygons, std::vector<float3>& vertex);
void rotateLight(float x, float y, float z, light& l);
void moveLightPos(float x, float y, float z, light& l);
GLuint generateNoiseTexture();
int loadModel(const char* filename, std::vector<polygon>& polygons, std::vector<float3>& vertex);
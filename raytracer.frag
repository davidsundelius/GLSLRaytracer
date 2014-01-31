#version 130

out vec4 fragmentColor;

uniform vec2 resolution;

uniform vec3 camPos;
uniform vec2 camRot;

uniform sampler2D ntex;

#define SPHERE   0
#define QUAD	 1
#define TRIANGLE 2
#define MAX_VERTICES 150
#define MAX_POLYGONS 100
#define MAX_LIGHTS 5
#define PI 3.14159268

#define AMBIENT 0.5
#define DIFFUSE 0.8
#define SPECULAR 0.5
#define SHINEINESS 50.0
//Datastructures

struct Settings {
	int numSamples;
	int numShadowSamples;
	bool textures;
};

//Polygondata
struct Polygons {
	vec4 color [MAX_POLYGONS];
	int type [MAX_POLYGONS];
	ivec4 indices [MAX_POLYGONS];
	vec3 vertices [MAX_VERTICES];
	float reflect [MAX_POLYGONS];
};

struct Lights {
	vec4 color[MAX_LIGHTS];
	vec3 pos[MAX_LIGHTS];
};

struct Result {
	vec4 color;
	bool hit;
	vec3 hitPoint;
	vec3 normal;
	float distance;
};

//Objects in use
uniform Settings settings;
uniform int numPolygons;
uniform int numLights;
uniform Lights lights;
uniform Polygons polygons;
uniform sampler2D spheretex;
uniform sampler2D quadtex;
uniform sampler2D moontex;


Result res;

//Math
mat4 makeTranslation(vec3 pos){
	return mat4(1, 0, 0, pos.x,
				0, 1, 0, pos.y,
				0, 0, 1, pos.z,
				0, 0, 0, 1		);
}

vec3 rotateX(vec3 vec, float r) {
	mat3 rotMat = mat3(1, 0,       0,
					   0, cos(r), sin(r),
					   0, -sin(r),  cos(r));
	return rotMat*vec;
}

vec3 rotateY(vec3 vec, float r) {
	mat3 rotMat = mat3(cos(r), 0, -sin(r),
					   0,      1,  0,
					   sin(r), 0,  cos(r));
	return rotMat*vec;
}

vec3 rotateZ(vec3 vec, float r) {
	mat3 rotMat = mat3(cos(r),  sin(r), 0,
					   -sin(r), cos(r), 0,
					   0,       0,      1);
	return rotMat*vec;
}

vec3 rotate(vec3 vec, vec3 rot) {
	vec = rotateX(vec, rot.x);
	vec = rotateY(vec, rot.y);
	return rotateZ(vec, rot.z);
}

float atan2 (float x, float y) {
	return 2*atan((sqrt(x*x+y*y)-x)/y);
}

//Tracing functions
void intersectSphere(vec3 ray, vec3 camera, int p) {
	vec3 rv = polygons.vertices[polygons.indices[p].x] - polygons.vertices[polygons.indices[p].y];
	vec3 rs = camera - polygons.vertices[polygons.indices[p].x];
	float r = sqrt(dot(rv, rv));
	float b = dot(rs, ray);
	float c = dot(rs, rs) - r;
	float d = b*b - c;
	if(d > 0.0) {
		float t = -b - sqrt(d);
		if(t > 1.0 && (t < res.distance)) {
			res.hit=true;
			res.distance=t;
			res.hitPoint = camera + t*ray;
			vec3 cs = normalize(polygons.vertices[polygons.indices[p].x] - res.hitPoint);
			float u = 0.5-atan2(cs.z,cs.x)/(2*PI);
			float v = 0.5-2*asin(cs.y)/(2*PI);
			if(settings.textures) {
				if(p==1) res.color = texture2D(spheretex, vec2(u,v));
				else res.color = texture2D(moontex, vec2(u,v));
			} else {
				res.color = polygons.color[p];
			}
			res.normal = normalize(res.hitPoint);
		}
	}
}

void intersectTriangle(vec3 ray, vec3 camera, ivec3 inds, int p, bool swt) {
    vec3 u, v, n;
    vec3 w0, w;
    float r, a, b;
	int i1 = inds.x;
	int i2 = inds.y;
	int i3 = inds.z;

    u = polygons.vertices[i2] - polygons.vertices[i1];
    v = polygons.vertices[i3] - polygons.vertices[i1];
    n = cross(u, v);

    w0 = camera - polygons.vertices[i1];
    a = -dot(n, w0);
    b = dot(n, ray);
    if (abs(b) < 1e-3) return;

    r = a / b;
    if (r < 0.0) return;

    vec3 I = camera + r * ray;
    float uu, uv, vv, wu, wv, D;
    uu = dot(u, u);
    uv = dot(u, v);
    vv = dot(v, v);
    w = I - polygons.vertices[i1];
    wu = dot(w, u);
    wv = dot(w, v);
    D = uv * uv - uu * vv;

    float s, t;
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)
        return;
    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)
        return;

	if (r > 1e-3 && (r < res.distance)) {
		res.hit = true;
		if(settings.textures) {
			if(swt) {
				res.color = texture2D(quadtex, vec2(s+t,t));
				res.normal = normalize(-n+(normalize(u))*s+(normalize(v))*t);
			} else {
				res.color = texture2D(quadtex, vec2(t,s+t));
				res.normal = normalize(n+(normalize(u))*s+(normalize(v))*t);
			}
		} else {
			if(swt) {
				res.normal = normalize(-n+(normalize(u))*s+(normalize(v))*t);
			} else {
				res.normal = normalize(n+(normalize(u))*s+(normalize(v))*t);
			}
			res.color = polygons.color[p];
		}
		res.distance = r;
		res.hitPoint = camera + r*ray;
		
	}
}

void intersectQuad(vec3 ray, vec3 camera, int p) {
	intersectTriangle(ray, camera, ivec3(polygons.indices[p].x,  polygons.indices[p].y, polygons.indices[p].z), p, false);
	intersectTriangle(ray, camera, ivec3(polygons.indices[p].x,  polygons.indices[p].w, polygons.indices[p].z), p, true);
}

Result intersectObject(vec3 ray, vec3 camera) {
	res.distance = 1000;
	res.hit = false;
	for(int j = 0; j < numPolygons; j++) {
		if(polygons.type[j] == TRIANGLE) {
			intersectTriangle(ray, camera, ivec3(polygons.indices[j]), j, false);
		} else if(polygons.type[j] == QUAD){
			intersectQuad(ray, camera, j);
		} else if(polygons.type[j] == SPHERE) {
			intersectSphere(ray, camera, j);
		}
	}
	return res;
}

//Computation
void fog() {
	float alpha = res.distance/3;
	res.color -= vec4(alpha, alpha, alpha, 0.0);
}

vec4 light(Result orig, vec3 ray, vec3 camera, int light) {
	vec4 color;
	float theta, phi;
	vec4 ambient = res.color*AMBIENT;
	
	vec3 lv = normalize(lights.pos[light]-res.hitPoint);
	theta = dot(orig.normal, lv);

	//if(asin(theta)<0) return color;

	vec3 rv = reflect(lv, orig.normal);
	phi = dot(rv, normalize(ray));

	vec4 diffuse = ((res.color+lights.color[light])/2)*DIFFUSE*theta;
	color = vec4(ambient.xyz+diffuse.xyz,orig.color.w); 

	if(phi > 0) {
		vec4 spec = vec4(1.0, 1.0, 1.0, 0.0) * SPECULAR * pow(phi, SHINEINESS);
		color +=spec;
	}
	return color;
}

Result shadow(Result orig) {
	Result tmp = orig;
	float scaling = pow(0.2, float(1)); //numLights
	bool hit=false;
	vec3 lightcoord;
	vec4 dec;
	float c = texture2D(ntex, vec2(gl_FragCoord.x/resolution.x,gl_FragCoord.y/resolution.y)).x;
	vec2 uv;
	//for(int i=0;i<1;i++) {
		for(int j=0;j<settings.numShadowSamples;j++) {
			uv = vec2(c*(j+1)/float(settings.numShadowSamples-1), c*(j+1)/float(settings.numShadowSamples-1)); //j/float(settings.numShadowSamples) 
			lightcoord = (texture2D(ntex, uv).rgb); //- vec3(0.5,0.5,0.5));

			tmp = intersectObject(normalize(lights.pos[0]+lightcoord-orig.hitPoint),orig.hitPoint);
			if(tmp.hit) {
				hit = true;
				dec += vec4(orig.color.rgb*scaling,0.0);
			}

		}
	//}
	if (hit) {
		dec/=settings.numShadowSamples;
		orig.color -= vec4(dec.rgb, 0.0);
	}
	return orig;
}

//Shader program main
void main()
{
	vec3 camera = vec3(0, 0, 0);
	camera += camPos;

	vec3 ray;
	Result r;
	vec4 color = vec4 (0.0, 0.0, 0.0, 0.0);
	vec3 lookat[10];
	for(int i=0;i<settings.numSamples;i++) {
		lookat[i] = vec3(gl_FragCoord.x-((texture2D(ntex, vec2(gl_FragCoord.x*i/5000,gl_FragCoord.y*i/3000)).x)-1)*3-(resolution.x/2), gl_FragCoord.y-((texture2D(ntex, vec2(gl_FragCoord.x*1/5000,gl_FragCoord.y*i/3000)).y)-1)*3-(resolution.y/2), 800.0);
	}
	
	int hits=0;
	vec3 tmplookat;
	for(int i=0;i<settings.numSamples;i++) {
		tmplookat = rotate(lookat[i],vec3(camRot.x,camRot.y,0.0f)) + camPos;
		ray = normalize(tmplookat - camera);
		r = intersectObject(ray, camera);
		if(r.hit) {
			vec4 tmp = r.color;
			for(int j=0;j<numLights;j++) {
				tmp += light(r, ray, camera, j);
			}
			r.color = tmp; //vec4( / numLights,r.color.w);
			hits++;
			r = shadow(r);
			color+=r.color;
		} else {
			r.color = vec4(0.0, 0.0, 0.0, 0.0);
		}
	}
	if(hits!=0) color /= hits;

	/*if(hits!=settings.numSamples) {
		color.w *= (float(hits)/float(settings.numSamples));
	}*/
	vec3 noisecolor = texture2D(ntex, vec2(gl_FragCoord.x/float(resolution.x),gl_FragCoord.y/float(resolution.y))).rgb;
	fragmentColor = color; //vec4(0.0f, (gl_FragCoord.y), 0.0, gl_FragCoord.x/1000.0); 
}
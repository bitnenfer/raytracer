#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <time.h>

#if _DEBUG
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define SAMPLE_COUNT 1
#else
#define IMAGE_WIDTH 1024
#define IMAGE_HEIGHT 768
#define SAMPLE_COUNT 800
#endif
#define IMAGE_PIXEL_SIZE 3
#define IMAGE_SIZE (IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_PIXEL_SIZE)
#define MAX_DIST 100.0f
#define MIN_DIST 0.001f
#define RAYMARCH_STEPS 500
#define RAY_BOUNCE_COUNT 3
#define THREADED 1

#if defined(_WIN32)
#include <stdarg.h>
#include <Windows.h>
CRITICAL_SECTION gCS;
void LOG(const char* pFmt, ...)
{
	EnterCriticalSection(&gCS);
	static char buffer[512 * 3];
	static int bufferIdx = 0;
	va_list vl;
	va_start(vl, pFmt);
	vsprintf_s(&buffer[bufferIdx * 512], 512, pFmt, vl);
	va_end(vl);
	OutputDebugStringA(&buffer[bufferIdx * 512]);
	LeaveCriticalSection(&gCS);
}
void MsgBox(const char* pTitle, const char* pFmt, ...)
{
	EnterCriticalSection(&gCS);
	static char buffer[512 * 3];
	static int bufferIdx = 0;
	va_list vl;
	va_start(vl, pFmt);
	vsprintf_s(&buffer[bufferIdx * 512], 512, pFmt, vl);
	va_end(vl);
	MessageBoxA(NULL, &buffer[bufferIdx * 512], pTitle, MB_OK | MB_ICONINFORMATION);
	LeaveCriticalSection(&gCS);
}
#else
#define LOG printf
#define MsgBox(a, b, ...)
#endif

// Vector 3
typedef struct _vec3
{
	float x, y, z;
} vec3;
vec3 vec3_make(float x, float y, float z)
{
	vec3 v = { x, y, z };
	return v;
}
vec3 vec3_make2(float x)
{
	vec3 v = { x, x, x };
	return v;
}
vec3 vec3_add(const vec3& __restrict a, const vec3& b)
{
	vec3 c;
	c.x = a.x + b.x;
	c.y = a.y + b.y;
	c.z = a.z + b.z;
	return c;
}
vec3 vec3_add2(const vec3& __restrict a, float x, float y, float z)
{
	vec3 c;
	c.x = a.x + x;
	c.y = a.y + y;
	c.z = a.z + z;
	return c;
}
vec3 vec3_sub(const vec3& __restrict a, const vec3& __restrict b)
{
	vec3 c;
	c.x = a.x - b.x;
	c.y = a.y - b.y;
	c.z = a.z - b.z;
	return c;
}
vec3 vec3_mul(const vec3& __restrict a, const vec3& __restrict b)
{
	vec3 c;
	c.x = a.x * b.x;
	c.y = a.y * b.y;
	c.z = a.z * b.z;
	return c;
}
vec3 vec3_div(const vec3& __restrict a, const vec3& __restrict b)
{
	vec3 c;
	c.x = a.x / b.x;
	c.y = a.y / b.y;
	c.z = a.z / b.z;
	return c;
}
vec3 vec3_scalar_add(const vec3& __restrict a, float b)
{
	vec3 c;
	c.x = a.x + b;
	c.y = a.y + b;
	c.z = a.z + b;
	return c;
}
vec3 vec3_scalar_sub(const vec3& __restrict a, float b)
{
	vec3 c;
	c.x = a.x - b;
	c.y = a.y - b;
	c.z = a.z - b;
	return c;
}
vec3 vec3_scalar_mul(const vec3& __restrict a, float b)
{
	vec3 c;
	c.x = a.x * b;
	c.y = a.y * b;
	c.z = a.z * b;
	return c;
}
vec3 vec3_scalar_div(const vec3& __restrict a, float b)
{
	vec3 c;
	c.x = a.x / b;
	c.y = a.y / b;
	c.z = a.z / b;
	return c;
}
float vec3_dot(const vec3& __restrict a, const vec3& __restrict b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
float vec3_length(const vec3& a)
{
	return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}
vec3 vec3_cross(const vec3& __restrict a, const vec3& __restrict b)
{
	vec3 c;
	c.x = a.y * b.z - a.z - b.y;
	c.y = a.z * b.x - a.x * b.z;
	c.z = a.x * b.y - a.y * b.x;
	return c;
}
vec3 vec3_reflect(const vec3& __restrict a, const vec3& __restrict b)
{
	float dotBA2 = 2.0f * vec3_dot(b, a);
	vec3 dotBA2B = vec3_scalar_mul(b, dotBA2);
	return vec3_sub(a, dotBA2B);
}
vec3 vec3_refract(const vec3& __restrict a, const vec3& __restrict b, float eta)
{
	float dotBA = vec3_dot(b, a);
	float k = 1.0f - eta * eta * (1.0f - dotBA * dotBA);
	if (k < 0.0f)
	{
		vec3 zero = { 0.0f, 0.0f, 0.0f };
		return zero;
	}
	vec3 etaA = vec3_scalar_mul(a, eta);
	float etaDotBASqrtK = eta * dotBA + sqrtf(k);
	vec3 etaDotBASqrtKB = vec3_scalar_mul(b, etaDotBASqrtK);
	return vec3_sub(etaA, etaDotBASqrtKB);
}
vec3 vec3_normalize(const vec3& __restrict a)
{
	float length = vec3_length(a);
	if (length != 0.0f) return vec3_scalar_div(a, length);
	return a;
}
vec3 vec3_abs(const vec3& a)
{
	vec3 b = { fabsf(a.x), fabsf(a.y), fabsf(a.z) };
	return b;
}
vec3 vec3_max(const vec3& __restrict a, const vec3& __restrict b)
{
	vec3 c = { max(a.x, b.x), max(a.y, b.y), max(a.z, b.z) };
	return c;
}
vec3 vec3_min(const vec3& __restrict a, const vec3& __restrict b)
{
	vec3 c = { min(a.x, b.x), min(a.y, b.y), min(a.z, b.z) };
	return c;
}
void vec3_rotate_axis(float& __restrict axisX, float& __restrict axisY, float r)
{
	float c = cosf(r);
	float s = sinf(r);
	float x = axisX;
	float y = axisY;
	float vx = x * c - y * s;
	float vy = x * s + y * c;
	axisX = vx;
	axisY = vy;
}
vec3 vec3_mod(const vec3& __restrict a, const vec3& __restrict b)
{
	vec3 c;
	c.x = fmodf(a.x, b.x);
	c.y = fmodf(a.y, b.y);
	c.z = fmodf(a.z, b.z);
	return c;
}
vec3 vec3_mod2(const vec3& __restrict a, const float& __restrict b)
{
	vec3 c;
	c.x = fmodf(a.x, b);
	c.y = fmodf(a.y, b);
	c.z = fmodf(a.z, b);
	return c;
}

const vec3 kResolution = { IMAGE_WIDTH, IMAGE_HEIGHT, 1.0f };
const vec3 kAspectRatio = { (float)IMAGE_WIDTH / (float)IMAGE_HEIGHT, 1.0f, 1.0f };
const vec3 kVOffsetX = { MIN_DIST, 0.0f, 0.0f };
const vec3 kVOffsetY = { 0.0f, MIN_DIST, 0.0f };
const vec3 kVOffsetZ = { 0.0f, 0.0f, MIN_DIST };
vec3 kLightDir = { 0.8f, 1.0f, 0.25f };

typedef enum _hit_type
{
	HIT_NONE,
	HIT_SPHERE,
	HIT_SPHERE_RED,
	HIT_FLOOR,
	HIT_BOX
} hit_type;

typedef struct _ray_result
{
	vec3 position;
	vec3 normal;
	float depth;
	hit_type type;
} ray_result;

typedef struct _sdf_result
{
	float depth;
	hit_type type;
} sdf_result;

const sdf_result& sdf_union(const sdf_result& __restrict a, const sdf_result& __restrict b)
{
	return a.depth < b.depth ? a : b;
}

float sdf_sphere(const vec3& __restrict rayPoint, float r)
{
	return vec3_length(rayPoint) - r;
}

float sdf_box(const vec3& __restrict  rayPoint, const vec3& __restrict  size)
{
	vec3 d = vec3_sub(vec3_abs(rayPoint), size);
	vec3 vmax = vec3_max(d, vec3_make(0.0f, 0.0f, 0.0f));
	float vmaxLength = vec3_length(vmax);
	return vmaxLength + min(max(d.x, max(d.y, d.z)), 0.0f);
}

sdf_result sdf_map_scene(const vec3& __restrict  rayPoint)
{
	vec3 bp = vec3_add2(rayPoint, -0.8f, 0.053f, 0.4f);
	vec3_rotate_axis(bp.x, bp.z, 3.15f / 4.0f);
	float a = sdf_sphere(vec3_add2(rayPoint, 0.0f, -0.15f, 0.0f), 0.5f);
	float b = sdf_sphere(vec3_add2(rayPoint, 0.75f, 0.05f, 0.3f), 0.3f);
	float c = sdf_sphere(vec3_add2(rayPoint, 0.2f, 0.2f, 0.7f), 0.1f);
	float d = sdf_box(bp, vec3_make(0.20f, 0.20f, 0.20f)) - 0.05f;
	float e = rayPoint.y + 0.3f;
	float f = sdf_box(rayPoint, vec3_make(1.4f, 1.2f, 1.0f));
	float g = sdf_box(rayPoint, vec3_make(1.2f, 1.0f, 1.3f));
	vec3 rep = { 0.0f, 0.0f, 0.3f };
	vec3 hp = vec3_sub(vec3_mod(vec3_add(rayPoint, vec3_make(0.0f, 0.0f, -0.8f)), rep), vec3_scalar_mul(rep, -0.5));
	float h = sdf_box(hp, vec3_make(0.1f, 1.4f, 0.1f));

	f = max(f, -g);
	f = max(f, -h);

	sdf_result scene;
	sdf_result sdfA = { a, HIT_SPHERE };
	sdf_result sdfB = { b, HIT_SPHERE_RED };
	sdf_result sdfC = { c, HIT_SPHERE };
	sdf_result sdfD = { d, HIT_BOX };
	sdf_result sdfE = { e, HIT_FLOOR };
	sdf_result sdfF = { f, HIT_FLOOR };
	
	scene = sdf_union(sdfA, sdfB);
	scene = sdf_union(scene, sdfC);
	scene = sdf_union(scene, sdfD);
	scene = sdf_union(scene, sdfE);
	scene = sdf_union(scene, sdfF);
	
	return scene;
}

vec3 sdf_get_normal(const vec3& __restrict rayPoint)
{
	float d = sdf_map_scene(rayPoint).depth;
	float x = sdf_map_scene(vec3_sub(rayPoint, kVOffsetX)).depth;
	float y = sdf_map_scene(vec3_sub(rayPoint, kVOffsetY)).depth;
	float z = sdf_map_scene(vec3_sub(rayPoint, kVOffsetZ)).depth;
	vec3 n = { x, y, z };
	vec3 vd = { d, d, d };
	n = vec3_sub(vd, n);
	return vec3_normalize(n);
}

ray_result sdf_ray_march(const vec3& __restrict rayOrigin, const vec3& __restrict rayDirection)
{
	float totalDistance = 0.0f;
	for (int i = 0; i < RAYMARCH_STEPS; ++i)
	{
		vec3 rayPoint = vec3_add(rayOrigin, vec3_scalar_mul(rayDirection, totalDistance));
		sdf_result res = sdf_map_scene(rayPoint);
		if (res.depth < MIN_DIST)
		{
			ray_result result;
			result.position = rayPoint;
			result.normal = sdf_get_normal(rayPoint);
			result.depth = totalDistance;
			result.type = res.type;
			return result;
		}
		totalDistance += res.depth;
		if (totalDistance > MAX_DIST)
			break;
	}
	ray_result noHit;
	noHit.type = HIT_NONE;
	return noHit;
}


float get_random_value()
{
#if 0
	float value = ((float)rand() / (float)RAND_MAX);
	return value;
#else
	static unsigned int seed = 0x12345678;
	unsigned int x = seed;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	seed = x;
	float value = (float)seed / (float)UINT_MAX;
	return value;
#endif
}

vec3 random_hemisphere(const float& __restrict r1, const float& __restrict r2)
{
	float r = sqrtf(r1);
	float theta = 2.0f * 3.15f * r2;
	float x = r * cosf(theta);
	float y = r * sinf(theta);
	return vec3_make(x, sqrtf(max(0.0f, 1.0f - r1)), y);
}

vec3 trace(vec3& __restrict  rayOrigin, vec3& __restrict rayDirection)
{
	vec3 color = { 0.0f, 0.0f, 0.0f };
	float attenuation = 1.0f;
	for (int bounce = 0; bounce < RAY_BOUNCE_COUNT; ++bounce)
	{
		ray_result ray = sdf_ray_march(rayOrigin, rayDirection);
		float x = get_random_value();
		float y = get_random_value();
		vec3 rh = random_hemisphere(x, y);
		float diff = max(0.0f, vec3_dot(ray.normal, kLightDir));

		if (ray.type == HIT_SPHERE)
		{
			rayDirection = vec3_normalize(vec3_reflect(rayDirection, vec3_add(ray.normal, vec3_scalar_mul(rh, 0.6f))));
			rayOrigin = vec3_add(ray.position, vec3_scalar_mul(rayDirection, 0.01f));
			attenuation *= 0.9f;
		}
		else if (ray.type == HIT_SPHERE_RED)
		{
			rayDirection = vec3_normalize(vec3_reflect(rayDirection, vec3_add(ray.normal, vec3_scalar_mul(rh, 0.1f))));
			rayOrigin = vec3_add(ray.position, vec3_scalar_mul(rayDirection, 0.01f));
			color = vec3_add(color, vec3_scalar_mul(vec3_scalar_mul(vec3_make(1.0f, 0.0f, 0.0f), max(0.2f, diff)), attenuation));
			attenuation *= 0.3f;
		}
		else if (ray.type == HIT_BOX)
		{
			rayDirection = vec3_normalize(vec3_reflect(rayDirection, vec3_add(ray.normal, vec3_scalar_mul(rh, 0.1f))));
			rayOrigin = vec3_add(ray.position, vec3_scalar_mul(rayDirection, 0.01f));
			color = vec3_add(color, vec3_scalar_mul(vec3_scalar_mul(vec3_make(0.0f, 1.0f, 0.0f), max(0.2f, diff)), attenuation));
			attenuation *= 0.5f;
		}
		else if (ray.type == HIT_FLOOR)
		{
			rayDirection = vec3_normalize(vec3_reflect(rayDirection, vec3_add(ray.normal, vec3_scalar_mul(rh, 0.6f))));
			rayOrigin = vec3_add(ray.position, vec3_scalar_mul(rayDirection, 0.01f));
			color = vec3_add(color, vec3_scalar_mul(vec3_scalar_mul(vec3_make(0.15f, 0.15f, 0.15f), diff), attenuation));
			attenuation *= 0.6f;
		}
		else if (ray.type == HIT_NONE)
		{
			color = vec3_add(color, vec3_scalar_mul(vec3_make(1.0f, 2.0f, 3.0f), attenuation));
			break;
		}
	
		if (diff > 0.0f)
		{
			ray_result shadow = sdf_ray_march(vec3_add(ray.position, vec3_scalar_mul(ray.normal, 0.01f)), vec3_add(kLightDir, vec3_scalar_mul(rh, 0.1f)));
			if (shadow.type == HIT_NONE)
			{
				color = vec3_add(color, vec3_scalar_mul(vec3_scalar_mul(vec3_make(0.9f, 0.9f, 0.99f), attenuation), diff));
			}
		}
	}
	return vec3_scalar_div(color, (float)RAY_BOUNCE_COUNT);
}

void process_pixel(const vec3& __restrict pixelCoord, vec3& __restrict outColor)
{
	vec3 uv = vec3_mul(vec3_scalar_sub(vec3_div(pixelCoord, kResolution), 0.5f), kAspectRatio);
	vec3 rayOrigin = { 0.0f, 0.5f, -2.0f };
	vec3 rayDirection = vec3_normalize(uv);
	vec3 rh = random_hemisphere(get_random_value(), get_random_value());
	outColor = trace(rayOrigin, vec3_add(rayDirection, vec3_scalar_mul(rh, 0.001f)));
}

struct JobData
{
	float fLineY;
	int lineY;
	unsigned char* pOutput;
};

HANDLE gWorkThreads[IMAGE_HEIGHT];
JobData gJobData[IMAGE_HEIGHT];

DWORD WINAPI process_line(LPVOID pData)
{
	JobData* pJobData = (JobData*)pData;

#define CLAMP(v) ((v) < 0.0f ? 0.0f : (v) > 1.0f ? 1.0f : (v) * 255.0f)

	vec3 sampleColor = { 0.0f, 0.0f, 0.0f };
	float fy = pJobData->fLineY;
	int y = pJobData->lineY;
	unsigned char* pFramebuffer = pJobData->pOutput;
	for (int x = 0; x < IMAGE_WIDTH; ++x)
	{
		vec3 pixelCoord = { (float)x, fy, 1.0f };
		vec3 pixelColor = { 0.0f, 0.0f, 0.0f };
		for (int sample = 0; sample < SAMPLE_COUNT; ++sample)
		{
			process_pixel(pixelCoord, sampleColor);
			pixelColor = vec3_add(pixelColor, sampleColor);
		}
		pixelColor = vec3_scalar_div(pixelColor, (float)SAMPLE_COUNT);
		pixelColor.x = powf(pixelColor.x, 1.0f / 2.2f);
		pixelColor.y = powf(pixelColor.y, 1.0f / 2.2f);
		pixelColor.z = powf(pixelColor.z, 1.0f / 2.2f);
		int index = x * IMAGE_PIXEL_SIZE;
		unsigned char red = (unsigned char)(CLAMP(pixelColor.x));
		unsigned char green = (unsigned char)(CLAMP(pixelColor.y));
		unsigned char blue = (unsigned char)(CLAMP(pixelColor.z));
		pFramebuffer[index + 0] = blue; // Blue
		pFramebuffer[index + 1] = green; // Green
		pFramebuffer[index + 2] = red; // Red
	}
	
	printf("Processed Row %d\n", y);

	return 0;
}


void create_job(int lineY, unsigned char* pOutput)
{
	gJobData[lineY].lineY = lineY;
	gJobData[lineY].fLineY = (float)lineY;
	gJobData[lineY].pOutput = pOutput;
	gWorkThreads[lineY] = CreateThread(NULL, 4096, &process_line, &gJobData[lineY], 0, (LPDWORD)&gWorkThreads[lineY]);
}

void wait_for_jobs()
{
	for (int i = 0; i < IMAGE_HEIGHT; ++i)
	{
		WaitForSingleObject(gWorkThreads[i], INFINITE);
		CloseHandle(gWorkThreads[i]);
	}
}

int tga_write_rgb_buffer(const char* pFileName, int width, int height, unsigned char* pBuffer);

int main(int argc, const char** argv)
{
	LARGE_INTEGER start;
	LARGE_INTEGER end;
	LARGE_INTEGER freq;

	InitializeCriticalSectionAndSpinCount(&gCS, 0x00000400);
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);
	unsigned char* pFramebuffer = (unsigned char*)malloc(IMAGE_SIZE);
	srand((unsigned int)time(NULL));
	kLightDir = vec3_normalize(kLightDir);

	printf("\nStarting Ray Tracer\n");

#if THREADED
	for (int y = IMAGE_HEIGHT - 1; y >= 0; --y)
	{
		int index = (y * IMAGE_WIDTH) * IMAGE_PIXEL_SIZE;
		create_job(y, &pFramebuffer[index]);
	}
	wait_for_jobs();
#else
	vec3 sampleColor;

	for (int y = IMAGE_HEIGHT - 1; y >= 0; --y)
	{
		for (int x = 0; x < IMAGE_WIDTH; ++x)
		{
			vec3 pixelCoord = { (float)x, (float)y, 1.0f };
			vec3 pixelColor = { 0.0f, 0.0f, 0.0f };
			for (int sample = 0; sample < SAMPLE_COUNT; ++sample)
			{
				process_pixel(pixelCoord, sampleColor);
				pixelColor = vec3_add(pixelColor, sampleColor);
			}
			pixelColor = vec3_scalar_div(pixelColor, (float)SAMPLE_COUNT);
			pixelColor.x = powf(pixelColor.x, 1.0f / 1.4f);
			pixelColor.y = powf(pixelColor.y, 1.0f / 1.4f);
			pixelColor.z = powf(pixelColor.z, 1.0f / 1.4f);
			int index = (y * IMAGE_WIDTH + x) * IMAGE_PIXEL_SIZE;
			unsigned char red = (unsigned char)(CLAMP(pixelColor.x));
			unsigned char green = (unsigned char)(CLAMP(pixelColor.y));
			unsigned char blue = (unsigned char)(CLAMP(pixelColor.z));
			pFramebuffer[index + 0] = blue; // Blue
			pFramebuffer[index + 1] = green; // Green
			pFramebuffer[index + 2] = red; // Red
		}
	}
#endif

#undef CLAMP
	assert(tga_write_rgb_buffer("output.tga", IMAGE_WIDTH, IMAGE_HEIGHT, pFramebuffer));
	free(pFramebuffer);
	QueryPerformanceCounter(&end);
	double diff_sec = (end.QuadPart - start.QuadPart) / (double)freq.QuadPart;
	printf("\n### Process took: %.4lf seconds ###\n\n", diff_sec);
	MsgBox("Process Complete", "\n### Process took: %.4lf seconds ###\n\n", diff_sec);
	DeleteCriticalSection(&gCS);
	return 0;
}

int tga_write_rgb_buffer(const char* pFileName, int width, int height, unsigned char* pBuffer)
{
	FILE* pFile = NULL;
#if defined(_WIN32)
	fopen_s(&pFile, pFileName, "wb+");
#else
	pFile = fopen(pFileName, "wb+");
#endif
	if (pFile == NULL) return 0;
	putc(0, pFile); // ID Length
	putc(0, pFile); // Color Map Type
	putc(2, pFile); // Image Type
	putc(0, pFile); // Color Map Spec - First Entry Index - 0
	putc(0, pFile); // Color Map Spec - First Entry Index - 1
	putc(0, pFile); // Color Map Spec - Color Map Length - 0
	putc(0, pFile); // Color Map Spec - Color Map Length - 1
	putc(0, pFile); // Color Map Spec - Color Map Entry Size
	putc(0, pFile); // Color Map Spec - X Origin - 0
	putc(0, pFile); // Color Map Spec - X Origin - 1
	putc(0, pFile); // Color Map Spec - Y Origin - 0
	putc(0, pFile); // Color Map Spec - Y Origin - 1
	putc((width & 0xFF), pFile); // Image Spec - Width - 0
	putc((width >> 8) & 0xFF, pFile); // Image Spec - Width - 1
	putc((height & 0xFF), pFile); // Image Spec - Height - 0
	putc((height >> 8) & 0xFF, pFile); // Image Spec - Height - 1
	putc(24, pFile); // Image Spec - Pixel Depth
	putc(0, pFile); // Image Spec - Image Descriptor
	fwrite(pBuffer, width * height * 3, 1, pFile);
	fclose(pFile);
	return 1;
}
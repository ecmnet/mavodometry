
#include "detect_c_wrapper.h"
#include "detectNet.h"
#include "segNet.h"
#include "cudaMappedMemory.h"
#include "commandLine.h"
#include "stb_image.h"

static uint32_t width; 
static uint32_t height;

static float* imgCPU  = NULL;
static float* imgCUDA = NULL;

static float* outCPU  = NULL;
static float* outCUDA = NULL;

bool convertRGBA(char* img, float4** cpu, float4** gpu, int width, int height, int imgChannels );
bool convertRGB8(char* img, float4** cpu, float4** gpu, int width, int height, int imgChannels );

extern "C" {


segNet* createSegNet(int network,  uint32_t w, uint32_t h) {
   width  = w;
   height = h;

   const size_t imgSize = width * height * sizeof(float) * 4;
if( imgCUDA == NULL) 
{
   if( !cudaAllocMapped((void**)&imgCPU, (void**)&imgCUDA, imgSize) )
	{
		printf(LOG_CUDA "failed to allocate %zu bytes for image\n", imgSize);
		return NULL;
	}
}

if( outCUDA == NULL) 
{
  if( !cudaAllocMapped((void**)&outCPU, (void**)&outCUDA, imgSize) )
	{
		printf("segnet-console:  failed to allocate CUDA memory for output image\n");
		return 0;
	}
}
return segNet::Create((segNet::NetworkType)network);
}


detectNet* createDetectNet(int network, float threshold, uint32_t w, uint32_t h) {
	width  = w;
	height = h;

const size_t imgSize = width * height * sizeof(float) * 4;
if( imgCUDA == NULL) 
{
if( !cudaAllocMapped((void**)&imgCPU, (void**)&imgCUDA, imgSize) )
	{
		printf(LOG_CUDA "failed to allocate %zu bytes for image\n", imgSize);
		return NULL;
	}
}
return detectNet::Create((detectNet::NetworkType)network, threshold, 1);
  
}

detectNet* createDetectNetCustom(const char* prototxt_path, const char* model_path, const char* class_labels, const char* out_layer_name, float threshold, uint32_t w, uint32_t h) {
	width  = w;
	height = h;

const size_t imgSize = width * height * sizeof(float) * 4;
if( imgCUDA == NULL) 
{
if( !cudaAllocMapped((void**)&imgCPU, (void**)&imgCUDA, imgSize) )
	{
		printf(LOG_CUDA "failed to allocate %zu bytes for image\n", imgSize);
		return NULL;
	}
}
   return detectNet::Create(prototxt_path,model_path, 0.0f, class_labels, threshold, "data", out_layer_name, out_layer_name);
}

const char*  getClassDescription(detectNet* net, uint32_t ClassID) {
   return net->GetClassDesc(ClassID);
}



int detect(detectNet* net, char* img, Result* result, uint32_t overlay = 0 ) {

   convertRGBA(img, (float4**)&imgCPU, (float4**)&imgCUDA, width, height, 3 );

   detectNet::Detection* detections = NULL;
   const int numDetections = net->Detect(imgCUDA, width, height, &detections, overlay);
   
   
   
   for( int n=0; n < numDetections; n++ ) {
        result->Instance   = detections[n].Instance;
	    result->Confidence = detections[n].Confidence;
        result->ClassID    = detections[n].ClassID;
        result->Left       = detections[n].Left;
        result->Right      = detections[n].Right;
        result->Top        = detections[n].Top;
        result->Bottom     = detections[n].Bottom;
        result++;
   }

   if(overlay > 0)
     convertRGB8(img, (float4**)&imgCPU, (float4**)&imgCUDA, width, height, 3 );

   return numDetections;

}

int segmenting(segNet* net, char* img ) {

   const char* ignoreClass = "void";
   const char* visualization = "overlay";

   const segNet::FilterMode filterMode = segNet::FilterModeFromStr("linear");

   convertRGBA(img, (float4**)&imgCPU, (float4**)&imgCUDA, width, height, 3 );

   if( !net->Process(imgCUDA, width, height, ignoreClass) )
	{
		printf("segnet-console:  failed to process segmentation\n");
		return 0;
	}
   
   if( !net->Overlay(outCUDA, width, height, filterMode) )
		{
			printf("segnet-console:  failed to generate overlay.\n");
			return 0;
		}
   
   CUDA(cudaDeviceSynchronize());
   
   convertRGB8(img, (float4**)&outCPU, (float4**)&outCUDA, width, height, 3 );

   return 1;

}




/**
void       close(detectNet* det) {
	SAFE_DELETE(det);
}

**/

}



bool convertRGBA(char* img, float4** cpu, float4** gpu, int width, int height, int imgChannels )
{

const float4& mean=make_float4(0,0,0,0);

	// convert uint8 image to float4
	float4* cpuPtr = *cpu;
	
	for( int y=0; y < height; y++ )
	{
		const size_t yOffset = y * width * imgChannels * sizeof(unsigned char);

		for( int x=0; x < width; x++ )
		{
			#define GET_PIXEL(channel)	    float(img[offset + channel])
			#define SET_PIXEL_FLOAT4(r,g,b,a) cpuPtr[y*width+x] = make_float4(r,g,b,a)

			const size_t offset = yOffset + x * imgChannels * sizeof(unsigned char);
					
			switch(imgChannels)
			{
				case 1:	
				{
					const float grey = GET_PIXEL(0);
					SET_PIXEL_FLOAT4(grey - mean.x, grey - mean.y, grey - mean.z, 255.0f - mean.w); 
					break;
				}
				case 2:	
				{
					const float grey = GET_PIXEL(0);
					SET_PIXEL_FLOAT4(grey - mean.x, grey - mean.y, grey - mean.z, GET_PIXEL(1) - mean.w);
					break;
				}
				case 3:
				{
					SET_PIXEL_FLOAT4(GET_PIXEL(0) - mean.x, GET_PIXEL(1) - mean.y, GET_PIXEL(2) - mean.z, 255.0f - mean.w);
					break;
				}
				case 4:
				{
					SET_PIXEL_FLOAT4(GET_PIXEL(0) - mean.x, GET_PIXEL(1) - mean.y, GET_PIXEL(2) - mean.z, GET_PIXEL(3) - mean.w);
					break;
				}
			}
		}
	}
	
	return true;
}

bool convertRGB8(char* img, float4** cpu, float4** gpu, int width, int height, int imgChannels )
{


	float4* cpuPtr = *cpu;
	
	for( int y=0; y < height; y++ )
	{
		const size_t yOffset = y * width * imgChannels * sizeof(unsigned char);

		for( int x=0; x < width; x++ )
		{
			#define SET_PIXEL(channel, v)	    img[offset + channel] = (uint8_t)((float *) (&v))[channel]
			#define GET_PIXEL_FLOAT4()          (float4)cpuPtr[y*width+x] 

			const size_t offset = yOffset + x * imgChannels * sizeof(unsigned char);
					
			switch(imgChannels)
			{
				
				case 3:
				{
					const float4 pixel = GET_PIXEL_FLOAT4();
					SET_PIXEL(0, pixel); SET_PIXEL(1, pixel);  SET_PIXEL(2, pixel); 
					break;
				}
				
			}
		}
	}
	
	return true;
}








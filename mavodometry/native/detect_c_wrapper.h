#ifndef __DETECT_C_WRAPPER_H__
#define __DETECT_C_WRAPPER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif



typedef struct detectNet detectNet;	


typedef struct
{
		// Object Info
		uint32_t Instance;	
		uint32_t ClassID;	
		float Confidence;	

		// Bounding Box Coordinates
		float Left;		
		float Right;		
		float Top;		
		float Bottom;		
} Result;	


detectNet*   instance(int argc, char** argv, uint32_t w, uint32_t h);

const char*  getClassDescription(detectNet* det, uint32_t ClassID);
int          detect(detectNet* det, char* img, Result* result,uint32_t overlay );

#ifdef __cplusplus
}
#endif




#endif


#include "detect_c_wrapper.h"
#include "stb_image.h"

int main( int argc, char** argv )
{


  int imgWidth = 0;
  int imgHeight = 0;
  int imgChannels = 0;

  unsigned char* img = stbi_load("/home/ecm/IMG2.jpg", &imgWidth, &imgHeight, &imgChannels, 0);
  if(!img) {
     printf("..load failed\n");
    return 0;
  }


  printf("..load ok; %i channels\n", imgChannels);
  struct detectNet* net = createDetectNetCustom("networks/TrailNet_SResNet18/TrailNet_SResNet-18.prototxt",
                                         "networks/TrailNet_SResNet18/TrailNet_SResNet-18.caffemodel",
                                         "networks/TrailNet_SResNet18/label_map.txt",
                                         "out",
                                         0.5f,481,640);	


  printf("..initializing ok\n");
  Result result[100];

  int count = detect(net, img, result,0 );
  printf("..detect ok with %i classes\n", count);


  for(int i=0; i < count; i++)
    printf("%s: %f\n",getClassDescription(net,result[i].ClassID),result[i].Confidence);
     

 
	

	
}


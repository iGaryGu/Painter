#include<stdio.h>
#include<stdlib.h>


/*qcif = 176*144*/
int height = 176;
int width = 144;
float luma[3] = {0.299,0.587,0.114};
float fxaa_edge_threshold_min = 0.05;
float fxaa_edge_threshold = 0.1;
float bitmap[176*144][3];

void load_bitmap(){
	FILE *fpt = fopen("IMAGE.TXT","r");
	int r;
	int i = 0;
	while((r = fscanf(fpt,"%f %f %f %f %f %f\r\n",&bitmap[i][0],&bitmap[i][1],&bitmap[i][2],&bitmap[i+1][0],&bitmap[i+1][1],&bitmap[i+1][2]))!=-1){
		i+=2;
	}
	int j,k;

}

float process_luma(float* rgb){
	return (float)(rgb[0]*luma[0] + rgb[1]*luma[1] + rgb[2]*luma[2]);
}
float min(float x, float y){
	if(x <= y)
		return x;
	else
		return y;
}

float max(float x,float y){
	if(x >= y)
		return x;
	else
		return y;
}
float* sampleTexture(int x,int y){
	float* rgb = malloc(3*sizeof(float));

	// check bound
	if(x < 0) x = 0;
	if(x >= width) x = width-1;
	if(y < 0) y = 0;
	if(y >= height) y = height-1;
	int i;
	for(i = 0 ;i < 3;i++){
		int index = x*144+y;
		rgb[i] = (float)bitmap[index][i]/255; // normalize to [0,1]
	}
	return rgb;
}
float* processPoint(int x,int y){
	float* rgbN;
	float* rgbW;
	float* rgbM;
	float* rgbE;
	float* rgbS;
	float* newrgb;
	newrgb = malloc(3*sizeof(float));
	newrgb[0] = 1.0;
	newrgb[1] = 0.0;
	newrgb[2] = 0.0;

	rgbN = sampleTexture(x,y-1);
	rgbW = sampleTexture(x-1,y);
	rgbM = sampleTexture(x,y);
	rgbE = sampleTexture(x+1,y);
	rgbS = sampleTexture(x,y+1);

	// brightness of RGB
	float lumaN = process_luma(rgbN);
	float lumaW = process_luma(rgbW);
	float lumaM = process_luma(rgbM);
	float lumaE = process_luma(rgbE);
	float lumaS = process_luma(rgbS);

	float rangeMin = min(lumaM,min(min(lumaN,lumaW),min(lumaS,lumaE)));
	float rangeMax = max(lumaM,max(min(lumaN,lumaW),min(lumaS,lumaE)));
	
	float range = rangeMax - rangeMin;

	if(range < max(fxaa_edge_threshold_min,rangeMax*fxaa_edge_threshold)){
		return rgbM;
	}
	else{
		return newrgb;
	}

}
void doFxAA(){
	
	load_bitmap();
	int i,j,k;
	float* rgb;
	rgb = malloc(3*sizeof(float));
	float newBitMap[176*144][3];
	for(i = 0 ; i < 176 ;i++){
		for(j = 0 ; j < 144;j++){
			rgb = processPoint(i,j);
			for(k = 0 ; k < 3;k++)
				newBitMap[i*144+j][k] = rgb[k]*255;
		}
	}
	//show the processed bitmap result(edge detection)
	for(i = 0 ; i < 176*144;i++){
		for(k = 0 ; k< 3;k++){
			printf("%f ",newBitMap[i][k]);
		}
		printf("\n");
	}

}

int main(){
	doFxAA();
	return 0;
}



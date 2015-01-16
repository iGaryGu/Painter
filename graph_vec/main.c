#include<stdio.h>
#include<stdlib.h>
int f_x;
int f_y;
int height;
int width;
int sampleDis = 5;
int pre_x = -1;
int pre_y = -1;
int first_x = 1;
int first_y = 1;
int first_vecx;
int first_vecy;
int first = 0;
FILE* outfile;
int last_x;
int last_y;
int command_count = 0;

int vectorTest(int x_dif1,int y_dif1,int x_dif2,int y_dif2){
	if(pre_x == -1){
		return 0;
	}else{
		return x_dif1 * x_dif2 + y_dif1 * y_dif2;
	}
}

int disTest(int x,int y,int min_x,int min_y){
	printf("distest\n");
	int dif_x = x - min_x;
	int dif_y = y - min_y;
	int distance = (dif_x)*(dif_x)+(dif_y)*(dif_y);
	printf("distance = %d\n",distance);
	return distance > sampleDis*sampleDis;

}
//radus 範圍
int windowQuery(int x,int y,int radus,int data[height][width],int flag[height][width]){
	printf("window\n");
	int i,j;
	int x_up;
	int x_down;
	int y_left;
	int y_right;
	int max = 0;
	int next_x = -1;
	int next_y = -1;
	if(x-radus<0){
		x_up = 0;
	}else{
		x_up = x - radus;
	}
	if(x+radus>height){
		x_down = height;
	}else{
		x_down = x + radus;
	}
	if(y-radus<0){
		y_left = 0;
	}else{
		y_left = y - radus;
	}
	if(y+radus> width){
		y_right = width;
	}else{
		y_right = y+radus;
	}
	printf("x_up = %d , x_down = %d , y_left = %d, y_right = %d\n",x_up,x_down,y_left,y_right);
	for(i = x_up ; i < x_down ; i++){
		for(j = y_left;j< y_right;j++){
			if((data[i][j] == 1)&&(flag[i][j]!=1)){
				printf("x = %d , y = %d\n",x,y);
				printf("index_x = %d,index_y = %d\n",i,j);
				int start_dif = (first_x-i)*(first_x-i)+(first_y-j)*(first_y-j);
				if((start_dif < radus*radus) && vectorTest(first_x-i,first_y-j,first_vecx,first_vecy)>0){
					printf("out\n");
					printf("x = %d , y = %d\n",i,j);
					last_x = i;
					last_y = j;
					fprintf(outfile,"{%d,%d,%d},",i,j,1);
					command_count++;
					data[i][j] = 100; // sample point  著色
					return 1;
				}
				if(disTest(i,j,x,y) == 1){
					printf("pass\n");
					int dif1_x = x-pre_x;
					int dif1_y = y-pre_y;
					int dif2_x = i - x;
					int dif2_y = j - y;
					printf("pre_x = %d,pre_y = %d\n",pre_x,pre_y);
					printf("dif1_x = %d,dif1_y = %d,dif2_x = %d,dif2_y = %d\n",dif1_x,dif1_y,dif2_x,dif2_y);
					int cos = vectorTest(dif1_x,dif1_y,dif2_x,dif2_y);
					printf("cos = %d\n",cos);
					if(cos == 0 ){
						next_x = i;
						next_y = j;
					}else{
						if(cos > max){
							next_x = i;
							next_y = j;
							max = cos;
						}
					}
				}
			}
		}
	}
	if(first = 0){
		first_vecx = x-first_x;
		first_vecy = y-first_y;
		first++;
	}
	flag[next_x][next_y] = 1;
	pre_x = x;
	pre_y = y;
	printf("fd x = %d , y = %d\n",next_x,next_y);
	if(next_x == -1){
		return 1;
	}
	last_x = next_x;
	last_y = next_y;
	fprintf(outfile,"{%d,%d,%d},",next_x,next_y,1);
	command_count++;
	data[next_x][next_y] = 100; // sample point 著色
	windowQuery(next_x,next_y,radus,data,flag);

}

int nextStartTest(int x,int y,int matrix[height][width],int radus){
	int i,j;
	int x_up;
	int x_down;
	int y_left;
	int y_right;
	//邊界檢查
	if(x-radus<0){
		x_up = 0;
	}else{
		x_up = x - radus;
	}
	if(x+radus>width){
		x_down = width;
	}else{
		x_down = x + radus;
	}
	if(y-radus<0){
		y_left = 0;
	}else{
		y_left = y - radus;
	}
	if(y+radus> height){
		y_right = height;
	}else{
		y_right = y+radus;
	}
	for(i = x_up;i<x_down;i++){
		for(j = y_left;j<y_right;j++){
			//附近很可能已經畫過了
			if(matrix[i][j] == 100){
				return 0;
			}
		}
	}
	return 1;
}
//find the start point to draw
void scanFirstPoint(int matrix[height][width],int flag[height][width]){
	int i,j;
	int radus = 10;
	int stop = 0;
	outfile = fopen("rabbit_vector_command.txt","w");
	fprintf(outfile,"{");
	for(i = 0 ; i < height;i++){
		for(j = 0; j < width;j++){
			// find the first point to sample
			if(matrix[i][j]==1&&flag[i][j]==0){
				// 可以當起始點
				if(nextStartTest(i,j,matrix,radus)==1){
					//move pen and pen down to draw
					fprintf(outfile,"{%d,%d,%d},",i,j,0);
					command_count++;
					fprintf(outfile,"{%d,%d,%d},",i,j,1);
					command_count++;
					printf("touch!\n");
					printf("i = %d, j = %d\n",i,j);
					flag[i][j] = 1;
					first_x = i;
					first_y = j;
					windowQuery(i,j,radus,matrix,flag);
					//pen up!
					fprintf(outfile,"{%d,%d,%d},",last_x,last_y,0);
					command_count++;
					first = 0;
					pre_x = 0;
					pre_y = 0;
				}
			}
		}
	}
	fprintf(outfile,"}");
	fprintf(outfile,"%d",command_count);
	FILE* out = fopen("rabbit_vector.txt","w");
	for(i = 0 ;i < height;i++){
		for(j = 0 ; j < width;j++){
			if(matrix[i][j] == 1){
				matrix[i][j] = 0;
			}
			fprintf(out,"%d ",matrix[i][j]);
		}
		fprintf(out,"\n");
	}
	fclose(out);

}

int main(){
	FILE* fpr;
	int i;
	int j;
	fpr = fopen("rabbit_edge.txt","r");
	fscanf(fpr,"%d %d\n",&height,&width);
	int edge_point[height][width];
	int flag[height][width];
	//read the edge detection file
	for(i = 0 ; i < height;i++){
		for(j = 0 ; j< width;j++){
			fscanf(fpr,"%d ",&edge_point[i][j]);
			flag[i][j] = 0;
		}
		fscanf(fpr,"\n");
	}
	scanFirstPoint(edge_point,flag);

	return 0;
}

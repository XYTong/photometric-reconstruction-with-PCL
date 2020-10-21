//#include "stdafx.h"
#include"SAD.h"
#include<iostream>
int main()
{
	Mat Img_L=imread("cubel.jpg",0);
	Mat Img_R=imread("cuber.jpg",0);
    Mat Disparity;    //视差图
    
	//SAD mySAD;
	SAD mySAD(7,30);
	Disparity=mySAD.computerSAD(Img_L,Img_R);
    cout << Disparity.at<char>(1,1) << endl;
 
	//imshow("Img_L",Img_L);
	//imshow("Img_R",Img_R);
    imwrite("SAD2.jpg",Disparity);
	//imshow("Disparity",Disparity);
	//waitKey();
	return 0;
}


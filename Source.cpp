#include "cstdio"
#include  "iostream"
#include "cmath"
#include "opencv2/opencv.hpp" 
#include "opencv2/core/core.hpp"  
#include "opencv2/highgui/highgui.hpp"
#include "ctime"
#include "opencv2/video/background_segm.hpp"  
#include <cstdio>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/core/core.hpp>    
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/core/core.hpp>    
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#define RANGE		100
#define	NUM_LEDS	  8

using namespace cv;
using namespace std;

float y_ortalama = 0.0f;
float y_hiz = 0.0f;
std::clock_t start;
double duration;
void PutText(cv::Mat& img, const std::string& text, const cv::Rect& roi, const cv::Scalar& color, int fontFace, double fontScale, int thickness = 1, int lineType = 8)
{
    CV_Assert(!img.empty() && (img.type() == CV_8UC3 || img.type() == CV_8UC1));
    CV_Assert(roi.area() > 0);
    CV_Assert(!text.empty());

    int baseline = 0;

    // Calculates the width and height of a text string
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);

    // Y-coordinate of the baseline relative to the bottom-most text point
    baseline += thickness;

    // Render the text over here (fits to the text size)
    cv::Mat textImg(textSize.height + baseline, textSize.width, img.type());

    if (color == cv::Scalar::all(0)) textImg = cv::Scalar::all(255);
    else textImg = cv::Scalar::all(0);

    // Estimating the resolution of bounding image
    cv::Point textOrg((textImg.cols - textSize.width) / 2, (textImg.rows + textSize.height - baseline) / 2);

    // TR and BL points of the bounding box
    cv::Point tr(textOrg.x, textOrg.y + baseline);
    cv::Point bl(textOrg.x + textSize.width, textOrg.y - textSize.height);

    cv::putText(textImg, text, textOrg, fontFace, fontScale, color, thickness);

    // Resizing according to the ROI
    cv::resize(textImg, textImg, roi.size());

    cv::Mat textImgMask = textImg;
    if (textImgMask.type() == CV_8UC3)
        cv::cvtColor(textImgMask, textImgMask, cv::COLOR_BGR2GRAY);

    // Creating the mask
    cv::equalizeHist(textImgMask, textImgMask);

    if (color == cv::Scalar::all(0)) cv::threshold(textImgMask, textImgMask, 1, 255, cv::THRESH_BINARY_INV);
    else cv::threshold(textImgMask, textImgMask, 254, 255, cv::THRESH_BINARY);

    // Put into the original image
    cv::Mat destRoi = img(roi);
    textImg.copyTo(destRoi, textImgMask);
}

void projePutText(Mat& frame,string hizValue="yavas",string surusValue="iyi"){

	Rect r(10,20,160,20);
	Rect r2(230,20,160,20);
	int fontFace = cv:: FONT_HERSHEY_DUPLEX;
	double fontScale = 1.5;
	int thickness = 4;
	string hiz="Hiz: ";
	string serit="Serit: ";
	string surus="Surus: ";

	PutText(frame, hiz+hizValue,r, Scalar(0,0,0), fontFace, fontScale,thickness);
	//PutText(frame, serit+seritValue,r1, Scalar(0,0,0), fontFace, fontScale,thickness);
	PutText(frame, surus+surusValue,r2, Scalar(0,0,0), fontFace, fontScale,thickness);

}
void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step, const Scalar& color) {
	y_ortalama = 0.0f;
	int  y_num = 0;
	float y_sum = 0.0f;

	for (int y = 0; y < cflowmap.rows; y += step)
		for (int x = 0; x < cflowmap.cols; x += step)
		{
			const Point2f& fxy = flow.at< Point2f>(y, x);
			if (fxy.y > 0.8){
				line(cflowmap, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
					color);

				++y_num;
				y_sum += fxy.y;

				circle(cflowmap, Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), 1, color, -1);
			}
		}
	if (y_num != 0){
		y_ortalama = y_sum / y_num;
	}
}


void my_handler(int s){
           printf("Caught signal %d\n",s);
			softPwmWrite (0, 0) ;
			softPwmWrite (4, 0) ;
			softPwmWrite (5, 0) ;
			sleep(1);
         exit(1);

}
int sol_serit_kesikli=0;
int sag_serit_kesikli=0;

int main()
{
	Mat mask;
	Mat crop;
	Mat imgOriginal;
	vector< vector<Point> > contours;
	vector< Rect > output2;
	vector< vector< Point> >::iterator itc;
	Mat ContourImg;
string hiz_str,serit_str,surus_str;
 wiringPiSetup ()  ;

    softPwmCreate (0, 0, RANGE) ; /* motor */
   softPwmCreate (4, 0, RANGE) ; /*sol   led */
   softPwmCreate (5, 0, RANGE) ; /*sag  led */
   struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = my_handler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);


	float y_max = 0.0f;
	int s = 5;
	int k = 0;
	//global variables  
	Mat GetImg;
	Mat prvs, next; //current frame  

	Mat output, TEST, TEST1;
	Mat src, dst, color_dst;
	//global variables  
	Mat frame; //current frame  
	Mat resize_blur_Img;


	char fileName[100] = "Ankara.avi"; //video\\mm2.avi"; //mm2.avi"; //cctv 2.mov"; //mm2.avi"; //";//_p1.avi";  
	//VideoCapture stream1(fileName);   //0 is the id of video device.0 if you have only one camera     
	VideoCapture stream1(0);
	//morphology element  
	if (!(stream1.read(frame))) //get one frame form video  
		return 0;
	 
	resize(frame, prvs, Size(frame.size().width / s, frame.size().height / s));
	cvtColor(prvs, prvs, CV_BGR2GRAY);	



Mat element = getStructuringElement(MORPH_RECT, Size(7, 7), Point(3, 3));
	int hiz_flag=1;
	int count = 0;
int yon=0;
	//unconditional loop     
	while (true) {
imgOriginal=frame.clone();
	count++;
		Mat cameraFrame;
		Point solp1=Point(0, 0);
		Point solp2 = Point(0, frame.size().height);

		Point sol2p1 = Point(0, 0);
		Point sol2p2 = Point(0, 0);

		Point sagp1 = Point(frame.size().width / 2, frame.size().height);
		Point sagp2 = Point(0, 0);

		Point sag2p1 = Point(0, 0);
		Point sag2p2 = Point(0, 0);
		float angle=0.0f;
		float sol_angle=0.0f;
		float sag_angle=0.0f;
		float sol2_angle = 0.0f;
		float sag2_angle = 0.0f;

		if (!(stream1.read(frame))) //get one frame form video     
			break;
	//HIZ BASLANGICCCCCCCCCCCCCCC	
		//Resize  
		resize(frame, next, Size(frame.size().width / s, frame.size().height / s));
		cvtColor(next, next, CV_BGR2GRAY);
		///////////////////////////////////////////////////////////////////  
		Mat flow;
		calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 30, 3, 5, 1.2, 1);

		Mat cflow;
		cvtColor(prvs, cflow, CV_GRAY2BGR);
		drawOptFlowMap(flow, cflow, 10, CV_RGB(0, 255, 0));
		//imshow("OpticalFlowFarneback", cflow);
		prvs = next.clone();

		if (y_max < y_ortalama){
			y_max = y_ortalama;

		}
		int deger = (y_ortalama / y_max) * 90;

		if (deger != 0){
			//cout <<  " HIZ " << deger << endl;
			if (count >= 5){
				count = 0;
				//cout << "HIZ DURUMU ";
				if (deger < 20){
				start =std::clock();
				   hiz_flag=0;
				}else if (deger < 30){
				start =std::clock();
				   hiz_flag=1;
					cout << " YAVAS " << endl;
					hiz_str=" YAVAS ";
				}
				else if ( deger < 60){
				start =std::clock();
					hiz_flag=1;
					cout << " ORTA " << endl;
					hiz_str=" ORTA ";
				}else{  
					start =std::clock();
					hiz_flag=1;
					cout << " HIZLI " << endl;
					hiz_str=" HIZLI ";
					}

			}
			if (k < 80){
				y_hiz += deger;
				++k;
			}
			else{


				cout << "\n ORTALAMA HIZ " << (y_hiz / k) << endl;
				y_hiz = 0;
				k = 0;
			}
		}else if(!hiz_flag){
			duration = (std::clock()-start)/(double)CLOCKS_PER_SEC;
				cout<<" duration "<<duration<<endl;
				if(duration>6){
				cout << " DURUYOR " << endl;
				hiz_str="DURUYOR";
				}
				

		}

//HIZ BITISSSSSSSSSSSSSSSSS
		resize(frame, resize_blur_Img, Size(frame.size().width, frame.size().height ));

		cv::Mat frame1 = cv::Mat(frame, cv::Rect(0, ((frame.size().height / 3)*2), frame.size().width, (frame.size().height / 5))).clone();
		

		GaussianBlur(frame1, frame1, Size(5, 5), 100, 0);
		Canny(frame1, dst,50, 80);
		namedWindow("frame11", 1);
		imshow("frame11", frame1);
		int y_yuksek = (frame.size().height / 3)*2;
		vector<Vec4i> lines;
		HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 30, 10);
		for (size_t i = 0; i < lines.size(); i++)
		{
			int y = (frame.size().height / 3)*2;
			Point p1 = Point(lines[i][0], y + lines[i][1]);
			Point p2 = Point(lines[i][2], y + lines[i][3]);
			//abs(angle) >= 30 angle < -10 && angle>-170
			int dx = p2.x - p1.x;
			int dy = p2.y - p1.y;
			angle = atan2f(dy, dx) * 180 / CV_PI;
			if (angle < -10 && angle>-182){		//sol taraf p1 asagisi p2 yukari sag �st capraza  
				
				
				if ((frame.size().width) / 2 > p1.x && (frame.size().width) / 2 > p2.x){
					if (p1.x <= p2.x && p1.y > p2.y){

						if (solp2.x <= p2.x && solp2.y > p2.y){
							solp1 = p1;
							solp2 = p2;
							sol_angle = angle;
						}
						if (sol2p1.y < p1.y && sol2p1.x <= p1.x){
							sol2p1 = p1;
							sol2p2 = p2;
							sol2_angle = angle;
						}

						//line(frame, Point(lines[i][2], y + lines[i][3]), Point(lines[i][0], y + lines[i][1]), Scalar(0, 0, 255), 3, 10);
					}
				}
				
			}
			if (angle > 10 && angle<182){		//sag taraf p1 yukardan p2 asagi sag alt �apraz
				
				if ((frame.size().width) / 2 < p1.x && (frame.size().width) / 2 < p2.x){
					if (p1.x <= p2.x && p1.y < p2.y){

						if (sagp1.x <= p1.x && sagp1.y > p1.y){
							sagp1 = p1;
							sagp2 = p2;
							sag_angle = angle;
						}
						if (sag2p2.y < p2.y && sag2p2.x <= p2.x){
							sag2p1 = p1;
							sag2p2 = p2;
							sag2_angle = angle;
						}

						//line(frame, Point(lines[i][0], y + lines[i][1]),Point(lines[i][2], y + lines[i][3]), Scalar(0, 255, 0), 3, 10);
					}
				}
			}

			// orta cizgi siyah
			line(frame, Point(frame.size().width / 2, ((frame.size().height / 3)*2)),
				Point(frame.size().width / 2, frame.size().height), Scalar(0, 0, 0), 3, 10);

			if (abs(dx) <30 ){

				if(angle<-50){
					cout<<"	SOLLLL DIKKKKKK "<<endl;
					
					if(yon==0){
						yon=-1;
					}
					
				}
				if(angle>50){
					cout<<"	SAGG DIKKKKKK "<<endl;
						if(yon==0){
						yon=1;
					}
				}
				
				 cout << "\n dik "<<angle<<"\n ";
			}
			
			
		}

/*			//sol taraf p1 asagisi p2 yukari sag �st capraza  
			//line(frame, solp2, solp1, Scalar(0, 0, 0), 3, 10);

			//sag taraf p1 yukardan p2 asagi sag alt �apraz
			//line(frame, sagp1, sagp2, Scalar(0, 0, 0), 3, 10);
		
			//sag taraf p1 yukardan p2 asagi sag alt �apraz
			//line(frame, sag2p1, sag2p2, Scalar(0, 0, 0), 3, 10);
*/


		if (sol_angle < -65){
			cout << "\n SOLLL EGIM : " << sol_angle;
		}
		if (sag_angle > 65){
			cout << "\n SAGGG EGIM : " << sag_angle;
		}

		int sol_alt = 0;
		int sag_alt = 0;
		float egim_sol = 0.f;

		float egim_sag = 0.f;
		float x_ler_farki_ust = 0.f;
		float x_ler_farki_alt = 0.f;
		Point sol_ust(0, 0);
		Point sag_ust(0, 0);



		if (solp1 != Point(0, 0) && sol2p1 != Point(0, 0) && abs(sol2_angle - sol_angle)==0){
			float x = ((float)sol2p1.x - (float)solp2.x) / ((float)sol2p1.y - (float)solp2.y)*((float)y_yuksek - (float)sol2p1.y) + (float)sol2p1.x;
			float y = y_yuksek;
			egim_sol = ((float)sol2p1.y - (float)solp2.y) / ((float)sol2p1.x - (float)solp2.x);
			float ust_acisi = atanf(egim_sol) * 180 / CV_PI;
			x_ler_farki_ust = x*(-1.0f);
			sol_ust.x = x;
			sol_ust.y = y;
			Vec3b color = frame.at<Vec3b>(Point(x, y));
			

			if (ust_acisi == sol2_angle){
				
				circle(frame, sol_ust, 2, Scalar(0, 0, 255));
				
				double res_sol = cv::norm(cv::Mat(sol_ust), cv::Mat(solp2));
				
				if (res_sol > 53){
					//cout << "\n  SOL mesafesi ::: " << res_sol << endl;
					cout << "\n ------------------ SOL KESIKLI SERIT ------------------\n";
					sol_serit_kesikli+=1;
				}else{
					//sol_serit_kesikli=0;
				}
				

			}
				line(frame, sol2p1, solp2, Scalar(220, 0, 0), 5, 10);
			
			
			
			sol_alt = sol2p2.x;
		}
		if (sagp2 != Point(0, 0) && sag2p2 != Point(0, 0)&&abs(sag2_angle - sag_angle)==0){
			
			float x = ((float)sagp1.x - (float)sag2p2.x) / ((float)sagp1.y - (float)sag2p2.y)*((float)y_yuksek - (float)sagp1.y) + (float)sagp1.x;
			float y = y_yuksek;
			egim_sag = ((float)sagp1.y - (float)sag2p2.y) / ((float)sagp1.x - (float)sag2p2.x);
			float ust_acisi = atanf(egim_sag) * 180 / CV_PI;
			x_ler_farki_ust += x;
			sag_ust.x = x;
			sag_ust.y = y;
			Vec3b color = frame.at<Vec3b>(Point(x, y));
			
			if (ust_acisi == sag2_angle){
				
				circle(frame, sag_ust, 2, Scalar(0, 0, 255));
				double res_sag = cv::norm(cv::Mat(sag_ust), cv::Mat(sagp1));
				
				if (res_sag > 53){
					//cout << "\n  SAG mesafesi ::: " << res_sag << endl;
					cout << "\n ------------------ SAG KESIKLI SERIT ------------------\n";
					sag_serit_kesikli=+1;
				}else{
					//sag_serit_kesikli=0;
				}
			}
				line(frame, sagp1, sag2p2, Scalar(110, 0, 0), 5, 10);
			
			sag_alt = sag2p2.x;
		}
		if (sol_alt != 0 && sag_alt != 0){

			float dogru_arasindaki_aci=(egim_sol - egim_sag) / (1 + (egim_sag*egim_sol));
			float acisi=atanf(dogru_arasindaki_aci) * 180 / CV_PI;
			if(( (frame.size().width / 2)-sol_ust.x)<100){
				cout<<"//////////////////////SOL "<<endl;
					if(((frame.size().width / 2)-sol_ust.x)<90){
					if(sol_serit_kesikli>0){
					 softPwmWrite (0, 50) ;
						delay(500);
					}else{
					softPwmWrite (0, 100) ;
					delay(500);
					}
					}
					softPwmWrite (5, 0) ;
					softPwmWrite (4, 100) ;
					delay(5);

			}
			 if( (sag_ust.x - (frame.size().width / 2))<100){
				cout<<"///////////////////SAG "<<endl;
					 if( (sag_ust.x - (frame.size().width / 2))<90){
					if(sag_serit_kesikli>0){
					 softPwmWrite (0, 50) ;
					delay(500);
					}else{
					softPwmWrite (0, 100) ;
					delay(500);
					}
					}
					softPwmWrite (4, 0) ;
					softPwmWrite (5, 100) ;
					delay(5);
			}else{
			softPwmWrite (0, 0) ;
			
			sag_serit_kesikli=0;
			sol_serit_kesikli=0;
			softPwmWrite (5, 0) ;
			softPwmWrite (4, 0) ;

			}

			//cout << "\n IKI DOGRU ARASI ACI : " << acisi<<endl;
			//cout << "\n IKI arasi mesafe : " << x_ler_farki_ust << endl;
			//cout << " \n ASAGI X ARALIK : " << sag_alt - sol_alt<<endl;
			cout << "\n SOLLL EGIM : " << sol2_angle;
			cout << "\n SAGGG EGIM : " << sag2_angle;
			float sum_degree = sol2_angle + sag2_angle;
			cout << "\n ****** *****          Sum EGIM : " << abs(sum_degree);
			if (abs(sum_degree) < 5){

				cout << "	\n\t  : IYI SURUS "<<endl;
				surus_str="IYI SURUS";
			}else if (abs(sum_degree) < 10){

				cout << "	\n\t  : ORTA SURUS " << endl;
				surus_str="ORTA SURUS";
			}else if (abs(sum_degree) < 15){

				cout << "	\n\t  : KOTU SURUS " << endl;
				surus_str="KOTU SURUS";
			}
			
//cout << "	\n\t  :" << ((sum_degree > 0) ? "SAGA SINYAL" : "SOLA SINYAL") << endl;

				
/*
				if(sum_degree>15 || sol2_angle<-55 || sag2_angle<30){
					cout<<"\n SAGA SINYAL \n";
					softPwmWrite (4, 0) ;
					softPwmWrite (5, 100) ;
					delay(5);
				}else if(sum_degree<-15 || sag2_angle>48 || sol2_angle>-30){
					cout<<"\n SOL SINYAL \n";
					softPwmWrite (5, 0) ;
					softPwmWrite (4, 100) ;
					delay(5);
				}else{ 
				yon=0;
				softPwmWrite (5, 0) ;
				softPwmWrite (4, 0) ;
				}
				yon=0;
*/
			}	


		/* eksik serit kismi
		if (sol_alt != 0 && sag_alt == 0){
			sag_ust.x = sol_ust.x + 286;
			sag_ust.y = sol_ust.y;
			circle(frame, sag_ust, 2, Scalar(0, 0, 255));
		
		}
		if (sol_alt == 0 && sag_alt != 0){
			sol_ust.x = sag_ust.x - 286;
			sol_ust.y = sag_ust.y;
			circle(frame, sol_ust, 2, Scalar(0, 0, 255));

		}
		*/

		
	//	namedWindow("Detected Lines2", 1);
	//	imshow("Detected Lines2", dst);
Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		
		Mat imgThresholded;
		
		inRange(imgHSV, Scalar(50, 121, 70), Scalar(132, 252, 255), imgThresholded);//mavi-yesil

		//inRange(imgHSV, Scalar(69, 65, 70), Scalar(101, 252, 255), imgThresholded);//yesil
		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


		ContourImg = imgThresholded.clone();
		//less blob delete  
		findContours(ContourImg,
			contours, // a vector of contours  
			CV_RETR_EXTERNAL, // retrieve the external contours  
			CV_CHAIN_APPROX_NONE); // all pixels of each contours  

		 itc = contours.begin();
		while (itc != contours.end()) {

			//Create bounding rect of object  
			//rect draw on origin image  

			double area0 = contourArea(Mat(*itc), false);
			Rect mr = boundingRect(Mat(*itc));
			int x = mr.x;
			int y = mr.y;

			
			rectangle(frame, mr, cv::Scalar(255, 255, 0));
			

			++itc;
		}
		/*
		//inRange(imgHSV, Scalar(177, 0, 0), Scalar(179, 255, 255), imgThresholded);//kirmizi1
		inRange(imgHSV, Scalar(177, 0, 0), Scalar(179, 255, 255), imgThresholded);//kirmizi1


		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


		ContourImg = imgThresholded.clone();
		//less blob delete  
		findContours(ContourImg,
			contours, // a vector of contours  
			CV_RETR_EXTERNAL, // retrieve the external contours  
			CV_CHAIN_APPROX_NONE); // all pixels of each contours  

		 itc = contours.begin();
		while (itc != contours.end()) {

			//Create bounding rect of object  
			//rect draw on origin image  

			double area0 = contourArea(Mat(*itc), false);
			Rect mr = boundingRect(Mat(*itc));
			int x = mr.x;
			int y = mr.y;
			rectangle(imgOriginal, mr, cv::Scalar(255, 255, 0));


			++itc;
		}
	*/


projePutText(frame,hiz_str,surus_str);
		imshow("OOinput", frame);

	//	imshow("Blur_Resize", resize_blur_Img);
		
		if (waitKey(5) >= 0){
			for (int i = 0; i < 800; i++)
				frame = stream1.grab();
		}
	}


}


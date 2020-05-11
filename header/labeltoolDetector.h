/* BSD 3-Clause License
 *  
 *  Copyright (c) 2020, tmkhoyan (Tigran Mkhoyan)
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *  
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Description: 	     Labeltool Detector simple data class: data type definitions
 *                       dependencies: LabelData, Blob_conf, 
 * 					example compile: make
 *					then run with:   ./runMain
 *
 * Author: 			Tigran Mkhoyan
 * Email : 			t.mkhoyan@tudelft.nl
 */


// #pragma once 
namespace  invdbscan{

 #ifndef Pixel
  typedef cv::Point3_<uint8_t> Pixel;     
 #endif

 // data: [ 255, 20, 50 ]

// green filter
struct removeGreen {
  Pixel pixelbgr_treshold = Pixel(0,225,0); // green no other colors
  Pixel pixelbgr_target  = Pixel(0,0,0);
  	removeGreen() {} 
  	removeGreen(Pixel pixelbgr_treshold_): pixelbgr_treshold(pixelbgr_treshold_) {} 
  	removeGreen(Pixel pixelbgr_treshold_, Pixel pixelbgr_target_): pixelbgr_treshold(pixelbgr_treshold_), pixelbgr_target(pixelbgr_target_) {} 
	~removeGreen() {} 
  void operator ()(Pixel &pixel, const int * position) const
  {
  	// if  B<   G>    R<
  	if((pixel.x<pixelbgr_treshold.x) and (pixel.y>pixelbgr_treshold.y) and (pixel.z<pixelbgr_treshold.z)){
	  	pixel.x = pixelbgr_target.x;
	  	pixel.y = pixelbgr_target.y;
	  	pixel.z = pixelbgr_target.z;
  	} //else nothing
  }
};

	typedef cv::Point2f pWorkType;

	template<typename T_>
		class LabelDetector
		{

			public: 
				typedef typename T_::value_type E;
				typedef cv::Point_ <E> T2d;          // needed to define 2d point
				E wzoom   = WRECT;
				E hzoom   = HRECT;
				E wzoomed ; // size of selected region to zoom in same as rect size 
				E hzoomed ;
				E factor = 10.0;
				int thr_min = 150;
				int thr_max = 255;
				bool isColorFilterImage = false;
				bool isMorphFilter = false;
				Pixel colFiltpx = Pixel( 255, 20, 50);
				int pixErode	=MORPH_PIX_ERO;
				int pixDilate	=MORPH_PIX_DIL;
				/* ----------------------------------core contents loop ---------------- */
				T_ p;     // in         
				T_ p_c;              
				T_ p_c_zoom;     
				T_ p_closest;    
				T_ p_closest_zoom;  
				int closestindex=0;
				std::vector<T_> P_c ;
				std::vector<cv::KeyPoint>keypoints;
				size_t psize = 100; 
				cv::Mat zoomed_thr;
				cv::Mat zoomed;
				cv::Mat selected;
				T2d wzoom_center;
				/* ----------------------------------desired detector here ---------------- */
				cv::Ptr<cv::SimpleBlobDetector> detector;

				/* ----------------------------------core contents info ---------------- */
				int cols, rows;
				E x, y;
				/* ----------------------------------constructor and destructor  ---------------- */
				LabelDetector(){};
				LabelDetector(E wzoom_, E hzoom_, E factor_, size_t psize_=100, bool isColorFilterImage_=false,bool isMorphFilter_=false):
					wzoom(wzoom_),
					hzoom(hzoom_),
					wzoomed(wzoom_*factor_),
					hzoomed(hzoom_*factor_),
					factor(factor_),
					psize(psize_),
					selected(cv::Mat(wzoom_,hzoom_, CV_8UC3)),
					zoomed(cv::Mat(wzoom_*factor_,hzoom_*factor_, CV_8UC3)),
					zoomed_thr(cv::Mat(wzoom_*factor_,hzoom_*factor_, CV_8UC3)),
					wzoom_center((wzoom_*factor_)/2,(wzoom_*factor_)/2),
					isColorFilterImage(isColorFilterImage_),
					isMorphFilter(isMorphFilter_)
			{
				P_c.reserve(psize);
				keypoints.reserve(psize);
				init();
			}
				// LabelDetector operator()(E wzoom_, E hzoom_, E factor_, size_t psize_=10):{
				//     LabelDetector d;
				//     return d;
				// };
				~LabelDetector(){std::cout << "LabelDetector object deleted" << std::endl;};

				/* ---------------------------------- load detector ---------------------------- */
				void init(){
					detector = cv::SimpleBlobDetector::create(setupBlobDetector());
				}
				void setColorFilter(Pixel pix_in){
					isColorFilterImage = true;
					colFiltpx = Pixel(pix_in);
				}
				void setMorphFilter(int pixErode_, int pixDilate_){
					pixErode = pixErode_;
					pixDilate = pixDilate_;
					isMorphFilter = true;
				}
				/* ---------------------------------- main ---------------------------- */
				void detect(cv::Mat img,E x_, E y_){
					
					x = x_; cols = img.cols;
					y = y_; rows = img.rows;
					
					// extract zoom region
					std::vector<cv::KeyPoint> keypoints_; keypoints_.reserve(psize);
					// T2d wzoom_center(wzoomed/2,hzoomed/2)
					// cv::Mat selected = img(centeredRect(x,y,wzoom,hzoom)).clone(); 
					selected = img(centeredRect(x,y,wzoom,hzoom)).clone(); 
					// apply green filter 
					if(isColorFilterImage)
						selected.forEach<Pixel>(removeGreen(colFiltpx)); // look for high green and blue this is the cleanest but 

					zoomed  = cv::Mat(wzoom, hzoom,CV_8UC3); 
					// cv::Mat zoomed  = cv::Mat(wzoom, hzoom,CV_8UC3); 
					cv::resize(selected, zoomed, cv::Size(wzoom*factor,hzoom*factor), 0, 0);//,  cv::INTER_NEAREST); //wzoom*factor,wzoom*factor,
					// apply thresholding
					cv::Mat zoomed_g; cv::cvtColor(zoomed, zoomed_g, cv::COLOR_BGR2GRAY);
					cv::threshold (zoomed_g, zoomed_thr, thr_min, thr_max, cv::THRESH_OTSU);
					// morphological
					if(isMorphFilter)
						filterImage(zoomed_thr);

					// cv::Mat zoomed_thr; cv::threshold (zoomed_g, zoomed_thr, thr_min, thr_max, cv::THRESH_OTSU);
					cv::threshold(zoomed_thr, zoomed_thr, thr_min, thr_max, cv::THRESH_BINARY_INV);

											
					// run detector
					detector->detect(zoomed_thr, keypoints_);
					// sort
					p                   = T2d(x,y);
					P_c                 = toGlobalCoords(keypoints_,x,y);
					p_c                 = getCentroid(P_c, x, y, P_c); //  keypoints_global is output 
					p_c_zoom            = fromImgCoords(p_c,x,y,factor,wzoom,hzoom); // centroid in zoom coordinates

					closestindex        = closestPoint2Centroid(P_c,p_c); // find point closest to centroid 
					p_closest           = ((closestindex>=0)? P_c[closestindex]   : T2d(x,y));
					p_closest_zoom      = ((closestindex>=0)? keypoints_[closestindex].pt       : wzoom_center);

					keypoints           = keypoints_;
				}
				cv::Mat getThresholdImg(){
					return zoomed_thr;
				}
				cv::Mat getZoomImg(){
					return zoomed;
				}
				cv::Mat getZoomImgAny(E factor_){

					E wzoomed = wzoom*factor_;
					E hzoomed = wzoom*factor_;

					cv::Mat zoomed_any;//  = cv::Mat(wzoomed, wzoomed,CV_8UC3); 
					cv::resize(selected, zoomed_any, cv::Size(wzoomed,hzoomed), 0, 0);//,  cv::INTER_NEAREST); //wzoom*factor,wzoom*factor,
					
					return zoomed_any;
				}
				cv::Mat putZoomImgAny(cv::Mat img, E x_, E y_, E factor_){
					// custom bounds check
					E wzoomed = wzoom*factor_;
					E hzoomed = wzoom*factor_;

					E xmax_zoom = img.cols - wzoomed/2; E xmin_zoom = wzoomed/2; // W 
					E ymax_zoom = img.rows - hzoomed/2; E ymin_zoom = hzoomed/2; // H

					auto x = x_;	x = std::min<E>(x,xmax_zoom); x = std::max<E>(x,xmin_zoom); 
					auto y = y_;	y = std::min<E>(y,ymax_zoom); y = std::max<E>(y,ymin_zoom); 

					auto img__ = img.clone();
					auto zoomed_any = getZoomImgAny(factor_); // changes x,y
					
			// std::cout << "--->pxy" << zoomed_any.size() << std::endl;
			// std::cout << "-->pxy" << (centeredRect(x,y,wzoomed,wzoomed)).size() << std::endl;

			// std::cout << "--->pxy" << wzoomed << std::endl;
			// std::cout << "--->pxy" << hzoomed << std::endl;
			// std::cout << "--->pxy" << factor_ << std::endl;

					// draw rectangle white border
					cv::rectangle(zoomed_any, centeredRect(wzoomed/2,hzoomed/2,wzoomed,hzoomed), COL_WHITE, 3);

					zoomed_any.copyTo(img__(centeredRect(x,y,wzoomed,wzoomed)));
					return img__;
				}
				inline int getDetectedPoints( T2d & p_c_ , T2d &p_c_zoom_ , T2d & p_closest_ , T2d & p_closest_zoom_, 
                                  std::vector<T2d> & P_c_, std::vector<cv::KeyPoint>& keypoints_ ){                          

					p_c_                 = p_c;            //  P_cglobal is output 
					p_c_zoom_            = p_c_zoom;       // centroid in zoom coordinates
					p_closest_           = p_closest;      //
					p_closest_zoom_      = p_closest_zoom; //

					keypoints_           = keypoints;
					P_c_           = P_c;                  //  P_cglobal is output 
					return closestindex;
				}
				inline int clearDetectPoints(T2d & p_c_ , T2d &p_c_zoom_ , T2d & p_closest_ , T2d & p_closest_zoom_ ,
                                 std::vector<T2d> & P_c_, std::vector<cv::KeyPoint>& keypoints_){     
					p_c_             = p;                  // centroid in zoom 
					p_closest_       = p;                  // closest point
					p_c_zoom_        = wzoom_center;       // centroid in zoom
					p_closest_zoom_  = wzoom_center;       // closest point
					P_c_.clear();
					keypoints_.clear();
					return -1;
				}
				/* ------------------------------------------- filter  morphological  -------------------------- */
				void filterImage(cv::Mat &img_th){
							//apply erode and dilate to cleaup markers
							cv::Mat erodeElement  = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(pixErode,pixErode));
							cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(pixDilate,pixDilate));

							cv::erode(img_th,img_th,erodeElement);
							cv::dilate(img_th,img_th,dilateElement);
							// std::cout << "filterred " << pixErode <<"   " << pixDilate << std::endl;
						}
				/* ------------------------------------------- helper  function  -------------------------- */
				inline std::vector<T2d> toGlobalCoords(std::vector<cv::KeyPoint> keypoints_in, E x, E y){
					std::vector<T_> P_c_ ; P_c_.reserve(keypoints_in.size());
					for(auto kp : keypoints_in)
						P_c_.emplace_back(toImgCoords(kp,x,y,factor,wzoom,hzoom));
					return P_c_; 
				}
				inline T2d getCentroid(std::vector<T_> P_in, E x, E y, std::vector<T2d> &){
					// calculate centroid
					E xcentr=0, ycentr=0;
					for(const auto &p : P_in){
						xcentr+=p.x;
						ycentr+=p.y;
					}
					size_t size_p = P_in.size(); 
					xcentr = xcentr/size_p;
					ycentr = ycentr/size_p;
					auto pxy_cent = P_in.empty()? T2d(x,y): T2d(xcentr,ycentr); // set to current  constrained mouse point
					return pxy_cent; 
				}
				inline int closestPoint2Centroid(std::vector<T2d> points, T2d centroid){ // returns value type T2d
					std::vector<E> vecnorm; vecnorm.reserve(points.size());
					for(const auto &p : points){
						auto dxy = p - centroid;
						vecnorm.emplace_back(cv::sqrt(dxy.x*dxy.x + dxy.y*dxy.y));
					}
					// E minelem = vecnorm.empty()?  -1: *std::min_element(vecnorm.begin(),vecnorm.end()); // returs the  elemnt 
					// int minelemindex = vecnorm.empty()?  -1: (std::min_element(vecnorm.begin(),vecnorm.end())  -vecnorm.begin()); // return index
					return (vecnorm.empty()?  -1: (std::min_element(vecnorm.begin(),vecnorm.end())  -vecnorm.begin()));
				}
				inline T2d toImgCoords(cv::KeyPoint kp, E x, E y, E factor ,E wzoom_=WRECT, E hzoom_=HRECT ){                
					T2d p_global = T2d(x-wzoom_/2,y-hzoom_/2);              // scale 
					T2d p = kp.pt/factor + p_global; // scale back from zoom and translate to global coords from top left coordinate of roi
					return p;
				}
				inline T2d fromImgCoords(T2d p_global, E x, E y, E factor ,E wzoom_=WRECT, E hzoom_=HRECT ){                 
					T2d p_zoom = T2d(x-wzoom_/2,y-hzoom_/2);                // scale 
					T2d p = (p_global-p_zoom)*factor; // scale back from zoom and translate to global coords from top left coordinate of roi
					return p;
				}
				inline cv::Rect centeredRect(E x, E y, E wrect=WRECT, E hrect=HRECT){
					cv::Rect_<E> r( x - wrect/2, y - hrect/2,wrect,hrect);
					return r;
				}
				inline bool isInRect(E x_in, E y_in){
					return( std::abs(x-x_in)<wzoom and std::abs(y-y_in)<wzoom );
				}
				/* -------------------------------------------end of class  -------------------------------------------*/
		};
	/* -------------------------------------------read stdout -------------------------------------------*/
	template<typename T_>
		static std::ostream& operator<<(std::ostream& out, const LabelDetector<T_>& s)
		{
			out << "\t" << "centroid p      :"      << s.p_c           << "\n";
			out << "\t" << "closest  p      :"      << s.p_c           << "\n";
			out << "\t" << "centroid p_zoom :"      << s.p_c_zoom      << "\n";
			out << "\t" << "closest  p_zoom :"      << s.p_c_zoom      << "\n";
			out << "\t" << "points   P_c = [:"      << s.p_c_zoom      << "\n";
			for(auto p: s.P_c){out << p << "," ;}; out     << "];" << "\n";
			return out;
		}
	/* -------------------------------------------end of namespace  -------------------------------------------*/

}



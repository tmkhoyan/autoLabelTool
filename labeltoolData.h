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
 * Description: 	     Labeltool Data class: data type definitions 
 *                       dependencies: labeltool.h
 * 					example compile: make
 *					then run with:   ./runMain
 *
 * Author: 			Tigran Mkhoyan
 * Email : 			t.mkhoyan@tudelft.nl
 */


namespace  invdbscan{
	/* -------------------------------------------class to manage tool data  -------------------------------------------*/

	template<typename T_>
		class LabelData {
			public: 
				// typedefs
				typedef typename T_::value_type E;
				typedef cv::Point_ <E> T2d;  		// needed to define 2d point
				cv::Point_<E> nanp = cv::Point_<E>(0.0/0.0,0.0/0.0);
				/* --------------------------------  main input parameters  --------------------------------------*/			
				size_t nmarkers =100;
				int nframes = -1;
				E wrect = WRECT;
				E hrect = HRECT;
				E wzoom = WRECT; // size of selected region to zoom in same as rect size 
				E hzoom = HRECT;
				E wzoomu; // size of selected region to zoom in same as rect size 
				E hzoomu;
				E factor_u = WHRECT_UNCERTAIN_FACTOR; // uncertain rectangle factor
				E factor_zoom_u = WHRECT_UNCERTAIN_FACTOR; // uncertain rectangle factor
				E factor = WHRECT_ZOOMFACTOR; // normal zoom factor
				E factor_zoom_inner = WHRECT_INNER_ZOOM_FACTOR;
				/* --------------------------------  main label point containers  --------------------------------------*/
				// point data containers
				T2d pm; // mouse 
				T2d p;  // constrained 
				T2d p_c;  // constrained  centroid
				T2d p_closest;  // constrained  centroid closest point
				T2d p_c_zoom;  // constrained  centroid in zoom coords
				T2d p_closest_zoom;  // constrained  centroid closest point in zoom coords
				T2d p_tl_zoom;  // constrained  top left coordinate of zoomed roi

				cv::Rect r;
				std::vector<T2d> Pxy;   		// constrained detected
				std::vector<T2d> Pxy_c;   	// constrained detected centroid points
				std::vector<T2d> Pxy_closest; // constrained detected centroid points
				std::vector<T2d> Pxy_prev;   	// constrained detected previous
				std::vector<T2d> Pxy_auto;   	// auto selected points detected previous
				std::vector<unsigned int> indeces;
				std::vector<unsigned int> indeces_prev_auto;
				/* --------------------------------  main image containers  --------------------------------------*/
				cv::Mat img;
				cv::Mat img_;
				cv::Mat zoomed;
				cv::Mat zoomed_invb; // inverted binary trensholded image
				cv::Mat zoomed_auto;
				/* --------------------------------  main derived parameters  --------------------------------------*/
				int cols = 1088;
				int rows = 600;

				E xmax,xmin  ; 
				E ymax,ymin  ;
				E xmax_zoom,xmin_zoom  ; 
				E ymax_zoom,ymin_zoom  ;
				E xmax_zoomu,xmin_zoomu  ; 
				E ymax_zoomu,ymin_zoomu  ;
				/* --------------------------------  state machine detector   --------------------------------------*/
				Status currentStatus;
				KeyState currentKeyState;
				DetectMode currentDetectMode;

				std::vector<E>stridefactor = {1,2,5,10};
				cv::Ptr<cv::SimpleBlobDetector> detector;
				LabelDetector <T_> labelDetector;
				LabelDetector <T_> labelDetectorAuto; // automatic tracking through frames
				// colorfilter
				bool isColorFilterImage = true;
				Pixel colFiltpx = Pixel(255, 20, 50);
				bool isMorphFilter = true;

				// LabelDetector <T_> *labelDetector;

				int closestindex;
				std::vector<cv::KeyPoint> keypoints; 
				std::vector<T2d> P_cglobal;   
				/* --------------------------------  loop parameter  --------------------------------------*/
				bool isNewMarkerData = false; // ! important
				bool isVisited = false;
				bool isShowsPrev = false;
				bool isAutoDetect = true;
				bool isShowsSlidingZoom = false;
				bool isShowAutoThresh = false;
				bool isShowHints = true;
				bool isFastForward = false;
				size_t nmarkersDone = 0;
				long totalFrameCount = 0;
				long frameCounterFinished = 0; // frames annotaded
				long currentFrame= 0; // frames annotaded
				int auto_update = NSTEP_AUTO_DETECT;  // update rate of auto
				/* --------------------------------  image helper  --------------------------------------*/
				std::vector<cv::Scalar> colorMap;
				// draw parameters
				std::string window;
				std::string zoomwindow;
				std::string thresholdwindow = "threshold";
				std::string zoomautowindow = "auto_seq";
				// loop parameters

				/* --------------------------------  constructor and destructor --------------------------------------*/
				LabelData(){};
				LabelData(cv::Mat img_in, int nmarkers_=100, int nframes_=-1, E wrect_=WRECT, E hrect_=HRECT, E factor_=WHRECT_ZOOMFACTOR, E factor_u_ =WHRECT_UNCERTAIN_FACTOR, E factor_zoom_inner_=WHRECT_INNER_ZOOM_FACTOR, DetectMode mode_=DetectMode::AUTO):
					img(img_in), 
					nmarkers(nmarkers_), 
					nframes(nframes_), 
					factor(factor_),
					factor_u(factor_u_),
					factor_zoom_u(factor_), // 
					factor_zoom_inner(factor_zoom_inner_), // 
					wrect(wrect_), hrect(hrect_),
					wzoom(wrect_), hzoom(hrect_),
					wzoomu(wrect_*factor_u_), hzoomu(hrect_*factor_u_),
					zoomed(cv::Mat(wrect_*factor_,hrect_*factor_, CV_8UC3)),
					zoomed_invb(cv::Mat(wrect_*factor_,hrect_*factor_, CV_8UC3)),
					zoomed_auto(cv::Mat(wrect_*factor_*factor_u_,hrect_*factor_*factor_u_, CV_8UC3)),
					currentDetectMode(mode_)
			{
				/* --------------------------------  initialisation derived --------------------------------------*/
				cols = img.cols;
				rows = img.rows;
				// upper and lower boundaries 
				xmax = cols - wrect/2; xmin = wrect/2; // W   
				ymax = rows - hrect/2; ymin = hrect/2; // H
				// max of zoom rectangle 
				xmax_zoom = cols - wzoom/2; xmin_zoom = wrect/2; // W
				ymax_zoom = rows - hzoom/2; ymin_zoom = hrect/2; // H
				// max of zoom rectangle 
				xmax_zoomu = cols - wzoomu/2; xmin_zoomu = wzoomu/2; // W
				ymax_zoomu = rows - hzoomu/2; ymin_zoomu = hzoomu/2; // H
				/* ----------------------  reserve containers   ---------------------------*/
				Pxy.reserve(nmarkers); 
				indeces.reserve(nmarkers); 
				generateColorMap(nmarkers);
				img_ = img.clone();
				std::stringstream ss; ss << "zoom [" << wrect << "x" << hrect << "] x" << factor;
				zoomwindow = ss.str();
				//detector parameters
				keypoints.reserve(100);
				P_cglobal.reserve(100);
				Pxy_auto.reserve(100);


				labelDetector 		= LabelDetector<T_>(wzoom, hzoom, factor);
				labelDetectorAuto  	= LabelDetector<T_>(wzoomu, hzoomu, factor_zoom_u); // WHRECT_UNCERTAIN_FACTOR larger in wrect and has same zoom factor as standard zoom
				// color filtering on
				labelDetector.setColorFilter(colFiltpx);		
				labelDetectorAuto.setColorFilter(colFiltpx);  	
				// morphological filter
				labelDetector.setMorphFilter(MORPH_PIX_ERO,MORPH_PIX_DIL);		
				labelDetectorAuto.setMorphFilter(MORPH_PIX_ERO,MORPH_PIX_DIL); 
				
				cv::namedWindow(thresholdwindow,cv::WINDOW_AUTOSIZE);
				cv::namedWindow(zoomautowindow,cv::WINDOW_AUTOSIZE);

			}
				~LabelData(){std::cout << "LabelData object deleted" << std::endl; }
				// sets image w and h!!!
				void setImage(cv::Mat img_in){
					img_in.copyTo(img);
					img_in.copyTo(img_);
					// img_in.copyTo(img__);
				}

				/* --------------------------------  enum methods --------------------------------------*/
				inline const char* getStatus()
				{
					// enum class Status { IDLE, STORE_POINT, NEW_FRAME, PAUSE, MEASURING,DONE, EXIT, START };

					switch (currentStatus)
					{
						case Status::IDLE:   		return "idle";
						case Status::STORE_POINT: 	return "store point";
						case Status::SKIP_POINT: 	return "skip point";
						case Status::DELETE_POINT: 	return "delete point";
						case Status::CLEAR_POINTS: 	return "clear points";
						case Status::NEW_FRAME: 		return "new frame";
						case Status::PREV_FRAME: 	return "prev frame";
						case Status::PAUSE: 		return "paused";
						case Status::MEASURING:  	return "measuring";
						case Status::DONE: 			return "done";
						case Status::REVISITING: 	return "revisiting";
						case Status::EXIT: 			return "exit";
						case Status::START: 		return "start";
						case Status::ADJUSTING: 	   	return "adjusting";
						case Status::AUTOSET_SEQ: 	return "auto seq set";
						default:      				return "idle";
					}
				}
				inline const char* getDetectMode(){
					switch(currentDetectMode){
						case DetectMode::AUTO:   		return "AUTO";
						case DetectMode::AUTO_SEQ:   		return "AUTO-SEQ";
						case DetectMode::MANUAL:   		return "MANUAL";
						case DetectMode::SEMI_AUTO:   	return "SEMI-AUTO";
						default:      					return "MANUAL";
					}

				}
				/* --------------------------------  inline methods points manipulation --------------------------------------*/
				inline void setDetectMode(DetectMode d){
					currentDetectMode = d;
				}
				inline void setStatus(Status s){
					currentStatus = s;
				}
				inline bool isStatus(Status s){
					// auto flag = (currentStatus==s)? true: false;
					return (currentStatus==s);
				}
				inline bool isDetectMode(DetectMode d){
					// auto flag = (currentDetectMode==d)? true: false;
					return(currentDetectMode==d);
				}
				inline void setPoint(float x, int y){
					pm.x = static_cast<E>(x);
					pm.y = static_cast<E>(y);
					// constrain point cannot exeed the bounds of image since rects is defined as top left coordinate
					x = std::min<E>(x,xmax); x = std::max<E>(x,xmin); p.x = x; 
					y = std::min<E>(y,ymax); y = std::max<E>(y,ymin); p.y = y;
				}
				inline void adjustMarker(cv::Point2f offset=T_(15,-15),int size=2){
					if(Pxy.size()<=nmarkers){

						switch (currentKeyState){
							case KeyState::UP:
								p.y -= PIX_STRIDE*stridefactor.front();
								break;
							case KeyState::DOWN:
								p.y += PIX_STRIDE*stridefactor.front();
								break;
							case KeyState::RIGHT:
								p.x += PIX_STRIDE*stridefactor.front();
								break;
							case KeyState::LEFT:
								p.x -= PIX_STRIDE*stridefactor.front();
								break;
								// ---------------------------------------------> hitting enter when auto mode sets zoom and p.x automatically automatically updates the 
							case KeyState::SPACEBAR:
								// if(isDetectMode(DetectMode::AUTO)){
								if(isDetectMode(DetectMode::AUTO) or isDetectMode(DetectMode::AUTO_SEQ)){
									p = (p_c==nanp)? p : p_c; // set at detected centroid
								}
								// else no extra action required
								break;
						}
						p.x = std::min<E>(p.x,xmax); p.x = std::max<E>(p.x,xmin);
						p.y = std::min<E>(p.y,ymax); p.y = std::max<E>(p.y,ymin);

						cv::rectangle(img_, centeredRect(p.x,p.y,wrect,hrect), COL_WHITE, size);
						cv::drawMarker(img_, p,  COL_WHITE, cv::MARKER_CROSS, 10, 1);
						std::stringstream ss; ss << " [" << nmarkersDone <<  + "]";
						cv::putText(img_,ss.str(),p +offset,  FONT_YINFO,FONTSIZE_YINFO,COL_YINFO,size/2);

					} else{
						currentStatus= Status::DONE;
					}
				}
				inline void setMarker(){
					if(Pxy.size()<=nmarkers)
					{
						if(isDetectMode(DetectMode::AUTO) or isDetectMode(DetectMode::AUTO_SEQ)){
							Pxy.emplace_back(p_c);
							Pxy_closest.emplace_back(p_closest);
							indeces.emplace_back(nmarkersDone);
						} else{
							Pxy.emplace_back(p);
							Pxy_closest.emplace_back(nanp);
							indeces.emplace_back(nmarkersDone);
						}
						nmarkersDone++;
						isNewMarkerData=true; // sets the flag new data true when we add marker or new marker in new frame 
					} else{
						currentStatus= Status::DONE;
					}
				}
				inline void skipMarker(){
					if(Pxy.size()<=nmarkers)
					{
						Pxy.emplace_back(nanp);
						indeces.emplace_back(nmarkersDone);
						nmarkersDone++;
						// TODO check if causes no problems in map structure
						isNewMarkerData=true; 
					} else{
						currentStatus= Status::DONE;
					}
				}
				inline void deleteMarker(){
					if(!Pxy.empty())
					{
						Pxy.pop_back();
						indeces.pop_back();
						nmarkersDone--;
						isNewMarkerData=true;
					}
				}
				inline void setPredictedMarkers(){
					if(!Pxy_prev.empty())
					{
						Pxy = Pxy_auto;
						indeces = indeces_prev_auto; //--> same as previous index
						nmarkersDone = Pxy.size();
						isNewMarkerData=true;
					}
				}
	inline cv::Rect centeredRect(E x, E y, E wrect=WRECT, E hrect=HRECT){
		cv::Rect_<E> r( x - wrect/2, y - hrect/2,wrect,hrect);
		return r;
	}
	inline T2d toImgCoords(cv::KeyPoint kp, E x, E y, E factor ,E wzoom_=WRECT, E hzoom_=HRECT ){				 
		T2d p_global = T2d(x-wzoom_/2,y-hzoom_/2); 				// scale 
		T2d p = kp.pt/factor + p_global; // scale back from zoom and translate to global coords from top left coordinate of roi
		return p;
	}
	inline T2d fromImgCoords(T2d p_global, E x, E y, E factor ,E wzoom_=WRECT, E hzoom_=HRECT ){				 
		T2d p_zoom = T2d(x-wzoom_/2,y-hzoom_/2); 				// scale 
		T2d p = (p_global-p_zoom)*factor; // scale back from zoom and translate to global coords from top left coordinate of roi
		return p;
	}
	/* --------------------------------  inline methods image manipulation --------------------------------------*/
	inline void clearImage(){
		img_ = img.clone();
	}
	inline void getImage(cv::Mat &img_in){
		// drawPreviousPoints(true); // draw the rects before passing back, since points are cleared Pxy=Prev, draw previous oivveride is show prev flag
		drawPreviousPoints(); // draw the rects before passing back, since points are cleared Pxy=Prev, draw previous oivveride is show prev flag
		img_.copyTo(img_in);
	}
	inline void saveSnapZoom(){
		cv::imwrite("zoomedsnap_"+std::to_string(nmarkersDone) +".jpg",zoomed);
	}
	inline void saveSnap(){
		cv::imwrite("snap"+std::to_string(nmarkersDone) +".jpg",img_);
	}
	inline void generateColorMap(size_t n){
		cv::RNG rng(12345);
		colorMap.reserve(n);
		for(int i=0;i<n;i++){
			colorMap.push_back(cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255)));
		}
	}
	/* --------------------------------  inline methods data gui manipulation --------------------------------------*/
	inline void clearLabelData(){
		Pxy_prev = (Pxy.empty())? Pxy_prev: Pxy; // keep prediction when moving through frames
		indeces_prev_auto = (indeces.empty())? indeces_prev_auto: indeces; // keep prediction when moving through frames
		nmarkersDone = 0;
		isNewMarkerData = 0;
		Pxy.clear();
		Pxy_c.clear();				
		Pxy_closest.clear();
		Pxy_auto.clear();
		indeces.clear();
		isVisited = false;
		p_closest = nanp; // in case of semi_auto mode may not be closest point present
	}
	inline void advance(){
		clearLabelData();
		currentFrame++;
	}
	inline void stepBack(){
		clearLabelData();
		currentFrame = currentFrame==0? currentFrame: currentFrame-1 ;
	}
	/* --------------------------------  detection methods --------------------------------------*/
	void detectZoom(int thickness=1){
		// get x,y pos and ensure boundaries. when wrect=wzoom etc than no changes with normal contrained x,y
		auto x = p.x;	x = std::min<E>(x,xmax_zoom); x = std::max<E>(x,xmin_zoom); //double wzoomed = wzoom*factor; 
		auto y = p.y;	y = std::min<E>(y,ymax_zoom); y = std::max<E>(y,ymin_zoom); //double hzoomed = hzoom*factor;

		// TODO fix this.  // int nstep = (currentStatus==Status::IDLE or currentStatus==Status::ADJUSTING)?  1 : auto_update;
		int nstep = 1;
		if((currentDetectMode==DetectMode::AUTO or currentDetectMode==DetectMode::AUTO_SEQ or currentDetectMode==DetectMode::SEMI_AUTO ) and !(totalFrameCount % nstep)){ // if the rtemainder is zedro

			/* -------------------- loop through image and detect points automatically -----------------*/
			if(isColorFilterImage){

			}

			labelDetector.detect(img,x,y);

			closestindex = labelDetector.getDetectedPoints(p_c,p_c_zoom, p_closest,p_closest_zoom,
													P_cglobal, keypoints);
			zoomed_invb  = labelDetector.getThresholdImg();

		} else{
			// in case no points found just choose the middle of the frame
			closestindex = labelDetector.clearDetectPoints( p_c, p_c_zoom, p_closest,p_closest_zoom, 
												       P_cglobal, keypoints);
		}
		zoomed 				= labelDetector.getZoomImg(); //.clone();
		p_tl_zoom 			= T2d(x,y);

	}
	void drawZoom(){
		E wzoomed = wzoom*factor; 
		E hzoomed = hzoom*factor;

		// TODO fix this // int nstep = (currentStatus==Status::IDLE or currentStatus==Status::ADJUSTING)?  1 : auto_update;
		int nstep = 1;
		if((currentDetectMode!=DetectMode::MANUAL ) and !(totalFrameCount % nstep)){ // if the rtemainder is zedro

			// draw keypoints
			cv::drawKeypoints(zoomed, keypoints, zoomed, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

			// draw markers in zoom and normal window
			for(auto kp : keypoints)
				cv::drawMarker(zoomed, kp.pt, COL_RED, cv::MARKER_CROSS, 50, 2);
			for(auto  p : P_cglobal)
				cv::drawMarker(img_, p , COL_ORANGE, cv::MARKER_CROSS, 20, 2);

			// draw centroids
			cv::drawMarker(zoomed, p_c_zoom , COL_GREEN, cv::MARKER_DIAMOND, 50, 2);
			cv::drawMarker(img_, p_c , COL_GREEN, cv::MARKER_DIAMOND, 10, 1);

			// draw closest point
			if(closestindex>=0){
				cv::drawMarker(zoomed, p_closest_zoom  , COL_BLUE, cv::MARKER_TILTED_CROSS, 50, 2);
				cv::drawMarker(img_, p_closest , cv::Scalar(255,0,255), cv::MARKER_TILTED_CROSS, 10, 1);
			}
			// show thresholded image in auto mode
			cv::imshow(thresholdwindow,zoomed_invb);
		} else{
			// cv::Mat zoomed_b = cv::zeros(zoomed_.size(),CV_8UC3);
			cv::imshow(thresholdwindow,cv::Mat(1,zoomed.cols,CV_8UC3));

		}
		drawCenterLinesInfo(zoomed, wzoomed,hzoomed); // draw centerlines

		if(currentStatus==Status::STORE_POINT) // blink green to indicate the point is stored
			cv::drawMarker(zoomed, T_(wzoomed/2,hzoomed/2) , COL_GREEN, cv::MARKER_CROSS, 50, 2);
		// zoomed = zoomed_;
	}
	void detectPredictAuto(){

		std::vector<T_> Pxy_auto_; Pxy_auto_.reserve(100);
		int k = 0;
		for(const auto &p_: Pxy_prev){

		int nstep = 1;
		if((currentDetectMode==DetectMode::AUTO_SEQ ) and !(totalFrameCount % nstep)){ // if the rtemainder is zedro
			
			// get x,y pos and ensure boundaries. when wrect=wzoom etc than no changes with normal contrained x,y
			auto x = p_.x;	x = std::min<E>(x,xmax_zoomu); x = std::max<E>(x,xmin_zoomu); //double wzoomed = wzoomu*factor_whu; 
			auto y = p_.y;	y = std::min<E>(y,ymax_zoomu); y = std::max<E>(y,ymin_zoomu); //double hzoomed = wzoomu*factor_whu;

			/* -------------------- run detect  -----------------*/
			labelDetectorAuto.detect(img,x,y);

			auto p_c_                 = labelDetectorAuto.p_c;            
			auto p_c_zoom_            = labelDetectorAuto.p_c_zoom;    

			// draw centroid
			cv::drawMarker(img_, p_c_ , COL_GREEN, cv::MARKER_DIAMOND, 20, 1);
			Pxy_auto_.emplace_back(p_c_);
			/* --------------------           -----------------*/
			if(labelDetectorAuto.isInRect(pm.x,pm.y)){
				// zoomed_auto 	= labelDetectorAuto.getZoomImgAny(img,x,y,factor); //.clone();
				zoomed_auto 	= labelDetectorAuto.getZoomImg(); //.clone();
				cv::drawMarker(zoomed_auto, p_c_zoom_ , COL_GREEN, cv::MARKER_DIAMOND, 50, 2);
				cv::drawKeypoints(zoomed_auto, labelDetectorAuto.keypoints, zoomed_auto, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			if(isShowAutoThresh)
				cv::imshow("zoomautowindow",labelDetectorAuto.getThresholdImg());
			
			}

		} else{
			Pxy_auto_.emplace_back(p_);
		}
			Pxy_auto = Pxy_auto_;
		}
	}
	void drawPredictAuto(){
		if(isDetectMode(DetectMode::AUTO_SEQ)){
			drawCenterLinesInfo(zoomed_auto, wzoom*factor*factor_u, hzoom*factor*factor_u);
			cv::imshow(zoomautowindow,zoomed_auto);
		}
	}
	void drawSlidingMouseZoom(){
		E xyshift = 30;
		if(isShowsSlidingZoom){
			img_ = labelDetector.putZoomImgAny(img_,p.x,p.y, factor_zoom_inner);//, p.x+xyshift,p.y+xyshift);
			// cv::imshow("zoomautowindow",img__);
		}
	}
	void setDetectFilter(){ // sets the morph and color filter
		labelDetector.setColorFilter(colFiltpx);		
		labelDetectorAuto.setColorFilter(colFiltpx);  	
		// morphological filter
		labelDetector.setMorphFilter(MORPH_PIX_ERO,MORPH_PIX_DIL);		
		labelDetectorAuto.setMorphFilter(MORPH_PIX_ERO,MORPH_PIX_DIL); 
	}
	void toggleDetectFilter(){ // sets the morph and color filter
		labelDetector.isColorFilterImage = isColorFilterImage;		
		labelDetector.isMorphFilter = isMorphFilter; 

		labelDetectorAuto.isColorFilterImage = isColorFilterImage;		
		labelDetectorAuto.isMorphFilter = isMorphFilter; 
	}
	/* --------------------------------  main draw methods --------------------------------------*/
	void drawPreviousPoints(bool ovveridePrev=false, cv::Point2f offset=cv::Point2f(15,-15),int size=2){
		if(isShowsPrev or ovveridePrev){
			int k = 0;
			for(const auto &p: Pxy_prev){
				cv::rectangle(img_, centeredRect(p.x,p.y,wrect,hrect), cv::Scalar::all(100), size);
				// cv::rectangle(img_, cv::Point2f(p.x-(wrect/2),p.y-(hrect/2)),cv::Point2f(p.x+(wrect/2),p.y+(hrect/2)), colorMap[k], size);
				cv::circle(img_,p, size, colorMap[k], -1);
				cv::putText(img_,"[" + std::to_string(indeces[k]) + "]",p +offset, FONT_YINFO,.5,cv::Scalar::all(100),size/2);
				k++;
			}
		}
	}
	void drawCenterLinesInfo(cv::Mat &zoomed_, E wzoomed, E hzoomed, int thickness=1){
		cv::line(zoomed_,cv::Point(0 	 	  , hzoomed/2), cv::Point(wzoomed   , hzoomed/2),COL_WHITE,thickness);
		cv::line(zoomed_,cv::Point(wzoomed/2 ,         0), cv::Point(wzoomed/2 , hzoomed  ),COL_WHITE,thickness);

		// std::stringstream ss; ss << "x/xmouse, y/ymouse = (" << p.x << "/" << pm.x <<  "," << p.y << "/" << pm.y << ")";
		std::stringstream ss;  ss  << "p_lim = " << p << ", " << "p_mouse = " << pm;
		cv::putText(zoomed_,ss.str(),cv::Point(20,10), FONT_YINFO,FONTSIZE_YINFO*0.8,COL_YINFO,1);
		std::stringstream ss2; ss2 << "p_c   = " << p_c_zoom << ", " << "p_closest = " << p_closest;
		cv::putText(zoomed_,ss2.str(),cv::Point(20,20), FONT_YINFO,FONTSIZE_YINFO*0.8,COL_YINFO,1);
	}
	void drawCenteredRect(cv::Point2f offset=T_(15,-15),int size=2){
		int k = 0;
		for(const auto &p: Pxy){
			cv::rectangle(img_, centeredRect(p.x,p.y,wrect,hrect), colorMap[k], size);
			// cv::rectangle(img_, cv::Point2f(p.x-(wrect/2),p.y-(hrect/2)),cv::Point2f(p.x+(wrect/2),p.y+(hrect/2)), colorMap[k], size);
			cv::circle(img_,p, size, colorMap[k], -1);
			cv::putText(img_,"[" + std::to_string(indeces[k]) + "]",p +offset, FONT_YINFO,.5,colorMap[k],size/2);
			k++;
		}
	}
	void drawUncertainRect(cv::Point2f offset=T_(15,-15),int size=1){
		int k = 0;
		if(currentDetectMode == DetectMode::AUTO_SEQ){
		for(const auto &p: Pxy_auto){
			cv::rectangle(img_, centeredRect(p.x,p.y,wzoomu,hzoomu), COL_WHITE, size);
			// cv::rectangle(img_, cv::Point2f(p.x-(wrect/2),p.y-(hrect/2)),cv::Point2f(p.x+(wrect/2),p.y+(hrect/2)), colorMap[k], size);
			cv::circle(img_,p, size, colorMap[k], -1);
			cv::putText(img_,"[" + std::to_string(indeces[k]) + "]",p +offset, FONT_YINFO,.5,COL_WHITE,size/2);
			k++;
		}
	}
	}
	void drawMouseRect(cv::Point2f offset=T_(15,-15),int size=2){
		cv::rectangle(img_, centeredRect(pm.x,pm.y,wrect,hrect), COL_WHITE, size, cv::LINE_AA );
		// cv::rectangle(img_, cv::Point2f(pm.x-(wrect/2),pm.y-(hrect/2)),cv::Point2f(pm.x+(wrect/2),pm.y+(hrect/2)), cv::Scalar(255,255,255), size, cv::LINE_AA );
		if(currentStatus== Status::ADJUSTING){
			cv::drawMarker(img_, pm,  COL_WHITE, cv::MARKER_CROSS, 10, 1);
		} else {
			cv::circle(img_,p, size,  COL_WHITE, -1);
		}
	}
	void updateWindow(){
		// cv::imshow("window",img__);
		cv::imshow(window,img_);
		cv::imshow(zoomwindow,zoomed);

		img.copyTo(img_);
		totalFrameCount++;
	}
	std::stringstream getInfo(){
		std::stringstream ss; 
		// ss << "Frame [ " 		<< currentFrame 		<< "/" << nframes <<" ], " 	
		ss   << "Marker[ " 		<< nmarkersDone      			  << " ], " 
			<< "Frame [ " 		<< currentFrame 			<< "/" << nframes <<" ], "
			<< "Visited [ " 	<< isVisited 				<< " ], "
			<< "nAnnotated [ "	<< frameCounterFinished     	<< " ], " 	 		
			<< "pos = "		<< p   				  	<< " , " 	
			<< "stride [ x"	<< stridefactor.front() 		<< " ], " 	
			// << "FPS   [ " 		<< fps          			<< " ], " 
			;	
		return ss;
	}
	std::vector<std::string> getHintMsg(){
		std::vector<std::string> ssv; ssv.reserve(10);
	if(isStatus(Status::ADJUSTING) and (isDetectMode(DetectMode::AUTO) or isDetectMode(DetectMode::AUTO_SEQ))){
		ssv.emplace_back("[ Press "); ssv.emplace_back("SPACEBAR or Click"); ssv.emplace_back(" to save predicted marker ]");
	} else if(isStatus(Status::AUTOSET_SEQ) and isDetectMode(DetectMode::AUTO_SEQ)){
		ssv.emplace_back("[ Press "); ssv.emplace_back("ENTER or UP"); ssv.emplace_back(" to save predicted marker sequence ]");
	}
	else{
		ssv.emplace_back("[ Hover mouse and click to set marker ]");
	}
	return ssv;
	}
	std::stringstream getToggleInfo(){
		std::stringstream ss; ss << "TOGGLE [ ";
		if(isShowsPrev)
			ss << "PREV ON ";
		if(isShowsSlidingZoom)
			ss << "ZOOM ON ";
		if(isFastForward)
			ss << "AUTOFF ON ";
		if(!isShowsPrev and !isShowsSlidingZoom and !isFastForward)
			ss << "OFF ";
		ss << "] ";
		return ss;
	}
	void writeInfo(double fps=-1){                
		std::stringstream status_ss; status_ss<< "STATUS [ " << getStatus() << " ]";

		auto ss = getInfo(); auto toggle_ss = getToggleInfo();
		// status
		cv::putText(img_,status_ss.str(), T_(100,30),  	FONT_YINFO,FONTSIZE_YINFO,COL_YINFO,TICKNESS_YINFO);
		// toggle opt
		cv::putText(img_,toggle_ss.str(), T_(100,50),  	FONT_YINFO,FONTSIZE_YINFO,COL_YINFO,TICKNESS_YINFO);
		// info
		cv::putText(img_,ss.str(), T_(400,30),  		FONT_YINFO,FONTSIZE_YINFO,COL_YINFO,TICKNESS_YINFO);
		cv::putText(img_,getDetectMode(), T_(100,70),  	FONT_YINFO,FONTSIZE_YINFO,COL_BLUE,TICKNESS_YINFO);

		// write hint
		if(isShowHints)
			drawKeyHints();
	}
	void writeInfo(cv::Mat &img_in, double fps=-1){ // laternative implementation
		std::stringstream status_ss; status_ss<< "STATUS [ " << getStatus() << " ]";

		auto ss = getInfo(); auto toggle_ss = getToggleInfo();
		// status
		cv::putText(img_in,status_ss.str(), T_(100,30),  	FONT_YINFO,FONTSIZE_YINFO,COL_YINFO,TICKNESS_YINFO);
		// toggle opt
		cv::putText(img_in,toggle_ss.str(), T_(100,50),  	FONT_YINFO,FONTSIZE_YINFO,COL_YINFO,TICKNESS_YINFO);
		// info
		cv::putText(img_in,ss.str(), T_(400,30),  		FONT_YINFO,FONTSIZE_YINFO,COL_YINFO,TICKNESS_YINFO);
		cv::putText(img_in,getDetectMode(), T_(100,70),  	FONT_YINFO,FONTSIZE_YINFO,COL_BLUE,TICKNESS_YINFO);
	}
	void updateOnkeyPress(){
		detectZoom();
		drawZoom();
		drawCenteredRect();
		detectPredictAuto();
		drawPredictAuto();
		drawUncertainRect();
		drawCenteredRect();
		drawPreviousPoints();
		drawSlidingMouseZoom();
		writeInfo();
		updateWindow();
	}
	// store data return structure datawriter
	LabelWriter<T_> storeSetDataWriter(){
		LabelWriter<T_> datawriter(Pxy, indeces, currentFrame, nmarkersDone); //isVisited set to true
		// LabelWriter(std::vector<T_> Pxy_ ,std::vector<unsigned int> indeces_, int frameId, int ndone_): 
		return datawriter;
	}
	// store data return structure datawriter
	void loadSavedDataWriter(const LabelWriter<T_> &datawriter){

		Pxy 		= datawriter.Pxy;
		indeces 	= datawriter.indeces;
		nmarkersDone	= datawriter.nmarkersDone;
		isVisited = datawriter.isVisited;
		//currentFrame is advanced through advance 
	}
	void drawKeyHints(){
		// auto ss = getInfo(); auto toggle_ss = getToggleInfo();
		auto ssv = getHintMsg();
	     // message trailing
	     if(ssv.size()>=1){
	     	// message leading
			cv::putText(img_,ssv.front(), T_(400,50), FONT_YINFO,FONTSIZE_YINFO,COL_WHITE,TICKNESS_YINFO);
		}
		if(ssv.size()==3){
			cv::putText(img_,ssv.back(), T_(620,50), FONT_YINFO,FONTSIZE_YINFO,COL_WHITE,TICKNESS_YINFO);
			// key combo
			cv::putText(img_,ssv[1], T_(480,50), FONT_YINFO,FONTSIZE_YINFO,COL_YINFO,TICKNESS_YINFO2);
		}

	}
	/* ------------------------------------------- helper functions-------------------------------------------*/
	E euclideanDist(T2d& a, T2d& b){
		T2d diff = a - b;
		return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
	}
};
/* -------------------------------------------end of class  -------------------------------------------*/

/* -------------------------------------------end of namespace  -------------------------------------------*/
}



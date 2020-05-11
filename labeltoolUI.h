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
 * Description: 	     Labeltool UI class: Ui definitions
 *                       dependencies: 
 * 					example compile: make
 *					then run with:   ./runMain
 *
 * Author: 			Tigran Mkhoyan
 * Email : 			t.mkhoyan@tudelft.nl
 *
 */

// #pragma once 
namespace  invdbscan{

	/* -------------------------------------------class to manage tool and ui -------------------------------------------*/
	template<typename T>
		class LabelTool
		{
			public:
				typedef typename T::value_type E;
				typedef cv::Point_ <E> T2d;  		// needed to define 2d point
				typedef cv::Point3_<E> T3d;  		// needed to define 2d point
				typedef std::chrono::high_resolution_clock clk;
				typedef std::chrono::high_resolution_clock::time_point clkTp;

				enum class Mode { LIST, CAM };

				inline cv::Rect centeredRect(int x, int y, int wrect=WRECT, int hrect=HRECT){
					cv::Rect r( x - wrect/2, y - hrect/2,wrect,hrect);
					return r;
				}
				/* --------------------------------------  static  parameters  ----------------------------------------------*/

				/* --------------------------------------  main parameters  -------------------------------------------------*/
				int nmarkers =14;
				int nprevImg = 3;
				int nframes=0;
				bool isShowInfo = true;
				bool isShowPrevImg = true;
				bool isRecord = true;
				/* --------------------------------------  main containers  -------------------------------------------------*/
				LabelData<T> labelData; //class containing image, point and state
				// LabelData *ptrLabelData;
				std::vector<cv::Mat> recImg; 
				std::vector<cv::Mat> imgt; 
				Mode camopt = Mode::LIST;
				cv::VideoCapture cap;

				std::map<unsigned int, LabelWriter<T>> datawritersMap;
				/* --------------------------------------- file storage  param  ----------------------------------------------*/
				cv::FileStorage fs_i;
				cv::FileStorage fs_o;

				std::string flname_in = IN_FL_NAME;
				std::string flname_out = OUT_FL_NAME;
				std::string flname_out_nested = OUT_FL_NAME_N;
				std::string out_path = OUT_FL_PATH;
				std::string line;
				std::vector<std::string> lines;
				/* --------------------------  label data shared param   ------------------------------------------*/
				std::string annotateWindow = "Annotation";
				std::string previmgWindow =  "Previous";

				int rows = IMG_ROWS; //H 
				int cols = IMG_COLS; //W

				Status currentEvent = Status::IDLE;


				/* --------------------------  loop param   ------------------------------------------*/
				std::time_t timeBegin;
				int keyPressed = -1;
				int prevkeyPressed = -1;
				int tick = 0;
				double fps=0;
				long currentFrame = 0;
				long frameFpsCounter = 0;
				int recframe_cnt=0;
				int k=0;
				/* ##------------------------------------
				   --  constructor -----------------------------------------------------*/
				LabelTool(int nmarkers_, std::string flname_in_,std::string flname_out_=OUT_FL_NAME, std::string  out_path_=OUT_FL_PATH ): 
					nmarkers(nmarkers_),
					flname_in(flname_in_), 
					flname_out(flname_out_), 
					out_path(out_path_)
			{
				makeValidDir(out_path); //out_path = out_path+"/"; 
				flname_out = out_path+flname_out;
				flname_out_nested = out_path+flname_out_nested;

			}
				~LabelTool(){std::cout << "LabelTool object deleted" << std::endl; }; //use brackets otherswise produces an error
				void init(){

					cv::Mat img;
					/*---------------    get image data-------------------   */
					std::ifstream input;
					switch (camopt){
						case Mode::LIST: 
							input = std::ifstream(flname_in);
							// Read lines as long as the file is
							while (std::getline(input, line))              
								lines.push_back(line);

							nframes = lines.size();
							img = imread(lines[0].c_str(),cv::IMREAD_COLOR);
							break;
						case Mode::CAM: 
							cap.open(flname_in);
							fps = cap.get(cv::CAP_PROP_FPS);
							nframes = cap.get(cv::CAP_PROP_FRAME_COUNT);
							cap >> img;

							break;
					}
					/*---------------    initi label data-------------------   */
					// initiate data object to pass to callback functions
					labelData = LabelData<T>(img,nmarkers,nframes);
					labelData.window = annotateWindow;

					cols = labelData.img.cols; 
					rows = labelData.img.rows; 

					int WHmax = std::max(rows,cols);
					int WHmin = std::min(rows,cols);
					auto Wzoom = labelData.wzoom;
					auto Hzoom = labelData.hzoom; 
					int xmove = XO_POS; 
					imgt.resize(nprevImg,cv::Mat(rows,cols,CV_8UC3)); // previous images

					/*---------------    sort windows -------------------   */
					cv::namedWindow(previmgWindow,cv::WINDOW_NORMAL);
					cv::namedWindow(labelData.zoomwindow,cv::WINDOW_AUTOSIZE);
					cv::namedWindow(annotateWindow,cv::WINDOW_NORMAL); //cv::WINDOW_KEEPRATIO)
					/*---------------   fix this -------------------   */

					// cv::resizeWindow(labelData.zoomwindow	, Wzoom, Hzoom);
					cv::resizeWindow(annotateWindow		, cols*IMG_SCALE, rows*IMG_SCALE);
					cv::resizeWindow(previmgWindow,WHmin, WHmax); // 

					// // move windows
					// 				cv::moveWindow(labelData.zoomwindow, 	xmove ,0); xmove+= Wzoom;
					// cv::moveWindow(annotateWindow, 		xmove ,0); xmove+= W*IMG_SCALE;
					// cv::moveWindow(previmgWindow,  		xmove ,0);

					// TODO: clean up
					if(isRecord)
						recImg.reserve(nframes);

					/*---------------    call back setup -------------------   */
					setCallBack();
					timeBegin = std::time(0);
					// ptrLabelData =  &labelData;
					labelData.setStatus(Status::START);
				}
				void sample(int & keyPressed){

					if(labelData.isStatus(Status::NEW_FRAME) or labelData.isStatus(Status::REVISITING) or labelData.isStatus(Status::START)){

						cv::Mat img(rows,cols,CV_8UC3);
						img = getImage(labelData.currentFrame); 
						// copy image to lavel data

						// --------------------->  updates only after key press otherwise stuck in mouse callback
						labelData.setImage(img);
						// <---------------------  the we get the original image with the selected markers
						labelData.getImage(img);

						// compile into one stacked image
						cv::Mat combined = cv::Mat(0,cols,CV_8UC3); 
						for(auto img_ : imgt ){
							cv::vconcat(img_,combined,combined);
						}

						// cv::Mat img_clone = img.clone(); 
						std::rotate(imgt.begin(), imgt.begin()+1,imgt.end()); imgt.back() = img.clone();

						std::time_t timeNow = std::time(0) - timeBegin; // calculates only integer seconds

						if(isShowInfo)
							labelData.writeInfo(img, fps);

						if(isShowPrevImg)
							cv::imshow(previmgWindow,combined);

						cv::imshow(annotateWindow,img);
					}

					onKeyPress(labelData.currentFrame); // increments frame counter

					// ---------------------------------------------- 
					frameFpsCounter++;
					if(labelData.currentFrame>nframes){
						// TODO: prompt
						// status
					}
				}
				void setCallBack(){
					cv::setMouseCallback( annotateWindow, onMouse,&labelData );
					// call window
					cv::imshow(annotateWindow,labelData.img);
					// cv::waitKey(0); if called will be stuck here untill keypress
				}

				/* ##---------------------------------------   helper  data writer  -----------------------------------------------------*/
				// void printMapData(const std::map<unsigned int, invdbscan::LabelWriter<point2f> > &datawritersMap ){
				void printMapData(){
					for(const auto &m: datawritersMap){  // more efficient no copy
						std::cout << m.first <<  '\t' << m.second << std::endl;
					}
				}
				// void writeMapData(const std::map<unsigned int, invdbscan::LabelWriter<point2f> > &datawritersMap){
				void writeMapData(){ 	
					cv::FileStorage fs_unrolled;
					cv::FileStorage fs_nested;
					fs_nested.open(	flname_out_nested ,cv::FileStorage::WRITE);   
					fs_unrolled.open(	flname_out 	   ,cv::FileStorage::WRITE);   

					auto header_str = "n";
					for(const auto &m: datawritersMap){  // more efficient no copy
						// id and data nested

						fs_nested << (header_str+std::to_string(m.first)) << m.second;
						// id and data self
						m.second.writeThis(fs_unrolled); // pointer object
					}
				}
				/* ##---------------------------------------   helper  img  -----------------------------------------------------*/
				cv::Mat getImage(int currentcnt){

					cv::Mat img_ = cv::Mat(rows,cols,CV_8UC3);

					std::string line;
					size_t index;

					switch (camopt){
						case Mode::LIST: 
							index =  currentcnt  % lines.size();
							// std::size_t index = std::addressof()-lines.data();  
							line = lines[index];
							img_ = imread(line.c_str(),cv::IMREAD_COLOR);

							break;
						case Mode::CAM: 
							cap>> img_;
							break;
					}

					if (img_.empty()){ 
						std::cout << "Error loading the image" << std::endl;
						img_ = cv::Mat(rows,cols,CV_8UC3);
					}

					return img_;
				}
				/* ##---------------------------------------   helper   -----------------------------------------------------*/
				inline void onKeyPress(long &currentFrame){
					getKeyStroke();

					labelData.currentKeyState = getKeyState(keyPressed);
					if(labelData.currentStatus!= Status::ADJUSTING){

						switch(labelData.currentKeyState){
							case KeyState::Q:
								labelData.setStatus(Status::EXIT);
								std::cout << "save data, exit" << std::endl;
								printMapData();
								writeMapData();
								genExit();
								break;

							case KeyState::SPACEBAR: // save auto selected point
								if(labelData.isDetectMode(DetectMode::AUTO) or   // hitting enter to label
										labelData.isDetectMode(DetectMode::AUTO_SEQ)){
									labelData.setStatus(Status::ADJUSTING);
									labelData.adjustMarker(); // call to correct positon of zoom
									labelData.setStatus(Status::STORE_POINT);
									labelData.setMarker();
									labelData.updateOnkeyPress();
								}
								break; 
								// case KeyState::SPACEBAR: // save auto selected point
							case KeyState::ENTER: // save auto selected point
							case KeyState::UP: // save auto selected point
								if(labelData.isDetectMode(DetectMode::AUTO_SEQ)){
									labelData.updateOnkeyPress();
									labelData.setStatus(Status::AUTOSET_SEQ);
									labelData.detectPredictAuto(); //--> not really needed to run again 
									labelData.setPredictedMarkers();
									labelData.updateOnkeyPress();
								}
								break; 

							case KeyState::C: // copuy from auto
								if(labelData.isDetectMode(DetectMode::AUTO_SEQ)){
									labelData.setStatus(Status::AUTOSET_SEQ);
									labelData.setPredictedMarkers();
									labelData.updateOnkeyPress();
								}
								break; 

								// TODO cleanup check waitkey statemets
								// moving through frames ---> 
							case KeyState::RIGHT:
								if(labelData.isFastForward and !labelData.isStatus(Status::AUTOSET_SEQ)) {
									// carry out case ENTER
									if(labelData.isDetectMode(DetectMode::AUTO_SEQ)){
									labelData.updateOnkeyPress();

										labelData.setStatus(Status::AUTOSET_SEQ);
										labelData.detectPredictAuto(); //--> not really needed to run again 
										labelData.setPredictedMarkers();
										labelData.updateOnkeyPress();
										cv::waitKey(1); // needed to break loop
									}
								} else{ // advance ENTER was carried out
									std::cout << "labelData.getStatus()" << labelData.getStatus() << std::endl; 
								evaluateCurrentAnnotations();  // saves if needed 
								labelData.advance(); // clears points
								evaluatePreviousAnnotations(); // clear points loads
								labelData.updateOnkeyPress();
								// cv::waitKey(1); // needed to break loop
								}
								break;

							case KeyState::LEFT:
								evaluateCurrentAnnotations();  // saves if needed 
								labelData.stepBack(); // clears points							 
								evaluatePreviousAnnotations(); // clear points loads
								labelData.updateOnkeyPress();
								std::cout << " previous frame" << std::endl;
								break;

							case KeyState::S:
								labelData.saveSnap();
								labelData.saveSnapZoom();
								std::cout << " saved snap" << std::endl;
								break;

							case KeyState::M: //toggle
								labelData.currentDetectMode++; // increment
								labelData.updateOnkeyPress();
								std::cout << "toggle mode: " << labelData.getDetectMode() <<  std::endl;
								break;

							case KeyState::A: 
								if(isKeyStrokeSeq(KeyState::A, KeyState::A)) // double A
									labelData.setDetectMode(DetectMode::AUTO_SEQ);
								else 
									labelData.setDetectMode(DetectMode::AUTO);

								labelData.updateOnkeyPress();
								std::cout << "toggle mode: " << labelData.getDetectMode() <<  std::endl;
								break;

							case KeyState::ARROWR:
								labelData.setStatus(Status::SKIP_POINT);
								labelData.skipMarker();
								labelData.updateOnkeyPress();
								std::cout << " marker ignored" << std::endl;
								break;

							case KeyState::ARROWL:
								labelData.setStatus(Status::DELETE_POINT);
								labelData.deleteMarker();
								labelData.updateOnkeyPress();
								std::cout << " marker deleted" << std::endl;
								break;

							case KeyState::BACKSPACE:
								labelData.setStatus(Status::CLEAR_POINTS);
								labelData.clearLabelData();
								deleteCurrentAnnotation();
								labelData.updateOnkeyPress();
								std::cout << " marker cleared" << std::endl;
								break;

							case KeyState::W:
								writeMapData();
								std::cout << " data written to file: " << flname_out  << std::endl;
								break;

							case KeyState::P:
								if(isKeyStrokeSeq(KeyState::P,KeyState::P)){
									labelData.isShowsPrev = labelData.isShowsPrev==true ? false: true;
									std::cout << " toggle show previous" << std::endl;
								}
								else{
									printMapData();
								}
								break;

							case KeyState::H:
								labelData.isShowHints = labelData.isShowHints==true ? false: true;
								std::cout << " toggle show hints" << std::endl;
								break;

							case KeyState::F:
								if(isKeyStrokeSeq(KeyState::F,KeyState::F)){
									labelData.isMorphFilter = labelData.isMorphFilter==true ? false: true;
									labelData.toggleDetectFilter();
									std::cout << " toggle custom morphological [" << labelData.isMorphFilter <<  "] and color [" << labelData.isColorFilterImage <<  "] filter on detection: [" << std::endl;
								}else{
									labelData.isColorFilterImage = labelData.isColorFilterImage==true ? false: true;
									labelData.toggleDetectFilter();
									std::cout << " toggle custom color [" << labelData.isColorFilterImage <<  "] filter on detection" << std::endl;
								} 
								break;

							case KeyState::T: // actibvate trehsold for auto-sew
								labelData.isShowAutoThresh = labelData.isShowAutoThresh==true ? false: true;
								std::cout << " toggle show auto threshold windows" << std::endl;
								break;

							case KeyState::Z:
								labelData.isShowsSlidingZoom = labelData.isShowsSlidingZoom==true ? false: true;
								std::cout << " toggle sliding zoom: "  << std::endl;
								break;

							case KeyState::R: //toggle full auto fast forward 
								if(labelData.isDetectMode(DetectMode::AUTO_SEQ)){
									labelData.isFastForward = labelData.isFastForward==true ? false: true;
									std::cout << " toggle full auto fast forward: "  << std::endl;
								}
								break;

							default:
								// nothing
								break;
						} // end of switch
					} else {
						evaluateAdjustState(labelData.currentKeyState);
					}

					std::cout << "KeyEvent  :: current status: " <<  labelData.getStatus() <<", " 
						<< "key pressed: "  			<<  keyPressed  << ", " 
						<< "pos : "  					<<  labelData.p << "  " 
						<< std::endl;

				}
				bool isStoredInWritersmap(){
					auto flag = (datawritersMap.count(labelData.currentFrame)>0);
					return(flag); //if exists returns 1 else 0
				}
				bool isNewAnnotationData(){
					auto flag = false;
					if(!isStoredInWritersmap() and labelData.nmarkersDone>0){
						// if stored and at least one marek annotated 
						flag = (labelData.nmarkersDone>0)? true: false;	
					} else {
						// if stored but new data
						flag = (labelData.isNewMarkerData)? true: false;
					}
					return flag;
				}
				// decides when to store
				void deleteCurrentAnnotation(){
					// erase if backspace
					datawritersMap.erase(labelData.currentFrame);
				}
				void evaluateCurrentAnnotations(){
					// if new annotation data and valid store 
					if(isNewAnnotationData())
						datawritersMap[labelData.currentFrame] = labelData.storeSetDataWriter();
				}
				// decide what data to load
				void evaluatePreviousAnnotations(){
					// labelData.clearLabelData(); // initialize
					labelData.frameCounterFinished = datawritersMap.size();
					if(isStoredInWritersmap()){
						labelData.loadSavedDataWriter(datawritersMap[labelData.currentFrame]);
						labelData.setStatus(Status::REVISITING);
					} else{
						labelData.setStatus(Status::NEW_FRAME);	
					}
				}			
				void evaluateAdjustState(KeyState state){
					switch(state){
						case KeyState::TAB:
							std::rotate(labelData.stridefactor.begin(), labelData.stridefactor.begin()+1,labelData.stridefactor.end());
							labelData.adjustMarker();
							break;
						case KeyState::SPACEBAR:
							labelData.setStatus(Status::STORE_POINT);
							labelData.setMarker();
							break;
						case KeyState::M: // toggle
							labelData.currentDetectMode++; // increment
							std::cout << "toggle mode: " << labelData.getDetectMode() <<  std::endl;
							break;

						case KeyState::A:
							if(isKeyStrokeSeq(KeyState::A, KeyState::A)) // double A
								labelData.setDetectMode(DetectMode::AUTO_SEQ);
							else 
								labelData.setDetectMode(DetectMode::AUTO);

							std::cout << "set mode: " << labelData.getDetectMode() <<  std::endl;
							break;

						case KeyState::H:
							labelData.isShowHints = labelData.isShowHints==true ? false: true;
							std::cout << " toggle show hints" << std::endl;
							break;

						default: 
							labelData.adjustMarker();
							break;
					}
					labelData.updateOnkeyPress();
				}
				inline void getKeyStroke(){
					// prevkeyPressed = (keyPressed<0)? prevkeyPressed: keyPressed;
					prevkeyPressed = keyPressed;
					keyPressed 	= (cv::waitKey(0));
				}
				inline bool isKeyStroke(KeyState key){
					return(getKeyState(keyPressed)==key);
				}
				inline const bool isKeyStrokeSeq(KeyState key1,KeyState key2){ // swap orde!
					prevkeyPressed = keyPressed;
					int nticks =0;
					keyPressed = -1; 
					while(keyPressed<0 and (nticks++)<NMS_TICKS){
						keyPressed 	= (cv::waitKey(KEY_MS));
						// std::cout << "set mode  keyPressed: " << keyPressed<<  std::endl;
					}
					if(getKeyState(keyPressed)==key1 and getKeyState(prevkeyPressed)==key2){
						keyPressed = -1; // reset key!
						return(true);
					}
					return(false);
				}
				inline void getFps(double &fps, std::time_t timeNow, int &tick, long &frameFpsCounter){
					if (timeNow - tick >= 1){
						tick++;
						fps = frameFpsCounter;
						// cout << "Frames per second: " << currentFrame << 
						frameFpsCounter = 0;
					}
				}
				inline std::string makeValidDir(const std::string & str){

					if (mkdir(str.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
					{
						if( errno == EEXIST ) { // alredy exists
							std::cout << "folder: " << str << " --- exists. " << std::endl;
						} else {
							std::cout << "cannot create folder error:" << strerror(errno) << std::endl;
							exit(0);
						}
					}
					return (str.back() == '/' ? str : (str +"/"));
				}
				std::string getTime(std::string format="%d-%m-%Y_%H-%M-%S"){
					auto t = std::time(nullptr);
					auto tm = *std::localtime(&t);

					std::ostringstream oss;
					oss << std::put_time(&tm,format.c_str());
					auto str = oss.str();
					return str;
				}
				inline cv::Mat rotateImg90(cv::Mat image){
					cv::transpose(image, image);
					cv::flip(image, image, +1);
					return image;
				}
				void genExit(){
					std::cout << "exiting program" << std::endl;
					exit(0);
				}

		};

		/* -------------------------------------------end of class  -------------------------------------------*/
		}
		/* -------------------------------------------end of namespace  -------------------------------------------*/



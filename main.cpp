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
 * Description:     main file for Labeltool class 
 *                  dependencies: 
 *                  example compile: make
 *                  then run with:   ./runMain
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 *
 */


#include "header/labeltool.h"


#define VERBOSE_LVL1

    /* ----------------------------------------    typedefs -----------------------------------------------------*/
typedef cv::Point_<float> point2f;



    /* -----------------------------------------   loop parameters  	-----------------------------------------------------*/

int keyPressed = -1;

    /* -----------------------------------------   main 			--------------------------------------------*/

int main( int argc, char** argv){

  /* ----------------------------  parse input   -----------------------------------------*/

	if(argc<2){
		std::cout << "No arguments priovided. Usage cmd [path img file] + " 
		<< "{optional}[output path]" 
		<< "{optional}[nstep to save images ]" 
		<< std::endl;
		return -1;
	}

	std::string img_list_path  = (argc>1)? argv[1] : "input/input_le_jpg_R1_osx.txt";
	std::string img_out_path   = (argc>2)? argv[2] : "data_out/";
  // int nstep = (argc>3)? atoi(argv[3]) : 1;



  /* ----------------------------  initialise LabelTool    -----------------------------------------*/

	invdbscan::LabelTool<point2f> labeltool(14,img_list_path);
 // provide image list or path to video file
	    std::cout << " nmarkers	 		: " <<  labeltool.nmarkers  << std::endl;
	    std::cout << " flname_in	 		: " <<  labeltool.flname_in  << std::endl;
	    std::cout << " flname_out	 		: " <<  labeltool.flname_out  << std::endl;
	    std::cout << " out_path 	 		: " <<  labeltool.out_path  << std::endl;
	    std::cout << " mode				: " <<  (labeltool.camopt==invdbscan::LabelTool<point2f>::Mode::LIST)  << std::endl;
	    // std::cout << " 					"<<  labeltool  << std::endl;
	    // std::cout << " 					"<<  labeltool  << std::endl;


	    labeltool.init();
	    std::cout << " fps				"<<  labeltool.fps  << std::endl;
	    std::cout << " H				"<<  labeltool.rows  << std::endl;
	    std::cout << " W				"<<  labeltool.cols  << std::endl;

				// int H = IMG_H; 
				// int W = IMG_W;
  /* ----------------------------  containers  -----------------------------------------*/

	cv::Mat img = labeltool.labelData.img;

	int framecounter = 0;

	for(;;){

	labeltool.sample(keyPressed);
	
	}


	return 0;
}
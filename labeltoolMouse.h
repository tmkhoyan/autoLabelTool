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
 * Description: 	     Labeltool Ui class: mousde callback 
 *                       dependencies: LabelData, DataWriter 
 * 					example compile: make
 *					then run with:   ./runMain
 *
 * Author: 			Tigran Mkhoyan
 * Email : 			t.mkhoyan@tudelft.nl
 */


// #pragma once 
namespace  invdbscan{
/* -------------------------------------------callback function outside the class  -------------------------------------------*/
	void onMouse(int event, int x, int y, int flags, void* ptrLabelData){

		LabelData<pxyWorkType>*ptrData = (LabelData<pxyWorkType>*)ptrLabelData; // get pointer tor struct labeldata

		ptrData->setPoint(x,y);
		// if( x < 0 || x >= W_ || y < 0 || y >= H_ )
		// 	return;
		// Status currentStatus;

		/* ------------------------------callback function outside the class  ----- */
		switch(event){
			case  cv::EVENT_LBUTTONDOWN :
				if(ptrData->currentStatus != Status::ADJUSTING){
					ptrData->currentStatus = Status::ADJUSTING;
				} else{
					ptrData->currentStatus = Status::STORE_POINT;
					ptrData->setMarker();
				}
				// msg()
				// std::cout << "LB 	clicked - position (" << x << ", " << y << ")" << std::endl;
				break;
			case cv::EVENT_RBUTTONDOWN : 
				// ptrData->currentStatus = (ptrData->currentStatus == Status::ADJUSTING)? ptrData->currentStatus : Status::NEW_FRAME;
				// std::cout << "RB 	clicked - position (" << x << ", " << y << ")" << std::endl;
				// only move to next frame from keypress

				break;
			case cv::EVENT_MBUTTONDOWN : 
				ptrData->currentStatus = (ptrData->currentStatus == Status::ADJUSTING)? ptrData->currentStatus : Status::PAUSE;
				// std::cout << "MB 	clicked - position (" << x << ", " << y << ")" << std::endl;
				break;
			case cv::EVENT_MOUSEMOVE :
				ptrData->currentStatus = (ptrData->currentStatus == Status::ADJUSTING)? 	ptrData->currentStatus : Status::MEASURING;
				ptrData->currentStatus = (!ptrData->isVisited)? 						ptrData->currentStatus : Status::REVISITING;
				
				// std::cout << "M      Moved  - position (" << x << ", " << y << ")" << std::endl;
				break;
			default: 
				if(ptrData->currentStatus==Status::DONE){
					ptrData->currentStatus = Status::IDLE;
				} else if(ptrData->currentStatus==Status::ADJUSTING){
					ptrData->currentStatus = Status::ADJUSTING;
				} else{
					ptrData->currentStatus = Status::IDLE;
					std::cout << "idle" << std::endl;

				}
				break;
		}
		if(ptrData->currentStatus == Status::ADJUSTING){ 							
			ptrData->adjustMarker();
		} else{
			ptrData->drawMouseRect();
		}
		ptrData->detectZoom();
		ptrData->drawZoom();
		ptrData->detectPredictAuto();
		ptrData->drawPredictAuto();
		ptrData->drawUncertainRect();
		ptrData->drawCenteredRect();
		ptrData->drawPreviousPoints();
		ptrData->drawSlidingMouseZoom();
		ptrData->writeInfo();
		ptrData->updateWindow();
		
		std::cout << "MouseEvent:: current status: " << ptrData->getStatus() << ", "
				<< "pos: " << ptrData->p << " "
				<< std::endl;
		return;
	}
	/* -------------------------------------------end of namespace  -------------------------------------------*/
}
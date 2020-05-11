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
 * Description: 	     Labeltool: data type definitions
 *                       dependencies: 
 * 					example compile: make
 *					then run with:   ./runMain
 *
 * Author: 			Tigran Mkhoyan
 * Email : 			t.mkhoyan@tudelft.nl
 *
 */


#pragma once 
namespace  invdbscan{

 #ifndef Pixel
  typedef cv::Point3_<uint8_t> Pixel;     
 #endif

#define COL_KF_PT_PRED CV_RGB( 255, 0, 0 )
#define COL_KF_ID_PRED cv::Scalar( 0, 0, 255 )
#define COL_KF_PT_CORR CV_RGB( 255, 125, 0 )
#define COL_KF_ID_CORR cv::Scalar( 0, 125, 255 )
#define COL_KEYPOINT   cv::Scalar(0,0,255)

// BGR
#define COL_BLUE 	cv::Scalar( 255,   0,   0 )
#define COL_GREEN  	cv::Scalar(   0, 255,   0 )
#define COL_RED  	cv::Scalar(   0,   0, 255 )
#define COL_ORANGE  cv::Scalar( 150,   0, 255 )
#define COL_BLACK  	cv::Scalar(   0,   0,   0 )
#define COL_WHITE  	cv::Scalar( 255, 255, 255 )
// #define COL_YINFO   cv::Scalar(   0, 225, 225 )
#define COL_YINFO   cv::Scalar(   0, 195, 195 )

#define FONTSIZE_YINFO .42
#define TICKNESS_YINFO  1
#define TICKNESS_YINFO2 2
#define FONT_YINFO_ cv::FONT_HERSHEY_COMPLEX
#define FONT_YINFO cv::FONT_ITALIC


#define NANP2_FLOAT	cv::Point2f<> (0.0/0.0,0.0/0.0);
#define NANP3_FLOAT	cv::Point3f<> (0.0/0.0,0.0/0.0,0.0/0.0);

#define IN_FL_NAME  	"input_le_jpg_R1_osx.txt"
#define OUT_FL_NAME  	"labels.yml"
#define OUT_FL_NAME_N  	"labels_nested.yml"
#define OUT_FL_PATH  	"out/"



#define MYFONT 5
#define IMG_COLS 1088
#define IMG_ROWS 600
#define IMG_SCALE 0.5
#define XO_POS 100
#define WRECT 40
#define HRECT 40
#define PIX_STRIDE 1				//striude for adjustkey
#define NSTEP_AUTO_DETECT 10
#define WHRECT_ZOOMFACTOR 10.0			// zoom factor
#define WHRECT_UNCERTAIN_FACTOR 1.2  	// bounding box uncertainty around previously detected 
#define WHRECT_INNER_ZOOM_FACTOR 1.5  	// inner zoom factor inside

#define MORPH_PIX_ERO 15
// #define MORPH_PIX_ERO 50
#define MORPH_PIX_DIL 8

				
				

#define KEY_MS 10
#define NMS_TICKS 15				// 20*KEY_MS total wait time

typedef cv::Point2f pxyWorkType;

	/* -------------------------------------------helper funcs  -------------------------------------------*/
// template<typename T>
// void msg(T msg_val){
// #ifdef VERBOSE_LVL1
// 	std::cout << msg_val << std::endl;
// #endif
// }


enum lvl {
    debug, error, warning, info
};

class Logger
{
    std::ostream* ss;
public:
    Logger( std::ostream* ss_ ) : ss( ss_ ) {}

    template <typename T>
    Logger& operator<<( T const& obj )
    {
        if ( ss != nullptr ) {
            *ss << obj;
        }
        return *this;
    }

};

// #define msg(level) Logger( Logger( level, __FILE__, __LINE__ ) )
// GetLoogerForLevel

	/* -------------------------------------------class enums statemachine  -------------------------------------------*/

	enum class Status { IDLE, STORE_POINT, SKIP_POINT, DELETE_POINT, CLEAR_POINTS,  NEW_FRAME, PREV_FRAME, PAUSE, MEASURING,DONE, EXIT, START, ADJUSTING, REVISITING, AUTOSET_SEQ  };

	enum class KeyState { P, Q, R, S, M, A, W, D, C, Z, F, H, T,   UP, DOWN, LEFT, RIGHT, SPACEBAR, ARROWR, ARROWL, TAB, NONE, ENTER, BACKSPACE };
	inline const KeyState getKeyState(int pressed)
	{
		switch (pressed)
		{
			case 113 	: return KeyState::Q;
			case 81 	: return KeyState::Q;
			case 112 	: return KeyState::P;
			case 80 	: return KeyState::P;
			case 114 	: return KeyState::R;
			case 82 	: return KeyState::R;
			case 115 	: return KeyState::S;
			case 83 	: return KeyState::S;
			case 109 	: return KeyState::M;
			case 77 	: return KeyState::M;
			case 97 	: return KeyState::A;
			case 65 	: return KeyState::A;
			case 100 	: return KeyState::D;
			case 68 	: return KeyState::D;
			case 87 	: return KeyState::W;
			case 119 	: return KeyState::W;
			case 67 	: return KeyState::C;
			case 99 	: return KeyState::C;
			case 90 	: return KeyState::Z;
			case 122 	: return KeyState::Z;
			case 70 	: return KeyState::F;
			case 102 	: return KeyState::F;
			case 72 	: return KeyState::H;
			case 104 	: return KeyState::H;
			case 84 	: return KeyState::T;
			case 116 	: return KeyState::T;
			case 0 	: return KeyState::UP;
			case 1 	: return KeyState::DOWN;
			case 3 	: return KeyState::RIGHT;
			case 2 	: return KeyState::LEFT;
			case 46 	: return KeyState::ARROWR;
			case 44 	: return KeyState::ARROWL;
			// linux use numpad arrows
			case 56 	: return KeyState::UP;
			case 50 	: return KeyState::DOWN;
			case 54 	: return KeyState::RIGHT;
			case 52	: return KeyState::LEFT;
			case 32	: return KeyState::SPACEBAR;
			case 9  	: return KeyState::TAB;
			case 13  	: return KeyState::ENTER;
			case 127  : return KeyState::BACKSPACE;
			default   : return KeyState::NONE;
		}
	}
	/* ------------------------------------------- detectmode  -------------------------------------*/ 
	enum class DetectMode { AUTO, AUTO_SEQ, SEMI_AUTO, MANUAL, NUM_MODES };
	DetectMode operator++(DetectMode& mode){
	    mode = static_cast<DetectMode>((static_cast<int>(mode) + 1) % (static_cast<int>(DetectMode::NUM_MODES)));
	    return mode;
	}
	DetectMode operator++(DetectMode& mode, int){ // postfix operator 
    	DetectMode result = mode;
    ++mode;
    return result;
}

	template<typename T_>
		static std::ostream& operator<<(std::ostream& out, const std::vector<T_>& v)
		{
			for(const auto &p: v){out << p << "," ;}; out     << "];" << "\n";
			return out;
		}

	/* -------------------------------------------end of enum defs  -------------------------------------------*/
}

	/* -------------------------------------------end of namespace  -------------------------------------------*/

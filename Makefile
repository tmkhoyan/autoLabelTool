# /* BSD 3-Clause License
#  *  
#  *  Copyright (c) 2020, tmkhoyan (Tigran Mkhoyan)
#  *  All rights reserved.
#  *  
#  *  Redistribution and use in source and binary forms, with or without
#  *  modification, are permitted provided that the following conditions are met:
#  *  
#  *  1. Redistributions of source code must retain the above copyright notice, this
#  *     list of conditions and the following disclaimer.
#  *  
#  *  2. Redistributions in binary form must reproduce the above copyright notice,
#  *     this list of conditions and the following disclaimer in the documentation
#  *     and/or other materials provided with the distribution.
#  *  
#  *  3. Neither the name of the copyright holder nor the names of its
#  *     contributors may be used to endorse or promote products derived from
#  *     this software without specific prior written permission.
#  *  
#  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  */

# /*
#  * Description: 	     Make file Labeltool project 
#  * 			     example compile: make
#  *					then run with:   ./runMain
#  *
#  * Author: 			Tigran Mkhoyan
#  * Email : 			t.mkhoyan@tudelft.nl
#  */

.PHONY: all clean

# The program to build
NAME       := main

# detect OS
OS := $(shell uname)

# if OSX
ifeq ($(OS),Darwin) 
SYSTEM_LIBRARY_DIR := /Library

CXX+=-std=c++11
LD         := $(CXX)
CPPFLAGS   := -I /usr/local/include/opencv4
CXXFLAGS   := -o3 -Wno-switch #e.g., CXXFLAGS=-g -O0 for debugging
LDFLAGS    := -I /usr/local/include/opencv4
LDLIBS     := -L /Library/Frameworks -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio \
									   -lopencv_video -lopencv_imgproc -lopencv_tracking  -lopencv_features2d \
									   -lopencv_calib3d -pthread  -lopencv_highgui
# linux
else
# Build tools and flags
CXX+=-std=c++11
LD         := $(CXX)
CPPFLAGS   := -DUSE_GIGE -I /usr/local/include/opencv4
CXXFLAGS   := -o3 -Wno-switch#e.g., CXXFLAGS=-g -O0 for debugging
LDFLAGS    := -I /usr/local/include/opencv4
LDLIBS     := -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lopencv_video  -lopencv_imgproc -lopencv_tracking -lopencv_features2d -lopencv_calib3d -pthread 
# LFLIBS = -lopencv_core -lopencv_calib3d -lopencv_imgproc -lopencv_tracking -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_highgui -lopencv_flann

endif
# Rules for building

# tbb
CPPFLAGS+=-I /usr/local/Cellar/tbb/2019_U3_1/include 
LDFLAGS+=-I /usr/local/Cellar/tbb/2019_U3_1/include 
LDLIBS+=-L /usr/local/Cellar/tbb/2019_U3_1/lib -L/usr/local/lib/ 


all: $(NAME)

$(NAME): $(NAME).o
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)
# 	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS) jetsonGPIO.c

$(NAME).o: $(NAME).cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	$(RM) $(NAME).o $(NAME)

debug: 
	@echo  "$(OS) 		" ;
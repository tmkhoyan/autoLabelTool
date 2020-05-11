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
 * Description:     Labeltool Write class: data type definitions
 *                  dependencies: 
 *                  example compile: make
 *                  then run with:   ./runMain
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 *
 */


// #pragma once 
namespace  invdbscan{


#define P_STR   "p"
#define IDX_STR "idx"
#define I_STR   "i"         // frame id
#define ND_STR  "ndone"     // markers done
#define N_STR   "n"         // markers

#define NROW_ALLOOC 100

typedef cv::Point2f pWorkType;

/* ----------------------------------helper ---------------- */
template <typename E>
inline std::vector<cv::Point_<E>> mat2Dpoint(cv::Mat m){
  // cv::MatIterator_<E> it;
  std::vector <cv::Point_<E>> v; v.reserve(m.rows);
  for(auto it = m.begin<E>();it != m.end<E>();++it){
    auto x = (*it); it++; // assign x and advance
    auto y = (*it); //asign y
    v.emplace_back(cv::Point_<E>(x,y));
  }
  return v;
}
template <typename E>
inline cv::Mat point2Dmat(std::vector< cv::Point_<E> > p){
  return cv::Mat(p.size(),2,cv::DataType<E>::type,p.data());
}
/* ----------------------------------helper ---------------- */

template<typename T_>
class LabelWriter
{

public: 
        typedef typename T_::value_type E;
        // typedef cv::Point_ <E> T2d;          // needed to define 2d point
        /* ----------------------------------core contents loop ---------------- */
        std::string id = "";;       // unique
        std::vector<T_> Pxy;   // vector of points mat needed in order toprint
        std::vector<unsigned int> indeces; // note different types! due to file storages
        int nmarkers = 0;               // total markers
        int nmarkersDone = 0;           // total markers
        int isVisited = false;              // whennon default constructor
        // cv::Mat mPxy;
        /* ----------------------------------core contents info ---------------- */

        /* ----------------------------------constructor and destructor  ---------------- */
        LabelWriter(){}
        LabelWriter(int frameId): id(std::to_string(frameId))
        {}
        LabelWriter(std::vector<T_> Pxy_ ,std::vector<unsigned int> indeces_, int frameId, int ndone_): 
        Pxy(Pxy_),
        // mPxy(cv::Mat(Pxy_.size(),2,cv::DataType<E>::type,Pxy_.data()).clone()),
        // mIndeces(cv::Mat(indeces_.size(),1,CV_32SC1,indeces_.data()).clone()), cast to matrix if doesnt work
        indeces(indeces_.begin(),indeces_.end()), // range constructor to cast unsigned int to int
        id(std::to_string(frameId)),
        nmarkers(Pxy_.size()),
        nmarkersDone(ndone_),
        isVisited(true) // normally false but sets true when new data
        {
            // mPxy = cv::Mat(Pxy_.size(),2,cv::DataType<E>::type,Pxy_.data()).clone();
            // mIndeces = cv::Mat(indeces_.size(),1,CV_32SC1,indeces_.data()).clone();

        }
        ~LabelWriter(){std::cout << "LabelWiter object deleted" << std::endl;};

        /* ----------------------------------filenode read write << operator writes nested nodes !---------------- */

    void write(cv::FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{"   << P_STR         << point2Dmat(Pxy)          
                    // << IDX_STR   << indeces     
                    << IDX_STR       << (std::vector<int>(indeces.begin(),indeces.end()))     
                    << I_STR         << id
                    << ND_STR        << nmarkersDone    
                    << N_STR         << nmarkers    
                    << "}"; 
    }
    void read(const cv::FileNode& node)                          //Read serialization for this class
    {
        std::vector< int> indeces_; indeces_.reserve(NROW_ALLOOC); // for 
        cv::Mat mPxy(NROW_ALLOOC,2,cv::DataType<E>::type); // allocate
            // map structures need to be read in with another filenode item
            auto node_P_STR   = node[P_STR    ]; // type is cv::FileNode read into the variable
            auto node_IDX_STR = node[IDX_STR  ];
            node_P_STR     >> mPxy;
            node_IDX_STR   >> indeces_;
            // 
        id          =(std::string)      node[I_STR      ];
        nmarkersDone    =(int)          node[ND_STR ];  
        nmarkers        =(int)          node[N_STR  ];
        
                // conversions
        indeces = std::vector<unsigned int>(indeces_.begin(),indeces_.end());
        Pxy     = mat2Dpoint<E>(mPxy);
    }
        /* ------------------------------------------- custom write function  -------------------------- */
    void writeThis(cv::FileStorage &fs) const                   //need to be declared const in order to be used efficeiently in auto range based loop    {
    {    
        fs << P_STR     +id << point2Dmat(Pxy);            //matrix 2,N
        // fs << IDX_STR   +id << indeces;         //vector 1xN int
        fs << IDX_STR   +id << std::vector<int>(indeces.begin(),indeces.end());         //vector 1xN int
        fs << I_STR     +id << id;
        fs << ND_STR    +id << nmarkersDone;    //int
        fs << N_STR     +id << nmarkers;        //int
    }
    void readThis(cv::FileStorage &fs, int frameId)                      
    {
        id = std::to_string(frameId);
        std::vector< int> indeces_; indeces_.reserve(NROW_ALLOOC);
        cv::Mat mPxy(NROW_ALLOOC,2,cv::DataType<E>::type); // allocate

        fs[P_STR        +id] >> mPxy         ;  
        fs[IDX_STR      +id] >> indeces_     ;  
        fs[I_STR        +id] >> id       ; 
        fs[ND_STR       +id] >> nmarkersDone ; 
        fs[N_STR        +id] >> nmarkers     ; 
        // conversions
        indeces = std::vector<unsigned int>(indeces_.begin(),indeces_.end());
        Pxy     = mat2Dpoint<E>(mPxy);
    }
        /* ------------------------------------------- functions  -------------------------- */

    /* -------------------------------------------end of class  -------------------------------------------*/
};

    /* -------------------------------------------read write filenode outside class  -------------------------------------------*/
template<typename T_>
static void write(cv::FileStorage& fs, const std::string&, const LabelWriter<T_>& x){
    x.write(fs);
}
template<typename T_>
static void read(const cv::FileNode& node, LabelWriter<T_>& x, const LabelWriter<T_>& default_value = LabelWriter<T_>()){
    if(node.empty()){
        x = default_value;
        }
    else{
        x.read(node);
   }
}
    /* -------------------------------------------read stdout -------------------------------------------*/
template<typename T_>
static std::ostream& operator<<(std::ostream& out, const LabelWriter<T_>& s)
{
    out <<         "id ="   << s.id  << "{"                                 << "\n";   

    out <<  "\t" << P_STR   << " = "    << point2Dmat(s.Pxy)                           << "\n";
    out <<  "\t" << IDX_STR     << " = [";
                        for(auto i: s.indeces){out << i << "," ;}; out << "];" << "\n";
    out <<  "\t" << I_STR       << " = "    << s.id                                 << "\n";
    out <<  "\t" << ND_STR  << " = "  << s.nmarkersDone                         << "\n";    
    out <<  "\t" << N_STR   << " = "    << s.nmarkers                           << "\n";

    out                                                         << "}"  << "\n";

    return out;
}
    /* -------------------------------------------end of namespace  -------------------------------------------*/

}
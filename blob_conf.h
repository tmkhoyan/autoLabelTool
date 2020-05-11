   // thresholds
  #define BLOB_minThr 10;
  #define BLOB_maxThr 100;
  // #define BLOB_maxThr 300;

  // #define //Area.
  #define BLOB_filtA true;
  #define BLOB_minA 300;
  #define BLOB_maxA 10000;


  // #define // Circularity
  #define BLOB_filtCirc true;
  #define BLOB_minCirc 0.1;

  // #define //Convexity
  #define BLOB_filConv true;
  #define BLOB_minConv 0.87;

  // #define //Inertia
  #define BLOB_filtIn true;
  #define BLOB_mintInR 0.1;


// params.minThreshold = 10;
// params.maxThreshold = 100;

// // Filter by Area.
// params.filterByArea = true;
// params.minArea = 1000;
// params.maxArea = 10000;

// // Filter by Circularity
// params.filterByCircularity = true;
// params.minCircularity = 0.1;

// // Filter by Convexity
// params.filterByConvexity = true;
// params.minConvexity = 0.87;

// // Filter by Inertia
// params.filterByInertia = true;
// params.minInertiaRatio = 0.1;

  //drawing
  cv::Scalar BLOB_CENT_COL(0.0,255.0,0.0);
  size_t  BLOB_CENT_SIZE = 2;


 //function to setup parameters
    cv::SimpleBlobDetector::Params setupBlobDetector(){
    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold         = BLOB_minThr;
    params.maxThreshold         = BLOB_maxThr;

    // Filter by Area.
    params.filterByArea         = BLOB_filtA;
    params.minArea              = BLOB_minA;
    params.maxArea              = BLOB_maxA;

    // Filter by Circularity
    params.filterByCircularity  = BLOB_filtCirc;
    params.minCircularity       = BLOB_minCirc;

    // Filter by Convexity
    params.filterByConvexity    = BLOB_filConv;
    params.minConvexity         = BLOB_minConv;

    // Filter by Inertia
    params.filterByInertia      = BLOB_filtIn;
    params.minInertiaRatio      = BLOB_mintInR;
    return params;

    }

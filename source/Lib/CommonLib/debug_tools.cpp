//
// Created by Office on 10.02.21.
//

#include "debug_tools.h"
#ifdef OpenCV_FOUND
#include <opencv2/opencv.hpp>
#endif

namespace debug_tools {
  void showBuf(const CPelBuf &buf, const char* title) {
#ifdef OpenCV_FOUND
    cv::Mat img;
    cv::Mat(buf.height, buf.width, CV_16UC1, (Pel*)buf.buf, 2 * buf.stride).copyTo(img);
    img.convertTo(img, CV_16UC1, 1.0*(1 << 6), 0.0);
    cv::namedWindow(title, cv::WINDOW_KEEPRATIO);
    cv::imshow(title, img);
    cv::waitKey(10);
#endif
  }
}

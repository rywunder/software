#include "vision/aard/AprilTagWrapper.hpp"  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char ** argv){
  cout << "AprilTagTest Running\n";


  //Load an image
  //Make sure we've got a valid image name
  // if( argc != 2){
  //    cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
  //    return 0;
  //   }
  //Create a Mat object that will store the data of a loaded image
  Mat image;
  //Load the image...UNCHANGED -> as is, COLOR -> BGR format
  image = imread("/home/ryan/software/bin/pics/data/rgb2.png", CV_LOAD_IMAGE_UNCHANGED);
  ///////////////////////////////////////////////
  // //Create a display window
  // nameWindow("Display Window", WINDOW_AUTOSIZE);
  // //Show image
  // imshow("Display Window", image);
  // //Wait key
  // waitKey(0);
  //////////////////////////////////////////////

  //Detect the AprilTag
  AprilTagWrapper testTag("tag36h11");
  vector<detInfo> d = testTag.getTagDetection(image);
  auto& stuff = testTag.tag_pose_estimation();
  //AprilTagWrapper testTag = AprilTagWrapper("tag36h11");
  //testTag.tagDetection(image);
  //testTag.detSummary();
  //Print out the detection
  for(size_t i = 0; i<d.size(); ++i){
    cout << d[i].id << endl;
    cout<<"Center x: "<<d[i].c[0]<<endl;
    cout<<"Center y: "<<d[i].c[1]<<endl;
  }

  for(size_t i =0; i<d.size();++i){
    cout<<endl;
    cout<<"Image : " << i+1 <<endl;
    cout<< "Error: " <<  stuff[i].second<< endl;
    cout<<"Translational: "<< endl;
    testTag.printMatd(stuff[i].first.t);
  }
  return 0; 
  
  }

#include "vision/aard/AprilTagWrapper.hpp"
#include <vector>
#include <iostream>
#include <string>
/**
* instantiate a detector when a tag family string is passed in. 
* the AprilTagWrapper constructor ceates a tag family by passing in
* the string and then creates a tagfamily. 
*/
AprilTagWrapper::AprilTagWrapper(std::string tfName){
  tagFamName = tfName;  
  td = apriltag_detector_create();
  tagFamCreate(tagFamName);
  apriltag_detector_add_family(td, tf);
}

/**  
 *  Required: quadDecimate, quadSigma, refineEdge and decodeSharp.
 *  Modifies: value of td is affected.  
 *  Effects: Adds the attributes provided to the image detection object *td
 *  essentially, td is provided by the external library and 
 */
void AprilTagWrapper::setTagDetectorParam(double quadDecimate, double quadSigma, double refineEdge, double decodeSharp){
  td->quad_decimate = quadDecimate;
  td->quad_sigma = quadSigma;
  td->refine_edges = refineEdge;
  td->decode_sharpening = decodeSharp;
}

// prints the data found in the matd
void AprilTagWrapper::printMatd(const matd_t *m)
{
    std::cout << "[\n";
    for(unsigned i = 0; i < m->nrows; ++i) {
        std::cout << "\t";
        for(unsigned j = 0; j < m->ncols; ++j) {
            std::cout << m->data[i*m->ncols + j] << " ";
        }
        std::cout << "\n";
    }
    std::cout << "]\n";
}


std::vector<std::pair<apriltag_pose_t,double>>& AprilTagWrapper::tag_pose_estimation(){ 

    //contains a vector of all the distances//
    //should i create a vector of vector with the id??//
    //Ryan should i implement vector of vector with id and pose estimate?//
    //loop goes through all the values in the detections 
    //loop also converts the given eigen vectors to matd format and changes pose.t and pose.R
    for(int i = 0; i<zarray_size(detections);++i){
        
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        apriltag_detection_info_t info;
        //det tagsize fx fy cx cy 
        info = {det,0.18,615.073,615.166,313.623,234.231};
        // typedef struct {
        // apriltag_detection_t* det;
        // double tagsize; // In meters.
        // double fx; // In pixels.
        // double fy; // In pixels.
        // double cx; // In pixels.
        // double cy; // In pixels.
        // } apriltag_detection_info_t;
        
        std::pair<apriltag_pose_t,double> poseErr;
	    poseErr.second = estimate_tag_pose(&info,&poseErr.first);
        answer.push_back(poseErr);
        
        //if you want t and R.
        //answer[i].t = convertTranslational(pose.t);
        //answer[i].R = convertRotational(pose.R);
    }
    return answer;
}

/**  
 *  Required: a valid Mat image object(open cv object)
 *  Modifies: vector<detInfo> 
 *  Effects: returns the number of tag that the image contains. if the image 
 *  contains two apriltags, a vector of size two will be retured. 
 */
std::vector<detInfo> AprilTagWrapper::getTagDetection(cv::Mat image){
  cvtColor(image, image, cv::COLOR_BGR2GRAY);
  image_u8_t img_header = {
    image.cols, //width
    image.rows, //height
    image.cols, //stride
    image.data //buf
  };

  if(detections){apriltag_detections_destroy(detections);}

  detections = apriltag_detector_detect(td, &img_header);
  std::vector<detInfo> dV;
  std::cout<< zarray_size(detections) <<" tags detected"<<std::endl;
  for(int i = 0; i < zarray_size(detections); ++i){ 
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        detInfo d;
        d.id = det->id;
        for(int j = 0; j<2; ++j){
        d.c[j] = det->c[j];
        }
        for(int j = 0; j<4;++j){
            for(int k = 0; k<2;++k){
            d.p[j][k] = det->p[j][k];
        }
    }
    detVect.push_back(d);
    dV.push_back(d);

  }
  
  return dV;  
}

//destructor
AprilTagWrapper::~AprilTagWrapper(){
  apriltag_detector_destroy(td);
  tagFamDestroy(tagFamName);
  apriltag_detections_destroy(detections);
  for(size_t i = 0; i<answer.size();++i){
      matd_destroy(answer[i].first.t);
      matd_destroy(answer[i].first.R);
  }
 }

 // ----------------------------------------------------------------------------
 //helper to create specific tag family
 void AprilTagWrapper::tagFamCreate(std::string tagFamName){
   if (tagFamName == "tag36h11") {
       tf = tag36h11_create();
   } else if (tagFamName == "tag25h9") {
       tf = tag25h9_create();
   } else if (tagFamName == "tag16h5") {
       tf = tag16h5_create();
   } else if (tagFamName == "tagCircle21h7") {
       tf = tagCircle21h7_create();
   } else if (tagFamName == "tagCircle49h12") {
       tf = tagCircle49h12_create();
   } else if (tagFamName == "tagStandard41h12") {
       tf = tagStandard41h12_create();
   } else if (tagFamName == "tagStandard52h13") {
       tf = tagStandard52h13_create();
   } 
   /**else if(tagFamName =="tag36h10"){
       tf = tag36h10_create();
   }**/

 }

 //helper to destroy specific tag family
void AprilTagWrapper::tagFamDestroy(std::string tagFamName){
   if (tagFamName =="tag36h11") {
       tag36h11_destroy(tf);
   } else if (tagFamName == "tag25h9") {
       tag25h9_destroy(tf);
   } else if (tagFamName == "tag16h5") {
       tag16h5_destroy(tf);
   } else if (tagFamName == "tagCircle21h7") {
       tagCircle21h7_destroy(tf);
   } else if (tagFamName == "tagCircle49h12") {
       tagCircle49h12_destroy(tf);
   } else if (tagFamName == "tagStandard41h12") {
       tagStandard41h12_destroy(tf);
   } else if (tagFamName == "tagStandard52h13") {
       tagStandard52h13_destroy(tf);
   } /**else if(tagFamName =="tag36h10"){
       tag36h10_destroy(tf);
   }**/
 }
 // -----------------------------------------------------------------------------

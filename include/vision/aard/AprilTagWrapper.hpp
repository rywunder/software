#ifndef APRIL_TAG_WRAPPER_HPP
#define APRIL_TAG_WRAPPER_HPP


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#include <vector>
#include <Eigen/Core>

extern "C" {
  #include "apriltag.h"
  #include "common/matd.h"
  #include "apriltag_pose.h"
  #include "tag36h11.h"
  #include "tag25h9.h"
  #include "tag16h5.h"
  #include "tagCircle21h7.h"
  #include "tagCircle49h12.h"
  #include "tagCustom48h12.h"
  #include "tagStandard41h12.h"
  #include "tagStandard52h13.h"
}


  /*!
  Struct detInfo is the struct class that holds information about each tag on the drone. 
  */
  struct detInfo{
    
    int id; // The decoded ID of the tag
    double c[2]; // The center of the detection in image pixel coordinates.
    double p[4][2]; // The corners of the tag in image pixel coordinates.
    //These always wrap counter-clock wise around the tag.

  };
  struct poseEstimates{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    double err=0;
  };
  
/*! AprilTagWrapper class has the default constructors, 

    public functions: 
    settagDetectorParam: method which is set by the user to make changes to the image. 
    tagDetection: is the method that uses the library and detects the image, storing the data
    in a vector. 
    
    private methods: 
    tagFamCreate(): method which creates a tag family object by checking tag input from the user.
    tagFamDestroy(): Method which deletes the object. 

    detVect: variable containing all the detections. 
*/
class AprilTagWrapper{
  public:

   /**  
   *  Required: tag family name.
   *  Modifies:  
   *  Effects: Adds the attributes provided to the image detection object 
   * *td
   *  the AprilTagWrapper constructor ceates a tag family by passing in
   *  the string.
   */
    AprilTagWrapper(std::string tfName);

    std::vector<std::pair<apriltag_pose_t,double>>& tag_pose_estimation();
   /**  
   *  Required: quadDecimate, quadSigma, refineEdge and decodeSharp.
   *  Modifies: value of td is affected.  
   *  Effects: calls the constructor and 
   *  adds the attributes provided to the image detection object *td
   *  essentially, td is provided by the external library and 
   */
    void setTagDetectorParam(double quadDecimate, double quadSigma, double refineEdge, double decodeSharp);
    
   /**  
   *  Required: a valid Mat image object(open cv object)
   *  Modifies: vector<detInfo> 
   *  Effects: returns the number of tag that the image contains. if the image 
   *  contains two apriltags, a vector of size two will be retured. 
   */
    std::vector<detInfo> getTagDetection(cv::Mat image); 
    
    //destructor
    ~AprilTagWrapper();

    // Helper for printing the matrix
    void printMatd(const matd_t *m);

  private:

    //a apriltag object provided by the external APRILTAG library.
    apriltag_detector_t *td;

    /**a apriltag family  object prvided by the external APRILTAG library.
    * an april tag object/ every apriltag belongs to a april 
    * tag family object
    */ 
    apriltag_family_t *tf;

    //the user defined tagFamName. 
    std::string tagFamName;

    //a helper funtion to create a tag family object.
    void tagFamCreate(std::string tagFamName);

    //a helper function to destroy a tagFam to keep memory clean. 
    void tagFamDestroy(std::string tagFamName);

    //vector of detections
    std::vector<detInfo> detVect; 

    //a vector of detections
    zarray_t *detections = nullptr;

    //vector of pose_estimates
    std::vector< std::pair<apriltag_pose_t,double> > answer;


    // template<_MatrixType T>
    // matd_t* convertMatd_ttoEigen(T x){
    //   return matd_create_data(x.rows(),x.cols(),x.data());
    // }

    Eigen::Matrix3d convertRotational(matd_t *R){
      Eigen::Matrix3d mat(R->nrows,R->ncols);
      int k = 0;
      for(int i = 0; i<3;++i){
        for(int j = 0;j<3;++j){
          mat(i,j) = R->data[k];
          ++k;
        }
      }
      delete R;
      return mat;
    }


    Eigen::Vector3d convertTranslational(matd_t *T){
      Eigen::Vector3d mat(T->nrows,T->ncols);
      int k = 0;
      for(int i = 0; i<3;++i){
          mat(i,0) = T->data[k];
          ++k;
      }
      delete T;
      return mat;
    }

};

#endif

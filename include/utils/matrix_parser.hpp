#ifndef __FOLDING_UTILS__
#define __FOLDING_UTILS__

#include <Eigen/Dense>
#include <ros/ros.h>
#include <stdexcept>
#include <math.h>

namespace folding_utils{

  class MatrixParser
  {
  public:
    MatrixParser();
    ~MatrixParser();

    /**
      Initialize a nxn matrix with values obtained from the ros parameter
      server.

      @param M The matrix to be initialized
      @param param_name The parameter server location
      @param n The ros nodehandle used to query the parameter server

      @throw logic_error in case vals does not have square dimensions.
      @return True for success, False if parameter is not available.
    **/
    bool parseMatrixData(Eigen::MatrixXd &M, const std::string param_name, const ros::NodeHandle &n);

  private:
    /**
      Fill in a nxn matrix with the given values.

      @param M The matrix to be filled in. Will be set to the size nxn.
      @param vals A vector with the values to fill in
      @throw logic_error in case vals does not have square dimensions.
    **/
    void initializeEigenMatrix(Eigen::MatrixXd &M, const std::vector<double> &vals);
  };
}

#endif

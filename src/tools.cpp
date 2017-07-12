#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if(estimations.size()==0){
	  cout<<"Error- empty estimations"<<endl;
	  //return -1;
	}
	if(estimations.size()!=ground_truth.size()){
	  cout<<"Error- estimations and ground truth have different size."<<endl;
	  //return -1;
	}
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd residual = estimations[i] -ground_truth[i];
		VectorXd residual_squre = residual.array()*residual.array();
		rmse = rmse + residual_squre;
	}

	//calculate the mean
	// ... your code here
	rmse = rmse/estimations.size();

	//calculate the squared root
	// ... your code here
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}
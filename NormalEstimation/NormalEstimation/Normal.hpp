//#define EIGEN_USE_MKL_ALL
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/QR>
#include <Eigen/SVD>
#include "spectral.h"
#include <cmath>
#include<QtDebug>
#include <pcl/common/time.h>

using namespace Eigen;
using namespace std;

inline double log2( double n ) 
{  
	// log(n)/log(2) is log2.  
	return log( n ) / log( (double)2 );  
}

template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
	return ( (x - x).array() == (x - x).array()).all();
}

template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
	return ((x.array() == x.array())).all();
}

// inline int round(double x) 
// { 
	// return (floor(x + 0.5));
// }

class NormalUtil {
public:
	static pcl::PointCloud<pcl::PointXYZ>::Ptr readPointCloud(std::string);
	static void readPointCloud(std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr);
	static pcl::PointCloud<pcl::Normal>::Ptr estimatePCANormal(pcl::PointCloud<pcl::PointXYZ>::Ptr,int);
	static Eigen::MatrixXd optimization(Eigen::MatrixXd& X,double lambda);
	static Eigen::MatrixXd orth(Eigen::MatrixXd&);
	static MatrixXd solve_l21(MatrixXd &, double);
	static int solve_svs(MatrixXd &, double, MatrixXd&, VectorXd&, MatrixXd&);
	static void solve_soft(MatrixXd&, double);
	static double eps();
    static double norm2(MatrixXd&);
	static int predictClusterNumber(MatrixXd&, double tau);
	static vcluster spectral_clustering(MatrixXd&);
    static MatrixXd compute_laplacian(MatrixXd&);
    static void write_pointset(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals, std::string filename);

    // FOR DEBUG USE
    static void save2file(MatrixXd&, std::string);
    static void save2file(VectorXd&, std::string);
    static void printVector(VectorXd&);
    static void write_local(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals, int idx, vcluster& labels, std::map<int,int>& mapping);
};


class Parameter {
public:
    int k;
    double lambda;
    double tau; // smaller --> smoother 
	Parameter():k(0),lambda(1.0),tau(8e-1){}
    static Parameter* instance(){
        if (instance_) {
            return instance_;
        }else{
            instance_=new Parameter();
            return instance_;
        }
    }
    ~Parameter(){
        if (instance_) { 
            delete instance_;
        }
    }
private: 
    static Parameter* instance_;
};



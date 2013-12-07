//#define USE_OPENMP_FOR_NORMEST //you don't want to use OPENMP comment this line

#include "PCL_normEst.h"
#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>


using namespace std;

int main(int argc,char** argv){

	pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>());
	// filling the vector with the point cloud

	std::string file=argv[1];
	int k=200;
	if (argc>2)
	{
		k=boost::lexical_cast<int>(argv[2]);
	}
	cout<<file<<endl;
	ifstream infile(file.c_str());

	double x,y,z;

	while(infile>>x>>y>>z){
		//cout<<x;
		//cout<<y;
		//cout<<z;
		//cout<<endl;
		pcl::PointXYZ p(x,y,z);
		points->push_back(p);
	}

	cout<<points->size()<<endl;

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	PCL_Normal_Estimator NE(points,normals);

	//change values of the parameters
	//NE.number_of_planes() = 700;
	//NE.normal_selection_mode() = PCL_Normal_Estimator::BEST;

	//estimate the normals
	//NE.estimate(); // with default parameters
	NE.estimate(PCL_Normal_Estimator::CUBES, PCL_Normal_Estimator::KNN, k); //changing method

	// output
	ofstream outf("sgp_out.xyzn");
	//ofstream outf("sgp_out.xyz");

	for (int i=0;i<points->size();i++)
	{
		// check for #IND
		if (normals->points[i].normal_x!=normals->points[i].normal_x)
		{
			outf<<points->points[i].x<<" "<<points->points[i].y<<" "<<points->points[i].z<<" "<<0<<" "<<0<<" "<<0<<endl;
		}else{
			outf<<points->points[i].x<<" "<<points->points[i].y<<" "<<points->points[i].z<<" "<<normals->points[i].normal_x<<" "<<normals->points[i].normal_y<<" "<<normals->points[i].normal_z<<endl;
		}
		
		//outf<<points->points[i].x<<" "<<points->points[i].y<<" "<<points->points[i].z<<endl;
	}
	

	outf.close();

	return 0;
}

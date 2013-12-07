#include "normalestimation.h"
#include <QFileDialog>
#include <QMessageBox>
#include "Normal.hpp"
#include <omp.h>
#include <pcl/common/time.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <boost/lexical_cast.hpp>

#include <utility> // defines std::pair
#include <list>
#include <fstream>

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

// Point with normal vector stored in a std::pair.
typedef std::pair<Point, Vector> PointVectorPair;

void orientation_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals,int nb_neighbors){
	// Reads a .xyz point set file in points[].
	std::list<PointVectorPair> points;
	for (int i=0;i<(int)cloud->size();i++)
	{
		pcl::PointXYZ p=cloud->points[i];
		pcl::Normal n=normals->points[i];
		PointVectorPair pair(make_pair(Point(p.x,p.y,p.z),Vector(n.normal_x,n.normal_y,n.normal_z)));
		points.push_back(pair);
	}

	// Orients normals.
	// Note: mst_orient_normals() requires an iterator over points
	// as well as property maps to access each point's position and normal.
	std::list<PointVectorPair>::iterator unoriented_points_begin =
		CGAL::mst_orient_normals(points.begin(), points.end(),
		CGAL::First_of_pair_property_map<PointVectorPair>(),
		CGAL::Second_of_pair_property_map<PointVectorPair>(),
		nb_neighbors);

	int i=0;
	for (std::list<PointVectorPair>::iterator it=points.begin();it!=points.end();it++)
	{
		PointVectorPair pp=*it;
		Point point=pp.first;
		Vector normal=pp.second;
		pcl::Normal& n=normals->points[i];
		n.normal_x=normal.x();
		n.normal_y=normal.y();
		n.normal_z=normal.z();
		++i;
	}
}


NormalEstimation::NormalEstimation(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
}

NormalEstimation::~NormalEstimation()
{

}

void NormalEstimation::open(){
	QString path = QFileDialog::getOpenFileName(this, tr("Open Model"), ".", tr("3D Model Files(*.xyz *.xyzn)")); 
	if(path.length() == 0) { 
		QMessageBox::information(NULL, tr("Path"), tr("You didn't select any files.")); 
		return;
	} else { 
		//QMessageBox::information(NULL, tr("Path"), tr("You selected ") + path); 
	}
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    int neighbor_nb=500;
    Parameter::instance()->k=50;
	
	QByteArray ba = path.toLatin1();
	const char *c_str2 = ba.data();
	std::string filename=c_str2;

    if (path.endsWith("xyz",Qt::CaseInsensitive)) {
        cloud=NormalUtil::readPointCloud(filename);
        normals=NormalUtil::estimatePCANormal(cloud,neighbor_nb);
        qDebug()<<"pca normal compute complete";
        NormalUtil::write_pointset(cloud,normals,"pca.xyzn");
        orientation_normals(cloud, normals, 100);
        qDebug()<<"normal orientation complete";
    }else if(path.endsWith("xyzn",Qt::CaseInsensitive)){
        NormalUtil::readPointCloud(filename,cloud,normals);
        pcl::PointCloud<pcl::Normal>::Ptr newnormals(new pcl::PointCloud<pcl::Normal>);
        newnormals=NormalUtil::estimatePCANormal(cloud,neighbor_nb);
        for ( int i = 0; i < cloud->size(); ++i )
        {
            normals->points[i].curvature=newnormals->points[i].curvature;
        }
    }else{
        QMessageBox::information(NULL, tr("Path"), tr("Wrong file type!")); 
    }

    qDebug()<<"Updating normal";
	pcl::PointCloud<pcl::Normal>::Ptr out_normals(new pcl::PointCloud<pcl::Normal>);
	out_normals->resize(normals->size());
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (cloud);

    pcl::StopWatch timer;
    double passed=0.0;
    int skip=0;
    timer.reset();
    // for each point
    #pragma omp parallel for
	for (int i=0;i<cloud->size();i++)
	{
        double curvature=normals->points[i].curvature;
        if ( curvature < 0.01 )
        {
            out_normals->points[i]=normals->points[i];
            ++skip;
            continue;
        }
        std::map<int,int> mapping;
        int curIdx;
		pcl::PointXYZ searchPoint=cloud->points[i];
		// K nearest neighbor search
		int K = Parameter::instance()->k;

		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
			MatrixXd X(3,K);
			assert(K==pointIdxNKNSearch.size());
			for (int j=0;j<pointIdxNKNSearch.size();j++)
			{
                // local to global index mapping
				int idx=pointIdxNKNSearch.at(j); // real index
                mapping.insert(std::make_pair(j,idx));
                if (idx==i) {
                    curIdx=j;
                }
				X(0,j)=normals->points[idx].normal_x;
				X(1,j)=normals->points[idx].normal_y;
				X(2,j)=normals->points[idx].normal_z;
			}
			double lambda=Parameter::instance()->lambda;
            
            // timer.reset();
			MatrixXd Z=NormalUtil::optimization(X,lambda);
            // passed=timer.getTime();
            // qDebug()<<"optimization takes="<<passed;

            // timer.reset();
            vcluster labels=NormalUtil::spectral_clustering(Z);
            // passed=timer.getTime(); 
            // qDebug()<<"spectral takes="<<passed;

            // update normal
            // timer.reset();
            int type=labels[curIdx];
            std::vector<int> clusters;
            for (int j = 0; j < labels.size(); j++) {
                if (labels[j]==type) {
                    clusters.push_back(mapping[j]);
                }
            }
            EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
            Eigen::Vector4f xyz_centroid;
            pcl::computeMeanAndCovarianceMatrix (*cloud, clusters, covariance_matrix, xyz_centroid);
            float nx,ny,nz;
            // EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
            // EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
            // pcl::eigen33 (covariance_matrix, eigen_value, eigen_vector);
            EigenSolver<Eigen::Matrix3f> eigensolver(covariance_matrix);
            VectorXcf v = eigensolver.eigenvectors().col(0);
            nx=v(0).real();
            ny=v(1).real();
            nz=v(2).real();
            if (nx!=nx) {
                qDebug()<<"ERROR! Cluster nb="<<clusters.size();
                NormalUtil::write_local(cloud,normals,i,labels,mapping);
            }
            // passed=timer.getTime(); 
            // qDebug()<<"update normal takes="<<passed;
            // qDebug()<<"nx="<<nx<<",ny="<<ny<<",nz="<<nz;
            out_normals->points[i]=pcl::Normal(nx,ny,nz);
		}
	}
    passed=timer.getTimeSeconds(); 
    qDebug()<<"overall takes="<<passed;
    qDebug()<<"skiped points="<<skip<<", "<<(double)skip/(double)cloud->size();

    // output
    NormalUtil::write_pointset(cloud,out_normals,"out.xyzn");
}

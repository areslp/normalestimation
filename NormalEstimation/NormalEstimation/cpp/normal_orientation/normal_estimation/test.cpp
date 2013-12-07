#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <boost/lexical_cast.hpp>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>

#include <utility> // defines std::pair
#include <list>
#include <fstream>

// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef CGAL::Point_with_normal_3<Kernel> Point_with_normal;
typedef Kernel::Sphere_3 Sphere;
typedef std::list<Point_with_normal> PointList;

int main(int argc,char** argv)
{
	std::string fname;
	std::string oname;
	int k;
	if (argc==3)
	{
		fname=argv[1];
		oname=argv[2];
		k=10;
	}else if (argc==4)
	{
		fname=argv[1];
		k=boost::lexical_cast<int>(argv[2]);
		oname=argv[3];
	}
	else{
		std::cout<<"argc:"<<argc<<",exiting..."<<std::endl;
		std::cout<<"normal_orientation infile k outfile"<<std::endl;
		return 0;
	}
	// Reads a .xyz point set file in points[].
	PointList points;
	std::ifstream stream(fname.c_str());
	if (!stream ||
		!CGAL::read_xyz_points_and_normals(
		stream,
		std::back_inserter(points),
		CGAL::make_normal_of_point_with_normal_pmap(std::back_inserter(points))))
	{
		std::cerr << "Error: cannot read file" << std::endl;
		return EXIT_FAILURE;
	}

	int nb_neighbors=k;
	// Orients normals.
	// Note: mst_orient_normals() requires an iterator over points
	// as well as property maps to access each point's position and normal.
	std::list<Point_with_normal>::iterator unoriented_points_begin =
		CGAL::mst_orient_normals(points.begin(), points.end(),
		CGAL::make_normal_of_point_with_normal_pmap(points.begin()),
		nb_neighbors);

	// Optional: delete points with an unoriented normal
	// if you plan to call a reconstruction algorithm that expects oriented normals.
	points.erase(unoriented_points_begin, points.end());

	// Optional: after erase(), use Scott Meyer's "swap trick" to trim excess capacity
	std::list<Point_with_normal>(points).swap(points);

	std::ofstream ot(oname.c_str());

	for (std::list<Point_with_normal>::iterator it=points.begin();it!=points.end();it++)
	{
		Point_with_normal pp=*it;
		Point point=pp.position();
		Vector normal=pp.normal();
		ot<<point.x()<<" "<<point.y()<<" "<<point.z()<<" "<<normal.x()<<" "<<normal.y()<<" "<<normal.z()<<std::endl;
	}

	ot.close();

	return EXIT_SUCCESS;
}
/*
 * =====================================================================================
 *
 *       Filename:  Normal.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2013年07月14日 14时28分50秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#include <stdlib.h>
#include "Normal.hpp"
Parameter * Parameter::instance_=NULL;


//implementation
void NormalUtil::write_local(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals, int idx, vcluster& labels, std::map<int,int>& mapping){
    ofstream outfile("local.xyznq");
    int nb=labels.size();
    for (int i = 0; i < nb; i++) {
        int idx=mapping[i];
        pcl::PointXYZ& p=points->points[idx];
        pcl::Normal& n=normals->points[idx];
        int type=labels[i];
        outfile<<p.x<<" "<<p.y<<" "<<p.z<<" "<<n.normal_x<<" "<<n.normal_y<<" "<<n.normal_z<<" "<<type<<endl;
    }
    outfile.close();
}

void NormalUtil::write_pointset(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals, std::string filename){
    ofstream outfile(filename.c_str());
    for (int i = 0; i < points->size(); i++) {
        pcl::PointXYZ& p=points->points[i];
        pcl::Normal& n=normals->points[i];
        outfile<<p.x<<" "<<p.y<<" "<<p.z<<" "<<n.normal_x<<" "<<n.normal_y<<" "<<n.normal_z<<endl;
    }
    outfile.close();
}

//
pcl::PointCloud<pcl::PointXYZ>::Ptr NormalUtil::readPointCloud(std::string filename){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::ifstream file(filename.c_str());
	double x,y,z;
	while(file>>x>>y>>z){
		cloud->push_back(pcl::PointXYZ(x,y,z));
	}
	return cloud;
}

void NormalUtil::readPointCloud(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
	std::ifstream file(filename.c_str());
	double x,y,z,nx,ny,nz;
	while(file>>x>>y>>z>>nx>>ny>>nz){
		cloud->push_back(pcl::PointXYZ(x,y,z));
		normals->push_back(pcl::Normal(nx,ny,nz));
	}
}
pcl::PointCloud<pcl::Normal>::Ptr NormalUtil::estimatePCANormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int k=30){
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 3cm
	ne.setKSearch(k);
	// Compute the features
	ne.compute (*cloud_normals);
	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()
	return cloud_normals;
}

double NormalUtil::eps(){
	return 2.2204e-16;
}

double NormalUtil::norm2(MatrixXd& a){
	// Eigenvalues
	// MatrixXd temp = a.adjoint()*a;
	// SelfAdjointEigenSolver<MatrixXd> es(temp);
	// return sqrt(es.eigenvalues()[temp.rows()-1]);

	// SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(a);
	return svd.singularValues()[0];
}
Eigen::MatrixXd NormalUtil::orth(Eigen::MatrixXd& a){
	//QR

	//Eigen::HouseholderQR<Eigen::MatrixXd> qrofa(a);
	//Eigen::MatrixXd Q=qrofa.householderQ();
	//Eigen::MatrixXd R=qrofa.matrixQR().template triangularView<Upper>();
	//double tol=eps()*R.norm();
	//Eigen::VectorXd T=R.diagonal().cwiseAbs(); // TODO: R may be a vector
	//Eigen::MatrixXd Z=MatrixXd::Zero(T.rows(),T.cols());
	//T=T.array()-tol;
	//T=T.cwiseMax(Z);
	//double r=T.sum();
	//if (r>0)
	//{
	//	int m=Q.rows();
	//	int n=Q.cols();
	//	Q=Q.topLeftCorner(m,r);
	//	Q*=-1;
	//	Q.block(0,r-1,m,1)*= -1;
	//}else{
	//	throw std::exception("QR decomposition failed");
	//}
	//return Q;

	//SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU);
	int m=a.rows();
	int n=a.cols();
	VectorXd s=svd.singularValues();
	double tol=max(m,n)*s.maxCoeff()*eps();
	int r=0;
	for (int i=0;i<s.size();i++)
	{
		if (s(i)>tol)
		{
			r++;
		}
	}
	MatrixXd Q=svd.matrixU().leftCols(r);
	return Q;
}

MatrixXd NormalUtil::solve_l21(MatrixXd &input, double alpha){
	// pcl::StopWatch timer;
	// timer.reset();
	MatrixXd output=input;
	int m=input.rows();
	int n=input.cols();
	for (int i=0;i<n;i++)
	{
		VectorXd col=input.col(i);
		double norm_col=col.norm();
		//if (norm_col-norm_col!=norm_col-norm_col)
		//{
		//	for (int ii=0;ii<col.size();ii++)
		//	{
		//		qDebug()<<col(ii);
		//	}
		//	qDebug()<<norm_col; //inf, norm value > MAX_double
		//}	
		if (norm_col-alpha<=0)
		{
			output.col(i).fill(0);
		}else{
			output.col(i)*=(norm_col-alpha)/norm_col;
		}
	}
	// double passed=timer.getTimeSeconds();
	// qDebug()<<"L21 takes="<<QString("%1").arg(passed,0,'g',10);
	return output;
}

int NormalUtil::solve_svs(MatrixXd &input, double tau,MatrixXd& U, VectorXd& s, MatrixXd& V){
	// pcl::StopWatch timer;
	// timer.reset();
	// qDebug()<<"tau="<<QString("%1").arg(tau,0,'e',5);
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(input, Eigen::ComputeThinU |
		Eigen::ComputeThinV);
	// double passed=timer.getTimeSeconds();
	// qDebug()<<"svd takes="<<QString("%1").arg(passed,0,'g',10);
	VectorXd s1=svd.singularValues();
	MatrixXd U1=svd.matrixU();
	MatrixXd V1=svd.matrixV();

	int size=s1.size();
	int svp=0;
	for (int i=0;i<size;i++)
	{
		if (s1(i)>tau)
		{
			svp++;
		}
	}
	s=s1.head(svp);
	s=s.array()-tau;
	U=U1.leftCols(svp);
	V=V1.leftCols(svp);

	// double passed=timer.getTimeSeconds();
	// qDebug()<<"singular value thresholding takes="<<QString("%1").arg(passed,0,'g',10);
	return svp;
}

void NormalUtil::solve_soft(MatrixXd& X, double tau){
	// TODO: not implemented yet
}


void NormalUtil::optimization(Eigen::MatrixXd& X,double lambda,Eigen::MatrixXd& U,Eigen::VectorXd& s,Eigen::MatrixXd& V,Eigen::MatrixXd& P){
	// acc as lrr
	MatrixXd T=X.adjoint();
	P=orth(T);
	int r=P.cols();
	MatrixXd A=X*P;
	// parameters
	bool convergenced=false;
	int iter=0;
	double rho=1.9;
	double normfX=X.norm();
	double tol1=1e-4;
	double tol2=1e-5;
	int d=X.rows();
	int n=X.cols();
	int maxIter=1000;
	double max_mu=1e10;
	double norm2X=norm2(X);
	double mu=min(d,n)*tol2;
	MatrixXd Xg=X;
	double eta=norm2X*norm2X*1.02;
	// init vars
	int sv=1;
	U=MatrixXd::Zero(r,sv);
	V=MatrixXd::Zero(n,sv);
	s=VectorXd::Zero(sv,1);
	MatrixXd AZ=MatrixXd::Zero(d,n);
	MatrixXd E=MatrixXd::Zero(d,n);
	MatrixXd Y=MatrixXd::Zero(d,n);
	MatrixXd Ek=E;
	MatrixXd Uk=U;
	VectorXd sk=s;
	MatrixXd Vk=V;
	int svp=0;
	// main loop
	while (iter<maxIter)
	{
		Ek=E;
		Uk=U;
		sk=s;
		Vk=V;
		MatrixXd T=X-AZ+Y/mu;
		E=solve_l21(T,lambda/mu);
		//qDebug()<<"E is finite?------>"<<is_finite(E);
		T=U*s.asDiagonal()*V.adjoint()+A.adjoint()*(X-AZ-E+Y/mu)/eta;
		// qDebug()<<"T is finite?------>"<<is_finite(T);
		svp=solve_svs(T,1/(mu*eta),U,s,V);
		// T=U*s.asDiagonal()*V.adjoint();
		// qDebug()<<"Z is finite?------>"<<is_finite(T);
		// check convergence
		double diffZ=sqrt(abs(s.norm()*s.norm()+sk.norm()*sk.norm()-
			2*((s.asDiagonal()*(V.adjoint()*Vk)).cwiseProduct((U.adjoint()*Uk)*sk.asDiagonal())).sum()));
		double relChgZ=diffZ/normfX;
		double relChgE=(E-Ek).norm()/normfX;
		//qDebug()<<"relChgZ="<<relChgZ;
		//qDebug()<<"relChgE="<<relChgE;
		double relChg=max(relChgE,relChgZ);
		AZ=A*U;
		for (int i=0;i<U.cols();i++)
		{
			AZ.col(i)*=s(i);
		}
		AZ*=V.adjoint();
		MatrixXd dY=X-AZ-E;
		double recErr=dY.norm()/normfX;
		convergenced=(recErr<tol1) && (relChg<tol2);
		// if (convergenced || iter==maxIter-1 || iter % 1 ==0)
		// {
		// qDebug()<<"iter="<<iter+1<<",mu="<<QString("%1").arg(mu,0,'g',10)<<",rank(Z)="<<svp<<",relChg="<<relChg<<",recErr="<<recErr;
		// }
		if (convergenced)
		{
			break;
		}
		Y=Y+mu*dY;
		mu=min(max_mu,mu*rho);
		iter++;
	}
}


Eigen::MatrixXd NormalUtil::optimization(Eigen::MatrixXd& X,double lambda){
	// acc as lrr
	MatrixXd T=X.adjoint();
	MatrixXd P=orth(T);
	int r=P.cols();
	MatrixXd A=X*P;
	// parameters
	bool convergenced=false;
	int iter=0;
	double rho=1.9;
	double normfX=X.norm();
	double tol1=1e-4;
	double tol2=1e-5;
	int d=X.rows();
	int n=X.cols();
	int maxIter=1000;
	double max_mu=1e10;
	double norm2X=norm2(X);
	double mu=min(d,n)*tol2;
	MatrixXd Xg=X;
	double eta=norm2X*norm2X*1.02;
	// init vars
	int sv=1;
	MatrixXd U=MatrixXd::Zero(r,sv);
	MatrixXd V=MatrixXd::Zero(n,sv);
	VectorXd s=VectorXd::Zero(sv,1);
	MatrixXd AZ=MatrixXd::Zero(d,n);
	MatrixXd E=MatrixXd::Zero(d,n);
	MatrixXd Y=MatrixXd::Zero(d,n);
	MatrixXd Ek=E;
	MatrixXd Uk=U;
	VectorXd sk=s;
	MatrixXd Vk=V;
	int svp=0;
	// main loop
	while (iter<maxIter)
	{
		Ek=E;
		Uk=U;
		sk=s;
		Vk=V;
		MatrixXd T=X-AZ+Y/mu;
		E=solve_l21(T,lambda/mu);
		//qDebug()<<"E is finite?------>"<<is_finite(E);
		T=U*s.asDiagonal()*V.adjoint()+A.adjoint()*(X-AZ-E+Y/mu)/eta;
		// qDebug()<<"T is finite?------>"<<is_finite(T);
		svp=solve_svs(T,1/(mu*eta),U,s,V);
		// T=U*s.asDiagonal()*V.adjoint();
		// qDebug()<<"Z is finite?------>"<<is_finite(T);
		// check convergence
		double diffZ=sqrt(abs(s.norm()*s.norm()+sk.norm()*sk.norm()-
			2*((s.asDiagonal()*(V.adjoint()*Vk)).cwiseProduct((U.adjoint()*Uk)*sk.asDiagonal())).sum()));
		double relChgZ=diffZ/normfX;
		double relChgE=(E-Ek).norm()/normfX;
		//qDebug()<<"relChgZ="<<relChgZ;
		//qDebug()<<"relChgE="<<relChgE;
		double relChg=max(relChgE,relChgZ);
		AZ=A*U;
		for (int i=0;i<U.cols();i++)
		{
			AZ.col(i)*=s(i);
		}
		AZ*=V.adjoint();
		MatrixXd dY=X-AZ-E;
		double recErr=dY.norm()/normfX;
		convergenced=(recErr<tol1) && (relChg<tol2);
		// if (convergenced || iter==maxIter-1 || iter % 1 ==0)
		// {
		// qDebug()<<"iter="<<iter+1<<",mu="<<QString("%1").arg(mu,0,'g',10)<<",rank(Z)="<<svp<<",relChg="<<relChg<<",recErr="<<recErr;
		// }
		if (convergenced)
		{
			break;
		}
		Y=Y+mu*dY;
		mu=min(max_mu,mu*rho);
		iter++;
	}

	MatrixXd Z=U*s.asDiagonal()*V.adjoint();
    // save2file(U,"U.txt");
    // save2file(s,"s.txt");
    // save2file(V,"V.txt");
    // save2file(P,"P.txt");
	Z=P*Z;
	return Z;
}
int NormalUtil::predictClusterNumber(const VectorXd& s, double tau){
    int n=s.size();
	double soft_thresholding=0.0;
	for (int i=0;i<n;i++)
	{
		if(s(i)>=tau)
			soft_thresholding=soft_thresholding+1;
		else
			soft_thresholding=soft_thresholding+(double)log2(double(1+(s(i)/tau)*(s(i)/tau)));
	}
	int subspace_num=n-round(soft_thresholding);
	if (subspace_num>3)
	{
		qDebug()<<"subspace_num:"<<subspace_num;
	}
	return subspace_num;
}

int NormalUtil::predictClusterNumber(MatrixXd& L, double tau){
	// save2file(L,"L.txt");
	int n=L.rows();
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(L, Eigen::ComputeThinU |
		Eigen::ComputeThinV);
	VectorXd s=svd.singularValues();
	// printVector(s);
	double soft_thresholding=0.0;
	for (int i=0;i<n;i++)
	{
		if(s(i)>=tau)
			soft_thresholding=soft_thresholding+1;
		else
			soft_thresholding=soft_thresholding+(double)log2(double(1+(s(i)/tau)*(s(i)/tau)));
	}
	int subspace_num=n-round(soft_thresholding);
	if (subspace_num>3)
	{
		qDebug()<<"subspace_num:"<<subspace_num;
	}
	return subspace_num;
}


vcluster NormalUtil::spectral_clustering(Eigen::MatrixXd& U,Eigen::VectorXd& s,Eigen::MatrixXd& V,Eigen::MatrixXd& P){
    // pcl::StopWatch timer;
    // double passed=0.0;
    // timer.reset();
	s=s.cwiseSqrt();
	MatrixXd U_tiled=P*U*(s.asDiagonal());
	for (int i=0;i<U_tiled.rows();i++)
	{
		U_tiled.row(i)=U_tiled.row(i)/U_tiled.row(i).norm();
	}
	MatrixXd S=U_tiled*(U_tiled.adjoint());
	S=S.array().square();
    // passed=timer.getTime();
    // qDebug()<<"preop takes="<<passed;
    // timer.reset();
	MatrixXd L=compute_laplacian(S);
    // passed=timer.getTime();
    // qDebug()<<"compute Laplacian takes="<<passed;
    // predict the cluster number
    // timer.reset();
    //直接计算L的特征值和特征向量，避免在spectral里再计算一次
    SelfAdjointEigenSolver<MatrixXd> es(L);
	int k=predictClusterNumber(es.eigenvalues(),Parameter::instance()->tau);
    // passed=timer.getTime();
    // qDebug()<<"predictClusterNumber takes="<<passed;
	// spectral clustering
    // timer.reset();
	vcluster clusters = spectral(es.eigenvectors(), k); //每一列是是一个特征向量
    // passed=timer.getTime();
    // qDebug()<<"k-means takes="<<passed;
	return clusters;

}


vcluster NormalUtil::spectral_clustering(MatrixXd& W){
    pcl::StopWatch timer;
    double passed=0.0;
    timer.reset();
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU |
		Eigen::ComputeThinV);
    passed=timer.getTime();
    qDebug()<<"svd takes="<<passed;
    timer.reset();
	VectorXd s=svd.singularValues();
	MatrixXd U=svd.matrixU();
	MatrixXd V=svd.matrixV();
    // save2file(U,"U1.txt");
    // save2file(s,"s1.txt");
    // save2file(V,"V1.txt");

	s=s.cwiseSqrt();
	MatrixXd U_tiled=U*(s.asDiagonal());
	for (int i=0;i<U_tiled.rows();i++)
	{
		U_tiled.row(i)=U_tiled.row(i)/U_tiled.row(i).norm();
	}
	MatrixXd S=U_tiled*(U_tiled.adjoint());
	S=S.array().square();
    passed=timer.getTime();
    qDebug()<<"preop takes="<<passed;
    timer.reset();
	MatrixXd L=compute_laplacian(S);
    passed=timer.getTime();
    qDebug()<<"compute Laplacian takes="<<passed;
	// predict the cluster number
    timer.reset();
	int k=predictClusterNumber(L,Parameter::instance()->tau);
    passed=timer.getTime();
    qDebug()<<"predictClusterNumber takes="<<passed;
	// spectral clustering
    timer.reset();
	vcluster clusters = spectral(S, k, 0.01, 10);
    passed=timer.getTime();
    qDebug()<<"k-means takes="<<passed;
	return clusters;
}

void NormalUtil::save2file(MatrixXd& X, std::string filename){
	ofstream file;
	file.open(filename);
	for (int i=0;i<X.rows();i++)
	{
		for (int j=0;j<X.cols();j++)
		{
			file<<X(i,j)<<" ";
		}
		file<<endl;
	}
	file.close();
}
void NormalUtil::save2file(VectorXd& X, std::string filename){
	ofstream file;
	file.open(filename);
	for (int i=0;i<X.size();i++)
	{
		file<<X(i)<<" ";
	}
	file.close();
}
void NormalUtil::printVector(VectorXd& v){
	for (int i = 0; i < v.size(); i++) {
		qDebug()<<v(i)<<" ";
	}
}
MatrixXd NormalUtil::compute_laplacian(MatrixXd& W){
	// pcl::StopWatch timer;
	// double passed=0.0;
	int i, j, n = W.rows();
	double c;
	MatrixXd D = MatrixXd::Zero(n, n);
	forn(i, n) {
		c = 0;
		forn(j, n) c += W(i, j);
		D(i, i) = c;
	}
	forn(i, n) {
		D(i,i)=1/sqrt(D(i,i));
	}
	// timer.reset();
	MatrixXd L=MatrixXd::Identity(n,n)-D*W*D;
	// passed=timer.getTime();
	//qDebug()<<"lap2="<<passed;
	return L;
}


void Parameter::loadparam(){
	// Parameter():k(100),lambda(1.0),tau(8e-1),kpca(200),orientation(100),curvature(0.01){}
    ifstream paramfile("param.txt");
    paramfile>>Parameter::instance()->kpca;
    paramfile>>Parameter::instance()->k;
    paramfile>>Parameter::instance()->orientation;
    paramfile>>Parameter::instance()->tau;
    paramfile>>Parameter::instance()->curvature;
    paramfile.close();
}

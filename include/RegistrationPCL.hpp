#include <Registration_.h>
#pragma comment(lib,"../dll/Registration.lib") 

#include <pcl/common/transforms.h>
#include <Macro.hpp>
#include "GraspSystem.hpp"
#include "Opencv.hpp"
#include "Classification.hpp"
#include <opencv2\core.hpp>
using std::pair;


struct MyObject{
	string category;
	string model_path;
	string grasp_path;
	PointCloudNT::Ptr model; // (new PointCloudNT);
	PointCloudT::Ptr grasp; // (new PointCloudNT);
	float leaf;
};

typedef vector<MyObject> Objects;
class DesktopObj{
public:
	/**
	* @brief DesktopObj: put the models and grasp points cloud in directory "./models/xxx/"
	*					 and rename the model file as "xxx_-scaled.pcd" and rename the
	*					 grasp file as 	"xxx_-grasp-scaled.pcd"
	* @param catergories
	*/
	DesktopObj(vector<string> catergories);
	int load_(MyObject &mobj);
	
	MyObject& operator[](string query){
		const int &index = name2index[query];
		return objects[index];
	}
	MyObject& operator[](int query){
		return objects[query];
	}
	


private:
	map<string, int> name2index;
	Objects objects;
	float leaf;
};

DesktopObj::DesktopObj(vector<string> catergories)
{
	static string prefix = ".//mdoels//";
	static string suffix_grasp = "-grasp-scaled.pcd";
	static string suffix_model = "-scaled.pcd";
	int cnt = 0;
	objects.resize(catergories.size()); // If one load failed, can load the next
	for (auto name : catergories){
		string model_path = prefix + name + suffix_model;
		string grasp_path = prefix + name + suffix_grasp;
		PointCloudNT::Ptr m(new PointCloudNT); // (new PointCloudNT);
		PointCloudT::Ptr g(new PointCloudT); // (new PointCloudNT);
		MyObject& curobj = objects[cnt] = { name, model_path, grasp_path, m, g, leaf };
		load_(curobj);
		name2index[name] = ++cnt;
	}
}

int DesktopObj::load_(MyObject &mobj)
{
	//Load 3D Model
	if (!LoadModel(mobj.model_path, mobj.model)){
		MESSAGE_COUT("ERROR", "Failed to load model from [" << mobj.model_path << "]");
		return -1;
	}
	Downsample(mobj.model, mobj.leaf);
	//Load grasping point region
	if (!LoadGraspPcd(mobj.grasp_path, mobj.grasp)){
		MESSAGE_COUT("ERROR", "Failed to load model from [" << mobj.model_path << "]");
		return -2;
	}
	return 0;
}



class RegistrationPCL{
public:
	// Construction
	RegistrationPCL()
	{}
	RegistrationPCL(RegisterParameter p) :para_(p)
	{}
	//// Set parameters 
	//void setLeaf(float l); // Number of RANSAC iterations
	//void setMaximumIterationsRANSA(int mi); // Number of RANSAC iterations
	//void setNumberOfSamples(int n); // Number of points to sample for generating/prerejecting a pose
	//void setCorrespondenceRandomness(int c); // Number of nearest features to use
	//void setSimilarityThreshold(float sim); // Polygonal edge length similarity threshold
	//void setMaxCorrespondence(float mc); // Inlier threshold
	//void setInlierFraction(float irate);

	// Get parameters
	RegisterParameter getParameters(){
		return para_;
	}
	// Preparation
	int Preparation(string model_path, string grasp_path);
	// Apply
	Matrix4f Apply(PointCloudNT::Ptr &seg);
private:	
	RegisterParameter para_;
	Objects objects_;
};

//int RegistrationPCL::Preparation(string category, string model_path, string grasp_path)
//{
//	//Load 3D Model
//	if (!LoadModel(model_path, mesh_.model)){
//		MESSAGE_COUT("ERROR", "Failed to load model from [" << model_path << "]");
//		return -1;
//	}
//	Downsample(mesh_.model, para_.leaf);
//	//Load grasping point region
//	if (!LoadGraspPcd(grasp_path, mesh_.grasp)){
//		MESSAGE_COUT("ERROR", "Failed to load model from [" << model_path << "]");
//		return -2;
//	}
//	return 0;
//}

Matrix4f RegistrationPCL::Apply(PointCloudNT::Ptr &seg)
{
	//generate Point Cloud
	PointCloudNT::Ptr model_align(new PointCloudNT);
	PointCloudT::Ptr grasp_align(new PointCloudT);
	/***
	need to scale mesh
		size_t sz = PXC2PCL(segment, vertices, mesh, 1.0 / scale);
		MESSAGE_COUT("INFO", "Generate Point Cloud: " << sz);
	***/
	//Alignment
	Matrix4f transformation;
	transformation = RegistrationNoShow(mesh_.model, seg, model_align, para_);
	if (transformation != Matrix4f::Identity()) // If equal to E, Alignment failed 
		pcl::transformPointCloud(*mesh_.grasp, *grasp_align, transformation);
	return transformation;
}


////Reflect
//Reflect_Result show2d;
//show2d.model.resize(model_align->size());
//show2d.grasp.resize(grasp_align->size());
//vector<PXCPoint3DF32> result;
//for (auto &pc : *model_align) {
//	result.push_back({ scale * pc.x, scale * pc.y, scale * pc.z });
//}
//projection->ProjectCameraToDepth(result.size(), &result[0], &show2d.model[0]);
//result.clear();
//for (auto &pc : *grasp_align) {
//	result.push_back({ scale * pc.x, scale * pc.y, scale * pc.z });
//}
//projection->ProjectCameraToDepth(result.size(), &result[0], &show2d.grasp[0]);
//return show2d;

// Convert Realsense's PXC to PCL's PointCloud
size_t PXC2PCL(	const PointSet &pSet, 
				vector<PXCPoint3DF32> &vertices, 
				PointCloudNT::Ptr &scene, 
				float scale = 1.f / 300.f)
{
	for (auto p : pSet) {
		p += p;
		PXCPoint3DF32 ppp = vertices[p.y * 640 + p.x];   // 640 is magic number
		scene->push_back(PointNT());
		PointNT& ps = scene->back();
		ps.x = ppp.x*scale;
		ps.y = ppp.y*scale;
		ps.z = ppp.z*scale;
	}
	return scene->size();
}


void myRegistration( )
{
	
}

void myAlgorithm()
{
	clock_t start, end;
	Mat color, depth, depth2, color2;
	// configure segmentation
	Size segSize(320, 240);
	unsigned topk = 6;
	short threshold = 2;
	Segmentation myseg(segSize, topk, threshold);
	// configure classification
	string classifier_path;
	Classification myclass(classifier_path);
	// configure registration
	string model_path;
	string grasp_path;
	RegisterParameter rp;
	double scale = 300.0;
	RegistrationPCL myreg(rp);
	myreg.Preparation(model_path, grasp_path);

	while (1){
		start = clock();
		//////////////////////////////////////////////
		// Got data
		// ....
		//
		/////////////////////////////////////////////
		end = clock();
		// segmentation
		resize(color, color2, segSize);
		resize(depth, depth2, segSize);
		myseg.Segment(depth2, color2);
		const vector<PointSet>& mainRegions = myseg.mainSegmentation();
		//vector<Rect> mainRegions = myseg.boundBoxes_;
		// classification
		vector<pair<int,int>> candidates;
		for (int i = 0; i < mainRegions.size(); i++) {
			Rect r = boundingRect(mainRegions[i]);
			r.x += r.x;
			r.y += r.y;
			r.width += r.width;
			r.height += r.height;
			float p = myclass.predict(color(r));
			if (p) 
				candidates.push_back(pair<int, int>(p, i));
		}
		// registration
		for (auto c : candidates){
			const PointSet &ps = mainRegions[c.second];
			vector<PXCPoint3DF32> p3d;
			PointCloudNT::Ptr sceneSeg;
			PXC2PCL(ps, p3d, sceneSeg, 1 / scale);
			Matrix4f transformation = myreg.Apply(sceneSeg);
			if (transformation != Matrix4f::Identity()) {
				
			}
		}
		

		myseg.clear();
		waitKey(1);
	}

	
}
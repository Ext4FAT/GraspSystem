#include "Opencv.hpp"
#include "Registration_.h"
#pragma comment(lib,"../dll/Registration.lib") 

namespace _IDLER_{
	/**
	* @class MyObject: 
	*/
	struct Object{
		string category;
		string model_path;
		string grasp_path;
		PointCloudNT::Ptr model; // (new PointCloudNT);
		PointCloudT::Ptr grasp; // (new PointCloudNT);
		float leaf;
	};

	/**
	* @class DesktopObj: 
	*/
	class Objects{
	public:
		/**
		* @brief DesktopObj: put the models and grasp points cloud in directory "./models/xxx/"
		*					 and rename the model file as "xxx_-scaled.pcd" and rename the
		*					 grasp file as 	"xxx_-grasp-scaled.pcd"
		* @param catergories	objects, e.g. bottle, cup, can, etc.
		*/
		Objects()
		{}
		Objects(vector<string> catergories);
		/**
		* @brief []: get MyObject from string/int
		* @param query	object name / object index
		*/
		inline Object& operator[](string query){
			const int &index = name2index_[query];
			return objects_[index];
		}
		inline Object& operator[](int query){
			return objects_[query];
		}
	private:
		/**
		* @brief load_: load model and grasp from MyObject's path
		* @param mobj	an empty object points cloud only with path
		*/
		int load_(Object &mobj);
	private:
		vector<Object> objects_; //MyObject's vector 
		map<string, int> name2index_; //mapping name to index
		float leaf; // Down sampling with leaf size
	};

	/**
	* @class RegistrationPCL:
	*/
	class RegistrationPCL{
	public:
		// Construction
		RegistrationPCL()
		{}
		RegistrationPCL(RegisterParameter p) :para_(p)
		{}

		// Preparation
		int Preparation();
		// Apply
		Matrix4f Apply(PointCloudNT::Ptr &seg);

		// Get parameters
		inline const RegisterParameter& getParameters(){
			return para_;
		}
		// Set parameters 
		inline void setLeaf(float l) { // Number of RANSAC iterations
			para_.leaf = l;
		}
		inline void setMaximumIterationsRANSA(int mi) { // Number of RANSAC iterations
			para_.MaximumIterationsRANSAC = mi;
		}
		inline void setNumberOfSamples(int n) { // Number of points to sample for generating/prerejecting a pose
			para_.NumberOfSamples = n;
		}
		inline void setCorrespondenceRandomness(int c) { // Number of nearest features to use
			para_.CorrespondenceRandomness = c;
		}
		inline void setSimilarityThreshold(float sim) { // Polygonal edge length similarity threshold
			para_.SimilarityThreshold = sim;
		}
		inline void setMaxCorrespondence(float mc) { // Inlier threshold
			para_.MaxCorrespondence = mc;
		}
		inline void setInlierFraction(float orate){ // Outlier rate
			para_.InlierFraction = orate;
		}
	private:
		RegisterParameter para_;
		Objects objects_;
	};

}
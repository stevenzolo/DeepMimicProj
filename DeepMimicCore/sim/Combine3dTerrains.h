#pragma once
#include "TerrainGen3D.h"

class cCombine3dTerrains : virtual public cTerrainGen3D
{
public:

	static Eigen::VectorXd mCombBlendParams;

	enum eCombParams
	{
		eOverBoxSpacing,
		eOverNarrowWidth,
		eOverGapDepth,
		eOverWallHeight,
		eOverBeamHeight,
		eOverBeamSpacing,
		eOverSlopeRatio,
		eOverSlopeHeight,
		eOverStairSpacing,
		eOverStairIncease,
		eOverStairHeight,
		eCombParamsMax
	};
	static const tParamDef gParamDefs[];
	static void GetDefaultParams(Eigen::VectorXd& out_params);
	static bool ParseParamsJson(const Json::Value& json, Eigen::VectorXd& out_params);

	typedef void(*tCombTerrainFunc)(
		const tVector& origin, const tVector& ground_size, double spacing_x, double spacing_z,
		const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags
		);
	static tCombTerrainFunc GetTerrainFunc(cGround::eType terrain_type);

	static void BuildStairRamp(
		const tVector& origin, const tVector& ground_size, double spacing_x, double spacing_z,
		const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags
	);

protected:

	static void AddFlat(const double h, const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
		const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags);
	static void AddRamp(const double start_h, const double end_h, const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
		double slope, const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags);
	static void AddStairs(const double start_h, const double end_h, const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
		double spacing_min, double spacing_max, double step_h_min, double step_h_max,
		const Eigen::Vector2i& out_res, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags);

};


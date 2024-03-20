#pragma once
#include "TerrainGen3D.h"

class cOverlayTerrainGen3D : virtual public cTerrainGen3D
{
public:

	static Eigen::VectorXd mOverBlendParams;

	enum eOverParams
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
		eOverParamsMax
	};
	static const tParamDef gParamDefs[];
	static void GetDefaultParams(Eigen::VectorXd& out_params);
	static bool ParseParamsJson(const Json::Value& json, Eigen::VectorXd& out_params);

	typedef void(*tOverTerrainFunc)(
		int s, double spacing_x, double spacing_z, const tVector& bound_min, const tVector& bound_max,
		const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags
		);
	static tOverTerrainFunc GetTerrainFunc(cGround::eType terrain_type);

	static void BuildDemo(
		int s, double spacing_x, double spacing_z, const tVector& bound_min, const tVector& bound_max,
		const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags
	);

protected:
	static const int gNumSlabs = 4;		// same to value in GroundVar3D.h
	static Json::Value mSlabOverlayTerrains[gNumSlabs];

	static void oBuildGaps(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildPit(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildWall(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildBeam(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildBars(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildSlopes(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildStairs(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildSlopeStair(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildStairSlope(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);

	static double oAddBox(
		const float spacing, const float depth, const float length, const tVector& origin,
		const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
		const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags);
	static double oAddSlope(
		const double slope, const double start_height, const tVector& origin,
		const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
		const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags);
	static double oAddStair(
		const double start_height, const float stair_space, const float stair_increase, const tVector& origin,
		const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
		const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oAddLanding(const float depth, 
		const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
		const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags);

};


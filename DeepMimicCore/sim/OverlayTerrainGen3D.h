#pragma once
#include "TerrainGen3D.h"
class cOverlayTerrainGen3D : public cTerrainGen3D
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
	static void LoadParams(const Json::Value& root);

	typedef void(*tOverTerrainFunc)(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);

	static void oBuildGaps(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void cOverlayTerrainGen3D::oBuildPit(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void cOverlayTerrainGen3D::oBuildWall(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void cOverlayTerrainGen3D::oBuildBeam(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void cOverlayTerrainGen3D::oBuildBars(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void cOverlayTerrainGen3D::oBuildSlopes(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void cOverlayTerrainGen3D::oBuildStairs(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void cOverlayTerrainGen3D::oBuildSlopeStair(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void cOverlayTerrainGen3D::oBuildStairSlope(
		const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);


protected:
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


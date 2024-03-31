#pragma once
#include "TerrainGen3D.h"

class cOverlayTerrainGen3D : virtual public cTerrainGen3D
{
public:

	static Eigen::VectorXd mOverBlendParams;

	enum eType
	{
		eTypePlateau,
		eTypeGaps,
		eTypePit,
		eTypeWall,
		eTypeBeam,
		eTypeBars,
		eTypeSlopes,
		eTypeStairs,
		eTypeSlopeStair,
		eTypeStairSlope,
		eTypeMax
	};

	enum eOverParams
	{
		eOverBoxSpacing,
		eOverNarrowWidth,
		eOverGapDepth,
		eOverWallHeight,
		eOverBeamHeight,
		eOverBeamSpacing,
		eOverSlopeRatio,
		eOverStairSpacing,
		eOverStairIncease,
		eOverPlateauHeight,
		eOverParamsMax
	};
	static const tParamDef gParamDefs[];
	static void GetDefaultParams(Eigen::VectorXd& out_params);
	static bool ParseParamsJson(const Json::Value& json, Eigen::VectorXd& out_params);
	static void ParseType(const std::string& str, eType& out_type);

	typedef void(*tOverTerrainFunc)(
		int s, tVector slab_offset, double spacing_x, double spacing_z, const tVector& bound_min, const tVector& bound_max,
		const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags
		);
	static tOverTerrainFunc GetTerrainFunc(cGround::eType terrain_type);

	static void BuildDefaultDemo(
		int s, tVector slab_offset, double spacing_x, double spacing_z, const tVector& bound_min, const tVector& bound_max,
		const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags
	);
	static void BuildRover(
		int s, tVector slab_offset, double spacing_x, double spacing_z, const tVector& bound_min, const tVector& bound_max,
		const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags
	);

protected:
	static const int gNumSlabs = 4;		// same to value in GroundVar3D.h
	static Json::Value mSlabOverlayTerrains[gNumSlabs];
	static Eigen::Vector2i mbuild_terrain_point_dir;

	typedef void(*tOBuildFunc)(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static tOBuildFunc GetOBuildFunc(eType terrain_type);

	static void oBuildPlateau(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildGaps(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildPit(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildWall(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildBeam(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildBars(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildSlopes(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildStairs(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildSlopeStair(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oBuildStairSlope(
		const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
		tVector& overlay_bound_min, tVector& overlay_bound_max,
		double spacing_x, double spacing_z, cRand& rand,
		std::vector<float>& out_data, std::vector<int>& out_flags);

	static double oAddBox(
		const double start_height, const Eigen::Vector2i& point_dir, const float spacing, const float depth, const float length,
		const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, 
		double spacing_z, const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oAddSlope(
		const double slope, const double end_height, const double start_height, const Eigen::Vector2i& point_dir, const tVector& origin,
		const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
		const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oAddStair(
		const double start_height, const double end_height, const Eigen::Vector2i& point_dir, const float stair_space,
		const float stair_increase, const tVector& origin,
		const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
		const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags);
	static void oAddLanding(const float depth, const Eigen::Vector2i& point_dir,
		const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
		const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags);

};


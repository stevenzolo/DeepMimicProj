#include "OverlayTerrainGen3D.h"

Json::Value cOverlayTerrainGen3D::mSlabOverlayTerrains[gNumSlabs] = {};

Eigen::VectorXd cOverlayTerrainGen3D::mOverBlendParams = Eigen::VectorXd::Zero(eOverParamsMax);

Eigen::Vector2i cOverlayTerrainGen3D::mbuild_terrain_point_dir = Eigen::Vector2i(1, 0);

std::string gBuildTypeNames[cOverlayTerrainGen3D::eTypeMax] =
{
	"plateau",
	"gaps",
	"pit",
	"wall",
	"beam",
	"bars",
	"slopes",
	"stairs",
	"slopestair",
	"stairslope"
};

const cOverlayTerrainGen3D::tParamDef cOverlayTerrainGen3D::gParamDefs[] =
{
	// the minimum value should be larger than cTerrainGen3D::gVertSpacing(0.2)
	{ "BoxSpacing", 1 },	// space between the plane and other terrains
	{ "NarrowWidth", 0.1 },	// allow character cross over, less than max step length of man (~0.8)
	{ "GapDepth", -2 },	// depth for gap/pit
	{ "WallHeight", 0.5 },	// used in wall/landing, allow character climb up
	{ "BeamHeight", 0.2 },	// used in beam, allow character jump up
	{ "BeamSpacing", 0.6 },	// narrow, < 2 * foot width
	{ "SlopeRatio", 0.3 },	// ratio of height/width, positive slope means up-then-down, while neg for down-up
	{ "StairSpacing", 0.7 },	
	{ "StairIncease", -0.2 },	// positive increase means up-then-down, while neg for down-up
	{ "PlateauHeight", 3.0 }
};

void cOverlayTerrainGen3D::GetDefaultParams(Eigen::VectorXd& out_params)
{
	// revised eParamsMax -> eOverParamsMax to load default parameters of overlay terrains. @Yan
	out_params = Eigen::VectorXd::Zero(eOverParamsMax);
	assert(sizeof(gParamDefs) / sizeof(gParamDefs[0]) == eOverParamsMax);
	for (int i = 0; i < eOverParamsMax; ++i)
	{
		out_params[i] = gParamDefs[i].mDefaultVal;
	}
}

bool cOverlayTerrainGen3D::ParseParamsJson(const Json::Value& json, Eigen::VectorXd& out_params)
{
	if (!json["SlabTerrains"].isNull())
	{
		Json::Value slabs_terrain_arr = json["SlabTerrains"];
		assert(slabs_terrain_arr.isArray());
		assert(gNumSlabs == slabs_terrain_arr.size());

		for (int i = 0; i < gNumSlabs; ++i)
		{
			mSlabOverlayTerrains[i] = slabs_terrain_arr[i];
		}
	}

	GetDefaultParams(mOverBlendParams);
	for (int i = 0; i < eOverParamsMax; ++i)
	{
		const std::string& name = gParamDefs[i].mName;
		if (!json[name].isNull())
		{
			double val = json[name].asDouble();
			mOverBlendParams[i] = val;
		}
	}

	cTerrainGen3D::LoadParams(json, out_params);
	return true;
}

void cOverlayTerrainGen3D::ParseType(const std::string& str, eType& out_type)
{
	bool valid = false;
	for (int i = 0; i < eTypeMax; ++i)
	{
		const std::string& curr_name = gBuildTypeNames[i];
		if (curr_name == str)
		{
			out_type = static_cast<eType>(i);
			valid = true;
			break;
		}
	}

	if (!valid)
	{
		printf("Invalid ground Type%s\n", str.c_str());
		assert(false);
	}
}


cOverlayTerrainGen3D::tOverTerrainFunc cOverlayTerrainGen3D::GetTerrainFunc(cGround::eType terrain_type)
{
	switch (terrain_type)
	{
	case cGround::eTypeOverlay3DDemo:
		return BuildDefaultDemo;
	case cGround::eTypeOverlay3DRover:
		return BuildRover;
	default:
		printf("Unsupported ground var3d type.\n");
		assert(false);
		return BuildDefaultDemo;
	}
}

cOverlayTerrainGen3D::tOBuildFunc cOverlayTerrainGen3D::GetOBuildFunc(eType build_type)
{
	switch (build_type)
	{
	case eTypePlateau:
		return oBuildPlateau;
	case eTypeGaps:
		return oBuildGaps;
	case eTypePit:
		return oBuildPit;
	case eTypeWall:
		return oBuildWall;
	case eTypeBeam:
		return oBuildBeam;
	case eTypeBars:
		return oBuildBars;
	case eTypeSlopes:
		return oBuildSlopes;
	case eTypeStairs:
		return oBuildStairs;
	case eTypeSlopeStair:
		return oBuildSlopeStair;
	case eTypeStairSlope:
		return oBuildStairSlope;
	default:
		printf("Unsupported ground var3d type.\n");
		assert(false);
		return oBuildPlateau;
	}
}


void cOverlayTerrainGen3D::BuildDefaultDemo(
	int s, tVector slab_offset, double spacing_x, double spacing_z, const tVector& bound_min, const tVector& bound_max,
	const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags
)
{
	// BuildFlat as base
	BuildFlat(bound_min, bound_max - bound_min, spacing_x, spacing_z, params, rand, out_data, out_flags);

	tVector overlay_bound_min = tVector::Zero();
	tVector overlay_bound_max = tVector::Zero();
	switch (s)
	{
	case 0:
		overlay_bound_min[0] = -10;
		overlay_bound_min[2] = -10;
		overlay_bound_max[0] = -3;
		overlay_bound_max[2] = -4;
		overlay_bound_min += slab_offset;
		overlay_bound_max += slab_offset;
		oBuildGaps(
			mbuild_terrain_point_dir, bound_min, bound_max, overlay_bound_min, overlay_bound_max,
			spacing_x, spacing_z, rand, out_data, out_flags
		);
		break;
	case 1:
		overlay_bound_min[0] = 3;
		overlay_bound_min[2] = -10;
		overlay_bound_max[0] = 12;
		overlay_bound_max[2] = -5;
		overlay_bound_min += slab_offset;
		overlay_bound_max += slab_offset;
		oBuildPit(
			mbuild_terrain_point_dir, bound_min, bound_max, overlay_bound_min, overlay_bound_max,
			spacing_x, spacing_z, rand, out_data, out_flags);
		break;
	case 2:
		overlay_bound_min[0] = -12;
		overlay_bound_min[2] = 3;
		overlay_bound_max[0] = -6;
		overlay_bound_max[2] = 9;
		overlay_bound_min += slab_offset;
		overlay_bound_max += slab_offset;
		oBuildSlopeStair(
			mbuild_terrain_point_dir, bound_min, bound_max, overlay_bound_min, overlay_bound_max,
			spacing_x, spacing_z, rand, out_data, out_flags);
		break;
	case 3:
		overlay_bound_min[0] = 3;
		overlay_bound_min[2] = 3;
		overlay_bound_max[0] = 9;
		overlay_bound_max[2] = 10;
		overlay_bound_min += slab_offset;
		overlay_bound_max += slab_offset;
		oBuildStairSlope(
			mbuild_terrain_point_dir, bound_min, bound_max, overlay_bound_min, overlay_bound_max,
			spacing_x, spacing_z, rand, out_data, out_flags);
		break;
	default:
		break;
	}
}

void cOverlayTerrainGen3D::BuildRover(
	int s, tVector slab_offset, double spacing_x, double spacing_z, const tVector& bound_min, const tVector& bound_max,
	const Eigen::VectorXd& params, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags
)
{
	// BuildFlat as base
	BuildFlat(bound_min, bound_max - bound_min, spacing_x, spacing_z, params, rand, out_data, out_flags);

	tVector overlay_bound_min = tVector::Zero();
	tVector overlay_bound_max = tVector::Zero();
	double ground_width = std::abs(bound_max[0] - bound_min[0]);	// approximately

	Json::Value terrain_root = mSlabOverlayTerrains[s];
	for (const std::string& terrain_func_key : terrain_root.getMemberNames())
	{
		eType slab_curr_build_type;
		ParseType(terrain_func_key, slab_curr_build_type);
		tOBuildFunc slab_curr_build = GetOBuildFunc(slab_curr_build_type);

		int overlay_num = terrain_root[terrain_func_key].size();
		for (int i = 0; i < overlay_num; ++i)
		{
			Json::Value overlay_params_arr = terrain_root[terrain_func_key][i];
			tVector overlay_bound_min = tVector::Zero();
			tVector overlay_bound_max = tVector::Zero();
			overlay_bound_min[0] = overlay_params_arr[0].asDouble() * ground_width;
			overlay_bound_min[2] = overlay_params_arr[1].asDouble() * ground_width;
			overlay_bound_max[0] = overlay_params_arr[2].asDouble() * ground_width;
			overlay_bound_max[2] = overlay_params_arr[3].asDouble() * ground_width;
			if (std::abs(overlay_bound_min[0]) == ground_width || std::abs(overlay_bound_max[0]) == ground_width)
			{	// evaluate terrain orientation with fixed given position
				mbuild_terrain_point_dir = Eigen::Vector2i(0, 1);
			}
			else if (std::abs(overlay_bound_min[2]) == ground_width || std::abs(overlay_bound_max[2]) == ground_width)
			{
				mbuild_terrain_point_dir = Eigen::Vector2i(1, 0);
			}
			// build terrain w.r.t slab offset.
			overlay_bound_min += slab_offset;
			overlay_bound_max += slab_offset;
			(*slab_curr_build)(mbuild_terrain_point_dir, bound_min, bound_max, overlay_bound_min, overlay_bound_max,
				spacing_x, spacing_z, rand, out_data, out_flags);
		}
	}
}

void cOverlayTerrainGen3D::oBuildPlateau(
	const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags
)
{
	const double h = mOverBlendParams[eOverPlateauHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	if (overlay_bound_min[0] != global_bound_min[0])
	{
		start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	}
	if (overlay_bound_min[2] != global_bound_min[2])
	{
		start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	}

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	return oAddLanding(h, point_dir, start_coord, overlay_ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildGaps(
	const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float gap_spacing = mOverBlendParams[eOverBoxSpacing];
	const float gap_width = mOverBlendParams[eOverNarrowWidth];
	const float depth = mOverBlendParams[eOverGapDepth];
	double terrain_start_h = mOverBlendParams[eOverPlateauHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;
	float gap_length = overlay_ground_size[2];	// gap across the whole overlay area
	double box_width_bound = overlay_ground_size[0];
	if (point_dir[1] == 1) 
	{ 
		gap_length = overlay_ground_size[0]; 
		box_width_bound = overlay_ground_size[2];
	}

	double total_w = 0;
	tVector box_size = overlay_ground_size;
	if (point_dir[1] == 1) { box_size[2] = gap_spacing + gap_width; }
	else { box_size[0] = gap_spacing + gap_width; }
	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	while (total_w < box_width_bound)
	{
		start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
		start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
		double width_added = oAddBox(
			terrain_start_h, point_dir, gap_spacing, depth, gap_length, overlay_bound_min, start_coord,
			box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
		total_w += width_added;
		overlay_bound_min[0] += width_added;
	}
}

void cOverlayTerrainGen3D::oBuildPit(
	const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float pit_spacing = mOverBlendParams[eOverBoxSpacing];
	const float depth = mOverBlendParams[eOverGapDepth];
	double terrain_start_h = mOverBlendParams[eOverPlateauHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;
	float pit_length = overlay_ground_size[2];
	if (point_dir[1] == 1) { pit_length = overlay_ground_size[0]; }

	double width_added = oAddBox(
		terrain_start_h, point_dir, pit_spacing, depth, pit_length, overlay_bound_min,
		start_coord, overlay_ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildWall(
	const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float wall_spacing = mOverBlendParams[eOverBoxSpacing];
	const float height = mOverBlendParams[eOverWallHeight];
	double terrain_start_h = mOverBlendParams[eOverPlateauHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;
	float wall_length = overlay_ground_size[2];
	if (point_dir[1] == 1) { wall_length = overlay_ground_size[0]; }

	double width_added = oAddBox(
		terrain_start_h, point_dir, wall_spacing, height, wall_length, overlay_bound_min,
		start_coord, overlay_ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildBeam(
	const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float box_spacing = mOverBlendParams[eOverBoxSpacing];
	const float height = mOverBlendParams[eOverBeamHeight];
	const float beam_space = mOverBlendParams[eOverBeamSpacing];
	double terrain_start_h = mOverBlendParams[eOverPlateauHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	double width_added = oAddBox(
		terrain_start_h, point_dir, box_spacing, height, beam_space, overlay_bound_min, start_coord,
		overlay_ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildBars(
	const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float bar_spacing = mOverBlendParams[eOverBoxSpacing];
	const float bar_width = mOverBlendParams[eOverNarrowWidth];
	const float height = mOverBlendParams[eOverBeamHeight];
	double terrain_start_h = mOverBlendParams[eOverPlateauHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;
	const float bar_length = overlay_ground_size[2];	// gap across the whole overlay area
	const double box_width_bound = overlay_ground_size[0];

	double total_w = 0;
	tVector box_size = overlay_ground_size;
	if (point_dir[1] == 1) { box_size[2] = bar_spacing + bar_width; }
	else { box_size[0] = bar_spacing + bar_width; }

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	while (total_w < box_width_bound)
	{
		start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
		start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
		double width_added = oAddBox(
			terrain_start_h, point_dir, bar_spacing, height, bar_length, overlay_bound_min,
			start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
		total_w += width_added;
		overlay_bound_min[0] += width_added;
	}
}

void cOverlayTerrainGen3D::oBuildSlopes(
	const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float slope_ratio = mOverBlendParams[eOverSlopeRatio];
	double terrain_start_h = mOverBlendParams[eOverPlateauHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);
	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	// additional 0.5 * width for landing
	float slope_width = 0;
	if (point_dir[0] == 1) { slope_width = overlay_ground_size[0] / 2.5; }
	else { slope_width = overlay_ground_size[2] / 2.5; }

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector box_size = overlay_ground_size;
	if (point_dir[0] == 1) { box_size[0] = slope_width; }
	else { box_size[2] = slope_width; }		// (point_dir[0] == 1) 

	oAddSlope(slope_ratio, terrain_start_h, -1000, 
		point_dir, overlay_bound_min, start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
	double slope_end_depth = 0;
	if (point_dir[0] == 1) { slope_end_depth = terrain_start_h + box_size[2] * slope_ratio; }
	else { slope_end_depth = terrain_start_h + box_size[0] * slope_ratio; }

	if (point_dir[0] == 1) { overlay_bound_min[0] += slope_width; }
	else { overlay_bound_min[2] += slope_width; }
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector landing_size = overlay_ground_size;
	if (point_dir[0] == 1) { landing_size[0] = overlay_ground_size[0] - 2 * slope_width; }
	else { landing_size[2] = overlay_ground_size[2] - 2 * slope_width; }
	oAddLanding(static_cast<float>(slope_end_depth), point_dir, start_coord, landing_size,
		spacing_x, spacing_z, out_res, out_data, out_flags);

	if (point_dir[0] == 1) { overlay_bound_min[0] += landing_size[0]; }
	else { overlay_bound_min[2] += landing_size[2]; }
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	oAddSlope(-slope_ratio, slope_end_depth, terrain_start_h, point_dir, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildStairs(
	const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float stair_spacing = mOverBlendParams[eOverStairSpacing];
	const float stair_increase = mOverBlendParams[eOverStairIncease];
	double terrain_start_h = mOverBlendParams[eOverPlateauHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);
	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	// additional 0.5 * width for landing
	float stair_width = 0;
	if (point_dir[0] == 1) { stair_width = overlay_ground_size[0] / 2.5; }
	else { stair_width = overlay_ground_size[2] / 2.5; }

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector box_size = overlay_ground_size;
	if (point_dir[0] == 1) { box_size[0] = stair_width; }
	else { box_size[2] = stair_width; }

	oAddStair(	// -1000 represents very small value
		terrain_start_h, -1000, point_dir, stair_spacing, stair_increase, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);

	double stair_end_depth = 0;
	if (point_dir[1] == 1)
	{
		stair_end_depth = terrain_start_h + (static_cast<int>(box_size[2] / stair_spacing)) * stair_increase;
	}
	else { stair_end_depth = terrain_start_h + (static_cast<int>(box_size[0] / stair_spacing)) * stair_increase; }


	if (point_dir[0] == 1) { overlay_bound_min[0] += stair_width; }
	else { overlay_bound_min[2] += stair_width; }
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector landing_size = overlay_ground_size;
	if (point_dir[0] == 1) { landing_size[0] = overlay_ground_size[0] - 2 * stair_width; }
	else { landing_size[2] = overlay_ground_size[2] - 2 * stair_width; }
	oAddLanding(static_cast<float>(stair_end_depth), point_dir, start_coord, landing_size,
		spacing_x, spacing_z, out_res, out_data, out_flags);

	if (point_dir[0] == 1) { overlay_bound_min[0] += landing_size[0]; }
	else { overlay_bound_min[2] += landing_size[2]; }
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);

	oAddStair(
		stair_end_depth, terrain_start_h, point_dir, stair_spacing, -stair_increase, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildSlopeStair(
	const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float slope_ratio = mOverBlendParams[eOverSlopeRatio];
	const float stair_spacing = mOverBlendParams[eOverStairSpacing];
	float stair_increase = mOverBlendParams[eOverStairIncease];
	double terrain_start_h = mOverBlendParams[eOverPlateauHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);
	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	// adjust stair and slope width by adjusting stair/slope height, to save the landing space
	// stair_width + slope_width = size * 2.0 / 2.5; another 0.5 for landing.
	float stair_width = 0;
	float slope_width = 0;
	if (point_dir[0] == 1)
	{
		float slope_height = (overlay_ground_size[0] * 2.0 / 2.5) / (
			std::abs(stair_spacing / stair_increase) + std::abs(1 / slope_ratio));
		float stair_height = slope_height;
		stair_width = std::abs(stair_spacing * stair_height / stair_increase);
		slope_width = std::abs(slope_height / slope_ratio);
	}
	else	// point_dir[1] == 1
	{
		float slope_height = (overlay_ground_size[2] * 2.0 / 2.5) / (
			std::abs(stair_spacing / stair_increase) + std::abs(1 / slope_ratio));
		float stair_height = slope_height;
		stair_width = std::abs(stair_spacing * stair_height / stair_increase);
		slope_width = std::abs(slope_height / slope_ratio);
	}

	if (!(slope_ratio * stair_increase < 0))
	{
		stair_increase *= -1;	// ensure the start-end height is zero.
	}

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector slope_box_size = overlay_ground_size;
	if (point_dir[0] == 1) { slope_box_size[0] = slope_width; }
	else { slope_box_size[2] = slope_width; }

	oAddSlope(
		slope_ratio, terrain_start_h, -1000, point_dir, overlay_bound_min,
		start_coord, slope_box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
	double slope_end_depth = 0;
	if (point_dir[1] == 1) { slope_end_depth = terrain_start_h + slope_box_size[2] * slope_ratio; }
	else { slope_end_depth = terrain_start_h + slope_box_size[0] * slope_ratio; }

	if (point_dir[0] == 1) { overlay_bound_min[0] += slope_width; }
	else { overlay_bound_min[2] += slope_width; }
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector landing_size = overlay_ground_size;
	if (point_dir[0] == 1) { landing_size[0] = overlay_ground_size[0] - slope_width - stair_width; }
	else { landing_size[2] = overlay_ground_size[2] - slope_width - stair_width; }
	oAddLanding(static_cast<float>(slope_end_depth), point_dir, start_coord, landing_size,
		spacing_x, spacing_z, out_res, out_data, out_flags);

	if (point_dir[0] == 1) { overlay_bound_min[0] += landing_size[0]; }
	else { overlay_bound_min[2] += landing_size[2]; }
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector stair_box_size = overlay_ground_size;
	if (point_dir[0] == 1) { stair_box_size[0] = stair_width; }
	else { stair_box_size[2] = stair_width; }
	oAddStair(
		slope_end_depth, terrain_start_h, point_dir, stair_spacing, stair_increase, overlay_bound_min,
		start_coord, stair_box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildStairSlope(
	const Eigen::Vector2i& point_dir, const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	float slope_ratio = mOverBlendParams[eOverSlopeRatio];
	const float stair_spacing = mOverBlendParams[eOverStairSpacing];
	const float stair_increase = mOverBlendParams[eOverStairIncease];
	double terrain_start_h = mOverBlendParams[eOverPlateauHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);
	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	// adjust stair and slope width by adjusting stair/slope height, to save the landing space
	// stair_width + slope_width = size * 2.0 / 2.5; another 0.5 for landing.
	float stair_width = 0;
	float slope_width = 0;
	if (point_dir[0] == 1)
	{
		float slope_height = (overlay_ground_size[0] * 2.0 / 2.5) / (
			std::abs(stair_spacing / stair_increase) + std::abs(1 / slope_ratio));
		float stair_height = slope_height;
		stair_width = std::abs(stair_spacing * stair_height / stair_increase);
		slope_width = std::abs(slope_height / slope_ratio);
	}
	else
	{
		float slope_height = (overlay_ground_size[2] * 2.0 / 2.5) / (
			std::abs(stair_spacing / stair_increase) + std::abs(1 / slope_ratio));
		float stair_height = slope_height;
		stair_width = std::abs(stair_spacing * stair_height / stair_increase);
		slope_width = std::abs(slope_height / slope_ratio);
	}


	if (!(slope_ratio * stair_increase < 0))
	{
		slope_ratio *= -1;	// ensure the start-end height is zero.
	}

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector stair_box_size = overlay_ground_size;
	if (point_dir[0] == 1) { stair_box_size[0] = stair_width; }
	else { stair_box_size[2] = stair_width; }

	oAddStair(
		terrain_start_h, -1000, point_dir, stair_spacing, stair_increase, overlay_bound_min,
		start_coord, stair_box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
	double stair_end_depth = 0;
	if (point_dir[1] == 1)
	{
		stair_end_depth = terrain_start_h + (static_cast<int>(stair_box_size[2] / stair_spacing)) * stair_increase;
	}
	else { stair_end_depth = terrain_start_h + (static_cast<int>(stair_box_size[0] / stair_spacing)) * stair_increase; }


	if (point_dir[0] == 1) { overlay_bound_min[0] += stair_width; }
	else { overlay_bound_min[2] += stair_width; }
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector landing_size = overlay_ground_size;
	if (point_dir[0] == 1) { landing_size[0] = overlay_ground_size[0] - slope_width - stair_width; }
	else { landing_size[2] = overlay_ground_size[2] - slope_width - stair_width; }
	oAddLanding(static_cast<float>(stair_end_depth), point_dir, start_coord, landing_size,
		spacing_x, spacing_z, out_res, out_data, out_flags);

	if (point_dir[0] == 1) { overlay_bound_min[0] += landing_size[0]; }
	else { overlay_bound_min[2] += landing_size[2]; }
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector slope_box_size = overlay_ground_size;
	if (point_dir[0] == 1) { slope_box_size[0] = slope_width; }
	else { slope_box_size[2] = slope_width; }
	oAddSlope(
		slope_ratio, stair_end_depth, terrain_start_h, point_dir, overlay_bound_min,
		start_coord, slope_box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}


double cOverlayTerrainGen3D::oAddBox(
	const double start_height, const Eigen::Vector2i& point_dir, const float spacing, const float depth, const float length,
	const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, 
	double spacing_z, const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags
)
{
	// add a gap unit, the height start and end with zero	
	int res_x = CalcResX(size[0], spacing_x);
	int res_z = CalcResX(size[2], spacing_z);
	int num_verts = res_x * res_z;

	for (int j = 0; j < res_z; ++j)
	{
		double z = (j / (res_z - 1.0)) * size[2];
		size_t coord_z = j + start_coord[1];
		if (!(coord_z < out_res[1])) { break; }
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;

			float x = (i / (res_x - 1.0)) * size[0];
			double h = start_height;

			if ( point_dir[0] == 1 )	// regard as [0, 1], z direction
			{
				if (spacing < x && std::abs(z - 0.5 * size[2]) < length) { h += depth; }
			}
			else	// regard as [1, 0], x direction
			{
				if (spacing < z && std::abs(x - 0.5 * size[0]) < length) { h += depth; }
			}

			if (coord_x < out_res[0])
			{
				int curr_flags = 1 << eVertFlagEnableTex;
				out_data[idx] = h;
				out_flags[idx] = curr_flags;
			}
		}
	}
	double width_added = 0;
	if (point_dir[1] == 1) { width_added = size[2]; }
	else { width_added = size[0]; }
	return width_added;
}

void cOverlayTerrainGen3D::oAddSlope(
	const double slope, const double start_height, const double end_height, 
	const Eigen::Vector2i& point_dir, const tVector& origin,
	const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
	const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags)
{
	int res_x = CalcResX(size[0], spacing_x);
	int res_z = CalcResX(size[2], spacing_z);
	int num_verts = res_x * res_z;
	double h = 0;

	for (int j = 0; j < res_z; ++j)
	{
		double z = (j / (res_z - 1.0)) * size[2];	// origin[2] + 
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;

			if (point_dir[1] == 1)
			{
				h = start_height + slope * z;
			}
			else
			{
				double x = (i / (res_x - 1.0)) * size[0];	// origin[0] + 
				h = start_height + slope * x;
			}

			int curr_flags = 1 << eVertFlagEnableTex;
			out_data[idx] = cMathUtil::Sign(h) * std::min(std::abs(h), std::abs(end_height));
			out_flags[idx] = curr_flags;
		}
	}
}

void cOverlayTerrainGen3D::oAddStair(
	const double start_height, const double end_height, const Eigen::Vector2i& point_dir, 
	const float stair_space, const float stair_increase,
	const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, 
	double spacing_x, double spacing_z,
	const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags)
{
	int res_x = CalcResX(size[0], spacing_x);
	int res_z = CalcResX(size[2], spacing_z);
	int num_verts = res_x * res_z;
	double h = 0;

	for (int j = 0; j < res_z; ++j)
	{
		double z = (j / (res_z - 1.0)) * size[2];	// origin[2] + 
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;

			if (point_dir[1] == 1)
			{
				int step_num = static_cast<int>(z / stair_space);
				h = start_height + step_num * stair_increase;
			}
			else
			{
				float x = (i / (res_x - 1.0)) * size[0];	// origin[0] + 
				int step_num = static_cast<int>(x / stair_space);
				h = start_height + step_num * stair_increase;
			}

			int curr_flags = 1 << eVertFlagEnableTex;
			h = cMathUtil::Sign(h) * std::min(std::abs(h), std::abs(end_height));
			out_data[idx] = h;
			out_flags[idx] = curr_flags;
		}
	}
}

void cOverlayTerrainGen3D::oAddLanding(
	const float depth, const Eigen::Vector2i& point_dir, const Eigen::Vector2i& start_coord, const tVector& size, 
	double spacing_x, double spacing_z, const Eigen::Vector2i& out_res, std::vector<float>& out_data, 
	std::vector<int>& out_flags
)
{
	// add a landing to connect slopes/stairs	
	int res_x = CalcResX(size[0], spacing_x);
	int res_z = CalcResX(size[2], spacing_z);
	int num_verts = res_x * res_z;

	for (int j = 0; j < res_z; ++j)
	{
		// CalcResX results additional "1", which will out of bound for last terrain part
		size_t coord_z = j + start_coord[1];	
		if (!(coord_z < out_res[1])) { break; }
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;

			// to fix error "index out of range"
			if (coord_x < out_res[0])
			{
				int curr_flags = 1 << eVertFlagEnableTex;
				out_data[idx] = depth;
				out_flags[idx] = curr_flags;
			}
		}
	}
}

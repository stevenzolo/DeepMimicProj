#include "OverlayTerrainGen3D.h"

Eigen::VectorXd cOverlayTerrainGen3D::mOverBlendParams = Eigen::VectorXd::Zero(eOverParamsMax);

const cOverlayTerrainGen3D::tParamDef cOverlayTerrainGen3D::gParamDefs[] =
{
	// the minimum value should be larger than cTerrainGen3D::gVertSpacing(0.2)
	{ "BoxSpacing", 0.6 },	// space between the plane and other terrains
	{ "NarrowWidth", 0.6 },	// allow character cross over, less than max step length of man (~0.8)
	{ "GapDepth", -1.0},	// depth for gap/pit
	{ "WallHeight", 1.0},	// used in wall/landing, allow character climb up
	{ "BeamHeight", 0.3},	// used in beam, allow character jump up
	{ "BeamSpacing", 0.6},	// narrow, < 2 * foot width
	{ "SlopeRatio", 0.5},	// ratio of height/width, positive slope means up-then-down, while neg for down-up
	{ "SlopeHeight", 7.0},	
	{ "StairSpacing", 0.6},	
	{ "StairIncease", -0.3},	// positive increase means up-then-down, while neg for down-up
	{ "StairHeight", 3.0}
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

void cOverlayTerrainGen3D::LoadParams(const Json::Value& root)
{
	cOverlayTerrainGen3D::GetDefaultParams(mOverBlendParams);
	for (int i = 0; i < eOverParamsMax; ++i)
	{
		const std::string& name = gParamDefs[i].mName;
		if (!root[name].isNull())
		{
			double val = root[name].asDouble();
			mOverBlendParams[i] = val;
		}
	}
}

void cOverlayTerrainGen3D::oBuildGaps(
	const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float gap_spacing = mOverBlendParams[eOverBoxSpacing];
	const float gap_width = mOverBlendParams[eOverNarrowWidth];
	const float depth = mOverBlendParams[eOverGapDepth];
	
	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;
	const float gap_length = overlay_ground_size[2];	// gap across the whole overlay area
	const double box_width_bound = overlay_ground_size[0];

	double total_w = 0;
	tVector box_size = overlay_ground_size;
	box_size[0] = gap_spacing + gap_width;
	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	while (total_w < box_width_bound)
	{
		start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
		start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
		double width_added = oAddBox(gap_spacing, depth, gap_length, overlay_bound_min, start_coord,
			box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
		total_w += width_added;
		overlay_bound_min[0] += width_added;
	}
}

void cOverlayTerrainGen3D::oBuildPit(
	const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float pit_spacing = mOverBlendParams[eOverBoxSpacing];
	const float depth = mOverBlendParams[eOverGapDepth];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	double width_added = oAddBox(pit_spacing, depth, overlay_ground_size[2], overlay_bound_min, start_coord,
		overlay_ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildWall(
	const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float wall_spacing = mOverBlendParams[eOverBoxSpacing];
	const float height = mOverBlendParams[eOverWallHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	double width_added = oAddBox(wall_spacing, height, overlay_ground_size[2], overlay_bound_min, start_coord,
		overlay_ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildBeam(
	const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float box_spacing = mOverBlendParams[eOverBoxSpacing];
	const float height = mOverBlendParams[eOverBeamHeight];
	const float beam_space = mOverBlendParams[eOverBeamSpacing];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	double width_added = oAddBox(box_spacing, height, beam_space, overlay_bound_min, start_coord,
		overlay_ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildBars(
	const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float bar_spacing = mOverBlendParams[eOverBoxSpacing];
	const float bar_width = mOverBlendParams[eOverNarrowWidth];
	const float height = mOverBlendParams[eOverBeamHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);

	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;
	const float bar_length = overlay_ground_size[2];	// gap across the whole overlay area
	const double box_width_bound = overlay_ground_size[0];

	double total_w = 0;
	tVector box_size = overlay_ground_size;
	box_size[0] = bar_spacing + bar_width;
	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	while (total_w < box_width_bound)
	{
		start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
		start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
		double width_added = oAddBox(bar_spacing, height, bar_length, overlay_bound_min, start_coord,
			box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
		total_w += width_added;
		overlay_bound_min[0] += width_added;
	}
}

void cOverlayTerrainGen3D::oBuildSlopes(
	const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float slope_ratio = mOverBlendParams[eOverSlopeRatio];
	const float slope_height = mOverBlendParams[eOverSlopeHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);
	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	float slope_width = std::abs(slope_height / slope_ratio);
	if (!(overlay_ground_size[0] > slope_width * 2.5))		// additional 0.5 * width for landing 
	{
		// Decrease slope height to save width for landing
		slope_width = overlay_ground_size[0] / 2.5;
	}

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector box_size = overlay_ground_size;
	box_size[0] = slope_width;

	double slope_end_depth = oAddSlope(slope_ratio, 0, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);

	overlay_bound_min[0] += slope_width;
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector landing_size = overlay_ground_size;
	landing_size[0] = overlay_ground_size[0] - 2 * slope_width;
	oAddLanding(static_cast<float>(slope_end_depth), start_coord, landing_size,
		spacing_x, spacing_z, out_res, out_data, out_flags);

	overlay_bound_min[0] += landing_size[0];
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	double _ = oAddSlope(-slope_ratio, slope_end_depth, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildStairs(
	const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float stair_spacing = mOverBlendParams[eOverStairSpacing];
	const float stair_increase = mOverBlendParams[eOverStairIncease];
	const float stair_height = mOverBlendParams[eOverStairHeight];

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);
	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	float stair_width = std::abs(stair_spacing * stair_height / stair_increase);
	if (!(overlay_ground_size[0] > stair_width * 2.5))		// additional 0.5 * width for landing 
	{
		// Decrease slope height to save width for landing
		stair_width = overlay_ground_size[0] / 2.5;
	}

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector box_size = overlay_ground_size;
	box_size[0] = stair_width;

	double stair_end_depth = oAddStair(0, stair_spacing, stair_increase, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);

	overlay_bound_min[0] += stair_width;
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector landing_size = overlay_ground_size;
	landing_size[0] = overlay_ground_size[0] - 2 * stair_width;
	oAddLanding(static_cast<float>(stair_end_depth), start_coord, landing_size,
		spacing_x, spacing_z, out_res, out_data, out_flags);

	overlay_bound_min[0] += landing_size[0];
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	double _ = oAddStair(stair_end_depth, stair_spacing, -stair_increase, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildSlopeStair(
	const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const float slope_ratio = mOverBlendParams[eOverSlopeRatio];
	float slope_height = mOverBlendParams[eOverSlopeHeight];
	const float stair_spacing = mOverBlendParams[eOverStairSpacing];
	float stair_increase = mOverBlendParams[eOverStairIncease];
	float stair_height = mOverBlendParams[eOverStairHeight];
	stair_height = (slope_height != stair_height) ? slope_height : stair_height;

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);
	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	float stair_width = std::abs(stair_spacing * stair_height / stair_increase);	// eq.(1)
	float slope_width = std::abs(slope_height / slope_ratio);	// eq.(2)
	if (!(overlay_ground_size[0] > (stair_width * 1.5 + slope_width)))		// additional 0.5 * width for landing 
	{
		// adjust stair and slope width by adjusting stair/slope height, to save the landing space
		// eq.£¨1£©+ eq.(2) = size * 2.0 / 2.5; another 0.5 for landing.
		slope_height = (overlay_ground_size[0] * 2.0 / 2.5) / (
			std::abs(stair_spacing / stair_increase) + std::abs(1 / slope_ratio));
		stair_height = slope_height;
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
	tVector box_size = overlay_ground_size;
	box_size[0] = slope_width;

	double slope_end_depth = oAddSlope(slope_ratio, 0, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);

	overlay_bound_min[0] += slope_width;
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector landing_size = overlay_ground_size;
	landing_size[0] = overlay_ground_size[0] - slope_width - stair_width;
	oAddLanding(static_cast<float>(slope_end_depth), start_coord, landing_size,
		spacing_x, spacing_z, out_res, out_data, out_flags);

	overlay_bound_min[0] += landing_size[0];
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	double _ = oAddStair(slope_end_depth, stair_spacing, stair_increase, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cOverlayTerrainGen3D::oBuildStairSlope(
	const tVector& global_bound_min, const tVector& global_bound_max,
	tVector& overlay_bound_min, tVector& overlay_bound_max,
	double spacing_x, double spacing_z, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	float slope_ratio = mOverBlendParams[eOverSlopeRatio];
	float slope_height = mOverBlendParams[eOverSlopeHeight];
	const float stair_spacing = mOverBlendParams[eOverStairSpacing];
	const float stair_increase = mOverBlendParams[eOverStairIncease];
	float stair_height = mOverBlendParams[eOverStairHeight];
	slope_height = (slope_height != stair_height) ? stair_height : slope_height;

	tVector global_ground_size = global_bound_max - global_bound_min;
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(global_ground_size[0], spacing_x);
	out_res[1] = CalcResZ(global_ground_size[2], spacing_z);
	tVector overlay_ground_size = overlay_bound_max - overlay_bound_min;

	float stair_width = std::abs(stair_spacing * stair_height / stair_increase);	// eq.£¨1£©
	float slope_width = std::abs(slope_height / slope_ratio);	// eq.£¨2£©
	if (!(overlay_ground_size[0] > (slope_width * 1.5 + stair_width)))		// additional 0.5 * width for landing 
	{
		// adjust stair and slope width by adjusting stair/slope height, to save the landing space
		// eq.£¨1£©+ eq.(2) = size * 2.0 / 2.5; another 0.5 for landing.
		slope_height = (overlay_ground_size[0] * 2.0 / 2.5) / (
			std::abs(stair_spacing / stair_increase) + std::abs(1 / slope_ratio));
		stair_height = slope_height;
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
	tVector box_size = overlay_ground_size;
	box_size[0] = stair_width;

	double end_stair_depth = oAddStair(0, stair_spacing, stair_increase, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);

	overlay_bound_min[0] += stair_width;
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	tVector landing_size = overlay_ground_size;
	landing_size[0] = overlay_ground_size[0] - slope_width - stair_width;
	oAddLanding(static_cast<float>(end_stair_depth), start_coord, landing_size,
		spacing_x, spacing_z, out_res, out_data, out_flags);

	overlay_bound_min[0] += landing_size[0];
	start_coord[0] = CalcResX(std::abs(overlay_bound_min[0] - global_bound_min[0]), spacing_x);
	start_coord[1] = CalcResZ(std::abs(overlay_bound_min[2] - global_bound_min[2]), spacing_z);
	double _ = oAddSlope(slope_ratio, end_stair_depth, overlay_bound_min,
		start_coord, box_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}


double cOverlayTerrainGen3D::oAddBox(
	const float spacing, const float depth, const float length, const tVector& origin,
	const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
	const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags
)
{
	// add a gap unit, the height start and end with zero	
	int res_x = CalcResX(size[0], spacing_x);
	int res_z = CalcResX(size[2], spacing_z);
	int num_verts = res_x * res_z;

	for (int j = 0; j < res_z; ++j)
	{
		double z = (j / (res_z - 1.0)) * size[2];	// origin[2] + 
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;

			float x = (i / (res_x - 1.0)) * size[0];	// origin[0] + 

			double h = 0;
			if (spacing < x)
			{
				if (std::abs(z - 0.5 * size[2]) < length)
				{
					h = depth;
				}
			}

			int curr_flags = 1 << eVertFlagEnableTex;
			out_data[idx] = h;
			out_flags[idx] = curr_flags;
		}
	}
	return size[0];		// width added
}

double cOverlayTerrainGen3D::oAddSlope(
	const double slope, const double start_height, const tVector& origin,
	const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
	const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags)
{
	int res_x = CalcResX(size[0], spacing_x);
	int res_z = CalcResX(size[2], spacing_z);
	int num_verts = res_x * res_z;

	for (int j = 0; j < res_z; ++j)
	{
		double z = (j / (res_z - 1.0)) * size[2];	// origin[2] + 
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;

			double x = (i / (res_x - 1.0)) * size[0];	// origin[0] + 
			double h = start_height + slope * x;

			int curr_flags = 1 << eVertFlagEnableTex;
			out_data[idx] = h;
			out_flags[idx] = curr_flags;
		}
	}

	double max_h = start_height + size[0] * slope;
	return max_h;
}

double cOverlayTerrainGen3D::oAddStair(
	const double start_height, const float stair_space, const float stair_increase, const tVector& origin,
	const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
	const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags)
{
	int res_x = CalcResX(size[0], spacing_x);
	int res_z = CalcResX(size[2], spacing_z);
	int num_verts = res_x * res_z;

	for (int j = 0; j < res_z; ++j)
	{
		double z = (j / (res_z - 1.0)) * size[2];	// origin[2] + 
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;

			float x = (i / (res_x - 1.0)) * size[0];	// origin[0] + 

			int step_num = static_cast<int>(x / stair_space);
			double h = start_height + step_num * stair_increase;

			int curr_flags = 1 << eVertFlagEnableTex;
			out_data[idx] = h;
			out_flags[idx] = curr_flags;
		}
	}

	double max_h = start_height + (static_cast<int>(size[0] / stair_space)) * stair_increase;
	return max_h;
}

void cOverlayTerrainGen3D::oAddLanding(
	const float depth, const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
	const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags
)
{
	// add a landing to connect slopes/stairs	
	int res_x = CalcResX(size[0], spacing_x);
	int res_z = CalcResX(size[2], spacing_z);
	int num_verts = res_x * res_z;

	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;

			int curr_flags = 1 << eVertFlagEnableTex;
			out_data[idx] = depth;
			out_flags[idx] = curr_flags;
		}
	}
}

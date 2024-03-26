#include "Combine3dTerrains.h"

Eigen::VectorXd cCombine3dTerrains::mCombBlendParams = Eigen::VectorXd::Zero(eCombParamsMax);

const cCombine3dTerrains::tParamDef cCombine3dTerrains::gParamDefs[] =
{
	// to adapt
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

void cCombine3dTerrains::GetDefaultParams(Eigen::VectorXd& out_params)
{
	// revised eParamsMax -> eCombParamsMax to load default parameters of overlay terrains. @Yan
	out_params = Eigen::VectorXd::Zero(eCombParamsMax);
	assert(sizeof(gParamDefs) / sizeof(gParamDefs[0]) == eCombParamsMax);
	for (int i = 0; i < eCombParamsMax; ++i)
	{
		out_params[i] = gParamDefs[i].mDefaultVal;
	}
}

bool cCombine3dTerrains::ParseParamsJson(const Json::Value& json, Eigen::VectorXd& out_params)
{
	GetDefaultParams(mCombBlendParams);
	for (int i = 0; i < eCombParamsMax; ++i)
	{
		const std::string& name = gParamDefs[i].mName;
		if (!json[name].isNull())
		{
			double val = json[name].asDouble();
			mCombBlendParams[i] = val;
		}
	}

	cTerrainGen3D::LoadParams(json, out_params);
	return true;
}

cCombine3dTerrains::tCombTerrainFunc cCombine3dTerrains::GetTerrainFunc(cGround::eType terrain_type)
{
	switch (terrain_type)
	{
	case cGround::eTypeCombined3DTerrains:
		return BuildStairRamp;
	default:
		printf("Unsupported ground var3d type.\n");
		assert(false);
		return BuildStairRamp;
	}
}

void cCombine3dTerrains::BuildStairRamp(const tVector& origin, const tVector& ground_size, double spacing_x, double spacing_z
	, const Eigen::VectorXd& params, cRand& rand,
	std::vector<float>& out_data, std::vector<int>& out_flags)
{
	double spacing_min = params[eParamsStepSpacingMin];
	double spacing_max = params[eParamsStepSpacingMax];
	double step_h_min = params[eParamsStepHeightMin];
	double step_h_max = params[eParamsStepHeightMax];

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(ground_size[0], spacing_x);
	out_res[1] = CalcResZ(ground_size[2], spacing_z);

	double stair_start_h = (std::floor(origin[0])) * 0.1;
	tVector flat_ground_size = ground_size;
	flat_ground_size[0] = std::floor(ground_size[0] * 0.1);
	AddFlat(stair_start_h, origin, start_coord, flat_ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);

	tVector stair_ground_size = ground_size;
	stair_ground_size[0] = std::floor(ground_size[0] * 0.35);
	tVector stair_origin = origin;
	stair_origin[0] += std::floor(ground_size[0] * 0.1);
	Eigen::Vector2i stair_start_coord = start_coord;
	stair_start_coord[0] = CalcResX(std::floor(ground_size[0] * 0.1), spacing_x);
	double stair_end_h = (std::floor(origin[0] + ground_size[0] * 0.5)) * 0.1;
	AddStairs(stair_start_h, stair_end_h, stair_origin, stair_start_coord, stair_ground_size, spacing_x, spacing_z, spacing_min, spacing_max,
		step_h_min, step_h_max, out_res, rand, out_data, out_flags);

	tVector flat2_origin = origin;
	flat2_origin[0] += std::floor(ground_size[0] * 0.45);
	Eigen::Vector2i flat2_start_coord = start_coord;
	flat2_start_coord[0] = CalcResX(std::floor(ground_size[0] * 0.45), spacing_x);
	tVector flat2_ground_size = ground_size;
	flat2_ground_size[0] = std::floor(ground_size[0] * 0.1);
	AddFlat(stair_end_h, flat2_origin, flat2_start_coord, flat2_ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);

	tVector ramp_ground_size = ground_size;
	ramp_ground_size[0] = std::floor(ground_size[0] * 0.35);
	tVector ramp_origin = origin;
	ramp_origin[0] += std::floor(ground_size[0] * 0.55);
	Eigen::Vector2i ramp_start_coord = start_coord;
	ramp_start_coord[0] = CalcResX(std::floor(ground_size[0] * 0.55), spacing_x);
	double slope = params[eParamsSlope];
	double ramp_start_h = stair_end_h;
	double ramp_end_h = (std::floor(origin[0] + ground_size[0])) * 0.1;
	AddRamp(ramp_start_h, ramp_end_h, ramp_origin, ramp_start_coord, ramp_ground_size, spacing_x, spacing_z, slope, out_res, out_data, out_flags);

	tVector flat3_origin = origin;
	flat3_origin[0] += std::floor(ground_size[0] * 0.9);
	Eigen::Vector2i flat3_start_coord = start_coord;
	flat3_start_coord[0] = CalcResX(std::floor(ground_size[0] * 0.9), spacing_x);
	tVector flat3_ground_size = ground_size;
	flat3_ground_size[0] = (ground_size[0] - std::floor(ground_size[0] * 0.9));
	return AddFlat(ramp_end_h, flat3_origin, flat3_start_coord, flat3_ground_size, spacing_x, spacing_z, out_res, out_data, out_flags);
}

void cCombine3dTerrains::AddFlat(const double h, const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
	const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags)
{
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
			//double h = 0;

			if (idx < out_res[0] * (static_cast<int>(coord_z) + 1))		// to fix error "index out of range"
			{
				int curr_flags = 1 << eVertFlagEnableTex;
				out_data[idx] = static_cast<float>(h);
				out_flags[idx] = curr_flags;
			}
		}
	}
}

void cCombine3dTerrains::AddRamp(const double start_h, const double end_h, const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z,
	double slope, const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const double pad = 1;

	int res_x = CalcResX(size[0], spacing_x);
	int res_z = CalcResX(size[2], spacing_z);
	int num_verts = res_x * res_z;

	double nslope = 1.01 * (end_h - start_h) / size[0];
	double h = 0;

	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			int curr_flags = 1 << eVertFlagEnableTex;

			tVector pos = origin;
			pos[0] += (i / (res_x - 1.0)) * size[0];
			pos[2] += (j / (res_z - 1.0)) * size[2];

			double x = pos[0];
			h = cMathUtil::Clamp((x - origin[0]) * nslope + start_h, start_h, end_h);

			out_data[idx] = static_cast<float>(h);
			out_flags[idx] = curr_flags;
		}
	}
}

void cCombine3dTerrains::AddStairs(const double start_h, const double end_h, const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double spacing_x, double spacing_z
	, double spacing_min, double spacing_max, double step_h_min, double step_h_max,
	const Eigen::Vector2i& out_res, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const double pad = 1;
	double h = start_h;

	int res_x = CalcResX(size[0], spacing_x);
	int res_z = CalcResX(size[2], spacing_z);
	int num_verts = res_x * res_z;

	double randomness = rand.RandDouble();

	for (int j = 0; j < res_z; ++j)
	{
		//Check the new step and reset the height if we are done with the last step
		double z = origin[2] + (j / (res_z - 1.0)) * size[2];
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			double x = origin[0] + (i / (res_x - 1.0)) * size[0];

			// add stairs, i % num decide the width of the stairs
			if (i % 8 == 0) {	
				h = start_h + static_cast<int>(i / 8) * 0.15;
			}
			h = h > end_h ? end_h : h;
			int curr_flags = 1 << eVertFlagEnableTex;
			out_data[idx] = h;
			out_flags[idx] = curr_flags;
		}
	}
}


#include "DeepMimicCharController.h"
#include "sim/SimCharacter.h"
#include <iostream>
#include <ctime>
#include "util/json/json.h"
#include "util/FileUtil.h"
#include "util/JsonUtil.h"

const int gValLogSize = 50;
const std::string gViewDistMinKey = "ViewDistMin";
const std::string gViewDistMaxKey = "ViewDistMax";

cDeepMimicCharController::cDeepMimicCharController() : cCharController()
{
	mTime = 0;
	mPosDim = 0;
	SetViewDistMin(-0.5);
	SetViewDistMax(10);

	mPrevActionTime = mTime;
	mPrevActionCOM.setZero();

	mValLog.Reserve(gValLogSize);
}

cDeepMimicCharController::~cDeepMimicCharController()
{
}

void cDeepMimicCharController::Init(cSimCharacter* character, const std::string& param_file)
{
	cCharController::Init(character);
	LoadParams(param_file);
	ResetParams();

	mPosDim = GetPosDim();
	InitResources();

	mValid = true;
}

void cDeepMimicCharController::Reset()
{
	cCharController::Reset();
	ResetParams();
	NewActionUpdate();
}

void cDeepMimicCharController::Clear()
{
	cCharController::Clear();
	ResetParams();
}

void cDeepMimicCharController::Update(double time_step)
{
	cCharController::Update(time_step);
	UpdateCalcTau(time_step, mTau);
	UpdateApplyTau(mTau);
}

void cDeepMimicCharController::PostUpdate(double timestep)
{
	mNeedNewAction = CheckNeedNewAction(timestep);
	if (mNeedNewAction)
	{
		NewActionUpdate();
	}
}

void cDeepMimicCharController::UpdateCalcTau(double timestep, Eigen::VectorXd& out_tau)
{
	mTime += timestep;
	if (mNeedNewAction)
	{
		HandleNewAction();
	}
}

void cDeepMimicCharController::UpdateApplyTau(const Eigen::VectorXd& tau)
{
	mTau = tau;
	mChar->ApplyControlForces(tau);
}

void cDeepMimicCharController::SetGround(std::shared_ptr<cGround> ground)
{
	mGround = ground;
}

void cDeepMimicCharController::SetViewDistMin(double dist)
{
	mViewDistMin = dist;
}

void cDeepMimicCharController::SetViewDistMax(double dist)
{
	mViewDistMax = dist;
}

double cDeepMimicCharController::GetViewDistMin() const
{
	return mViewDistMin;
}

double cDeepMimicCharController::GetViewDistMax() const
{
	return mViewDistMax;
}

void cDeepMimicCharController::GetViewBound(tVector& out_min, tVector& out_max) const
{
	tVector origin = mChar->GetRootPos();
	double max_len = mViewDistMax;
	out_min = origin - tVector(max_len, 0, max_len, 0);
	out_max = origin + tVector(max_len, 0, max_len, 0);
}

double cDeepMimicCharController::GetPrevActionTime() const
{
	return mPrevActionTime;
}

const tVector& cDeepMimicCharController::GetPrevActionCOM() const
{
	return mPrevActionCOM;
}

double cDeepMimicCharController::GetTime() const
{
	return mTime;
}

const Eigen::VectorXd& cDeepMimicCharController::GetTau() const
{
	return mTau;
}

const cCircularBuffer<double>& cDeepMimicCharController::GetValLog() const
{
	return mValLog;
}

void cDeepMimicCharController::LogVal(double val)
{
	mValLog.Add(val);
}

bool cDeepMimicCharController::NeedNewAction() const
{
	return mNeedNewAction;
}

void cDeepMimicCharController::ApplyAction(const Eigen::VectorXd& action)
{
	assert(action.size() == GetActionSize());
	mAction = action;
	PostProcessAction(mAction);
}

void cDeepMimicCharController::RecordState(Eigen::VectorXd& out_state)
{
	int state_size = GetStateSize();
	// fill with nans to make sure we don't forget to set anything
	out_state = std::numeric_limits<double>::quiet_NaN() * Eigen::VectorXd::Ones(state_size);

	Eigen::VectorXd ground;
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
    // added by Yifan
	Eigen::VectorXd map;

	BuildStatePose(pose);
	BuildStateVel(vel);
    // added by Yifan
	BuildHeightMap(map);

	int pose_offset = GetStatePoseOffset();
	int pose_size = GetStatePoseSize();
	int map_offset = GetStateMapOffset();
	int vel_offset = GetStateVelOffset();
	int vel_size = GetStateVelSize();
	int map_size = GetStateMapSize();

	out_state.segment(pose_offset, pose_size) = pose;
	out_state.segment(vel_offset, vel_size) = vel;
	out_state.segment(map_offset, map_size) = map;

    // added by Yifan
//	printf("state size in C: %d\n", state_size);
//	for(int i = 0; i < state_size; i++)
//	{
//	    printf(" state %d: %.5f ", i, out_state[i]);
//	}
}

eActionSpace cDeepMimicCharController::GetActionSpace() const
{
	return eActionSpaceContinuous;
}

void cDeepMimicCharController::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = mAction;
}

int cDeepMimicCharController::GetStateSize() const
{
	int state_size = 0;
	state_size += GetStatePoseSize();
	state_size += GetStateVelSize();
    // added by Yifan
	state_size += GetStateMapSize();
	return state_size;
}

double cDeepMimicCharController::GetRewardMin() const
{
	return 0;
}

double cDeepMimicCharController::GetRewardMax() const
{
	return 1;
}

bool cDeepMimicCharController::ParseParams(const Json::Value& json)
{
	bool succ = cCharController::ParseParams(json);

	mViewDistMin = json.get(gViewDistMinKey, mViewDistMin).asDouble();
	mViewDistMax = json.get(gViewDistMaxKey, mViewDistMax).asDouble();
	
	return succ;
}

void cDeepMimicCharController::ResetParams()
{
	mTime = 0;
	mNeedNewAction = true;
	mTau.setZero();
	mValLog.Clear();

	mPrevActionTime = mTime;
	mPrevActionCOM.setZero();
}

void cDeepMimicCharController::InitResources()
{
	InitAction();
	InitTau();
}

void cDeepMimicCharController::InitAction()
{
	mAction = Eigen::VectorXd::Zero(GetActionSize());
}

void cDeepMimicCharController::InitTau()
{
	mTau = Eigen::VectorXd::Zero(mChar->GetNumDof());
}

int cDeepMimicCharController::GetPosDim() const
{
	int dim = 3;
	return dim;
}

bool cDeepMimicCharController::CheckNeedNewAction(double timestep) const
{
	return false;
}

void cDeepMimicCharController::NewActionUpdate()
{
}

void cDeepMimicCharController::HandleNewAction()
{
	mPrevActionTime = mTime;
	mPrevActionCOM = mChar->CalcCOM();
	mNeedNewAction = false;
}

void cDeepMimicCharController::PostProcessAction(Eigen::VectorXd& out_action) const
{
}

bool cDeepMimicCharController::HasGround() const
{
	return mGround != nullptr;
}

double cDeepMimicCharController::SampleGroundHeight(const tVector& pos) const
{
	double h = 0;
	if (mGround != nullptr)
	{
		h = mGround->SampleHeight(pos);
	}
	return h;
}

void cDeepMimicCharController::BuildStatePose(Eigen::VectorXd& out_pose) const
{
	tMatrix origin_trans = mChar->BuildOriginTrans();

	tVector root_pos = mChar->GetRootPos();
	tVector root_pos_rel = root_pos;

	root_pos_rel[3] = 1;
	root_pos_rel = origin_trans * root_pos_rel;
	root_pos_rel[3] = 0;

	out_pose = Eigen::VectorXd::Zero(GetStatePoseSize());
	out_pose[0] = root_pos_rel[1];
	
	int idx = 1;
	int num_parts = mChar->GetNumBodyParts();
	for (int i = 1; i < num_parts; ++i)
	{
		if (mChar->IsValidBodyPart(i))
		{
			const auto& curr_part = mChar->GetBodyPart(i);
			tVector curr_pos = curr_part->GetPos();

			curr_pos[3] = 1;
			curr_pos = origin_trans * curr_pos;
			curr_pos[3] = 0;
			curr_pos -= root_pos_rel;

			out_pose.segment(idx, mPosDim) = curr_pos.segment(0, mPosDim);
			idx += mPosDim;
		}
	}
}

void cDeepMimicCharController::BuildStateVel(Eigen::VectorXd& out_vel) const
{
	out_vel.resize(GetStateVelSize());
	tMatrix origin_trans = mChar->BuildOriginTrans();

	tVector root_pos = mChar->GetRootPos();
	
	int idx = 0;
	int num_parts = mChar->GetNumBodyParts();
	for (int i = 0; i < num_parts; ++i)
	{
		tVector curr_vel = mChar->GetBodyPartVel(i);
		curr_vel = origin_trans * curr_vel;
		out_vel.segment(idx, mPosDim) = curr_vel.segment(0, mPosDim);
		idx += mPosDim;
	}
}

int cDeepMimicCharController::GetStatePoseOffset() const
{
	return 0;
}

int cDeepMimicCharController::GetStateVelOffset() const
{
	return GetStatePoseOffset() + GetStatePoseSize();
}

// added by Yifan
int cDeepMimicCharController::GetStateMapOffset() const
{
    return GetStateVelOffset() + GetStateVelSize();
}

int cDeepMimicCharController::GetStatePoseSize() const
{
	return mChar->GetNumBodyParts() * mPosDim - 1; // -1 for root x
}

int cDeepMimicCharController::GetStateVelSize() const
{
	return mChar->GetNumBodyParts() * mPosDim;
}

// added by Yifan
int cDeepMimicCharController::GetStateMapSize() const
{
    return 1024;
}

//void cDeepMimicCharController::BuildHeightMap(Eigen::VectorXd& map) const
//{
//    // printf("Height map is building in C \n");
//	int map_index = 0;
//	int map_size = 32;
//	int right_foot_id = 5;
//	int left_foot_id = 11;
//	double half_length = 11;
//	double grid_size = 2*half_length/(map_size - 1);
//	double curr_height = 0;
//	map = Eigen::VectorXd::Zero(GetStateMapSize());
//	tVector root_pos0 = mChar->GetRootPos();
//	tVector curr_pos = root_pos0;
//	tVector left_corner = root_pos0;
//	left_corner[0] = left_corner[0] - half_length;
//	left_corner[2] = left_corner[2] - half_length;
//	for (int i = 0; i < map_size; i++)
//	{
//	    for (int j = 0; j < map_size; j++)
//	    {
//            curr_pos[0] = left_corner[0] + i*grid_size;
//            curr_pos[2] = left_corner[2] + j*grid_size;
//            curr_height = mGround->SampleHeight(curr_pos);
//            map(map_index) = curr_height;
//            // printf("point (%d, %d) at position (%.5f, %.5f) with height: %.5f\n", i, j, curr_pos[0], curr_pos[2], map(map_index));
//            map_index = map_index + 1;
//        }
//	}
//}

// revised @ Yan
void cDeepMimicCharController::BuildHeightMap(Eigen::VectorXd& map) const
{
	double sample_step = sample_span / sample_num;
	double curr_height = 0;
	map = Eigen::VectorXd::Zero(GetStateMapSize());
	tVector curr_pos = mChar->GetRootPos();
	for (int i = 0; i < sample_num; i++)
	{
		curr_pos[0] += sample_step;
		curr_height = mGround->SampleHeight(curr_pos);
		map(i) = curr_height;
	}
}
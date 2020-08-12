#include "TerrainGen3D.h"
#include <algorithm>
#include <iostream>
using namespace std;
const double cTerrainGen3D::gInvalidHeight = -std::numeric_limits<double>::infinity();
const float cTerrainGen3D::gVertSpacing = 0.2f;

const cTerrainGen3D::tParamDef cTerrainGen3D::gParamDefs[] =
{
	{ "PathWidth0", 3 },
	{ "PathWidth1", 3 },
	{ "PathWidthDeltaStdev", 0.1 },
	{ "PathMaxTurnDelta", 1.5},
	{ "PathLength", 10 },
	{ "PathTurnLength", 5 },
	{ "PathHeight0", 0 },
	{ "PathHeight1", -2 },
	{ "StepHeightMin", 0.1 },
	{ "StepHeightMax", 0.2 },
	{ "TrailStepProb", 0.2 },
	{ "NumObstacles", 10 },
	{ "ObstacleDensity", 0.015 },
	{ "ObstacleWidth0", 0.5 },
	{ "ObstacleWidth1", 7 },
	{ "ObstacleHeight0", 1 },
	{ "ObstacleHeight1", 5 },
	{ "ObstacleSpeed0", 0.1 },
	{ "ObstacleSpeed1", 2 },
	{ "ObstacleSpeedLerpPow", 1 },
	{ "Slope", 0 },
	{ "ConveyorStripLength", 10 },
	{ "ConveyorNumStrips", 4 },
	{ "ConveyorNumSlices", 10 },
	{ "ConveyorSpacing", 3 },
    
	{"StepSpacingMin", 1},
	{"StepSpacingMax", 2}
};

void cTerrainGen3D::GetDefaultParams(Eigen::VectorXd& out_params)
{
	out_params = Eigen::VectorXd::Zero(eParamsMax);
	assert(sizeof(gParamDefs) / sizeof(gParamDefs[0]) == eParamsMax);
	for (int i = 0; i < eParamsMax; ++i)
	{
		out_params[i] = gParamDefs[i].mDefaultVal;
	}
}

void cTerrainGen3D::LoadParams(const Json::Value& root, Eigen::VectorXd& out_params)
{
	cTerrainGen3D::GetDefaultParams(out_params);
	for (int i = 0; i < eParamsMax; ++i)
	{
		const std::string& name = gParamDefs[i].mName;
		if (!root[name].isNull())
		{
			double val = root[name].asDouble();
			out_params[i] = val;
		}
	}
}

int cTerrainGen3D::CalcNumVerts(const tVector& ground_size)
{
	int res_x = CalcResX(ground_size[0]);
	int res_z = CalcResZ(ground_size[2]);
	return res_x * res_z;
}

int cTerrainGen3D::CalcResX(double x_size)
{
	return static_cast<int>(std::ceil(x_size / gVertSpacing)) + 1;
}

int cTerrainGen3D::CalcResZ(double z_size)
{
	return static_cast<int>(std::ceil(z_size / gVertSpacing)) + 1;
}


cTerrainGen3D::tTerrainFunc cTerrainGen3D::GetTerrainFunc(cGround::eType terrain_type)
{
	switch(terrain_type)
	{
	case cGround::eTypeVar3DFlat:
		return BuildFlat;
	case cGround::eTypeVar3DPath:
		return BuildPath;
	case cGround::eTypeVar3DCliff:
		return BuildCliff;
	case cGround::eTypeVar3DRamp:
		return BuildRamp;
    case cGround::eTypeVar3DCheckers:
		return BuildCheckers;
    case cGround::eTypeVar3DStairs:
		return BuildStairs;
	default:
		printf("Unsupported ground var3d type.\n");
		assert(false);
		return BuildFlat;
	}
}

void cTerrainGen3D::BuildFlat(const tVector& origin, const tVector& ground_size, const Eigen::VectorXd& params, cRand& rand, 
								std::vector<float>& out_data, std::vector<int>& out_flags)
{
	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(ground_size[0]);
	out_res[1] = CalcResZ(ground_size[1]);
	return AddFlat(origin, start_coord, ground_size, out_res, out_data, out_flags);
}

void cTerrainGen3D::BuildPath(const tVector& origin, const tVector& ground_size, const Eigen::VectorXd& params, cRand& rand, 
								std::vector<float>& out_data, std::vector<int>& out_flags)
{
	double w = params[eParamsPathWidth0];
	double len = params[eParamsPathLength];
	double turn_len =0;
	int mode = params[eParamsPathTurnLength];
	double h0 = params[eParamsPathHeight0];
	double h1 = params[eParamsPathHeight1];
	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(ground_size[0]);
	out_res[1] = CalcResZ(ground_size[1]);
	return AddPath(origin, start_coord, ground_size, out_res, w, len, turn_len, h0, h1, out_data, out_flags,mode);
}

void cTerrainGen3D::BuildCliff(const tVector& origin, const tVector& ground_size, const Eigen::VectorXd& params, cRand& rand, 
								std::vector<float>& out_data, std::vector<int>& out_flags)
{
	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(ground_size[0]);
	out_res[1] = CalcResZ(ground_size[1]);
	return AddCliff(origin, start_coord, ground_size, out_res, out_data, out_flags);
}

void cTerrainGen3D::BuildRamp(const tVector& origin, const tVector& ground_size, const Eigen::VectorXd& params, cRand& rand,
								std::vector<float>& out_data, std::vector<int>& out_flags)
{
	double slope = params[eParamsSlope];
	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(ground_size[0]);
	out_res[1] = CalcResZ(ground_size[1]);
	return AddRamp(origin, start_coord, ground_size, slope, out_res, out_data, out_flags);
}


void cTerrainGen3D::BuildCheckers(const tVector& origin, const tVector& ground_size, const Eigen::VectorXd& params, cRand& rand,
								std::vector<float>& out_data, std::vector<int>& out_flags)
{
	double slope = params[eParamsSlope];

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(ground_size[0]);
	out_res[1] = CalcResZ(ground_size[1]);
	return AddCheckers(origin, start_coord, ground_size, slope, out_res, out_data, out_flags);
}


void cTerrainGen3D::BuildStairs(const tVector& origin, const tVector& ground_size, const Eigen::VectorXd& params, cRand& rand,
								std::vector<float>& out_data, std::vector<int>& out_flags)
{
    double spacing_min = params[eParamsStepSpacingMin];
	double spacing_max = params[eParamsStepSpacingMax];
	double step_h_min = params[eParamsStepHeightMin];
	double step_h_max = params[eParamsStepHeightMax];

	Eigen::Vector2i start_coord = Eigen::Vector2i::Zero();
	Eigen::Vector2i out_res = Eigen::Vector2i::Zero();
	out_res[0] = CalcResX(ground_size[0]);
	out_res[1] = CalcResZ(ground_size[1]);
	return AddStairs(origin, start_coord, ground_size, spacing_min, spacing_max, step_h_min, step_h_max,  out_res, rand, out_data, out_flags);
}


void cTerrainGen3D::AddFlat(const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, const Eigen::Vector2i& out_res, 
								std::vector<float>& out_data, std::vector<int>& out_flags)
{
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;

	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			double h = 0;
			//if (coord_x % 5 == 0 && coord_z % 5 == 1) h = -0.1;
			int curr_flags = 1 << eVertFlagEnableTex;

			out_data[idx] = h;// +cMathUtil::RandDouble() * 0.05;
			out_flags[idx] = curr_flags;
		}
	}
}

void cTerrainGen3D::AddPath(const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, const Eigen::Vector2i& out_res, 
							double w, double len, double turn_len, double h0, double h1, 
							std::vector<float>& out_data, std::vector<int>& out_flags,int mode)
{
	if(mode == 0){   //mode 0 means stairs
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;
	double stepheight = 0.3;
	double plus=0;
	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{			
			int mod40 = i % 40;
			if(mod40 >=0 && mod40 <4){
				plus= stepheight;
			}else if(mod40 >= 4 && mod40 <8){
				plus= stepheight*2;
			}else if(mod40 >= 8 && mod40 <12){
				plus= stepheight*3;
			}else if(mod40 >= 12 && mod40 <16){
				plus= stepheight*4;
			}else if(mod40 >= 16 && mod40 <24){
				plus= stepheight*5;
			}else if(mod40 >= 24 && mod40 <28){
				plus= stepheight*4;
			}else if(mod40 >= 28 && mod40 <32){
				plus= stepheight*3;
			}else if(mod40 >= 32 && mod40 <36){
				plus= stepheight*2;
			}else if(mod40 >= 36 && mod40 <40){
				plus= stepheight*1;
			}	
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			tVector curr_pos = origin + tVector(i * gVertSpacing, 0, j * gVertSpacing, 0);
			int curr_flags = 1 << eVertFlagEnableTex;

			double curr_h = h1;
			int interval = std::floor(curr_pos[0] / len);
			if (interval % 2 == 0)
			{
				curr_h = (std::abs(curr_pos[2]) < 0.5 * w) ? (h0+plus) : h1;
			}
			else
			{
				curr_h = (std::abs(turn_len - curr_pos[2]) < 0.5 * w) ? (h0+plus) : h1;
			}

			float interval_start = interval * len;
			float interval_end = (interval + 1) * len;
			if (std::abs(interval_start - curr_pos[0]) < 0.5 * w
				|| std::abs(interval_end - curr_pos[0]) < 0.5 * w)
			{
				if (curr_pos[2] > -0.5 * w
					&& curr_pos[2] < turn_len + 0.5 * w)
				{
					curr_h = h0+plus;
				}
			}

			out_data[idx] = curr_h;
			out_flags[idx] = curr_flags;
		}
	}

	}else if(mode == 1){ // Ramp
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;
	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			tVector curr_pos = origin + tVector(i * gVertSpacing, 0, j * gVertSpacing, 0);
			int curr_flags = 1 << eVertFlagEnableTex;

			double curr_h = h1;
			//
			double addH = 0;
			double oriH = 0;
			if(j ==0 && i ==0){
				oriH =1;
			}
			if(j>=0 && j<=9){
				if(i%15 ==10 || i%15 ==11 || i%15 ==12 || i%15==13){
					if(j>=0 && j<=4){
						addH = (j-0)*0.1;
					}else{
						addH = (8-j)*0.1;
						if(addH<0){
							addH=0;
						}
					}
				}
			}
			if(i%15==9){
				if(j==2||j==3 || j==5 ||j==6){
					addH =0.2;
				}else if(j==1 || j==7){
					addH=0.1;
				}else if(j==4){
					addH =0.25;
				}
			}
			if(i%15==8){
				if(j>=1 && j<=6){
					addH =0.1;
				}
			}
			if(i%15 ==14){
				if(j==2||j==3 || j==5 ||j==6){
					addH =0.2;
				}else if(j==1 || j==7){
					addH=0.1;
				}else if(j==4){
					addH =0.25;
				}
			}
			if(i%15==0){
				if(j>=1 && j<=6){
					addH =0.1;
				}
			}

			int interval = std::floor(curr_pos[0] / len);
			if (interval % 2 == 0)
			{
				curr_h = (std::abs(curr_pos[2]) < 0.5 * w) ? h0+addH+oriH : h1;
			}
			else
			{
				curr_h = (std::abs(turn_len - curr_pos[2]) < 0.5 * w) ? h0+addH+oriH: h1;
			}

			float interval_start = interval * len;
			float interval_end = (interval + 1) * len;
			if (std::abs(interval_start - curr_pos[0]) < 0.5 * w
				|| std::abs(interval_end - curr_pos[0]) < 0.5 * w)
			{
				if (curr_pos[2] > -0.5 * w
					&& curr_pos[2] < turn_len + 0.5 * w)
				{
					curr_h = h0+addH+oriH;
				}
			}

			out_data[idx] = curr_h;
			out_flags[idx] = curr_flags;
		}
	}
	
	}else if(mode ==2){ // SLALON
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;
	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			tVector curr_pos = origin + tVector(i * gVertSpacing, 0, j * gVertSpacing, 0);
			int curr_flags = 1 << eVertFlagEnableTex;
			//
			double addH = 0;
			double oriH = 0;
			if(j ==0 && i ==0){
				oriH =1;
			}
			if(j==4 || j ==5){
				if(i % 15 ==10 || i%15 ==11){
					addH =1;
				}
			}
			double curr_h = h1;
			int interval = std::floor(curr_pos[0] / len);
			if (interval % 2 == 0)
			{
				curr_h = (std::abs(curr_pos[2]) < 0.5 * w) ? h0+addH+oriH : h1;
			}
			else
			{
				curr_h = (std::abs(turn_len - curr_pos[2]) < 0.5 * w) ? h0+addH+oriH : h1;
			}

			float interval_start = interval * len;
			float interval_end = (interval + 1) * len;
			if (std::abs(interval_start - curr_pos[0]) < 0.5 * w
				|| std::abs(interval_end - curr_pos[0]) < 0.5 * w)
			{
				if (curr_pos[2] > -0.5 * w
					&& curr_pos[2] < turn_len + 0.5 * w)
				{
					curr_h = h0+addH+oriH;
				}
			}

			out_data[idx] = curr_h;
			out_flags[idx] = curr_flags;
		}
	}
	}else if(mode ==3){   //Rough terrain
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;
	double height_rough =0.1;
	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			tVector curr_pos = origin + tVector(i * gVertSpacing, 0, j * gVertSpacing, 0);
			int curr_flags = 1 << eVertFlagEnableTex;

			double curr_h = h1;
			//
			double addH = 0;
			double oriH = 0;
			double xPos =i%30;
			if(j ==0 && i ==0){
				oriH =1;
			}

			if(xPos ==6){
				if(j>=3 && j<=6){
					addH = height_rough;
				}
			}

			if(xPos == 10){
				if(j>=4 && j<=5){
					addH = height_rough;
				}
			}
			if(xPos == 24){
				if(j>=3 && j<=6){
					addH = height_rough;
				}
			}
			if(xPos == 19){
				if(j>=4 && j<=5){
					addH = height_rough;
				}
			}
			if(xPos>=14 && xPos<16){
				addH =height_rough;
			}
			if(xPos == j+5){   // left bottom
				if(j>=1 && j<=3){
					addH =height_rough;
				}
				
			}
			if(xPos == j+9){  // left bottom
				if(j>=1 && j<=3){
					addH =height_rough;
				}
			}
			if(xPos == -j+20){ //left top
				if(j>=1 && j<=3){
					addH =height_rough;
				}
			}
			if(xPos == -j+25){ // left top
				if(j>=1 && j<=3){
					addH =height_rough;
				}
			}
			if(xPos == -j+14){ // right bottom
				if(j>=6 && j<=8){
					addH =height_rough;
				}
			}
			if(xPos == -j+18){ //ritht bottom
				if(j>=6 && j<=8){
					addH =height_rough;
				}
			}
			if(xPos == j+11){  // right top
				if(j>=6 && j<=8){
					addH =height_rough;
				}
			} 
			if(xPos == j+15){  //right top
				if(j>=6 && j<=8){
					addH =height_rough;
				}
			}
			//
			int interval = std::floor(curr_pos[0] / len);
			if (interval % 2 == 0)
			{
				curr_h = (std::abs(curr_pos[2]) < 0.5 * w) ? h0+addH+oriH : h1;
			}
			else
			{
				curr_h = (std::abs(turn_len - curr_pos[2]) < 0.5 * w) ? h0+addH+oriH : h1;
			}

			float interval_start = interval * len;
			float interval_end = (interval + 1) * len;
			if (std::abs(interval_start - curr_pos[0]) < 0.5 * w
				|| std::abs(interval_end - curr_pos[0]) < 0.5 * w)
			{
				if (curr_pos[2] > -0.5 * w
					&& curr_pos[2] < turn_len + 0.5 * w)
				{
					curr_h = h0+addH+oriH;
				}
			}

			out_data[idx] = curr_h;
			out_flags[idx] = curr_flags;
		}
	}
	}else if(mode ==4 ){    // for stairs up train
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;
	double stepheight = 0.3;
	double plus=0;
	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{			
			if( i >=0 && i<=4){
				plus = 0;
			}else if(i>=5 && i<=184){
				int p =(i-(i%4))/4-1;
				plus = p*stepheight;
			}else{
				plus = 0;
			}
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			tVector curr_pos = origin + tVector(i * gVertSpacing, 0, j * gVertSpacing, 0);
			int curr_flags = 1 << eVertFlagEnableTex;

			double curr_h = h1;
			int interval = std::floor(curr_pos[0] / len);
			if (interval % 2 == 0)
			{
				curr_h = (std::abs(curr_pos[2]) < 0.5 * w) ? (h0+plus) : h1;
			}
			else
			{
				curr_h = (std::abs(turn_len - curr_pos[2]) < 0.5 * w) ? (h0+plus) : h1;
			}

			float interval_start = interval * len;
			float interval_end = (interval + 1) * len;
			if (std::abs(interval_start - curr_pos[0]) < 0.5 * w
				|| std::abs(interval_end - curr_pos[0]) < 0.5 * w)
			{
				if (curr_pos[2] > -0.5 * w
					&& curr_pos[2] < turn_len + 0.5 * w)
				{
					curr_h = h0+plus;
				}
			}

			out_data[idx] = curr_h;
			out_flags[idx] = curr_flags;
		}
	}
	}else if(mode ==5 ){    // for stairs down train
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;
	double stepheight = -0.3;
	double plus=0;
	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{			
			if( i >=0 && i<=4){
				plus = 0;
			}else if(i>=5 && i<=184){
				int p =(i-(i%4))/4-1;
				plus = p*stepheight;
			}else{
				plus = 0;
			}
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			tVector curr_pos = origin + tVector(i * gVertSpacing, 0, j * gVertSpacing, 0);
			int curr_flags = 1 << eVertFlagEnableTex;

			double curr_h = h1;
			int interval = std::floor(curr_pos[0] / len);
			if (interval % 2 == 0)
			{
				curr_h = (std::abs(curr_pos[2]) < 0.5 * w) ? (h0+plus) : h1;
			}
			else
			{
				curr_h = (std::abs(turn_len - curr_pos[2]) < 0.5 * w) ? (h0+plus) : h1;
			}

			float interval_start = interval * len;
			float interval_end = (interval + 1) * len;
			if (std::abs(interval_start - curr_pos[0]) < 0.5 * w
				|| std::abs(interval_end - curr_pos[0]) < 0.5 * w)
			{
				if (curr_pos[2] > -0.5 * w
					&& curr_pos[2] < turn_len + 0.5 * w)
				{
					curr_h = h0+plus;
				}
			}

			out_data[idx] = curr_h;
			out_flags[idx] = curr_flags;
		}
	}
	}
	else{ // This mode means normal path
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;
	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			tVector curr_pos = origin + tVector(i * gVertSpacing, 0, j * gVertSpacing, 0);
			int curr_flags = 1 << eVertFlagEnableTex;

			double curr_h = h1;
			int interval = std::floor(curr_pos[0] / len);
			if (interval % 2 == 0)
			{
				curr_h = (std::abs(curr_pos[2]) < 0.5 * w) ? h0 : h1;
			}
			else
			{
				curr_h = (std::abs(turn_len - curr_pos[2]) < 0.5 * w) ? h0 : h1;
			}

			float interval_start = interval * len;
			float interval_end = (interval + 1) * len;
			if (std::abs(interval_start - curr_pos[0]) < 0.5 * w
				|| std::abs(interval_end - curr_pos[0]) < 0.5 * w)
			{
				if (curr_pos[2] > -0.5 * w
					&& curr_pos[2] < turn_len + 0.5 * w)
				{
					curr_h = h0;
				}
			}

			out_data[idx] = curr_h;
			out_flags[idx] = curr_flags;
		}
	}

	}
	
}


void cTerrainGen3D::AddCliff(const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, const Eigen::Vector2i& out_res,
							std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const double h0 = 0;
	const double h1 = 2;
	const double h2 = -2;
	const double w = 5;
	const double period_len = 12;
	const double amplitude = 5;
	const double slope = 0.5;
	const double bump_size = 0.2;

	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;

	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			tVector curr_pos = origin + tVector(i * gVertSpacing, 0, j * gVertSpacing, 0);
			int curr_flags = 0;

			double curr_h = h0;
			double mode_val = std::cos(2 * M_PI * curr_pos[0] / period_len) - 1;
			//double mode_val = 0.5 * std::sin(2 * M_PI * curr_pos[0] / period_len)
			//				+ 2 * std::cos(0.5 * M_PI * curr_pos[0] / period_len);
			//mode_val *= 0.5 + std::cos(0.1 * M_PI * curr_pos[0] / period_len);

			mode_val *= amplitude;
			double curr_dist = curr_pos[2] - mode_val;

			if (curr_dist < -0.5 * w)
			{
				curr_h = h1;
			}
			else if (curr_dist > 0.5 * w)
			{
				curr_h = h2;
			}

			double h_lerp = std::abs(curr_dist) - 0.5 * w;
			h_lerp *= slope;
			h_lerp = cMathUtil::Clamp(h_lerp, 0.0, 1.0);
			curr_h = (1 - h_lerp) * h0 + h_lerp * curr_h;

			bool on_edge = (h_lerp > 0) && (h_lerp < 1);
			if (on_edge)
			{
				double bump_h = cMathUtil::RandDoubleSeed(curr_pos[0] * curr_pos[0] * M_PI + curr_pos[2]);
				bump_h = 2 * bump_h - 1;
				bump_h *= bump_size;
				curr_h += bump_h;
			}
			else
			{
				curr_flags |= 1 << eVertFlagEnableTex;
			}

			out_data[idx] = curr_h;
			out_flags[idx] = curr_flags;
		}
	}
}

void cTerrainGen3D::AddRamp(const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double slope, 
							const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const double pad = 0;
	
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	// int res_x = 2;
	// int res_z = 10;
	int num_verts = res_x * res_z;

	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			double h = 0;
			int curr_flags = 1 << eVertFlagEnableTex;

			tVector pos = origin;
			pos[0] += (i / (res_x - 1.0)) * size[0];
			pos[2] += (j / (res_z - 1.0)) * size[2];

			double x = pos[0];
			x = cMathUtil::Sign(x) * std::max(0.0, (std::abs(x) - pad));
			h = slope * x;

			// hack hack hack
			/*
			double x0 = 1;
			double x1 = 3;
			double x2 = 4;
			double x3 = 6;
			double x4 = 8;
			double slope0 = 0.16;
			double slope1 = -0.11;
			double slope2 = -0.3;

			if (x < x0)
			{
				h = 0;
			}
			else if (x >= x0 && x < x1)
			{
				h = slope0 * (x - x0);
			}
			else if (x >= x1 && x < x2)
			{
				h = slope0 * (x1 - x0);
			}
			else if (x >= x2 && x < x3)
			{
				h = slope0 * (x1 - x0) + slope1 * (x - x2);
			}
			else if (x >= x3 && x < x4)
			{
				h = slope0 * (x1 - x0) + slope1 * (x3 - x2);
			}
			else
			{
				h = slope0 * (x1 - x0) + slope1 * (x3 - x2) + slope2 * (x - x4);
			}
			*/

			out_data[idx] = h;
			out_flags[idx] = curr_flags;
		}
	}
}

void cTerrainGen3D::AddCheckers(const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double slope, 
							const Eigen::Vector2i& out_res, std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const double pad = 1;
	
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;

	for (int j = 0; j < res_z; ++j)
	{
		size_t coord_z = j + start_coord[1];
		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;
			double h = 0;
			int curr_flags = 1 << eVertFlagEnableTex;

			tVector pos = origin;
			pos[0] += (i / (res_x - 1.0)) * size[0];
			pos[2] += (j / (res_z - 1.0)) * size[2];

			double x = pos[0];
			double z = pos[2];
			x = std::max(0.0, (std::abs(x)));
			z = std::max(0.0, (std::abs(z)));
            h = ((int)std::floor(z)+(int)std::floor(x))%2*slope;

			out_data[idx] = h;
			out_flags[idx] = curr_flags;
		}
	}
}

void cTerrainGen3D::AddStairs(const tVector& origin, const Eigen::Vector2i& start_coord, const tVector& size, double spacing_min, double spacing_max, double step_h_min, double step_h_max, 
							const Eigen::Vector2i& out_res, cRand& rand, std::vector<float>& out_data, std::vector<int>& out_flags)
{
	const double pad = 1;
	
	int res_x = CalcResX(size[0]);
	int res_z = CalcResX(size[2]);
	int num_verts = res_x * res_z;
    //double last_step_z = origin[2];

    //double current_spacing = rand.RandDouble(spacing_min, spacing_max);
    //double current_height = 0;

    // double randomness = rand.RandDouble();
    double randomness = 0.5;

	for (int j = 0; j < res_z; ++j)
	{
        //Check the new step and reset the height if we are done with the last step 
        
        double z = origin[2] + (j / (res_z - 1.0)) * size[2];
        /*
        if (z > (last_step_z + current_spacing)) 
        {
            last_step_z = z; 
            current_height += cMathUtil::Sign(z)*rand.RandDouble(step_h_min, step_h_max);
            current_spacing = rand.RandDouble(spacing_min, spacing_max);
        }
        */
        
		size_t coord_z = j + start_coord[1];

		for (int i = 0; i < res_x; ++i)
		{
			size_t coord_x = i + start_coord[0];
			size_t idx = coord_z * out_res[0] + coord_x;

			double x = origin[0] + (i / (res_x - 1.0)) * size[0];

			x = cMathUtil::Sign(x) * std::max(0.0, (std::abs(x) - pad));

			// double h=0;
            double h = (std::floor(x*2))*0.1/2;
			// double h = 0;
			// cout<<"i= "<<i<<"; h= "<<h<<endl;
            //rand.Seed(h);
            // h += rand.RandDouble(step_h_min, step_h_max);
            h += cMathUtil::RandDoubleSeed(h*randomness)*(step_h_max-step_h_min)+step_h_min;
			// h += i/10 *0.2;
			// h += 0.5;
			// h += 0.3;
			// h += i*0.3;
			// cout<<"i = "<<i<<" ;fh = "<<h<<endl;
			int curr_flags = 1 << eVertFlagEnableTex;
			out_data[idx] = h;
			out_flags[idx] = curr_flags;
		}
	}
}

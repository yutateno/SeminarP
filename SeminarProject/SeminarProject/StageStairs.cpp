#include "StageStairs.hpp"


StageStairs::StageStairs(int draw, int collStageHandle, VECTOR area) : BasicObject(collStageHandle)
{
	this->area = area;


	shadowHeight = 10.0f;
	shadowSize = 5.0f;


	this->modelHandle = 0;
	this->modelHandle = MV1DuplicateModel(draw);


	MV1SetPosition(modelHandle, area);
}

StageStairs::~StageStairs()
{
	if (modelHandle != -1)
	{
		MV1DeleteModel(modelHandle);
	}
}

void StageStairs::Draw()
{
	BasicObject::ShadowFoot();


	BasicObject::Draw();
}

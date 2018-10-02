#include "EnemyMove2.hpp"


EnemyMove2::EnemyMove2(const int collStageHandle, const VECTOR area, const int modelHandle) : BasicCreature(collStageHandle)
{
	this->area = area;

	modelHeight = 10.0f;
	modelWigth = 10.0f;

	shadowHeight = 10.0f;
	shadowSize = 15.0f;

	this->modelHandle = MV1DuplicateModel(modelHandle);

	MV1SetPosition(this->modelHandle, area);
}

EnemyMove2::~EnemyMove2()
{
	MODEL_RELEASE(modelHandle);
}

void EnemyMove2::Draw()
{
	BasicObject::ShadowFoot();

	BasicObject::Draw();
}

void EnemyMove2::Process()
{
}

void EnemyMove2::SetArea(const VECTOR area)
{
	this->area = area;
}

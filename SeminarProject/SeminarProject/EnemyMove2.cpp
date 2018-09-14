#include "EnemyMove2.hpp"


EnemyMove2::EnemyMove2(int collStageHandle, VECTOR area, int modelHandle) : BasicCreature(collStageHandle)
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
	MV1DeleteModel(modelHandle);
}

void EnemyMove2::Draw()
{
	BasicObject::ShadowFoot();

	BasicObject::Draw();
}

void EnemyMove2::Process()
{
}

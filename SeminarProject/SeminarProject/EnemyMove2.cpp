#include "EnemyMove2.hpp"


EnemyMove2::EnemyMove2(const int collStageHandle, const VECTOR area, const int modelHandle) : BasicCreature(collStageHandle)
{
	this->area = VAdd(area, VGet(0.0f, 10.0f, 0.0f));

	modelHeight = 100.0f;
	modelWigth = 50.0f;

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


#ifdef _ENEMY2_DEBUG
	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// 当たり判定を確認用の表示テスト
#endif // _ENEMY2_DEBUG
}

void EnemyMove2::Process()
{
	// ステージのあたり判定
	StageHit();

	MV1SetPosition(this->modelHandle, area);
}

void EnemyMove2::SetArea(const VECTOR area)
{
	this->area = area;
}

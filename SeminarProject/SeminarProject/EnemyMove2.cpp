#include "EnemyMove2.hpp"


EnemyMove2::EnemyMove2(int collStageHandle, VECTOR area) : BasicCreature(collStageHandle)
{
	this->area = area;

	modelHeight = 10.0f;
	modelWigth = 10.0f;

	shadowHeight = 10.0f;
	shadowSize = 15.0f;
}

EnemyMove2::~EnemyMove2()
{
}

void EnemyMove2::Draw()
{
	BasicObject::ShadowFoot();

	// Ｚバッファを有効にする
	SetUseZBuffer3D(TRUE);
	// Ｚバッファへの書き込みを有効にする
	SetWriteZBuffer3D(TRUE);
	DrawSphere3D(VAdd(area, VGet(0.0f, 10.0f, 0.0f)), modelWigth, 16, GetColor(120, 120, 120), GetColor(120, 120, 120), TRUE);
	// Ｚバッファへの書き込みを有効にする
	SetWriteZBuffer3D(FALSE);
	// Ｚバッファを有効にする
	SetUseZBuffer3D(FALSE);
}

void EnemyMove2::Process()
{
}

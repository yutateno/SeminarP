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

	// �y�o�b�t�@��L���ɂ���
	SetUseZBuffer3D(TRUE);
	// �y�o�b�t�@�ւ̏������݂�L���ɂ���
	SetWriteZBuffer3D(TRUE);
	DrawSphere3D(VAdd(area, VGet(0.0f, 10.0f, 0.0f)), modelWigth, 16, GetColor(120, 120, 120), GetColor(120, 120, 120), TRUE);
	// �y�o�b�t�@�ւ̏������݂�L���ɂ���
	SetWriteZBuffer3D(FALSE);
	// �y�o�b�t�@��L���ɂ���
	SetUseZBuffer3D(FALSE);
}

void EnemyMove2::Process()
{
}

#include "Character.hpp"

Character::Character()
{
	// ３Ｄモデルの読み込み
	this->charamodelhandle = MV1LoadModel("media\\CLPH\\motion\\CLPH_idle.fbx");

	area = VGet(0, 0, 0);
}

Character::~Character()
{
	if (charamodelhandle != -1)
	{
		MV1DeleteModel(charamodelhandle);
	}
}

void Character::Process()
{
	// 指定位置にモデルを配置
	MV1SetPosition(charamodelhandle, area);
}

void Character::Draw()
{
	MV1DrawModel(charamodelhandle);
}

VECTOR Character::GetArea()
{
	return area;
}
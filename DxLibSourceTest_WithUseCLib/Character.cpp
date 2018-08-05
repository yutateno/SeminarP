#include "Character.hpp"

Character::Character()
{
	// �R�c���f���̓ǂݍ���
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
	// �w��ʒu�Ƀ��f����z�u
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
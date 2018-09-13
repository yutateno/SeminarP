#pragma once
#include "InputPad.hpp"
#include "BasicCreature.hpp"


class CharacterSword : public BasicCreature
{
private:
	// ���ꂼ��̈ʒu�Ɋւ���
	float direZAngle;		// �O��̃L���������������ϐ�
	float direXAngle;		// ���E�̃L���������������ϐ�


	// ���[�V�����Ɋւ���
	enum MOTION { attack, no };


	// �����Ɋւ���
	void MoveProcess(unsigned __int8 controllNumber);


public:
	CharacterSword(int modelHandle, int collStageHandle);
	~CharacterSword();


	void Draw();
	void Process(unsigned __int8 controllNumber, float getAngle);


	void PositionReset();
};


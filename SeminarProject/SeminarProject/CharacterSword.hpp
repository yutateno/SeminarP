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
	enum MOTION { idle, action1, action2, action3, skyAction1, skyAction2, skyAction3, dash, walk, jump, fall };


	// �K�i�Ɋւ���
	int stairsHandle[10];
	VECTOR stairsArea[10];


	// �����Ɋւ���
	void MoveProcess(unsigned __int8 controllNumber);


	// �U���Ɋւ���
	bool attackNow;
	bool attackNext;
	float attackFrame;
	int attackNumber;
	int preAttackNumber;
	void AttackProcess(unsigned __int8 controllNumber);


public:
	CharacterSword(const int modelHandle, const int collStageHandle, const int stairsHandle);
	~CharacterSword();


	void SetStairsArea(const VECTOR stairsArea, const int num);


	void Draw();
	void Process(const unsigned __int8 controllNumber, const float getAngle);


	void PositionReset();
};


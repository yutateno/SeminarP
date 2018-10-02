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
	enum MOTION { idle, action1, action2, action3, skyAction1, skyAction2, skyAction3, dash, walk, jump, fall
				, damage, death, gunAction, gunActionWalk, nagiharai, handUp};


	// �K�i�Ɋւ���
	int stairsHandle[10];
	VECTOR stairsArea[10];


	// �����Ɋւ���
	bool walkNow;
	void MoveProcess(unsigned __int8 controllNumber);


	// �U���Ɋւ���
	bool attackNow;											// ���U�����Ă��邩�ǂ���
	bool attackNext;										// ���̃R���{�ւȂ��邩�ǂ���
	float attackFrame;										// ���̍U���̃t���[����
	int attackNumber;										// �U���̔ԍ�
	int preAttackNumber;									// ���O�̍U���̔ԍ�
	void AttackProcess(unsigned __int8 controllNumber);		// �U���̃v���Z�X


	// �W�����v�Ɋւ���
	bool jumpNow;
	bool jumpUpNow;
	float jumpPower;
	float gravity;
	float flyJumpPower;
	float fallJumpPower;
	void JumpProcess(unsigned __int8 controllNumber);


	// �s���ɂ���ăA�j���[�V�����̊Ǘ�
	void AnimProcess();


public:
	CharacterSword(const int modelHandle, const int collStageHandle, const int stairsHandle);
	~CharacterSword();


	void SetStairsArea(const VECTOR stairsArea, const int num);


	void Draw();
	void Process(const unsigned __int8 controllNumber, const float getAngle);


	void PositionReset();


	const VECTOR GetPreArea() const;
};


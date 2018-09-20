#pragma once
#include "BasicCreature.hpp"


#include <random>


class OrdinaryPerson : public BasicCreature
{
private:
	// ���ꂼ��̈ʒu�Ɋւ���
	float direZAngle;		// �O��̃L���������������ϐ�
	float direXAngle;		// ���E�̃L���������������ϐ�
	float nextDireZAngle;
	float nextDireXAngle;


	// ���[�V�����Ɋւ���
	enum MOTION { run, idle, walk };


	// �����Ɋւ���
	void MoveProcess();
	int moveCount;


public:
	OrdinaryPerson(const int modelHandle, const int collStageHandle, const VECTOR area);
	~OrdinaryPerson();


	void Draw();
	void Process();
};


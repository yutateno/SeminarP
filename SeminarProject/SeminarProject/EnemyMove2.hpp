#pragma once
#include "BasicCreature.hpp"


#include <random>


class EnemyMove2 : public BasicCreature
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
	EnemyMove2(int modelHandle, int collStageHandle);
	~EnemyMove2();


	void Draw();
	void Process();
};


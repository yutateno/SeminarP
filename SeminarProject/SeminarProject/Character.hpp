#pragma once
#include "BasicCreature.hpp"


class Character : public BasicCreature
{
private:
	// ���ꂼ��̈ʒu�Ɋւ���
	float direZAngle;		// �O��̃L���������������ϐ�
	float direXAngle;		// ���E�̃L���������������ϐ�


	// ���[�V�����Ɋւ���
	enum MOTION { run, idle, walk };


	// �����Ɋւ���
	void MoveProcess(unsigned __int8 controllNumber);


public:
	Character(const int modelHandle, const int collStageHandle);	// �R���X�g���N�^
	~Character();					// �f�X�g���N�^


	void Draw();
	void Process(const unsigned __int8 controllNumber, const float getAngle);


	void PositionReset();
};
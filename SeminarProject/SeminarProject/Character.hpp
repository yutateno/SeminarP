#pragma once
#include "InputPad.hpp"
#include "InputKey.hpp"
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

	int lightHandle;			// ���C�g�n���h�����ێ�
	float lightRange;

	int enemyCatchNum;

public:
	Character(int modelHandle, int collStageHandle);	// �R���X�g���N�^
	~Character();					// �f�X�g���N�^

	void Draw();
	void Process(unsigned __int8 controllNumber, float getAngle);

	void DoEnemyCatchNum();
};
#pragma once
#include "InputPad.hpp"
#include "InputKey.hpp"
#include "Basic.hpp"
#include "BasicActor.hpp"

class Character : public BasicActor
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
	Character(int collStageHandle);	// �R���X�g���N�^
	~Character();					// �f�X�g���N�^
	
	void Draw();
	void Process(unsigned __int8 controllNumber, float getAngle);
};
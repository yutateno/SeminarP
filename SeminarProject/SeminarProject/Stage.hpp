#pragma once
#include "Basic.hpp"


class Stage
{
private:
	int drawStageHandle;		// �X�e�[�W


public:
	Stage(int drawStageHandle);		// �R���X�g���N�^
	~Stage();						// �f�X�g���N�^


	void Draw();				// �`��
};
#pragma once
#include "DxLib.h"
#include "LoadFile.hpp"
#include "Basic.hpp"

class Stage
{
private:
	int drawStageHandle;		// �X�e�[�W
	int collStageHandle;

public:
	Stage();
	~Stage();

	void LoadInit();		// �ێ�����K�v�̂Ȃ����[�h���f�����폜

	void Draw();

	int GetCollStageHandle();
};
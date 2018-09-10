#pragma once
#include "BaseMove.hpp"
#include "MainMove1.hpp"
#include "LoadThread.hpp"

class Manager
{
private:
	ESceneNumber e_preScene;		// ���O�̃V�[��
	ESceneNumber e_nowScene;		// ���̃V�[��

	BaseMove* p_baseMove;			// �V�[���̊��N���X
	LoadThread* p_loadThread;

	void SceneChange();


	// ���[�u�P�Ɋւ���
	const int max1 = 4;
	std::string move1str[4];
	ELOADFILE load1[4];

public:
	Manager();
	~Manager();

	void Update(unsigned __int8 controllNumber);
};


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
	LoadThread* p_loadThread;		// ���[�h�̃N���X


	void SceneChange();				// �V�[����؂�ւ���


	// ���[�u�P�Ɋւ���
	const int max1 = 4;
	std::string move1str[4];
	ELOADFILE load1[4];



public:
	Manager();			// �R���X�g���N�^
	~Manager();			// �f�X�g���N�^

	void Update(unsigned __int8 controllNumber);		// ���C���v���Z�X
};
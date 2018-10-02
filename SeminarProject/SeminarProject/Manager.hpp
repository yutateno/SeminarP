#pragma once
#include "BaseMove.hpp"
#include "MainMove1.hpp"
#include "LoadThread.hpp"
#include "MainMove2.hpp"


class Manager
{
private:
	ESceneNumber e_preScene;		// ���O�̃V�[��
	ESceneNumber e_nowScene;		// ���̃V�[��


	BaseMove* p_baseMove;			// �V�[���̊��N���X
	LoadThread* p_loadThread;		// ���[�h�̃N���X


	void SceneChange();				// �V�[����؂�ւ���


	// ���[�u�P�Ɋւ���
	const int max1 = 5;
	std::string move1str[5];
	ELOADFILE load1[5];


	// ���[�u�Q�Ɋւ���
	const int max2 = 7;
	std::string move2str[7];
	ELOADFILE load2[7];


	// ���[�u�R�Ɋւ���
	const int max3 = 7;
	std::string move3str[7];
	ELOADFILE load3[7];


	// ���[�u�S�Ɋւ���
	const int max4 = 7;
	std::string move4str[7];
	ELOADFILE load4[7];


	// ���[�u�T�Ɋւ���
	const int max5 = 7;
	std::string move5str[7];
	ELOADFILE load5[7];


	// ���[�u�U�Ɋւ���
	const int max6 = 7;
	std::string move6str[7];
	ELOADFILE load6[7];


	// �^�C�g���Ɋւ���
	const int maxTitle = 7;
	std::string moveTitleStr[7];
	ELOADFILE loadTitle[7];



	int antiAliasScreen;



public:
	Manager();			// �R���X�g���N�^
	~Manager();			// �f�X�g���N�^

	void Update(const unsigned __int8 controllNumber);		// ���C���v���Z�X
};
#pragma once
#include "Stage.hpp"
#include "CharacterSword.hpp"
#include "EnemyMove2.hpp"
#include "Camera.hpp"
#include "BaseMove.hpp"
#include "StageStairs.hpp"
#include "StageStreetLight.hpp"

#include <random>

/*
�I�u�W�F�N�g���W�߂�V�[��
�����󂵂ăh���b�v�A�C�e���𓾂�A������t�B�[���h�̓���̒n�_�Ŏg�p���ăI�u�W�F�N�g�𐶐�����B
*/
class MainMove2 : public BaseMove
{
private:
	enum EFILE { stage, characterAttack, block, stairs, stairsColl, streetLight, skyBox };			// ���[�h�œn�����t�@�C��


	// �X�e�[�W
	Stage* p_stage;			// �X�e�[�W�̃|�C���^
	StageStairs* p_stageStairs[10];
	StageStreetLight* p_stageStreetLight[50];


	// �L�����N�^�[
	CharacterSword* p_character;		// �L�����N�^�[�̃|�C���^


	// ��ʐl
	EnemyMove2* p_enemy;				// �G�̃|�C���^


	// �J����
	Camera* p_camera;					// �J�����̃|�C���^


	void ShadowDraw();


	//void AttackProcess();

public:
	MainMove2(const std::vector<int> v_file);
	~MainMove2();

	
	void Draw();
	void Process(const unsigned __int8 controllNumber);
	void CameraProcess();
};


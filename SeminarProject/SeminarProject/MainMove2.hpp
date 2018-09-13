#pragma once
#include "Stage.hpp"
#include "CharacterSword.hpp"
#include "EnemyMove2.hpp"
#include "Camera.hpp"
#include "BaseMove.hpp"

#include <random>


class MainMove2 : public BaseMove
{
private:
	enum EFILE { stage, characterAttack, character };			// ���[�h�œn�����t�@�C��


	// �X�e�[�W
	Stage* p_stage;			// �X�e�[�W�̃|�C���^


	// �L�����N�^�[
	CharacterSword* p_character;		// �L�����N�^�[�̃|�C���^


	// �G
	EnemyMove2* p_enemy;				// �G�̃|�C���^


	// �J����
	Camera* p_camera;					// �J�����̃|�C���^



	void ShadowDraw();

public:
	MainMove2(std::vector<int> v_file);
	~MainMove2();

	
	void Draw();
	void Process(unsigned __int8 controllNumber);
};


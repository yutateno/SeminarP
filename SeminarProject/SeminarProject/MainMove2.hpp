#pragma once
#include "Stage.hpp"
#include "CharacterSword.hpp"
#include "EnemyMove2.hpp"
#include "Camera.hpp"
#include "BaseMove.hpp"
#include "StageStairs.hpp"
#include "StageStreetLight.hpp"

#include <random>


class MainMove2 : public BaseMove
{
private:
	enum EFILE { stage, characterAttack, block, stairs, stairsColl, streetLight };			// ロードで渡されるファイル


	// ステージ
	Stage* p_stage;			// ステージのポインタ
	StageStairs* p_stageStairs[10];
	StageStreetLight* p_stageStreetLight[50];


	// キャラクター
	CharacterSword* p_character;		// キャラクターのポインタ


	// 一般人
	EnemyMove2* p_enemy;				// 敵のポインタ


	// カメラ
	Camera* p_camera;					// カメラのポインタ



	void ShadowDraw();

public:
	MainMove2(std::vector<int> v_file);
	~MainMove2();

	
	void Draw();
	void Process(unsigned __int8 controllNumber);
};


#pragma once
#include "Stage.hpp"
#include "Character.hpp"
#include "EnemyMove1.hpp"
#include "Camera.hpp"
#include "BaseMove.hpp"

#include <random>

class MainMove1 : public BaseMove
{
private:
	Stage* stage;
	Character* character;
	const int enemyNum = 30;
	struct SEnemyAggre
	{
		EnemyMove1* enemyMove;
		bool aliveNow;
	};
	SEnemyAggre enemyAggre[30];
	Camera* camera;

	void ActorHit();

	//void ShadowDraw();

	const int lightNum = 4;
	int lightHandle[4];			// ���C�g�n���h�����ێ�
	float lightRange[4];				// ���C�g�͈̔�
	VECTOR lightArea;				// ���C�g�̍��W

public:
	MainMove1(std::vector<int> file);
	~MainMove1();

	void Draw();
	void Process(unsigned __int8 controllNumber);
};

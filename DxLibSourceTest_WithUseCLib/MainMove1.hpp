#pragma once
#include "Stage.hpp"
#include "Character.hpp"
#include "EnemyMove1.hpp"
#include "Camera.hpp"
#include "PointLight.hpp"
#include "BaseMove.hpp"

#include <random>

class MainMove1 : public BaseMove
{
private:
	int enemyModel;

	Stage* stage;
	Character* character;
	const int enemyNum = 10;
	EnemyMove1* enemyAggre[10];
	Camera* camera;
	PointLight* light;

	void ActorHit();

public:
	MainMove1();
	~MainMove1();

	void Draw();
	void Process(unsigned __int8 controllNumber);
};

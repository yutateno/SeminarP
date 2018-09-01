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
	Stage* stage;
	Character* character;
	const int enemyNum = 30;
	EnemyMove1* enemyAggre[30];
	Camera* camera;
	PointLight* light;

	void ActorHit();

public:
	MainMove1(std::vector<int> file);
	~MainMove1();

	void Draw();
	void Process(unsigned __int8 controllNumber);
};

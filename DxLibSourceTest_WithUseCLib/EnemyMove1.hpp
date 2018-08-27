#pragma once
#include "BasicActor.hpp"

class EnemyMove1 : public BasicActor
{
public:
	EnemyMove1(int collStageHandle);
	~EnemyMove1();

	void Draw();
	void Process();
};

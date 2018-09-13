#pragma once
#include "BasicCreature.hpp"


class EnemyMove2 : public BasicCreature
{
private:
	

public:
	EnemyMove2(int collStageHandle, VECTOR area);
	~EnemyMove2();


	void Draw();
	void Process();
};


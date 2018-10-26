#pragma once
#include "BasicCreature.hpp"


class EnemyMove2 : public BasicCreature
{
private:


public:
	EnemyMove2(const int collStageHandle, const VECTOR area, const int modelHandle);
	~EnemyMove2();


	void Draw();
	void Process();

	void SetArea(const VECTOR area);
};


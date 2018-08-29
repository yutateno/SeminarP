#pragma once
#include "BasicActor.hpp"

class EnemyMove1 : public BasicActor
{
private:
	// “®‚«‚ÉŠÖ‚µ‚Ä
	bool upNow;
	float flyMove;
	void MoveProcess();

public:
	EnemyMove1(int modelHandle, int collStageHandle, float areaX, float areaZ);
	~EnemyMove1();

	void Draw();
	void Process();

	void SetArea(float areaX, float areaZ);
};

#pragma once
#include "BasicCreature.hpp"

class EnemyMove1 : public BasicCreature
{
private:
	// “®‚«‚ÉŠÖ‚µ‚Ä
	bool upNow;
	float flyMove;
	void MoveProcess();
	MATERIALPARAM material;
	bool viewNow;

public:
	EnemyMove1(int collStageHandle, float areaX, float areaZ, float color);
	~EnemyMove1();

	void Draw();
	void Process();

	void SetViewNow(bool viewNow);
};

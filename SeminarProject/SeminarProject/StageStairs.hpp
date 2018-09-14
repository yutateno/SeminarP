#pragma once
#include "BasicObject.hpp"


class StageStairs : public BasicObject
{
private:

public:
	StageStairs(int draw, int collStageHandle, VECTOR area);
	~StageStairs();


	void Draw();
};


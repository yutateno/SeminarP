#pragma once
#include "BasicObject.hpp"


class StageStairs : public BasicObject
{
private:

public:
	StageStairs(const int draw, const int collStageHandle, const VECTOR area);
	~StageStairs();


	void Draw();
};


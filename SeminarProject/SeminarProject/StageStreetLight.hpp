#pragma once
#include "BasicObject.hpp"


class StageStreetLight : public BasicObject
{
private:

public:
	StageStreetLight(int draw, int collStageHandle, VECTOR area);
	~StageStreetLight();


	void Draw();
};


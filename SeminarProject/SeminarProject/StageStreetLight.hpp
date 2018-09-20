#pragma once
#include "BasicObject.hpp"


class StageStreetLight : public BasicObject
{
private:

public:
	StageStreetLight(const int draw, const int collStageHandle, const VECTOR area);
	~StageStreetLight();


	void Draw();
};


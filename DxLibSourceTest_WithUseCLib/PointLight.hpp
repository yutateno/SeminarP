#pragma once
#include "DxLib.h"

class PointLight
{
private:
	float OutAngle, InAngle, Range, Atten0, Atten1, Atten2;

public:
	PointLight();
	~PointLight();

	void Process(VECTOR area);
	void Draw(VECTOR area);
};


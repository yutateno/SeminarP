#pragma once
#include "BasicObject.hpp"


class DropItemMove1 : public BasicObject
{
private:
	int draw;


public:
	DropItemMove1(int draw, int collStageHandle);
	~DropItemMove1();


	void Draw();
};


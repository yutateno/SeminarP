#pragma once
#include "Basic.hpp"
#include "LoadFile.hpp"


class BasicObject
{
protected:
	// 足元の影
	float shadowHeight;
	float shadowSize;
	void ShadowFoot();
	VECTOR area;

private:
	// 足元の影
	MV1_COLL_RESULT_POLY_DIM ShadowHitResDim;
	MV1_COLL_RESULT_POLY *ShadowHitRes;
	VERTEX3D ShadowVertex[3];
	VECTOR ShadowSlideVec;
	int shadowHandle;

	int stageHandle;


public:
	BasicObject(int collStageHandle);
	virtual ~BasicObject();
};


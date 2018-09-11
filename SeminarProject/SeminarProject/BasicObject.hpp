#pragma once
#include "Basic.hpp"
#include "LoadFile.hpp"


class BasicObject
{
protected:
	// ‘«Œ³‚Ì‰e
	float shadowHeight;
	float shadowSize;
	void ShadowFoot();
	VECTOR area;

private:
	// ‘«Œ³‚Ì‰e
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


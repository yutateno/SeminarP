#pragma once
#include "InputPad.hpp"
#include "BasicCreature.hpp"


class CharacterSword : public BasicCreature
{
private:
	// それぞれの位置に関して
	float direZAngle;		// 前後のキャラ向きを扱う変数
	float direXAngle;		// 左右のキャラ向きを扱う変数


	// モーションに関して
	enum MOTION { idle, action1, action2, action3, skyAction1, skyAction2, skyAction3, dash, walk, jump, fall
				, damage, death, gunAction, gunActionWalk, nagiharai, handUp};


	// 階段に関して
	int stairsHandle[10];
	VECTOR stairsArea[10];


	// 動きに関して
	bool walkNow;
	void MoveProcess(unsigned __int8 controllNumber);


	// 攻撃に関して
	bool attackNow;											// 今攻撃しているかどうか
	bool attackNext;										// 次のコンボへつなげるかどうか
	float attackFrame;										// 今の攻撃のフレーム数
	int attackNumber;										// 攻撃の番号
	int preAttackNumber;									// 直前の攻撃の番号
	void AttackProcess(unsigned __int8 controllNumber);		// 攻撃のプロセス


	// ジャンプに関して
	bool jumpNow;
	bool jumpUpNow;
	float jumpPower;
	float gravity;
	float flyJumpPower;
	float fallJumpPower;
	void JumpProcess(unsigned __int8 controllNumber);


	// 行動によってアニメーションの管理
	void AnimProcess();


public:
	CharacterSword(const int modelHandle, const int collStageHandle, const int stairsHandle);
	~CharacterSword();


	void SetStairsArea(const VECTOR stairsArea, const int num);


	void Draw();
	void Process(const unsigned __int8 controllNumber, const float getAngle);


	void PositionReset();


	const VECTOR GetPreArea() const;
};


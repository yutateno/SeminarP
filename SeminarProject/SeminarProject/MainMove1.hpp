#pragma once
#include "Stage.hpp"
#include "Character.hpp"
#include "EnemyMove1.hpp"
#include "Camera.hpp"
#include "BaseMove.hpp"

#include <random>

class MainMove1 : public BaseMove
{
private:
	enum EFILE1 { drawStage, collStage, character, feedWhite };				// ロードから渡されるファイルの順番

	Stage* p_stage;					// ステージのポインタ
	Character* p_character;			// キャラクターのポインタ
	const int enemyNum = 30;		// 敵の数

	struct EnemyAggre
	{
		EnemyMove1* p_enemyMove;		// 敵のポインタ
		bool aliveNow;					// 生きているか
	};
	EnemyAggre s_enemyAggre[30];		// 敵の構造体を所持
	

	Camera* p_camera;					// カメラのポインタ

	void ActorHit();				// アクター同士のあたり判定(簡易)
	int catchEnemyNum;				// 敵を手に入れた数

	//void ShadowDraw();			// 影

	const int lightNum = 4;			// ライトハンドルの数
	int lightHandle[4];			// ライトハンドル情報保持
	float lightRange[4];				// ライトの範囲
	VECTOR lightArea[4];				// ライトの座標
	void LightProcess();				// ライトに関する関数

	bool lightEventStart;				// イベントを行う
	bool lightEventEnd;					// イベントの終了を確認
	int lightEventCount;				// イベントのカウント
	bool lightEnd;						// ライトを消す
	float lightRangePreMax;
	float lightRangeSpeed;

	int backgroundColor;		// 背景色

	int drawWhite;			// フェードイン用ホワイト画像


#ifdef _DEBUG
	void DebugKeyControll();
#endif // _DEBUG


public:
	MainMove1(std::vector<int> v_file);			// コンストラクタ
	~MainMove1();							// デストラクタ


	void Draw();										// 描画
	void Process(unsigned __int8 controllNumber);		// プロセス
};
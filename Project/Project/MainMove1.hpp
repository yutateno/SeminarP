#pragma once
#include "Stage.hpp"
#include "Character.hpp"
#include "EnemyMove1.hpp"
#include "Camera.hpp"
#include "BaseMove.hpp"
#include "DropItemMove1.hpp"

#include <random>


/*
光を集めるシーン
浮いている玉を集めて光を手に入れる
*/
class MainMove1 : public BaseMove
{
private:
	enum EFILE { drawStage, collStage, character, sword, sound };				// ロードから渡されるファイルの順番

	// ステージ
	Stage* p_stage;					// ステージのポインタ


	// キャラクター
	Character* p_character;			// キャラクターのポインタ
	int catchEnemyNum;				// 敵を手に入れた数


	// 敵
	const int enemyNum = 30;		// 敵の数
	struct EnemyAggre
	{
		EnemyMove1* p_enemyMove;		// 敵のポインタ
		bool aliveNow;					// 生きているか
	};
	EnemyAggre s_enemyAggre[30];		// 敵の構造体を所持


	// カメラ
	Camera* p_camera;					// カメラのポインタ


	// 簡単なあたり判定
	void ActorHit();				// アクター同士のあたり判定(簡易)


	// ライトの基本
	const int lightNum = 4;				// ライトハンドルの数
	int lightHandle[4];					// ライトハンドル情報保持
	float lightRange[4];				// ライトの範囲
	VECTOR lightArea[4];				// ライトの座標
	// ライトに変化を加える
	void LightProcess();				// ライトに関する関数
	bool lightEventStart;				// イベントを行う
	bool lightEventEnd;					// イベントの終了を確認
	int lightEventCount;				// イベントのカウント
	bool lightEnd;						// ライトを消す
	float lightRangePreMax;				// 光源の広さの直前マックス
	float lightRangeSpeed;				// 光源の広さを広げるスピード


	// 背景色
	int backgroundColor;		// 背景色


	// 落ちてる剣
	DropItemMove1* p_dropItem;		// 落ちるアイテム（剣）
	bool touchSword;				// 落ちてるアイテムに触れる


	// サウンド
	int soundBG;


#ifdef _DEBUG
	void DebugKeyControll();
#endif // _DEBUG


public:
	MainMove1(const std::vector<int> v_file);			// コンストラクタ
	~MainMove1();							// デストラクタ


	void Draw();										// 描画
	void Process(const unsigned __int8 controllNumber);		// プロセス
	void CameraProcess();
};
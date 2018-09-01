#pragma once
#include "BaseMove.hpp"
#include "MainMove1.hpp"
#include "LoadThread.hpp"

class Manager
{
private:
	ESceneNumber preScene;
	ESceneNumber nowScene;

	BaseMove* baseMove;
	LoadThread* loadThread;

	void SceneChange();


	const int max1 = 3;
	std::string move1str[3];
	ELOADFILE load1[3];

public:
	Manager();
	~Manager();

	void Update(unsigned __int8 controllNumber);
};


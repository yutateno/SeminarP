#include "LoadThread.hpp"

using namespace std;

std::mutex mtx;

void LoadThread::MyNextLoad(const std::string path, int& file, const ELOADFILE type)
{
	std::lock_guard<std::mutex> lock(mtx);

	LoadFile::MyLoad(path, file, type);
}

LoadThread::LoadThread()
{
	num = 0;
	p_loadScreen = NULL;
	p_loadScreen = new LoadScreen();
}

LoadThread::~LoadThread()
{
	POINTER_RELEASE(p_loadScreen);
}

void LoadThread::Process(const int max, const std::string* path, const ELOADFILE* type)
{
	if (num < max)
	{
		fileName.push_back(0);
		ths = std::thread(&LoadThread::MyNextLoad, this, path[num], ref(fileName[num]), type[num]);
		ths.join();
		num++;
		ClearDrawScreen();
		p_loadScreen->Process(num, max);			// ロード画面
		ScreenFlip();
		Process(max, path, type);
	}
	p_loadScreen->Process(num, max);			// ロード画面
}

const vector<int> LoadThread::GetFile() const
{
	return fileName;
}

const int LoadThread::GetNum() const
{
	return num;
}

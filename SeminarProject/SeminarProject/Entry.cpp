#include <windows.h>
#include "DIRECTOR.h"

INT WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, INT)
{
	DIRECTOR* pDirector = new DIRECTOR;
	if (pDirector == NULL)
	{
		MessageBox(0, L"�N���X�������s�@�A�v�����I�����܂�", NULL, MB_OK);
		return 0;
	}
	//�i�s�̓f�B���N�^�[��
	pDirector->Run(hInstance);

	//�A�v���I��
	delete pDirector;

	return 0;
}
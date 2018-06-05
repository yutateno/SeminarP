#pragma once
#define _WIN32_DCOM
#include <stdio.h>
#include <windows.h>
#include <mmsystem.h>
#include <xaudio2.h>
#include "BASE.h"
#include "WINDOW.h"

#pragma comment(lib,"winmm.lib")

#define MAX_WAV 110 //WAV�T�E���h�ő吔

//
//
//
class SOUND : public CELEMENT
{
public:
	HWND m_hWnd;
	IXAudio2* m_pXAudio2;
	IXAudio2MasteringVoice* m_pMasteringVoice;
	IXAudio2SourceVoice* m_pSourceVoice[MAX_WAV];
	BYTE* m_pWavBuffer[MAX_WAV];//�g�`�f�[�^�i�t�H�[�}�b�g�����܂܂Ȃ��A�����ɔg�`�f�[�^�̂݁j
	DWORD m_dwWavSize[MAX_WAV];//�g�`�f�[�^�̃T�C�Y
	int m_iSoundIndex[11];//����ǂݍ��މ��̔ԍ�
	int m_iSoundCurrentIndex;//���̌��݂̍Đ��ԍ�

	SOUND();
	~SOUND();
	HRESULT Init();
	int LoadSound(char* szFileName);
	void  PlaySound(int iSoundIndex, bool boLoop);
	void StopSound(int iSoundIndex);
};
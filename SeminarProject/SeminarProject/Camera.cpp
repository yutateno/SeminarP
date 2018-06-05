#include "CAMERA.h"

//
//
//
CAMERA::CAMERA()
{
	ZeroMemory(this, sizeof(CAMERA));
}

//
//
//
CAMERA::~CAMERA()
{
}
//
//
//
void CAMERA::SetCameraPosition(float x, float y, float z)
{
	m_fX = x;
	m_fY = y;
	m_fZ = z;
	m_vPos = D3DXVECTOR3(x, y, z);
	SetViewProj();
}
//
//
//
void CAMERA::SetCameraPositionGaze(float x, float y, float z, float gx, float gy, float gz)
{
	m_fX = x;
	m_fY = y;
	m_fZ = z;
	m_vPos = D3DXVECTOR3(x, y, z);
	m_fGX = gx;
	m_fGY = gy;
	m_fGZ = gz;

	m_boGaze = true;
	SetViewProj();
}
//
//
//
HRESULT CAMERA::SetViewProj()
{
	// �r���[
	if (!m_boGaze)
	{
		D3DXVECTOR3 vEyePt(m_fX, m_fY, m_fZ);//�J�����i���_�j�ʒu
		D3DXVECTOR3 vLookatPt(m_fX, m_fY, m_fZ + 1);//�����ʒu
		D3DXVECTOR3 vUpVec(0.0f, 1.0f, 0.0f);//����ʒu

		D3DXMatrixLookAtLH(&m_mView, &vEyePt, &vLookatPt, &vUpVec);
	}
	else
	{
		D3DXVECTOR3 vEyePt(m_fX, m_fY, m_fZ);//�J�����i���_�j�ʒu
		D3DXVECTOR3 vLookatPt(m_fGX, m_fGY, m_fGZ);//�����ʒu
		D3DXVECTOR3 vUpVec(0.0f, 1.0f, 0.0f);//����ʒu

		D3DXMatrixLookAtLH(&m_mView, &vEyePt, &vLookatPt, &vUpVec);
	}
	m_boGaze = false;
	// �v���W�F�N�V����

	D3DXMatrixPerspectiveFovLH(&m_mProj, D3DX_PI / 4, (FLOAT)m_dwWindowWidth / (FLOAT)m_dwWindowHeight, 0.1f, 1100.0f);

	return S_OK;
}
//
//
//
HRESULT CAMERA::Init(DWORD dwWidth, DWORD dwHeight)
{
	m_dwWindowWidth = dwWidth;
	m_dwWindowHeight = dwHeight;
	return S_OK;
}

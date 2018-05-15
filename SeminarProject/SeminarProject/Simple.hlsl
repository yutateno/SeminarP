//�O���[�o��
cbuffer global
{
	matrix g_WVP; //���[���h����ˉe�܂ł̕ϊ��s��
};

//
//
//�o�[�e�b�N�X�V�F�[�_�[
float4 VS(float4 Pos : POSITION) : SV_POSITION
{
	Pos = mul(Pos,g_WVP);
return Pos;
}

//
//
//�s�N�Z���V�F�[�_�[
float4 PS(float4 Pos : SV_POSITION) : SV_Target
{
	return float4(1,1,1,1);
}

// �������I�����炱��ɕς��āA���Ԃ���v
// ���������ĂȂ���Ԃł��ƃ������j�]�N��������
/*#define MAX_LIGHT 100
#define ATTENU 2

Texture2D g_texColor : register(t0);
SamplerState g_samLinear : register(s0);

cbuffer global_0 : register(b0)
{
	matrix g_mW;
	matrix g_mWVP;
	float4 g_vLightPos[MAX_LIGHT];
	float4 g_vEye;
};

cbuffer global_1 : register(b1)
{
	float4 g_Ambient = float4(0, 0, 0, 0);
	float4 g_Diffuse = float4(1, 0, 0, 0);
	float4 g_Specular = float4(1, 1, 1, 1);
};

// ���_�V�F�[�_�̏o��
struct VS_OUTPUT
{
	float4 Pos : SV_POSITION;
	float3 vWorldPos : POSITION;
	float3 Normal : TEXCOORD1;
	float2 Tex : TEXCOORD3;
};

// ���_�V�F�[�_
VS_OUTPUT VS(float4 Pos : POSITION, float4 Norm : NORMAL, float2 Tex : TEXCOORD)
{
	VS_OUTPUT output = (VS_OUTPUT)0;

	output.Normal = normalize(mul(Norm, (float3x3)g_mW));
	output.Pos = mul(Pos, g_mWVP);
	output.vWorldPos = mul(Pos, g_mW);
	output.Tex = Tex;

	return output;
}

// ���C�g�̖��邳����
float4 PLight(float3 Pos, float3 LPos, float3 Normal, float2 UV, float3 vEyeVector, float4 LightColor)
{
	float3 vLightDir = LPos - Pos;
	float Distance = length(vLightDir);
	vLightDir = normalize(vLightDir);

	float4 vDiffuse = g_texColor.Sample(g_samLinear, UV);
	float3 vDiffuseIntensity = saturate(dot(vLightDir, Normal));
	float3 vSpecularIntensity = pow(max(0, dot(vEyeVector, reflect(-vLightDir, Normal))), 2);

	float4 FinalColor;
	FinalColor.rgb = vDiffuseIntensity * (vDiffuse + LightColor) + vSpecularIntensity * g_Specular;
	FinalColor.a = 1;

	FinalColor *= pow(saturate(ATTENU / Distance), 4);

	return FinalColor;
}

// �s�N�Z���V�F�[�_
float4 PS(VS_OUTPUT input) : SV_Target
{
	float4 FinalColor = (float4)0;

	for (int i = 0; i < MAX_LIGHT; i++)
	{
		if (length(g_vLightPos[i] - input.vWorldPos) < ATTENU * 2)
		{
			FinalColor += PLight(input.vWorldPos[i], input.Normal, input.Tex, normalize(g_vEye - input.vWorldPos), input.Tex.yxy);
		}
	}

	return FinalColor;
}*/
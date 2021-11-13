Shader "Unlit/PointCloudShaderCamera2"
{
    Properties
    {
        [HDR] _Color ("Tint", Color) = (0, 0, 0, 1)
        _MainTex ("Texture", 2D) = "white" {}
    }
    SubShader
    {
        Tags {"Queue"="Transparent" "IgnoreProjector"="True" "RenderType"="Transparent"}
        Blend SrcAlpha OneMinusSrcAlpha
        
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            StructuredBuffer<float3> vertexPosition;
            StructuredBuffer<int> vertexIndiced;
            StructuredBuffer<float4> vertexColor;
            Texture2D<float4> colorTexture;
            float4x4 cameraTransform;

            sampler2D _MainTex;
			float4    _MainTex_ST;
            float4 _Color;

            struct appdata
			{
				float4 vertex : POSITION;
				float2 uv     : TEXCOORD0;
			};

            // vertex shader outputs ("vertex to fragment")
            struct v2f 
            {
                float2 uv : TEXCOORD0;
                fixed4 color : SV_Target;
                float4 pos : SV_POSITION;
                float dis    : TEXCOORD1;
            };

            v2f vert(uint vertex_id: SV_VertexID)
            {
                v2f test;
                float3 position = vertexPosition[vertex_id];
                fixed4 color = (fixed4)vertexColor[vertex_id];
                if(position[2] > 0)
                {
                    test.color = color;
                }
                else
                {
                    test.color = fixed4(1,1,1,0);
                }
                
                test.pos = mul(UNITY_MATRIX_VP, float4(position, 1));
                return test;
            }
            
            fixed4 frag(v2f i) : SV_TARGET
            {
                return i.color;
            }
            ENDCG
        }
    }
}

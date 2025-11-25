Shader "Custom/PointCloudGeometryShader"
{
    Properties
    {
        _PointScale ("Point Scale", Range(0.001, 10.0)) = 1.0
        _MinPointSize ("Min Point Size (pixels)", Range(0.1, 5.0)) = 0.5
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma geometry geom

            #include "UnityCG.cginc"

            float _PointScale;
            float _MinPointSize;

            struct appdata
            {
                float4 vertex : POSITION;
                fixed4 color : COLOR;
            };

            struct v2g
            {
                float4 pos : SV_POSITION;
                fixed4 color : COLOR;
                float size : TEXCOORD0;
            };

            // Vertex Shader
            v2g vert(appdata v)
            {
                v2g o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.color = fixed4(v.color.rgb, 1.0); // 保留 RGB，alpha 設為 1
                o.size = v.color.a; // 把原本的 alpha (點大小) 存到 size
                return o;
            }

            // Geometry Shader
            [maxvertexcount(4)]
            void geom(point v2g p[1], inout TriangleStream<v2g> triStream)
            {
                // 計算點大小：原始大小 * 全域縮放
                float baseSize = p[0].size * _PointScale;
                
                // 轉換到像素單位，確保最小大小
                float2 pixelSize = float2(2.0 / _ScreenParams.x, 2.0 / _ScreenParams.y);
                float pointSizePixels = max(baseSize * _ScreenParams.y, _MinPointSize);
                
                float2 halfSize = pointSizePixels * pixelSize * 0.5;

                v2g v0, v1, v2, v3;
                v0.color = v1.color = v2.color = v3.color = p[0].color;
                v0.size = v1.size = v2.size = v3.size = p[0].size;

                // 建立四邊形 (使用 triangle strip)
                v0.pos = p[0].pos + float4(-halfSize.x, -halfSize.y, 0, 0);
                v1.pos = p[0].pos + float4( halfSize.x, -halfSize.y, 0, 0);
                v2.pos = p[0].pos + float4(-halfSize.x,  halfSize.y, 0, 0);
                v3.pos = p[0].pos + float4( halfSize.x,  halfSize.y, 0, 0);

                triStream.Append(v0);
                triStream.Append(v1);
                triStream.Append(v2);
                triStream.Append(v3);
            }

            // Fragment Shader
            fixed4 frag(v2g i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
}
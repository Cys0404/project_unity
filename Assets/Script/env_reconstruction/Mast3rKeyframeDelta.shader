Shader "Hidden/Mast3rKeyframeDelta"
{
    Properties
    {
        _PointSize ("Point Size (px)", Float) = 3.0
        _Tint ("Tint", Color) = (1,1,1,1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        Pass
        {
            ZWrite On
            Cull Off
            CGPROGRAM
            #pragma target 4.5
            #pragma vertex vert
            #pragma geometry geom
            #pragma fragment frag
            #include "UnityCG.cginc"

            StructuredBuffer<float3> _Positions;
            StructuredBuffer<uint> _Colors;
            float4x4 _KeyframeTransform;
            float _PointSize;
            fixed4 _Tint;

            struct v2g { float4 clipPos : SV_POSITION; fixed4 col : COLOR; };
            struct g2f { float4 clipPos : SV_POSITION; fixed4 col : COLOR; };

            fixed4 DecodeColor(uint rgb)
            {
                return fixed4(
                    ((rgb >> 16) & 255) / 255.0,
                    ((rgb >> 8) & 255) / 255.0,
                    (rgb & 255) / 255.0,
                    1.0);
            }

            v2g vert(uint id : SV_VertexID)
            {
                v2g output;
                float4 world = mul(_KeyframeTransform, float4(_Positions[id], 1.0));
                output.clipPos = mul(UNITY_MATRIX_VP, world);
                output.col = DecodeColor(_Colors[id]) * _Tint;
                return output;
            }

            [maxvertexcount(4)]
            void geom(point v2g input[1], inout TriangleStream<g2f> stream)
            {
                float4 position = input[0].clipPos;
                float2 pixel = float2(2.0 / _ScreenParams.x, 2.0 / _ScreenParams.y);
                // Match Hidden/PointCloudGPU exactly for an apples-to-apples
                // FULL/DELTA comparison.  Both Inspector point sizes now mean
                // the same rendered quad size at the same depth.
                float2 dx = float2(_PointSize, 0.0) * pixel;
                float2 dy = float2(0.0, _PointSize) * pixel;
                g2f output;
                output.col = input[0].col;
                output.clipPos = position + float4(-dx - dy, 0, 0); stream.Append(output);
                output.clipPos = position + float4( dx - dy, 0, 0); stream.Append(output);
                output.clipPos = position + float4( dx + dy, 0, 0); stream.Append(output);
                output.clipPos = position + float4(-dx + dy, 0, 0); stream.Append(output);
                stream.RestartStrip();
            }

            fixed4 frag(g2f input) : SV_Target { return input.col; }
            ENDCG
        }
    }
}

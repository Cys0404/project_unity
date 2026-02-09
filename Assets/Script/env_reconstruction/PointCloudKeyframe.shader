Shader "Custom/PointCloudKeyframe"
{
    Properties 
    { 
        _PointSize ("Point Size (px)", Float) = 3.0  
        _Tint ("Tint", Color) = (1,1,1,1) 
    }
    SubShader {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        Pass {
            ZWrite On  
            Cull Off
            CGPROGRAM
            #pragma target 4.0
            #pragma vertex   vert
            #pragma geometry geom
            #pragma fragment frag
            #include "UnityCG.cginc"

            StructuredBuffer<float3> _Positions;
            StructuredBuffer<uint>   _Colors;   // 0xRRGGBB
            float4x4 _KeyframeTransform;        // Per-keyframe transform (includes scale)
            float    _PointSize; 
            fixed4   _Tint;

            struct v2g { float4 clipPos : SV_POSITION; fixed4 col : COLOR; };
            struct g2f { float4 clipPos : SV_POSITION; fixed4 col : COLOR; };

            fixed4 DecodeColor(uint rgb){
                float r = ((rgb >> 16) & 255) / 255.0;
                float g = ((rgb >> 8)  & 255) / 255.0;
                float b = ((rgb >> 0)  & 255) / 255.0;
                return fixed4(r,g,b,1);
            }

            v2g vert(uint id : SV_VertexID)
            {
                v2g o;
                float3 p_local = _Positions[id];
                // Apply keyframe transform (T_WC * scale)
                float4 p_world = mul(_KeyframeTransform, float4(p_local, 1));
                o.clipPos = mul(UNITY_MATRIX_VP, p_world);
                o.col = DecodeColor(_Colors[id]) * _Tint;
                return o;
            }

            [maxvertexcount(4)]
            void geom(point v2g IN[1], inout TriangleStream<g2f> triStream)
            {
                float4 pos = IN[0].clipPos;
                float2 ndcPerPixel = float2(2.0/_ScreenParams.x, 2.0/_ScreenParams.y);
                float2 dx = float2(_PointSize,0) * ndcPerPixel;
                float2 dy = float2(0,_PointSize) * ndcPerPixel;
                g2f o; o.col = IN[0].col;
                o.clipPos = pos + float4(-dx - dy,0,0); triStream.Append(o);
                o.clipPos = pos + float4( dx - dy,0,0); triStream.Append(o);
                o.clipPos = pos + float4( dx + dy,0,0); triStream.Append(o);
                o.clipPos = pos + float4(-dx + dy,0,0); triStream.Append(o);
                triStream.RestartStrip();
            }
            
            fixed4 frag(g2f i):SV_Target{ return i.col; }
            ENDCG
        }
    }
}

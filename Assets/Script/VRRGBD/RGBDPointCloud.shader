Shader "VRRGBD/RGBD Point Cloud"
{
    Properties
    {
        _PointSizeMeters ("Point Size Meters", Float) = 0.008
        _MaxDepthMeters ("Max Depth Meters", Float) = 3.0
        _ProjectionMode ("Projection Mode", Float) = 1
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        Cull Off
        ZWrite On

        Pass
        {
            CGPROGRAM
            #pragma target 4.0
            #pragma vertex vert
            #pragma geometry geom
            #pragma fragment frag

            #include "UnityCG.cginc"

            sampler2D _ColorTex;
            sampler2D _DepthTex;
            float4 _Intrinsics; // fx, fy, cx, cy
            float4 _ImageSize;  // width, height, 1/width, 1/height
            float _DepthUnitMeters;
            float _MaxDepthMeters;
            float _PointSizeMeters;
            float _ProjectionMode;

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2g
            {
                float4 clip : SV_POSITION;
                float3 world : TEXCOORD0;
                float2 uv : TEXCOORD1;
                float valid : TEXCOORD2;
            };

            struct g2f
            {
                float4 clip : SV_POSITION;
                float2 uv : TEXCOORD0;
                float2 colorUv : TEXCOORD1;
            };

            v2g vert(appdata v)
            {
                v2g o;
                float raw = tex2Dlod(_DepthTex, float4(v.uv, 0, 0)).r;
                float z = raw * 65535.0 * _DepthUnitMeters;

                float u = v.uv.x * _ImageSize.x - 0.5;
                float yPixel = (1.0 - v.uv.y) * _ImageSize.y - 0.5;
                float x = (u - _Intrinsics.z) * z / _Intrinsics.x;
                float yRosDown = (yPixel - _Intrinsics.w) * z / _Intrinsics.y;
                float y = -yRosDown;

                float3 localPos = float3(x, y, z);
                if (_ProjectionMode > 0.5)
                {
                    localPos = float3(x, z, yRosDown);
                }
                float4 world = mul(unity_ObjectToWorld, float4(localPos, 1.0));
                o.clip = UnityWorldToClipPos(world.xyz);
                o.world = world.xyz;
                o.uv = v.uv;
                o.valid = z > 0.02 && z < _MaxDepthMeters ? 1.0 : 0.0;
                return o;
            }

            [maxvertexcount(4)]
            void geom(point v2g input[1], inout TriangleStream<g2f> stream)
            {
                if (input[0].valid < 0.5) return;

                float3 camRight = UNITY_MATRIX_I_V._m00_m01_m02;
                float3 camUp = UNITY_MATRIX_I_V._m10_m11_m12;
                float halfSize = _PointSizeMeters * 0.5;
                float3 center = input[0].world;

                float3 offsets[4] =
                {
                    (-camRight - camUp) * halfSize,
                    (-camRight + camUp) * halfSize,
                    ( camRight - camUp) * halfSize,
                    ( camRight + camUp) * halfSize
                };

                float2 uvs[4] =
                {
                    float2(0, 0),
                    float2(0, 1),
                    float2(1, 0),
                    float2(1, 1)
                };

                [unroll]
                for (int i = 0; i < 4; i++)
                {
                    g2f o;
                    o.clip = UnityWorldToClipPos(center + offsets[i]);
                    o.uv = uvs[i];
                    o.colorUv = input[0].uv;
                    stream.Append(o);
                }
            }

            fixed4 frag(g2f i) : SV_Target
            {
                fixed4 c = tex2D(_ColorTex, i.colorUv);
                return fixed4(c.rgb, 1);
            }
            ENDCG
        }
    }
}

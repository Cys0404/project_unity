Shader "Custom/PointCloudGeometryShader"
{
    Properties
    {
        
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

            struct appdata
            {
                float4 vertex : POSITION;
                fixed4 color : COLOR;
            };

            struct v2g
            {
                float4 pos : SV_POSITION;
                fixed4 color : COLOR;
            };

            // Vertex Shader
            v2g vert(appdata v)
            {
                v2g o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.color = v.color;
                return o;
            }

            // Geometry Shader
            [maxvertexcount(8)]
            void geom(point v2g p[1], inout TriangleStream<v2g> triStream)
            {
                float pointSize = p[0].color.a;
                v2g v[8];
                float halfSize = pointSize * 0.5;

                // Define offsets for a triangles)
                float2 offsets[8] = {
                    // First triangle
                    float2(-halfSize, halfSize), // Top-left
                    float2(halfSize, halfSize), // Top-right
                    float2(-halfSize, -halfSize), // Bottom-left
                    float2(-halfSize, halfSize), // Top-left
                    // Second triangle
                    float2(halfSize, halfSize), // Top-right
                    float2(halfSize, -halfSize), // Bottom-right
                    float2(-halfSize, -halfSize), // Bottom-left
                    float2(halfSize, halfSize), // Top-right
                };

                for (int i = 0; i < 8; i++)
                {
                    v[i] = p[0]; // Copy input vertex to all vertices of the quad
                    v[i].pos.xy += offsets[i]; // Apply offsets to position
                    triStream.Append(v[i]); // Append vertex to triangle stream
                    if (i == 3) triStream.RestartStrip(); // Start second triangle
                }
            }

            // Fragment Shader
            fixed4 frag(v2g i) : SV_Target
            {
                // Output the color passed through from the vertex shader
                return i.color;
            }
            ENDCG
        }
    }
}
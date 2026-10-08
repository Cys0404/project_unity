using System;
using System.Globalization;
using System.Text.RegularExpressions;

namespace VRRGBD
{
    public static class RgbdHeaderParser
    {
        public static RgbdFrameHeader Parse(string json)
        {
            var header = new RgbdFrameHeader
            {
                camera_id = ReadString(json, "camera_id", "cam0"),
                frame_id = ReadString(json, "frame_id", ReadIntegerToken(json, "frame_id")),
                ros_time = ReadDouble(json, "ros_time", 0.0),
                send_time_unix = ReadDouble(json, "send_time_unix", 0.0)
            };

            string colorJson = ReadObject(json, "color");
            string depthJson = ReadObject(json, "depth");
            header.color.width = ReadInt(colorJson, "width", ReadInt(json, "color_width", 0));
            header.color.height = ReadInt(colorJson, "height", ReadInt(json, "color_height", 0));
            header.color.format = ReadString(colorJson, "format", "jpeg");

            header.depth.width = ReadInt(depthJson, "width", ReadInt(json, "depth_width", header.color.width));
            header.depth.height = ReadInt(depthJson, "height", ReadInt(json, "depth_height", header.color.height));
            header.depth.format = ReadString(depthJson, "format", "png16");
            header.depth.unit_m = ReadFloat(ReadObject(depthJson, "unit"), "m", ReadFloat(depthJson, "unit_m", ReadFloat(json, "unit_m", 0.001f)));

            string intrinsicsJson = ReadObject(json, "camera");
            if (string.IsNullOrEmpty(intrinsicsJson)) intrinsicsJson = ReadObject(json, "intrinsics");
            header.camera.k = ReadFloatArray(intrinsicsJson, "k");
            header.camera.d = ReadFloatArray(intrinsicsJson, "d");

            header.camera.fx = ReadFloat(intrinsicsJson, "fx", header.camera.k.Length > 0 ? header.camera.k[0] : 0f);
            header.camera.fy = ReadFloat(intrinsicsJson, "fy", header.camera.k.Length > 4 ? header.camera.k[4] : 0f);
            header.camera.cx = ReadFloat(intrinsicsJson, "cx", header.camera.k.Length > 2 ? header.camera.k[2] : 0f);
            header.camera.cy = ReadFloat(intrinsicsJson, "cy", header.camera.k.Length > 5 ? header.camera.k[5] : 0f);

            if (header.camera.fx <= 0f || header.camera.fy <= 0f)
            {
                header.camera.fx = ReadFloat(json, "fx", header.camera.fx);
                header.camera.fy = ReadFloat(json, "fy", header.camera.fy);
                header.camera.cx = ReadFloat(json, "cx", header.camera.cx);
                header.camera.cy = ReadFloat(json, "cy", header.camera.cy);
            }

            return header;
        }

        public static string ReadString(string json, string key, string fallback = "")
        {
            if (string.IsNullOrEmpty(json)) return fallback;
            var match = Regex.Match(json, Quote(key) + "\\s*:\\s*\"(?<v>(?:\\\\.|[^\"])*)\"");
            return match.Success ? Regex.Unescape(match.Groups["v"].Value) : fallback;
        }

        public static int ReadInt(string json, string key, int fallback = 0)
        {
            double value = ReadDouble(json, key, fallback);
            return (int)value;
        }

        private static string ReadIntegerToken(string json, string key)
        {
            if (string.IsNullOrEmpty(json)) return string.Empty;
            var match = Regex.Match(json, Quote(key) + "\\s*:\\s*(?<v>-?\\d+)(?=\\s*[,}])");
            return match.Success ? match.Groups["v"].Value : string.Empty;
        }

        public static float ReadFloat(string json, string key, float fallback = 0f)
        {
            return (float)ReadDouble(json, key, fallback);
        }

        public static double ReadDouble(string json, string key, double fallback = 0.0)
        {
            if (string.IsNullOrEmpty(json)) return fallback;
            var match = Regex.Match(json, Quote(key) + "\\s*:\\s*(?<v>-?\\d+(?:\\.\\d+)?(?:[eE][+-]?\\d+)?)");
            return match.Success && double.TryParse(match.Groups["v"].Value, NumberStyles.Float, CultureInfo.InvariantCulture, out double value)
                ? value
                : fallback;
        }

        public static float[] ReadFloatArray(string json, string key)
        {
            if (string.IsNullOrEmpty(json)) return Array.Empty<float>();
            var match = Regex.Match(json, Quote(key) + "\\s*:\\s*\\[(?<v>[^\\]]*)\\]");
            if (!match.Success) return Array.Empty<float>();

            string[] tokens = match.Groups["v"].Value.Split(',');
            var values = new float[tokens.Length];
            int count = 0;
            for (int i = 0; i < tokens.Length; i++)
            {
                if (float.TryParse(tokens[i].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out float value))
                {
                    values[count++] = value;
                }
            }

            if (count == values.Length) return values;
            Array.Resize(ref values, count);
            return values;
        }

        public static string ReadObject(string json, string key)
        {
            if (string.IsNullOrEmpty(json)) return string.Empty;
            var match = Regex.Match(json, Quote(key) + "\\s*:\\s*\\{");
            if (!match.Success) return string.Empty;

            int start = match.Index + match.Length - 1;
            int depth = 0;
            bool inString = false;
            bool escaped = false;
            for (int i = start; i < json.Length; i++)
            {
                char c = json[i];
                if (escaped)
                {
                    escaped = false;
                    continue;
                }
                if (c == '\\' && inString)
                {
                    escaped = true;
                    continue;
                }
                if (c == '"')
                {
                    inString = !inString;
                    continue;
                }
                if (inString) continue;
                if (c == '{') depth++;
                if (c == '}')
                {
                    depth--;
                    if (depth == 0) return json.Substring(start, i - start + 1);
                }
            }

            return string.Empty;
        }

        private static string Quote(string key)
        {
            return "\"" + Regex.Escape(key) + "\"";
        }
    }
}

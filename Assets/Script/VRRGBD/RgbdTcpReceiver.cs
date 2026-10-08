using System;
using System.Collections.Concurrent;
using System.IO;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace VRRGBD
{
    public sealed class RgbdTcpReceiver : MonoBehaviour
    {
        [Header("RGB-D TCP")]
        public string host = "192.168.1.10";
        public int port = 50010;
        public bool connectOnStart = true;
        public int reconnectDelayMs = 1000;
        public int receiveBufferBytes = 4 * 1024 * 1024;

        [Header("Safety")]
        public int maxHeaderBytes = 128 * 1024;
        public int maxRgbBytes = 8 * 1024 * 1024;
        public int maxDepthBytes = 8 * 1024 * 1024;
        public bool verbose;

        private readonly ConcurrentDictionary<string, RgbdFrame> latestFrames = new ConcurrentDictionary<string, RgbdFrame>();
        private readonly ConcurrentDictionary<string, RgbdReceiverCameraDebug> cameraDebugById = new ConcurrentDictionary<string, RgbdReceiverCameraDebug>();
        private Thread worker;
        private TcpClient activeClient;
        private readonly object connectionLock = new object();
        private volatile bool running;
        private uint sequence;

        public bool IsConnected { get; private set; }
        public uint ReceivedFrames { get; private set; }
        public uint DroppedFrames { get; private set; }
        public ICollection<string> LatestCameraIds => latestFrames.Keys;
        [Header("Runtime Debug")]
        public List<RgbdReceiverCameraDebug> cameraDebug = new List<RgbdReceiverCameraDebug>();

        private void Start()
        {
            if (connectOnStart) Connect();
        }

        public void Connect()
        {
            if (running) return;
            running = true;
            worker = new Thread(ReceiveLoop) { IsBackground = true, Name = "RGBD TCP Receiver" };
            worker.Start();
        }

        public bool TryConsumeLatest(string cameraId, out RgbdFrame frame)
        {
            return latestFrames.TryRemove(cameraId, out frame);
        }

        public bool TryPeekLatest(string cameraId, out RgbdFrame frame)
        {
            return latestFrames.TryGetValue(cameraId, out frame);
        }

        private void ReceiveLoop()
        {
            while (running)
            {
                try
                {
                    using (var client = new TcpClient())
                    {
                        lock (connectionLock)
                        {
                            if (!running) return;
                            activeClient = client;
                        }
                        client.NoDelay = true;
                        client.ReceiveBufferSize = receiveBufferBytes;
                        client.Connect(host, port);
                        client.ReceiveTimeout = 3000;
                        IsConnected = true;

                        using (NetworkStream stream = client.GetStream())
                        {
                            while (running && client.Connected)
                            {
                                RgbdFrame frame = ReadFrame(stream);
                                if (frame == null) break;

                                frame.sequence = ++sequence;
                                frame.receivedTimeUnix = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

                                latestFrames.AddOrUpdate(
                                    frame.CameraId,
                                    frame,
                                    (_, __) =>
                                    {
                                        DroppedFrames++;
                                        UpdateCameraDebug(frame, true);
                                        return frame;
                                    });
                                UpdateCameraDebug(frame, false);
                                ReceivedFrames++;
                            }
                        }
                    }
                }
                catch (Exception e)
                {
                    if (verbose) Debug.LogWarning($"[RgbdTcpReceiver] {e.Message}");
                }
                finally
                {
                    lock (connectionLock) activeClient = null;
                    IsConnected = false;
                }

                if (running) Thread.Sleep(Mathf.Max(100, reconnectDelayMs));
            }
        }

        private RgbdFrame ReadFrame(Stream stream)
        {
            byte[] fixedHeader = ReadExact(stream, 13);
            if (fixedHeader == null) return null;
            if (fixedHeader[0] != (byte)'R' || fixedHeader[1] != (byte)'G' || fixedHeader[2] != (byte)'B' || fixedHeader[3] != (byte)'D')
                throw new InvalidDataException("Bad RGBD magic.");
            if (fixedHeader[4] != 1)
                throw new InvalidDataException($"Unsupported RGBD version {fixedHeader[4]}.");

            uint headerLen = ReadUInt32LE(fixedHeader, 5);
            uint rgbLen = ReadUInt32LE(fixedHeader, 9);
            byte[] depthLenBytes = ReadExact(stream, 4);
            if (depthLenBytes == null) return null;
            uint depthLen = ReadUInt32LE(depthLenBytes, 0);

            if (headerLen == 0 || headerLen > maxHeaderBytes || rgbLen > maxRgbBytes || depthLen > maxDepthBytes)
                throw new InvalidDataException($"Invalid RGBD packet sizes h={headerLen} rgb={rgbLen} depth={depthLen}.");

            byte[] headerBytes = ReadExact(stream, (int)headerLen);
            byte[] rgb = ReadExact(stream, (int)rgbLen);
            byte[] depth = ReadExact(stream, (int)depthLen);
            if (headerBytes == null || rgb == null || depth == null) return null;

            string json = Encoding.UTF8.GetString(headerBytes);
            return new RgbdFrame
            {
                header = RgbdHeaderParser.Parse(json),
                rgbJpeg = rgb,
                depthPng = depth
            };
        }

        private static byte[] ReadExact(Stream stream, int count)
        {
            var data = new byte[count];
            int offset = 0;
            while (offset < count)
            {
                int read = stream.Read(data, offset, count - offset);
                if (read <= 0) return null;
                offset += read;
            }

            return data;
        }

        private static uint ReadUInt32LE(byte[] data, int offset)
        {
            return (uint)(data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16) | (data[offset + 3] << 24));
        }

        private void UpdateCameraDebug(RgbdFrame frame, bool overwritten)
        {
            var debug = cameraDebugById.GetOrAdd(frame.CameraId, id => new RgbdReceiverCameraDebug { cameraId = id });
            if (!overwritten)
            {
                debug.receivedFrames++;
                debug.width = frame.Width;
                debug.height = frame.Height;
                debug.lastFrameId = frame.header?.frame_id;
                debug.lastLatencyMs = frame.header != null && frame.header.send_time_unix > 0.0
                    ? (float)((frame.receivedTimeUnix - frame.header.send_time_unix) * 1000.0)
                    : 0f;
            }
            else
            {
                debug.overwrittenFrames++;
            }
        }

        private void Update()
        {
            cameraDebug.Clear();
            foreach (var kvp in cameraDebugById)
            {
                cameraDebug.Add(kvp.Value);
            }
        }

        private void OnDestroy()
        {
            running = false;
            lock (connectionLock) activeClient?.Close();
            try { worker?.Join(1500); } catch { }
        }
    }
}

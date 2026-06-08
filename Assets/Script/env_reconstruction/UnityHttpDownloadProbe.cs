using System.Collections;
using System.IO;
using UnityEngine;
using UnityEngine.Networking;

public class UnityHttpDownloadProbe : MonoBehaviour
{
    [Header("Test URL")]
    public string url = "http://192.168.10.10:8012/map_latest.ply";

    [Header("Download")]
    public bool useDownloadHandlerFile = true;
    public int timeoutSec = 30;
    public float progressLogIntervalSec = 1.0f;
    public bool startOnPlay = true;

    private bool isRunning = false;

    void Start()
    {
        Debug.Log("[HttpProbe] Start");
        if (startOnPlay)
        {
            StartCoroutine(RunDownload());
        }
    }

    [ContextMenu("Run Download Probe")]
    public void RunFromInspector()
    {
        if (!isRunning)
        {
            StartCoroutine(RunDownload());
        }
    }

    private IEnumerator RunDownload()
    {
        isRunning = true;
        string path = Path.Combine(Application.temporaryCachePath, "unity_http_probe_download.ply");
        Debug.Log($"[HttpProbe] Downloading {url}");
        Debug.Log($"[HttpProbe] temporaryCachePath={Application.temporaryCachePath}");

        float start = Time.realtimeSinceStartup;
        using (UnityWebRequest req = UnityWebRequest.Get(url))
        {
            req.timeout = timeoutSec;
            if (useDownloadHandlerFile)
            {
                req.downloadHandler = new DownloadHandlerFile(path)
                {
                    removeFileOnAbort = true
                };
                Debug.Log($"[HttpProbe] Using DownloadHandlerFile path={path}");
            }
            else
            {
                Debug.Log("[HttpProbe] Using DownloadHandlerBuffer");
            }

            UnityWebRequestAsyncOperation op = req.SendWebRequest();
            float lastLog = Time.realtimeSinceStartup;
            while (!op.isDone)
            {
                float now = Time.realtimeSinceStartup;
                if (now - lastLog >= progressLogIntervalSec)
                {
                    float mb = req.downloadedBytes / (1024f * 1024f);
                    Debug.Log($"[HttpProbe] progress={req.downloadProgress:P1} downloaded={mb:F1}MB elapsed={now - start:F2}s");
                    lastLog = now;
                }
                yield return null;
            }

            float elapsed = Time.realtimeSinceStartup - start;
            if (req.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError($"[HttpProbe] FAILED result={req.result} error={req.error} code={req.responseCode} bytes={req.downloadedBytes} elapsed={elapsed:F2}s");
                isRunning = false;
                yield break;
            }

            long bytes = useDownloadHandlerFile && File.Exists(path)
                ? new FileInfo(path).Length
                : (req.downloadHandler.data != null ? req.downloadHandler.data.LongLength : 0L);
            float mbDone = bytes / (1024f * 1024f);
            Debug.Log($"[HttpProbe] DONE code={req.responseCode} size={mbDone:F1}MB elapsed={elapsed:F3}s speed={mbDone / Mathf.Max(elapsed, 0.001f):F1}MB/s");
        }

        isRunning = false;
    }
}

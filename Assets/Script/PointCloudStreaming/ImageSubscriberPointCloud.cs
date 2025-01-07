/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class ImageSubscriberPointCloud : UnitySubscriber<MessageTypes.Sensor.CompressedImage>
    {
        public MeshRenderer meshRenderer;

        private Texture2D texture2D;
        private byte[] imageData;
        public byte[] ImageData
        {
            get { return imageData; }
        }
        private bool isMessageReceived;

        private MessageTypes.BuiltinInterfaces.Time stamp;
        private MessageTypes.Std.Header header;
        public MessageTypes.Std.Header HEADER
        {
            get {return header;}
        }
        public MessageTypes.BuiltinInterfaces.Time Stamp
        {
            get { return stamp; }
        }

        protected override void Start()
        {
			base.Start();
            texture2D = new Texture2D(1, 1);
            meshRenderer.material = new Material(Shader.Find("Standard"));
            stamp = new MessageTypes.BuiltinInterfaces.Time();
            header = new MessageTypes.Std.Header();
        }
        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(MessageTypes.Sensor.CompressedImage compressedImage)
        {
            header = compressedImage.header;
            stamp = compressedImage.header.stamp;
            imageData = compressedImage.data;
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            texture2D.LoadImage(imageData);
            texture2D.Apply();
            meshRenderer.material.SetTexture("_MainTex", texture2D);
            isMessageReceived = false;
        }

    }
}


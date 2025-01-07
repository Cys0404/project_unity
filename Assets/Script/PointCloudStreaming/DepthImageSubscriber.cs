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
using System;
using UnityEngine;
namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class DepthImageSubscriber : UnitySubscriber<MessageTypes.Sensor.CompressedImage>
    {
        private byte[] imageData;
        public byte[] ImageData
        {
            get { return imageData; }
        }
        private bool isMessageReceived;

        private MessageTypes.BuiltinInterfaces.Time stamp;
        public MessageTypes.BuiltinInterfaces.Time Stamp
        {
            get { return stamp; }
        }
        
        [Tooltip("Flag of to remove the first 12byte header, true to remove")]
        public bool remove_header = false;
        [Tooltip("Flag of to show data size for duebugging, true to show.")]
        public bool debug_size = false;



        // private float timeWait;

        // uint taipei_secs;
        // double nsecs;
        // double delay_secs;
        // uint32 sub_seq;

        protected override void Start()
        {
			base.Start();
            stamp = new MessageTypes.BuiltinInterfaces.Time();
        }
        private void Update()
        {
            // timeWait += Time.deltaTime;
            if (isMessageReceived)
            {
                ProcessMessage();
                // if (timeWait > 1.5f && GetComponent<CameraSync>() != null)
                // {
                //     timeWait = 0f;
                // }
            }

        }

        protected override void ReceiveMessage(MessageTypes.Sensor.CompressedImage compressedImage)
        {
            // taipei_secs = compressedImage.header.stamp.secs + 28800;
            // nsecs = (double)compressedImage.header.stamp.nsecs / 1000000000.0D;
            // delay_secs = (double)DateTime.Now.Subtract(new DateTime(1970, 1, 1)).TotalSeconds - (double)taipei_secs;
            // delay_secs -= nsecs;
            // Debug.Log("depth delay secs " + delay_secs);

            // sub_seq = compressedImage.header.seq;
            stamp = compressedImage.header.stamp;
            imageData = compressedImage.data;
            if (debug_size)
            {
                Debug.Log("Depth_data size: " + imageData.Length);
            }
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            if (imageData.Length > 12 && remove_header)
            {
                byte[] trimmedData = new byte[imageData.Length - 12];
                Array.Copy(imageData, 12, trimmedData, 0, imageData.Length - 12);
                imageData = trimmedData;
            }
            isMessageReceived = false;
        }
    }
}
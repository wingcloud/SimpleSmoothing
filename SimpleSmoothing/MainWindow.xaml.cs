using System;
using System.Collections.Generic;
using System.Windows;
using Microsoft.Kinect;
using System.ComponentModel;
using System.Runtime.InteropServices;

namespace SimpleSmoothing
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        const int INPUT_MOUSE = 0;
        const int INPUT_KEYBOARD = 1;
        const int INPUT_HARDWARE = 2;
        const uint MOUSEEVENTF_MOVE = 0x0001;
        const uint MOUSEEVENTF_ABSOLUTE = 0x8000;

        struct INPUT
        {
            public int type;
            public InputUnion u;
        }

        [StructLayout(LayoutKind.Explicit)]
        struct InputUnion
        {
            [FieldOffset(0)]
            public MOUSEINPUT mi;
        }

        [StructLayout(LayoutKind.Sequential)]
        struct MOUSEINPUT
        {
            public int dx;
            public int dy;
            public uint mouseData;
            public uint dwFlags;
            public uint time;
            public IntPtr dwExtraInfo;
        }

        [DllImport("user32.dll")]
        static extern IntPtr GetMessageExtraInfo();

        [DllImport("user32.dll", SetLastError = true)]
        static extern uint SendInput(uint nInputs, INPUT[] pInputs, int cbSize);

        private KinectSensor kinectSensor = null;
        private BodyFrameReader bodyFrameReader = null;
        private Body[] bodies = null;
        private float centerX = -1000;
        private float centerY = -1000;

        int filterFlag = 0;
        private Queue<CameraSpacePoint> pointBuffer = new Queue<CameraSpacePoint>();
        private Queue<CameraSpacePoint> pointBuffer2 = new Queue<CameraSpacePoint>();

        private CameraSpacePoint filtered;
        public MainWindow()
        {
            this.kinectSensor = KinectSensor.GetDefault();
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            this.kinectSensor.Open();
            InitializeComponent();
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += bodyFrameReader_FrameArrived;
            }
        }
        void bodyFrameReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if(bodyFrame!=null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }
            if(dataReceived)
            {
                foreach(Body body in this.bodies)
                {
                    if(body.IsTracked)
                    {
                        switch (filterFlag)
                        {
                            case 0:
                                filtered = body.Joints[JointType.HandRight].Position;
                                break;
                            case 1:
                                filtered = SimpleAverageFilter(body.Joints[JointType.HandRight].Position, 5);
                                break;
                            case 2:
                                filtered = DoubleMovingAverage(body.Joints[JointType.HandRight].Position, 5);
                                break;
                            case 3:
                                filtered = ModifiedDoubleMovingAverage(body.Joints[JointType.HandRight].Position, 5);
                                break;
                            case 4:
                                filtered = ExponentialSmoothing(body.Joints[JointType.HandRight].Position, 0.5f);
                                break;
                            case 5:
                                filtered = DoubleExponentialSmoothing(body.Joints[JointType.HandRight].Position, 0.4f, 0.5f);
                                break;
                        }
                        float x = filtered.X;
                        float y = filtered.Y;
                        xValue.Text = "X:" + body.Joints[JointType.HandRight].Position.X.ToString();
                        yValue.Text = "Y:" + body.Joints[JointType.HandRight].Position.Y.ToString();
                        if(centerX==-1000)
                        {
                            centerX = x;
                        }
                        if(centerY==-1000)
                        {
                            centerY = y;
                        }
                        INPUT[] inputs = new INPUT[]
                        {
                            new INPUT
                            {
                                type = INPUT_MOUSE,
                                u = new InputUnion
                                {
                                    mi = new MOUSEINPUT
                                    {
                                        dwFlags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE,
                                        dx = (int)(65535/2+(x-centerX)*65535),
                                        dy = (int)(65535/2-(y-centerY)*65535),
                                    }
                                }
                            }
                        };
                        SendInput((uint)inputs.Length, inputs, Marshal.SizeOf(typeof(INPUT)));
                    }
                }
            }
        }

        private CameraSpacePoint SimpleAverageFilter(CameraSpacePoint newPoint, int parameter)
        {
            pointBuffer.Enqueue(newPoint);
            if(pointBuffer.Count<=parameter)
            {
                return newPoint;
            }
            pointBuffer.Dequeue();
            CameraSpacePoint[] list = pointBuffer.ToArray();
            CameraSpacePoint point = new CameraSpacePoint();
            float x = 0;
            float y = 0;
            float z = 0;
            int n = pointBuffer.Count;
            for (int i = 0; i < pointBuffer.Count; i++)
            {
                CameraSpacePoint p = list[i];
                x += p.X;
                y += p.Y;
                z += p.Z;
            }
            point.X = x / n;
            point.Y = y / n;
            point.Z = z / n;

            return point;
        }
        private CameraSpacePoint DoubleMovingAverage(CameraSpacePoint newPoint, int parameter)
        {
            CameraSpacePoint newSimpleAverage = SimpleAverageFilter(newPoint, parameter);
            pointBuffer2.Enqueue(newSimpleAverage);
            if(pointBuffer2.Count<=parameter)
            {
                return newSimpleAverage;
            }
            pointBuffer2.Dequeue();
            CameraSpacePoint[] list = pointBuffer2.ToArray();
            CameraSpacePoint point = new CameraSpacePoint();
            float x = 0;
            float y = 0;
            float z = 0;
            int n = pointBuffer2.Count;
            for (int i = 0; i<pointBuffer2.Count;i++ )
            {
                CameraSpacePoint p = list[i];
                x += p.X;
                y += p.Y;
                z += p.Z;
            }
            point.X = x / n;
            point.Y = y / n;
            point.Z = z / n;
            return point;
        }
        private CameraSpacePoint ModifiedDoubleMovingAverage(CameraSpacePoint newPoint, int parameter)
        {
            CameraSpacePoint newSimpleAverage = SimpleAverageFilter(newPoint, parameter);
            pointBuffer2.Enqueue(newSimpleAverage);
            if (pointBuffer2.Count <= parameter)
            {
                return newSimpleAverage;
            }
            pointBuffer2.Dequeue();
            CameraSpacePoint[] list = pointBuffer2.ToArray();
            CameraSpacePoint point = new CameraSpacePoint();
            float x = 0;
            float y = 0;
            float z = 0;
            int n = pointBuffer2.Count;
            for (int i = 0; i < pointBuffer2.Count; i++)
            {
                CameraSpacePoint p = list[i];
                x += p.X;
                y += p.Y;
                z += p.Z;
            }
            point.X = 2 * newSimpleAverage.X - x / n;
            point.Y = 2 * newSimpleAverage.Y - y / n;
            point.Z = 2 * newSimpleAverage.Z - z / n;
            return point;
        }
        private CameraSpacePoint ExponentialSmoothing(CameraSpacePoint newPoint, float alpha)
        {
            if(pointBuffer.Count==0)
            {
                pointBuffer.Enqueue(newPoint);
                return newPoint;
            }
            CameraSpacePoint p = pointBuffer.Dequeue();
            CameraSpacePoint point = new CameraSpacePoint();
            point.X = alpha * newPoint.X + (1 - alpha) * p.X;
            point.Y = alpha * newPoint.Y + (1 - alpha) * p.Y;
            point.Z = alpha * newPoint.Z + (1 - alpha) * p.Z;
            pointBuffer.Enqueue(point);
            return point;
        }
        private CameraSpacePoint DoubleExponentialSmoothing(CameraSpacePoint newPoint, float alpha,float gamma)
        {
            if(pointBuffer.Count==0)
            {
                pointBuffer.Enqueue(newPoint);
                pointBuffer2.Enqueue(newPoint);
                return newPoint;
            }
            CameraSpacePoint bi_1 = pointBuffer.Dequeue();
            CameraSpacePoint hXi_1 = pointBuffer2.Dequeue();
            CameraSpacePoint hXi = new CameraSpacePoint();
            hXi.X = alpha * newPoint.X + (1 - alpha) * (hXi_1.X + bi_1.X);
            hXi.Y = alpha * newPoint.Y + (1 - alpha) * (hXi_1.Y + bi_1.Y);
            hXi.Z = alpha * newPoint.Z + (1 - alpha) * (hXi_1.Z + bi_1.Z);
            pointBuffer2.Enqueue(hXi);

            CameraSpacePoint bi = new CameraSpacePoint();
            bi.X = gamma * (hXi.X - hXi_1.X) + (1 - gamma) * bi_1.X;
            bi.Y = gamma * (hXi.Y - hXi_1.Y) + (1 - gamma) * bi_1.Y;
            bi.Z = gamma * (hXi.Z - hXi_1.Z) + (1 - gamma) * bi_1.Z;
            pointBuffer.Enqueue(bi);

            return hXi;
            
        }
        private void ClearBuffers()
        {
            pointBuffer.Clear();
            pointBuffer2.Clear();
        }

        private void RawData_Checked(object sender, RoutedEventArgs e)
        {
            filterFlag = 0;
            ClearBuffers();
        }
        private void SimpleAverage_Checked(object sender, RoutedEventArgs e)
        {
            filterFlag = 1;
            ClearBuffers();
        }

        private void DoubleMovingAverage_Checked(object sender, RoutedEventArgs e)
        {
            filterFlag = 2;
            ClearBuffers();
        }
        private void ModifiedDoubleMovingAverage_Checked(object sender, RoutedEventArgs e)
        {
            filterFlag = 3;
            ClearBuffers();
        }

        private void ExponentialSmoothing_Checked(object sender, RoutedEventArgs e)
        {
            filterFlag = 4;
            ClearBuffers();
        }

        private void DoubleExponentialSmoothing_Checked(object sender, RoutedEventArgs e)
        {
            filterFlag = 5;
            ClearBuffers();
        }
    }
}

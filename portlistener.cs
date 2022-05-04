using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System;
using UnityEngine.UI;
using TMPro;
using System.Linq;

[RequireComponent(typeof(LineRenderer))]

public class portlistener : MonoBehaviour
{
    //Game Objects
        public GameObject[] squareArray;
        public GameObject[] menuArray;
        public GameObject menuGameMode;
        public GameObject[] Select1Block;
        public GameObject[] PistonOneEnd;
        public GameObject[] BoomEnd;
        public GameObject[] Overshoot;
        public GameObject[] SubMenuArray;
        public GameObject[] LockMenuArray;
        public GameObject[] AnimateMenuArray;
        public GameObject[] AnimateObjects;
        public GameObject boomoneanimation;
        public GameObject ExcavatorBase;
        public GameObject ExcavatorBaseCircle;
        public GameObject ObjectToRetrieve;
        public GameObject SelectorHighlighter;
        public GameObject SliderPositionHighlighter;
        public LineRenderer[] BoomLine;
        public LineRenderer[] PistonMovingLine;
        public LineRenderer[] PistonFixedLine;
        public LineRenderer[] BoomCurve;
        public LineRenderer[] OverShootCurve;
        public LineRenderer[] Graphsplot;
        public LineRenderer PistonOneCurve;
        public LineRenderer maxreachcurve;
        public TextMeshProUGUI[] EditTableText;
        public TextMeshProUGUI[] AnimateTable;
        public TextMeshProUGUI GameModeErrorMessage;
        public TextMeshProUGUI GameModeObjectRetrieveMessage;
        public TextMeshProUGUI[] Velocity;
        public TextMeshProUGUI[] MenuText;
        public TextMeshProUGUI[] SliderPositionText;
        public TextMeshProUGUI[] SubMenuText;
        public TextMeshProUGUI[] AnimateMenuText;
        public TextMeshProUGUI[] DataText;
    //Thread and Positions
        Thread mThread;
        public string connectionIP = "127.0.0.1";
        public int connectionPort = 25001;
        IPAddress localAdd;
        TcpListener listener;
        TcpClient client;
        public Vector3 receivedPos5 = new Vector3(-100,0,0);
        public Vector3 receivedPos6 = new Vector3(-100,0,0);
        public Vector3 receivedPos7 = new Vector3(-100,0,0);
        public Vector3 receivedPos8 = new Vector3(-100,0,0);
        public Vector3 receivedPos9 = new Vector3(-100,0,0);
        public Vector3 receivedPos10 = new Vector3(-100,0,0);
        public Vector3 receivedPos11 = new Vector3(-100,0,0);
        public Vector3 receivedPos12 = new Vector3(-100,0,0);
        public Vector3 receivedPos13 = new Vector3(-100,0,0);
        public Vector3 receivedPos25 = new Vector3(-100,0,0);
        public Vector3 receivedPos26 = new Vector3(-100,0,0);
        public Vector3 receivedPos27 = new Vector3(-100,0,0);
        public Vector3 receivedPos28 = new Vector3(-100,0,0);
        Vector3 size = Vector3.zero;
        Vector3 zeros = Vector3.zero;
        Vector3 outofframe = new Vector3(-100,0,0);
        public Vector3 MenuData = new Vector3(10,0,0);
        public bool AnimationOneStatus = false;
        public bool AnimationTwoStatus = false;
        public bool PistonFractionStatus1 = true;
        public bool JointFractionStatus1 = true;
        public bool LinkOverShootStatus1 = true;
        public bool PistonFractionStatus2 = true;
        public bool JointFractionStatus2 = true;
        public bool LinkOverShootStatus2 = true;
        public bool PistonFractionStatus3 = true;
        public bool JointFractionStatus3 = true;
        public bool LinkOverShootStatus3 = true;
        public bool PositionStatus1 = true;
        public bool PositionStatus2 = true;
        public bool running; //nothing
        public float PistonFraction1;  //set default values in portlistener in unity
        public float BoomOverShootFraction1;
        public float PistonFraction2;
        public float BoomOverShootFraction2;
        public float JointFraction2;
        public float PistonFraction3;
        public float JointFraction3;
        public float BoomOverShootFraction3;
        public float AnimationPosition1;
        public float AnimationPosition2;
        public int BoomArray1AnimatePosition;
        public int BoomArray2AnimatePosition;

    public void Start() //private
        {

        ThreadStart ts = new ThreadStart(GetInfo);
        mThread = new Thread(ts);
        mThread.Start();
        Display.displays[1].Activate();
        Display.displays[2].Activate();

        }
    public void GetInfo() //nothing
     {
        localAdd = IPAddress.Parse(connectionIP);
        listener = new TcpListener(IPAddress.Any, connectionPort);
        listener.Start();
        client = listener.AcceptTcpClient();

        running = true;
        while (running)
        {
            SendAndReceiveData();
        }
        listener.Stop();
     }

    public void SendAndReceiveData() //nothing
        {
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];

        //---receiving Data from the Host----
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize); //Getting data in Bytes from Python
        string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead); //Converting byte data to string

        if (dataReceived != null)
        {
            //---Using received data---
            MenuData = StringMenu(dataReceived);
            receivedPos5 = StringToVector3(dataReceived,"5");
            receivedPos6 = StringToVector3(dataReceived,"6");
            receivedPos7 = StringToVector3(dataReceived,"7");
            receivedPos8 = StringToVector3(dataReceived,"8");
            receivedPos9 = StringToVector3(dataReceived,"9");
            receivedPos10 = StringToVector3(dataReceived,"10");
            receivedPos11 = StringToVector3(dataReceived,"11");
            receivedPos12 = StringToVector3(dataReceived,"12");
            receivedPos13 = StringToVector3(dataReceived,"13");
            receivedPos25 = StringToVector3(dataReceived,"25");
            receivedPos26 = StringToVector3(dataReceived,"26");
            receivedPos27 = StringToVector3(dataReceived,"27");
            receivedPos28 = StringToVector3(dataReceived,"28");


            //---Sending Data to Host----
            byte[] myWriteBuffer = Encoding.ASCII.GetBytes("Hey I got your message Python! Do You see this massage?"); //Converting string to byte data
            nwStream.Write(myWriteBuffer, 0, myWriteBuffer.Length); //Sending the data in Bytes to Python
        }
        }
    public static Vector3 StringToVector3(string sVector, string whichSquare)
        {
        // Remove the parentheses
        if (sVector.StartsWith("(") && sVector.EndsWith(")"))
        {
            sVector = sVector.Substring(1, sVector.Length - 2);
        }

        // split the items
        string[] sArray = sVector.Split(',');
        //get index of whichSquare
        int index = Array.IndexOf(sArray, whichSquare);
        // store as a Vector3
        Vector3 result = new Vector3(
            float.Parse(sArray[index+1]),
            float.Parse(sArray[index+2]),
            float.Parse(sArray[index+3]));

        return result;
        }
    public static Vector3 StringMenu(string sVector)
        {
        // Remove the parentheses
        if (sVector.StartsWith("(") && sVector.EndsWith(")"))
        {
            sVector = sVector.Substring(1, sVector.Length - 2);
        }

        // split the items
        string[] sArray = sVector.Split(',');
        //get index of 24
        int index = Array.IndexOf(sArray, "17");
        Vector3 result = new Vector3(
            float.Parse(sArray[index+1]),0,0);
        return result;
        }

    //Yuna Game Mode
    public string InMenuRegion(int xmin, int xmax, int ymin, int ymax, Vector3 ArucoPos)
        {
        if(xmin<ArucoPos[0] && ArucoPos[0]<xmax && ymin<ArucoPos[1] && ArucoPos[1]<ymax){
            return "in range";
        }
        else
        {
            return "out of range";
        }
        }

    public float TheTimeStep()
        {
        float dt = 1f;
        return dt;
        }
    public float DistanceBetweenPoints(Vector3 v1, Vector3 v2)
        {
            float xDiff = (float)Math.Pow((v2[0] - v1[0]),2.0);
            float yDiff = (float)Math.Pow((v2[1] - v1[1]),2.0);

            return (float)Math.Sqrt(xDiff + yDiff);
        }

    //Ben Slider
    public float sliderAngle(Vector3 Handle_Position, Vector3 Slider_Position)
        {
        float dx = Slider_Position[1] - Handle_Position[1];
        float dy = Slider_Position[0] - Handle_Position[0];
        float thetaSlider = (float)Math.Atan2(dy, dx);

        return thetaSlider;
        }
    public float sliderValue(Vector3 Handle_Position, Vector3 Slider_Position)
    {
        float sliderdistance = DistanceBetweenPoints(Slider_Position, Handle_Position);
        float start_magnitude = 40f;
        float end_magnitude = 120f;
        float slider_current_value = (float)Math.Round(((sliderdistance - start_magnitude) / end_magnitude),1);
        if(slider_current_value>=1)
        {
            return 1;
        }
        else if (slider_current_value<=0)
        {
            return 0;
        }
        else
        {
            return slider_current_value;
        }

    }
    // George's Calculation here
    public Vector3 BoomStartFinder(Vector3 BoomFixed, Vector3 BoomEnd, float BoomOverShootFraction)
        {
            Vector3 VectorBoomEndToBoomFixed = BoomFixed - BoomEnd;
            Vector3 BoomStart = BoomFixed + (BoomOverShootFraction*VectorBoomEndToBoomFixed);
            return BoomStart;
        }
    public Vector3 PistonEnd(Vector3 BoomStart, Vector3 BoomEnd, float PistonFraction)
        {
            float BoomStartX = BoomStart[0];
            float BoomStartY = BoomStart[1];
            float BoomEndX = BoomEnd[0];
            float BoomEndY = BoomEnd[1];
            float PistonEndX = (BoomEndX-BoomStartX)*PistonFraction +BoomStartX;
            float PistonEndY = (BoomEndY-BoomStartY)*PistonFraction +BoomStartY;

            Vector3 PistonEnd = new Vector3(PistonEndX, PistonEndY, 0f);
            return PistonEnd;

        }
    public Vector3[] BoomRotationCalculation(Vector3 BoomFixed, Vector3 BoomEnd, Vector3 PistonStart, float BoomOverShootFraction, float PistonFraction, float TimeStep)
        {
            float BoomFixedX = BoomFixed[0];
            float BoomFixedY = BoomFixed[1];
            float BoomEndX = BoomEnd[0];
            float BoomEndY = BoomEnd[1];
            float PistonStartX = PistonStart[0];
            float PistonStartY = PistonStart[1];

            Vector3 VectorBoomEndToBoomFixed = BoomFixed - BoomEnd;
            Vector3 BoomStart = BoomFixed + (BoomOverShootFraction*VectorBoomEndToBoomFixed);
            float BoomStartX = BoomStart[0];
            float BoomStartY = BoomStart[1];
            float PistonEndX = (BoomEndX-BoomStartX)*PistonFraction +BoomStartX;
            float PistonEndY = (BoomEndY-BoomStartY)*PistonFraction +BoomStartY;

            Vector3 PistonEnd = new Vector3(PistonEndX, PistonEndY, 0f);

        //Data from inputs
            float BoomLength = DistanceBetweenPoints(BoomStart, BoomEnd);
            float BoomStartToPistonLink = BoomLength*PistonFraction;
            float BoomFixedToPistonStart = DistanceBetweenPoints(BoomFixed, PistonStart);


            Vector3 BoomFinish = new Vector3(BoomEndX, BoomEndY, 0);
            if (BoomStartToPistonLink < BoomLength*(BoomOverShootFraction/(BoomOverShootFraction+1)))
            {
            BoomFinish[0] = BoomStartX;
            BoomFinish[1] = BoomStartY;
            }
            float BoomFinishX = BoomFinish[0];
            float BoomFinishY = BoomFinish[1];

        //Requied for solve
            float BoomFixedToFinished = DistanceBetweenPoints(BoomFixed, BoomFinish);
            float BoomFixedToPistonLink = DistanceBetweenPoints(BoomFixed, PistonEnd);
            float PistonFractionAlongBoomFixedToFinish = BoomFixedToPistonLink/BoomFixedToFinished;

        //Piston Data
            float PistonRatio = 1.8f;
            float PistonRate = 0.5f;
            bool Max = false; //Assume piston placed in minimum position unless defined as maximum
            float PistonMin = DistanceBetweenPoints(PistonStart, PistonEnd);
            float PistonMax = 0f;

            if (Max == false)
            {
            if ((PistonMin*PistonRatio)<(BoomFixedToPistonStart+BoomFixedToPistonLink))
            {
            PistonMax = PistonMin*PistonRatio;
            }
            else
            {
            PistonMax = BoomFixedToPistonStart+BoomFixedToPistonLink;
            }
            }
            else if (Max == true)
            {
            PistonMax = DistanceBetweenPoints(PistonStart, PistonEnd);
            PistonMin = PistonMax/PistonRatio;
            }
        //Time Data
            float PistonStep = PistonRate*TimeStep;
            int ArrayLength = (int)Math.Round((PistonMax-PistonMin)/PistonStep);

        //Piston Length Array
            float[] PistonLength = new float [ArrayLength];
            for (int i = 0; i<ArrayLength; i++)
            {
            PistonLength[i] = PistonMin + i*PistonStep;
            }
            // Array.ForEach(PistonLength, Console.WriteLine);

        //Time Array
            float[] Time = new float [ArrayLength];
            for (int i = 0; i<ArrayLength; i++)
            {
                Time[i] = i*TimeStep;
            }
        //Solve
        //z array
            float[] z = new float[ArrayLength];
            for (int i = 0; i<ArrayLength; i++)
            {
                z[i] = (((float)Math.Pow(BoomFixedToPistonLink,2.0) + (float)Math.Pow(BoomFixedToPistonStart,2.0) - (float)Math.Pow(PistonLength[i],2.0))/(2*BoomFixedToPistonLink*BoomFixedToPistonStart));
            }
        //Theta - angle between Boom and vector BoomFixed to PistonStart - Alpha is the polar angle of Piston Start from Boom Fixed
            float Alpha = (float)Math.Atan2( (PistonStartY-BoomFixedY), (PistonStartX-BoomFixedX));
            float AlphaThetaOne = (float)Math.Atan2( (BoomFinishY-BoomFixedY), (BoomFinishX -BoomFixedX));

            int o1 = (int)Math.Round((AlphaThetaOne - Alpha - (float)Math.Acos(z[0]))*180/Math.PI);
            int o2 = (int)Math.Round((AlphaThetaOne - Alpha + (float)Math.Acos(z[0]))*180/Math.PI);
            int o3 = (int)Math.Round((AlphaThetaOne + Alpha - (float)Math.Acos(z[0]))*180/Math.PI);
            int o4 = (int)Math.Round((AlphaThetaOne + Alpha + (float)Math.Acos(z[0]))*180/Math.PI);
            // Console.WriteLine(o1);
            // Console.WriteLine(o2);
            // Console.WriteLine(o3);
            // Console.WriteLine(o4);

        float[] Theta = new float[ArrayLength];
            if (o1 == 0 || o1 == 360 || o1 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] = (float)Math.Acos(z[i]);
            }
         }
            if (o2 == 0 || o2 == 360 || o2 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  -(float)Math.Acos(z[i]);
            }
         }
            if (o3 == 0 || o3 == 360 || o3 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  (float)Math.Acos(z[i]) ;
            }
            Alpha = -Alpha;
         }
            if (o4 == 0 || o4 == 360 || o4 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  - (float)Math.Acos(z[i]);
            }
            Alpha = -Alpha;
         }

        //Using correct angles
        //Calc Piston Vectors
            Vector3[] PistonArray = new Vector3[ArrayLength];
		    for ( int i = 0; i < ArrayLength; i++)
		    {
                float x = BoomFixedToPistonLink*(float)Math.Cos(Theta[i]+Alpha) + BoomFixedX;
                float y = BoomFixedToPistonLink*(float)Math.Sin(Theta[i]+Alpha) + BoomFixedY;
		    	PistonArray[i] = new Vector3(x, y, 0f) ;
		    }

            Vector3[] BoomArray = new Vector3[ArrayLength];
		    for ( int i = 0; i < ArrayLength; i++)
		    {
                float x = (PistonArray[i][0] - BoomFixedX)/PistonFractionAlongBoomFixedToFinish + BoomFixedX;
                float y = (PistonArray[i][1] - BoomFixedY)/PistonFractionAlongBoomFixedToFinish + BoomFixedY;
		    	BoomArray[i] = new Vector3(x, y, 0f) ;
		    }
            if (BoomFinish == BoomStart)
            {

		    for ( int i = 0; i < ArrayLength; i++)
		    {
                float x = BoomArray[i][0] + (BoomFixedX - BoomArray[i][0])*((1+BoomOverShootFraction)/BoomOverShootFraction);
                float y = BoomArray[i][1]+ (BoomFixedY - BoomArray[i][1])*((1+BoomOverShootFraction)/BoomOverShootFraction);
		    	BoomArray[i] = new Vector3(x, y, 0f);
		    }
            }

            return BoomArray;

        }
    public Vector3[] PistonRotationCalculation(Vector3 BoomFixed, Vector3 BoomEnd, Vector3 PistonStart, float BoomOverShootFraction, float PistonFraction, float TimeStep)
        {
            float BoomFixedX = BoomFixed[0];
            float BoomFixedY = BoomFixed[1];
            float BoomEndX = BoomEnd[0];
            float BoomEndY = BoomEnd[1];
            float PistonStartX = PistonStart[0];
            float PistonStartY = PistonStart[1];

            Vector3 VectorBoomEndToBoomFixed = BoomFixed - BoomEnd;
            Vector3 BoomStart = BoomFixed + (BoomOverShootFraction*VectorBoomEndToBoomFixed);
            float BoomStartX = BoomStart[0];
            float BoomStartY = BoomStart[1];
            float PistonEndX = (BoomEndX-BoomStartX)*PistonFraction +BoomStartX;
            float PistonEndY = (BoomEndY-BoomStartY)*PistonFraction +BoomStartY;

            Vector3 PistonEnd = new Vector3(PistonEndX, PistonEndY, 0f);

        //Data from inputs
            float BoomLength = DistanceBetweenPoints(BoomStart, BoomEnd);
            float BoomStartToPistonLink = BoomLength*PistonFraction;
            float BoomFixedToPistonStart = DistanceBetweenPoints(BoomFixed, PistonStart);


            Vector3 BoomFinish = new Vector3(BoomEndX, BoomEndY, 0);
            if (BoomStartToPistonLink < BoomLength*(BoomOverShootFraction/(BoomOverShootFraction+1)))
            {
            BoomFinish[0] = BoomStartX;
            BoomFinish[1] = BoomStartY;
            }
            float BoomFinishX = BoomFinish[0];
            float BoomFinishY = BoomFinish[1];

        //Requied for solve
            float BoomFixedToFinished = DistanceBetweenPoints(BoomFixed, BoomFinish);
            float BoomFixedToPistonLink = DistanceBetweenPoints(BoomFixed, PistonEnd);
            float PistonFractionAlongBoomFixedToFinish = BoomFixedToPistonLink/BoomFixedToFinished;

        //Piston Data
            float PistonRatio = 1.8f;
            float PistonRate = 0.5f;
            bool Max = false; //Assume piston placed in minimum position unless defined as maximum
            float PistonMin = DistanceBetweenPoints(PistonStart, PistonEnd);
            float PistonMax = 0f;

            if (Max == false)
            {
            if ((PistonMin*PistonRatio)<(BoomFixedToPistonStart+BoomFixedToPistonLink))
            {
            PistonMax = PistonMin*PistonRatio;
            }
            else
            {
            PistonMax = BoomFixedToPistonStart+BoomFixedToPistonLink;
            }
            }
            else if (Max == true)
            {
            PistonMax = DistanceBetweenPoints(PistonStart, PistonEnd);
            PistonMin = PistonMax/PistonRatio;
            }
        //Time Data
            float PistonStep = PistonRate*TimeStep;
            int ArrayLength = (int)Math.Round((PistonMax-PistonMin)/PistonStep);

        //Piston Length Array
            float[] PistonLength = new float [ArrayLength];
            for (int i = 0; i<ArrayLength; i++)
            {
            PistonLength[i] = PistonMin + i*PistonStep;
            }
            // Array.ForEach(PistonLength, Console.WriteLine);

        //Time Array
            float[] Time = new float [ArrayLength];
            for (int i = 0; i<ArrayLength; i++)
            {
                Time[i] = i*TimeStep;
            }
        //Solve
        //z array
            float[] z = new float[ArrayLength];
            for (int i = 0; i<ArrayLength; i++)
            {
                z[i] = (((float)Math.Pow(BoomFixedToPistonLink,2.0) + (float)Math.Pow(BoomFixedToPistonStart,2.0) - (float)Math.Pow(PistonLength[i],2.0))/(2*BoomFixedToPistonLink*BoomFixedToPistonStart));
            }
        //Theta - angle between Boom and vector BoomFixed to PistonStart - Alpha is the polar angle of Piston Start from Boom Fixed
            float Alpha = (float)Math.Atan2( (PistonStartY-BoomFixedY), (PistonStartX-BoomFixedX));
            float AlphaThetaOne = (float)Math.Atan2( (BoomFinishY-BoomFixedY), (BoomFinishX -BoomFixedX));

            int o1 = (int)Math.Round((AlphaThetaOne - Alpha - (float)Math.Acos(z[0]))*180/Math.PI);
            int o2 = (int)Math.Round((AlphaThetaOne - Alpha + (float)Math.Acos(z[0]))*180/Math.PI);
            int o3 = (int)Math.Round((AlphaThetaOne + Alpha - (float)Math.Acos(z[0]))*180/Math.PI);
            int o4 = (int)Math.Round((AlphaThetaOne + Alpha + (float)Math.Acos(z[0]))*180/Math.PI);
            // Console.WriteLine(o1);
            // Console.WriteLine(o2);
            // Console.WriteLine(o3);
            // Console.WriteLine(o4);

         float[] Theta = new float[ArrayLength];
            if (o1 == 0 || o1 == 360 || o1 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] = (float)Math.Acos(z[i]);
            }
         }
            if (o2 == 0 || o2 == 360 || o2 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  -(float)Math.Acos(z[i]);
            }
         }
            if (o3 == 0 || o3 == 360 || o3 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  (float)Math.Acos(z[i]) ;
            }
            Alpha = -Alpha;
         }
            if (o4 == 0 || o4 == 360 || o4 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  - (float)Math.Acos(z[i]);
            }
            Alpha = -Alpha;
         }

        //Using correct angles
        //Calc Piston Vectors
            Vector3[] PistonArray = new Vector3[ArrayLength];
		    for ( int i = 0; i < ArrayLength; i++)
		    {
                float x = BoomFixedToPistonLink*(float)Math.Cos(Theta[i]+Alpha) + BoomFixedX;
                float y = BoomFixedToPistonLink*(float)Math.Sin(Theta[i]+Alpha) + BoomFixedY;
		    	PistonArray[i] = new Vector3(x, y, 0f) ;
		    }

            return PistonArray;

        }

    public Vector3[] BoomStartArray(Vector3 BoomFixed, Vector3[] BoomArray, float BoomOvershoot)
        {
        int ArrayLength = BoomArray.Length;
        Vector3[] NewBoomStartArray = new Vector3[ArrayLength];
        for(int i=0; i < ArrayLength; i++)
        {
            float x = BoomFixed[0] + ((BoomFixed[0] - BoomArray[i][0])*BoomOvershoot);
            float y = BoomFixed[1] + ((BoomFixed[1] - BoomArray[i][1])*BoomOvershoot);
            NewBoomStartArray[i] = new Vector3(x, y, 0f);
        }
        return NewBoomStartArray;
        }
    public float[] RotationFromStartCalculation(Vector3[] BoomArray, Vector3 FixedPoint)
        {
        int ArrayLength = BoomArray.Length;
        float[] AngleChange = new float[ArrayLength];
        float StartAngle = (float)Math.Atan2(BoomArray[0][1]-FixedPoint[1], BoomArray[0][0]-FixedPoint[0]);
        //Clockwise or anticlockwise
        float x1 = BoomArray[0][0]-FixedPoint[0];
        float y1 = BoomArray[0][1]-FixedPoint[1];
        float x2 = BoomArray[1][0]-FixedPoint[0];
        float y2 = BoomArray[1][1]-FixedPoint[1];
        string Direction = "";
        if(y1<0 && y2>0 && x1<0 && x2<0){Direction = "Clockwise";}
        else if(y1>0 && y2<0 && x1<0 && x2<0){Direction = "Anticlockwise";}
        else if((float)Math.Atan2(y2,x2)-(float)Math.Atan2(y1,x1) < 0){Direction = "Clockwise";}
        else if((float)Math.Atan2(y2,x2)-(float)Math.Atan2(y1,x1) > 0){Direction = "Anticlockwise";}
        for (int i = 0; i<ArrayLength; i++)
        {
        float PointAngle = (float)Math.Atan2(BoomArray[i][1]-FixedPoint[1], BoomArray[i][0]-FixedPoint[0]);

        if(StartAngle<0 && PointAngle>0 && Direction=="Clockwise")
            {
            AngleChange[i] = PointAngle - StartAngle - 2*(float)Math.PI;
            }
        if(StartAngle>0 && PointAngle<0 && Direction=="Anticlockwise")
            {
            AngleChange[i] = PointAngle - StartAngle + 2*(float)Math.PI;
            }
        else
        {
            AngleChange[i] = PointAngle - StartAngle;
        }
        }
        return AngleChange;
        }
    public float BoomRangeCalculation(float[] RotationArray)
        {
        int End = RotationArray.Length - 1;
        float StartAngle = RotationArray[0];
        float EndAngle = RotationArray[End];
        float BoomRange = (EndAngle - StartAngle)*180f/((float)Math.PI);
        if(BoomRange<0)
            {
                BoomRange = -BoomRange;
            }
        return BoomRange;
        }
    public Vector3[] AngularVelocityCalculation(Vector3[] BoomArray, float TimeStep, Vector3 FixedPoint)
        {
        int ArrayLength = BoomArray.Length;
        Vector3[] AngularVelocity = new Vector3[ArrayLength];
        AngularVelocity[0] = new Vector3(0f, 0f, 0f) ;
        for ( int i = 1; i < ArrayLength; i++)
	    {
        float AngleChange = (float)Math.Atan2(BoomArray[i][1]-FixedPoint[1], BoomArray[i][0]-FixedPoint[0]) - (float)Math.Atan2(BoomArray[(i-1)][1]-FixedPoint[1], BoomArray[(i-1)][0]-FixedPoint[0]);
        if((BoomArray[i][1]-FixedPoint[1])>0 && (BoomArray[i-1][1]-FixedPoint[1])<0 && (AngleChange > (float)Math.PI))
            {
            AngleChange = AngleChange - 2*(float)Math.PI;
            }
        if((BoomArray[i][1]-FixedPoint[1])<0 && (BoomArray[i-1][1]-FixedPoint[1])>0 && (AngleChange < -(float)Math.PI))
            {
            AngleChange = AngleChange + 2*(float)Math.PI;
            }

        float x =  i*TimeStep;
        float y =  AngleChange/TimeStep;
        AngularVelocity[i] = new Vector3(x, y, 0f);

        }
        for ( int i = 1; i <ArrayLength-1; i++)
	    {
        float y = (AngularVelocity[i][1] + AngularVelocity[i+1][1] )/2;
        AngularVelocity[i][1] = y;
        }
        AngularVelocity[ArrayLength-1][1] = 0;

        return AngularVelocity;

        }
    public Vector3[] VelocityContracting(Vector3[] ExtendingV)
        {
        int ArrayLength = ExtendingV.Length;
        Vector3[] ContractingV = new Vector3[ArrayLength];
        for(int i=0; i<ArrayLength; i++)
        {
            float y = -ExtendingV[i][1];
            float x = ExtendingV[ArrayLength-1-i][0];
            ContractingV[i] = new Vector3(x, y, 0f);
        }
        return ContractingV;
        }
    public Vector3[] VelocityCalculation(Vector3[] AngularVelocity, Vector3 BoomFixed, Vector3 BoomEnd)
        {
        int ArrayLength = AngularVelocity.Length;
        Vector3[] Velocity = new Vector3[ArrayLength];
        float BoomFixedToEnd = DistanceBetweenPoints(BoomFixed, BoomEnd);
        for ( int i = 0; i < ArrayLength; i++)
	    {
        float x =  AngularVelocity[i][0];
        float y =  AngularVelocity[i][1]*BoomFixedToEnd;
        Velocity[i] = new Vector3(x, y, 0f);
        }
        return Velocity;
        }
    public Vector3 BoomFixedFinder(Vector3 PreviousBoomFixed, Vector3 PreviousBoomEnd, float JointFraction)
        {
      Vector3 PreviousFixedToEnd = PreviousBoomEnd - PreviousBoomFixed;
      Vector3 NextFixed = PreviousBoomFixed + JointFraction*PreviousFixedToEnd;
      return NextFixed;
        }
    public Vector3[] RelativePosition(Vector3 PreviousFixedPoint, Vector3 PointToBeTranslated, float[] Rotation)
        {
        int ArrayLength = Rotation.Length;
        Vector3[] NewPosition = new Vector3[ArrayLength];
        float OriginalAngle = (float)Math.Atan2((PointToBeTranslated[1]-PreviousFixedPoint[1]), (PointToBeTranslated[0]-PreviousFixedPoint[0]));
        float Length = DistanceBetweenPoints(PreviousFixedPoint, PointToBeTranslated);
        for(int i=0; i<ArrayLength; i++)
        {
            float NewAngle = OriginalAngle + Rotation[i];
            float x = Length*(float)Math.Cos(NewAngle) + PreviousFixedPoint[0];
            float y = Length*(float)Math.Sin(NewAngle) + PreviousFixedPoint[1];
            NewPosition[i] = new Vector3(x, y, 0f);
        }
        return NewPosition;

        }
    public Vector3[,] ArrayRelativePosition(Vector3 PreviousFixedPoint, Vector3[] ArrayToBeTranslated, float[] Rotation)
        {
        int ArrayLength = ArrayToBeTranslated.Length;
        int PreviousArrayLength = Rotation.Length;
        Vector3[,] TranslatedArrays = new Vector3[PreviousArrayLength, ArrayLength];
        for(int i=0; i<ArrayLength; i++)
        {
            float OriginalAngle = (float)Math.Atan2((ArrayToBeTranslated[i][1]-PreviousFixedPoint[1]), (ArrayToBeTranslated[i][0]-PreviousFixedPoint[0]));
            float Length = DistanceBetweenPoints(PreviousFixedPoint, ArrayToBeTranslated[i]);
            for(int j=0; j<PreviousArrayLength; j++)
            {
                float NewAngle = OriginalAngle + Rotation[j];
                float x = Length*(float)Math.Cos(NewAngle) + PreviousFixedPoint[0];
                float y = Length*(float)Math.Sin(NewAngle) + PreviousFixedPoint[1];
                TranslatedArrays[j,i] = new Vector3(x, y, 0f);
            }
        }
        return TranslatedArrays;
        }
    public Vector3[] MaxRangePosition(Vector3 OriginBoomFixed, Vector3[] BoomArray, Vector3[,] NextBoomMovement)
        {
        int OriginArrayLength = NextBoomMovement.GetLength(0);
        int NextArrayLength = NextBoomMovement.GetLength(1);
        Vector3[] MaxRangePositionArray = new Vector3[OriginArrayLength];
        Vector3 MaxRange = new Vector3(0f, 0f, 0f);
        for(int i=0; i<OriginArrayLength; i++)
        {
            MaxRange = BoomArray[i];
            float MaxDistance = DistanceBetweenPoints(OriginBoomFixed, BoomArray[i]);
            for(int j=0; j<NextArrayLength; j++)
            {
                float Distance = DistanceBetweenPoints(OriginBoomFixed, NextBoomMovement[i,j]);

                if(Distance>MaxDistance)
                {
                    MaxDistance = Distance;
                    MaxRange = NextBoomMovement[i,j];
                }
            }
            float x = MaxRange[0];
            float y = MaxRange[1];
            MaxRangePositionArray[i] = new Vector3(x, y, 0f);
        }
        return MaxRangePositionArray;
        }
    public float[,] TwoMovingBoomsVelocity(Vector3[] FirstBoomAngularVelocity, Vector3 FirstFixedPoint, Vector3[] SecondBoomFixedArray, Vector3[] SecondBoomVelocity, Vector3[,] SecondBoomArrays)
        {
        int FirstArrayLength = FirstBoomAngularVelocity.Length;
        int SecondArrayLength = SecondBoomArrays.GetLength(1);
        float[,] EndVelocity = new float[FirstArrayLength,SecondArrayLength];
        for(int i=0; i<FirstArrayLength; i++)
        {
            Vector3 SecoundFixedPosition = SecondBoomFixedArray[i];
            float FirstBoomAV = FirstBoomAngularVelocity[i][1];
            for(int j=0; j<SecondArrayLength; j++)
            {
                Vector3 SecondBoomEndPosition = SecondBoomArrays[i,j];
                //Velocity due to Boom1 rotation
                float RadiusBoom2FromFixed1 = DistanceBetweenPoints(FirstFixedPoint, SecondBoomEndPosition);
                float VelocityDueBoom1 = RadiusBoom2FromFixed1*FirstBoomAV;
                //Velocity due to Boom2 rotation
                //X Y components
                float OriginalVB2 = SecondBoomVelocity[j][1];
                float AngleB2 = (float)Math.Atan2((SecondBoomEndPosition[1]-SecoundFixedPosition[1]), (SecondBoomEndPosition[0]-SecoundFixedPosition[0]));
                float AngleF1toB2 = (float)Math.Atan2((SecondBoomEndPosition[1]-FirstFixedPoint[1]), (SecondBoomEndPosition[0]-FirstFixedPoint[0]));
                float VelocityDueBoom2 = OriginalVB2*(float)Math.Cos(AngleF1toB2-AngleB2);
                float V = VelocityDueBoom2 + VelocityDueBoom1;
                EndVelocity[i,j] = V;
            }


        }
            return EndVelocity;
        }
    public float[,,] ThreeMovingBoomsVelocity(Vector3[] FirstBoomAngularVelocity, Vector3 FirstFixedPoint, Vector3[] SecondBoomFixedArray, float[,] EndTwoBoomVelocity, Vector3[,,] FinalBoomArrays)
        {
        int FirstArrayLength = FirstBoomAngularVelocity.Length;
        int SecondArrayLength = EndTwoBoomVelocity.GetLength(0);
        int ThirdArrayLength = EndTwoBoomVelocity.GetLength(1);
        float[,,] EndVelocity = new float[FirstArrayLength,SecondArrayLength, ThirdArrayLength];
        for(int i=0; i<FirstArrayLength; i++)
        {
            Vector3 SecondFixedPosition = SecondBoomFixedArray[i];
            float FirstBoomAV = FirstBoomAngularVelocity[i][1];
            for(int j=0; j<SecondArrayLength; j++)
            {
                for(int k=0; k<ThirdArrayLength; k++)
                {
                Vector3 FinalBoomEndPosition = FinalBoomArrays[i,j,k];
                //Velocity due to Boom1 rotation
                float RadiusBoom3FromFixed1 = DistanceBetweenPoints(FirstFixedPoint, FinalBoomEndPosition);
                float VelocityDueBoom1 = RadiusBoom3FromFixed1*FirstBoomAV;
                //Velocity due to Boom2 rotation
                //X Y components
                float OriginalVB3 = EndTwoBoomVelocity[j,k];
                float AngleB3 = (float)Math.Atan2((FinalBoomEndPosition[1]-SecondFixedPosition[1]), (FinalBoomEndPosition[0]-SecondFixedPosition[0]));
                float AngleF1toB3 = (float)Math.Atan2((FinalBoomEndPosition[1]-FirstFixedPoint[1]), (FinalBoomEndPosition[0]-FirstFixedPoint[0]));
                float VelocityDueBoom2and3 = OriginalVB3*(float)Math.Cos(AngleF1toB3-AngleB3);
                float V = VelocityDueBoom2and3 + VelocityDueBoom1;
                EndVelocity[i,j,k] = V;
                }
            }


        }
            return EndVelocity;
        }
    public Vector3[,,] ArrayRelativePosition3Booms(Vector3 StartFixedPoint, Vector3[,] ArrayToBeTranslated, float[] Rotation)
        {
        int ArrayLength2 = ArrayToBeTranslated.GetLength(0);
        int ArrayLength3 = ArrayToBeTranslated.GetLength(1);
        int ArrayLength1 = Rotation.Length;
        Vector3[,,] TranslatedArrays = new Vector3[ArrayLength1, ArrayLength2, ArrayLength3];
        for(int i=0; i<ArrayLength1; i++)
        {
            float AngleChange = Rotation[i];

            for(int j=0; j<ArrayLength2; j++)
            {
                for(int k=0; k<ArrayLength3; k++)
                {
                    float OriginalAngle = (float)Math.Atan2(ArrayToBeTranslated[j,k][1] - StartFixedPoint[1], ArrayToBeTranslated[j,k][0] - StartFixedPoint[0]);
                    float NewAngle = OriginalAngle+AngleChange;
                    Vector3 EndPoint = ArrayToBeTranslated[j,k];
                    float Length = DistanceBetweenPoints(StartFixedPoint, EndPoint);
                    float x = Length*(float)Math.Cos(NewAngle) + StartFixedPoint[0];
                    float y = Length*(float)Math.Sin(NewAngle) + StartFixedPoint[1];
                    TranslatedArrays[i,j,k] = new Vector3(x ,y ,0f);
                }
            }
        }
        return TranslatedArrays;
        }

public async void Update()
    {
        //Time Step Generic
            float TimeStep = TheTimeStep();
        //Top data
            //Edit Table
            EditTableText[0].transform.position = new Vector3(200,-30,0); //text
            EditTableText[10].transform.position = new Vector3(332,-17,0); //number
            EditTableText[1].transform.position = new Vector3(285,-13,0); //p1
            EditTableText[4].transform.position = new Vector3(285,-30,0); //p2
            EditTableText[7].transform.position = new Vector3(285,-47,0); //p3
            EditTableText[2].transform.position = new Vector3(330,-13,0); //l1
            EditTableText[5].transform.position = new Vector3(330,-30,0); //l2
            EditTableText[8].transform.position = new Vector3(330,-47,0); //l3
            EditTableText[3].transform.position = new Vector3(380,-13,0); //j1
            EditTableText[6].transform.position = new Vector3(380,-30,0); //j2
            EditTableText[9].transform.position = new Vector3(380,-47,0); //j3



        //Slider Positions
            Vector3 SliderPosition = receivedPos8; //Moving
            Vector3 HandlePosition = receivedPos9; //Still

        //B3 fractions
            float BoomOverShootFraction3 = 0f;
            float PistonFraction3 = 0.7f;
            float JointFraction3 = 1f;




        //Slider Assignment
            Vector3 EditSubMenuAruco = receivedPos26;
            Vector3 TypeSelectionAruco = receivedPos27;
            Vector3 LockMenuAruco = receivedPos28;
            string EditSubMenuOne = InMenuRegion(0, 100, -400, -350, EditSubMenuAruco);
            string EditSubMenuTwo = InMenuRegion(0, 100, -450, -400, EditSubMenuAruco);
            string EditSubMenuThree = InMenuRegion(0, 100, -500, -450, EditSubMenuAruco);
            string TypeSelectionOne = InMenuRegion(850, 950, -120, -70, TypeSelectionAruco);
            string TypeSelectionTwo = InMenuRegion(850, 950, -170, -120, TypeSelectionAruco);
            string TypeSelectionThree = InMenuRegion(850, 950, -220, -170, TypeSelectionAruco);
            string TypeSelectionFour = InMenuRegion(850, 950, -270, -220, TypeSelectionAruco);
            string TypeSelectionWhole = InMenuRegion(850, 950, -270, -70, TypeSelectionAruco);
            string LockMenu = InMenuRegion(850,950,-400,-350,LockMenuAruco);
            string UnlockMenu = InMenuRegion(850,950,-450,-400,LockMenuAruco);

        //Assigning Slider Values
            if(MenuData[0]==0)
            {
                    //SubMenu Text
                    BoomCurve[0].GetComponent<Renderer>().enabled=false;
                    BoomCurve[1].GetComponent<Renderer>().enabled=false;
                    menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                    menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                    menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
                    GameModeObjectRetrieveMessage.enabled=false;
                    SubMenuText[0].text = "Piston";
                    SubMenuText[1].text = "Link";
                    SubMenuText[2].text = "Joint";

                    //Default Values Text
                    EditTableText[1].text = PistonFraction1.ToString();
                    EditTableText[2].text = PistonFraction2.ToString();
                    EditTableText[3].text = PistonFraction3.ToString();
                    EditTableText[4].text = BoomOverShootFraction1.ToString();
                    EditTableText[5].text = BoomOverShootFraction2.ToString();
                    EditTableText[6].text = BoomOverShootFraction3.ToString();
                    EditTableText[8].text = JointFraction2.ToString();

                    //disable animate bottom menu
                    AnimateMenuText[0].enabled = false;
                    AnimateMenuText[1].enabled = false;
                    AnimateMenuText[2].enabled = false;
                    AnimateMenuArray[0].GetComponent<SpriteRenderer>().enabled = false;
                    AnimateMenuArray[1].GetComponent<SpriteRenderer>().enabled = false;
                    AnimateMenuArray[2].GetComponent<SpriteRenderer>().enabled = false;

                    //disable animate table
                    AnimateTable[0].enabled = false;
                    AnimateTable[1].enabled = false;
                    AnimateTable[2].enabled = false;
                    AnimateTable[3].enabled = false;
                    AnimateTable[4].enabled = false;
                    AnimateTable[5].enabled = false;
                    AnimateTable[6].enabled = false;
                    AnimateTable[7].enabled = false;

                    BoomCurve[0].GetComponent<Renderer>().enabled=false;
                    BoomCurve[1].GetComponent<Renderer>().enabled=false;
                    menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                    menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                    menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;

            if(EditSubMenuOne == "in range")//Piston Selected
                {
                    //Highlight Piston To Blue
                    SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                    SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                    SubMenuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
                    SliderPositionText[0].enabled = true;
                    SliderPositionText[1].enabled = false;
                    SliderPositionText[2].enabled = false;
                    Select1Block[0].GetComponent<Renderer>().enabled = false;
                    SliderPositionText[0].transform.position = new Vector3(475,-530,0);
                    MenuText[0].text = "Piston Selection";
                    //Piston Fraction or Boom Overshoot
                    if(TypeSelectionOne == "in range") //Piston One
                    {
                        //Highlights Piston One Selection
                        SelectorHighlighter.GetComponent<Renderer>().enabled = true;
                        SelectorHighlighter.transform.position = new Vector3(890,-95,0);
                        //if slider present
                        if(SliderPosition!=outofframe && HandlePosition!=outofframe)
                        {
                            if(LockMenu == "in range" && PistonFractionStatus1 == true)
                            {
                                PistonFraction1 = sliderValue(HandlePosition,SliderPosition);
                                LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                                LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                                EditTableText[1].text = PistonFraction1.ToString();
                                PistonFractionStatus1 = false;
                            }
                            else if(UnlockMenu == "in range")
                            {
                                PistonFractionStatus1 = true;
                                PistonFraction1 = sliderValue(HandlePosition,SliderPosition);
                                EditTableText[1].text = PistonFraction1.ToString();
                                LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                                LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            }

                        }
                    }
                    if(TypeSelectionTwo == "in range")//Piston Two
                    {
                        SelectorHighlighter.GetComponent<Renderer>().enabled = true;
                        SelectorHighlighter.transform.position = new Vector3(890,-145,0); //Highlights Piston Two Selection
                        if(SliderPosition!=outofframe && HandlePosition!=outofframe)
                        {
                            if(LockMenu == "in range" && PistonFractionStatus2 == true)
                            {
                                PistonFraction2 = sliderValue(HandlePosition,SliderPosition);
                                LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                                LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                                EditTableText[2].text = PistonFraction2.ToString();
                                PistonFractionStatus2 = false;
                            }
                            else if(UnlockMenu == "in range")
                            {
                                PistonFractionStatus2 = true;
                                PistonFraction2 = sliderValue(HandlePosition,SliderPosition);
                                EditTableText[2].text = PistonFraction2.ToString();
                                LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                                LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            }

                        }
                    }
                    if(TypeSelectionThree == "in range")//PistonThree
                    {
                        SelectorHighlighter.GetComponent<Renderer>().enabled = true;
                        SelectorHighlighter.transform.position = new Vector3(890,-195,0); //Highlights Piston Three
                        if(SliderPosition!=outofframe && HandlePosition!=outofframe)
                        {
                            if(LockMenu == "in range" && PistonFractionStatus2 == true)
                            {
                                PistonFraction3 = sliderValue(HandlePosition,SliderPosition);
                                LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                                LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                                EditTableText[3].text = PistonFraction3.ToString();
                                PistonFractionStatus3 = false;
                            }
                            else if(UnlockMenu == "in range")
                            {
                                PistonFractionStatus3 = true;
                                PistonFraction3 = sliderValue(HandlePosition,SliderPosition);
                                EditTableText[3].text = PistonFraction3.ToString();
                                LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                                LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            }

                        }
                    }
                    if(TypeSelectionWhole == "out of range")
                    {
                        SelectorHighlighter.GetComponent<Renderer>().enabled = false;
                    }

                }
                else if(EditSubMenuTwo == "in range") //Link Selected
                {
                    //Highlight Boom To Blue?
                    SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                    SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                    SubMenuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
                    MenuText[0].text = "Link Selection";
                    SliderPositionText[0].enabled = false;
                    SliderPositionText[1].enabled = true;
                    SliderPositionText[2].enabled = false;
                    Select1Block[0].GetComponent<Renderer>().enabled = false;
                    SliderPositionText[1].transform.position = new Vector3(475,-530,0);
                    if(TypeSelectionOne == "in range")//Link One
                    {
                        SelectorHighlighter.transform.position = new Vector3(890,-95,0);
                        SelectorHighlighter.GetComponent<Renderer>().enabled = true;
                        if(LockMenu == "in range" && LinkOverShootStatus1 == true)
                        {
                            BoomOverShootFraction1 = sliderValue(HandlePosition,SliderPosition);
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                            EditTableText[4].text = BoomOverShootFraction1.ToString();
                            LinkOverShootStatus1 = false;
                        }
                        else if(UnlockMenu == "in range")
                        {
                            LinkOverShootStatus1 = true;
                            BoomOverShootFraction1 = sliderValue(HandlePosition,SliderPosition);
                            EditTableText[4].text = BoomOverShootFraction1.ToString();
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                        }
                    }
                    if(TypeSelectionTwo == "in range")//Link Two
                    {
                        SelectorHighlighter.transform.position = new Vector3(890,-145,0); //Highlights Piston Two Selection
                        SelectorHighlighter.GetComponent<Renderer>().enabled = true;
                        if(LockMenu == "in range" && LinkOverShootStatus2 == true)
                        {
                            BoomOverShootFraction2 = sliderValue(HandlePosition,SliderPosition);
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                            EditTableText[5].text = BoomOverShootFraction2.ToString();
                            LinkOverShootStatus2 = false;
                        }
                        else if(UnlockMenu == "in range")
                        {
                            LinkOverShootStatus2 = true;
                            BoomOverShootFraction2 = sliderValue(HandlePosition,SliderPosition);
                            EditTableText[5].text = BoomOverShootFraction2.ToString();
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                        }
                    }
                    if(TypeSelectionThree == "in range")//Link Three
                    {
                        SelectorHighlighter.transform.position = new Vector3(890,-195,0); //Highlights Piston Three
                        SelectorHighlighter.GetComponent<Renderer>().enabled = true;
                        if(LockMenu == "in range" && LinkOverShootStatus2 == true)
                        {
                            BoomOverShootFraction3 = sliderValue(HandlePosition,SliderPosition);
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                            EditTableText[6].text = BoomOverShootFraction3.ToString();
                            LinkOverShootStatus3 = false;
                        }
                        else if(UnlockMenu == "in range")
                        {
                            LinkOverShootStatus3 = true;
                            BoomOverShootFraction3 = sliderValue(HandlePosition,SliderPosition);
                            EditTableText[6].text = BoomOverShootFraction3.ToString();
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                        }
                    }
                    if(TypeSelectionWhole == "out of range")
                    {
                        SelectorHighlighter.GetComponent<Renderer>().enabled = false;
                    }
                }
                else if(EditSubMenuThree == "in range") ///Joint Selected
                {
                    SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                    SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                    SubMenuArray[2].GetComponent<SpriteRenderer>().material.color = Color.blue;
                    MenuText[0].text = "Joint Selection";
                    SliderPositionText[0].enabled = false;
                    SliderPositionText[1].enabled = false;
                    SliderPositionText[2].enabled = true;
                    SliderPositionText[2].transform.position = new Vector3(475,-530,0);
                    Select1Block[0].GetComponent<Renderer>().enabled = true;
                    Select1Block[0].transform.position = new Vector3(925, -95, 0);
                    if(TypeSelectionTwo == "in range")//Link Two
                    {
                        SelectorHighlighter.transform.position = new Vector3(890,-145,0); //Highlights Piston Two Selection
                        SelectorHighlighter.GetComponent<Renderer>().enabled = true;
                        if(LockMenu == "in range" && JointFractionStatus2 == true)
                        {
                            JointFraction2 = sliderValue(HandlePosition,SliderPosition);
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                            EditTableText[8].text = JointFraction2.ToString();
                            JointFractionStatus2 = false;
                        }
                        else if(UnlockMenu == "in range")
                        {
                            JointFractionStatus2 = true;
                            JointFraction2 = sliderValue(HandlePosition,SliderPosition);
                            EditTableText[8].text = JointFraction2.ToString();
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                        }
                    }
                    if(TypeSelectionThree == "in range")//Link Three
                    {
                        SelectorHighlighter.transform.position = new Vector3(890,-195,0); //Highlights Piston Three
                        SelectorHighlighter.GetComponent<Renderer>().enabled = true;
                        if(LockMenu == "in range" && JointFractionStatus3 == true)
                        {
                            JointFraction3 = sliderValue(HandlePosition,SliderPosition);
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                            EditTableText[9].text = JointFraction3.ToString();
                            JointFractionStatus3 = false;
                        }
                        else if(UnlockMenu == "in range")
                        {
                            JointFractionStatus3 = true;
                            JointFraction3 = sliderValue(HandlePosition,SliderPosition);
                            EditTableText[9].text = JointFraction3.ToString();
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                        }
                    }
                    if(TypeSelectionWhole == "out of range")
                    {
                        SelectorHighlighter.GetComponent<Renderer>().enabled = false;
                    }
                }
                else
                {
                    SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                    SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                    SubMenuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
                    MenuText[0].text = "Choose Sub-Menu Option";
                }



            }


        //Boom 1 Positions
            Vector3 FixedBoom1 = receivedPos5;
            Vector3 EndBoom1 = receivedPos6;
            Vector3 StartPiston1 = receivedPos7;


            Vector3 StartBoom1 = BoomStartFinder(FixedBoom1, EndBoom1, BoomOverShootFraction1);
            Vector3 EndPiston1 = PistonEnd(StartBoom1, EndBoom1, PistonFraction1);

        //Boom 2 Positions

            Vector3 EndBoom2 = receivedPos10; //???
            Vector3 StartPiston2 = receivedPos11; //???

            Vector3 FixedBoom2 = BoomFixedFinder(FixedBoom1, EndBoom1, JointFraction2);
            Vector3 StartBoom2 = BoomStartFinder(FixedBoom2, EndBoom2, BoomOverShootFraction2);
            Vector3 EndPiston2 = PistonEnd(StartBoom2, EndBoom2, PistonFraction2);


         //B3 Positions
            Vector3 EndBoom3 = receivedPos12;
            Vector3 StartPiston3 = receivedPos13;

            Vector3 FixedBoom3 = BoomFixedFinder(FixedBoom2, EndBoom2, JointFraction3);
            Vector3 StartBoom3 = BoomStartFinder(FixedBoom3, EndBoom3, BoomOverShootFraction3);
            Vector3 EndPiston3 = PistonEnd(StartBoom3, EndBoom3, PistonFraction3);

        //render components only if they exist
            void IfExistFixedBoom(Vector3 theAruco, int i){
                if(theAruco != outofframe)
                {
                    squareArray[i].SetActive(true);
                    squareArray[i].transform.position = theAruco;
                    Overshoot[i].transform.position = StartBoom1;
                }
                else
                {
                    squareArray[i].SetActive(false);
                }
            }
            void IfExistFreeBoom(Vector3 theAruco, int i, int j){
                if(theAruco != outofframe)
                {
                    squareArray[i].SetActive(true);
                    squareArray[i].transform.position = theAruco;
                    BoomEnd[j].transform.position = theAruco;
                }
                else
                {
                    squareArray[i].SetActive(false);
                }
            }
            void IfExistPiston(Vector3 theAruco, Vector3 PistonEndPos, int i,int j){
                if(theAruco != outofframe)
                {
                    squareArray[i].SetActive(true);
                    squareArray[i].transform.position = theAruco;
                    PistonOneEnd[j].transform.position = PistonEndPos;
                }
                else
                {
                    squareArray[i].SetActive(false);
                }
            }

            IfExistFixedBoom(FixedBoom1,0);
            IfExistFreeBoom(EndBoom1,1,0);
            IfExistPiston(StartPiston1,EndPiston1,2,0);
            IfExistFreeBoom(EndBoom2,3,1);
            IfExistPiston(StartPiston2,EndPiston2,4,1);



            void IfExistBoomLine(Vector3 StartBoomPos, Vector3 FixedBoomPos, Vector3 EndBoomPos, int i )
            {
                if(FixedBoomPos!=outofframe&&EndBoomPos!=outofframe)
                {
                    BoomLine[i].GetComponent<Renderer>().enabled = true;
                    Vector3[] LinePosition1 = {StartBoomPos,EndBoomPos};
                    BoomLine[i].SetPositions(LinePosition1);
                }
                else
                {
                    BoomLine[i].GetComponent<Renderer>().enabled = false;
                }

            }

            IfExistBoomLine(StartBoom1,FixedBoom1,EndBoom1,0);
            IfExistBoomLine(StartBoom2,EndBoom1,EndBoom2,1);

            void IfExistPistonMovingLine(Vector3 EndPistonPos,Vector3 StartPistonPos,  Vector3 StartBoomPos, Vector3 EndBoomPos, int i)
            {
                if(StartPistonPos!=outofframe&&StartBoomPos!=outofframe&&EndBoomPos!=outofframe)
                {
                    PistonMovingLine[i].GetComponent<Renderer>().enabled = true;
                    Vector3[] LinePosition2 = {StartPistonPos,EndPistonPos};
                    PistonMovingLine[i].SetPositions(LinePosition2);
                }
                else
                {
                    PistonMovingLine[i].GetComponent<Renderer>().enabled = false;
                }
            }
            IfExistPistonMovingLine(EndPiston1,StartPiston1,FixedBoom1,EndBoom1,0);
            IfExistPistonMovingLine(EndPiston2,StartPiston2,EndBoom1,EndBoom2,1);


            //void IfExistPistonFixedLine(Vector3 StartPistonPos, Vector3 EndPistonPos, Vector3 StartBoomPos, Vector3 EndBoomPos, int i)
            // {
            //     if(StartPistonPos!=outofframe&&StartBoomPos!=outofframe&&EndBoomPos!=outofframe)
            //     {
            //         PistonFixedLine[i].GetComponent<Renderer>().enabled = true;
            //         Vector3[] LinePosition2 = {StartPistonPos,EndPistonPos};
            //         PistonFixedLine[i].SetPositions(LinePosition2);
            //     }
            //     else
            //     {
            //         PistonFixedLine[i].GetComponent<Renderer>().enabled = false;
            //     }
            // }
            //IfExistPistonFixedLine(StartPiston1,EndPiston1,FixedBoom1,EndBoom1,0);
            //IfExistPistonFixedLine(StartPiston2,EndPiston2,EndBoom1,EndBoom2,1);

        //Boom 1 Calcs

	        Vector3[] BoomArray1 = BoomRotationCalculation(FixedBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
            Vector3[] BoomArray1Start = BoomStartArray(FixedBoom1, BoomArray1, BoomOverShootFraction1);
            Vector3[] PistonArray1 = PistonRotationCalculation(FixedBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
            float[] AngleChangeBoom1 = RotationFromStartCalculation(BoomArray1, FixedBoom1);
            float TotalBoomRange1 = BoomRangeCalculation(AngleChangeBoom1);
            Vector3[] Omega1 = AngularVelocityCalculation(BoomArray1, TimeStep, FixedBoom1); //x=time, y=omega
            Vector3[] Omega1Contract = VelocityContracting(Omega1); //Array position still correspond to position in BoomArray - therefore time is going from total time to 0 from i=0 to end
            Vector3[] V1 = VelocityCalculation(Omega1, FixedBoom1, EndBoom1); //x=time, y=v
            Vector3[] V1Contract = VelocityContracting(V1);

            int ArrayLength1 = BoomArray1.Length;
            int OverShootArrayLength1 = BoomArray1Start.Length;

            BoomCurve[0].positionCount = BoomArray1.Length;
            OverShootCurve[0].positionCount = BoomArray1Start.Length;

        //Game Mode Menu
            Vector3 GameModeAruco = receivedPos25;
            int xminGame = 775;
            int xmaxGame = 825;
            int yminGame = -100;
            int ymaxGame = 0;
            string InGameMode = InMenuRegion(xminGame,xmaxGame,yminGame,ymaxGame,GameModeAruco);

        //Excavator Fixed Pos Range
            Vector3 ExcavatorPos = new Vector3(265,-369,0);
            int xminExc = (int)ExcavatorPos[0]-10;
            int xmaxExc = (int)ExcavatorPos[0]+10;
            int yminExc = (int)ExcavatorPos[1]-10;
            int ymaxExc = (int)ExcavatorPos[1]+10;
            string InExcavatorCircle = InMenuRegion(xminExc,xmaxExc,yminExc,ymaxExc,FixedBoom1);

        //Object To Retrieve (sand) Fixed Pos Range
            Vector3 ObjectToRetrievePos = new Vector3(673,-391,0);
            int xminObj = (int)ObjectToRetrievePos[0]-70;
            int xmaxObj = (int)ObjectToRetrievePos[0]+70;
            int yminObj = (int)ObjectToRetrievePos[1]-50;
            int ymaxObj = (int)ObjectToRetrievePos[1]+5;

            string InObjectPosBoomArray1 = InMenuRegion(xminObj,xmaxObj,yminObj,ymaxObj,BoomArray1[ArrayLength1-1]);



        //Boom 2
        //if(NumberOfBooms>1)
        //    {
        //if(receivedPos10!=outofframe && receivedPos11!=outofframe)
        //{
         //Arrays of Translated Positions relative to boom1 rotaions
            Vector3[] EndBoom2Array = RelativePosition(FixedBoom1, EndBoom2, AngleChangeBoom1);
            Vector3[] StartPiston2Array = RelativePosition(FixedBoom1, StartPiston2, AngleChangeBoom1);
            Vector3[] FixedBoom2Array = RelativePosition(FixedBoom1, FixedBoom2, AngleChangeBoom1);
            Vector3[] StartBoom2Array = RelativePosition(FixedBoom1, StartBoom2, AngleChangeBoom1);
            Vector3[] EndPiston2Array = RelativePosition(FixedBoom1, EndPiston2, AngleChangeBoom1);

         //Boom2 Calcs
            Vector3[] BoomArray2 = BoomRotationCalculation(FixedBoom2, EndBoom2, StartPiston2, BoomOverShootFraction2, PistonFraction2,TimeStep);
            Vector3[] BoomArray2Start = BoomStartArray(FixedBoom2, BoomArray2, BoomOverShootFraction2);
            Vector3[] PistonArray2 = PistonRotationCalculation(FixedBoom2, EndBoom2, StartPiston2, BoomOverShootFraction2, PistonFraction2,TimeStep);
            float[] AngleChangeBoom2 = RotationFromStartCalculation(BoomArray2, FixedBoom2);
            float TotalBoomRange2 = BoomRangeCalculation(AngleChangeBoom2);
            Vector3[] Omega2 = AngularVelocityCalculation(BoomArray2, TimeStep, FixedBoom2);
            Vector3[] Omega2Contract = VelocityContracting(Omega2);
            Vector3[] V2 = VelocityCalculation(Omega2, FixedBoom2, EndBoom2);
            Vector3[] V2Contract = VelocityContracting(V2);

         //Boom2 Translated Arrays - Vector3[Position Through Boom 1 Movement, Position Through Boom 2 Movement]
            Vector3[,] BoomArray2Array = ArrayRelativePosition(FixedBoom1, BoomArray2, AngleChangeBoom1);
            Vector3[,] BoomArray2StartArray = ArrayRelativePosition(FixedBoom1, BoomArray2Start, AngleChangeBoom1);
            Vector3[,] PistonArray2Array = ArrayRelativePosition(FixedBoom1, PistonArray2, AngleChangeBoom1);

         //Max Reach of Both Booms
            Vector3[] MaxReachPositionsBoom2 = MaxRangePosition(FixedBoom1, BoomArray1, BoomArray2Array);
         //Velocity at Combined
            float[,] EndVelocityBoom1ExtendBoom2Extend = TwoMovingBoomsVelocity(Omega1, FixedBoom1, FixedBoom2Array, V2, BoomArray2Array); //Boom1 Extend Boom2 Extending
            float[,] EndVelocityBoom1ExtendBoom2Contract = TwoMovingBoomsVelocity(Omega1, FixedBoom1, FixedBoom2Array, V2Contract, BoomArray2Array); //Boom1 Extend Boom2 Contract
            float[,] EndVelocityBoom1ContractBoom2Extend = TwoMovingBoomsVelocity(Omega1Contract, FixedBoom1, FixedBoom2Array, V2, BoomArray2Array); //Boom1 Contract Boom2 Extending
            float[,] EndVelocityBoom1ContractBoom2Contract = TwoMovingBoomsVelocity(Omega1Contract, FixedBoom1, FixedBoom2Array, V2Contract, BoomArray2Array); //Boom1 Contract Boom2 Contract

         //Boom 2 Array Length
            int ArrayLength2 = BoomArray2Array.GetLength(0);
            BoomCurve[1].positionCount = ArrayLength2;
        // //}

        //Boom3
        //if(NumberOfBooms>2)
        //    {


            //Fixed Position due to boom 2 movement
        //     Vector3[] FixedBoom3Array = RelativePosition(FixedBoom2, FixedBoom3, AngleChangeBoom2);
        //     //Fixed Position due for each boom 1 and 2 movement [i,j]
        //     Vector3[,] FixedBoom3ArrayArray = ArrayRelativePosition(FixedBoom1, BoomArray2Start, AngleChangeBoom1);
        //  //Boom3 Calcs
        //     Vector3[] BoomArray3 = BoomRotationCalculation(FixedBoom3, EndBoom3, StartPiston3, BoomOverShootFraction3, PistonFraction3,TimeStep);
        //     Vector3[] BoomArray3Start = BoomStartArray(FixedBoom3, BoomArray3, BoomOverShootFraction3);
        //     Vector3[] PistonArray3 = PistonRotationCalculation(FixedBoom3, EndBoom3, StartPiston3, BoomOverShootFraction3, PistonFraction3,TimeStep);
        //     float[] AngleChangeBoom3 = RotationFromStartCalculation(BoomArray3, FixedBoom3);
        //     float TotalBoomRange3 = BoomRangeCalculation(AngleChangeBoom3);
        //     Vector3[] Omega3 = AngularVelocityCalculation(BoomArray3, TimeStep, FixedBoom3);
        //     Vector3[] Omega3Contract = VelocityContracting(Omega3);
        //     Vector3[] V3 = VelocityCalculation(Omega3, FixedBoom3, EndBoom3);
        //     Vector3[] V3Contract = VelocityContracting(V3);
        //  //Boom 3 Relative to Other Booms Movement
        //     //Boom 3 Relative to Boom 2 Movement [j,k]
        //     Vector3[,] BoomArray3Array = ArrayRelativePosition(FixedBoom2, BoomArray3, AngleChangeBoom2);
        //     Vector3[,] BoomArray3StartArray = ArrayRelativePosition(FixedBoom2, BoomArray3Start, AngleChangeBoom2);
        //     Vector3[,] PistonArray3Array = ArrayRelativePosition(FixedBoom2, PistonArray3, AngleChangeBoom2);
        //     //Boom 3 Relative to Both Boom previous [i,j,k]
        //     Vector3[,,] BoomArray3Array3D = ArrayRelativePosition3Booms(FixedBoom1, BoomArray3Array, AngleChangeBoom1);
        //     Vector3[,,] BoomArray3StartArray3D = ArrayRelativePosition3Booms(FixedBoom1, BoomArray3StartArray, AngleChangeBoom1);
        //     Vector3[,,] PistonArray3Array3D = ArrayRelativePosition3Booms(FixedBoom1, PistonArray3Array, AngleChangeBoom1);
        //  //MaxBoomRange
        //     //Boom3 from Boom2
        //     Vector3[] MaxReachPositionsBoom3FromBoom2 = MaxRangePosition(FixedBoom2, BoomArray2, BoomArray3Array);
        //     //Boom3 from Boom1
        //     Vector3[,] MaxReachBoom3From2RelativeArrayForBoom1Rotation = ArrayRelativePosition(FixedBoom1, MaxReachPositionsBoom3FromBoom2, AngleChangeBoom1);
        //     Vector3[] MaxReachPositionsBoom3 = MaxRangePosition(FixedBoom1, BoomArray1, MaxReachBoom3From2RelativeArrayForBoom1Rotation);
        //  //Velocities Per Position
        //     //Boom3 with boom 2
        //     float[,] EndVelocityBoom2ExtendBoom3Extend = TwoMovingBoomsVelocity(Omega2, FixedBoom2, FixedBoom3Array, V3, BoomArray3Array); //Boom1 Extend Boom2 Extending
        //     float[,] EndVelocityBoom2ExtendBoom3Contract = TwoMovingBoomsVelocity(Omega2, FixedBoom2, FixedBoom3Array, V3Contract, BoomArray3Array); //Boom1 Extend Boom2 Contract
        //     float[,] EndVelocityBoom2ContractBoom3Extend = TwoMovingBoomsVelocity(Omega2Contract, FixedBoom2, FixedBoom3Array, V3, BoomArray3Array); //Boom1 Contract Boom2 Extending
        //     float[,] EndVelocityBoom2ContractBoom3Contract = TwoMovingBoomsVelocity(Omega2Contract, FixedBoom2, FixedBoom3Array, V3Contract, BoomArray3Array); //Boom1 Contract Boom2 Contract
        //     //Boom 3 end due to boom 2 and boom 1
        //     float[,,] EndVelocity3BoomEEE = ThreeMovingBoomsVelocity(Omega1, FixedBoom1, FixedBoom2Array, EndVelocityBoom2ExtendBoom3Extend,BoomArray3Array3D);
        //     float[,,] EndVelocity3BoomEEC = ThreeMovingBoomsVelocity(Omega1, FixedBoom1, FixedBoom2Array, EndVelocityBoom2ExtendBoom3Contract,BoomArray3Array3D);
        //     float[,,] EndVelocity3BoomECE = ThreeMovingBoomsVelocity(Omega1, FixedBoom1, FixedBoom2Array, EndVelocityBoom2ContractBoom3Extend,BoomArray3Array3D);
        //     float[,,] EndVelocity3BoomECC = ThreeMovingBoomsVelocity(Omega1, FixedBoom1, FixedBoom2Array, EndVelocityBoom2ContractBoom3Contract,BoomArray3Array3D);
        //     float[,,] EndVelocity3BoomCEE = ThreeMovingBoomsVelocity(Omega1Contract, FixedBoom1, FixedBoom2Array, EndVelocityBoom2ExtendBoom3Extend,BoomArray3Array3D);
        //     float[,,] EndVelocity3BoomCEC = ThreeMovingBoomsVelocity(Omega1Contract, FixedBoom1, FixedBoom2Array, EndVelocityBoom2ExtendBoom3Contract,BoomArray3Array3D);
        //     float[,,] EndVelocity3BoomCCE = ThreeMovingBoomsVelocity(Omega1Contract, FixedBoom1, FixedBoom2Array, EndVelocityBoom2ContractBoom3Extend,BoomArray3Array3D);
        //     float[,,] EndVelocity3BoomCCC = ThreeMovingBoomsVelocity(Omega1Contract, FixedBoom1, FixedBoom2Array, EndVelocityBoom2ContractBoom3Contract,BoomArray3Array3D);
        //    }
        // if(MenuData[0]==0)
        //     {



        //     //Game Mode - show excavator
        //     if(InGameMode == "in range")
        //     {
        //     ExcavatorBase.GetComponent<Renderer>().enabled=true;
        //     ObjectToRetrieve.GetComponent<Renderer>().enabled=true;
        //     ExcavatorBase.transform.position = ExcavatorPos;
        //     ObjectToRetrieve.transform.position = ObjectToRetrievePos;
        //     ExcavatorBaseCircle.GetComponent<Renderer>().enabled=true;
        //     ExcavatorBaseCircle.transform.position = ExcavatorPos;
        //     menuGameMode.GetComponent<SpriteRenderer>().material.color = Color.blue;
        //     //Fixed Joint in Excavator Circle Error Message
        //     if(InExcavatorCircle == "out of range")
        //     {
        //         GameModeErrorMessage.enabled = true;
        //         GameModeErrorMessage.text = "Place Fixed Joint on Excavator";
        //         GameStatus = false;
        //     }
        //     else
        //     {
        //         GameModeErrorMessage.enabled = false;
        //         GameStatus = true;
        //     }
        //     }
        //     else if (InGameMode == "out of range")
        //     {
        //     GameStatus = true;
        //     GameModeErrorMessage.enabled = false;
        //     ExcavatorBase.GetComponent<Renderer>().enabled=false;
        //     ExcavatorBaseCircle.GetComponent<Renderer>().enabled=false;
        //     ObjectToRetrieve.GetComponent<Renderer>().enabled=false;
        //     menuGameMode.GetComponent<SpriteRenderer>().material.color = Color.white;
        //     }



         //   }
        if(MenuData[0]==1)
            {
            //Menu
            SubMenuText[0].text = "Play";
            SubMenuText[1].text = "Adjust Position";
            SubMenuText[2].enabled = false;
            SubMenuArray[2].GetComponent<Renderer>().enabled=false;
            AnimateMenuText[0].enabled = true;
            AnimateMenuText[1].enabled = true;
            AnimateMenuText[2].enabled = true;
            AnimateMenuArray[0].GetComponent<SpriteRenderer>().enabled = true;
            AnimateMenuArray[1].GetComponent<SpriteRenderer>().enabled = true;
            AnimateMenuArray[2].GetComponent<SpriteRenderer>().enabled = true;
            AnimateMenuText[0].transform.position = new Vector3(700,-530,0);
            AnimateMenuText[1].transform.position = new Vector3(746,-530,0);
            AnimateMenuText[2].transform.position = new Vector3(800,-530,0);
            AnimateMenuArray[0].transform.position = new Vector3(700,-565,0);
            AnimateMenuArray[1].transform.position = new Vector3(750,-565,0);
            AnimateMenuArray[2].transform.position = new Vector3(800,-565,0);
            SubMenuArray[2].transform.position = new Vector3(-100,0,0);
            //Animate Table
            //enable
            AnimateTable[0].enabled = true;
            AnimateTable[1].enabled = true;
            AnimateTable[2].enabled = true;
            AnimateTable[3].enabled = true;
            AnimateTable[4].enabled = true;
            AnimateTable[5].enabled = true;
            AnimateTable[6].enabled = true;
            AnimateTable[7].enabled = true;
            //transform
            AnimateTable[0].transform.position = new Vector3(500,-32,0); //text
            AnimateTable[1].transform.position = new Vector3(640,7,0); //number
            AnimateTable[2].transform.position = new Vector3(575,-15,0); //percentage 1
            AnimateTable[3].transform.position = new Vector3(665,-15,0); //percentage 2
            AnimateTable[4].transform.position = new Vector3(755,-15,0); //percentage 3
            AnimateTable[5].transform.position = new Vector3(580,-32,0); //animation type 1
            AnimateTable[6].transform.position = new Vector3(670,-32,0); //animation type 2
            AnimateTable[7].transform.position = new Vector3(760,-32,0); //animation type 3


            BoomCurve[0].GetComponent<Renderer>().enabled=true;
            menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
            menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
            menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;

            if(EditSubMenuTwo == "in range") //if in adjust
            {
                SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue; //highlight blue
                if(TypeSelectionOne == "in range") // if in link 1
                {
                    SelectorHighlighter.GetComponent<Renderer>().enabled = true;
                    SelectorHighlighter.transform.position = new Vector3(890,-95,0); //highlighter line
                    if(LockMenu == "in range" && PositionStatus1 == true)
                        {
                            AnimationPosition1 = sliderValue(HandlePosition,SliderPosition);
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                            AnimateTable[2].text = (AnimationPosition1*100).ToString()+"%";
                            BoomArray1AnimatePosition = (int)((ArrayLength1-1)*AnimationPosition1);
                            PositionStatus1 = false;
                        }
                        else if(UnlockMenu == "in range")
                        {
                            PositionStatus1 = true;
                            AnimationPosition1 = sliderValue(HandlePosition,SliderPosition);
                            AnimateTable[2].text = (AnimationPosition1*100).ToString()+"%";
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            BoomArray1AnimatePosition = (int)((ArrayLength1-1)*AnimationPosition1);
                        }
                }
                if(TypeSelectionTwo == "in range") // if in link 2
                {
                    SelectorHighlighter.GetComponent<Renderer>().enabled = true;
                    SelectorHighlighter.transform.position = new Vector3(890,-145,0);; //highlighter line
                    if(LockMenu == "in range" && PositionStatus2 == true)
                        {
                            AnimationPosition2 = sliderValue(HandlePosition,SliderPosition);
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                            AnimateTable[3].text = (AnimationPosition2*100).ToString()+"%";
                            BoomArray2AnimatePosition = (int)((ArrayLength2-1)*AnimationPosition2);
                            PositionStatus2 = false;
                        }
                        else if(UnlockMenu == "in range")
                        {
                            PositionStatus2= true;
                            AnimationPosition2= sliderValue(HandlePosition,SliderPosition);
                            AnimateTable[3].text = (AnimationPosition2*100).ToString()+"%";
                            LockMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                            LockMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                            BoomArray2AnimatePosition = (int)((ArrayLength2-1)*AnimationPosition2);
                        }
                }
            }


            void IfExistBoomLineAnimate(Vector3 StartBoomPos, Vector3 FixedBoomPos, Vector3 EndBoomPos, Vector3 FractionThroughBoomPos, int i )
            {
                if(FixedBoomPos!=outofframe&&EndBoomPos!=outofframe)
                {
                    BoomLine[i].GetComponent<Renderer>().enabled = true;
                    Vector3[] LinePosition1 = {StartBoomPos,FractionThroughBoomPos};
                    BoomLine[i].SetPositions(LinePosition1);
                }
                else
                {
                    BoomLine[i].GetComponent<Renderer>().enabled = false;
                }

            }
            //plot link line one where the chosen slider position is
            IfExistBoomLineAnimate(BoomArray1Start[BoomArray1AnimatePosition],FixedBoom1,EndBoom1,BoomArray1[BoomArray1AnimatePosition],0);
            //link two
            IfExistBoomLineAnimate(BoomArray2StartArray[BoomArray1AnimatePosition,BoomArray2AnimatePosition],EndBoom1,EndBoom2,BoomArray2Array[BoomArray1AnimatePosition,BoomArray2AnimatePosition],1);


            void IfExistPistonMovingLineAnimate(Vector3 EndPistonPos,Vector3 StartPistonPos,  Vector3 StartBoomPos, Vector3 EndBoomPos, int i)
            {
                if(StartPistonPos!=outofframe&&StartBoomPos!=outofframe&&EndBoomPos!=outofframe)
                {
                    PistonMovingLine[i].GetComponent<Renderer>().enabled = true;
                    Vector3[] LinePosition2 = {StartPistonPos,EndPistonPos};
                    PistonMovingLine[i].SetPositions(LinePosition2);
                }
                else
                {
                    PistonMovingLine[i].GetComponent<Renderer>().enabled = false;
                }
            }
            //plot piston link line one where chosen slider position is
            IfExistPistonMovingLineAnimate(PistonArray1[BoomArray1AnimatePosition],StartPiston1,FixedBoom1,EndBoom1,0);
            //piston two
             IfExistPistonMovingLineAnimate(PistonArray2Array[BoomArray1AnimatePosition,BoomArray2AnimatePosition],StartPiston2,EndBoom1,EndBoom2,1);



            // if(EditSubMenuOne == "in range" && AnimationOneStatus==false)
            // {
            // AnimationOneStatus = true;
            // SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
            // SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
            // StartCoroutine(FollowPistonOnePath());
            // StartCoroutine(FollowLinkOnePath());

            // }
            // if(EditSubMenuTwo == "in range")
            // {
            // Vector3 StoppedPos = BoomEnd[0].transform.position;
            // int StoppedPosIndex = System.Array.IndexOf(BoomArray1,StoppedPos);
            // print(StoppedPosIndex);
            // }

            //render purple circles

            void IfExistFreeBoomAnimate(Vector3 theAruco, Vector3 FractionThroughBoomPos, int i){
                if(theAruco != outofframe)
                {
                    AnimateObjects[i].SetActive(true);
                    AnimateObjects[i].transform.position = FractionThroughBoomPos;
                    BoomEnd[i].transform.position = FractionThroughBoomPos;
                }
                else
                {
                    AnimateObjects[i].SetActive(false);
                }
            }

            IfExistFreeBoomAnimate(EndBoom1,BoomArray1[BoomArray1AnimatePosition],0); //circle one
            IfExistFreeBoomAnimate(EndBoom2,BoomArray2Array[BoomArray1AnimatePosition,BoomArray2AnimatePosition],1); //two



            Vector3[] BoomArray2Array1D = new Vector3[ArrayLength2];
            for (int j=0; j<ArrayLength2; j++)
            {
                Vector3 PositionInArray = BoomArray2Array[BoomArray1AnimatePosition,j];
                float x = PositionInArray[0];
                float y = PositionInArray[1];
                BoomArray2Array1D[j] = new Vector3(x,y,0f);
            }


            //void BoomCurveWorkspace(Vector3 StartPiston,Vector3 StartBoom,Vector3 EndBoom, Vector3[] theArray,int ArrayLength,int whichCurve)
            //{
                // if(StartPiston!=outofframe&&StartBoom!=outofframe&&EndBoom!=outofframe)
                // {
                //     for(int i=0; i<ArrayLength; i++)
                //     {
                //         BoomCurve[whichCurve].GetComponent<Renderer>().enabled = true;
                //         BoomCurve[whichCurve].SetPosition(i,theArray[i]);
                //     }
                // }
                // else
                // {
                //     BoomCurve[whichCurve].GetComponent<Renderer>().enabled = false;
                // }
            //}
            //workspace one
            //BoomCurveWorkspace(StartPiston1,FixedBoom1,EndBoom1,BoomArray1,ArrayLength1,0);
            //two
            //BoomCurveWorkspace(StartPiston2,EndBoom1,EndBoom2,BoomArray2Array1D,ArrayLength2,1);

            if(StartPiston1!=outofframe&&FixedBoom1!=outofframe&&EndBoom1!=outofframe)
                {
                    for(int i=0; i<ArrayLength1; i++)
                    {
                        BoomCurve[0].GetComponent<Renderer>().enabled = true;
                        BoomCurve[0].SetPosition(i,BoomArray1[i]);
                    }
                }
            else
            {
                BoomCurve[0].GetComponent<Renderer>().enabled = false;
            }
            if(StartPiston2!=outofframe&&EndBoom1!=outofframe&&EndBoom2!=outofframe)
                {
                    for(int i=0; i<ArrayLength2; i++)
                    {
                        BoomCurve[1].GetComponent<Renderer>().enabled = true;
                        BoomCurve[1].SetPosition(i,BoomArray2Array1D[i]);
                    }
                }
            else
            {
                BoomCurve[1].GetComponent<Renderer>().enabled = false;
            }


            //In Game Mode if retrieves object
            // if(InObjectPosBoomArray1 == "in range")
            // {
            // GameModeObjectRetrieveMessage.text = "Success!";
            // }
            // else if(InObjectPosBoomArray1 == "out of range")
            // {
            // GameModeObjectRetrieveMessage.text = "u suck shit try again";
            //}
            //If in Stop Animation One Box
            // if(InStopAnimationOne == "in range")
            // {
            //     StopAnimation[0].GetComponent<Renderer>().material.color = Color.blue;
            // }
            // else if(InStopAnimationOne == "out of range")
            // {
            //     StopAnimation[0].GetComponent<Renderer>().material.color = Color.white;
            // }

            }

            //Drawing graph




        if (MenuData[0]==2)
            {
            menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
            menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
            menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.blue;

            //drawing graph
            float XmaxV1 = FindMaxX(V1);
            Vector3[] NewV1Contract = NewContract(XmaxV1, V1Contract);

            Vector3[] V1Total = CombineVector3Arrays(V1,NewV1Contract); //combining V and VContract
            float Xminori = FindMinX(V1Total); //original data
            float Xmaxori = FindMaxX(V1Total);
            float Yminori = FindMinY(V1Total);
            float Ymaxori = FindMaxY(V1Total);

            float lengthYori = Math.Abs(Yminori)+Math.Abs(Ymaxori);
            float ratioa = Ymaxori/lengthYori; //upper positive ratio
            float ratiob = 1f-ratioa; //lower negative ratio
            Debug.Log("print"+ratiob);

            float zeroX = 1250f; //fixed
            Vector3 fixedYmax = new Vector3(zeroX, -130f, 0f); //with offset
            Vector3 fixedYmin = new Vector3(zeroX, -550f, 0f);

            float Xtotal = 720f; //without offset - fixed
            float Xscale = Xtotal/Xmaxori; //scaling factor
            float Ytotal = 420f; //without offset - fixed based on fixedymax and fixedymin
            float Yscale = Ytotal/lengthYori;

            Vector3[] scaledV1 = ScaleData(V1Total, Xscale, Yscale); //times everything in data to scaling factor
            float Xminscaled = FindMinX(scaledV1); //scaled
            float Xmaxscaled = FindMaxX(scaledV1);
            float Yminscaled = FindMinY(scaledV1);
            float Ymaxscaled = FindMaxY(scaledV1);

            float lengthYscaled = Math.Abs(Yminscaled)+Math.Abs(Ymaxscaled);
            // float zeroY = Yminscaled + (lengthYscaled*ratiob);
            float zeroY = -325f;
            Debug.Log("print"+zeroY);

            Vector3 zerozero = new Vector3 (zeroX, zeroY, 0f);

            Vector3[] finalV1 = MoveData(scaledV1, zerozero);
            float Xminfinal = FindMinX(finalV1); //moved and scaled
            float Xmaxfinal = FindMaxX(finalV1);
            float Yminfinal = FindMinY(finalV1);
            float Ymaxfinal = FindMaxY(finalV1);

            //Drawing axis
            Vector3 axisXmax = Xfloattov3(Xmaxfinal, zerozero, 20);

            Vector3[] xaxis = {zerozero, axisXmax};
            Graphsplot[0].SetPositions(xaxis);

            Vector3[] yaxis = {fixedYmin, fixedYmax};
            Graphsplot[1].SetPositions(yaxis);

            //plot graph
            int finalV1length = finalV1.Count();
            Graphsplot[2].positionCount = finalV1length;
            for(int i=0; i<finalV1length; i++)
            {
                Graphsplot[2].SetPosition(i,finalV1[i]);
            }
            //text
            DataText[0].text = "Velocity";
            DataText[0].transform.position = new Vector3(1250,-50,0);
            DataText[1].text = "Time";
            DataText[1].transform.position = new Vector3(2010,-325,0);
            DataText[2].text = "Velocity Graph";
            DataText[2].transform.position = new Vector3(1400,-30,0);
            DataText[3].text = "Range of movement:";
            DataText[3].transform.position = new Vector3(1600,-30,0);
            DataText[4].text = ((float)Math.Round(Xmaxori,1)).ToString();
            DataText[4].transform.position = new Vector3(1970,-355,0);
            DataText[5].text = ((float)Math.Round(Ymaxori,1)).ToString();
            DataText[5].transform.position = new Vector3(1220,-110,0);
            DataText[6].text = ((float)Math.Round(Yminori,1)).ToString();
            DataText[6].transform.position = new Vector3(1220,-530,0);
            DataText[7].text = ((float)Math.Round(TotalBoomRange1,1)).ToString();
            DataText[7].transform.position = new Vector3(1850,-30,0);


            //plot max reach
            void MaxReachCurve(Vector3 StartPiston,Vector3 StartBoom,Vector3 EndBoom, Vector3[] theArray,int ArrayLength,int whichCurve)
            {
                if(StartPiston!=outofframe&&StartBoom!=outofframe&&EndBoom1!=outofframe)
                {
                    for(int i=0; i<ArrayLength; i++)
                    {
                        BoomCurve[whichCurve].GetComponent<Renderer>().enabled = true;
                        BoomCurve[whichCurve].SetPosition(i,theArray[i]);
                    }
                }
                else
                {
                    BoomCurve[whichCurve].GetComponent<Renderer>().enabled = false;
                }
            }

            MaxReachCurve(StartPiston1,FixedBoom1,EndBoom1,BoomArray1,ArrayLength1,0);
            MaxReachCurve(StartPiston2,EndBoom1,EndBoom2,MaxReachPositionsBoom2,MaxReachPositionsBoom2.Length,1);



            }

        //What needs animating though movement of boom one at the same time
            //       through number of points = ArrayLength1    //   for(i=0; i<ArrayLength1 ; i++)   //    Time between each step = TimeStep? maybe
            //       Boom1 - Line from BoomArray1Start[i] to BoomArray1[i]
            //       Piston1 - Line from StartPiston1 to PistonArray[i]
            //       Boom1 Curve - BoomArray1
            //       Boom2 - Line from BoomArray2StartArray[i,0] to BoomArray2Array[i,0]
            //       Piston2 - Line from StartPiston2Array[i] to PistonArray2Array[i,0]
            //       Boom2 Curve - BoomArray2Array[i, ]



    }

    public IEnumerator FollowLinkOnePath()
        {
        //Maths
            //Time Step Generic
                float TimeStep = TheTimeStep();

            //Boom 1 Data
                float BoomOverShootFraction1 = 0f;
                //float PistonFraction1 = sliderValue(receivedPos9,receivedPos8);
                float PistonFraction1 = 1f;

            //Boom 1 Positions
                Vector3 FixedBoom1 = receivedPos5;
                Vector3 EndBoom1 = receivedPos6;
                Vector3 StartPiston1 = receivedPos7;
            //Boom 1 Calcs
	            Vector3[] BoomArray1 = BoomRotationCalculation(FixedBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
                Vector3[] BoomArray1Start = BoomStartArray(FixedBoom1, BoomArray1, BoomOverShootFraction1);
                Vector3[] Omega1 = AngularVelocityCalculation(BoomArray1, TimeStep, FixedBoom1); //x=time, y=omega
                Vector3[] V1 = VelocityCalculation(Omega1, FixedBoom1, EndBoom1); //x=time, y=v
                Vector3 EditSubMenuAruco = receivedPos26;
            string EditSubMenuTwo = InMenuRegion(0, 100, -450, -400, EditSubMenuAruco);
            string EditSubMenuOne = InMenuRegion(0, 100, -400, -350, EditSubMenuAruco);
        //Animates if Boom1 and Piston1 Exist
         if(StartPiston1!=outofframe && FixedBoom1!=outofframe && EndBoom1!=outofframe)
            {
            //while(EditSubMenuTwo=="out of range" && EditSubMenuOne == "in range"){
            for(int i=0; i<BoomArray1.Length;i++)
            {
            //if(EditSubMenuTwo=="out of range"){
            Vector3 endpos = BoomArray1[i];
            Vector3 begpos = BoomArray1Start[i];
            float endspeed = 2f*DistanceBetweenPoints(BoomArray1[i], BoomArray1[i+1])/TimeStep;
            float begspeed = 2f*DistanceBetweenPoints(BoomArray1Start[i], BoomArray1Start[i+1])/TimeStep;
            // Velocity[0].enabled = true;
            // Velocity[0].text = V1[i][1].ToString();
            yield return StartCoroutine(DrawLinkOneLine(endpos,begpos,endspeed,begspeed));
            //}
            // else if(EditSubMenuTwo=="in range"){
            // yield break;

            // }
            }
            }

        AnimationOneStatus = false;

        }

    public IEnumerator DrawLinkOneLine(Vector3 posonlinkend, Vector3 posonlinkovershoot, float endspeed, float begspeed)
        {
        Vector3 EditSubMenuAruco = receivedPos26;
        string EditSubMenuTwo = InMenuRegion(0, 100, -450, -400, EditSubMenuAruco);
        string EditSubMenuOne = InMenuRegion(0, 100, -400, -350, EditSubMenuAruco);
        if(EditSubMenuTwo=="out of range")
        {
        while(BoomEnd[0].transform.position != posonlinkend ){
            BoomEnd[0].transform.position = Vector3.MoveTowards (BoomEnd[0].transform.position, posonlinkend, endspeed);
            Overshoot[0].transform.position = Vector3.MoveTowards(Overshoot[0].transform.position,posonlinkovershoot,begspeed);
            Vector3[] LinePosition1 = {posonlinkovershoot, posonlinkend};
            BoomLine[0].SetPositions(LinePosition1);
            yield return null;
        }
        }
        else if(EditSubMenuTwo =="in range")
        {
            SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
            SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
            yield return new WaitUntil(() => EditSubMenuOne == "in range");
        }



        }


    public IEnumerator FollowPistonOnePath()
        {
        //Time Step Generic
        float TimeStep = TheTimeStep();

        //Boom 1 Data
        float BoomOverShootFraction1=0f;
        //float PistonFraction1 = sliderValue(receivedPos9,receivedPos8);
        float PistonFraction1 = 1f;
        //Boom 1 Positions
        Vector3 FixedBoom1 = receivedPos5;
        Vector3 EndBoom1 = receivedPos6;
        Vector3 StartPiston1 = receivedPos7;
        Vector3 StartBoom1 = BoomStartFinder(FixedBoom1, EndBoom1, BoomOverShootFraction1);
        Vector3[] PistonArray1 = PistonRotationCalculation(FixedBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
        Vector3 EditSubMenuAruco = receivedPos26;
        string EditSubMenuTwo = InMenuRegion(0, 100, -450, -400, EditSubMenuAruco);
        string EditSubMenuOne = InMenuRegion(0, 100, -400, -350, EditSubMenuAruco);

        //RenderComponents();
        //while(EditSubMenuTwo == "out of range" && EditSubMenuOne =="in range"){
        if(StartPiston1!=outofframe && FixedBoom1!=outofframe && EndBoom1!=outofframe){
        for(int i=0; i<PistonArray1.Length;i++)
        {
            //if(EditSubMenuTwo=="out of range"){
            Vector3 pos = PistonArray1[i];
            float speed = 2f*DistanceBetweenPoints(PistonArray1[i], PistonArray1[i+1])/TimeStep;
            yield return StartCoroutine(DrawPistonOneLine(pos, speed));
            //} else if (EditSubMenuTwo == "in range"){
            //yield break;
            //}
            //Console.WriteLine(item)
        }
        }
        //}

        AnimationOneStatus = false;

        }

    public IEnumerator DrawPistonOneLine(Vector3 posonlink, float speed)
        {
        Vector3 EditSubMenuAruco = receivedPos26;
        string EditSubMenuTwo = InMenuRegion(0, 100, -450, -400, EditSubMenuAruco);
        string EditSubMenuOne = InMenuRegion(0, 100, -400, -350, EditSubMenuAruco);
        while(PistonOneEnd[0].transform.position != posonlink){
            if(EditSubMenuTwo=="out of range"){
            PistonOneEnd[0].transform.position = Vector3.MoveTowards (PistonOneEnd[0].transform.position, posonlink, speed);
            Vector3[] LinePosition1 = {squareArray[2].transform.position, posonlink};
            PistonMovingLine[0].SetPositions(LinePosition1);
            yield return null;
            } else if(EditSubMenuTwo == "in range"){
            yield return new WaitUntil(() => EditSubMenuOne == "in range");
            }
        }
        }
    public IEnumerator FollowLinkTwoPath()
        {
        //Time Step Generic
        float TimeStep = TheTimeStep();

        //Boom 1 Data
        float BoomOverShootFraction1=0f;
        //float PistonFraction1 = sliderValue(receivedPos9,receivedPos8);
        float PistonFraction1 = 1f;

        //Boom 2 Data
        float BoomOverShootFraction2 = 0f;
        float PistonFraction2 = 0.7f;
        float JointFraction2 = 0f;

        //Boom 1 Positions
        Vector3 FixedBoom1 = receivedPos5;
        Vector3 EndBoom1 = receivedPos6;
        Vector3 StartPiston1 = receivedPos7;
        //Boom 1 Calcs
	    Vector3[] BoomArray1 = BoomRotationCalculation(FixedBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
        //Boom 2 Positions
        Vector3 EndBoom2 = receivedPos10; //???
        Vector3 StartPiston2 = receivedPos11; //???
        //Boom 2 Calcs
        Vector3 FixedBoom2 = BoomFixedFinder(FixedBoom1, EndBoom1, JointFraction2);
        float[] AngleChangeBoom1 = RotationFromStartCalculation(BoomArray1, FixedBoom1);
        Vector3[] BoomArray2 = BoomRotationCalculation(FixedBoom2, EndBoom2, StartPiston2, BoomOverShootFraction2, PistonFraction2,TimeStep);
        //Boom2 Translated Arrays - Vector3[Position Through Boom 1 Movement, Position Through Boom 2 Movement]
        Vector3[,] BoomArray2Array = ArrayRelativePosition(FixedBoom1, BoomArray2, AngleChangeBoom1);

        //Stop Animation One Range
        //Vector3 StopAnimationAruco = receivedPos25;
        //int xminAnimOne = 143; int xmaxAnimOne = 243; int yminAnimOne = -500; int ymaxAnimOne = -590;
        //string InStopAnimationOne = InMenuRegion(xminAnimOne,xmaxAnimOne,yminAnimOne,ymaxAnimOne,StopAnimationAruco);
        //RenderComponents();
        if(StartPiston1!=outofframe && FixedBoom1!=outofframe && EndBoom1!=outofframe && EndBoom2!=outofframe && StartPiston2!=outofframe){
        //while(InStopAnimationOne == "in range"){
        for(int i=0; i<BoomArray2Array.Length;i++)
        {
            //fix this length thing
            Vector3 pos = BoomArray2Array[i,0];
            float speed = 10f*DistanceBetweenPoints(BoomArray2Array[i,0], BoomArray2Array[i+1,0])/TimeStep;
            yield return StartCoroutine(DrawLinkTwoLine(pos, speed));
        }
        //}
        }
        AnimationTwoStatus = false;

        }

    public IEnumerator DrawLinkTwoLine(Vector3 posonlink, float speed)
        {
        while(BoomEnd[1].transform.position != posonlink){
            BoomEnd[1].transform.position = Vector3.MoveTowards (BoomEnd[1].transform.position, posonlink, speed);
            Vector3[] LinePosition1 = {squareArray[1].transform.position, posonlink};
            BoomLine[1].SetPositions(LinePosition1);
            yield return null;
        }
        }

    public Vector3 Xfloattov3(float Xmaxscaled, Vector3 zerozero, float offset)
    {
        Vector3 vec= new Vector3 (Xmaxscaled+offset, zerozero.y, 0f);
        return vec;
    }

    public float FindMinX(Vector3[] array)
    {
        float MinX = array.Select(v=>v.x).Min();
        return MinX;
    }

    public float FindMinY(Vector3[] array)
    {
        float MinY = array.Select(v=>v.y).Min();
        return MinY;
    }

    public float FindMaxX(Vector3[] array)
    {
        float MaxX = array.Select(v=>v.x).Max();
        return MaxX;
    }

    public float FindMaxY(Vector3[] array)
    {
        float MaxY = array.Select(v=>v.y).Max();
        return MaxY;
    }

    public Vector3[] CombineVector3Arrays (Vector3[] array1, Vector3[] array2)
    {
        var array3 = new Vector3[array1.Count() + array2.Count()];
        System.Array.Copy (array1, array3, array1.Count());
        System.Array.Copy (array2, 0, array3, array1.Count(), array2.Count());
        return array3;
    }

    public Vector3[] NewContract(float XmaxV1, Vector3[] array)
    {

        int arraylength = array.Count();
        Vector3[] arrayout = new Vector3[arraylength];
        for (int i=0; i<arraylength; i++)
        {
            float x = array[i][0] + XmaxV1;
            float y = array[i][1];
            arrayout[i] = new Vector3(x,y,0f);
        }

        return arrayout;
    }
    public Vector3[] ScaleData(Vector3[] array, float Xscale, float Yscale)
    {

        int arraylength = array.Count();
        Vector3[] arrayout = new Vector3[arraylength];
        for (int i=0; i<arraylength; i++)
        {
            float x = array[i][0]*Xscale;
            float y = array[i][1]*Yscale;
            arrayout[i] = new Vector3(x,y,0f);
        }

        return arrayout;
    }

    public Vector3[] MoveData(Vector3[] array, Vector3 zerozero)
    {

        int arraylength = array.Count();
        Vector3[] arrayout = new Vector3[arraylength];
        for (int i=0; i<arraylength; i++)
        {
            float x = array[i][0] + zerozero[0];
            float y = array[i][1] + zerozero[1];
            arrayout[i] = new Vector3(x,y,0f);
        }

        return arrayout;
    }



}
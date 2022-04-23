

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

[RequireComponent(typeof(LineRenderer))]

public class portlistener : MonoBehaviour
{
    public GameObject[] squareArray;
    public GameObject[] menuArray;
    public GameObject menuGameMode;
    public GameObject[] PistonOneEnd;
    public GameObject[] BoomEnd;
    public GameObject[] Overshoot;
    public GameObject[] SubMenuArray;
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
    //public GameObject[] StopAnimation;
    public LineRenderer PistonOneCurve;
    public TextMeshProUGUI PistonFraction1Text;
    public TextMeshProUGUI PistonExtension1Text;
    public TextMeshProUGUI GameModeErrorMessage;
    public TextMeshProUGUI GameModeObjectRetrieveMessage;
    public TextMeshProUGUI[] Velocity;
    public TextMeshProUGUI[] MenuText;
    public TextMeshProUGUI[] SliderPositionText;
    public TextMeshProUGUI[] SubMenuText;
    //public TextMeshProUGUI[] StopAnimationText;
    //public GameObject[] LinkArray;
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
    public Vector3 receivedPos25 = new Vector3(-100,0,0);
    public Vector3 receivedPos26 = new Vector3(-100,0,0);
    public Vector3 receivedPos27 = new Vector3(-100,0,0);
    Vector3 size = Vector3.zero;
    Vector3 zeros = Vector3.zero;
    Vector3 outofframe = new Vector3(-100,0,0);
    public Vector3 MenuData = new Vector3(10,0,0);
    public bool AnimationOneStatus = false;
    public bool AnimationTwoStatus = false;
    public bool GameStatus = true;
    public bool running; //nothing

    public void Start() //private
    {

        ThreadStart ts = new ThreadStart(GetInfo);
        mThread = new Thread(ts);
        mThread.Start();

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
            receivedPos25 = StringToVector3(dataReceived,"25");
            receivedPos26 = StringToVector3(dataReceived,"26");
            receivedPos27 = StringToVector3(dataReceived,"27");
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
        int index = Array.IndexOf(sArray, "24");
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
        float dt = 5f;
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
        float slider_default_value = 0.7f;
        if(Handle_Position==outofframe||Slider_Position==outofframe)
        {
            return slider_default_value;
        }
        else if(0<=slider_current_value&&slider_current_value<=1)
        {
            return slider_current_value;
        }
        else if (slider_current_value<=0)
        {
            return 0;
        }
        else
        {
            return 1;
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
            if (o1 == 0 || o1 == 360  || o1 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] = (float)Math.Acos(z[i]);
            }
         }
            if (o2 == 0 || o2 == 360  || o2 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  -(float)Math.Acos(z[i]);
            }
         }
            if (o3 == 0 || o3 == 360  || o3 == -360)
         {
            for (int i = 0; i<ArrayLength; i++)
            {
                Theta[i] =  (float)Math.Acos(z[i]) ;
            }
            Alpha = -Alpha;
         }
            if (o4 == 0 || o4 == 360  || o4 == -360)
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
    // public Vector3[] BoomStartArray(Vector3 BoomFixed, Vector3[] BoomArray, float BoomOvershoot)
    // {
    //     int ArrayLength = BoomArray.Length;
    //     float MultiplyingFactor = 1f + BoomOvershoot;
    //     Vector3[] NewBoomStartArray = new Vector3[ArrayLength];
    //     for(int i=0; i < ArrayLength; i++)
    //     {
    //         float x = BoomArray[i][0] - ((BoomFixed[0] - BoomArray[i][0])*MultiplyingFactor);
    //         float y = BoomArray[i][1] - ((BoomFixed[1] - BoomArray[i][1])*MultiplyingFactor);
    //         NewBoomStartArray[i] = new Vector3(x, y, 0f);
    //     }
    //     return NewBoomStartArray;
    // }

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
    public float[] RotationFromStartCalculation(Vector3[] BoomArray)
        {
        int ArrayLength = BoomArray.Length;
        float[] AngleChange = new float[ArrayLength];
        float StartAngle = (float)Math.Atan2(BoomArray[0][1], BoomArray[0][0]);
        for (int i = 0; i<ArrayLength; i++)
        {
        float PointAngle = (float)Math.Atan2(BoomArray[i][1], BoomArray[i][0]);
        AngleChange[i] = PointAngle - StartAngle;
        }
        return AngleChange;
        }
    public float BoomRangeCalculation(Vector3[] BoomArray)
        {
        int End = BoomArray.Length - 1;
        float StartAngle = (float)Math.Atan2(BoomArray[0][1], BoomArray[0][0]);
        float EndAngle = (float)Math.Atan2(BoomArray[End][1], BoomArray[End][0]);
        float BoomRange = (EndAngle - StartAngle)*180f/((float)Math.PI);
        return BoomRange;
        }
    public Vector3[] AngularVelocityCalculation(Vector3[] BoomArray, float TimeStep)
        {
        int ArrayLength = BoomArray.Length;
        Vector3[] AngularVelocity = new Vector3[ArrayLength];
        AngularVelocity[0] = new Vector3(0f, 0f, 0f) ;
        for ( int i = 1; i < ArrayLength-1; i++)
	    {
        float AngleChange = (float)Math.Atan2(BoomArray[i][1], BoomArray[i][0]) - (float)Math.Atan2(BoomArray[(i-1)][1], BoomArray[(i-1)][0]);
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
            for(int j=0; j<NextArrayLength; i++)
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

public async void Update()
    {
        //Time Step Generic
        float TimeStep = TheTimeStep();

        //Boom 1 Data
        float BoomOverShootFraction1 = 0f;
        Vector3 SliderPosition = receivedPos8; //Moving
        Vector3 HandlePosition = receivedPos9; //Still
        float PistonFraction1 = 1f;
        //float PistonExtension1 = sliderValue(HandlePosition,SliderPosition);

        //Slider Assignment
        Vector3 EditSubMenuAruco = receivedPos26;
        Vector3 TypeSelectionAruco = receivedPos27;
        string EditSubMenuOne = InMenuRegion(0, 100, -400, -350, EditSubMenuAruco);
        string EditSubMenuTwo = InMenuRegion(0, 100, -450, -400, EditSubMenuAruco);
        string TypeSelectionOne = InMenuRegion(850, 950, -120, -70, TypeSelectionAruco);
        string TypeSelectionTwo = InMenuRegion(850, 950, -170, -120, TypeSelectionAruco);
        string TypeSelectionThree = InMenuRegion(850, 950, -220, -170, TypeSelectionAruco);
        string TypeSelectionFour = InMenuRegion(850, 950, -270, -220, TypeSelectionAruco);
        string TypeSelectionWhole = InMenuRegion(850, 950, -270, -70, TypeSelectionAruco);

        if(MenuData[0]==0){
            //SubMenu Text
            SubMenuText[0].text = "Piston";
            SubMenuText[1].text = "Link";
            SubMenuArray[2].GetComponent<Renderer>().enabled=false;
            SubMenuText[2].enabled = false;
            if(EditSubMenuOne == "in range")//Piston Selected
            {
                //Highlight Piston To Blue
                SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
                SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                SliderPositionText[0].enabled = true;
                SliderPositionText[1].enabled = true;
                MenuText[0].text = "Piston Selection";
                //Piston Fraction or Boom Overshoot
                if(TypeSelectionOne == "in range") //Piston One
                {
                    //Highlights Piston One Selection
                    SelectorHighlighter.transform.position = new Vector3(890,-95,0);
                    //If LHS
                    if(SliderPosition[0]<495 && SliderPosition!=outofframe && HandlePosition!=outofframe)
                    {
                    //PistonFraction1 = sliderValue(HandlePosition,SliderPosition);
                    PistonFraction1Text.text = PistonFraction1.ToString();
                    }
                    //If RHS
                    else if(SliderPosition[0]>=495 && SliderPosition!=outofframe && HandlePosition!=outofframe)
                    {
                    //PistonExtension1 = sliderValue(HandlePosition,SliderPosition);
                   // PistonExtension1Text.text =PistonExtension1.ToString();
                    }
                    //If Slider not in frame/undetected by webcam
                    else if(SliderPosition==outofframe || HandlePosition==outofframe)
                    {
                    PistonFraction1Text.text = "slider not detected";
                    PistonExtension1Text.text = "slider not detected";
                    }
                }
                if(TypeSelectionTwo == "in range")//Piston Two
                {
                    SelectorHighlighter.transform.position = new Vector3(890,-145,0); //Highlights Piston Two Selection
                }
                if(TypeSelectionThree == "in range")
                {
                    SelectorHighlighter.transform.position = new Vector3(890,-195,0); //Highlights Piston Three
                }
                if(TypeSelectionFour == "in range")
                {
                    SelectorHighlighter.transform.position = new Vector3(890,-245,0); // Highlights Piston Four
                }
                if(TypeSelectionWhole == "in range")
                {
                    //If RHS Highlights
                    if(SliderPosition[0]<495 && SliderPosition!=outofframe && HandlePosition!=outofframe)
                    {
                        SliderPositionHighlighter.SetActive(true);
                        SliderPositionHighlighter.transform.position = new Vector3(275,-585,0);
                    }
                    //If LHS Highlights
                    else if(SliderPosition[0]>=495 && SliderPosition!=outofframe && HandlePosition!=outofframe)
                    {
                        SliderPositionHighlighter.SetActive(true);
                        SliderPositionHighlighter.transform.position = new Vector3(675,-585,0);
                    }
                    else
                    {
                        SliderPositionHighlighter.SetActive(false);
                    }
                }
            }
            else if(EditSubMenuTwo == "in range") //Boom One
            {
                //Highlight Boom To Blue
                SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
                MenuText[0].text = "Link Selection";
                SliderPositionText[0].enabled = false;
                SliderPositionText[1].enabled = false;
                if(TypeSelectionTwo == "in range")//Link Two
                {
                    SelectorHighlighter.transform.position = new Vector3(890,-145,0); //Highlights Piston Two Selection
                }
                if(TypeSelectionThree == "in range")//Link Three
                {
                    SelectorHighlighter.transform.position = new Vector3(890,-195,0); //Highlights Piston Three
                }
                if(TypeSelectionFour == "in range")//Link Four
                {
                    SelectorHighlighter.transform.position = new Vector3(890,-245,0); // Highlights Piston Four
                }

            }
            else
            {
                SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
                SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
                MenuText[0].text = "Bitch pls pick Piston or Link ";
            }
        }





        //Boom 1 Positions
        Vector3 FixedBoom1 = receivedPos5;
        Vector3 EndBoom1 = receivedPos6;
        Vector3 StartPiston1 = receivedPos7;

        Vector3 StartBoom1 = BoomStartFinder(FixedBoom1, EndBoom1, BoomOverShootFraction1);
        Vector3 EndPiston1 = PistonEnd(FixedBoom1, EndBoom1, PistonFraction1);

        //Boom 1 Calcs
	    Vector3[] BoomArray1 = BoomRotationCalculation(FixedBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
        Vector3[] BoomArray1Start = BoomStartArray(FixedBoom1, BoomArray1, BoomOverShootFraction1);
        Vector3[] PistonArray1 = PistonRotationCalculation(StartBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
        float[] AngleChangeBoom1 = RotationFromStartCalculation(BoomArray1);
        float TotalBoomRange1 = BoomRangeCalculation(BoomArray1);
        Vector3[] Omega1 = AngularVelocityCalculation(BoomArray1, TimeStep); //x=time, y=omega
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

        //Stop Animation One Range
        //Vector3 StopAnimationAruco = receivedPos25;
        //int xminAnimOne = 143; int xmaxAnimOne = 243; int yminAnimOne = -590; int ymaxAnimOne = -500;
        //string InStopAnimationOne = InMenuRegion(xminAnimOne,xmaxAnimOne,yminAnimOne,ymaxAnimOne,StopAnimationAruco);

        //Boom 2
        //BData
        float BoomOverShootFraction2 = 0f;
        float PistonFraction2 = 0.7f;
        float JointFraction2 = 0f;

        // //Positions
        Vector3 EndBoom2 = receivedPos10; //???
        Vector3 StartPiston2 = receivedPos11; //???


        Vector3 FixedBoom2 = BoomFixedFinder(FixedBoom1, EndBoom1, JointFraction2);
        Vector3 StartBoom2 = BoomStartFinder(FixedBoom2, EndBoom2, BoomOverShootFraction2);
        Vector3 EndPiston2 = PistonEnd(EndBoom1, EndBoom2, PistonFraction2);

        //Arrays of Translated Positions relative to boom1 rotaions
        Vector3[] EndBoom2Array = RelativePosition(FixedBoom1, EndBoom2, AngleChangeBoom1);
        Vector3[] StartPiston2Array = RelativePosition(FixedBoom1, StartPiston2, AngleChangeBoom1);
        Vector3[] FixedBoom2Array = RelativePosition(FixedBoom1, EndBoom2, AngleChangeBoom1);
        Vector3[] StartBoom2Array = RelativePosition(FixedBoom1, StartBoom2, AngleChangeBoom1);
        Vector3[] EndPiston2Array = RelativePosition(FixedBoom1, EndPiston2, AngleChangeBoom1);

        //Boom2 Calcs
        Vector3[] BoomArray2 = BoomRotationCalculation(FixedBoom2, EndBoom2, StartPiston2, BoomOverShootFraction2, PistonFraction2,TimeStep);
        Vector3[] BoomArray2Start = BoomStartArray(FixedBoom2, BoomArray2, BoomOverShootFraction2);
        Vector3[] PistonArray2 = PistonRotationCalculation(StartBoom2, EndBoom2, StartPiston2, BoomOverShootFraction2, PistonFraction2,TimeStep);
        float[] AngleChangeBoom2 = RotationFromStartCalculation(BoomArray2);
        float TotalBoomRange2 = BoomRangeCalculation(BoomArray2);
        // Vector3[] Omega2 = AngularVelocityCalculation(BoomArray2, TimeStep);
        // Vector3[] Omega2Contract = VelocityContracting(Omega2);
        // Vector3[] V2 = VelocityCalculation(Omega2, FixedBoom2, EndBoom2);
        // Vector3[] V2Contract = VelocityContracting(V2);

        //Boom2 Translated Arrays - Vector3[Position Through Boom 1 Movement, Position Through Boom 2 Movement]
        Vector3[,] BoomArray2Array = ArrayRelativePosition(FixedBoom1, BoomArray2, AngleChangeBoom1);
        Vector3[,] BoomArray2StartArray = ArrayRelativePosition(FixedBoom1, BoomArray2Start, AngleChangeBoom1);
        Vector3[,] PistonArray2Array = ArrayRelativePosition(FixedBoom1, PistonArray2, AngleChangeBoom1);

        // //Max Reach of Both Booms
        // Vector3[] MaxReachPositions = MaxRangePosition(FixedBoom1, BoomArray1, BoomArray2Array);
        // //Velocity at Combined
        // float[,] EndVelocityBoom1ExtendBoom2Extend = TwoMovingBoomsVelocity(Omega1, FixedBoom1, FixedBoom2Array, V2, BoomArray2Array); //Boom1 Extend Boom2 Extending
        // float[,] EndVelocityBoom1ExtendBoom2Contract = TwoMovingBoomsVelocity(Omega1, FixedBoom1, FixedBoom2Array, V2Contract, BoomArray2Array); //Boom1 Extend Boom2 Contract
        // float[,] EndVelocityBoom1ContractBoom2Extend = TwoMovingBoomsVelocity(Omega1Contract, FixedBoom1, FixedBoom2Array, V2, BoomArray2Array); //Boom1 Contract Boom2 Extending
        // float[,] EndVelocityBoom1ContractBoom2Contract = TwoMovingBoomsVelocity(Omega1Contract, FixedBoom1, FixedBoom2Array, V2Contract, BoomArray2Array); //Boom1 Contract Boom2 Contract

        //Boom 2 Array Length
        int ArrayLength2 = BoomArray2Array.GetLength(0);
        BoomCurve[1].positionCount = ArrayLength2;
        //print(ArrayLength2);

        if(MenuData[0]==0)
        {
        //edit
        AnimationOneStatus = false;
        Velocity[0].enabled = false;
        //StopAnimation[0].GetComponent<Renderer>().enabled=false;
        //StopAnimationText[0].enabled = false;
        BoomCurve[0].GetComponent<Renderer>().enabled=false;
        BoomCurve[1].GetComponent<Renderer>().enabled=false;
        menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
        menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        GameModeObjectRetrieveMessage.enabled=false;

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

        void IfExistBoomLine(Vector3 StartBoomPos, Vector3 EndBoomPos, int i )
        {
            if(StartBoomPos!=outofframe&&EndBoomPos!=outofframe)
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

        IfExistBoomLine(StartBoom1,EndBoom1,0);
        IfExistBoomLine(EndBoom1,EndBoom2,1);

        void IfExistPistonMovingLine(Vector3 StartPistonPos, Vector3 EndPistonPos, Vector3 StartBoomPos, Vector3 EndBoomPos, int i)
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
        IfExistPistonMovingLine(StartPiston1,EndPiston1,FixedBoom1,EndBoom1,0);
        IfExistPistonMovingLine(StartPiston2,EndPiston2,EndBoom1,EndBoom2,1);

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


        //Game Mode - show excavator
        if(InGameMode == "in range")
        {
            ExcavatorBase.GetComponent<Renderer>().enabled=true;
            ObjectToRetrieve.GetComponent<Renderer>().enabled=true;
            ExcavatorBase.transform.position = ExcavatorPos;
            ObjectToRetrieve.transform.position = ObjectToRetrievePos;
            ExcavatorBaseCircle.GetComponent<Renderer>().enabled=true;
            ExcavatorBaseCircle.transform.position = ExcavatorPos;
            menuGameMode.GetComponent<SpriteRenderer>().material.color = Color.blue;
            //Fixed Joint in Excavator Circle Error Message
            if(InExcavatorCircle == "out of range")
            {
                GameModeErrorMessage.enabled = true;
                GameModeErrorMessage.text = "Place Fixed Joint on Excavator";
                GameStatus = false;
            }
            else
            {
                GameModeErrorMessage.enabled = false;
                GameStatus = true;
            }
        }
        else if (InGameMode == "out of range")
        {
            GameStatus = true;
            GameModeErrorMessage.enabled = false;
            ExcavatorBase.GetComponent<Renderer>().enabled=false;
            ExcavatorBaseCircle.GetComponent<Renderer>().enabled=false;
            ObjectToRetrieve.GetComponent<Renderer>().enabled=false;
            menuGameMode.GetComponent<SpriteRenderer>().material.color = Color.white;
        }



        }
        if(MenuData[0]==1 && AnimationOneStatus==false && GameStatus == true)
        {
        //Menu

        if(EditSubMenuOne == "in range" && AnimationOneStatus==false)
        {
            AnimationOneStatus = true;
            SubMenuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
            SubMenuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
            StartCoroutine(FollowPistonOnePath());
            StartCoroutine(FollowLinkOnePath());

        }


        GameStatus = true;
        //SubMenu Text and Box
        SubMenuText[0].text = "Play";
        SubMenuText[1].text = "Stop";
        SubMenuText[2].text = "Reset";
        SubMenuArray[2].GetComponent<Renderer>().enabled=true;

        //StopAnimationText[0].enabled = true;
        //StopAnimation[0].GetComponent<Renderer>().enabled=true;
        BoomCurve[0].GetComponent<Renderer>().enabled=true;
        menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
        menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;

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

        void IfExistBoomLine(Vector3 StartBoomPos, Vector3 EndBoomPos, int i )
        {
            if(StartBoomPos!=outofframe&&EndBoomPos!=outofframe)
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

        IfExistBoomLine(StartBoom1,EndBoom1,0);
        IfExistBoomLine(EndBoom1,EndBoom2,1);

        void IfExistPistonMovingLine(Vector3 StartPistonPos, Vector3 EndPistonPos, Vector3 StartBoomPos, Vector3 EndBoomPos, int i)
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
        IfExistPistonMovingLine(StartPiston1,EndPiston1,FixedBoom1,EndBoom1,0);
        IfExistPistonMovingLine(StartPiston2,EndPiston2,EndBoom1,EndBoom2,1);

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




        if(!StartPiston1.Equals(outofframe))
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

         if(!StartPiston1.Equals(outofframe)&&!StartPiston2.Equals(outofframe))
        {
        for(int i=0; i<ArrayLength2; i++)
        {
            BoomCurve[1].GetComponent<Renderer>().enabled = true;
            BoomCurve[1].SetPosition(i,BoomArray2Array[i,0]);

        }
        }
        else
        {
            BoomCurve[1].GetComponent<Renderer>().enabled = false;
        }








        //In Game Mode if retrieves object
        if(InObjectPosBoomArray1 == "in range")
        {
            GameModeObjectRetrieveMessage.text = "Success!";
        }
        else if(InObjectPosBoomArray1 == "out of range")
        {
            GameModeObjectRetrieveMessage.text = "u suck shit try again";
        }
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
        if (MenuData[0]==2)
        {
        //data
        menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.blue;
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
        Vector3[] Omega1 = AngularVelocityCalculation(BoomArray1, TimeStep); //x=time, y=omega
        Vector3[] V1 = VelocityCalculation(Omega1, FixedBoom1, EndBoom1); //x=time, y=v
        Vector3 EditSubMenuAruco = receivedPos26;
        string EditSubMenuTwo = InMenuRegion(0, 100, -450, -400, EditSubMenuAruco);
        string EditSubMenuOne = InMenuRegion(0, 100, -400, -350, EditSubMenuAruco);

        if(StartPiston1!=outofframe && FixedBoom1!=outofframe && EndBoom1!=outofframe){
        //while(EditSubMenuTwo=="out of range" && EditSubMenuOne == "in range"){
        for(int i=0; i<BoomArray1.Length;i++)
        {
            //if(EditSubMenuTwo=="out of range"){
            Vector3 endpos = BoomArray1[i];
            Vector3 begpos = BoomArray1Start[i];
            float endspeed = 2f*DistanceBetweenPoints(BoomArray1[i], BoomArray1[i+1])/TimeStep;
            float begspeed = 2f*DistanceBetweenPoints(BoomArray1Start[i], BoomArray1Start[i+1])/TimeStep;
            Velocity[0].enabled = true;
            Velocity[0].text = V1[i][1].ToString();
            yield return StartCoroutine(DrawLinkOneLine(endpos,begpos,endspeed,begspeed));
            //}
            // else if(EditSubMenuTwo=="in range"){
            // yield break;

            // }
        }
        }
        //}
        AnimationOneStatus = false;

    }

    public IEnumerator DrawLinkOneLine(Vector3 posonlinkend, Vector3 posonlinkovershoot, float endspeed, float begspeed)
    {
        Vector3 EditSubMenuAruco = receivedPos26;
        string EditSubMenuTwo = InMenuRegion(0, 100, -450, -400, EditSubMenuAruco);
        string EditSubMenuOne = InMenuRegion(0, 100, -400, -350, EditSubMenuAruco);
        while(BoomEnd[0].transform.position != posonlinkend ){
            BoomEnd[0].transform.position = Vector3.MoveTowards (BoomEnd[0].transform.position, posonlinkend, endspeed);
            Overshoot[0].transform.position = Vector3.MoveTowards(Overshoot[0].transform.position,posonlinkovershoot,begspeed);
            Vector3[] LinePosition1 = {posonlinkovershoot, posonlinkend};
            BoomLine[0].SetPositions(LinePosition1);
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
        Vector3[] PistonArray1 = PistonRotationCalculation(StartBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
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
           // if(EditSubMenuTwo=="out of range"){
            PistonOneEnd[0].transform.position = Vector3.MoveTowards (PistonOneEnd[0].transform.position, posonlink, speed);
            Vector3[] LinePosition1 = {squareArray[2].transform.position, posonlink};
            PistonMovingLine[0].SetPositions(LinePosition1);
            yield return new WaitUntil(() => EditSubMenuOne == "in range");
            // yield return null;
            // } else if(EditSubMenuTwo == "in range"){
            // yield break;
            // }
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
        float[] AngleChangeBoom1 = RotationFromStartCalculation(BoomArray1);
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



}

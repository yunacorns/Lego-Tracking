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
//using System.Numerics;

[RequireComponent(typeof(LineRenderer))]


public class portlistener : MonoBehaviour
{
    public GameObject[] squareArray;
    public GameObject[] menuArray;
    public GameObject PistonOneEnd;
    public GameObject BoomOneEnd;
    public GameObject boomoneanimation;
    public LineRenderer Line;
    public LineRenderer PistonLine;
    public LineRenderer BoomOneCurve;
    public TextMeshProUGUI PistonOneFraction;



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
    Vector3 size = Vector3.zero;
    Vector3 zeros = Vector3.zero;
    Vector3 outofframe = new Vector3(-100,0,0);
    public Vector3 MenuData = new Vector3(10,0,0);
    public bool status = false;
    //Vector3 LinkData = new Vector3(10,0,0);



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

    // // George's Calculation here
    // public float TimeStep()
    // {

    // }
    public float DistanceBetweenPoints(Vector3 v1, Vector3 v2)
        {
            float xDiff = (float)Math.Pow((v2[0] - v1[0]),2.0);
            float yDiff = (float)Math.Pow((v2[1] - v1[1]),2.0);

            return (float)Math.Sqrt(xDiff + yDiff);
        }
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
    public Vector3[] BoomStartArray(Vector3 BoomFixed, Vector3[] BoomArray, float BoomOvershoot)
    {
        int ArrayLength = BoomArray.Length;
        float MultiplyingFactor = 1f + BoomOvershoot;
        Vector3[] BoomStartArray = new Vector3[ArrayLength];
        for(int i=0; i < ArrayLength; i++)
        {
            Vector3 Translation = BoomFixed - BoomArray[i];
            Vector3 Translation = MultiplyingFactor*Translation;
            BoomStartArray[i] = BoomArray[i] + Translation;
        }
        return BoomStartArray;
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
        for ( int i = 1; i < ArrayLength; i++)
	    {
        float AngleChange = (float)Math.Atan2(BoomArray[i][1], BoomArray[i][0]) - (float)Math.Atan2(BoomArray[(i-1)][1], BoomArray[(i-1)][0]);
        float x =  i*TimeStep;
        float y =  AngleChange/TimeStep;
        AngularVelocity[i] = new Vector3(x, y, 0f);
        }
        for ( int i = 1; i <ArrayLength; i++)
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
                NewPosition[j,i] = new Vector3(x, y, 0f);
            }
        }
    }
    public Vector3[] MaxRangePosition(Vector3 OriginBoomFixed, Vector3[,] NextBoomMovement)
    {
        int OriginArrayLength = NextBoomMovement.GetLength(0);
        int NextArrayLength = NextBoomMovement.GetLength(1);
        Vector3[] MaxRangePositionArray = new Vector3[OriginArrayLength];
        Vector3 MaxRange = new Vector3(0f, 0f, 0f);
        for(int i=0; i<OriginArrayLength; i++)
        {
            float MaxDistance = 0f;
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
            MaxRangePositionArray = new Vector3(x, y, 0f);
        }

    }
    public float[,] TwoMovingBoomsVelocity(Vector3[] FirstBoomAngularVelocity, Vector3 FirstFixedPoint, Vector3[] SecondBoomFixedArray, Vector3[] SecondBoomVelocity, Vector3[,] SecondBoomArrays)
    {
        int FirstArrayLength = FirstBoomArray.Length;
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
    //Ben slider

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
    public async void Update()
    {
        //Time Step Generic
        float TimeStep = 0.5f;

        //Boom 1 Data
        float BoomOverShootFraction1 = 0f;
        float PistonFraction1 = sliderValue(receivedPos9, receivedPos8);
        if(receivedPos8!=outofframe&&receivedPos9!=outofframe)
        {
            PistonOneFraction.text = PistonFraction1.ToString();
        }
        else
        {
            PistonOneFraction.text = "slider not detected";
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

        float ArrayLength1 = BoomArray1.Length;

        BoomOneCurve.positionCount = BoomArray1.Length;

        //Boom 2
        //BData
        float BoomOverShootFraction2 = 0.3f;
        float PistonFraction2 = 0f;
        float JointFraction2 = 1f;

        //Positions
        Vector3 EndBoom2 = receivedPos10; //???
        Vector3 StartPiston2 = receivedPos11; //???

        Vector3 FixedBoom2 = BoomFixedFinder(FixedBoom1, EndBoom1, JointFraction2);
        Vector3 StartBoom2 = BoomStartFinder(FixedBoom2, EndBoom2, BoomOverShootFraction2);
        Vector3 EndPiston2 = PistonEnd(FixedBoom2, EndBoom2, PistonFraction2);

        //Arrays of Translated Positions relative to boom1 rotaions
        Vector3[] EndBoom2Array = RelativePosition(FixedBoom1, EndBoom2, AngleChangeBoom1);
        Vector3[] StartPiston2Array = RelativePosition(FixedBoom1, StartPiston2, AngleChangeBoom1);
        Vector3[] FixedBoom2Array = RelativePosition(FixedBoom1, EndBoom2, AngleChangeBoom1);
        Vector3[] StartBoom2Array = RelativePosition(FixedBoom1, StartBoom2, AngleChangeBoom1);
        Vector3[] EndPiston2Array = RelativePosition(FixedBoom1, PistonBoom2, AngleChangeBoom1);

        //Boom2 Calcs
        Vector3[] BoomArray2 = BoomRotationCalculation(FixedBoom2, EndBoom2, StartPiston2, BoomOverShootFraction2, PistonFraction2,TimeStep);
        Vector3[] BoomArray2Start = BoomStartArray(FixedBoom2, BoomArray2, BoomOverShootFraction2);
        Vector3[] PistonArray2 = PistonRotationCalculation(StartBoom2, EndBoom2, StartPiston2, BoomOverShootFraction2, PistonFraction2,TimeStep);
        float[] AngleChangeBoom2 = RotationFromStartCalculation(BoomArray2);
        float TotalBoomRange2 = BoomRangeCalculation(BoomArray2);
        Vector3[] Omega2 = AngularVelocityCalculation(BoomArray2, TimeStep);
        Vector3[] Omega2Contract = VelocityContracting(Omega2);
        Vector3[] V2 = VelocityCalculation(Omega2, FixedBoom2, EndBoom2);
        Vector3[] V2Contract = VelocityContracting(V2);

        //Boom2 Translated Arrays - Vector3[Position Through Boom 1 Movement, Position Through Boom 2 Movement]
        Vector3[,] BoomArray2Array = ArrayRelativePosition(FixedBoom1, BoomArray2, AngleChangeBoom1);
        Vector3[,] BoomArray2StartArray = ArrayRelativePosition(FixedBoom1, BoomArray2Start, AngleChangeBoom1);
        Vector3[,] PistonArray2Array = ArrayRelativePosition(FixedBoom1, PistonArray2, AngleChangeBoom1);

        //Max Reach of Both Booms
        Vector3[] MaxReachPositions = MaxRangePosition(FixedBoom1, BoomArray2Array);
        //Velocity at Combined
        float[,] EndVelocityBoom1ExtendBoom2Extend = TwoMovingBoomsVelocity(Omega1, FixedBoom1, FixedBoom2Array, V2, BoomArray2Array); //Boom1 Extend Boom2 Extending
        float[,] EndVelocityBoom1ExtendBoom2Contract = TwoMovingBoomsVelocity(Omega1, FixedBoom1, FixedBoom2Array, V2Contract, BoomArray2Array); //Boom1 Extend Boom2 Contract
        float[,] EndVelocityBoom1ContractBoom2Extend = TwoMovingBoomsVelocity(Omega1Contract, FixedBoom1, FixedBoom2Array, V2, BoomArray2Array); //Boom1 Contract Boom2 Extending
        float[,] EndVelocityBoom1ContractBoom2Contract = TwoMovingBoomsVelocity(Omega1Contract, FixedBoom1, FixedBoom2Array, V2Contract, BoomArray2Array); //Boom1 Contract Boom2 Contract


        if(MenuData[0]==0){
        //edit
        BoomOneCurve.GetComponent<Renderer>().enabled=false;
        menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
        menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        // squareArray[0].transform.position = receivedPos5;
        // squareArray[1].transform.position = receivedPos6;
        // squareArray[2].transform.position = receivedPos7;
        if(FixedBoom1!=outofframe){
            squareArray[0].GetComponent<Renderer>().enabled = true;
            squareArray[0].transform.position = FixedBoom1;
        }else{
            squareArray[0].GetComponent<Renderer>().enabled = false;
        }
        if(EndBoom1!=outofframe){
            squareArray[1].GetComponent<Renderer>().enabled = true;
            squareArray[1].transform.position = EndBoom1;
            BoomOneEnd.transform.position = EndBoom1;
        }else{
            squareArray[1].GetComponent<Renderer>().enabled = false;
        }
        if(StartPiston1!=outofframe){
            squareArray[2].GetComponent<Renderer>().enabled = true;
            squareArray[2].transform.position = StartPiston1;
        }else{
            squareArray[2].GetComponent<Renderer>().enabled = false;
        }
        if(EndBoom2!=outofframe){
            squareArray[3].GetComponent<Renderer>().enabled = true;
            squareArray[3].transform.position = StartPiston1;
        }else{
            squareArray[3].GetComponent<Renderer>().enabled = false;
        }
        if(StartPiston2!=outofframe){
            squareArray[4].GetComponent<Renderer>().enabled = true;
            squareArray[4].transform.position = StartPiston1;
        }else{
            squareArray[4].GetComponent<Renderer>().enabled = false;
        }
        if(FixedBoom1!=outofframe&&EndBoom1!=outofframe){
            Line.GetComponent<Renderer>().enabled = true;
            Vector3[] LinePosition1 = {FixedBoom1,EndBoom1};
            Line.SetPositions(LinePosition1);
        }
        else
        {
            Line.GetComponent<Renderer>().enabled = false;
        }
        if(!StartPiston1.Equals(outofframe)&&!FixedBoom1.Equals(outofframe)&&!EndBoom1.Equals(outofframe))
        {
            PistonLine.GetComponent<Renderer>().enabled = true;
            Vector3[] LinePosition2 = {StartPiston1,EndPiston1};
            PistonLine.SetPositions(LinePosition2);
        }
        else
        {
            PistonLine.GetComponent<Renderer>().enabled = false;
        }


        }
        if(MenuData[0]==1){
        //animate
        status = true;
        BoomOneCurve.GetComponent<Renderer>().enabled=true;
        menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
        menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        if(!StartPiston1.Equals(outofframe)){
            for(int i=0; i<BoomArray1.Length; i++){
                BoomOneCurve.GetComponent<Renderer>().enabled=true;
                BoomOneCurve.SetPosition(i,BoomArray1[i]);
            }
        }
        else
        {
            BoomOneCurve.GetComponent<Renderer>().enabled=false;
        }
        StartCoroutine(FollowPistonPath());
        StartCoroutine(FollowPath());
        }
        if (MenuData[0]==2){
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
        public void RenderComponents()
    {
        float BoomOverShootFraction1 = 0f;
        float PistonFraction1 = sliderValue(receivedPos9, receivedPos8);
        if(receivedPos8!=outofframe&&receivedPos9!=outofframe)
        {
            PistonOneFraction.text = PistonFraction1.ToString();
        }
        else
        {
            PistonOneFraction.text = "slider not detected";
        }
        float TimeStep = 5f;
        Vector3 FixedBoom1 = receivedPos5;
        Vector3 EndBoom1 = receivedPos6;
        Vector3 StartPiston1 = receivedPos7;
        Vector3 EndBoom2 = receivedPos10;
        Vector3 StartPiston2 = receivedPos11;
        Vector3 StartBoom1 = BoomStartFinder(FixedBoom1, EndBoom1, BoomOverShootFraction1);
        Vector3 EndPiston1 = PistonEnd(FixedBoom1, EndBoom1, PistonFraction1);
        Vector3[] BoomArray1 = BoomRotationCalculation(FixedBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
        Vector3[] PistonArray1 = PistonRotationCalculation(StartBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
        //render components only when they exist
        if(FixedBoom1!=outofframe){
        squareArray[0].GetComponent<Renderer>().enabled = true;
        squareArray[0].transform.position = FixedBoom1;
        }else{
        squareArray[0].GetComponent<Renderer>().enabled = false;
        }
        if(EndBoom1!=outofframe){
        squareArray[1].GetComponent<Renderer>().enabled = true;
        squareArray[1].transform.position = EndBoom1;
        }else{
        squareArray[1].GetComponent<Renderer>().enabled = false;
        }
        if(StartPiston1!=outofframe){
        squareArray[2].GetComponent<Renderer>().enabled = true;
        squareArray[2].transform.position = StartPiston1;
        PistonOneEnd.transform.position = EndPiston1;
        }else{
        squareArray[2].GetComponent<Renderer>().enabled = false;
        }
         if(EndBoom2!=outofframe){
            squareArray[3].GetComponent<Renderer>().enabled = true;
            squareArray[3].transform.position = StartPiston1;
        }else{
            squareArray[3].GetComponent<Renderer>().enabled = false;
        }
        if(StartPiston2!=outofframe){
            squareArray[4].GetComponent<Renderer>().enabled = true;
            squareArray[4].transform.position = StartPiston1;
        }else{
            squareArray[4].GetComponent<Renderer>().enabled = false;
        }
        if(FixedBoom1!=outofframe&&EndBoom1!=outofframe){
            Line.GetComponent<Renderer>().enabled = true;
            Vector3[] LinePosition1 = {FixedBoom1,EndBoom1};
            Line.SetPositions(LinePosition1);
        }
        else
        {
            Line.GetComponent<Renderer>().enabled = false;
        }

        if(!StartPiston1.Equals(outofframe)&&!EndPiston1.Equals(outofframe)&&!FixedBoom1.Equals(outofframe)&&!EndBoom1.Equals(outofframe))
        {
            PistonLine.GetComponent<Renderer>().enabled = true;
            Vector3[] LinePosition2 = {StartPiston1,EndPiston1};
            PistonLine.SetPositions(LinePosition2);
        }
        else
        {
            PistonLine.GetComponent<Renderer>().enabled = false;
        }

    }

    public IEnumerator FollowPath()
    {
        float BoomOverShootFraction1 = 0f;
        float PistonFraction1 = sliderValue(receivedPos9, receivedPos8);
        if(receivedPos8!=outofframe&&receivedPos9!=outofframe)
        {
            PistonOneFraction.text = PistonFraction1.ToString();
        }
        else
        {
            PistonOneFraction.text = "slider not detected";
        }
        float TimeStep = 5f;
        Vector3 FixedBoom1 = receivedPos5;
        Vector3 EndBoom1 = receivedPos6;
        Vector3 StartPiston1 = receivedPos7;
        Vector3[] BoomArray1 = BoomRotationCalculation(FixedBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);

        RenderComponents();
        if(StartPiston1!=outofframe && FixedBoom1!=outofframe && EndBoom1!=outofframe)
        {
        for(int i=0; i<BoomArray1.Length+1;i++)
        {
            Vector3 pos = BoomArray1[i];
            float speed = 10f*DistanceBetweenPoints(BoomArray1[i], BoomArray1[i+1])/TimeStep;
            yield return StartCoroutine(DrawLinkLine(pos, speed));
        }
        }
        status = false;
    }
     public IEnumerator FollowPistonPath()
    {
        float BoomOverShootFraction1 = 0f;
        float PistonFraction1 = sliderValue(receivedPos9, receivedPos8);
        if(receivedPos8!=outofframe&&receivedPos9!=outofframe)
        {
            PistonOneFraction.text = PistonFraction1.ToString();
        }
        else
        {
            PistonOneFraction.text = "slider not detected";
        }
        float TimeStep = 5f;
        Vector3 FixedBoom1 = receivedPos5;
        Vector3 EndBoom1 = receivedPos6;
        Vector3 StartPiston1 = receivedPos7;
        Vector3 StartBoom1 = BoomStartFinder(FixedBoom1, EndBoom1, BoomOverShootFraction1);
        Vector3 EndPiston1 = PistonEnd(FixedBoom1, EndBoom1, PistonFraction1);
        Vector3[] BoomArray1 = BoomRotationCalculation(FixedBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);
        Vector3[] PistonArray1 = PistonRotationCalculation(StartBoom1, EndBoom1, StartPiston1, BoomOverShootFraction1, PistonFraction1,TimeStep);

        RenderComponents();
        if(StartPiston1!=outofframe && FixedBoom1!=outofframe && EndBoom1!=outofframe)
        {
        for(int i=0; i<PistonArray1.Length;i++)
        {
            Vector3 pos = PistonArray1[i];
            float speed = 10f*DistanceBetweenPoints(PistonArray1[i], PistonArray1[i+1])/TimeStep;
            yield return StartCoroutine(DrawPistonLine(pos, speed));
        }
        }
        status = false;
    }
    public IEnumerator DrawLinkLine(Vector3 posonlink, float speed)
    {

        while(BoomOneEnd.transform.position != posonlink)
        {
            BoomOneEnd.transform.position = Vector3.MoveTowards (BoomOneEnd.transform.position, posonlink, speed);
            Vector3[] LinePosition1 = {squareArray[0].transform.position, posonlink};
            Line.SetPositions(LinePosition1);
            yield return null;
        }
    }
    public IEnumerator DrawPistonLine(Vector3 posonlink, float speed)
    {
        while(PistonOneEnd.transform.position != posonlink)
        {
            PistonOneEnd.transform.position = Vector3.MoveTowards (PistonOneEnd.transform.position, posonlink, speed);
            Vector3[] LinePosition1 = {squareArray[2].transform.position, posonlink};
            PistonLine.SetPositions(LinePosition1);
            yield return null;
        }
    }
}
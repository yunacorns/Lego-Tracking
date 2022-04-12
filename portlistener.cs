
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
    public LineRenderer PistonOneCurve;
    public TextMeshProUGUI PistonOneFraction;
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
    Vector3 size = Vector3.zero;
    Vector3 zeros = Vector3.zero;
    Vector3 outofframe = new Vector3(-100,0,0);
    public Vector3 MenuData = new Vector3(10,0,0);
    public bool status = false;
    // public float t;
    // public float speed;
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
            //receivedPos8 = StringToVector3(dataReceived,"8");
            //receivedPos9 = StringToVector3(dataReceived,"9");
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

            float PistonRate = 0.2f;

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

            Vector3 PistonEnd = BoomStart + (BoomEnd - BoomStart)*PistonFraction;

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

            float PistonRate = 0.2f;

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

        float BoomRange = (EndAngle - StartAngle)*180f/(float)Math.PI;

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

        for ( int i = 1; i < (ArrayLength-1); i++)

        {

        float y = (AngularVelocity[i][1] + AngularVelocity[i+1][1] )/2;

        AngularVelocity[i][1] = y;

        }

        AngularVelocity[ArrayLength-1][1] = 0;



        return AngularVelocity;



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
            return default_slider_value;
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
        return slider_current_value;
    }

    public async void Update()
    {
        float BoomOverShootFraction = 0f;
        float PistonFraction = sliderValue(receivedPos9,receivedPos8);
        if(receivedPos8!=outofframe && receivedPos9!==outofframe)
        {
            PistonOneFraction.text = PistonFraction.ToString();
        }
        else
        {
            PistonOneFraction.text = "slider not detected"
        }

        float TimeStep = 5f;

        Vector3 joint1 = receivedPos5;
        Vector3 joint2 = receivedPos6;
        Vector3 piston1 = receivedPos7;
        Vector3 BoomStart = BoomStartFinder(joint1, joint2, BoomOverShootFraction);
        Vector3 PistonOneEndPos = PistonEnd(BoomStart, joint2, PistonFraction);
        Vector3[] BoomArray = BoomRotationCalculation(joint1, joint2, piston1, BoomOverShootFraction, PistonFraction,TimeStep);
        Vector3[] PistonArray = PistonRotationCalculation(joint1, joint2, piston1, BoomOverShootFraction, PistonFraction,TimeStep);
        float[] BoomAngleChange = RotationFromStartCalculation(BoomArray);
        float TotalBoomRange = BoomRangeCalculation(BoomArray);
        Vector3[] Omega = AngularVelocityCalculation(BoomArray, TimeStep);
        Vector3[] V = VelocityCalculation(Omega, joint1, joint2);
        float PistonFractionTest = sliderValue(receivedPos9,receivedPos8);
        BoomOneCurve.positionCount = BoomArray.Length;

        if(MenuData[0]==0){
        //edit
        status = false;
        BoomOneCurve.GetComponent<Renderer>().enabled=false;
        menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.blue;
        menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        //render components only if they exist
        if(joint1!=outofframe){
        squareArray[0].GetComponent<Renderer>().enabled = true;
        squareArray[0].transform.position = joint1;
        }else{
        squareArray[0].GetComponent<Renderer>().enabled = false;
        }
        if(joint2!=outofframe){
        squareArray[1].GetComponent<Renderer>().enabled = true;
        squareArray[1].transform.position = joint2;
        BoomOneEnd.transform.position = joint2;
        }else{
        squareArray[1].GetComponent<Renderer>().enabled = false;
        }
        if(piston1!=outofframe){
        squareArray[2].GetComponent<Renderer>().enabled = true;
        squareArray[2].transform.position = piston1;
        }else{
        squareArray[2].GetComponent<Renderer>().enabled = false;
        }
         if(!joint1.Equals(outofframe)&&!joint2.Equals(outofframe))
        {
            Line.GetComponent<Renderer>().enabled = true;
            Vector3[] LinePosition1 = {BoomStart,joint2};
            Line.SetPositions(LinePosition1);
        }
          else
        {
            Line.GetComponent<Renderer>().enabled = false;
            Vector3[] LinePosition1 = {outofframe, outofframe};
            Line.SetPositions(LinePosition1);
        }

        if(!piston1.Equals(outofframe)&&!joint1.Equals(outofframe)&&!joint2.Equals(outofframe))
        {
            PistonLine.GetComponent<Renderer>().enabled = true;
            Vector3[] LinePosition2 = {piston1,PistonOneEndPos};
            PistonLine.SetPositions(LinePosition2);
        }
        else
        {
            PistonLine.GetComponent<Renderer>().enabled = false;
        }
        }

        if(MenuData[0]==1 && status==false){
        //animate
        status = true;
        BoomOneCurve.GetComponent<Renderer>().enabled=true;
        menuArray[0].GetComponent<SpriteRenderer>().material.color = Color.white;
        menuArray[1].GetComponent<SpriteRenderer>().material.color = Color.blue;
        menuArray[2].GetComponent<SpriteRenderer>().material.color = Color.white;
        if(!piston1.Equals(outofframe)){
        for(int i=0; i<BoomArray.Length; i++){
            BoomOneCurve.GetComponent<Renderer>().enabled = true;
            BoomOneCurve.SetPosition(i,BoomArray[i]);

        }
        } else
        {
            BoomOneCurve.GetComponent<Renderer>().enabled = false;
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
    }

    public void RenderComponents()
    {
        float BoomOverShootFraction = 0f;
        float PistonFraction = sliderValue(receivedPos9,receivedPos8);
        if(receivedPos8!=outofframe && receivedPos9!==outofframe)
        {
            PistonOneFraction.text = PistonFraction.ToString();
        }
        else
        {
            PistonOneFraction.text = "slider not detected"
        }

        float TimeStep = 5f;
        Vector3 joint1 = receivedPos5;
        Vector3 joint2 = receivedPos6;
        Vector3 piston1 = receivedPos7;
        Vector3 BoomStart = BoomStartFinder(joint1, joint2, BoomOverShootFraction);
        Vector3 PistonOneEndPos = PistonEnd(joint1, joint2, PistonFraction);
        Vector3[] BoomArray = BoomRotationCalculation(joint1, joint2, piston1, BoomOverShootFraction, PistonFraction,TimeStep);
        Vector3[] PistonArray = PistonRotationCalculation(joint1, joint2, piston1, BoomOverShootFraction, PistonFraction,TimeStep);
        //render components only when they exist
        if(joint1!=outofframe){
        squareArray[0].GetComponent<Renderer>().enabled = true;
        squareArray[0].transform.position = joint1;
        }else{
        squareArray[0].GetComponent<Renderer>().enabled = false;
        }
        if(joint2!=outofframe){
        squareArray[1].GetComponent<Renderer>().enabled = true;
        squareArray[1].transform.position = joint2;
        }else{
        squareArray[1].GetComponent<Renderer>().enabled = false;
        }
        if(piston1!=outofframe){
        squareArray[2].GetComponent<Renderer>().enabled = true;
        squareArray[2].transform.position = piston1;
        PistonOneEnd.transform.position = PistonOneEndPos;
        }else{
        squareArray[2].GetComponent<Renderer>().enabled = false;
        }

        if(!piston1.Equals(outofframe)&&!PistonOneEndPos.Equals(outofframe)&&!joint1.Equals(outofframe)&&!joint2.Equals(outofframe))
        {
            PistonLine.GetComponent<Renderer>().enabled = true;
            Vector3[] LinePosition2 = {piston1,PistonOneEndPos};
            PistonLine.SetPositions(LinePosition2);
        }
        else
        {
            PistonLine.GetComponent<Renderer>().enabled = false;
        }

    }

    public IEnumerator FollowPath()
    {
        float BoomOverShootFraction = 0f;
        float PistonFraction = sliderValue(receivedPos9,receivedPos8);
        if(receivedPos8!=outofframe && receivedPos9!==outofframe)
        {
            PistonOneFraction.text = PistonFraction.ToString();
        }
        else
        {
            PistonOneFraction.text = "slider not detected"
        }

        float TimeStep = 5f;
        Vector3 joint1 = receivedPos5;
        Vector3 joint2 = receivedPos6;
        Vector3 piston1 = receivedPos7;
        // Vector3 BoomStart = BoomStartFinder(joint1, joint2, BoomOverShootFraction);
        // Vector3 PistonOneEndPos = PistonEnd(joint1, joint2, PistonFraction);
        Vector3[] BoomArray = BoomRotationCalculation(joint1, joint2, piston1, BoomOverShootFraction, PistonFraction,TimeStep);
        //Vector3[] PistonArray = PistonRotationCalculation(joint1, joint2, piston1, BoomOverShootFraction, PistonFraction,TimeStep);
        RenderComponents();
        if(piston1!=outofframe && joint1!=outofframe && joint2!=outofframe){
        for(int i=0; i<BoomArray.Length+1;i++)
        {
            Vector3 pos = BoomArray[i];
            float speed = 10f*DistanceBetweenPoints(BoomArray[i], BoomArray[i+1])/TimeStep;
            yield return StartCoroutine(DrawLinkLine(pos, speed));
        }
        }

        status = false;

    }
     public IEnumerator FollowPistonPath()
    {
        float BoomOverShootFraction = 0f;
        float PistonFraction = sliderValue(receivedPos9,receivedPos8);
        if(receivedPos8!=outofframe && receivedPos9!==outofframe)
        {
            PistonOneFraction.text = PistonFraction.ToString();
        }
        else
        {
            PistonOneFraction.text = "slider not detected"
        }

        float TimeStep = 5f;
        Vector3 joint1 = receivedPos5;
        Vector3 joint2 = receivedPos6;
        Vector3 piston1 = receivedPos7;
        //Vector3 BoomStart = BoomStartFinder(joint1, joint2, BoomOverShootFraction);
        //Vector3 PistonOneEndPos = PistonEnd(joint1, joint2, PistonFraction);
        Vector3[] BoomArray = BoomRotationCalculation(joint1, joint2, piston1, BoomOverShootFraction, PistonFraction,TimeStep);
        Vector3[] PistonArray = PistonRotationCalculation(joint1, joint2, piston1, BoomOverShootFraction, PistonFraction,TimeStep);


        RenderComponents();
        if(piston1!=outofframe && joint1!=outofframe && joint2!=outofframe){
        for(int i=0; i<PistonArray.Length+1;i++)
        {
            Vector3 pos = PistonArray[i];
            float speed = 10f*DistanceBetweenPoints(PistonArray[i], PistonArray[i+1])/TimeStep;

            yield return StartCoroutine(DrawPistonLine(pos, speed));
        }
        }

        status = false;

    }



    public IEnumerator DrawLinkLine(Vector3 posonlink, float speed)
    {

        while(BoomOneEnd.transform.position != posonlink){
            BoomOneEnd.transform.position = Vector3.MoveTowards (BoomOneEnd.transform.position, posonlink, speed);
            Vector3[] LinePosition1 = {squareArray[0].transform.position, posonlink};
            Line.SetPositions(LinePosition1);
            yield return null;
        }
    }


    public IEnumerator DrawPistonLine(Vector3 posonlink, float speed)
    {
        while(PistonOneEnd.transform.position != posonlink){
            PistonOneEnd.transform.position = Vector3.MoveTowards (PistonOneEnd.transform.position, posonlink, speed);
            Vector3[] LinePosition1 = {squareArray[2].transform.position, posonlink};
            PistonLine.SetPositions(LinePosition1);
            yield return null;
        }
    }
}